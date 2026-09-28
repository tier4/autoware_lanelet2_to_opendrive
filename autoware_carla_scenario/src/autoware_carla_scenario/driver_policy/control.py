"""Trajectory tracking: alpasim's ``VDCService`` replaced by a local controller.

alpasim hands the driver's plan to a separate vehicle-dynamics-and-controller
service over gRPC (``controller.proto``).  CARLA already simulates the vehicle,
so all that is missing is the controller itself: turn a planned trajectory into
throttle / brake / steer.

Lateral control is pure pursuit, longitudinal control is a PI(D) on speed.
Neither touches the ``carla`` module, so both are unit-testable without a
simulator -- :class:`VehicleCommand` is a plain dataclass the policy
translates into ``carla.VehicleControl``.

Ported from ``carla_driver_interface.runtime.control``. The tuning defaults are
the same, so a driver behaves the same under either runtime.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from . import polyline
from .geometry import Pose, Trajectory

__all__ = ["ControlConfig", "TrajectoryFollower", "VehicleCommand"]


@dataclass(frozen=True)
class ControlConfig:
    """Tuning for :class:`TrajectoryFollower`."""

    #: Lookahead grows with speed: ``clamp(gain * speed, min, max)``.
    lookahead_gain_s: float = 0.9
    min_lookahead_m: float = 4.0
    max_lookahead_m: float = 20.0
    #: Distance from the rig origin (rear axle) to the front axle.
    wheelbase_m: float = 2.8
    #: Physical steering range, used to normalise into CARLA's [-1, 1].
    max_steer_angle_rad: float = math.radians(70.0)
    #: Rate limit on the normalised steering command, per second.
    max_steer_rate: float = 4.0

    speed_kp: float = 0.6
    speed_ki: float = 0.15
    speed_kd: float = 0.05
    #: Integrator clamp, in normalised command units.
    integral_limit: float = 1.0

    #: Below this target speed the controller brakes to a stop instead of
    #: modulating throttle, so it settles cleanly at red lights.
    stop_speed_mps: float = 0.2
    stop_brake: float = 0.6


@dataclass(frozen=True)
class VehicleCommand:
    """Actuation request, in CARLA's normalised units."""

    throttle: float = 0.0
    steer: float = 0.0
    brake: float = 0.0


class TrajectoryFollower:
    """Stateful tracker: feed it plans, get actuation.

    One instance per rollout -- it carries the speed integrator and the previous
    steering command.
    """

    def __init__(self, config: ControlConfig | None = None) -> None:
        self.config = config or ControlConfig()
        self._integral = 0.0
        self._previous_speed_error = 0.0
        self._previous_steer = 0.0

    def step(
        self,
        plan_in_local: Trajectory,
        pose_local_to_rig: Pose,
        current_speed_mps: float,
        dt_s: float,
    ) -> VehicleCommand:
        """Compute actuation for one control step.

        Args:
            plan_in_local: The driver's plan, in the ``local`` frame.
            pose_local_to_rig: Where the ego actually is now.
            current_speed_mps: Longitudinal speed.
            dt_s: Time since the previous call.
        """
        if len(plan_in_local) < 2 or dt_s <= 0.0:
            # Nothing to track: coast, holding the last steering angle.
            return VehicleCommand(throttle=0.0, steer=self._previous_steer, brake=0.3)

        # Only positions are needed downstream, so rotate the points rather
        # than composing 40-odd quaternions whose rotations are then discarded.
        points = pose_local_to_rig.inverse().transform_points(plan_in_local.positions)
        arc = polyline.arc_lengths(points)

        target_speed = _plan_speed(plan_in_local.timestamps_us, arc)
        steer = self._lateral(points, arc, current_speed_mps, dt_s)
        throttle, brake = self._longitudinal(target_speed, current_speed_mps, dt_s)
        return VehicleCommand(throttle=throttle, steer=steer, brake=brake)

    # -- lateral -----------------------------------------------------------

    def _lateral(
        self, points_in_rig: np.ndarray, arc: np.ndarray, speed_mps: float, dt_s: float
    ) -> float:
        cfg = self.config
        lookahead = min(
            cfg.max_lookahead_m,
            max(cfg.min_lookahead_m, cfg.lookahead_gain_s * speed_mps),
        )
        target = polyline.sample(points_in_rig, arc, lookahead)

        # Pure pursuit in the rig frame: the rig origin is the rear axle, so
        # the target's bearing `alpha` is measured straight off its coordinates.
        distance = float(np.linalg.norm(target[:2]))
        if distance < 1e-3:
            steer_angle = 0.0
        else:
            alpha = math.atan2(float(target[1]), float(target[0]))
            steer_angle = math.atan2(2.0 * cfg.wheelbase_m * math.sin(alpha), distance)

        # CARLA steers positive to the right; the rig frame is positive to the
        # left, so the command is the negated steering angle.
        command = -steer_angle / cfg.max_steer_angle_rad
        command = float(np.clip(command, -1.0, 1.0))

        max_delta = cfg.max_steer_rate * dt_s
        command = float(
            np.clip(
                command,
                self._previous_steer - max_delta,
                self._previous_steer + max_delta,
            )
        )
        self._previous_steer = command
        return command

    # -- longitudinal ------------------------------------------------------

    def _longitudinal(
        self, target_speed: float, current_speed: float, dt_s: float
    ) -> tuple[float, float]:
        cfg = self.config
        if target_speed <= cfg.stop_speed_mps:
            self._integral = 0.0
            self._previous_speed_error = 0.0
            return 0.0, cfg.stop_brake

        error = target_speed - current_speed
        derivative = (error - self._previous_speed_error) / dt_s
        self._previous_speed_error = error

        proportional_derivative = cfg.speed_kp * error + cfg.speed_kd * derivative
        candidate = self._integral + error * dt_s
        raw = proportional_derivative + cfg.speed_ki * candidate
        # Anti-windup: only integrate while the command is not saturated, or
        # while the error pushes it back out of saturation.
        if (
            -1.0 < raw < 1.0
            or (raw >= 1.0 and error < 0.0)
            or (raw <= -1.0 and error > 0.0)
        ):
            self._integral = float(
                np.clip(candidate, -cfg.integral_limit, cfg.integral_limit)
            )
        command = proportional_derivative + cfg.speed_ki * self._integral

        if command >= 0.0:
            return float(min(command, 1.0)), 0.0
        return 0.0, float(min(-command, 1.0))


# ---------------------------------------------------------------------------
# Plan helpers
# ---------------------------------------------------------------------------


def _plan_speed(timestamps_us: list[int], arc: np.ndarray) -> float:
    """Speed the plan implies over its first second (or its whole length).

    Averaging over a window rather than the first pair keeps the target steady
    when the driver emits a plan at a finer resolution than the control step.
    """
    t0 = timestamps_us[0]
    end = len(timestamps_us) - 1
    for i, ts in enumerate(timestamps_us):
        if ts - t0 >= 1_000_000:
            end = i
            break
    dt_s = (timestamps_us[end] - t0) / 1e6
    if dt_s <= 0.0:
        return 0.0
    return float(arc[end]) / dt_s
