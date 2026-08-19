"""Turn a planned trajectory into CARLA vehicle controls.

alpasim's contract stops at the plan: the driver policy returns *where the vehicle should
be*, not throttle and steering.  Upstream that gap is filled by alpasim's vehicle dynamics
service; here -- as in ``carla_driver_interface``'s runtime -- it is filled by a pure
pursuit lateral controller and a PID longitudinal controller feeding
``carla.VehicleControl``.

All geometry in this module is in the **rig frame**: x forward, y left, z up,
right-handed.  Steering angles follow the same convention (positive = left), and the flip
to CARLA's left-handed steering (positive = right) happens exactly once, in
:meth:`VehicleCommand.to_carla_control`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, fields
from typing import TYPE_CHECKING, Any, Mapping, Optional

import numpy as np

from .geometry import Pose, Trajectory

if TYPE_CHECKING:
    import carla


__all__ = ["ControlConfig", "TrajectoryFollower", "VehicleCommand"]

#: Denominator guard for divisions by elapsed time or lookahead distance.
_EPSILON: float = 1e-6


@dataclass(frozen=True)
class ControlConfig:
    """Gains and limits for :class:`TrajectoryFollower`.

    Defaults mirror ``carla_driver_interface``'s ``ControlConfig`` so a policy tuned
    against that runtime behaves the same here.
    """

    # -- Lateral (pure pursuit) ------------------------------------------------
    lookahead_gain_s: float = 0.9
    """Lookahead distance per unit speed, in seconds."""

    min_lookahead_m: float = 4.0
    """Lower bound on the lookahead distance."""

    max_lookahead_m: float = 20.0
    """Upper bound on the lookahead distance."""

    wheelbase_m: float = 2.8
    """Distance between front and rear axles, used by the pure pursuit law."""

    max_steer_angle_rad: float = math.radians(70.0)
    """Steering angle mapped to full lock, used to normalise the command."""

    max_steer_rate: float = 4.0
    """Maximum change in normalised steering per second."""

    # -- Longitudinal (PID) ----------------------------------------------------
    speed_kp: float = 0.6
    """Proportional gain on speed error."""

    speed_ki: float = 0.15
    """Integral gain on speed error."""

    speed_kd: float = 0.05
    """Derivative gain on speed error."""

    integral_limit: float = 1.0
    """Clamp on the integral term, preventing wind-up while braking."""

    stop_speed_mps: float = 0.2
    """Target speeds below this are treated as a request to hold still."""

    stop_brake: float = 0.6
    """Brake applied when holding still."""

    @classmethod
    def from_mapping(cls, mapping: Mapping[str, Any]) -> "ControlConfig":
        """Return gains built from a plain mapping (e.g. a Hydra node).

        ``max_steer_angle_deg`` is accepted as an alias for
        :attr:`max_steer_angle_rad`, since degrees read better in YAML.

        Raises:
            ValueError: If *mapping* holds a key this config does not define.
        """
        values = dict(mapping)
        degrees = values.pop("max_steer_angle_deg", None)
        if degrees is not None:
            values["max_steer_angle_rad"] = math.radians(float(degrees))

        known = {field.name for field in fields(cls)}
        unknown = sorted(set(values) - known)
        if unknown:
            raise ValueError(
                f"Unknown ControlConfig key(s): {unknown}. "
                f"Known keys: {sorted(known | {'max_steer_angle_deg'})}"
            )
        return cls(**values)


@dataclass(frozen=True)
class VehicleCommand:
    """An actuation command, in CARLA's convention.

    Attributes:
        throttle: Throttle in ``[0, 1]``.
        steer: Normalised steering in ``[-1, 1]``, **positive = right**.
        brake: Brake in ``[0, 1]``.
        hand_brake: Whether the hand brake is engaged.
        reverse: Whether reverse gear is engaged.
        target_speed_mps: Speed the longitudinal controller was aiming for.
        lookahead_lateral_offset_m: Lateral offset of the pursued point, for diagnostics.
    """

    throttle: float = 0.0
    steer: float = 0.0
    brake: float = 0.0
    hand_brake: bool = False
    reverse: bool = False
    target_speed_mps: float = 0.0
    lookahead_lateral_offset_m: float = 0.0

    def to_carla_control(self) -> "carla.VehicleControl":
        """Return the equivalent :class:`carla.VehicleControl`."""
        import carla as _carla  # noqa: PLC0415

        return _carla.VehicleControl(
            throttle=float(self.throttle),
            steer=float(self.steer),
            brake=float(self.brake),
            hand_brake=self.hand_brake,
            reverse=self.reverse,
        )


class TrajectoryFollower:
    """Tracks a planned trajectory with pure pursuit plus a speed PID.

    One instance drives one vehicle; call :meth:`reset` when the plan's provenance
    changes (e.g. a new session).

    Args:
        config: Gains and limits.  ``None`` uses the defaults.
    """

    def __init__(self, config: Optional[ControlConfig] = None) -> None:
        self._config = config or ControlConfig()
        self.reset()

    @property
    def config(self) -> ControlConfig:
        """Return the controller configuration."""
        return self._config

    def reset(self) -> None:
        """Clear the integral term, the derivative memory, and the steering state."""
        self._integral = 0.0
        self._previous_error = 0.0
        self._previous_steer = 0.0

    # ------------------------------------------------------------------
    # Control
    # ------------------------------------------------------------------

    def step(
        self,
        plan_in_local: Trajectory,
        pose_local_to_rig: Pose,
        current_speed_mps: float,
        dt_s: float,
    ) -> VehicleCommand:
        """Return the command that tracks *plan_in_local* from the current pose.

        Args:
            plan_in_local: The driver's plan, in the local frame.
            pose_local_to_rig: The ego's current pose in the local frame.
            current_speed_mps: Measured ground speed.
            dt_s: Time since the previous call, in seconds.

        Returns:
            The actuation command.  An empty plan yields a full stop.
        """
        if not plan_in_local:
            return self._hold_still()

        plan_in_rig = plan_in_local.transform(pose_local_to_rig.inverse())
        points = plan_in_rig.positions

        target_speed = self._target_speed(plan_in_rig)
        lookahead = float(
            np.clip(
                self._config.lookahead_gain_s * max(current_speed_mps, 0.0),
                self._config.min_lookahead_m,
                self._config.max_lookahead_m,
            )
        )
        target = self._lookahead_point(points, lookahead)
        if target is None:
            return self._hold_still()

        steer = self._lateral(target, lookahead, dt_s)
        throttle, brake = self._longitudinal(target_speed, current_speed_mps, dt_s)

        return VehicleCommand(
            throttle=throttle,
            steer=steer,
            brake=brake,
            target_speed_mps=target_speed,
            lookahead_lateral_offset_m=float(target[1]),
        )

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _hold_still(self) -> VehicleCommand:
        """Return a braking command and reset the controller state."""
        self.reset()
        return VehicleCommand(throttle=0.0, steer=0.0, brake=self._config.stop_brake)

    @staticmethod
    def _lookahead_point(
        points: np.ndarray, lookahead_m: float
    ) -> Optional[np.ndarray]:
        """Return the first plan point at least *lookahead_m* ahead of the rig origin.

        Falls back to the farthest point when the plan is shorter than the lookahead,
        and ignores points behind the vehicle so a plan that starts slightly behind the
        rig origin does not fold the steering back on itself.

        Returns:
            A ``(3,)`` point in the rig frame, or ``None`` when no usable point exists.
        """
        if points.shape[0] == 0:
            return None
        ahead = points[points[:, 0] > 0.0]
        candidates = ahead if ahead.shape[0] else points
        distances = np.linalg.norm(candidates[:, :2], axis=1)
        beyond = np.flatnonzero(distances >= lookahead_m)
        index = int(beyond[0]) if beyond.size else int(np.argmax(distances))
        if distances[index] < _EPSILON:
            return None
        return candidates[index]

    def _target_speed(self, plan_in_rig: Trajectory) -> float:
        """Return the speed implied by the plan's first timed segment.

        A plan carries positions and timestamps but no explicit speed, so the intended
        speed is the distance covered over the first segment divided by its duration.
        """
        if len(plan_in_rig) < 2:
            return 0.0
        positions = plan_in_rig.positions
        timestamps = plan_in_rig.timestamps_us
        for index in range(1, len(timestamps)):
            duration_s = (timestamps[index] - timestamps[0]) * 1e-6
            if duration_s < _EPSILON:
                continue
            distance = float(np.linalg.norm(positions[index][:2] - positions[0][:2]))
            return distance / duration_s
        return 0.0

    def _lateral(self, target: np.ndarray, lookahead_m: float, dt_s: float) -> float:
        """Return the normalised steering command for *target*, rate limited."""
        distance = max(float(np.linalg.norm(target[:2])), _EPSILON)
        alpha = math.atan2(float(target[1]), float(target[0]))
        steer_angle = math.atan2(
            2.0 * self._config.wheelbase_m * math.sin(alpha), max(distance, _EPSILON)
        )

        # Rig frame is right-handed (positive angle = left); CARLA steers positive right.
        command = -steer_angle / self._config.max_steer_angle_rad
        command = float(np.clip(command, -1.0, 1.0))

        max_delta = self._config.max_steer_rate * max(dt_s, 0.0)
        if max_delta > 0.0:
            lower, upper = (
                self._previous_steer - max_delta,
                self._previous_steer + max_delta,
            )
            command = float(np.clip(command, lower, upper))
        self._previous_steer = command
        return command

    def _longitudinal(
        self, target_speed_mps: float, current_speed_mps: float, dt_s: float
    ) -> tuple[float, float]:
        """Return ``(throttle, brake)`` for the requested speed."""
        if target_speed_mps < self._config.stop_speed_mps:
            self._integral = 0.0
            self._previous_error = 0.0
            return 0.0, self._config.stop_brake

        error = target_speed_mps - current_speed_mps
        if dt_s > _EPSILON:
            self._integral = float(
                np.clip(
                    self._integral + error * dt_s,
                    -self._config.integral_limit,
                    self._config.integral_limit,
                )
            )
            derivative = (error - self._previous_error) / dt_s
        else:
            derivative = 0.0
        self._previous_error = error

        output = (
            self._config.speed_kp * error
            + self._config.speed_ki * self._integral
            + self._config.speed_kd * derivative
        )
        if output >= 0.0:
            return float(np.clip(output, 0.0, 1.0)), 0.0
        return 0.0, float(np.clip(-output, 0.0, 1.0))
