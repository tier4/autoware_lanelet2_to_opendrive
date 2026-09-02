"""Unit tests for the pure-pursuit + PID trajectory follower."""

from __future__ import annotations

import math

import pytest

from autoware_carla_scenario.driver.control import (
    ControlConfig,
    TrajectoryFollower,
    VehicleCommand,
)
from autoware_carla_scenario.driver.geometry import Pose, Trajectory


_DT_S = 0.05
_STEP_US = 100_000


def _plan(points, *, speed_mps: float = 8.0) -> Trajectory:
    """Return a local-frame plan through *points* at a constant *speed_mps*.

    Timestamps are derived from the requested speed so that the follower's target speed
    comes out as *speed_mps*.
    """
    plan = Trajectory.empty()
    previous = None
    timestamp = 0
    for x, y in points:
        if previous is not None:
            distance = math.hypot(x - previous[0], y - previous[1])
            timestamp += int(round(distance / speed_mps * 1e6))
        plan.append(timestamp, Pose.from_xyz_yaw(x, y, 0.0, 0.0))
        previous = (x, y)
    return plan


def _straight(speed_mps: float = 8.0) -> Trajectory:
    return _plan([(step * 2.0, 0.0) for step in range(21)], speed_mps=speed_mps)


# ---------------------------------------------------------------------------
# Lateral control
# ---------------------------------------------------------------------------


def test_straight_plan_steers_straight() -> None:
    follower = TrajectoryFollower()
    command = follower.step(_straight(), Pose.identity(), 8.0, _DT_S)
    assert command.steer == pytest.approx(0.0, abs=1e-9)


def test_plan_to_the_left_steers_left() -> None:
    """The rig frame is right-handed, CARLA steers positive to the right."""
    follower = TrajectoryFollower()
    plan = _plan([(step * 2.0, step * 0.6) for step in range(21)])
    command = follower.step(plan, Pose.identity(), 8.0, _DT_S)
    assert command.steer < 0.0


def test_plan_to_the_right_steers_right() -> None:
    follower = TrajectoryFollower()
    plan = _plan([(step * 2.0, -step * 0.6) for step in range(21)])
    command = follower.step(plan, Pose.identity(), 8.0, _DT_S)
    assert command.steer > 0.0


def test_steering_is_rate_limited() -> None:
    """A hard turn cannot exceed max_steer_rate * dt in a single tick."""
    config = ControlConfig(max_steer_rate=1.0)
    follower = TrajectoryFollower(config)
    plan = _plan([(step * 1.0, -step * 3.0) for step in range(21)])
    command = follower.step(plan, Pose.identity(), 8.0, _DT_S)
    assert command.steer == pytest.approx(config.max_steer_rate * _DT_S, abs=1e-9)


def test_steering_accounts_for_the_ego_pose() -> None:
    """A plan that is straight in local coordinates curves once the ego is yawed."""
    follower = TrajectoryFollower()
    yawed = Pose.from_xyz_yaw(0.0, 0.0, 0.0, math.radians(-20.0))
    command = follower.step(_straight(), yawed, 8.0, _DT_S)
    # The path now runs off to the ego's left, so the command steers left.
    assert command.steer < 0.0


def test_points_behind_the_vehicle_are_ignored() -> None:
    """A plan starting behind the rig origin must not fold the steering backwards."""
    follower = TrajectoryFollower()
    plan = _plan([(-4.0, 0.0), *[(step * 2.0, 0.0) for step in range(1, 21)]])
    command = follower.step(plan, Pose.identity(), 8.0, _DT_S)
    assert command.steer == pytest.approx(0.0, abs=1e-9)


# ---------------------------------------------------------------------------
# Longitudinal control
# ---------------------------------------------------------------------------


def test_below_target_speed_opens_the_throttle() -> None:
    follower = TrajectoryFollower()
    command = follower.step(_straight(speed_mps=10.0), Pose.identity(), 1.0, _DT_S)
    assert command.throttle > 0.0
    assert command.brake == pytest.approx(0.0)
    assert command.target_speed_mps == pytest.approx(10.0, rel=1e-3)


def test_above_target_speed_brakes() -> None:
    follower = TrajectoryFollower()
    command = follower.step(_straight(speed_mps=2.0), Pose.identity(), 15.0, _DT_S)
    assert command.brake > 0.0
    assert command.throttle == pytest.approx(0.0)


def test_stationary_plan_holds_the_vehicle() -> None:
    """A plan that does not advance is a request to stand still."""
    config = ControlConfig()
    follower = TrajectoryFollower(config)
    plan = Trajectory.empty()
    for index in range(5):
        plan.append(index * _STEP_US, Pose.from_xyz_yaw(0.0, 0.0, 0.0, 0.0))
    command = follower.step(plan, Pose.identity(), 0.0, _DT_S)
    assert command.throttle == pytest.approx(0.0)
    assert command.brake == pytest.approx(config.stop_brake)


def test_empty_plan_brakes_and_resets() -> None:
    config = ControlConfig()
    follower = TrajectoryFollower(config)
    follower.step(_straight(), Pose.identity(), 1.0, _DT_S)
    command = follower.step(Trajectory.empty(), Pose.identity(), 5.0, _DT_S)
    assert command.brake == pytest.approx(config.stop_brake)
    assert command.steer == pytest.approx(0.0)


def test_integral_term_is_clamped() -> None:
    """Sustained error must not wind the integral term past its limit."""
    config = ControlConfig(speed_kp=0.0, speed_kd=0.0, speed_ki=1.0, integral_limit=0.5)
    follower = TrajectoryFollower(config)
    plan = _straight(speed_mps=10.0)
    for _ in range(200):
        command = follower.step(plan, Pose.identity(), 0.0, _DT_S)
    assert command.throttle == pytest.approx(0.5, abs=1e-6)


def test_reset_clears_controller_state() -> None:
    config = ControlConfig(speed_kp=0.0, speed_kd=0.0, speed_ki=1.0)
    follower = TrajectoryFollower(config)
    plan = _straight(speed_mps=10.0)
    for _ in range(10):
        follower.step(plan, Pose.identity(), 0.0, _DT_S)
    follower.reset()
    command = follower.step(plan, Pose.identity(), 0.0, _DT_S)
    assert command.throttle == pytest.approx(1.0 * 10.0 * _DT_S, abs=1e-6)


# ---------------------------------------------------------------------------
# Command conversion
# ---------------------------------------------------------------------------


def test_command_defaults_are_inert() -> None:
    command = VehicleCommand()
    assert command.throttle == 0.0
    assert command.brake == 0.0
    assert command.hand_brake is False
    assert command.reverse is False
