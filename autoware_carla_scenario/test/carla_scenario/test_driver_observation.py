"""Unit tests for the CARLA -> alpasim observation bridge.

The handedness flip is the subtle part: CARLA's world is left-handed (y = South,
yaw clockwise) while the driver contract is right-handed.
"""

from __future__ import annotations

import math
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import pytest

from autoware_carla_scenario.driver.observation import (
    ego_observation,
    encode_frame_jpeg,
    rear_axle_offset,
    route_waypoints_in_rig,
    to_local_pose,
)


def _transform(x=0.0, y=0.0, z=0.0, yaw=0.0) -> SimpleNamespace:
    return SimpleNamespace(
        location=SimpleNamespace(x=x, y=y, z=z),
        rotation=SimpleNamespace(roll=0.0, pitch=0.0, yaw=yaw),
    )


def _actor(transform: SimpleNamespace, **kinematics) -> MagicMock:
    actor = MagicMock()
    actor.get_transform.return_value = transform
    actor.get_location.return_value = transform.location
    actor.get_velocity.return_value = SimpleNamespace(
        **kinematics.get("velocity", {"x": 0.0, "y": 0.0, "z": 0.0})
    )
    actor.get_acceleration.return_value = SimpleNamespace(
        **kinematics.get("acceleration", {"x": 0.0, "y": 0.0, "z": 0.0})
    )
    actor.get_angular_velocity.return_value = SimpleNamespace(
        **kinematics.get("angular_velocity", {"x": 0.0, "y": 0.0, "z": 0.0})
    )
    return actor


# ---------------------------------------------------------------------------
# Handedness
# ---------------------------------------------------------------------------


def test_local_pose_flips_y_and_yaw() -> None:
    pose = to_local_pose(_transform(x=10.0, y=20.0, z=1.0, yaw=90.0))
    assert np.allclose(pose.position, [10.0, -20.0, 1.0])
    assert pose.yaw == pytest.approx(math.radians(-90.0))


def test_rear_axle_offset_shifts_backwards_along_heading() -> None:
    """A vehicle facing CARLA +x has its rig origin 1.4 m behind the actor origin."""
    pose = to_local_pose(_transform(x=10.0, y=0.0, yaw=0.0), rear_axle_offset_m=-1.4)
    assert np.allclose(pose.position, [8.6, 0.0, 0.0], atol=1e-6)


def test_rear_axle_offset_follows_the_vehicle_heading() -> None:
    """Facing CARLA +y (south), the shift must go along the local -y axis."""
    pose = to_local_pose(_transform(x=0.0, y=0.0, yaw=90.0), rear_axle_offset_m=-1.4)
    assert np.allclose(pose.position, [0.0, 1.4, 0.0], atol=1e-6)


def test_velocity_is_reported_in_the_rig_frame() -> None:
    """A vehicle facing CARLA +y moving along CARLA +y is moving straight ahead."""
    actor = _actor(_transform(yaw=90.0), velocity={"x": 0.0, "y": 5.0, "z": 0.0})
    observation = ego_observation(actor, timestamp_us=1_000)
    assert np.allclose(observation.linear_velocity, [5.0, 0.0, 0.0], atol=1e-6)
    assert observation.speed_mps == pytest.approx(5.0)


def test_angular_velocity_flips_sign_with_the_handedness() -> None:
    """Yaw rate is a pseudovector: the reflection adds a sign flip."""
    actor = _actor(_transform(), angular_velocity={"x": 0.0, "y": 0.0, "z": 90.0})
    observation = ego_observation(actor, timestamp_us=0)
    assert observation.angular_velocity[2] == pytest.approx(
        -math.radians(90.0), abs=1e-9
    )


def test_acceleration_is_rotated_into_the_rig_frame() -> None:
    actor = _actor(_transform(yaw=180.0), acceleration={"x": -2.0, "y": 0.0, "z": 0.0})
    observation = ego_observation(actor, timestamp_us=0)
    assert np.allclose(observation.linear_acceleration, [2.0, 0.0, 0.0], atol=1e-6)


def test_observation_carries_its_timestamp() -> None:
    observation = ego_observation(_actor(_transform()), timestamp_us=123_456)
    assert observation.timestamp_us == 123_456


# ---------------------------------------------------------------------------
# Rear-axle offset derivation
# ---------------------------------------------------------------------------


def test_explicit_override_wins() -> None:
    assert rear_axle_offset(MagicMock(), override=-1.25) == pytest.approx(-1.25)


def test_falls_back_to_the_bounding_box() -> None:
    """Without usable wheel physics, half the vehicle length is a sane estimate."""
    actor = MagicMock()
    actor.get_physics_control.side_effect = RuntimeError("no physics")
    actor.bounding_box.extent.x = 2.4
    assert rear_axle_offset(actor) == pytest.approx(-1.2)


def test_implausible_wheel_positions_fall_back(
    caplog: pytest.LogCaptureFixture,
) -> None:
    """CARLA 0.9.x reports wheels in world centimetres, which lands far off."""
    actor = _actor(_transform())
    wheel = SimpleNamespace(position=SimpleNamespace(x=50_000.0, y=0.0, z=0.0))
    actor.get_physics_control.return_value = SimpleNamespace(wheels=[wheel] * 4)
    actor.bounding_box.extent.x = 2.4

    with caplog.at_level("WARNING"):
        offset = rear_axle_offset(actor)

    assert offset == pytest.approx(-1.2)
    assert "implausible" in caplog.text


def test_unreadable_geometry_yields_zero() -> None:
    actor = MagicMock()
    actor.get_physics_control.side_effect = RuntimeError("no physics")
    type(actor).bounding_box = property(
        lambda self: (_ for _ in ()).throw(RuntimeError("no bbox"))
    )
    assert rear_axle_offset(actor) == pytest.approx(0.0)


# ---------------------------------------------------------------------------
# Route
# ---------------------------------------------------------------------------


def _map_along_x() -> MagicMock:
    carla_map = MagicMock()

    def _waypoint_at(distance: float) -> MagicMock:
        waypoint = MagicMock()
        waypoint.transform.location = SimpleNamespace(x=distance, y=0.0, z=0.0)
        waypoint.next.side_effect = lambda step: [_waypoint_at(distance + step)]
        return waypoint

    carla_map.get_waypoint.return_value = _waypoint_at(0.0)
    return carla_map


def test_route_is_walked_at_the_requested_resolution() -> None:
    actor = _actor(_transform())
    pose = to_local_pose(actor.get_transform())
    route = route_waypoints_in_rig(
        _map_along_x(), actor, pose, horizon_m=10.0, resolution_m=2.0
    )
    assert route.shape == (5, 3)
    assert np.allclose(route[:, 0], [0.0, 2.0, 4.0, 6.0, 8.0], atol=1e-6)


def test_route_stops_when_the_road_ends() -> None:
    carla_map = MagicMock()
    waypoint = MagicMock()
    waypoint.transform.location = SimpleNamespace(x=0.0, y=0.0, z=0.0)
    waypoint.next.return_value = []
    carla_map.get_waypoint.return_value = waypoint

    actor = _actor(_transform())
    route = route_waypoints_in_rig(
        carla_map, actor, to_local_pose(actor.get_transform())
    )
    assert route.shape == (1, 3)


def test_off_road_ego_yields_an_empty_route(caplog: pytest.LogCaptureFixture) -> None:
    carla_map = MagicMock()
    carla_map.get_waypoint.return_value = None
    actor = _actor(_transform())

    with caplog.at_level("WARNING"):
        route = route_waypoints_in_rig(carla_map, actor, to_local_pose(_transform()))

    assert route.shape == (0, 3)
    assert "not on a drivable waypoint" in caplog.text


# ---------------------------------------------------------------------------
# Frame encoding
# ---------------------------------------------------------------------------


def test_encode_frame_produces_jpeg_bytes() -> None:
    frame = np.zeros((16, 16, 3), dtype=np.uint8)
    encoded = encode_frame_jpeg(frame)
    assert encoded is not None
    assert encoded.startswith(b"\xff\xd8")  # JPEG SOI marker
    assert encoded.endswith(b"\xff\xd9")  # JPEG EOI marker


def test_encode_frame_honours_quality() -> None:
    rng = np.random.default_rng(0)
    frame = rng.integers(0, 255, size=(64, 64, 3), dtype=np.uint8)
    low, high = (
        encode_frame_jpeg(frame, quality=10),
        encode_frame_jpeg(frame, quality=95),
    )
    assert low is not None and high is not None
    assert len(low) < len(high)
