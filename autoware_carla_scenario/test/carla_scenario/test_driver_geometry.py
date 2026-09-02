"""Unit tests for the driver's pose and trajectory primitives."""

from __future__ import annotations

import math

import numpy as np
import pytest

from autoware_carla_scenario.driver._proto import common_pb2
from autoware_carla_scenario.driver.geometry import (
    Pose,
    Trajectory,
    waypoints_to_proto,
)


# ---------------------------------------------------------------------------
# Pose
# ---------------------------------------------------------------------------


def test_identity_pose_is_neutral() -> None:
    identity = Pose.identity()
    assert np.allclose(identity.position, np.zeros(3))
    assert identity.yaw == pytest.approx(0.0)
    assert np.allclose(identity.rotation_matrix, np.eye(3))


def test_from_xyz_yaw_round_trips_through_yaw() -> None:
    pose = Pose.from_xyz_yaw(1.0, 2.0, 3.0, math.radians(30.0))
    assert np.allclose(pose.position, [1.0, 2.0, 3.0])
    assert pose.yaw == pytest.approx(math.radians(30.0))


def test_degenerate_quaternion_falls_back_to_identity() -> None:
    """A default-constructed protobuf pose has an all-zero quaternion."""
    pose = Pose.from_proto(common_pb2.Pose())
    assert np.allclose(pose.quat_xyzw, [0.0, 0.0, 0.0, 1.0])
    assert np.allclose(pose.rotation_matrix, np.eye(3))


def test_compose_applies_rotation_then_translation() -> None:
    """``a @ b`` places *b* in *a*'s frame: rotate, then offset."""
    outer = Pose.from_xyz_yaw(1.0, 0.0, 0.0, math.radians(90.0))
    inner = Pose.from_xyz_yaw(2.0, 0.0, 0.0, 0.0)
    composed = outer @ inner
    # 2 m forward in a frame yawed 90 deg lands 2 m along +y, offset by (1, 0).
    assert np.allclose(composed.position, [1.0, 2.0, 0.0], atol=1e-9)
    assert composed.yaw == pytest.approx(math.radians(90.0))


def test_inverse_cancels_the_pose() -> None:
    pose = Pose.from_xyz_yaw(3.0, -4.0, 1.5, math.radians(37.0))
    identity = pose @ pose.inverse()
    assert np.allclose(identity.position, np.zeros(3), atol=1e-9)
    assert identity.yaw == pytest.approx(0.0, abs=1e-9)


def test_transform_points_matches_manual_transform() -> None:
    pose = Pose.from_xyz_yaw(1.0, 2.0, 0.0, math.radians(90.0))
    points = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    transformed = pose.transform_points(points)
    assert np.allclose(transformed, [[1.0, 3.0, 0.0], [0.0, 2.0, 0.0]], atol=1e-9)


def test_transform_points_handles_empty_input() -> None:
    assert Pose.identity().transform_points(np.zeros((0, 3))).shape == (0, 3)


def test_pose_proto_round_trip() -> None:
    pose = Pose.from_xyz_yaw(1.0, 2.0, 3.0, math.radians(45.0))
    restored = Pose.from_proto(pose.to_proto())
    assert np.allclose(restored.position, pose.position, atol=1e-6)
    assert restored.yaw == pytest.approx(pose.yaw, abs=1e-6)


def test_pose_proto_uses_wxyz_field_order() -> None:
    """The wire format orders the quaternion (w, x, y, z), not (x, y, z, w)."""
    message = Pose.from_xyz_yaw(0.0, 0.0, 0.0, math.radians(180.0)).to_proto()
    assert message.quat.z == pytest.approx(1.0, abs=1e-6)
    assert message.quat.w == pytest.approx(0.0, abs=1e-6)


def test_pose_rejects_wrong_dimensions() -> None:
    with pytest.raises(ValueError, match="expected 3 components"):
        Pose(np.zeros(2), np.array([0.0, 0.0, 0.0, 1.0]))
    with pytest.raises(ValueError, match="expected 4 quaternion components"):
        Pose(np.zeros(3), np.zeros(3))


def test_from_rotation_matrix_round_trips_the_rotation() -> None:
    for yaw, pitch, roll in [(0.0, 0.0, 0.0), (37.0, 0.0, 0.0), (10.0, 20.0, -30.0)]:
        rotation = (
            Pose.from_xyz_yaw(0.0, 0.0, 0.0, math.radians(yaw)).rotation_matrix
            @ _rotation_about_y(math.radians(pitch))
            @ _rotation_about_x(math.radians(roll))
        )
        pose = Pose.from_rotation_matrix(np.array([1.0, 2.0, 3.0]), rotation)
        assert np.allclose(pose.rotation_matrix, rotation, atol=1e-9)
        assert np.allclose(pose.position, [1.0, 2.0, 3.0])


def test_from_rotation_matrix_handles_a_180_degree_turn() -> None:
    """A trace-negative rotation must not fall through to a NaN quaternion."""
    rotation = Pose.from_xyz_yaw(0.0, 0.0, 0.0, math.radians(180.0)).rotation_matrix
    pose = Pose.from_rotation_matrix(np.zeros(3), rotation)
    assert np.allclose(pose.rotation_matrix, rotation, atol=1e-9)


def _rotation_about_x(angle: float) -> np.ndarray:
    cos, sin = math.cos(angle), math.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, cos, -sin], [0.0, sin, cos]])


def _rotation_about_y(angle: float) -> np.ndarray:
    cos, sin = math.cos(angle), math.sin(angle)
    return np.array([[cos, 0.0, sin], [0.0, 1.0, 0.0], [-sin, 0.0, cos]])


# ---------------------------------------------------------------------------
# Trajectory
# ---------------------------------------------------------------------------


def _straight_plan() -> Trajectory:
    """Return a 3-point plan moving 1 m along +x every 100 ms."""
    plan = Trajectory.empty()
    for index in range(3):
        plan.append(index * 100_000, Pose.from_xyz_yaw(float(index), 0.0, 0.0, 0.0))
    return plan


def test_empty_trajectory_is_falsy() -> None:
    assert not Trajectory.empty()
    assert len(Trajectory.empty()) == 0
    assert Trajectory.empty().last_pose is None
    assert Trajectory.empty().positions.shape == (0, 3)


def test_trajectory_rejects_mismatched_lengths() -> None:
    with pytest.raises(ValueError, match="same length"):
        Trajectory([0, 1], [Pose.identity()])


def test_trajectory_proto_round_trip() -> None:
    plan = _straight_plan()
    restored = Trajectory.from_proto(plan.to_proto())
    assert restored.timestamps_us == plan.timestamps_us
    assert np.allclose(restored.positions, plan.positions, atol=1e-6)


def test_transform_lifts_into_another_frame() -> None:
    """A rig-frame plan composed with the ego pose lands in the local frame."""
    ego = Pose.from_xyz_yaw(10.0, 5.0, 0.0, math.radians(90.0))
    lifted = _straight_plan().transform(ego)
    assert np.allclose(lifted.positions[0], [10.0, 5.0, 0.0], atol=1e-9)
    assert np.allclose(lifted.positions[2], [10.0, 7.0, 0.0], atol=1e-9)

    # ...and the inverse brings it back.
    restored = lifted.transform(ego.inverse())
    assert np.allclose(restored.positions, _straight_plan().positions, atol=1e-9)


def test_interpolate_between_samples() -> None:
    pose = _straight_plan().interpolate(50_000)
    assert pose is not None
    assert np.allclose(pose.position, [0.5, 0.0, 0.0], atol=1e-9)


def test_interpolate_clamps_outside_the_span() -> None:
    plan = _straight_plan()
    before, after = plan.interpolate(-1), plan.interpolate(10**9)
    assert before is not None and after is not None
    assert np.allclose(before.position, [0.0, 0.0, 0.0])
    assert np.allclose(after.position, [2.0, 0.0, 0.0])


def test_interpolate_takes_the_short_way_around() -> None:
    """Yaw interpolation across the +/-pi seam must not spin the long way."""
    plan = Trajectory.empty()
    plan.append(0, Pose.from_xyz_yaw(0.0, 0.0, 0.0, math.radians(175.0)))
    plan.append(100_000, Pose.from_xyz_yaw(0.0, 0.0, 0.0, math.radians(-175.0)))
    midpoint = plan.interpolate(50_000)
    assert midpoint is not None
    assert abs(midpoint.yaw) == pytest.approx(math.pi, abs=1e-6)


def test_interpolate_empty_returns_none() -> None:
    assert Trajectory.empty().interpolate(0) is None


def test_waypoints_to_proto() -> None:
    messages = waypoints_to_proto(np.array([[1.0, 2.0, 3.0]]))
    assert len(messages) == 1
    assert messages[0].x == pytest.approx(1.0)
    assert messages[0].z == pytest.approx(3.0)
