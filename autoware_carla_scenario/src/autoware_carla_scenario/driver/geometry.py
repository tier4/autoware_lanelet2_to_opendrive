"""Rigid-body primitives for the ``egodriver`` gRPC contract.

The alpasim wire format expresses every pose as a translation plus a quaternion
(:class:`~autoware_carla_scenario.driver._proto.common_pb2.Pose`) and every path as a
list of timestamped poses
(:class:`~autoware_carla_scenario.driver._proto.common_pb2.Trajectory`).  This module
provides the Python-side counterparts and the conversions between them.

Conventions, matching alpasim:

* Quaternions are stored in **scipy order** ``(x, y, z, w)`` internally and converted to
  the protobuf's ``(w, x, y, z)`` field order on the wire.
* Composition is the standard rigid transform: ``(a @ b).position ==
  a.rotation_matrix @ b.position + a.position``.  So ``ego_pose @ pose_in_rig`` lifts a
  rig-frame pose into the local frame, and ``ego_pose.inverse() @ pose_in_local`` does
  the reverse.
* The frame itself is **right-handed** (x forward, y left, z up), unlike CARLA's
  left-handed world.  :mod:`autoware_carla_scenario.driver.observation` performs that
  conversion; nothing in this module knows about CARLA.

The class and method names deliberately mirror ``carla_driver_interface.geometry`` so a
future environment that can install that package upstream can swap the import.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, List, Optional

import numpy as np
from numpy.typing import NDArray

from ._proto import common_pb2


__all__ = ["Pose", "Trajectory", "waypoints_to_proto"]

#: Quaternion norms below this are treated as degenerate and replaced by the identity.
_QUAT_NORM_EPSILON: float = 1e-12

#: Timestamp gaps below this (microseconds) are treated as coincident when interpolating.
_TIME_EPSILON_US: int = 1


def _as_vector3(values: Iterable[float]) -> NDArray[np.float64]:
    """Return *values* as a ``(3,)`` float array.

    Raises:
        ValueError: If *values* does not hold exactly three components.
    """
    array = np.asarray(values, dtype=np.float64).reshape(-1)
    if array.size != 3:
        raise ValueError(f"expected 3 components, got {array.size}")
    return array


def _normalize_quat(values: Iterable[float]) -> NDArray[np.float64]:
    """Return *values* as a unit ``(4,)`` quaternion in ``(x, y, z, w)`` order.

    A zero-norm quaternion is replaced by the identity rather than producing NaNs, so a
    default-constructed protobuf message (all fields zero) round-trips to an identity
    rotation instead of poisoning downstream maths.

    Raises:
        ValueError: If *values* does not hold exactly four components.
    """
    array = np.asarray(values, dtype=np.float64).reshape(-1)
    if array.size != 4:
        raise ValueError(f"expected 4 quaternion components, got {array.size}")
    norm = float(np.linalg.norm(array))
    if norm < _QUAT_NORM_EPSILON:
        return np.array([0.0, 0.0, 0.0, 1.0])
    return array / norm


@dataclass(frozen=True, eq=False)
class Pose:
    """A rigid transform: a translation and a rotation.

    Args:
        position: ``(3,)`` translation in metres.
        quat_xyzw: ``(4,)`` unit quaternion in ``(x, y, z, w)`` order.  Normalised on
            construction.
    """

    position: NDArray[np.float64]
    quat_xyzw: NDArray[np.float64]

    def __post_init__(self) -> None:
        object.__setattr__(self, "position", _as_vector3(self.position))
        object.__setattr__(self, "quat_xyzw", _normalize_quat(self.quat_xyzw))

    # ------------------------------------------------------------------
    # Constructors
    # ------------------------------------------------------------------

    @classmethod
    def identity(cls) -> "Pose":
        """Return the identity transform."""
        return cls(np.zeros(3), np.array([0.0, 0.0, 0.0, 1.0]))

    @classmethod
    def from_xyz_yaw(cls, x: float, y: float, z: float, yaw: float) -> "Pose":
        """Return a pose with a yaw-only rotation.

        Args:
            x: Translation along the frame's x axis (metres).
            y: Translation along the frame's y axis (metres).
            z: Translation along the frame's z axis (metres).
            yaw: Rotation about the z axis (radians, counter-clockwise).
        """
        half = 0.5 * yaw
        return cls(
            np.array([x, y, z], dtype=np.float64),
            np.array([0.0, 0.0, math.sin(half), math.cos(half)]),
        )

    @classmethod
    def from_proto(cls, message: common_pb2.Pose) -> "Pose":
        """Return the pose described by a protobuf message."""
        return cls(
            np.array([message.vec.x, message.vec.y, message.vec.z], dtype=np.float64),
            np.array(
                [message.quat.x, message.quat.y, message.quat.z, message.quat.w],
                dtype=np.float64,
            ),
        )

    # ------------------------------------------------------------------
    # Derived quantities
    # ------------------------------------------------------------------

    @property
    def rotation_matrix(self) -> NDArray[np.float64]:
        """Return the ``(3, 3)`` rotation matrix for this pose."""
        x, y, z, w = self.quat_xyzw
        return np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ]
        )

    @property
    def yaw(self) -> float:
        """Return the rotation about the z axis in radians."""
        x, y, z, w = self.quat_xyzw
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def as_matrix(self) -> NDArray[np.float64]:
        """Return the ``(4, 4)`` homogeneous transform for this pose."""
        matrix = np.eye(4)
        matrix[:3, :3] = self.rotation_matrix
        matrix[:3, 3] = self.position
        return matrix

    # ------------------------------------------------------------------
    # Algebra
    # ------------------------------------------------------------------

    def __matmul__(self, other: "Pose") -> "Pose":
        """Compose two transforms: ``self`` applied after *other*."""
        sx, sy, sz, sw = self.quat_xyzw
        ox, oy, oz, ow = other.quat_xyzw
        quat = np.array(
            [
                sw * ox + sx * ow + sy * oz - sz * oy,
                sw * oy - sx * oz + sy * ow + sz * ox,
                sw * oz + sx * oy - sy * ox + sz * ow,
                sw * ow - sx * ox - sy * oy - sz * oz,
            ]
        )
        return Pose(self.rotation_matrix @ other.position + self.position, quat)

    def inverse(self) -> "Pose":
        """Return the inverse transform."""
        x, y, z, w = self.quat_xyzw
        inv_quat = np.array([-x, -y, -z, w])
        inv = Pose(np.zeros(3), inv_quat)
        return Pose(-(inv.rotation_matrix @ self.position), inv_quat)

    def transform_points(self, points: NDArray[np.float64]) -> NDArray[np.float64]:
        """Apply this transform to an ``(N, 3)`` array of points.

        Args:
            points: Points to transform.  An empty array is returned unchanged.
        """
        array = np.asarray(points, dtype=np.float64).reshape(-1, 3)
        if array.size == 0:
            return array
        return array @ self.rotation_matrix.T + self.position

    # ------------------------------------------------------------------
    # Serialisation
    # ------------------------------------------------------------------

    def to_proto(self) -> common_pb2.Pose:
        """Return the protobuf representation of this pose."""
        x, y, z, w = self.quat_xyzw
        return common_pb2.Pose(
            vec=common_pb2.Vec3(
                x=self.position[0], y=self.position[1], z=self.position[2]
            ),
            quat=common_pb2.Quat(w=w, x=x, y=y, z=z),
        )

    def to_proto_at_time(self, timestamp_us: int) -> common_pb2.PoseAtTime:
        """Return this pose stamped with *timestamp_us*."""
        return common_pb2.PoseAtTime(pose=self.to_proto(), timestamp_us=timestamp_us)


@dataclass
class Trajectory:
    """A time-ordered sequence of poses.

    Args:
        timestamps_us: Microsecond timestamps, one per pose.
        poses: Poses, all expressed in the same frame.

    Raises:
        ValueError: If the two sequences differ in length.
    """

    timestamps_us: List[int]
    poses: List[Pose]

    def __post_init__(self) -> None:
        if len(self.timestamps_us) != len(self.poses):
            raise ValueError(
                "timestamps_us and poses must have the same length, got "
                f"{len(self.timestamps_us)} and {len(self.poses)}"
            )

    # ------------------------------------------------------------------
    # Constructors
    # ------------------------------------------------------------------

    @classmethod
    def empty(cls) -> "Trajectory":
        """Return a trajectory with no poses."""
        return cls([], [])

    @classmethod
    def from_proto(cls, message: common_pb2.Trajectory) -> "Trajectory":
        """Return the trajectory described by a protobuf message."""
        timestamps = [int(entry.timestamp_us) for entry in message.poses]
        poses = [Pose.from_proto(entry.pose) for entry in message.poses]
        return cls(timestamps, poses)

    # ------------------------------------------------------------------
    # Access
    # ------------------------------------------------------------------

    def __len__(self) -> int:
        return len(self.poses)

    def __bool__(self) -> bool:
        return bool(self.poses)

    @property
    def positions(self) -> NDArray[np.float64]:
        """Return the positions as an ``(N, 3)`` array."""
        if not self.poses:
            return np.zeros((0, 3))
        return np.stack([pose.position for pose in self.poses])

    @property
    def last_pose(self) -> Optional[Pose]:
        """Return the final pose, or ``None`` when the trajectory is empty."""
        return self.poses[-1] if self.poses else None

    # ------------------------------------------------------------------
    # Mutation and transforms
    # ------------------------------------------------------------------

    def append(self, timestamp_us: int, pose: Pose) -> None:
        """Append *pose* stamped at *timestamp_us*."""
        self.timestamps_us.append(int(timestamp_us))
        self.poses.append(pose)

    def transform(self, delta: Pose) -> "Trajectory":
        """Return a copy with *delta* applied to every pose.

        ``trajectory.transform(ego_pose)`` lifts a rig-frame trajectory into the local
        frame; ``trajectory.transform(ego_pose.inverse())`` does the reverse.
        """
        return Trajectory(
            list(self.timestamps_us), [delta @ pose for pose in self.poses]
        )

    def interpolate(self, timestamp_us: int) -> Optional[Pose]:
        """Return the pose at *timestamp_us*, interpolating between samples.

        Positions are interpolated linearly and yaw angles along the shortest arc.
        Timestamps outside the trajectory's span clamp to the nearest endpoint.

        Returns:
            The interpolated pose, or ``None`` when the trajectory is empty.
        """
        if not self.poses:
            return None
        if len(self.poses) == 1 or timestamp_us <= self.timestamps_us[0]:
            return self.poses[0]
        if timestamp_us >= self.timestamps_us[-1]:
            return self.poses[-1]

        index = int(np.searchsorted(self.timestamps_us, timestamp_us))
        previous, following = self.poses[index - 1], self.poses[index]
        start, end = self.timestamps_us[index - 1], self.timestamps_us[index]
        span = end - start
        if span < _TIME_EPSILON_US:
            return following

        alpha = (timestamp_us - start) / span
        position = previous.position + alpha * (following.position - previous.position)
        delta_yaw = math.atan2(
            math.sin(following.yaw - previous.yaw),
            math.cos(following.yaw - previous.yaw),
        )
        yaw = previous.yaw + alpha * delta_yaw
        return Pose.from_xyz_yaw(position[0], position[1], position[2], yaw)

    # ------------------------------------------------------------------
    # Serialisation
    # ------------------------------------------------------------------

    def to_proto(self) -> common_pb2.Trajectory:
        """Return the protobuf representation of this trajectory."""
        return common_pb2.Trajectory(
            poses=[
                pose.to_proto_at_time(timestamp)
                for timestamp, pose in zip(self.timestamps_us, self.poses)
            ]
        )


def waypoints_to_proto(points: NDArray[np.float64]) -> List[common_pb2.Vec3]:
    """Return *points* as protobuf ``Vec3`` messages.

    Args:
        points: ``(N, 3)`` array of waypoints.
    """
    return [
        common_pb2.Vec3(x=float(point[0]), y=float(point[1]), z=float(point[2]))
        for point in points
    ]
