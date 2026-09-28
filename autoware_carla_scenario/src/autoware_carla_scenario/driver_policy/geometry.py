"""Poses and trajectories in the alpasim convention, backed by numpy only.

Ported from ``carla_driver_interface.geometry`` at
:data:`~autoware_carla_scenario.driver_policy.wire.CARLA_DRIVER_INTERFACE_REV`.
That package cannot be imported here (it needs Python >= 3.11). Keep the two
in step: a convention that drifts shows up as a driver steering the wrong way.
The conventions follow upstream exactly:

* Right-handed frames.  ``local`` is ENU-like and inertial; ``rig`` is
  body-fixed with x forward, y left, z up, origin at the rear axle centre
  projected onto the ground.
* Poses are **active** transforms: ``pose_local_to_rig`` maps a point given in
  the rig frame into the local frame, and equally *is* the rig's pose in local.
* Composition is ``a_to_c = a_to_b @ b_to_c``.
* Quaternions are stored ``(x, y, z, w)`` (scipy order) but the proto carries
  ``(w, x, y, z)``; the conversions here are the only place that matters.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .wire import DynamicState, PoseAtTime, Quat, Vec3
from .wire import Pose as PoseProto
from .wire import Trajectory as TrajectoryProto

__all__ = [
    "Pose",
    "Trajectory",
    "dynamic_state_proto",
    "euler_zyx_to_quat_xyzw",
    "quat_conjugate",
    "quat_mul",
    "quat_rotate",
    "quat_to_yaw",
    "yaw_to_quat_xyzw",
]


def euler_zyx_to_quat_xyzw(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Intrinsic Z-Y-X Euler angles (radians) -> quaternion ``(x, y, z, w)``.

    Yaw is applied first, then pitch, then roll -- Unreal's order, which is what
    :mod:`.conversions` needs after mirroring.
    """
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return np.array(
        [
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        ],
        dtype=np.float64,
    )


def yaw_to_quat_xyzw(yaw: float) -> np.ndarray:
    """Quaternion ``(x, y, z, w)`` for a rotation of ``yaw`` radians about +z."""
    half = 0.5 * yaw
    return np.array([0.0, 0.0, math.sin(half), math.cos(half)], dtype=np.float64)


def quat_mul(a_xyzw: np.ndarray, b_xyzw: np.ndarray) -> np.ndarray:
    """Hamilton product ``a * b``, i.e. apply ``b`` first, then ``a``."""
    ax, ay, az, aw = (float(v) for v in a_xyzw)
    bx, by, bz, bw = (float(v) for v in b_xyzw)
    return np.array(
        [
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        ],
        dtype=np.float64,
    )


def quat_conjugate(quat_xyzw: np.ndarray) -> np.ndarray:
    """Conjugate, which for a unit quaternion is also its inverse."""
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array([-x, -y, -z, w], dtype=np.float64)


def quat_rotate(quat_xyzw: np.ndarray, vector: np.ndarray) -> np.ndarray:
    """Rotate a single 3-vector by a unit quaternion.

    Written out in scalars rather than with ``np.cross``: for 3-vectors numpy's
    per-call overhead is several times the arithmetic, and this sits on the
    closed loop's hot path.
    """
    qx, qy, qz, qw = (float(v) for v in quat_xyzw)
    vx, vy, vz = (float(v) for v in vector)

    # t = 2 * (q_vec x v)
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)

    # v + w * t + q_vec x t
    return np.array(
        [
            vx + qw * tx + qy * tz - qz * ty,
            vy + qw * ty + qz * tx - qx * tz,
            vz + qw * tz + qx * ty - qy * tx,
        ],
        dtype=np.float64,
    )


def _unchecked_pose(position: np.ndarray, quat_xyzw: np.ndarray) -> Pose:
    """Build a :class:`Pose` from values already known to be valid.

    ``Pose.__post_init__`` re-converts and re-normalises on every construction,
    which is right for the public API and pure overhead for results we just
    computed from unit quaternions. Composition allocates a `Pose` per call on
    the hot path, so this bypass is worth the sharp edge.
    """
    pose = object.__new__(Pose)
    object.__setattr__(pose, "position", position)
    object.__setattr__(pose, "quat_xyzw", quat_xyzw)
    return pose


def quat_to_yaw(quat_xyzw: np.ndarray) -> float:
    """Yaw in radians extracted from a ``(x, y, z, w)`` quaternion."""
    x, y, z, w = (float(v) for v in quat_xyzw)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _quat_to_matrix(quat_xyzw: np.ndarray) -> np.ndarray:
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _normalize_quat(quat_xyzw: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(quat_xyzw))
    if norm == 0.0:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    return quat_xyzw / norm


@dataclass(frozen=True)
class Pose:
    """An active rigid transform: rotate by ``quat``, then translate by ``position``."""

    position: np.ndarray
    quat_xyzw: np.ndarray

    def __post_init__(self) -> None:
        object.__setattr__(
            self, "position", np.asarray(self.position, dtype=np.float64).reshape(3)
        )
        object.__setattr__(
            self,
            "quat_xyzw",
            _normalize_quat(np.asarray(self.quat_xyzw, dtype=np.float64).reshape(4)),
        )

    # -- constructors ------------------------------------------------------

    @staticmethod
    def identity() -> Pose:
        return Pose(np.zeros(3), np.array([0.0, 0.0, 0.0, 1.0]))

    @staticmethod
    def from_xyz_yaw(x: float, y: float, z: float, yaw: float) -> Pose:
        return Pose(np.array([x, y, z], dtype=np.float64), yaw_to_quat_xyzw(yaw))

    @staticmethod
    def from_proto(proto: PoseProto) -> Pose:
        return Pose(
            np.array([proto.vec.x, proto.vec.y, proto.vec.z], dtype=np.float64),
            # proto order is (w, x, y, z)
            np.array(
                [proto.quat.x, proto.quat.y, proto.quat.z, proto.quat.w],
                dtype=np.float64,
            ),
        )

    # -- accessors ---------------------------------------------------------

    @property
    def rotation_matrix(self) -> np.ndarray:
        return _quat_to_matrix(self.quat_xyzw)

    @property
    def yaw(self) -> float:
        return quat_to_yaw(self.quat_xyzw)

    # -- algebra -----------------------------------------------------------

    def __matmul__(self, other: Pose) -> Pose:
        """``self @ other`` composes ``a_to_b @ b_to_c -> a_to_c``.

        Done in quaternion space rather than by multiplying 4x4 matrices and
        decomposing the result back: composition is on the closed loop's hot
        path, and the matrix round trip both costs more and loses precision.
        """
        return _unchecked_pose(
            self.position + quat_rotate(self.quat_xyzw, other.position),
            quat_mul(self.quat_xyzw, other.quat_xyzw),
        )

    def inverse(self) -> Pose:
        conjugate = quat_conjugate(self.quat_xyzw)
        return _unchecked_pose(-quat_rotate(conjugate, self.position), conjugate)

    def transform_points(self, points: np.ndarray) -> np.ndarray:
        """Apply this pose to an ``(N, 3)`` array of points."""
        points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
        return points @ self.rotation_matrix.T + self.position

    # -- proto -------------------------------------------------------------

    def to_proto(self) -> PoseProto:
        x, y, z, w = (float(v) for v in self.quat_xyzw)
        return PoseProto(
            vec=Vec3(
                x=float(self.position[0]),
                y=float(self.position[1]),
                z=float(self.position[2]),
            ),
            quat=Quat(w=w, x=x, y=y, z=z),
        )

    def to_proto_at_time(self, timestamp_us: int) -> PoseAtTime:
        return PoseAtTime(pose=self.to_proto(), timestamp_us=int(timestamp_us))


@dataclass
class Trajectory:
    """Time-ordered poses sharing one frame.

    Timestamps are microseconds and must be strictly increasing, matching what
    the alpasim runtime sends and expects back.
    """

    timestamps_us: list[int]
    poses: list[Pose]

    def __post_init__(self) -> None:
        if len(self.timestamps_us) != len(self.poses):
            raise ValueError(
                f"timestamps and poses must match in length, got "
                f"{len(self.timestamps_us)} and {len(self.poses)}"
            )

    def __len__(self) -> int:
        return len(self.poses)

    def __bool__(self) -> bool:
        return bool(self.poses)

    @staticmethod
    def from_proto(proto: TrajectoryProto) -> Trajectory:
        return Trajectory(
            [int(p.timestamp_us) for p in proto.poses],
            [Pose.from_proto(p.pose) for p in proto.poses],
        )

    def to_proto(self) -> TrajectoryProto:
        return TrajectoryProto(
            poses=[
                pose.to_proto_at_time(ts)
                for ts, pose in zip(self.timestamps_us, self.poses, strict=True)
            ]
        )

    @property
    def positions(self) -> np.ndarray:
        """``(N, 3)`` array of positions; ``(0, 3)`` when empty."""
        if not self.poses:
            return np.zeros((0, 3), dtype=np.float64)
        return np.stack([p.position for p in self.poses])


def dynamic_state_proto(
    linear_velocity: np.ndarray,
    angular_velocity: np.ndarray,
    linear_acceleration: np.ndarray,
) -> DynamicState:
    """Build a :class:`DynamicState`; all vectors resolved in the rig frame.

    CARLA does not report angular acceleration, so it is always zero.
    """

    def vec(values: np.ndarray) -> Vec3:
        arr = np.asarray(values, dtype=np.float64).reshape(3)
        return Vec3(x=float(arr[0]), y=float(arr[1]), z=float(arr[2]))

    return DynamicState(
        linear_velocity=vec(linear_velocity),
        angular_velocity=vec(angular_velocity),
        linear_acceleration=vec(linear_acceleration),
        angular_acceleration=Vec3(),
    )
