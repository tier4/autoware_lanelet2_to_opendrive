"""Minimal contract for delegating Autoware startup to ``autoware_carla_interface``.

The whole init sequence (localization pose initialization -> route/goal setting ->
engage / change-to-autonomous) is owned by the Autoware side, not driven step by
step from here:

* localization is auto-initialized by ``autoware_automatic_pose_initializer``
  (GNSS) or from a published ``/initialpose``;
* routing is auto-set by ``routing_adaptor`` from the goal pose;
* engaging and a single readiness signal are provided by small Autoware-side
  helpers (an auto-engage node + a readiness aggregator that ANDs
  localization/routing/operation-mode state).

So the scenario framework only needs to (1) hand Autoware the scenario's initial
pose and goal, and (2) wait for a single "ready" (initialized + routed + engaged
and now driving) signal.  That is the entire contract below.

The framework core depends only on the abstract :class:`AutowareBridge`; the
concrete gRPC transport (``GrpcAutowareBridge``) and the wire contract in
``proto/autoware_bridge/v0/autoware_bridge.proto`` are implemented separately so
this package never imports ROS 2 / rclpy.  Tests use
:class:`~autoware_carla_scenario.autoware_bridge.fake.FakeAutowareBridge`.

Ego pose/velocity for scenario conditions is NOT part of this contract: it is
read directly from the shared CARLA world (see :class:`AutowareEgoEntity`), so
the bridge carries only the readiness handshake.
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass


@dataclass(frozen=True)
class AutowareBridgeConfig:
    """Connection settings for a concrete :class:`AutowareBridge`.

    Attributes:
        address: ``host:port`` of the interface node's bridge gRPC server.
        timeout_s: Per-RPC timeout in seconds.
        ready_timeout_ticks: Maximum world ticks to wait for Autoware to become
            ready before the scenario fails.  At 20 Hz, ``1200`` is ~60 s.
        attach_timeout: Seconds to wait for the interface node to spawn the ego
            actor before :meth:`AutowareEgoEntity.spawn` raises.
    """

    address: str = "localhost:50052"
    timeout_s: float = 60.0
    ready_timeout_ticks: int = 1200
    attach_timeout: float = 30.0


@dataclass(frozen=True)
class Vector3:
    """A 3D vector (metres).  Shares the shape/name of the splatsim proto
    ``Vector3`` so the physics primitives can be unified in the future.

    Note: distinct from the frame-aware
    :class:`autoware_carla_scenario.kinematics.Vector3` - this one is the plain
    wire primitive that mirrors the proto; unifying the two is future work.
    """

    x: float
    y: float
    z: float


@dataclass(frozen=True)
class Quaternion:
    """An orientation quaternion in ``wxyz`` order.

    Matches the splatsim/gsplat proto convention (and the alpasim ``common.Quat``
    already vendored here).  The interface node converts to/from ROS
    ``geometry_msgs/Quaternion`` (``xyzw``) internally.
    """

    w: float
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class BridgePose:
    """A full 6-DoF map-frame pose for pose initialization and goal setting.

    Mirrors the splatsim proto ``Pose`` (a :class:`Vector3` ``position`` plus a
    :class:`Quaternion` ``rotation``) so the type is shareable with that stack;
    it is also structurally ``geometry_msgs/Pose``, how Autoware and
    ``autoware_carla_interface`` hold poses, so there is no lossy conversion at
    the boundary.

    The ``position.z`` disambiguates multi-level roads (overpasses): a 2D
    ``(x, y)`` alone can match both an elevated road and the surface beneath it,
    which would let localization or the interface node's ground projection snap
    to the wrong level.  The quaternion (not a bare yaw) represents pitch/roll on
    ramps and banked segments exactly.  The scenario already knows both from its
    snapped CARLA transform, so they are carried through rather than re-estimated.

    Use :meth:`from_yaw` when only a planar heading is available.

    Attributes:
        position: Map-frame position (metres).
        rotation: Map-frame orientation (``wxyz`` quaternion).
    """

    position: Vector3
    rotation: Quaternion

    @classmethod
    def from_yaw(cls, x: float, y: float, z: float, yaw: float) -> "BridgePose":
        """Build a pose from a position and a planar heading (yaw about +Z).

        Args:
            x: Position X in the map frame (metres).
            y: Position Y in the map frame (metres).
            z: Position Z in the map frame (metres).
            yaw: Heading in the map frame (radians).
        """
        half = yaw * 0.5
        return cls(
            Vector3(x, y, z),
            Quaternion(w=math.cos(half), x=0.0, y=0.0, z=math.sin(half)),
        )


class AutowareBridge(ABC):
    """Interface to the ``autoware_carla_interface`` node.

    Concrete implementations translate these calls into the transport used to
    reach the interface node (e.g. gRPC).  The contract is intentionally tiny:
    hand Autoware the mission, then poll a single readiness flag.
    """

    @abstractmethod
    def configure(self, initial_pose: BridgePose, goal: BridgePose) -> None:
        """Give Autoware the scenario's initial pose and goal.

        The Autoware side owns the rest: it initializes localization at
        *initial_pose*, plans a route to *goal*, and engages autonomous mode.

        Args:
            initial_pose: Map-frame pose to initialize localization at.
            goal: Map-frame goal pose to plan the route to.
        """

    @abstractmethod
    def is_ready(self) -> bool:
        """Return ``True`` once Autoware is initialized, routed, engaged, and driving.

        Must be non-blocking so it can be polled once per world tick.
        """

    def close(self) -> None:
        """Release any transport resources.  Must be idempotent.

        The default is a no-op; transport-backed implementations (e.g. a gRPC
        client) override this to close their channel.
        """
