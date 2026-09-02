"""Semantic contract for communicating with ``autoware_carla_interface``.

The ``egodriver`` gRPC contract in :mod:`autoware_carla_scenario.driver` is an
alpasim-compatible *driving policy* interface (sensor observations in → planned
trajectory out): the scenario framework stays in the control loop.  Autoware is
different — the ``autoware_carla_interface`` ROS 2 node reads CARLA sensors and
applies control to the ego **directly**, so the framework is *not* in the
control loop.  What the framework needs instead is an *initialization and
monitoring* contract: drive Autoware's startup handshake (pose init, routing,
engage) and observe its state.  That contract is defined here, deliberately
separate from ``egodriver``.

The framework core depends only on the abstract :class:`AutowareBridge`;
concrete transports (a gRPC client talking to the interface node) live behind
it, and tests use
:class:`~autoware_carla_scenario.autoware_bridge.fake.FakeAutowareBridge`.  The
gRPC wire contract mirroring these methods is in
``proto/autoware_bridge/v0/autoware_bridge.proto``; each method maps 1:1 onto an
RPC.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from typing import Optional


@dataclass(frozen=True)
class AutowareBridgeConfig:
    """Connection settings for a concrete :class:`AutowareBridge`.

    Mirrors the style of
    :class:`~autoware_carla_scenario.driver.base.DriverClientConfig`.  Poses are
    per-scenario and are passed to the entity separately, not stored here.

    Attributes:
        address: ``host:port`` of the interface node's bridge gRPC server.
        timeout_s: Per-RPC timeout in seconds.
        step_timeout: Maximum ticks a single ``WAIT_*`` handshake state may
            spend before the initialization sequence fails.  At 20 Hz, ``600``
            is ~30 s.
        attach_timeout: Seconds to wait for the interface node to spawn the ego
            actor before :meth:`AutowareEgoEntity.spawn` raises.
    """

    address: str = "localhost:50052"
    timeout_s: float = 60.0
    step_timeout: int = 600
    attach_timeout: float = 30.0


@dataclass(frozen=True)
class BridgePose:
    """A 2D map-frame pose used for pose initialization and goal setting.

    The pose is expressed in the Autoware ``map`` frame (metres, radians).
    A 2D pose (x, y, yaw) is sufficient for both ``initialpose`` and goal
    poses; the interface node projects it onto the road surface.

    Attributes:
        x: Position X in the map frame (metres).
        y: Position Y in the map frame (metres).
        yaw: Heading in the map frame (radians).
    """

    x: float
    y: float
    yaw: float


class LocalizationState(Enum):
    """Autoware localization initialization state.

    Mirrors ``autoware_adapi_v1_msgs/LocalizationInitializationState``.
    """

    UNKNOWN = 0
    UNINITIALIZED = 1
    INITIALIZING = 2
    INITIALIZED = 3


class RoutingState(Enum):
    """Autoware routing state.

    Mirrors ``autoware_adapi_v1_msgs/RouteState``.
    """

    UNKNOWN = 0
    UNSET = 1
    SET = 2
    ARRIVED = 3
    CHANGING = 4


class OperationMode(Enum):
    """Autoware operation mode.

    Mirrors ``autoware_adapi_v1_msgs/OperationModeState``.
    """

    UNKNOWN = 0
    STOP = 1
    AUTONOMOUS = 2
    LOCAL = 3
    REMOTE = 4


@dataclass(frozen=True)
class VehicleStatus:
    """Snapshot of the ego vehicle's reported status.

    Values originate from Autoware's ``/vehicle/status/*`` topics as relayed
    by the interface node.

    Attributes:
        longitudinal_velocity: Longitudinal velocity (m/s).
        steering_tire_angle: Front steering tire angle (radians).
        operation_mode: Current :class:`OperationMode`.
        is_autonomous_available: Whether autonomous mode can be engaged.
    """

    longitudinal_velocity: float
    steering_tire_angle: float
    operation_mode: OperationMode
    is_autonomous_available: bool


class AutowareBridge(ABC):
    """Interface to the ``autoware_carla_interface`` node.

    Concrete implementations translate these calls into the transport used to
    reach the interface node (e.g. gRPC).

    Control vs monitor plane.  The *command* methods (:meth:`initialize_pose`,
    :meth:`set_route`, :meth:`change_to_autonomous`) are infrequent and issued
    only at handshake transitions.  The *query* methods
    (:meth:`get_localization_state`, :meth:`get_routing_state`,
    :meth:`get_operation_mode`, :meth:`get_vehicle_status`) MUST be non-blocking
    and return promptly, because
    :class:`~autoware_carla_scenario.autoware_bridge.init_sequence.AutowareInitSequence`
    polls one per world tick and the framework's 20 Hz tick loop must never
    stall on the network.  A transport-backed implementation therefore does not
    make a blocking RPC per query: it opens a server-streaming subscription in
    :meth:`start`, mirrors the pushed snapshots into memory on a background
    thread, and the query methods simply read that mirror.  Commands and the
    state stream are handled concurrently by the server (grpc.aio or a thread
    pool).  High-bandwidth data (sensors, control, ``/clock``, tf) never flows
    over this bridge — it stays on ROS 2 / direct CARLA control in the interface
    node.
    """

    # -- Readiness -----------------------------------------------------

    @abstractmethod
    def is_autoware_ready(self) -> bool:
        """Return ``True`` once Autoware's stack is up and accepting commands.

        This gates the rest of the handshake: pose initialization should not
        be attempted before the localization and planning nodes are alive.
        """

    # -- Pose initialization ------------------------------------------

    @abstractmethod
    def initialize_pose(self, pose: BridgePose) -> None:
        """Request Autoware to initialize localization at *pose*.

        Corresponds to publishing ``initialpose`` (the interface node also
        repositions the CARLA ego actor accordingly).

        Args:
            pose: The initial map-frame pose of the ego vehicle.
        """

    @abstractmethod
    def get_localization_state(self) -> LocalizationState:
        """Return the current localization initialization state."""

    # -- Routing -------------------------------------------------------

    @abstractmethod
    def set_route(self, goal: BridgePose) -> None:
        """Request Autoware to plan a route to *goal*.

        Args:
            goal: The goal map-frame pose.
        """

    @abstractmethod
    def get_routing_state(self) -> RoutingState:
        """Return the current routing state."""

    # -- Engage --------------------------------------------------------

    @abstractmethod
    def change_to_autonomous(self) -> None:
        """Request Autoware to change the operation mode to autonomous (engage)."""

    @abstractmethod
    def get_operation_mode(self) -> OperationMode:
        """Return the current operation mode."""

    # -- Status --------------------------------------------------------

    @abstractmethod
    def get_vehicle_status(self) -> VehicleStatus:
        """Return the latest ego vehicle status snapshot."""

    def get_estimated_pose(self) -> Optional[BridgePose]:
        """Return Autoware's latest *estimated* map-frame pose, or ``None``.

        This is Autoware's localization belief (``/localization/kinematic_state``)
        relayed over :func:`StreamState`, useful for monitoring localization
        accuracy.  It is **not** how scenario pass/fail conditions get the ego
        pose: those read the CARLA actor's ground-truth transform directly from
        the shared world (see :class:`AutowareEgoEntity`), so the bridge is not
        on that path.  The default returns ``None``; transport-backed
        implementations override it.
        """
        return None

    # -- Lifecycle -----------------------------------------------------

    def start(self) -> None:
        """Open the transport and begin mirroring state.  Must be idempotent.

        The default is a no-op.  Transport-backed implementations (e.g. a gRPC
        client) override this to dial the channel and start the background
        :func:`StreamState` consumer that keeps the query methods non-blocking.
        Called by :meth:`AutowareEgoEntity.on_scenario_start` before the
        handshake begins.
        """

    def close(self) -> None:
        """Release any transport resources.  Must be idempotent.

        The default is a no-op; transport-backed implementations (e.g. a gRPC
        client) override this to stop the state stream and close their channel.
        """
