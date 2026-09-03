"""Concrete :class:`AutowareBridge` that hosts the gRPC server.

Splatsim-consistent topology: the scenario library **hosts** this
``AutowareBridge`` gRPC server and ``autoware_carla_interface`` is the **client**
that dials it (see the topology note in
``proto/autoware_bridge/v0/autoware_bridge.proto``).  Because only the client
initiates, the two RPCs are direction-inverted from a "framework pushes" model:

* ``GetMission`` - the client pulls the scenario's mission (initial pose + goal).
  It returns ``available=False`` until :meth:`configure` has been called, so the
  interface node can start polling before the scenario hands over the mission.
* ``ReportReadiness`` - the client pushes its readiness once Autoware is
  initialized, routed, engaged, and driving.  The value is latched so
  :meth:`is_ready` can be polled once per world tick by
  :class:`~autoware_carla_scenario.entity.autoware_entity.AutowareEgoEntity`.

The framework never imports ROS 2 / rclpy: all ROS 2 / Autoware AD API I/O lives
on the client, inside the interface node.  This class only serves the mission and
latches the reported readiness behind a lock, since the RPCs are handled on the
server's thread pool while the entity drives :meth:`configure` / :meth:`is_ready`
from the tick loop.
"""

from __future__ import annotations

import logging
import threading
from concurrent import futures
from typing import Optional, Tuple

import grpc

from ._proto import autoware_bridge_pb2 as pb2
from ._proto import autoware_bridge_pb2_grpc as pb2_grpc
from .base import AutowareBridge, AutowareBridgeConfig, BridgePose

logger = logging.getLogger(__name__)

__all__ = ["GrpcAutowareBridgeServer", "server_options"]

#: Maximum gRPC message size.  The readiness contract is tiny (a couple of poses
#: and a bool), but the limits are set explicitly for parity with the egodriver
#: client's channel options.
MAX_MESSAGE_BYTES: int = 1 * 1024 * 1024

#: Default worker threads for the server's RPC thread pool.  Only one client (the
#: interface node) ever connects, polling ``GetMission`` and pushing readiness, so
#: a small pool is plenty.
_DEFAULT_MAX_WORKERS: int = 4


def server_options() -> list[tuple[str, int]]:
    """Return the gRPC server options used for the bridge server."""
    return [
        ("grpc.max_send_message_length", MAX_MESSAGE_BYTES),
        ("grpc.max_receive_message_length", MAX_MESSAGE_BYTES),
    ]


def _pose_to_proto(pose: BridgePose) -> pb2.Pose:
    """Convert a :class:`BridgePose` to the wire ``Pose`` (both ``wxyz``)."""
    return pb2.Pose(
        position=pb2.Vector3(x=pose.position.x, y=pose.position.y, z=pose.position.z),
        rotation=pb2.Quaternion(
            w=pose.rotation.w,
            x=pose.rotation.x,
            y=pose.rotation.y,
            z=pose.rotation.z,
        ),
    )


class _AutowareBridgeServicer(pb2_grpc.AutowareBridgeServicer):
    """Serves the two RPCs by delegating to the owning bridge (thread-safe)."""

    def __init__(self, bridge: "GrpcAutowareBridgeServer") -> None:
        self._bridge = bridge

    def GetMission(self, request, context):  # noqa: N802, ANN001 - gRPC signature
        return self._bridge._mission_response()

    def ReportReadiness(self, request, context):  # noqa: N802, ANN001 - gRPC signature
        self._bridge._record_readiness(bool(request.ready))
        return pb2.ReportReadinessResponse()


class GrpcAutowareBridgeServer(AutowareBridge):
    """Hosts the ``AutowareBridge`` gRPC server for the interface-node client.

    The server starts listening as soon as the instance is constructed, so the
    interface node can begin polling ``GetMission`` before the scenario hands over
    a mission (it just gets ``available=False`` until :meth:`configure`).

    Args:
        config: Connection settings.  ``None`` uses
            :class:`~autoware_carla_scenario.autoware_bridge.base.AutowareBridgeConfig`
            defaults.  Use ``address="localhost:0"`` (or ``host:0``) to bind an
            ephemeral port and read the chosen port from :attr:`port` - handy for
            in-process tests.
        max_workers: Size of the server's RPC thread pool.

    Raises:
        RuntimeError: If the configured address cannot be bound.
    """

    def __init__(
        self,
        config: Optional[AutowareBridgeConfig] = None,
        *,
        max_workers: int = _DEFAULT_MAX_WORKERS,
    ) -> None:
        self._config = config or AutowareBridgeConfig()
        self._lock = threading.Lock()
        self._mission: Optional[Tuple[BridgePose, BridgePose]] = None
        self._ready: bool = False
        self._closed: bool = False

        self._server = grpc.server(
            futures.ThreadPoolExecutor(max_workers=max_workers),
            # so_reuseport=0 disables gRPC's default SO_REUSEPORT so that binding
            # an already-used port fails (returns 0) instead of silently sharing
            # the port with another server and splitting RPCs between them.
            options=[*server_options(), ("grpc.so_reuseport", 0)],
        )
        pb2_grpc.add_AutowareBridgeServicer_to_server(
            _AutowareBridgeServicer(self), self._server
        )
        self._port = self._server.add_insecure_port(self._config.address)
        # add_insecure_port returns 0 when the bind fails; a caller that asked for
        # a specific (non-zero) port must not silently get an unusable server.
        if self._port == 0 and not self._config.address.endswith(":0"):
            raise RuntimeError(
                f"Failed to bind the AutowareBridge server to "
                f"{self._config.address!r} (port already in use?)."
            )
        self._server.start()
        logger.info("AutowareBridge server listening on port %d", self._port)

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def config(self) -> AutowareBridgeConfig:
        """Return the bridge connection settings."""
        return self._config

    @property
    def port(self) -> int:
        """Return the port the server is actually listening on."""
        return self._port

    # ------------------------------------------------------------------
    # Servicer callbacks (run on the server thread pool)
    # ------------------------------------------------------------------

    def _mission_response(self) -> pb2.GetMissionResponse:
        """Build the ``GetMission`` response from the current mission state."""
        with self._lock:
            mission = self._mission
        if mission is None:
            return pb2.GetMissionResponse(available=False)
        initial, goal = mission
        return pb2.GetMissionResponse(
            available=True,
            initial_pose=_pose_to_proto(initial),
            goal=_pose_to_proto(goal),
        )

    def _record_readiness(self, ready: bool) -> None:
        """Latch the readiness the client reported."""
        with self._lock:
            was_ready = self._ready
            self._ready = ready
        if ready and not was_ready:
            logger.info("Autoware reported ready via ReportReadiness")

    # ------------------------------------------------------------------
    # AutowareBridge contract (driven by the entity / tick loop)
    # ------------------------------------------------------------------

    def configure(self, initial_pose: BridgePose, goal: BridgePose) -> None:
        with self._lock:
            self._mission = (initial_pose, goal)
        logger.info("AutowareBridge mission configured; awaiting Autoware readiness")

    def is_ready(self) -> bool:
        with self._lock:
            return self._ready

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        # grace=None stops immediately; only one short-lived client ever connects.
        self._server.stop(grace=None)
        logger.info("AutowareBridge server on port %d stopped", self._port)
