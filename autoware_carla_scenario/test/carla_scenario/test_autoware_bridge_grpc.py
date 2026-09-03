"""Round-trip tests for the gRPC-backed :class:`GrpcAutowareBridge` server.

The bridge hosts the ``AutowareBridge`` server in-process on a real loopback port;
each test plays the ``autoware_carla_interface`` client, pulling the mission and
pushing readiness over a real gRPC channel.  This exercises the generated stubs
and the server end to end without needing a live Autoware stack or CARLA.
"""

from __future__ import annotations

import math
from contextlib import contextmanager
from typing import Iterator

import grpc
import pytest

from autoware_carla_scenario.autoware_bridge import (
    AutowareBridgeConfig,
    BridgePose,
    GrpcAutowareBridgeServer,
)
from autoware_carla_scenario.autoware_bridge._proto import (
    autoware_bridge_pb2 as pb2,
    autoware_bridge_pb2_grpc as pb2_grpc,
)
from autoware_carla_scenario.autoware_bridge.grpc_server import server_options

_INITIAL = BridgePose.from_yaw(x=1.0, y=2.0, z=0.5, yaw=0.5)
_GOAL = BridgePose.from_yaw(x=10.0, y=20.0, z=3.0, yaw=1.5)

_RPC_TIMEOUT_S = 5.0


@pytest.fixture
def bridge() -> Iterator[GrpcAutowareBridgeServer]:
    """Yield a bridge bound to an ephemeral loopback port; closed afterwards."""
    server = GrpcAutowareBridgeServer(AutowareBridgeConfig(address="localhost:0"))
    try:
        yield server
    finally:
        server.close()


@contextmanager
def _client(
    bridge: GrpcAutowareBridgeServer,
) -> Iterator[pb2_grpc.AutowareBridgeStub]:
    """Yield a stub dialling *bridge* (the interface-node role); channel closed after."""
    channel = grpc.insecure_channel(
        f"localhost:{bridge.port}", options=server_options()
    )
    try:
        yield pb2_grpc.AutowareBridgeStub(channel)
    finally:
        channel.close()


# ---------------------------------------------------------------------------
# GetMission
# ---------------------------------------------------------------------------


def test_get_mission_unavailable_before_configure(
    bridge: GrpcAutowareBridgeServer,
) -> None:
    with _client(bridge) as stub:
        response = stub.GetMission(pb2.GetMissionRequest(), timeout=_RPC_TIMEOUT_S)
    assert response.available is False


def test_get_mission_returns_configured_mission(
    bridge: GrpcAutowareBridgeServer,
) -> None:
    bridge.configure(_INITIAL, _GOAL)

    with _client(bridge) as stub:
        response = stub.GetMission(pb2.GetMissionRequest(), timeout=_RPC_TIMEOUT_S)

    assert response.available is True
    assert response.initial_pose.position.x == pytest.approx(1.0)
    assert response.initial_pose.position.y == pytest.approx(2.0)
    assert response.initial_pose.position.z == pytest.approx(0.5)
    # wxyz quaternion for yaw=0.5 must survive the round trip.
    assert response.initial_pose.rotation.w == pytest.approx(math.cos(0.25))
    assert response.initial_pose.rotation.z == pytest.approx(math.sin(0.25))
    assert response.goal.position.x == pytest.approx(10.0)
    assert response.goal.position.z == pytest.approx(3.0)


# ---------------------------------------------------------------------------
# ReportReadiness
# ---------------------------------------------------------------------------


def test_not_ready_until_reported(bridge: GrpcAutowareBridgeServer) -> None:
    assert bridge.is_ready() is False


def test_report_readiness_latches(bridge: GrpcAutowareBridgeServer) -> None:
    with _client(bridge) as stub:
        assert bridge.is_ready() is False
        stub.ReportReadiness(
            pb2.ReportReadinessRequest(ready=True), timeout=_RPC_TIMEOUT_S
        )
    assert bridge.is_ready() is True


def test_report_readiness_false_keeps_not_ready(
    bridge: GrpcAutowareBridgeServer,
) -> None:
    with _client(bridge) as stub:
        stub.ReportReadiness(
            pb2.ReportReadinessRequest(ready=False), timeout=_RPC_TIMEOUT_S
        )
    assert bridge.is_ready() is False


def test_readiness_latches_against_a_later_false(
    bridge: GrpcAutowareBridgeServer,
) -> None:
    # Once ready is reported, a later regressed ``ready=False`` must not clear it,
    # or the tick loop could miss the transition and time out.
    with _client(bridge) as stub:
        stub.ReportReadiness(
            pb2.ReportReadinessRequest(ready=True), timeout=_RPC_TIMEOUT_S
        )
        stub.ReportReadiness(
            pb2.ReportReadinessRequest(ready=False), timeout=_RPC_TIMEOUT_S
        )
    assert bridge.is_ready() is True


# ---------------------------------------------------------------------------
# Lifecycle
# ---------------------------------------------------------------------------


def test_close_is_idempotent(bridge: GrpcAutowareBridgeServer) -> None:
    bridge.close()
    bridge.close()  # must not raise


def test_bind_failure_raises() -> None:
    # Bind an ephemeral port, then try to reuse that exact port for a second
    # server: the explicit (non-zero) request must fail loudly.
    first = GrpcAutowareBridgeServer(AutowareBridgeConfig(address="localhost:0"))
    try:
        with pytest.raises(RuntimeError):
            GrpcAutowareBridgeServer(
                AutowareBridgeConfig(address=f"localhost:{first.port}")
            )
    finally:
        first.close()


def test_implements_autoware_bridge_contract(
    bridge: GrpcAutowareBridgeServer,
) -> None:
    from autoware_carla_scenario.autoware_bridge import AutowareBridge

    assert isinstance(bridge, AutowareBridge)
