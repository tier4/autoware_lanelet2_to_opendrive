"""Round-trip tests for the egodriver gRPC client.

A stub policy is served in-process on a real gRPC channel, so these tests exercise the
generated stubs and the message construction end to end without needing an external
driver server or CARLA.
"""

from __future__ import annotations

import math
from concurrent import futures
from typing import Iterator, List

import grpc
import numpy as np
import pytest

from autoware_carla_scenario.driver._proto import (
    common_pb2,
    egodriver_pb2,
    egodriver_pb2_grpc,
)
from autoware_carla_scenario.driver.base import (
    DriverCameraConfig,
    DriverClientConfig,
    EgoObservation,
)
from autoware_carla_scenario.driver.egodriver_client import (
    EgoDriverGrpcClient,
    channel_options,
)
from autoware_carla_scenario.driver.geometry import Pose, Trajectory


class _StubPolicy(egodriver_pb2_grpc.EgodriverServiceServicer):
    """Records every request and answers ``drive`` with a canned straight-ahead plan."""

    def __init__(self, *, terminate: bool = False) -> None:
        self.sessions: List[egodriver_pb2.DriveSessionRequest] = []
        self.images: List[egodriver_pb2.RolloutCameraImage] = []
        self.egomotion: List[egodriver_pb2.RolloutEgoTrajectory] = []
        self.routes: List[egodriver_pb2.RouteRequest] = []
        self.drives: List[egodriver_pb2.DriveRequest] = []
        self.closed: List[str] = []
        self._terminate = terminate

    def get_version(self, request, context):  # noqa: ANN001, D102
        return common_pb2.VersionId(version_id="stub", git_hash="0" * 40)

    def start_session(self, request, context):  # noqa: ANN001, D102
        self.sessions.append(request)
        return common_pb2.SessionRequestStatus()

    def close_session(self, request, context):  # noqa: ANN001, D102
        self.closed.append(request.session_uuid)
        return common_pb2.Empty()

    def submit_image_observation(self, request, context):  # noqa: ANN001, D102
        self.images.append(request)
        return common_pb2.Empty()

    def submit_egomotion_observation(self, request, context):  # noqa: ANN001, D102
        self.egomotion.append(request)
        return common_pb2.Empty()

    def submit_route(self, request, context):  # noqa: ANN001, D102
        self.routes.append(request)
        return common_pb2.Empty()

    def drive(self, request, context):  # noqa: ANN001, D102
        self.drives.append(request)
        plan = Trajectory.empty()
        for index in range(1, 5):
            plan.append(
                request.time_now_us + index * 100_000,
                Pose.from_xyz_yaw(index * 0.8, 0.0, 0.0, 0.0),
            )
        return egodriver_pb2.DriveResponse(
            trajectory=plan.to_proto(),
            terminate_session=self._terminate,
            debug_info=egodriver_pb2.DriveResponse.DebugInfo(
                unstructured_debug_info=b"stub-debug"
            ),
        )


@pytest.fixture
def policy() -> Iterator[_StubPolicy]:
    """Yield a stub policy; the server is torn down afterwards."""
    yield from _serve(_StubPolicy())


@pytest.fixture
def terminating_policy() -> Iterator[_StubPolicy]:
    """Yield a stub policy that asks to end the session."""
    yield from _serve(_StubPolicy(terminate=True))


def _serve(stub: _StubPolicy) -> Iterator[_StubPolicy]:
    """Run *stub* on a real loopback server for the duration of a test."""
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=2))
    egodriver_pb2_grpc.add_EgodriverServiceServicer_to_server(stub, server)
    stub.port = server.add_insecure_port("localhost:0")  # type: ignore[attr-defined]
    server.start()
    try:
        yield stub
    finally:
        server.stop(grace=None)


def _client(stub: _StubPolicy, **overrides) -> EgoDriverGrpcClient:
    """Return a client wired to *stub* over a fresh channel."""
    config = DriverClientConfig(
        address=f"localhost:{stub.port}",  # type: ignore[attr-defined]
        timeout_s=10.0,
        cameras=(DriverCameraConfig(logical_id="camera_front_wide_120fov"),),
        **overrides,
    )
    channel = grpc.insecure_channel(config.address, options=channel_options())
    return EgoDriverGrpcClient(config, channel=channel)


def _observation(timestamp_us: int = 0) -> EgoObservation:
    return EgoObservation(
        timestamp_us=timestamp_us,
        pose=Pose.from_xyz_yaw(10.0, 20.0, 0.0, math.radians(45.0)),
        linear_velocity=np.array([5.0, 0.0, 0.0]),
        angular_velocity=np.array([0.0, 0.0, 0.1]),
        linear_acceleration=np.array([1.0, 0.0, 0.0]),
        speed_mps=5.0,
    )


# ---------------------------------------------------------------------------
# Session lifecycle
# ---------------------------------------------------------------------------


def test_start_session_sends_the_camera_rig(policy: _StubPolicy) -> None:
    client = _client(policy)
    try:
        client.start_session("session-1", "Town10HD_Opt")
    finally:
        client.close_session()

    assert len(policy.sessions) == 1
    request = policy.sessions[0]
    assert request.session_uuid == "session-1"
    assert request.debug_info.scene_id == "Town10HD_Opt"

    cameras = request.rollout_spec.vehicle.available_cameras
    assert [camera.logical_id for camera in cameras] == ["camera_front_wide_120fov"]
    intrinsics = cameras[0].intrinsics
    assert intrinsics.resolution_w == 960
    assert intrinsics.resolution_h == 604
    # 960 px across a 120 deg FOV.
    assert intrinsics.opencv_pinhole_param.focal_length_x == pytest.approx(
        960 / (2 * math.tan(math.radians(60.0))), rel=1e-6
    )
    assert intrinsics.opencv_pinhole_param.principal_point_x == pytest.approx(480.0)


def test_close_session_is_idempotent(policy: _StubPolicy) -> None:
    client = _client(policy)
    client.start_session("session-1", "scene")
    client.close_session()
    client.close_session()

    assert policy.closed == ["session-1"]
    assert client.session_uuid is None


def test_calls_before_start_session_raise(policy: _StubPolicy) -> None:
    client = _client(policy)
    with pytest.raises(RuntimeError, match="No driver session is open"):
        client.drive(0, 100_000)


# ---------------------------------------------------------------------------
# Observations
# ---------------------------------------------------------------------------


def test_submit_route_sends_rig_frame_waypoints(policy: _StubPolicy) -> None:
    client = _client(policy)
    try:
        client.start_session("session-1", "scene")
        client.submit_route(1_000, np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]))
    finally:
        client.close_session()

    assert len(policy.routes) == 1
    route = policy.routes[0].route
    assert route.timestamp_us == 1_000
    assert [(w.x, w.y, w.z) for w in route.waypoints] == [
        (1.0, 2.0, 3.0),
        (4.0, 5.0, 6.0),
    ]


def test_submit_image_observation(policy: _StubPolicy) -> None:
    client = _client(policy)
    try:
        client.start_session("session-1", "scene")
        client.submit_image_observation("camera_front_wide_120fov", 10, 20, b"jpeg")
    finally:
        client.close_session()

    assert len(policy.images) == 1
    image = policy.images[0].camera_image
    assert image.logical_id == "camera_front_wide_120fov"
    assert image.frame_start_us == 10
    assert image.frame_end_us == 20
    assert image.image_bytes == b"jpeg"


def test_submit_egomotion_observation(policy: _StubPolicy) -> None:
    client = _client(policy)
    try:
        client.start_session("session-1", "scene")
        client.submit_egomotion_observation(_observation(42))
    finally:
        client.close_session()

    assert len(policy.egomotion) == 1
    message = policy.egomotion[0]
    assert len(message.trajectory.poses) == 1
    assert message.trajectory.poses[0].timestamp_us == 42

    restored = Pose.from_proto(message.trajectory.poses[0].pose)
    assert np.allclose(restored.position, [10.0, 20.0, 0.0], atol=1e-5)
    assert restored.yaw == pytest.approx(math.radians(45.0), abs=1e-5)

    assert len(message.dynamic_states) == 1
    assert message.dynamic_states[0].linear_velocity.x == pytest.approx(5.0)


# ---------------------------------------------------------------------------
# Drive
# ---------------------------------------------------------------------------


def test_drive_returns_the_policy_plan(policy: _StubPolicy) -> None:
    client = _client(policy)
    try:
        client.start_session("session-1", "scene")
        outcome = client.drive(1_000_000, 1_100_000)
    finally:
        client.close_session()

    assert len(policy.drives) == 1
    assert policy.drives[0].time_now_us == 1_000_000
    assert policy.drives[0].time_query_us == 1_100_000

    assert len(outcome.trajectory) == 4
    assert outcome.trajectory.timestamps_us[0] == 1_100_000
    assert np.allclose(outcome.trajectory.positions[0], [0.8, 0.0, 0.0], atol=1e-5)
    assert outcome.terminate_session is False
    assert outcome.debug_info == b"stub-debug"


def test_drive_reports_termination(terminating_policy: _StubPolicy) -> None:
    client = _client(terminating_policy)
    try:
        client.start_session("session-1", "scene")
        outcome = client.drive(0, 100_000)
    finally:
        client.close_session()

    assert outcome.terminate_session is True


def test_unreachable_policy_fails_at_start_session() -> None:
    """A dead address must fail immediately, not midway through the scenario."""
    config = DriverClientConfig(address="localhost:1", timeout_s=1.0)
    client = EgoDriverGrpcClient(config)
    with pytest.raises(grpc.RpcError):
        client.start_session("session-1", "scene")
    client.close_session()
