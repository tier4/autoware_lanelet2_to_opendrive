"""Unit tests for the egodriver policy (``autoware_carla_scenario.driver_policy``).

The closed loop runs over real gRPC against an in-process driver, with a small
kinematic stand-in for the CARLA world, so no CARLA server is needed.
"""

from __future__ import annotations

import math
import sys
import types
from concurrent import futures
from dataclasses import dataclass, field
from typing import Any, Iterator, List, Optional

import grpc
import numpy as np
import pytest

from autoware_carla_scenario.driver_policy import (
    CameraConfig,
    EgoDriverEntity,
    EgoDriverPolicy,
    EgoDriverPolicyConfig,
)
from autoware_carla_scenario.driver_policy import conversions, wire
from autoware_carla_scenario.driver_policy.control import TrajectoryFollower
from autoware_carla_scenario.driver_policy.geometry import Pose, Trajectory
from autoware_carla_scenario.entity.ego import EgoVehicle

# ---------------------------------------------------------------------------
# A minimal stand-in for the parts of the CARLA API the policy touches
# ---------------------------------------------------------------------------


@dataclass
class _Vec:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class _Rot:
    pitch: float = 0.0
    yaw: float = 0.0
    roll: float = 0.0


@dataclass
class _Transform:
    location: _Vec = field(default_factory=_Vec)
    rotation: _Rot = field(default_factory=_Rot)


@dataclass
class _VehicleControl:
    throttle: float = 0.0
    steer: float = 0.0
    brake: float = 0.0
    hand_brake: bool = False
    reverse: bool = False


def _fake_carla_module() -> types.ModuleType:
    module = types.ModuleType("carla")
    module.VehicleControl = _VehicleControl  # type: ignore[attr-defined]
    module.Transform = _Transform  # type: ignore[attr-defined]
    module.Location = _Vec  # type: ignore[attr-defined]
    module.Rotation = _Rot  # type: ignore[attr-defined]
    return module


class _Waypoint:
    """A waypoint on a straight road along CARLA +x at ``y = lane_y``."""

    road_id = 1
    lane_id = -1
    is_junction = False

    def __init__(self, x: float, lane_y: float) -> None:
        self.transform = _Transform(_Vec(x, lane_y, 0.0))
        self._lane_y = lane_y

    def next(self, step: float) -> List["_Waypoint"]:
        return [_Waypoint(self.transform.location.x + step, self._lane_y)]


class _Map:
    name = "Carla/Maps/FakeTown"

    def __init__(self, lane_y: float) -> None:
        self._lane_y = lane_y

    def get_waypoint(self, location: _Vec, project_to_road: bool = True) -> _Waypoint:
        return _Waypoint(location.x, self._lane_y)


class _ActorList(list):
    def filter(self, pattern: str) -> "_ActorList":
        return _ActorList()


class _Ego:
    """Kinematic bicycle driven by the last applied control."""

    id = 1
    type_id = "vehicle.fake"
    bounding_box = types.SimpleNamespace(extent=_Vec(2.4, 1.0, 0.8))

    def __init__(self, y: float = 0.0, yaw_deg: float = 0.0) -> None:
        self.transform = _Transform(_Vec(0.0, y, 0.0), _Rot(yaw=yaw_deg))
        self.speed = 0.0
        self.control = _VehicleControl()
        self.controls: List[_VehicleControl] = []

    def get_transform(self) -> _Transform:
        return self.transform

    def get_location(self) -> _Vec:
        return self.transform.location

    def get_velocity(self) -> _Vec:
        yaw = math.radians(self.transform.rotation.yaw)
        return _Vec(self.speed * math.cos(yaw), self.speed * math.sin(yaw), 0.0)

    def get_acceleration(self) -> _Vec:
        return _Vec()

    def get_angular_velocity(self) -> _Vec:
        return _Vec()

    def get_physics_control(self) -> Any:
        raise RuntimeError("no physics in the fake")

    def get_speed_limit(self) -> float:
        return 36.0

    def is_at_traffic_light(self) -> bool:
        return False

    def get_traffic_light(self) -> None:
        return None

    def apply_control(self, control: _VehicleControl) -> None:
        self.control = control
        self.controls.append(control)

    def step(self, dt: float) -> None:
        accel = 4.0 * self.control.throttle - 8.0 * self.control.brake
        self.speed = max(0.0, self.speed + accel * dt)
        # CARLA steers positive to the right, i.e. towards +y, i.e. +yaw.
        yaw_rate = self.speed / 2.8 * math.tan(self.control.steer * math.radians(70))
        rot = self.transform.rotation
        rot.yaw += math.degrees(yaw_rate * dt)
        loc = self.transform.location
        loc.x += self.speed * math.cos(math.radians(rot.yaw)) * dt
        loc.y += self.speed * math.sin(math.radians(rot.yaw)) * dt


class _World:
    def __init__(self, ego: _Ego, lane_y: float = 0.0) -> None:
        self.ego = ego
        self.frame = 0
        self._map = _Map(lane_y)

    def get_settings(self) -> Any:
        return types.SimpleNamespace(fixed_delta_seconds=0.05)

    def get_map(self) -> _Map:
        return self._map

    def get_actors(self) -> _ActorList:
        return _ActorList()

    def get_weather(self) -> Any:
        return types.SimpleNamespace(cloudiness=10.0)

    def get_snapshot(self) -> Any:
        return types.SimpleNamespace(
            frame=self.frame,
            timestamp=types.SimpleNamespace(elapsed_seconds=self.frame * 0.05),
        )

    def tick(self) -> int:
        self.frame += 1
        self.ego.step(0.05)
        return self.frame


# ---------------------------------------------------------------------------
# An in-process driver that follows the route it is sent at a fixed speed
# ---------------------------------------------------------------------------


class _RecordingDriver(wire.EgodriverServiceServicer):
    SPEED_MPS = 6.0

    def __init__(self, terminate_after: Optional[int] = None) -> None:
        self.sessions: List[Any] = []
        self.closed: List[str] = []
        self.egomotion: List[Any] = []
        self.routes: List[Any] = []
        self.drives: List[Any] = []
        self.images: List[Any] = []
        self._terminate_after = terminate_after

    def get_version(self, request: Any, context: Any) -> Any:
        return wire.VersionId(
            version_id="recording-driver",
            grpc_api_version=wire.API_VERSION_MESSAGE,
        )

    def start_session(self, request: Any, context: Any) -> Any:
        self.sessions.append(request)
        return wire.SessionRequestStatus()

    def close_session(self, request: Any, context: Any) -> Any:
        self.closed.append(request.session_uuid)
        return wire.Empty()

    def submit_image_observation(self, request: Any, context: Any) -> Any:
        self.images.append(request)
        return wire.Empty()

    def submit_egomotion_observation(self, request: Any, context: Any) -> Any:
        self.egomotion.append(request)
        return wire.Empty()

    def submit_route(self, request: Any, context: Any) -> Any:
        self.routes.append(request)
        return wire.Empty()

    def submit_recording_ground_truth(self, request: Any, context: Any) -> Any:
        return wire.Empty()

    def drive(self, request: Any, context: Any) -> Any:
        self.drives.append(request)
        if (
            self._terminate_after is not None
            and len(self.drives) > self._terminate_after
        ):
            return wire.DriveResponse(terminate_session=True)

        ego = Pose.from_proto(self.egomotion[-1].trajectory.poses[-1].pose)
        route = self.routes[-1].route
        points = np.array([[p.x, p.y, p.z] for p in route.waypoints])
        distances = np.concatenate(
            [[0.0], np.cumsum(np.linalg.norm(np.diff(points[:, :2], axis=0), axis=1))]
        )
        plan = Trajectory(
            [
                request.time_now_us + int(d / self.SPEED_MPS * 1e6) + i
                for i, d in enumerate(distances)
            ],
            [ego @ Pose(p, np.array([0.0, 0.0, 0.0, 1.0])) for p in points],
        )
        return wire.DriveResponse(trajectory=plan.to_proto())


@pytest.fixture
def fake_carla(monkeypatch: pytest.MonkeyPatch) -> types.ModuleType:
    module = _fake_carla_module()
    monkeypatch.setitem(sys.modules, "carla", module)
    return module


def _serve(driver: _RecordingDriver) -> Iterator[grpc.Channel]:
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=2))
    wire.add_EgodriverServiceServicer_to_server(driver, server)
    port = server.add_insecure_port("127.0.0.1:0")
    server.start()
    channel = grpc.insecure_channel(f"127.0.0.1:{port}")
    try:
        yield channel
    finally:
        channel.close()
        server.stop(None)


@pytest.fixture
def driver() -> _RecordingDriver:
    return _RecordingDriver()


@pytest.fixture
def channel(driver: _RecordingDriver) -> Iterator[grpc.Channel]:
    yield from _serve(driver)


def _run(policy: EgoDriverPolicy, world: _World, ticks: int) -> None:
    for _ in range(ticks):
        policy.pre_tick(world)
        world.tick()
        policy.post_tick(world)


# ---------------------------------------------------------------------------
# Wire contract
# ---------------------------------------------------------------------------


class TestWireContract:
    def test_service_name_matches_upstream(self) -> None:
        descriptor = wire.DriveRequest.DESCRIPTOR.file.services_by_name[
            "EgodriverService"
        ]
        assert descriptor.full_name == wire.EGODRIVER_SERVICE_FULL_NAME

    def test_service_methods_match_upstream(self) -> None:
        descriptor = wire.DriveRequest.DESCRIPTOR.file.services_by_name[
            "EgodriverService"
        ]
        assert {m.name for m in descriptor.methods} == {
            "start_session",
            "close_session",
            "submit_image_observation",
            "submit_egomotion_observation",
            "submit_route",
            "submit_recording_ground_truth",
            "drive",
            "get_version",
        }

    def test_proto_packages_match_upstream(self) -> None:
        # The wire format depends on these names; the generated Python module
        # path is free to differ.
        assert wire.DriveRequest.DESCRIPTOR.full_name == "egodriver.DriveRequest"
        assert wire.Vec3.DESCRIPTOR.full_name == "common.Vec3"
        assert (
            wire.CarlaRendererData.DESCRIPTOR.full_name
            == "carla_driver.v0.CarlaRendererData"
        )

    def test_foreign_extension_payload_is_ignored(self) -> None:
        assert wire.unpack_renderer_data(b"") is None
        assert wire.unpack_debug_info(b"\xff\xff\xff") is None

    def test_api_mismatch_is_described(self) -> None:
        assert wire.describe_api_mismatch(wire.API_VERSION_MESSAGE) is None
        other = wire.VersionId.APIVersion(major=0, minor=1, patch=0)
        message = wire.describe_api_mismatch(other)
        assert message is not None and "0.1.0" in message


# ---------------------------------------------------------------------------
# Conversions and control
# ---------------------------------------------------------------------------


class TestConversions:
    def test_carla_y_is_mirrored(self) -> None:
        pose = conversions.carla_transform_to_pose((1.0, 2.0, 3.0), (0.0, 90.0, 0.0))
        np.testing.assert_allclose(pose.position, [1.0, -2.0, 3.0])
        # CARLA yaw +90 deg faces +y (right); mirrored, that is -90 deg.
        assert pose.yaw == pytest.approx(-math.pi / 2)

    def test_rig_origin_is_shifted_to_rear_axle(self) -> None:
        pose = conversions.rig_pose_from_actor_transform(
            (10.0, 0.0, 0.0), (0.0, 0.0, 0.0), -1.4
        )
        np.testing.assert_allclose(pose.position, [8.6, 0.0, 0.0])


class TestTrajectoryFollower:
    def _plan_towards(self, lateral: float) -> Trajectory:
        points = [np.array([x, lateral * x / 20.0, 0.0]) for x in np.arange(0, 21, 1.0)]
        return Trajectory(
            [int(i * 0.2e6) for i in range(len(points))],
            [Pose(p, np.array([0.0, 0.0, 0.0, 1.0])) for p in points],
        )

    def test_left_plan_steers_left_in_carla_terms(self) -> None:
        command = TrajectoryFollower().step(
            self._plan_towards(5.0), Pose.identity(), 5.0, 0.1
        )
        # The rig frame is positive to the left; CARLA steers positive right.
        assert command.steer < 0.0

    def test_stationary_plan_brakes(self) -> None:
        plan = Trajectory(
            [0, 1_000_000],
            [Pose.identity(), Pose.identity()],
        )
        command = TrajectoryFollower().step(plan, Pose.identity(), 3.0, 0.1)
        assert command.throttle == 0.0 and command.brake > 0.0


class TestConfig:
    def test_policy_step_must_be_multiple_of_tick(self) -> None:
        config = EgoDriverPolicyConfig(policy_timestep_s=0.1)
        assert config.ticks_per_policy_step(0.05) == 2
        with pytest.raises(ValueError):
            config.ticks_per_policy_step(0.03)
        with pytest.raises(ValueError, match="synchronous"):
            config.ticks_per_policy_step(0.0)

    def test_rejects_unsupported_image_format(self) -> None:
        with pytest.raises(ValueError):
            EgoDriverPolicyConfig(image_format=wire.ImageFormat.Value("AV1"))

    def test_rejects_duplicate_camera_ids(self) -> None:
        with pytest.raises(ValueError, match="unique"):
            EgoDriverPolicyConfig(
                cameras=(CameraConfig(logical_id="a"), CameraConfig(logical_id="a"))
            )


# ---------------------------------------------------------------------------
# Closed loop over gRPC
# ---------------------------------------------------------------------------


class TestClosedLoop:
    def _policy(self, channel: grpc.Channel, **overrides: Any) -> EgoDriverPolicy:
        config = EgoDriverPolicyConfig(rear_axle_offset_m=-1.4, **overrides)
        return EgoDriverPolicy(config, channel=channel)

    def test_session_is_opened_and_closed(
        self, fake_carla: Any, driver: _RecordingDriver, channel: grpc.Channel
    ) -> None:
        world = _World(_Ego())
        policy = self._policy(channel)
        policy.start(world, world.ego, scene_id="FakeTown:Test", random_seed=7)
        policy.close()

        assert len(driver.sessions) == 1
        session = driver.sessions[0]
        assert session.debug_info.scene_id == "FakeTown:Test"
        assert session.random_seed == 7
        assert driver.closed == [session.session_uuid]
        assert not policy.started

    def test_queries_driver_once_per_policy_step(
        self, fake_carla: Any, driver: _RecordingDriver, channel: grpc.Channel
    ) -> None:
        world = _World(_Ego())
        policy = self._policy(channel)
        policy.start(world, world.ego, scene_id="s")
        _run(policy, world, 40)
        policy.close()

        # 0.1 s policy step over 0.05 s ticks: every other tick.
        assert len(driver.drives) == 20
        for request in driver.drives:
            assert request.time_query_us - request.time_now_us == 100_000

    def test_egomotion_covers_every_tick_in_order(
        self, fake_carla: Any, driver: _RecordingDriver, channel: grpc.Channel
    ) -> None:
        world = _World(_Ego())
        policy = self._policy(channel)
        policy.start(world, world.ego, scene_id="s")
        _run(policy, world, 20)
        policy.close()

        timestamps = [
            p.timestamp_us for msg in driver.egomotion for p in msg.trajectory.poses
        ]
        assert timestamps == sorted(set(timestamps))
        # The start snapshot plus every tick before the last policy step.
        assert timestamps == [i * 50_000 for i in range(19)]
        assert all(
            len(msg.dynamic_states) == len(msg.trajectory.poses)
            for msg in driver.egomotion
        )

    def test_route_is_sent_ahead_in_rig_frame(
        self, fake_carla: Any, driver: _RecordingDriver, channel: grpc.Channel
    ) -> None:
        world = _World(_Ego())
        policy = self._policy(channel)
        policy.start(world, world.ego, scene_id="s")
        _run(policy, world, 2)
        policy.close()

        waypoints = driver.routes[0].route.waypoints
        assert waypoints[0].x == pytest.approx(1.4)  # rear axle is 1.4 m back
        assert waypoints[-1].x == pytest.approx(1.4 + 80.0)
        assert all(abs(w.y) < 1e-9 for w in waypoints)

    def test_renderer_data_reaches_the_driver(
        self, fake_carla: Any, driver: _RecordingDriver, channel: grpc.Channel
    ) -> None:
        world = _World(_Ego())
        policy = self._policy(channel)
        policy.start(world, world.ego, scene_id="s")
        _run(policy, world, 2)
        policy.close()

        data = wire.unpack_renderer_data(driver.drives[0].renderer_data)
        assert data is not None
        assert data.map_name == "Carla/Maps/FakeTown"
        assert data.speed_limit_mps == pytest.approx(10.0)
        assert data.ego_traffic_light == wire.TrafficLightState.Value(
            "TRAFFIC_LIGHT_STATE_NONE"
        )

    def test_renderer_data_can_be_switched_off(
        self, fake_carla: Any, driver: _RecordingDriver, channel: grpc.Channel
    ) -> None:
        world = _World(_Ego())
        policy = self._policy(channel, send_renderer_data=False)
        policy.start(world, world.ego, scene_id="s")
        _run(policy, world, 2)
        policy.close()

        assert driver.drives[0].renderer_data == b""

    def test_ego_follows_the_plan_back_onto_the_lane(
        self, fake_carla: Any, driver: _RecordingDriver, channel: grpc.Channel
    ) -> None:
        # The lane runs along CARLA y = 2 (to the right of the ego), and the
        # ego starts 20 degrees off it. Following the driver's plan must bring
        # it onto the lane, moving forward, with the handedness right.
        world = _World(_Ego(y=0.0, yaw_deg=-20.0), lane_y=2.0)
        policy = self._policy(channel)
        policy.start(world, world.ego, scene_id="s")
        _run(policy, world, 200)
        policy.close()

        location = world.ego.transform.location
        assert location.x > 30.0
        assert location.y == pytest.approx(2.0, abs=0.5)
        assert abs(world.ego.transform.rotation.yaw) < 5.0
        assert world.ego.speed == pytest.approx(_RecordingDriver.SPEED_MPS, abs=1.5)

    def test_driver_termination_brakes_and_stops_querying(
        self, fake_carla: Any
    ) -> None:
        driver = _RecordingDriver(terminate_after=3)
        for channel in _serve(driver):
            world = _World(_Ego())
            policy = self._policy(channel)
            policy.start(world, world.ego, scene_id="s")
            _run(policy, world, 20)
            policy.close()

        assert policy.terminated_by_driver
        assert policy.finished
        assert len(driver.drives) == 4
        assert world.ego.controls[-1].brake == 1.0


# ---------------------------------------------------------------------------
# Entity
# ---------------------------------------------------------------------------


class TestEgoDriverEntity:
    def test_autopilot_is_disabled(self) -> None:
        assert EgoDriverEntity.use_autopilot is False

    def test_factory_builds_a_fresh_entity_each_time(self) -> None:
        config = EgoDriverPolicyConfig(driver_address="example:1234")
        factory = EgoDriverEntity.factory(config)
        first, second = factory(), factory()
        assert isinstance(first, EgoDriverEntity)
        assert first is not second
        assert first.policy.config is config

    def test_tick_loop_start_needs_a_spawned_ego(self) -> None:
        entity = EgoDriverEntity()
        with pytest.raises(RuntimeError, match="spawned"):
            entity.on_tick_loop_start(_World(_Ego()), types.SimpleNamespace())  # type: ignore[arg-type]

    def test_plain_ego_hooks_are_no_ops(self) -> None:
        ego = EgoVehicle()
        world = _World(_Ego())
        ego.pre_tick(world)  # type: ignore[arg-type]
        ego.post_tick(world)  # type: ignore[arg-type]
        assert world.frame == 0
