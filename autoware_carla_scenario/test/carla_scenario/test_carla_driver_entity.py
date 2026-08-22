"""Unit tests for the ego entity driven by an external policy.

The CARLA world and the ego actor are mocked, and the driver client is replaced by a
recording fake, so the whole driver loop is exercised without a simulator or a policy
server.
"""

from __future__ import annotations

import math
from types import SimpleNamespace
from typing import List, Optional, Tuple
from unittest.mock import MagicMock

import numpy as np
import pytest

from autoware_carla_scenario.driver.base import (
    BaseEgoDriverClient,
    DriveOutcome,
    DriverCameraConfig,
    DriverClientConfig,
    EgoObservation,
)
from autoware_carla_scenario.driver._proto import carla_driver_pb2
from autoware_carla_scenario.driver.geometry import Pose, Trajectory
from autoware_carla_scenario.entity.carla_driver_entity import CarlaDriverEntity


_TICK_S = 0.05


class _FakeDriverClient(BaseEgoDriverClient):
    """Records every call and answers ``drive`` with a straight-ahead plan."""

    def __init__(
        self, config: DriverClientConfig, *, terminate_after: Optional[int] = None
    ) -> None:
        super().__init__(config)
        self.started: List[Tuple[str, str]] = []
        self.routes: List[np.ndarray] = []
        self.images: List[Tuple[str, bytes]] = []
        self.observations: List[EgoObservation] = []
        self.drives: List[Tuple[int, int]] = []
        self.renderer_payloads: List[bytes] = []
        self.closed = 0
        self._terminate_after = terminate_after

    def start_session(self, session_uuid: str, scene_id: str) -> None:
        self.started.append((session_uuid, scene_id))

    def submit_route(self, timestamp_us: int, waypoints_in_rig) -> None:
        self.routes.append(np.asarray(waypoints_in_rig))

    def submit_image_observation(
        self,
        logical_id: str,
        frame_start_us: int,
        frame_end_us: int,
        image_bytes: bytes,
    ) -> None:
        self.images.append((logical_id, image_bytes))

    def submit_egomotion_observation(self, observation: EgoObservation) -> None:
        self.observations.append(observation)

    def drive(
        self,
        time_now_us: int,
        time_query_us: int,
        renderer_data: bytes = b"",
    ) -> DriveOutcome:
        self.drives.append((time_now_us, time_query_us))
        self.renderer_payloads.append(renderer_data)
        plan = Trajectory.empty()
        for index in range(1, 5):
            plan.append(
                time_now_us + index * 100_000,
                Pose.from_xyz_yaw(index * 0.8, 0.0, 0.0, 0.0),
            )
        terminate = (
            self._terminate_after is not None
            and len(self.drives) >= self._terminate_after
        )
        return DriveOutcome(trajectory=plan, terminate_session=terminate)

    def close_session(self) -> None:
        self.closed += 1


class _EmptyActors(list):
    """Stands in for CARLA's ActorList when the world holds only the ego."""

    def filter(self, pattern: str) -> List[MagicMock]:
        return []


def _actor(x: float = 0.0, y: float = 0.0, yaw_deg: float = 0.0) -> MagicMock:
    """Return a mock CARLA vehicle actor with usable kinematics."""
    actor = MagicMock()
    actor.get_transform.return_value = SimpleNamespace(
        location=SimpleNamespace(x=x, y=y, z=0.0),
        rotation=SimpleNamespace(roll=0.0, pitch=0.0, yaw=yaw_deg),
    )
    actor.get_location.return_value = SimpleNamespace(x=x, y=y, z=0.0)
    actor.get_velocity.return_value = SimpleNamespace(x=0.0, y=0.0, z=0.0)
    actor.get_acceleration.return_value = SimpleNamespace(x=0.0, y=0.0, z=0.0)
    actor.get_angular_velocity.return_value = SimpleNamespace(x=0.0, y=0.0, z=0.0)
    return actor


def _world_with_route() -> MagicMock:
    """Return a mock world whose road graph walks straight along +x."""
    world = MagicMock()

    def _waypoint_at(distance: float) -> MagicMock:
        waypoint = MagicMock()
        waypoint.transform.location = SimpleNamespace(x=distance, y=0.0, z=0.0)
        waypoint.next.side_effect = lambda step: [_waypoint_at(distance + step)]
        return waypoint

    world.get_map.return_value.get_waypoint.return_value = _waypoint_at(0.0)
    world.get_map.return_value.name = "Town10HD_Opt"
    return world


def _entity(
    *,
    cameras: Tuple[DriverCameraConfig, ...] = (),
    terminate_after: Optional[int] = None,
    **config_overrides,
) -> Tuple[CarlaDriverEntity, _FakeDriverClient]:
    """Return an entity wired to a fake client, with the ego actor already 'spawned'."""
    config_overrides.setdefault("send_renderer_data", False)
    config = DriverClientConfig(
        cameras=cameras, rear_axle_offset_m=-1.4, **config_overrides
    )
    client = _FakeDriverClient(config, terminate_after=terminate_after)
    entity = CarlaDriverEntity(config, client=client)
    entity._vehicle = _actor()  # noqa: SLF001 - stands in for spawn()
    return entity, client


# ---------------------------------------------------------------------------
# Policy flags
# ---------------------------------------------------------------------------


def test_traffic_manager_is_disabled() -> None:
    """ScenarioRunner reads this flag to skip set_autopilot on the ego."""
    assert CarlaDriverEntity.use_autopilot is False


# ---------------------------------------------------------------------------
# Session start
# ---------------------------------------------------------------------------


def test_the_map_is_fetched_once() -> None:
    """CARLA rebuilds the map on every get_map() call; the walk must not pay that."""
    entity, _ = _entity(policy_timestep_s=0.05)
    world = _world_with_route()
    entity.on_scenario_start(world)
    for tick in range(4):
        entity.on_tick(world, tick * _TICK_S)

    assert world.get_map.call_count == 1


def test_the_route_rolls_forward_with_the_ego() -> None:
    """Re-sending the route each policy step keeps the horizon ahead of the vehicle."""
    entity, client = _entity(policy_timestep_s=0.05)
    world = _world_with_route()
    entity.on_scenario_start(world)

    for tick in range(3):
        entity.on_tick(world, tick * _TICK_S)

    # One at session start plus one per policy step.
    assert len(client.routes) == 4


def test_start_opens_a_session_and_sends_a_route() -> None:
    entity, client = _entity()
    world = _world_with_route()

    entity.on_scenario_start(world)

    assert len(client.started) == 1
    session_uuid, scene_id = client.started[0]
    assert session_uuid
    assert scene_id == "Town10HD_Opt"

    assert len(client.routes) == 1
    route = client.routes[0]
    assert route.shape[1] == 3
    # Waypoints are in the rig frame, so they start just ahead of the rear-axle origin.
    assert route[0][0] == pytest.approx(1.4, abs=1e-6)
    assert np.allclose(route[:, 1], 0.0, atol=1e-6)


def test_ground_truth_reaches_the_policy() -> None:
    """The payload carries traffic lights and other vehicles; the contract does not."""
    entity, client = _entity(send_renderer_data=True, policy_timestep_s=0.05)
    world = _world_with_route()
    world.get_actors.return_value = _EmptyActors()
    world.get_snapshot.return_value.frame = 1
    world.get_weather.return_value = SimpleNamespace(cloudiness=0.0)

    entity.on_scenario_start(world)
    entity.on_tick(world, 0.0)

    assert client.renderer_payloads
    payload = carla_driver_pb2.CarlaRendererData()
    payload.ParseFromString(client.renderer_payloads[0])
    assert payload.snapshot_timestamp_us == 50_000
    assert payload.map_name == "Town10HD_Opt"


def test_ground_truth_can_be_switched_off() -> None:
    """Off is indistinguishable from 'nothing to report' on the policy side."""
    entity, client = _entity(send_renderer_data=False, policy_timestep_s=0.05)
    world = _world_with_route()
    entity.on_scenario_start(world)
    entity.on_tick(world, 0.0)

    assert client.renderer_payloads == [b""]


def test_start_before_spawn_raises() -> None:
    config = DriverClientConfig(cameras=())
    entity = CarlaDriverEntity(config, client=_FakeDriverClient(config))
    with pytest.raises(RuntimeError, match="before spawn"):
        entity.on_scenario_start(_world_with_route())


def test_cameras_are_attached_and_streamed(monkeypatch: pytest.MonkeyPatch) -> None:
    from autoware_carla_scenario.sensor.carla_camera import CarlaCameraSensor

    frame = np.zeros((8, 8, 3), dtype=np.uint8)
    monkeypatch.setattr(CarlaCameraSensor, "get_image", lambda self: frame)

    entity, client = _entity(cameras=(DriverCameraConfig(logical_id="front"),))
    world = _world_with_route()

    entity.on_scenario_start(world)
    assert world.spawn_actor.call_count == 1

    entity.on_tick(world, 0.0)
    assert [logical_id for logical_id, _ in client.images] == ["front"]
    # The frame is JPEG encoded on the way out.
    assert client.images[0][1].startswith(b"\xff\xd8")


def test_a_camera_with_no_frame_is_skipped(monkeypatch: pytest.MonkeyPatch) -> None:
    from autoware_carla_scenario.sensor.carla_camera import CarlaCameraSensor

    monkeypatch.setattr(CarlaCameraSensor, "get_image", lambda self: None)

    entity, client = _entity(cameras=(DriverCameraConfig(logical_id="front"),))
    world = _world_with_route()
    entity.on_scenario_start(world)
    entity.on_tick(world, 0.0)

    assert client.images == []
    # The policy is still asked to plan.
    assert len(client.drives) == 1


# ---------------------------------------------------------------------------
# Tick loop
# ---------------------------------------------------------------------------


def test_control_is_applied_every_tick() -> None:
    entity, _ = _entity()
    world = _world_with_route()
    entity.on_scenario_start(world)

    actor = entity.actor
    assert actor is not None
    for tick in range(6):
        entity.on_tick(world, tick * _TICK_S)

    assert actor.apply_control.call_count == 6


def test_the_policy_is_queried_at_the_configured_cadence() -> None:
    """0.1 s of policy timestep over a 0.05 s tick means every other tick."""
    entity, client = _entity(policy_timestep_s=0.1)
    world = _world_with_route()
    entity.on_scenario_start(world)

    for tick in range(6):
        entity.on_tick(world, tick * _TICK_S)

    assert len(client.drives) == 3
    assert [now for now, _ in client.drives] == [50_000, 150_000, 250_000]
    # Each request tells the policy where the runtime is heading next.
    assert [query for _, query in client.drives] == [150_000, 250_000, 350_000]


def test_every_tick_queries_when_the_timestep_matches_the_tick() -> None:
    entity, client = _entity(policy_timestep_s=0.05)
    world = _world_with_route()
    entity.on_scenario_start(world)

    for tick in range(4):
        entity.on_tick(world, tick * _TICK_S)

    assert len(client.drives) == 4


def test_observations_accompany_every_policy_step() -> None:
    entity, client = _entity(policy_timestep_s=0.1)
    world = _world_with_route()
    entity.on_scenario_start(world)

    for tick in range(4):
        entity.on_tick(world, tick * _TICK_S)

    assert len(client.observations) == len(client.drives) == 2
    assert client.observations[0].timestamp_us == 50_000


def test_ticks_before_the_session_opens_do_nothing() -> None:
    entity, client = _entity()
    world = _world_with_route()
    actor = entity.actor
    assert actor is not None

    entity.on_tick(world, 0.0)

    assert client.drives == []
    assert actor.apply_control.call_count == 0


# ---------------------------------------------------------------------------
# Termination
# ---------------------------------------------------------------------------


def test_policy_termination_is_reported() -> None:
    entity, _ = _entity(terminate_after=2, policy_timestep_s=0.05)
    world = _world_with_route()
    entity.on_scenario_start(world)

    entity.on_tick(world, 0.0)
    assert entity.termination_requested is False

    entity.on_tick(world, _TICK_S)
    assert entity.termination_requested is True


def test_a_plain_ego_never_requests_termination() -> None:
    from autoware_carla_scenario.entity.ego import EgoVehicle

    assert EgoVehicle().termination_requested is False


# ---------------------------------------------------------------------------
# Teardown
# ---------------------------------------------------------------------------


def test_teardown_closes_the_session_and_destroys_cameras(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from autoware_carla_scenario.sensor.carla_camera import CarlaCameraSensor

    destroyed: List[int] = []
    monkeypatch.setattr(CarlaCameraSensor, "get_image", lambda self: None)
    monkeypatch.setattr(
        CarlaCameraSensor, "destroy", lambda self: destroyed.append(id(self))
    )

    entity, client = _entity(cameras=(DriverCameraConfig(logical_id="front"),))
    world = _world_with_route()
    entity.on_scenario_start(world)
    entity.on_scenario_end(world)

    assert client.closed == 1
    assert len(destroyed) == 1


def test_teardown_is_safe_when_the_policy_is_gone() -> None:
    """A dead policy must not abort the rest of the scenario cleanup."""
    entity, client = _entity()
    world = _world_with_route()
    entity.on_scenario_start(world)

    def _boom() -> None:
        raise ConnectionError("policy vanished")

    client.close_session = _boom  # type: ignore[method-assign]

    entity.on_scenario_end(world)  # must not raise


def test_teardown_without_a_session_is_a_noop() -> None:
    entity, client = _entity()
    entity.on_scenario_end(_world_with_route())
    assert client.closed == 0


# ---------------------------------------------------------------------------
# Diagnostics
# ---------------------------------------------------------------------------


def test_debug_state_reports_loop_progress() -> None:
    entity, _ = _entity(policy_timestep_s=0.05)
    world = _world_with_route()
    entity.on_scenario_start(world)
    entity.on_tick(world, 0.0)

    state = entity.debug_state()
    assert state["drive_count"] == 1.0
    assert state["plan_length"] == 4.0
    assert state["sim_time_us"] == pytest.approx(50_000.0)
    assert state["rear_axle_offset_m"] == pytest.approx(-1.4)


def test_yawed_ego_reports_a_rig_frame_route() -> None:
    """The route is expressed in the rig frame, so a yawed ego still sees it ahead."""
    entity, client = _entity()
    entity._vehicle = _actor(x=5.0, y=-5.0, yaw_deg=-90.0)  # noqa: SLF001
    world = _world_with_route()

    entity.on_scenario_start(world)

    route = client.routes[0]
    # CARLA yaw -90 deg is +90 deg in the right-handed frame: the ego faces +y, while
    # the road runs along +x, so the road is off to the ego's right (negative y in rig).
    assert route[-1][1] < 0.0
    assert math.isfinite(float(route[-1][0]))
