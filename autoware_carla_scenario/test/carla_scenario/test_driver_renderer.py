"""Unit tests for the CARLA ground-truth payload sent to a driver policy.

The payload is what carries traffic lights and other vehicles, neither of which
the alpasim contract has a field for. A policy reads it defensively -- a missing
one means "no light, no traffic" rather than an error -- so a mistake here does
not fail loudly. These tests pin the behaviour that would otherwise rot quietly.
"""

from __future__ import annotations

import math
from types import SimpleNamespace
from typing import List, Optional
from unittest.mock import MagicMock

import pytest

from autoware_carla_scenario.driver._proto import carla_driver_pb2
from autoware_carla_scenario.driver.base import DriverClientConfig
from autoware_carla_scenario.driver.geometry import Pose
from autoware_carla_scenario.driver.renderer import RendererDataBuilder


# ---------------------------------------------------------------------------
# Fakes
# ---------------------------------------------------------------------------


def _waypoint(
    x: float,
    *,
    road_id: int = 1,
    lane_id: int = -1,
    is_junction: bool = False,
    junction_at: Optional[float] = None,
) -> MagicMock:
    """A waypoint on a straight road along +x (CARLA coordinates).

    *junction_at* makes every waypoint at or beyond that x a junction waypoint,
    which is how the lane walk decides where to stop.
    """
    waypoint = MagicMock()
    waypoint.road_id = road_id
    waypoint.lane_id = lane_id
    waypoint.is_junction = is_junction or (junction_at is not None and x >= junction_at)
    waypoint.transform = SimpleNamespace(
        location=SimpleNamespace(x=x, y=0.0, z=0.0),
        rotation=SimpleNamespace(roll=0.0, pitch=0.0, yaw=0.0),
    )
    waypoint.next.side_effect = lambda step: [
        _waypoint(x + step, road_id=road_id, lane_id=lane_id, junction_at=junction_at)
    ]
    return waypoint


def _light(state: str = "Red", stop_x: float = 30.0, light_id: int = 7) -> MagicMock:
    """A traffic light whose stop waypoint sits at *stop_x*, junction just past."""
    light = MagicMock()
    light.id = light_id
    light.get_state.return_value = state
    light.type_id = "traffic.traffic_light"
    light.get_stop_waypoints.return_value = [
        _waypoint(stop_x, junction_at=stop_x + 5.0)
    ]
    light.get_transform.return_value = SimpleNamespace(
        location=SimpleNamespace(x=stop_x, y=0.0, z=0.0),
        rotation=SimpleNamespace(roll=0.0, pitch=0.0, yaw=0.0),
    )
    return light


def _vehicle(
    actor_id: int,
    x: float,
    y: float = 0.0,
    *,
    vx: float = 0.0,
    vy: float = 0.0,
    extent: tuple = (2.4, 1.0, 0.75),
) -> MagicMock:
    actor = MagicMock()
    actor.id = actor_id
    actor.type_id = "vehicle.tesla.model3"
    actor.get_transform.return_value = SimpleNamespace(
        location=SimpleNamespace(x=x, y=y, z=0.0),
        rotation=SimpleNamespace(roll=0.0, pitch=0.0, yaw=0.0),
    )
    actor.get_velocity.return_value = SimpleNamespace(x=vx, y=vy, z=0.0)
    actor.bounding_box.extent = SimpleNamespace(x=extent[0], y=extent[1], z=extent[2])
    return actor


class _Actors(list):
    """Stands in for CARLA's ActorList, which filters by blueprint pattern."""

    def filter(self, pattern: str) -> List[MagicMock]:
        needle = pattern.strip("*")
        return [a for a in self if needle in a.type_id]


def _world(
    *,
    actors: Optional[List[MagicMock]] = None,
    ego_x: float = 0.0,
    junction_at: Optional[float] = None,
    ego_in_junction: bool = False,
) -> MagicMock:
    world = MagicMock()
    world.get_actors.return_value = _Actors(actors or [])
    world.get_map.return_value.name = "Town10HD_Opt"
    world.get_map.return_value.get_waypoint.return_value = _waypoint(
        ego_x, is_junction=ego_in_junction, junction_at=junction_at
    )
    world.get_snapshot.return_value.frame = 4242
    world.get_weather.return_value = SimpleNamespace(
        cloudiness=10.0,
        precipitation=0.0,
        precipitation_deposits=0.0,
        wind_intensity=5.0,
        sun_azimuth_angle=90.0,
        sun_altitude_angle=45.0,
        fog_density=2.0,
        wetness=0.0,
    )
    return world


def _ego(actor_id: int = 1, speed_limit_kmh: float = 30.0) -> MagicMock:
    ego = _vehicle(actor_id, 0.0)
    ego.get_speed_limit.return_value = speed_limit_kmh
    ego.is_at_traffic_light.return_value = False
    ego.get_location.return_value = SimpleNamespace(x=0.0, y=0.0, z=0.0)
    return ego


def _build(world: MagicMock, ego: MagicMock, ego_pose=None, **overrides):
    """Build and parse the payload for an ego at the origin facing +x."""
    config = DriverClientConfig(**overrides)
    builder = RendererDataBuilder(world, ego, config)
    pose = ego_pose if ego_pose is not None else Pose.identity()
    blob = builder.build(1_000, pose)
    message = carla_driver_pb2.CarlaRendererData()
    message.ParseFromString(blob)
    return message


# ---------------------------------------------------------------------------
# Scalars
# ---------------------------------------------------------------------------


def test_snapshot_scalars_are_reported() -> None:
    message = _build(_world(), _ego())
    assert message.snapshot_timestamp_us == 1_000
    assert message.frame_id == 4242
    assert message.map_name == "Town10HD_Opt"


def test_speed_limit_is_converted_to_mps() -> None:
    message = _build(_world(), _ego(speed_limit_kmh=30.0))
    assert message.speed_limit_mps == pytest.approx(30.0 / 3.6, rel=1e-5)


def test_missing_speed_limit_reports_zero() -> None:
    ego = _ego()
    ego.get_speed_limit.return_value = None
    assert _build(_world(), ego).speed_limit_mps == pytest.approx(0.0)


def test_weather_is_copied_field_by_field() -> None:
    message = _build(_world(), _ego())
    assert message.weather.cloudiness == pytest.approx(10.0)
    assert message.weather.sun_altitude_angle == pytest.approx(45.0)


def test_weather_fields_absent_in_this_carla_read_as_zero() -> None:
    """CARLA 0.10 dropped some of 0.9's weather fields."""
    world = _world()
    world.get_weather.return_value = SimpleNamespace(cloudiness=20.0)
    message = _build(world, _ego())
    assert message.weather.cloudiness == pytest.approx(20.0)
    assert message.weather.wetness == pytest.approx(0.0)


# ---------------------------------------------------------------------------
# Traffic lights
# ---------------------------------------------------------------------------


def test_light_ahead_on_our_lane_governs_us() -> None:
    """The lane graph is walked; the light need not be in its trigger volume."""
    light = _light("Red", stop_x=30.0)
    message = _build(_world(actors=[light], junction_at=35.0), _ego())

    assert message.ego_traffic_light == carla_driver_pb2.TRAFFIC_LIGHT_STATE_RED
    # The reported point is the junction mouth, past the stop waypoint.
    assert message.ego_traffic_light_distance_m > 30.0
    assert message.ego_traffic_light_distance_m < 40.0


def test_light_state_mapping() -> None:
    for name, expected in [
        ("Red", carla_driver_pb2.TRAFFIC_LIGHT_STATE_RED),
        ("Yellow", carla_driver_pb2.TRAFFIC_LIGHT_STATE_YELLOW),
        ("Green", carla_driver_pb2.TRAFFIC_LIGHT_STATE_GREEN),
        ("Off", carla_driver_pb2.TRAFFIC_LIGHT_STATE_OFF),
    ]:
        message = _build(_world(actors=[_light(name)], junction_at=35.0), _ego())
        assert message.ego_traffic_light == expected


def test_unknown_light_state_is_reported_as_unknown() -> None:
    message = _build(_world(actors=[_light("Sideways")], junction_at=35.0), _ego())
    assert message.ego_traffic_light == carla_driver_pb2.TRAFFIC_LIGHT_STATE_UNKNOWN


def test_no_light_reports_none_and_a_negative_distance() -> None:
    """The proto documents the distance as negative when no line applies."""
    message = _build(_world(), _ego())
    assert message.ego_traffic_light == carla_driver_pb2.TRAFFIC_LIGHT_STATE_NONE
    assert message.ego_traffic_light_distance_m < 0.0


def test_a_light_already_behind_reports_a_negative_distance() -> None:
    """Crossing the line must flip the sign, not restart a growing distance.

    A policy that stopped inside the trigger volume would otherwise be told
    forever that there is a line ahead of it.
    """
    light = _light("Red", stop_x=30.0)
    # Ego is 50 m along, so the line (and its junction mouth) is behind it.
    ego_pose = Pose.from_xyz_yaw(50.0, 0.0, 0.0, 0.0)
    message = _build(
        _world(actors=[light], ego_x=50.0, junction_at=35.0), _ego(), ego_pose
    )
    assert message.ego_traffic_light_distance_m < 0.0


def test_nothing_governs_us_inside_a_junction() -> None:
    """Having crossed the line, the thing to do is clear the box."""
    message = _build(_world(actors=[_light("Red")], ego_in_junction=True), _ego())
    assert message.ego_traffic_light == carla_driver_pb2.TRAFFIC_LIGHT_STATE_NONE


def test_zero_sight_distance_falls_back_to_trigger_volumes() -> None:
    light = _light("Red")
    ego = _ego()
    ego.is_at_traffic_light.return_value = True
    ego.get_traffic_light.return_value = light

    message = _build(
        _world(actors=[light], junction_at=35.0),
        ego,
        traffic_light_sight_distance_m=0.0,
    )
    assert message.ego_traffic_light == carla_driver_pb2.TRAFFIC_LIGHT_STATE_RED


def test_light_on_another_lane_does_not_govern_us() -> None:
    light = _light("Red")
    light.get_stop_waypoints.return_value = [_waypoint(30.0, lane_id=+1)]
    message = _build(_world(actors=[light], junction_at=35.0), _ego())
    assert message.ego_traffic_light == carla_driver_pb2.TRAFFIC_LIGHT_STATE_NONE


def test_a_light_without_stop_waypoints_uses_its_own_position() -> None:
    light = _light("Red")
    light.get_stop_waypoints.return_value = []
    ego = _ego()
    ego.is_at_traffic_light.return_value = True
    ego.get_traffic_light.return_value = light

    message = _build(_world(actors=[light]), ego, traffic_light_sight_distance_m=0.0)
    assert message.ego_traffic_light == carla_driver_pb2.TRAFFIC_LIGHT_STATE_RED
    assert message.ego_traffic_light_distance_m == pytest.approx(30.0, abs=1e-3)


# ---------------------------------------------------------------------------
# Other vehicles
# ---------------------------------------------------------------------------


def test_other_vehicles_are_reported_in_the_local_frame() -> None:
    """CARLA's y is South; the local frame's y is North."""
    other = _vehicle(2, x=20.0, y=4.0, vx=6.0)
    message = _build(_world(actors=[other]), _ego())

    assert len(message.actors) == 1
    actor = message.actors[0]
    assert actor.track_id == "2"
    assert actor.type_id == "vehicle.tesla.model3"
    assert actor.pose_local_to_aabb.vec.x == pytest.approx(20.0)
    assert actor.pose_local_to_aabb.vec.y == pytest.approx(-4.0)
    assert actor.dynamic_state.linear_velocity.x == pytest.approx(6.0)


def test_actor_velocity_flips_with_the_handedness() -> None:
    other = _vehicle(2, x=10.0, vy=3.0)
    message = _build(_world(actors=[other]), _ego())
    assert message.actors[0].dynamic_state.linear_velocity.y == pytest.approx(-3.0)


def test_aabb_is_the_full_size_not_the_half_extent() -> None:
    other = _vehicle(2, x=10.0, extent=(2.4, 1.0, 0.75))
    message = _build(_world(actors=[other]), _ego())
    aabb = message.actors[0].aabb
    assert aabb.size_x == pytest.approx(4.8)
    assert aabb.size_y == pytest.approx(2.0)
    assert aabb.size_z == pytest.approx(1.5)


def test_the_ego_is_not_reported_as_traffic() -> None:
    ego = _ego()
    message = _build(_world(actors=[ego, _vehicle(2, x=10.0)]), ego)
    assert [a.track_id for a in message.actors] == ["2"]


def test_vehicles_beyond_the_horizon_are_dropped() -> None:
    message = _build(
        _world(actors=[_vehicle(2, x=40.0), _vehicle(3, x=400.0)]),
        _ego(),
        actor_horizon_m=150.0,
    )
    assert [a.track_id for a in message.actors] == ["2"]


def test_actor_ground_truth_can_be_switched_off() -> None:
    message = _build(
        _world(actors=[_vehicle(2, x=10.0)]),
        _ego(),
        send_actor_ground_truth=False,
    )
    assert list(message.actors) == []
    # The light and speed limit still come through.
    assert message.map_name == "Town10HD_Opt"


def test_traffic_lights_are_not_reported_as_vehicles() -> None:
    message = _build(_world(actors=[_light("Red"), _vehicle(2, x=10.0)]), _ego())
    assert [a.track_id for a in message.actors] == ["2"]


# ---------------------------------------------------------------------------
# Robustness
# ---------------------------------------------------------------------------


def test_a_carla_error_yields_an_empty_payload_rather_than_raising() -> None:
    """Ground truth is best-effort; losing it must not abort a scenario."""
    world = _world()
    world.get_weather.side_effect = RuntimeError("connection lost")
    builder = RendererDataBuilder(world, _ego(), DriverClientConfig())
    assert builder.build(0, Pose.identity()) == b""


def test_stop_line_geometry_is_cached() -> None:
    """Lights do not move, and the lane walk behind them is not free.

    Two caches read the stop waypoints -- the lane index and the stop points --
    so what matters is that neither refills, not the exact count.
    """
    light = _light("Red")
    world = _world(actors=[light], junction_at=35.0)
    builder = RendererDataBuilder(world, _ego(), DriverClientConfig())

    builder.build(0, Pose.identity())
    after_first = light.get_stop_waypoints.call_count

    for _ in range(4):
        builder.build(0, Pose.identity())

    assert light.get_stop_waypoints.call_count == after_first


def test_the_payload_round_trips_through_the_wire() -> None:
    world = _world(actors=[_light("Red"), _vehicle(2, x=12.0, vx=3.0)])
    builder = RendererDataBuilder(world, _ego(), DriverClientConfig())
    blob = builder.build(500, Pose.identity())

    restored = carla_driver_pb2.CarlaRendererData.FromString(blob)
    assert restored.snapshot_timestamp_us == 500
    assert len(restored.actors) == 1


def test_yawed_ego_measures_the_light_along_its_heading() -> None:
    """Distance is longitudinal in the rig frame, not a straight line."""
    light = _light("Red", stop_x=30.0)
    world = _world(actors=[light], junction_at=35.0)
    # Ego sits 10 m to the CARLA-south of the road, still facing +x. The stop
    # point is then off to one side, so the along-heading distance is shorter
    # than the straight-line distance.
    ego_pose = Pose.from_xyz_yaw(0.0, 10.0, 0.0, 0.0)
    message = _build(world, _ego(), ego_pose)

    straight = math.hypot(35.0, 10.0)
    assert message.ego_traffic_light_distance_m < straight
