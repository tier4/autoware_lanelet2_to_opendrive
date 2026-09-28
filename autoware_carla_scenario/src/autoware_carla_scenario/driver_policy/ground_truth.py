"""The CARLA ground truth a driver policy reads, gathered from a live world.

The alpasim contract has no field for a traffic light and none for the traffic
around the ego. Both ride inside ``DriveRequest.renderer_data`` as a
``carla_driver.v0.CarlaRendererData``, and everything that fills that message
lives here.

The reader needs a world, an ego and a map. It does not need the authority to
create any of them, which is what lets a scenario runner that owns its own
world use it.

Ported from ``carla_driver_interface.runtime.ground_truth``. The payload is what
a driver written against that package expects, field for field.
"""

from __future__ import annotations

from typing import Any

import numpy as np

from .config import EgoDriverPolicyConfig
from .conversions import (
    carla_transform_to_pose,
    carla_vector_to_local,
    waypoint_to_local,
)
from .geometry import Pose, dynamic_state_proto
from .wire import (
    AABB,
    CarlaActorState,
    CarlaRendererData,
    CarlaWeather,
    TrafficLightState,
)

__all__ = ["CarlaGroundTruth"]

#: How far past a stop waypoint to look for the junction it governs, and in
#: what steps. CARLA's stop waypoints sit a median of 5.5 m upstream of their
#: junction on Town10HD_Opt and at most 7.0 m, so eight is generous without
#: reaching the next junction along.
_STOP_LINE_SEARCH_M = 8.0
_STOP_LINE_STEP_M = 0.5

#: Blueprint patterns for the actors reported as ground truth.
#:
#: Pedestrians are matched by their own namespace rather than by ``*walker*``,
#: which would also catch the ``controller.ai.walker`` actors that steer them --
#: those are controllers with no body, and reporting one would put a
#: zero-extent obstacle wherever CARLA happens to keep it.
_ACTOR_PATTERNS = ("*vehicle*", "walker.pedestrian.*")


class CarlaGroundTruth:
    """Reads the CARLA ground truth for one ego.

    Traffic-light geometry is cached on first use: lights and junctions do not
    move, and the lane-graph walks behind them are not cheap enough to repeat at
    policy rate.

    Args:
        world: The CARLA world to read.
        ego: The ego vehicle actor.
        carla_map: The world's map, passed in rather than fetched because CARLA
            rebuilds the object on every ``get_map()`` call.
        config: Supplies the sight distance, the actor horizon and the lane-walk
            step.
        map_name: Reported to the policy as the scene it is driving.
    """

    def __init__(
        self,
        world: Any,
        ego: Any,
        carla_map: Any,
        config: EgoDriverPolicyConfig,
        map_name: str,
    ) -> None:
        self._world = world
        self._ego = ego
        self._map = carla_map
        self.config = config
        self._map_name = map_name
        #: Lane -> lights governing it, built on first use; lights do not move.
        self._stop_lines: dict[tuple[int, int], list[Any]] | None = None
        #: Light id -> where to stop for it, in the local frame. Same reason.
        self._stop_line_points_by_light: dict[int, list[np.ndarray]] = {}

    def read(
        self, timestamp_us: int, frame_id: int, pose_local_to_rig: Pose
    ) -> CarlaRendererData:
        light = self._governing_traffic_light()
        return CarlaRendererData(
            snapshot_timestamp_us=timestamp_us,
            frame_id=frame_id,
            map_name=self._map_name,
            weather=self._weather(),
            ego_traffic_light=self._traffic_light_state(light),
            ego_traffic_light_distance_m=self._traffic_light_distance(
                light, pose_local_to_rig
            ),
            speed_limit_mps=self._speed_limit_mps(),
            actors=(
                self._actor_states(pose_local_to_rig)
                if self.config.send_actor_ground_truth
                else []
            ),
        )

    def _governing_traffic_light(self) -> Any:
        """The light the ego must obey, found by looking down its own lane.

        CARLA's `is_at_traffic_light` answers a different question -- whether
        the ego is *inside the light's trigger volume* -- and those volumes are
        about a metre thick along the road. Across the fifteen lights of
        Town10HD_Opt they reach a median of 0.55 m back from the stop line, so
        a policy asking CARLA learns of a red light at the moment it arrives at
        it, when stopping from any ordinary speed is already impossible. What
        follows is an overrun that says nothing about the policy.

        So the lane graph is walked forward instead, up to
        ``traffic_light_sight_distance_m``, and any light whose stop line lies
        on one of those lanes governs us. That is the question a driver
        answers by looking.

        Inside a junction, nothing governs us at all. Having crossed the line
        the thing to do is clear the box, so no light in there is ours to read
        -- and the trigger volumes make that an active hazard rather than a
        nicety, since a volume reaches a couple of metres past its own line and
        a junction has four of them. A vehicle in the middle can be standing in
        the volume of a light governing traffic that crosses its path, and be
        told to stop where stopping is worst.

        Falls back to `is_at_traffic_light` when the sight distance is zero --
        which restores the previous behaviour exactly -- and, outside a
        junction, whenever the walk finds nothing: a light on a lane the graph
        does not reach still governs us once we are standing in its volume.
        """
        sight = self.config.traffic_light_sight_distance_m
        if sight > 0.0:
            waypoint = self._ego_waypoint()
            if waypoint is not None and waypoint.is_junction:
                return None
            for light in self._lights_by_lane_ahead(self._lanes_ahead(sight)):
                return light
        return (
            self._ego.get_traffic_light() if self._ego.is_at_traffic_light() else None
        )

    def _ego_waypoint(self) -> Any:
        """Where the ego sits on the lane graph, or ``None`` if nowhere."""
        return self._map.get_waypoint(
            self._ego.get_transform().location, project_to_road=True
        )

    def _lanes_ahead(self, distance_m: float) -> list[tuple[int, int]]:
        """``(road_id, lane_id)`` of the lanes up to the next junction.

        Walked rather than guessed, because a stop line sits on the lane it
        governs and the ego is often still on an earlier segment of road when
        it needs to know.

        The walk stops at the junction it reaches, and reports nothing at all
        once the ego is inside one, and both of those are the same rule: a
        light governs the vehicles waiting to enter its junction, not the ones
        already in it. Having crossed the line, the thing to do is clear the
        box -- which is also the law -- and a light beyond it belongs to a
        junction we have yet to arrive at.

        Left walking through, this reported the *next* junction's light from
        inside the current one, at 70-odd metres. Nothing needs braking for at
        that range, so the stop line was never the problem; the problem was
        that a policy gating "am I free to move" on whether a light applies
        then had a reason to stand still, and stood still in the middle of a
        junction for seventeen seconds. Walking through a junction is also
        what made the answer flicker: the lane the ego projects onto inside
        one is ambiguous, so consecutive steps took different branches and
        reported different lights, 76 m away one step and 3.9 m the next.
        """
        step = max(1.0, self.config.route_resolution_m)
        waypoint = self._ego_waypoint()
        if waypoint is None or waypoint.is_junction:
            return []
        lanes = [(waypoint.road_id, waypoint.lane_id)]
        travelled = 0.0
        while travelled < distance_m:
            options = waypoint.next(step)
            if not options:
                break
            waypoint = options[0]
            travelled += step
            lane = (waypoint.road_id, waypoint.lane_id)
            if lane != lanes[-1]:
                lanes.append(lane)
            # The stop line sits at the mouth of the junction, so this lane may
            # still carry it -- but nothing past it is ours to obey yet.
            if waypoint.is_junction:
                break
        return lanes

    def _lights_by_lane_ahead(self, lanes: list[tuple[int, int]]) -> list[Any]:
        """Lights whose stop lines lie on those lanes, nearest lane first.

        The lane list is in the order the ego will drive it, so the first hit
        is the first light it will meet.
        """
        if not lanes:
            return []
        stop_lines = self._stop_lines_by_lane()
        found: list[Any] = []
        for lane in lanes:
            found.extend(stop_lines.get(lane, ()))
        return found

    def _stop_lines_by_lane(self) -> dict[tuple[int, int], list[Any]]:
        """Which light governs which lane, built once -- lights do not move."""
        if self._stop_lines is None:
            index: dict[tuple[int, int], list[Any]] = {}
            for light in self._world.get_actors().filter("traffic.traffic_light*"):
                for waypoint in light.get_stop_waypoints():
                    index.setdefault((waypoint.road_id, waypoint.lane_id), []).append(
                        light
                    )
            self._stop_lines = index
        return self._stop_lines

    def _weather(self) -> CarlaWeather:
        weather = self._world.get_weather()

        # 0.10.x dropped some 0.9.x weather fields; read defensively.
        def value(name: str) -> float:
            return float(getattr(weather, name, 0.0))

        return CarlaWeather(
            cloudiness=value("cloudiness"),
            precipitation=value("precipitation"),
            precipitation_deposits=value("precipitation_deposits"),
            wind_intensity=value("wind_intensity"),
            sun_azimuth_angle=value("sun_azimuth_angle"),
            sun_altitude_angle=value("sun_altitude_angle"),
            fog_density=value("fog_density"),
            wetness=value("wetness"),
        )

    def _traffic_light_state(self, light: Any | None) -> TrafficLightState:
        if light is None:
            return TrafficLightState.TRAFFIC_LIGHT_STATE_NONE
        mapping = {
            "Red": TrafficLightState.TRAFFIC_LIGHT_STATE_RED,
            "Yellow": TrafficLightState.TRAFFIC_LIGHT_STATE_YELLOW,
            "Green": TrafficLightState.TRAFFIC_LIGHT_STATE_GREEN,
            "Off": TrafficLightState.TRAFFIC_LIGHT_STATE_OFF,
        }
        return mapping.get(
            str(light.get_state()), TrafficLightState.TRAFFIC_LIGHT_STATE_UNKNOWN
        )

    def _traffic_light_distance(
        self, light: Any | None, pose_local_to_rig: Pose
    ) -> float:
        """Distance the ego still has to travel before the stop line.

        Measured **along the ego's heading**, not as a straight-line distance,
        and negative once the line is behind.  The difference matters: a
        Euclidean distance starts growing again the moment the ego crosses the
        line, so a policy reading it cannot tell "1 m to go" from "1 m past",
        and ``is_at_traffic_light`` stays true throughout the trigger volume.

        A policy that stops inside that volume therefore never leaves it, and
        is told forever that there is a stop line ahead which it has in fact
        already crossed -- a deadlock, and one that only appears when the
        policy does the right thing and stops.

        Returning a negative value here matches what the field already means
        elsewhere: ``CarlaRendererData.ego_traffic_light_distance_m`` is
        documented as negative when no stop line applies to the ego, and once
        the line is behind, none does.
        """
        if light is None:
            return -1.0
        stop_points = self._stop_line_points(light)
        if not stop_points:
            location = light.get_transform().location
            stop_points = [carla_vector_to_local(location.x, location.y, location.z)]

        # Resolve into the rig frame, whose +x is straight ahead.
        to_rig = pose_local_to_rig.inverse()
        longitudinal = to_rig.transform_points(
            np.asarray(stop_points, dtype=np.float64)
        )[:, 0]

        # The nearest line still ahead governs us; if they are all behind, the
        # nearest of those does, so the value stays continuous as we cross.
        ahead = longitudinal[longitudinal >= 0.0]
        return float(ahead.min()) if len(ahead) else float(longitudinal.max())

    def _stop_line_points(self, light: Any) -> list[np.ndarray]:
        """Where a vehicle should stop for this light, in the local frame.

        Not `get_stop_waypoints()` itself, which sits further back than the
        line a driver aims at. Measured across all thirty stop waypoints of
        Town10HD_Opt, each is a median of 5.5 m upstream of the junction it
        governs, and up to 7.0 m. A policy told to stop there stops that far
        short of the junction mouth -- which, with the length of a car in
        front of the rear axle it measures from, is most of a car and a half
        of hesitation that belongs to nobody.

        So each stop waypoint is walked forward to the mouth of its junction,
        and that is the point reported. A waypoint with no junction ahead of
        it inside `_STOP_LINE_SEARCH_M` is reported where it is; there is
        nothing better to say about it.

        Cached, because lights and junctions do not move and this walks the
        lane graph a few metres at a time.
        """
        key = int(getattr(light, "id", 0))
        cached = self._stop_line_points_by_light.get(key)
        if cached is None:
            cached = [
                waypoint_to_local(self._junction_mouth(wp))
                for wp in light.get_stop_waypoints()
            ]
            self._stop_line_points_by_light[key] = cached
        return cached

    def _junction_mouth(self, stop: Any) -> Any:
        """The first waypoint of the junction the stop line governs.

        Falls back to the stop waypoint itself when the walk finds no
        junction -- a stop line on open road is unusual but not ours to
        second-guess.
        """
        waypoint, travelled = stop, 0.0
        while travelled < _STOP_LINE_SEARCH_M:
            options = waypoint.next(_STOP_LINE_STEP_M)
            if not options:
                return stop
            waypoint = options[0]
            travelled += _STOP_LINE_STEP_M
            if waypoint.is_junction:
                return waypoint
        return stop

    def _speed_limit_mps(self) -> float:
        return float(self._ego.get_speed_limit() or 0.0) / 3.6  # CARLA reports km/h

    def _reportable_actors(self) -> list[Any]:
        """Every actor a policy has to keep clear of.

        Vehicles and pedestrians alike: a specification that demands clearance
        from other road users means all of them, and a driver reading only
        vehicles satisfies "collision free" while walking through a crossing.
        The two namespaces are disjoint, so nothing is reported twice.
        """
        actors = self._world.get_actors()
        return [
            match for pattern in _ACTOR_PATTERNS for match in actors.filter(pattern)
        ]

    def _actor_states(self, pose_local_to_rig: Pose) -> list[CarlaActorState]:
        states = []
        ego_position = pose_local_to_rig.position
        for actor in self._reportable_actors():
            if actor.id == self._ego.id:
                continue
            transform = actor.get_transform()
            location, rotation = transform.location, transform.rotation
            pose = carla_transform_to_pose(
                (location.x, location.y, location.z),
                (rotation.pitch, rotation.yaw, rotation.roll),
            )
            if (
                float(np.linalg.norm(pose.position - ego_position))
                > self.config.actor_horizon_m
            ):
                continue

            velocity = actor.get_velocity()
            extent = actor.bounding_box.extent
            states.append(
                CarlaActorState(
                    track_id=str(actor.id),
                    type_id=actor.type_id,
                    pose_local_to_aabb=pose.to_proto(),
                    aabb=AABB(
                        size_x=2.0 * extent.x,
                        size_y=2.0 * extent.y,
                        size_z=2.0 * extent.z,
                    ),
                    dynamic_state=dynamic_state_proto(
                        linear_velocity=carla_vector_to_local(
                            velocity.x, velocity.y, velocity.z
                        ),
                        angular_velocity=np.zeros(3),
                        linear_acceleration=np.zeros(3),
                    ),
                )
            )
        return states
