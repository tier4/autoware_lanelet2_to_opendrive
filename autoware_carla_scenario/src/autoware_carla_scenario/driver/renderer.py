"""Build the CARLA ground-truth payload a driver policy reads.

The alpasim contract carries no traffic lights and no other vehicles.  Both ride
inside ``egodriver.DriveRequest.renderer_data``, an upstream-sanctioned ``bytes``
extension point, as a serialized ``carla_driver.v0.CarlaRendererData``.  A policy
that ignores those bytes stays fully compatible; one that wants CARLA ground
truth parses them.

Sending this matters more than it looks.  A policy such as
`stl_driver <https://github.com/hakuturu583/stl_driver>`_ reads the payload
defensively -- a missing one yields "no light applies" and "no other vehicles"
rather than an error -- so an empty ``renderer_data`` does not fail loudly.  It
silently disables every rule that depends on the world outside the ego.

The logic here is ported from ``carla_driver_interface``'s reference runtime
(``runtime/carla_world.py`` at ``af1dcd3``, Apache-2.0) so that a policy tuned
against that runtime sees the same numbers here.  The two subtleties worth
keeping are documented at :meth:`RendererDataBuilder._governing_traffic_light`
and :meth:`RendererDataBuilder._stop_line_points`; both were derived from
measurements on ``Town10HD_Opt`` upstream, and re-deriving them by intuition
produces a policy that brakes too late and stops too early.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Any, Dict, List, Optional, Tuple

import numpy as np
from numpy.typing import NDArray

from ._proto import carla_driver_pb2, common_pb2
from .base import DriverClientConfig
from .geometry import Pose
from .observation import to_local_pose, to_local_vector

if TYPE_CHECKING:
    import carla


logger = logging.getLogger(__name__)

__all__ = ["RendererDataBuilder"]

#: How far past a stop waypoint to look for the junction it governs, in metres.
_STOP_LINE_SEARCH_M: float = 8.0

#: Step size for that walk, in metres.
_STOP_LINE_STEP_M: float = 0.5

#: Distance reported when no stop line applies to the ego.  The proto documents
#: the field as negative in that case.
_NO_STOP_LINE_M: float = -1.0

#: CARLA reports posted speed limits in km/h.
_KMH_TO_MPS: float = 1.0 / 3.6

#: ``carla.TrafficLightState`` stringifies to these names.
_LIGHT_STATES: Dict[str, "carla_driver_pb2.TrafficLightState"] = {
    "Red": carla_driver_pb2.TRAFFIC_LIGHT_STATE_RED,
    "Yellow": carla_driver_pb2.TRAFFIC_LIGHT_STATE_YELLOW,
    "Green": carla_driver_pb2.TRAFFIC_LIGHT_STATE_GREEN,
    "Off": carla_driver_pb2.TRAFFIC_LIGHT_STATE_OFF,
}

#: Weather fields, read defensively because CARLA 0.10 dropped some of 0.9's.
_WEATHER_FIELDS = (
    "cloudiness",
    "precipitation",
    "precipitation_deposits",
    "wind_intensity",
    "sun_azimuth_angle",
    "sun_altitude_angle",
    "fog_density",
    "wetness",
)


class RendererDataBuilder:
    """Collects CARLA ground truth for one ego, once per policy step.

    Traffic-light geometry is cached: lights and junctions do not move, and the
    lane-graph walks behind them are not cheap enough to repeat at policy rate.

    Args:
        world: The CARLA world.
        ego_actor: The ego vehicle actor.
        config: Driver settings; supplies the sight distance, the actor horizon
            and the lane-walk step.
    """

    def __init__(
        self,
        world: "carla.World",
        ego_actor: "carla.Actor",
        config: DriverClientConfig,
    ) -> None:
        self._world = world
        self._ego = ego_actor
        self._config = config
        self._map = world.get_map()
        self._stop_lines: Optional[Dict[Tuple[int, int], List[Any]]] = None
        self._stop_line_points: Dict[int, List[NDArray[np.float64]]] = {}

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def build(self, timestamp_us: int, ego_pose: Pose) -> bytes:
        """Return the serialized payload for this instant.

        Args:
            timestamp_us: Simulation time of the snapshot.
            ego_pose: The ego's rig pose in the local frame, used to resolve
                distances along the ego's heading.

        Returns:
            The serialized ``CarlaRendererData``, or ``b""`` if collection
            failed -- a ground-truth hiccup must not abort a scenario.
        """
        try:
            return self._build(timestamp_us, ego_pose).SerializeToString()
        except (RuntimeError, AttributeError, TypeError, ValueError):
            # Best-effort by design: the payload is an optional extension, and a
            # CARLA hiccup here must not take the scenario down with it.
            logger.warning("Failed to collect CARLA ground truth", exc_info=True)
            return b""

    def _build(
        self, timestamp_us: int, ego_pose: Pose
    ) -> carla_driver_pb2.CarlaRendererData:
        """Assemble the payload."""
        light = self._governing_traffic_light(ego_pose)
        return carla_driver_pb2.CarlaRendererData(
            snapshot_timestamp_us=timestamp_us,
            frame_id=self._frame_id(),
            map_name=str(self._map.name),
            weather=self._weather(),
            ego_traffic_light=self._light_state(light),
            ego_traffic_light_distance_m=self._light_distance(light, ego_pose),
            speed_limit_mps=self._speed_limit_mps(),
            actors=(
                self._actor_states(ego_pose)
                if self._config.send_actor_ground_truth
                else []
            ),
        )

    # ------------------------------------------------------------------
    # Traffic lights
    # ------------------------------------------------------------------

    def _governing_traffic_light(self, ego_pose: Pose) -> Optional[Any]:
        """The light the ego must obey, found by looking down its own lane.

        CARLA's ``is_at_traffic_light()`` answers a different question -- whether
        the ego is *inside the light's trigger volume* -- and those volumes are
        about a metre thick along the road.  A policy asking CARLA therefore
        learns of a red light at the moment it arrives at it, when stopping from
        any ordinary speed is already impossible, and the overrun that follows
        says nothing about the policy.

        So the lane graph is walked forward instead, and any light whose stop
        line lies on one of those lanes governs us.  That is the question a
        driver answers by looking.

        Inside a junction nothing governs us: having crossed the line, the thing
        to do is clear the box.  Reading a light there is an active hazard, since
        a trigger volume reaches past its own line and a junction has four of
        them -- a vehicle in the middle can stand in the volume of a light
        governing traffic that crosses its path, and be told to stop where
        stopping is worst.

        Falls back to ``is_at_traffic_light()`` when the sight distance is zero,
        and, outside a junction, whenever the walk finds nothing.
        """
        if self._config.traffic_light_sight_distance_m > 0.0:
            waypoint = self._ego_waypoint()
            if waypoint is not None and waypoint.is_junction:
                return None
            lanes = self._lanes_ahead(self._config.traffic_light_sight_distance_m)
            light = self._nearest_light_ahead(lanes, ego_pose)
            if light is not None:
                return light
        try:
            if self._ego.is_at_traffic_light():
                return self._ego.get_traffic_light()
        except (RuntimeError, AttributeError):
            logger.debug("is_at_traffic_light() unavailable", exc_info=True)
        return None

    def _ego_waypoint(self) -> Optional[Any]:
        """Where the ego sits on the lane graph, or ``None`` if nowhere."""
        return self._map.get_waypoint(self._ego.get_location(), project_to_road=True)

    def _lanes_ahead(self, distance_m: float) -> List[Tuple[int, int]]:
        """``(road_id, lane_id)`` of the lanes up to the next junction.

        Walked rather than guessed, because a stop line sits on the lane it
        governs and the ego is often still on an earlier segment of road when it
        needs to know.  The walk stops at the junction it reaches -- the lane at
        the mouth may still carry the line, but nothing past it is ours to obey
        yet -- and reports nothing once the ego is already inside one.
        """
        waypoint = self._ego_waypoint()
        if waypoint is None or waypoint.is_junction:
            return []

        step = max(1.0, self._config.route_resolution_m)
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
            if waypoint.is_junction:
                break
        return lanes

    def _nearest_light_ahead(
        self, lanes: List[Tuple[int, int]], ego_pose: Pose
    ) -> Optional[Any]:
        """The nearest light on *lanes* whose stop line is still ahead of the ego.

        :meth:`_lights_on_lanes` keys its index only by ``(road_id, lane_id)``, so
        a light whose stop line sits behind the ego on the current lane -- or
        beyond the sight distance -- lands in the same bucket as one genuinely
        ahead, and a bucket holds its lights in CARLA's arbitrary actor order.
        Ranking the candidates by longitudinal distance and dropping any already
        passed or out of sight is what turns "a light on our lane" back into "the
        light we are about to reach".
        """
        sight = self._config.traffic_light_sight_distance_m
        nearest: Optional[Any] = None
        nearest_distance = float("inf")
        for light in self._lights_on_lanes(lanes):
            distance = self._light_distance(light, ego_pose)
            if distance < 0.0 or distance > sight:
                continue
            if distance < nearest_distance:
                nearest, nearest_distance = light, distance
        return nearest

    def _lights_on_lanes(self, lanes: List[Tuple[int, int]]) -> List[Any]:
        """Lights whose stop lines lie on *lanes*.

        The index is keyed by ``(road_id, lane_id)`` alone, so a lane's bucket can
        hold a light behind the ego or beyond the horizon, in CARLA's arbitrary
        actor order.  :meth:`_nearest_light_ahead` is what filters and ranks these
        candidates by their stop-line position relative to the ego.
        """
        if not lanes:
            return []
        index = self._stop_lines_by_lane()
        found: List[Any] = []
        for lane in lanes:
            found.extend(index.get(lane, ()))
        return found

    def _stop_lines_by_lane(self) -> Dict[Tuple[int, int], List[Any]]:
        """Which light governs which lane, built once -- lights do not move."""
        if self._stop_lines is None:
            index: Dict[Tuple[int, int], List[Any]] = {}
            for light in self._world.get_actors().filter("traffic.traffic_light*"):
                for waypoint in self._stop_waypoints(light):
                    key = (waypoint.road_id, waypoint.lane_id)
                    index.setdefault(key, []).append(light)
            self._stop_lines = index
        return self._stop_lines

    @staticmethod
    def _stop_waypoints(light: Any) -> List[Any]:
        """Return a light's stop waypoints, tolerating CARLA API differences."""
        try:
            return list(light.get_stop_waypoints())
        except (RuntimeError, AttributeError):
            logger.debug("get_stop_waypoints() unavailable", exc_info=True)
            return []

    def _light_state(
        self, light: Optional[Any]
    ) -> "carla_driver_pb2.TrafficLightState":
        """Map a CARLA light state onto the proto enum."""
        if light is None:
            return carla_driver_pb2.TRAFFIC_LIGHT_STATE_NONE
        return _LIGHT_STATES.get(
            str(light.get_state()), carla_driver_pb2.TRAFFIC_LIGHT_STATE_UNKNOWN
        )

    def _light_distance(self, light: Optional[Any], ego_pose: Pose) -> float:
        """Distance the ego still has to travel before the stop line.

        Measured **along the ego's heading**, not as a straight line, and
        negative once the line is behind.  The difference matters: a Euclidean
        distance starts growing again the moment the ego crosses the line, so a
        policy reading it cannot tell "1 m to go" from "1 m past", and a policy
        that correctly stops just inside the trigger volume is told forever that
        there is a line ahead which it has in fact already crossed.
        """
        if light is None:
            return _NO_STOP_LINE_M

        points = self._stop_line_points_for(light)
        if not points:
            return _NO_STOP_LINE_M

        # +x in the rig frame is straight ahead.
        longitudinal = ego_pose.inverse().transform_points(np.stack(points))[:, 0]
        ahead = longitudinal[longitudinal >= 0.0]
        if len(ahead):
            return float(ahead.min())
        # All behind: report the nearest of those, so the value stays continuous
        # as the ego crosses the line.
        return float(longitudinal.max())

    def _stop_line_points_for(self, light: Any) -> List[NDArray[np.float64]]:
        """Where a vehicle should stop for *light*, in the local frame.

        Not the stop waypoints themselves, which sit further back than the line a
        driver aims at -- upstream measured them a median of 5.5 m upstream of
        the junction they govern on ``Town10HD_Opt``.  A policy told to stop
        there hesitates most of a car-and-a-half short of the mouth.  So each
        stop waypoint is walked forward to the mouth of its junction and that is
        the point reported.

        Cached: lights and junctions do not move, and this walks the lane graph
        half a metre at a time.
        """
        key = int(getattr(light, "id", 0))
        cached = self._stop_line_points.get(key)
        if cached is None:
            cached = [
                to_local_pose(self._junction_mouth(waypoint).transform).position
                for waypoint in self._stop_waypoints(light)
            ]
            if not cached:
                # No stop waypoints: the light's own position is the best
                # available answer.
                cached = [to_local_pose(light.get_transform()).position]
            self._stop_line_points[key] = cached
        return cached

    @staticmethod
    def _junction_mouth(stop: Any) -> Any:
        """The first waypoint of the junction *stop* governs.

        Falls back to the stop waypoint itself when the walk finds no junction --
        a stop line on open road is unusual but not ours to second-guess.
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

    # ------------------------------------------------------------------
    # Ego and world state
    # ------------------------------------------------------------------

    def _frame_id(self) -> int:
        """The CARLA snapshot frame counter, for correlating with recordings."""
        try:
            return int(self._world.get_snapshot().frame)
        except (RuntimeError, AttributeError):
            return 0

    def _speed_limit_mps(self) -> float:
        """The posted speed limit for the ego lane, in m/s.  0 when unknown."""
        try:
            return float(self._ego.get_speed_limit() or 0.0) * _KMH_TO_MPS
        except (RuntimeError, AttributeError):
            return 0.0

    def _weather(self) -> carla_driver_pb2.CarlaWeather:
        """The current weather, reading each field defensively."""
        weather = self._world.get_weather()
        return carla_driver_pb2.CarlaWeather(
            **{
                name: float(getattr(weather, name, 0.0) or 0.0)
                for name in _WEATHER_FIELDS
            }
        )

    def _actor_states(self, ego_pose: Pose) -> List[carla_driver_pb2.CarlaActorState]:
        """Every other vehicle within the horizon, in the local frame.

        Poses carry yaw only.  A policy reads this quaternion to orient the
        actor's bounding box in the ground plane, which roll and pitch do not
        affect for a vehicle on a road.
        """
        states: List[carla_driver_pb2.CarlaActorState] = []
        horizon = self._config.actor_horizon_m
        ego_position = ego_pose.position

        for actor in self._world.get_actors().filter("*vehicle*"):
            if actor.id == self._ego.id:
                continue
            pose = to_local_pose(actor.get_transform())
            if float(np.linalg.norm(pose.position - ego_position)) > horizon:
                continue

            velocity = actor.get_velocity()
            extent = actor.bounding_box.extent
            states.append(
                carla_driver_pb2.CarlaActorState(
                    track_id=str(actor.id),
                    type_id=str(actor.type_id),
                    pose_local_to_aabb=pose.to_proto(),
                    aabb=common_pb2.AABB(
                        size_x=2.0 * extent.x,
                        size_y=2.0 * extent.y,
                        size_z=2.0 * extent.z,
                    ),
                    dynamic_state=common_pb2.DynamicState(
                        # Resolved in the local frame, not the actor's own: a
                        # policy rotates these into its rig frame and would
                        # double-count an actor-frame velocity.
                        linear_velocity=_vec3(
                            to_local_vector(velocity.x, velocity.y, velocity.z)
                        ),
                    ),
                )
            )
        return states


def _vec3(values: NDArray[np.float64]) -> common_pb2.Vec3:
    """Return *values* as a protobuf ``Vec3``."""
    return common_pb2.Vec3(x=values[0], y=values[1], z=values[2])
