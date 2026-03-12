"""Example scenario: Ego vehicle turns left at the next intersection.

This scenario demonstrates :class:`~autoware_carla_scenario.TurnAction` by:

1. Spawning the ego vehicle on lanelet 244, which approaches an intersection
   with a left turn option.
2. Setting all traffic lights to green.
3. Registering a :class:`TurnAction` (``LEFT``) that analyses the OpenDRIVE
   road network to find the next junction ahead of the ego, computes a left
   turn route, and applies it via ``TrafficManager.set_path``.
4. Verifying the ego traverses the expected post-turn OpenDRIVE roads.

Typical usage
-------------
Standalone (no pytest)::

    uv run left-turn map.name=NishishinjyukuMap

With pytest — see ``test/carla_scenario/test_examples.py``.
"""

from __future__ import annotations

import logging

import carla

from autoware_carla_scenario import (
    EGO_ROLE_NAME,
    AndCondition,
    BaseScenario,
    EgoConfig,
    EntityLanePositionCondition,
    Lanelet2Pose,
    SpawnTransform,
    StickyCondition,
    TickTiming,
    TimeoutCondition,
    TurnAction,
    TurnDirection,
    find_actor_by_role_name,
    set_all_traffic_lights_state,
    snap_to_carla_road,
    to_opendrive,
)

from .configs import LeftTurnConfig

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

#: Lanelet where the ego is spawned.  Lanelet 203 approaches an intersection
#: with a left turn option (lanelet 415 only goes straight).
SPAWN_LANELET_ID: int = 203

#: Lanelets the ego is expected to traverse **after** the left turn.
#: Adjust these IDs based on the target map.  The scenario converts each
#: lanelet to its OpenDRIVE road ID and passes the check when the ego has
#: visited every listed road.
POST_TURN_LANELET_IDS: list[int] = [411, 207]

#: Timeout in seconds.  Left turns require more time than going straight
#: because the vehicle decelerates, turns, and re-accelerates.
SCENARIO_TIMEOUT_SECONDS: float = 10.0

#: Initial ego speed (km/h).
INITIAL_SPEED_KMH: float = 20.0


def _lanelet_start_road_id(lanelet_id: int) -> str:
    """Return the OpenDRIVE road ID at the start of a lanelet centerline."""
    pose = Lanelet2Pose(lanelet_id=lanelet_id, s=0.0)
    return to_opendrive(pose).road_id


class LeftTurnScenario(BaseScenario):
    """Spawn the ego at lanelet 244 and verify it turns left at the next junction.

    The scenario:

    1. Snaps a Lanelet2 pose to the CARLA road surface for the ego spawn.
    2. Sets every traffic light to green so the ego proceeds without stopping.
    3. Registers a :class:`TurnAction` (``LEFT``) that fires on the first tick
       to compute a left turn route through the next junction and set it via
       ``TrafficManager.set_path``.
    4. Registers a pass condition: :class:`AndCondition` of sticky
       :class:`EntityLanePositionCondition` instances for each post-turn road.
    5. Registers a :class:`TimeoutCondition` as a fail-safe.
    """

    def __init__(
        self,
        ego_config: EgoConfig,
        config: LeftTurnConfig | None = None,
    ) -> None:
        super().__init__(ego_config)
        self._config = config or LeftTurnConfig()

    def setup(self) -> None:
        """Snap ego spawn, set lights green, register TurnAction and conditions."""
        world = self.world
        cfg = self._config
        # --- Ego spawn via OpenDRIVE pose ---
        # Convert Lanelet2 → OpenDRIVE first, then snap via
        # get_waypoint_xodr for accurate CARLA surface position (no
        # spawn-point z approximation needed).
        ll2_pose = Lanelet2Pose(lanelet_id=cfg.spawn_lanelet_id, s=cfg.spawn_s)
        od_pose = to_opendrive(ll2_pose)
        snapped = snap_to_carla_road(od_pose, world)

        logger.info(
            "Lanelet %d -> OpenDRIVE road='%s' lane=%d s=%.1f -> "
            "CARLA (%.1f, %.1f, %.3f) yaw=%.1f",
            cfg.spawn_lanelet_id,
            od_pose.road_id,
            od_pose.lane_id,
            od_pose.s,
            snapped.x,
            snapped.y,
            snapped.z,
            snapped.yaw,
        )

        self.ego_config.spawn_location = SpawnTransform(snapped.to_carla_transform())

        # --- Spectator and logging ---
        ego_actor = lambda: find_actor_by_role_name(world, EGO_ROLE_NAME)  # noqa: E731
        self.follow_with_spectator(ego_actor)
        self.log_actor_position(ego_actor, label="ego")

        # --- Set all traffic lights to green ---
        n = set_all_traffic_lights_state(world, carla.TrafficLightState.Green)
        logger.info("Set %d traffic lights to green", n)

        # --- TurnAction: left turn at the next junction ---
        # No condition specified → defaults to AlwaysTrueCondition, so the
        # action fires on the first tick (once=True by default).
        turn_action = TurnAction(
            entity_name=EGO_ROLE_NAME,
            direction=TurnDirection.LEFT,
            client=self.client,
            timing=TickTiming.PRE_TICK,
        )
        self.register_pre_tick(turn_action)

        # --- Pass: ego reaches post-turn roads ---
        seen: set[str] = set()
        route_road_ids: list[str] = []
        for ll_id in cfg.post_turn_lanelet_ids:
            rid = _lanelet_start_road_id(ll_id)
            logger.info("Post-turn lanelet %d -> OpenDRIVE road '%s'", ll_id, rid)
            if rid not in seen:
                seen.add(rid)
                route_road_ids.append(rid)
        logger.info("Pass condition: ego visits roads %s", route_road_ids)

        sticky_conditions = [
            StickyCondition(
                EntityLanePositionCondition(
                    entity_name=EGO_ROLE_NAME,
                    road_id=rid,
                )
            )
            for rid in route_road_ids
        ]

        self.register_pass_condition(AndCondition(sticky_conditions))

        # --- Fail-safe timeout ---
        self.register_fail_condition(TimeoutCondition(cfg.timeout_seconds))

    def is_done(self) -> bool:
        """Always ``False`` — termination is driven by pass/fail conditions."""
        return False
