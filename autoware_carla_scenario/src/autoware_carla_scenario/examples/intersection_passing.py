"""Example scenario: Ego vehicle passes through an intersection on expected roads.

This scenario spawns the ego vehicle on a configurable lanelet, sets all
traffic lights to green, and verifies that the vehicle traverses the expected
OpenDRIVE roads.  Each road check uses a
:class:`~autoware_carla_scenario.StickyCondition` so that the condition latches
once the vehicle visits the road, and the conditions are combined with
:class:`~autoware_carla_scenario.AndCondition` to assert the full route.

The scenario supports an optional ``turn_direction`` (``"left"`` / ``"right"``)
that registers a :class:`~autoware_carla_scenario.TurnAction` to steer the ego
through the junction, and an optional ``min_speed_kmh`` that adds a
:class:`~autoware_carla_scenario.SpeedCondition` fail condition.

Typical usage
-------------
Straight-through::

    uv run scenario scenario=intersection_passing

Left turn::

    uv run scenario scenario=left_turn

With pytest — see ``test/carla_scenario/test_examples.py``.
"""

from __future__ import annotations

import logging

import carla

from autoware_carla_scenario import (
    EGO_ROLE_NAME,
    AndCondition,
    BaseScenario,
    ComparisonRule,
    EgoConfig,
    ElapsedTimeCondition,
    EntityLanePositionCondition,
    Lanelet2Pose,
    SpawnTransform,
    SpeedCondition,
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

from .configs import IntersectionPassingConfig

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Lanelet IDs that define the expected route
# ---------------------------------------------------------------------------

#: Ordered lanelets the ego is expected to traverse through the intersection.
EXPECTED_ROUTE_LANELET_IDS: list[int] = [460, 265]

#: Timeout in seconds — if the ego has not completed the route by this time the
#: scenario fails.
SCENARIO_TIMEOUT_SECONDS: float = 5.0

#: Minimum speed (km/h) — the scenario fails if the ego drops below this speed.
MIN_SPEED_KMH: float = 5.0


def _lanelet_start_road_id(lanelet_id: int) -> str:
    """Return the OpenDRIVE road ID at the start of a lanelet centerline."""
    pose = Lanelet2Pose(lanelet_id=lanelet_id, s=0.0)
    od = to_opendrive(pose)
    return od.road_id


_TURN_DIRECTION_MAP: dict[str, TurnDirection] = {
    "left": TurnDirection.LEFT,
    "right": TurnDirection.RIGHT,
}


class IntersectionPassingScenario(BaseScenario):
    """Spawn the ego and verify it crosses expected roads through an intersection.

    The scenario:

    1. Snaps a Lanelet2 pose to the CARLA road surface for the ego spawn.
    2. Sets every traffic light in the world to green so the ego proceeds
       without stopping.
    3. If ``turn_direction`` is set, registers a :class:`TurnAction` to steer
       the ego through the junction.
    4. Registers a pass condition: an :class:`AndCondition` of sticky
       :class:`EntityLanePositionCondition` instances, one per expected road.
    5. If ``min_speed_kmh`` is set, registers a :class:`SpeedCondition` that
       fails the scenario when the ego drops below the threshold.
    6. Registers a :class:`TimeoutCondition` as a fail-safe.

    Once all expected roads have been visited the scenario passes.
    """

    def __init__(
        self,
        ego_config: EgoConfig,
        config: IntersectionPassingConfig | None = None,
        spawn_pose: Lanelet2Pose | None = None,
    ) -> None:
        super().__init__(ego_config)
        self._config = config or IntersectionPassingConfig()
        self._spawn_pose = spawn_pose

    def setup(self) -> None:
        """Snap ego spawn to CARLA road, set lights to green, register conditions."""
        world = self.world
        cfg = self._config
        # --- Compute ego spawn from Lanelet2Pose via OpenDrivePose ---
        if self._spawn_pose is None:
            msg = "spawn_pose is required for IntersectionPassingScenario"
            raise ValueError(msg)
        ll2_pose = self._spawn_pose
        od_pose = to_opendrive(ll2_pose)
        snapped = snap_to_carla_road(od_pose, world)

        logger.info(
            "Lanelet %d -> OpenDRIVE road='%s' lane=%d s=%.1f -> "
            "CARLA (%.1f, %.1f, %.3f) yaw=%.1f",
            ll2_pose.lanelet_id,
            od_pose.road_id,
            od_pose.lane_id,
            od_pose.s,
            snapped.x,
            snapped.y,
            snapped.z,
            snapped.yaw,
        )

        # Update ego_config so the framework spawns the ego at the snapped pose
        self.ego_config.spawn_location = SpawnTransform(snapped.to_carla_transform())

        # Use BaseScenario helpers for common post-tick patterns
        ego_actor = lambda: find_actor_by_role_name(world, EGO_ROLE_NAME)  # noqa: E731
        self.follow_with_spectator(ego_actor)
        self.log_actor_position(ego_actor, label="ego")

        # --- Set all traffic lights to green ---
        n = set_all_traffic_lights_state(world, carla.TrafficLightState.Green)
        logger.info("Set %d traffic lights to green", n)

        # --- Optional turn action ---
        if cfg.turn_direction is not None:
            direction = _TURN_DIRECTION_MAP[cfg.turn_direction]
            turn_action = TurnAction(
                entity_name=EGO_ROLE_NAME,
                direction=direction,
                client=self.client,
                timing=TickTiming.PRE_TICK,
                tm_port=self.tm_port,
            )
            self.register_pre_tick(turn_action)
            logger.info("Registered TurnAction: %s", cfg.turn_direction)

        # --- Build unique route road IDs from lanelet IDs ---
        seen: set[str] = set()
        route_road_ids: list[str] = []
        for ll_id in cfg.expected_route_lanelet_ids:
            rid = _lanelet_start_road_id(ll_id)
            logger.info("Lanelet %d -> OpenDRIVE road '%s'", ll_id, rid)
            if rid not in seen:
                seen.add(rid)
                route_road_ids.append(rid)

        # --- Register sticky lane-position conditions ---
        sticky_conditions = [
            StickyCondition(
                EntityLanePositionCondition(
                    entity_name=EGO_ROLE_NAME,
                    road_id=rid,
                )
            )
            for rid in route_road_ids
        ]

        pass_condition = AndCondition(sticky_conditions)
        self.register_pass_condition(pass_condition)

        # --- Fail: speed drops below minimum (if configured) ---
        # Wrapped with ElapsedTimeCondition so the check only activates after
        # a grace period, giving the ego time to reach cruising speed.
        if cfg.min_speed_kmh is not None:
            self.register_fail_condition(
                AndCondition(
                    [
                        ElapsedTimeCondition(cfg.speed_check_delay_seconds),
                        SpeedCondition(
                            entity_name=EGO_ROLE_NAME,
                            value=cfg.min_speed_kmh / 3.6,
                            rule=ComparisonRule.LESS_THAN,
                        ),
                    ]
                )
            )

        # --- Fail-safe timeout ---
        self.register_fail_condition(TimeoutCondition(cfg.timeout_seconds))

    def is_done(self) -> bool:
        """Always ``False`` — termination is driven by pass/fail conditions."""
        return False
