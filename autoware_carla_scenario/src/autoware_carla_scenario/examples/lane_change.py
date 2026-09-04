"""Example scenario: Ego vehicle performs a lane change on a straight road.

This scenario spawns the ego vehicle on a configurable lanelet, sets all
traffic lights to green, and registers a
:class:`~autoware_carla_scenario.LaneChangeAction` to force a lane change
via TrafficManager when the trigger condition is met.

The pass condition is derived automatically from the spawn pose and the
requested direction.  A lane change stays on the same OpenDRIVE road and
shifts the lane ID by one:

- **LEFT**  → ``lane_id + 1`` (toward the centre line)
- **RIGHT** → ``lane_id - 1`` (away from the centre line)

The scenario checks that the ego's lane ID changes as expected.

Typical usage
-------------
Left lane change::

    uv run scenario scenario=lane_change/left

Right lane change::

    uv run scenario scenario=lane_change/right

With pytest — see ``test/carla_scenario/test_examples.py``.
"""

from __future__ import annotations

import logging

import carla

from autoware_carla_scenario import (
    EGO_ROLE_NAME,
    BaseScenario,
    EgoConfig,
    EntityLanePositionCondition,
    GroundProjectionConfig,
    LaneChangeAction,
    LaneChangeDirection,
    Lanelet2Pose,
    NotCondition,
    StickyCondition,
    TickTiming,
    TimeoutCondition,
    TrafficLightTarget,
    TrafficSignalAction,
)

from .configs import LaneChangeConfig

logger = logging.getLogger(__name__)

_DIRECTION_MAP: dict[str, LaneChangeDirection] = {
    "left": LaneChangeDirection.LEFT,
    "right": LaneChangeDirection.RIGHT,
}

#: OpenDRIVE lane-ID offset per direction.
#: Right-side driving lanes have negative IDs; LEFT moves toward the
#: centre (id + 1), RIGHT moves away (id - 1).
_LANE_ID_DELTA: dict[LaneChangeDirection, int] = {
    LaneChangeDirection.LEFT: 1,
    LaneChangeDirection.RIGHT: -1,
}


class LaneChangeScenario(BaseScenario):
    """Spawn the ego and verify it completes a lane change.

    The scenario:

    1. Snaps a Lanelet2 pose to the CARLA road surface for the ego spawn.
    2. Sets every traffic light in the world to green.
    3. Registers a :class:`LaneChangeAction` that fires immediately.
    4. Derives the expected target lane from the spawn lane ID and direction,
       then registers a pass condition checking both road ID and lane ID.
    5. Registers a :class:`TimeoutCondition` as a fail-safe.
    """

    def __init__(
        self,
        ego_config: EgoConfig,
        spawn_pose: Lanelet2Pose,
        config: LaneChangeConfig | None = None,
        ground_projection: GroundProjectionConfig | None = None,
    ) -> None:
        super().__init__(
            ego_config, spawn_pose=spawn_pose, ground_projection=ground_projection
        )
        self._config = config or LaneChangeConfig()

    def setup(self) -> None:
        """Snap ego spawn to CARLA road, register lane-change action and conditions."""
        world = self.world
        cfg = self._config

        # --- Compute ego spawn from Lanelet2Pose via OpenDrivePose ---
        od_pose = self._setup_ego_spawn()

        # --- Set all traffic lights to green ---
        TrafficSignalAction(
            state=carla.TrafficLightState.Green,
            lanelet2_traffic_light_ids=TrafficLightTarget.ALL,
            label="set_all_green",
        ).execute(world)

        # --- Lane-change action (fires immediately) ---
        direction = _DIRECTION_MAP[cfg.direction]
        lane_change_action = LaneChangeAction(
            entity_name=EGO_ROLE_NAME,
            direction=direction,
            client=self.client,
            timing=TickTiming.PRE_TICK,
            tm_port=self.tm_port,
        )
        self.register_pre_tick(lane_change_action)
        logger.info("Registered LaneChangeAction: %s", cfg.direction)

        # --- Conditions based on expected outcome ---
        # Target lane ID is derived from spawn lane ID + direction delta.
        target_lane_id = od_pose.lane_id + _LANE_ID_DELTA[direction]
        expect_fail = cfg.expect == "fail"
        logger.info(
            "Expecting lane change: road='%s' lane %d -> %d (%s) [expect=%s]",
            od_pose.road_id,
            od_pose.lane_id,
            target_lane_id,
            cfg.direction,
            cfg.expect,
        )

        lane_position_condition = StickyCondition(
            EntityLanePositionCondition(
                entity_name=EGO_ROLE_NAME,
                road_id=od_pose.road_id,
                lane_id=target_lane_id,
                label="ego_target_lane",
            )
        )
        timeout_condition = TimeoutCondition(
            cfg.timeout_seconds, label="scenario_timeout"
        )

        if expect_fail:
            # expect=fail: lane change should NOT happen.
            # If ego reaches the target lane → fail.
            # If timeout expires without lane change → pass.
            self.register_fail_condition(NotCondition(lane_position_condition))
            self.register_pass_condition(timeout_condition)
        else:
            # expect=success: lane change should happen.
            # If ego reaches the target lane → pass.
            # If timeout expires → fail.
            self.register_pass_condition(lane_position_condition)
            self.register_fail_condition(timeout_condition)

    def is_done(self) -> bool:
        """Always ``False`` — termination is driven by pass/fail conditions."""
        return False
