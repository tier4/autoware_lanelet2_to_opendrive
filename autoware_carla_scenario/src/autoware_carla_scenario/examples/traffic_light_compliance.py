"""Example scenario: Ego vehicle stops at a red traffic light and proceeds when green.

This scenario verifies traffic light compliance by:

1. Spawning the ego vehicle on lanelet 242 with zero initial speed.
2. Setting all traffic lights to **red** immediately.
3. After 3 seconds, switching all traffic lights to **green** via
   :class:`~autoware_carla_scenario.TrafficSignalAction` with
   :class:`~autoware_carla_scenario.ElapsedTimeCondition`.
4. Asserting the ego vehicle remains stopped during the red phase (1–2.9 s)
   and begins moving after the green phase (3.1 s onward).

The pass condition combines :class:`~autoware_carla_scenario.ElapsedTimeCondition`
and :class:`~autoware_carla_scenario.SpeedCondition` via
:class:`~autoware_carla_scenario.AndCondition` and
:class:`~autoware_carla_scenario.StickyCondition`.

Typical usage
-------------
Standalone (no pytest)::

    uv run scenario scenario=traffic_light_compliance

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
    GroundProjectionConfig,
    Lanelet2Pose,
    SpawnTransform,
    SpeedCondition,
    StandstillCondition,
    StickyCondition,
    TimeoutCondition,
    TrafficLightTarget,
    TrafficSignalAction,
    find_actor_by_role_name,
    snap_to_carla_road,
    to_opendrive,
)

from .configs import TrafficLightComplianceConfig

logger = logging.getLogger(__name__)


class TrafficLightComplianceScenario(BaseScenario):
    """Verify ego obeys traffic signals: stop at red, proceed at green.

    The scenario:

    1. Snaps a Lanelet2 pose to the CARLA road surface for the ego spawn.
    2. Enables autopilot so the ego drives via the traffic manager.
    3. Sets every traffic light to **red** (frozen) using
       :class:`TrafficSignalAction`.
    4. Registers a :class:`TrafficSignalAction` with
       :class:`ElapsedTimeCondition` to switch lights to **green** after
       the configured delay.
    5. Registers pass conditions that verify the ego is stopped during the
       red phase and moving during the green phase.
    6. Registers a :class:`TimeoutCondition` as a fail-safe.
    """

    def __init__(
        self,
        ego_config: EgoConfig,
        config: TrafficLightComplianceConfig | None = None,
        spawn_pose: Lanelet2Pose | None = None,
        ground_projection: GroundProjectionConfig | None = None,
    ) -> None:
        super().__init__(ego_config)
        self._config = config or TrafficLightComplianceConfig()
        self._spawn_pose = spawn_pose
        self._ground_projection = ground_projection or GroundProjectionConfig()

    def setup(self) -> None:
        """Snap ego spawn, set lights to red, register conditions."""
        world = self.world
        cfg = self._config
        # --- Compute ego spawn from Lanelet2Pose ---
        if self._spawn_pose is None:
            msg = "spawn_pose is required for TrafficLightComplianceScenario"
            raise ValueError(msg)
        spawn_pose = self._spawn_pose
        od_pose = to_opendrive(spawn_pose)
        snapped = snap_to_carla_road(
            od_pose, world, ground_projection=self._ground_projection
        )

        logger.info(
            "Snap Lanelet2Pose(lanelet_id=%d, s=%.1f, t=%.1f) to CARLA road: "
            "(%.1f, %.1f, %.3f) yaw=%.1f",
            spawn_pose.lanelet_id,
            spawn_pose.s,
            spawn_pose.t,
            snapped.x,
            snapped.y,
            snapped.z,
            snapped.yaw,
        )

        # Update ego_config so the framework spawns the ego at the snapped pose
        self.ego_config.spawn_location = SpawnTransform(snapped.to_carla_transform())
        self.ego_config.od_pose = od_pose
        self.ego_config.ground_projection = self._ground_projection

        # Use BaseScenario helpers for common post-tick patterns
        ego_actor = lambda: find_actor_by_role_name(world, EGO_ROLE_NAME)  # noqa: E731
        self.follow_with_spectator(ego_actor)
        self.log_actor_position(ego_actor, label="ego")

        # --- Set all traffic lights to RED ---
        TrafficSignalAction(
            state=carla.TrafficLightState.Red,
            lanelet2_traffic_light_ids=TrafficLightTarget.ALL,
            label="set_all_red",
        ).execute(world)

        # --- Register action to switch lights to green after delay ---
        green_action = TrafficSignalAction(
            state=carla.TrafficLightState.Green,
            lanelet2_traffic_light_ids=TrafficLightTarget.ALL,
            condition=ElapsedTimeCondition(
                cfg.light_switch_delay_seconds, label="light_switch"
            ),
            label="switch_to_green",
        )
        self.register_pre_tick(green_action)

        # --- Pass conditions ---
        # 1) Verify ego stopped during red phase (standstill for >= cfg.light_switch_delay_seconds - merging time)
        stopped_at_red = StickyCondition(
            StandstillCondition(
                entity_name=EGO_ROLE_NAME,
                duration=cfg.light_switch_delay_seconds - cfg.merging_time_seconds,
                label="ego_standstill_at_red",
            )
        )

        # 2) Verify ego moving after green phase
        green_check_time = cfg.light_switch_delay_seconds + 0.5
        moving_after_green = StickyCondition(
            AndCondition(
                [
                    ElapsedTimeCondition(
                        green_check_time,
                        ComparisonRule.GREATER_THAN_OR_EQUAL,
                        label="green_phase_elapsed",
                    ),
                    SpeedCondition(
                        entity_name=EGO_ROLE_NAME,
                        value=cfg.moving_speed_kmh / 3.6,
                        rule=ComparisonRule.GREATER_THAN_OR_EQUAL,
                        label="ego_moving_after_green",
                    ),
                ]
            )
        )

        self.register_pass_condition(AndCondition([stopped_at_red, moving_after_green]))

        # --- Fail: ego moved during red phase ---
        # Window: [merging_time, light_switch_delay - 0.1]
        # After merging_time the ego should have stopped; the window
        # closes just before the green signal to avoid false positives
        # during the transition.
        red_window_start = cfg.merging_time_seconds
        red_window_end = cfg.light_switch_delay_seconds - 0.1
        self.register_fail_condition(
            AndCondition(
                [
                    ElapsedTimeCondition(
                        red_window_start,
                        ComparisonRule.GREATER_THAN_OR_EQUAL,
                        label="red_phase_start",
                    ),
                    ElapsedTimeCondition(
                        red_window_end,
                        ComparisonRule.LESS_THAN,
                        label="red_phase_end",
                    ),
                    SpeedCondition(
                        entity_name=EGO_ROLE_NAME,
                        value=cfg.moving_speed_kmh / 3.6,
                        rule=ComparisonRule.GREATER_THAN_OR_EQUAL,
                        label="ego_moving_during_red",
                    ),
                ]
            )
        )

        # --- Fail-safe timeout ---
        self.register_fail_condition(
            TimeoutCondition(cfg.timeout_seconds, label="scenario_timeout")
        )

    def is_done(self) -> bool:
        """Always ``False`` — termination is driven by pass/fail conditions."""
        return False
