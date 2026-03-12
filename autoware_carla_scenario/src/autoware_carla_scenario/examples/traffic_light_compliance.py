"""Example scenario: Ego vehicle stops at a red traffic light and proceeds when green.

This scenario verifies traffic light compliance by:

1. Spawning the ego vehicle on lanelet 242 with zero initial speed.
2. Setting all traffic lights to **red** immediately.
3. After 3 seconds, switching all traffic lights to **green**.
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
import time

import carla

from autoware_carla_scenario import (
    EGO_ROLE_NAME,
    AndCondition,
    BaseScenario,
    ComparisonRule,
    EgoConfig,
    ElapsedTimeCondition,
    Lanelet2Pose,
    SpawnTransform,
    SpeedCondition,
    StandstillCondition,
    StickyCondition,
    TimeoutCondition,
    find_actor_by_role_name,
    set_all_traffic_lights_state,
    snap_to_carla_road,
)

from .configs import TrafficLightComplianceConfig

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

#: Lanelet where the ego is spawned.
SPAWN_LANELET_ID: int = 242

#: Delay in seconds before traffic lights switch from red to green.
LIGHT_SWITCH_DELAY_SECONDS: float = 3.0

#: Timeout in seconds — fail-safe if the scenario takes too long.
SCENARIO_TIMEOUT_SECONDS: float = 5.0

#: Speed threshold (km/h) considered "moving".
MOVING_SPEED_KMH: float = 1.0


class TrafficLightComplianceScenario(BaseScenario):
    """Verify ego obeys traffic signals: stop at red, proceed at green.

    The scenario:

    1. Snaps a Lanelet2 pose to the CARLA road surface for the ego spawn.
    2. Enables autopilot so the ego drives via the traffic manager.
    3. Sets every traffic light to **red** (frozen).
    4. Registers a pre-tick callback to switch lights to **green** after
       :data:`LIGHT_SWITCH_DELAY_SECONDS`.
    5. Registers pass conditions that verify the ego is stopped during the
       red phase and moving during the green phase.
    6. Registers a :class:`TimeoutCondition` as a fail-safe.
    """

    def __init__(
        self,
        ego_config: EgoConfig,
        config: TrafficLightComplianceConfig | None = None,
    ) -> None:
        super().__init__(ego_config)
        self._config = config or TrafficLightComplianceConfig()

    def setup(self) -> None:
        """Snap ego spawn, set lights to red, register conditions."""
        world = self.world
        cfg = self._config
        # --- Compute ego spawn from Lanelet2Pose ---
        spawn_pose = Lanelet2Pose(lanelet_id=cfg.spawn_lanelet_id, s=cfg.spawn_s)
        snapped = snap_to_carla_road(spawn_pose, world)

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

        # Use BaseScenario helpers for common post-tick patterns
        ego_actor = lambda: find_actor_by_role_name(world, EGO_ROLE_NAME)  # noqa: E731
        self.follow_with_spectator(ego_actor)
        self.log_actor_position(ego_actor, label="ego")

        # --- Set all traffic lights to RED ---
        n = set_all_traffic_lights_state(world, carla.TrafficLightState.Red)
        logger.info("Set %d traffic lights to red", n)

        # --- Register callback to switch lights to green after delay ---
        switch_state: dict[str, object] = {
            "start": None,
            "switched": False,
        }

        def _switch_lights(w: carla.World) -> None:
            if switch_state["switched"]:
                return
            if switch_state["start"] is None:
                switch_state["start"] = time.monotonic()
                return
            elapsed = time.monotonic() - float(switch_state["start"])  # type: ignore[arg-type]
            if elapsed >= cfg.light_switch_delay_seconds:
                count = set_all_traffic_lights_state(w, carla.TrafficLightState.Green)
                logger.info(
                    "Switched %d traffic lights to green at %.2fs", count, elapsed
                )
                switch_state["switched"] = True

        self.register_pre_tick(_switch_lights)

        # --- Pass conditions ---
        # 1) Verify ego stopped during red phase (standstill for >= cfg.light_switch_delay_seconds - merging time)
        stopped_at_red = StickyCondition(
            StandstillCondition(
                entity_name=EGO_ROLE_NAME,
                duration=cfg.light_switch_delay_seconds - cfg.merging_time_seconds,
            )
        )

        # 2) Verify ego moving after green phase (>= 3.5 s)
        moving_after_green = StickyCondition(
            AndCondition(
                [
                    ElapsedTimeCondition(3.5, ComparisonRule.GREATER_THAN_OR_EQUAL),
                    SpeedCondition(
                        entity_name=EGO_ROLE_NAME,
                        value=cfg.moving_speed_kmh / 3.6,
                        rule=ComparisonRule.GREATER_THAN_OR_EQUAL,
                    ),
                ]
            )
        )

        self.register_pass_condition(AndCondition([stopped_at_red, moving_after_green]))

        # --- Fail: ego moved during red phase (1.0–2.9 s) ---
        self.register_fail_condition(
            AndCondition(
                [
                    ElapsedTimeCondition(1.0, ComparisonRule.GREATER_THAN_OR_EQUAL),
                    ElapsedTimeCondition(2.9, ComparisonRule.LESS_THAN),
                    SpeedCondition(
                        entity_name=EGO_ROLE_NAME,
                        value=cfg.moving_speed_kmh / 3.6,
                        rule=ComparisonRule.GREATER_THAN_OR_EQUAL,
                    ),
                ]
            )
        )

        # --- Fail-safe timeout ---
        self.register_fail_condition(TimeoutCondition(cfg.timeout_seconds))

    def is_done(self) -> bool:
        """Always ``False`` — termination is driven by pass/fail conditions."""
        return False
