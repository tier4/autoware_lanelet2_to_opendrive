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

    uv run traffic-light-compliance --map NishishinjyukuMap

With pytest — see ``test/carla_scenario/test_examples.py``.
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path

import carla

from autoware_carla_scenario import (
    EGO_ROLE_NAME,
    AndCondition,
    BaseScenario,
    ComparisonRule,
    EgoConfig,
    ElapsedTimeCondition,
    Lanelet2Pose,
    ScenarioQueue,
    SpawnTransform,
    SpeedCondition,
    StickyCondition,
    TimeoutCondition,
    find_actor_by_role_name,
    set_all_traffic_lights_state,
    snap_to_carla_road,
)

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

    def __init__(self, ego_config: EgoConfig) -> None:
        super().__init__(ego_config)

    def setup(self, world: carla.World) -> None:
        """Snap ego spawn, set lights to red, register conditions."""
        # --- Compute ego spawn from Lanelet2Pose ---
        spawn_pose = Lanelet2Pose(lanelet_id=SPAWN_LANELET_ID, s=15.0)
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
            if elapsed >= LIGHT_SWITCH_DELAY_SECONDS:
                count = set_all_traffic_lights_state(w, carla.TrafficLightState.Green)
                logger.info(
                    "Switched %d traffic lights to green at %.2fs", count, elapsed
                )
                switch_state["switched"] = True

        self.register_pre_tick(_switch_lights)

        # --- Pass conditions ---
        # 1) Verify ego stopped during red phase (1.0–2.9 s)
        stopped_at_red = StickyCondition(
            AndCondition(
                [
                    ElapsedTimeCondition(1.0, ComparisonRule.GREATER_THAN_OR_EQUAL),
                    ElapsedTimeCondition(2.9, ComparisonRule.LESS_THAN),
                    SpeedCondition(
                        entity_name=EGO_ROLE_NAME,
                        value=0.0,
                        rule=ComparisonRule.LESS_THAN_OR_EQUAL,
                    ),
                ]
            )
        )

        # 2) Verify ego moving after green phase (>= 3.5 s)
        moving_after_green = StickyCondition(
            AndCondition(
                [
                    ElapsedTimeCondition(3.5, ComparisonRule.GREATER_THAN_OR_EQUAL),
                    SpeedCondition(
                        entity_name=EGO_ROLE_NAME,
                        value=MOVING_SPEED_KMH / 3.6,
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
                        value=MOVING_SPEED_KMH / 3.6,
                        rule=ComparisonRule.GREATER_THAN_OR_EQUAL,
                    ),
                ]
            )
        )

        # --- Fail-safe timeout ---
        self.register_fail_condition(TimeoutCondition(SCENARIO_TIMEOUT_SECONDS))

    def is_done(self) -> bool:
        """Always ``False`` — termination is driven by pass/fail conditions."""
        return False


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------


def main() -> None:
    """Run TrafficLightComplianceScenario as a standalone script.

    Requires ``--map`` (and optionally ``--xodr`` to overwrite the built-in
    map's ``.xodr``) to load the Nishishinjuku map that contains lanelet 242.

    Example::

        uv run traffic-light-compliance --map NishishinjyukuMap
        uv run traffic-light-compliance --map NishishinjyukuMap --xodr path/to/nishishinjuku.xodr
    """
    parser = argparse.ArgumentParser(
        description="Run the traffic-light-compliance example scenario.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--host", default="localhost", help="CARLA server host")
    parser.add_argument("--port", type=int, default=2000, help="CARLA server port")
    parser.add_argument("--map", required=True, help="Built-in CARLA map name")
    parser.add_argument(
        "--xodr",
        type=Path,
        default=None,
        help="Path to OpenDRIVE (.xodr) file (overwrites the built-in map)",
    )
    parser.add_argument(
        "--lanelet2",
        type=Path,
        default=None,
        help="Path to Lanelet2 (.osm) file (required for coordinate transforms)",
    )
    parser.add_argument(
        "--vehicle",
        default="vehicle.mini.cooper",
        help="Ego vehicle blueprint ID",
    )

    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO, format="%(levelname)s %(name)s: %(message)s"
    )

    # Ego spawn location is determined in setup() via snap_to_carla_road;
    # provide a dummy transform that will be overwritten before ego.spawn().
    ego = EgoConfig(
        spawn_location=SpawnTransform(
            carla.Transform(carla.Location(x=0.0, y=0.0, z=0.0))
        ),
        vehicle_type=args.vehicle,
        initial_speed_kmh=0.0,
    )

    queue = ScenarioQueue(
        host=args.host,
        port=args.port,
        xodr_path=args.xodr,
        lanelet2_path=args.lanelet2,
        map_name=args.map,
    )
    queue.add(TrafficLightComplianceScenario(ego))

    with queue:
        results = queue.run_all()

    result = results[0]
    print(result)
    sys.exit(0 if result.passed else 1)


if __name__ == "__main__":
    main()
