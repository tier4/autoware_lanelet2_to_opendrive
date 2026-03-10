"""Example scenario: NPC vehicle passes through an intersection on expected roads.

This scenario spawns an NPC vehicle near lanelet 242, sets all traffic lights
to green, and verifies that the vehicle traverses the expected OpenDRIVE roads
corresponding to lanelets 460 and 265.  Each road check uses a
:class:`~autoware_carla_scenario.StickyCondition` so that the condition latches
once the vehicle visits the road, and the conditions are combined with
:class:`~autoware_carla_scenario.AndCondition` to assert the full route.

Typical usage
-------------
Standalone (no pytest)::

    uv run intersection-passing --xodr /path/to/nishishinjuku.xodr

With pytest — see ``test/carla_scenario/test_examples.py``.
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path
import carla

from autoware_carla_scenario import (
    AndCondition,
    BaseScenario,
    CarlaWorldPose,
    EgoConfig,
    EntityLanePositionCondition,
    Lanelet2Pose,
    ScenarioQueue,
    SpawnTransform,
    StickyCondition,
    TimeoutCondition,
    VehicleEntity,
    VehicleEntityConfig,
    set_all_traffic_lights_state,
    to_carla_world,
    to_lanelet2,
    to_opendrive,
)

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Lanelet IDs that define the expected route
# ---------------------------------------------------------------------------

#: Lanelet where the NPC is spawned (used to derive a nearby spawn position).
SPAWN_LANELET_ID: int = 242

#: Ordered lanelets the NPC is expected to traverse through the intersection.
EXPECTED_ROUTE_LANELET_IDS: list[int] = [460, 265]

#: Timeout in seconds — if the NPC has not completed the route by this time the
#: scenario fails.
SCENARIO_TIMEOUT_SECONDS: float = 120.0


def _lanelet_start_road_id(lanelet_id: int) -> str:
    """Return the OpenDRIVE road ID at the start of a lanelet centerline."""
    pose = Lanelet2Pose(lanelet_id=lanelet_id, s=0.0)
    od = to_opendrive(pose)
    return od.road_id


class IntersectionPassingScenario(BaseScenario):
    """Spawn an NPC near lanelet 242 and verify it crosses expected roads.

    The scenario:

    1. Converts lanelet 242 (s=0) to CARLA world coordinates for the spawn.
    2. Spawns an NPC vehicle with autopilot enabled.
    3. Sets every traffic light in the world to green so the NPC proceeds
       without stopping.
    4. Registers a pass condition: an :class:`AndCondition` of sticky
       :class:`EntityLanePositionCondition` instances, one per expected road.
    5. Registers a :class:`TimeoutCondition` as a fail-safe.

    Once all expected roads have been visited the scenario passes.
    """

    NPC_ROLE_NAME: str = "npc_intersection"

    #: Number of world ticks to wait before enabling autopilot.
    #: This gives CARLA time to stabilise after map load / actor spawn
    #: before the TrafficManager builds its InMemoryMap (which blocks).
    AUTOPILOT_DELAY_TICKS: int = 5

    def __init__(self, ego_config: EgoConfig) -> None:
        super().__init__(ego_config)
        self._npc: VehicleEntity | None = None
        self._autopilot_tick_count: int = 0
        self._autopilot_enabled: bool = False

    def setup(self, world: carla.World) -> None:
        """Spawn NPC, set lights to green, register route conditions."""
        # --- Spawn NPC near lanelet 242 (autopilot deferred) ---
        spawn_pose = Lanelet2Pose(lanelet_id=SPAWN_LANELET_ID, s=0.0)
        carla_pose = to_carla_world(spawn_pose)

        # Find the nearest CARLA spawn point to the Lanelet2 target position.
        # The exact Lanelet2 position may collide with static geometry (walls,
        # kerbs, etc.), so we use the map's validated spawn points instead.
        spawn_points = world.get_map().get_spawn_points()
        target = carla.Location(x=carla_pose.x, y=carla_pose.y, z=0.0)
        nearest_sp = min(
            spawn_points,
            key=lambda sp: target.distance(sp.location),
        )
        spawn_tf = nearest_sp
        logger.info(
            "Target from lanelet %d: (%.1f, %.1f, %.1f) -> nearest spawn point: "
            "(%.1f, %.1f, %.1f) yaw=%.1f  dist=%.1f m",
            SPAWN_LANELET_ID,
            carla_pose.x,
            carla_pose.y,
            carla_pose.z,
            nearest_sp.location.x,
            nearest_sp.location.y,
            nearest_sp.location.z,
            nearest_sp.rotation.yaw,
            target.distance(nearest_sp.location),
        )

        npc_config = VehicleEntityConfig(
            role_name=self.NPC_ROLE_NAME,
            spawn_location=SpawnTransform(spawn_tf),
            vehicle_type="vehicle.fuso.mitsubishi",
            autopilot=False,
        )
        self._npc = VehicleEntity(npc_config)
        self._npc.spawn(world)
        logger.info("NPC spawned near lanelet %d", SPAWN_LANELET_ID)

        # Defer autopilot activation to let CARLA stabilise first
        self.register_post_tick(self._enable_autopilot_after_delay)

        # Follow NPC with spectator camera for visual debugging
        self.register_post_tick(self._follow_npc_with_spectator)

        # Diagnostic: log NPC position every ~1 s (20 ticks)
        self._diag_tick = 0
        self.register_post_tick(self._log_npc_position)

        # --- Set all traffic lights to green ---
        n = set_all_traffic_lights_state(world, carla.TrafficLightState.Green)
        logger.info("Set %d traffic lights to green", n)

        # --- Build unique route road IDs from lanelet IDs ---
        seen: set[str] = set()
        route_road_ids: list[str] = []
        for ll_id in EXPECTED_ROUTE_LANELET_IDS:
            rid = _lanelet_start_road_id(ll_id)
            logger.info("Lanelet %d -> OpenDRIVE road '%s'", ll_id, rid)
            if rid not in seen:
                seen.add(rid)
                route_road_ids.append(rid)

        # --- Register sticky lane-position conditions ---
        sticky_conditions = [
            StickyCondition(
                EntityLanePositionCondition(
                    entity_name=self.NPC_ROLE_NAME,
                    road_id=rid,
                )
            )
            for rid in route_road_ids
        ]

        pass_condition = AndCondition(sticky_conditions)
        self.register_pass_condition(pass_condition)

        # --- Fail-safe timeout ---
        self.register_fail_condition(TimeoutCondition(SCENARIO_TIMEOUT_SECONDS))

    def _follow_npc_with_spectator(self, world: carla.World) -> None:
        """Move the spectator camera behind and above the NPC for debugging."""
        if self._npc is None or self._npc.actor is None:
            return
        try:
            npc_tf = self._npc.actor.get_transform()
        except RuntimeError:
            return
        # Place spectator 8m behind, 5m above, looking slightly down
        fwd = npc_tf.get_forward_vector()
        spectator_loc = carla.Location(
            x=npc_tf.location.x - 8.0 * fwd.x,
            y=npc_tf.location.y - 8.0 * fwd.y,
            z=npc_tf.location.z + 5.0,
        )
        spectator_rot = carla.Rotation(
            yaw=npc_tf.rotation.yaw,
            pitch=-15.0,
        )
        spectator = world.get_spectator()
        spectator.set_transform(carla.Transform(spectator_loc, spectator_rot))

    def _log_npc_position(self, world: carla.World) -> None:
        """Log the NPC's current OpenDRIVE road ID every ~1 second."""
        self._diag_tick += 1
        if self._diag_tick % 20 != 0:
            return
        if self._npc is None or self._npc.actor is None:
            return
        try:
            loc = self._npc.actor.get_location()
            vel = self._npc.actor.get_velocity()
        except RuntimeError:
            logger.warning("[diag] NPC actor no longer valid")
            return
        speed = (vel.x**2 + vel.y**2 + vel.z**2) ** 0.5
        carla_pose = CarlaWorldPose(x=loc.x, y=loc.y, z=loc.z)
        try:
            od = to_opendrive(carla_pose)
            ll = to_lanelet2(carla_pose)
            logger.info(
                "[diag] road='%s' lane=%d s=%.1f | lanelet=%d | speed=%.1f m/s | "
                "pos=(%.1f, %.1f, %.1f)",
                od.road_id,
                od.lane_id,
                od.s,
                ll.lanelet_id,
                speed,
                loc.x,
                loc.y,
                loc.z,
            )
        except Exception as exc:
            logger.warning("[diag] transform failed: %s", exc)

    def _enable_autopilot_after_delay(self, world: carla.World) -> None:
        """Enable autopilot on the NPC after a short delay."""
        if self._autopilot_enabled:
            return
        self._autopilot_tick_count += 1
        if self._autopilot_tick_count >= self.AUTOPILOT_DELAY_TICKS:
            assert self._npc is not None
            assert self._npc.actor is not None
            self._npc.actor.set_autopilot(True)
            self._autopilot_enabled = True
            logger.info("Autopilot enabled after %d ticks", self._autopilot_tick_count)

    def is_done(self) -> bool:
        """Always ``False`` — termination is driven by pass/fail conditions."""
        return False


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------


def main() -> None:
    """Run IntersectionPassingScenario as a standalone script.

    Requires ``--map`` (and optionally ``--xodr`` to overwrite the built-in
    map's ``.xodr``) to load the Nishishinjuku map that contains lanelet 242.

    Example::

        uv run intersection-passing --map NishishinjyukuMap
        uv run intersection-passing --map NishishinjyukuMap --xodr path/to/nishishinjuku.xodr
    """
    parser = argparse.ArgumentParser(
        description="Run the intersection-passing example scenario.",
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
        default="vehicle.fuso.mitsubishi",
        help="Ego vehicle blueprint ID",
    )
    parser.add_argument(
        "--spawn-index", type=int, default=0, help="Ego spawn point index"
    )

    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO, format="%(levelname)s %(name)s: %(message)s"
    )

    # Ego vehicle (required by framework but not the focus of this scenario)
    ego = EgoConfig(
        spawn_location=SpawnTransform(
            carla.Transform(carla.Location(x=0.0, y=0.0, z=0.5))
        ),
        vehicle_type=args.vehicle,
    )

    queue = ScenarioQueue(
        host=args.host,
        port=args.port,
        xodr_path=args.xodr,
        lanelet2_path=args.lanelet2,
        map_name=args.map,
    )
    queue.add(IntersectionPassingScenario(ego))

    with queue:
        results = queue.run_all()

    result = results[0]
    print(result)
    sys.exit(0 if result.passed else 1)


if __name__ == "__main__":
    main()
