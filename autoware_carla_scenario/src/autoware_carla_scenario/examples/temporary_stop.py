"""Example scenario: Ego vehicle performs a temporary stop at a designated position.

This scenario spawns the ego vehicle on lanelet 285 and verifies that it
performs a temporary stop using
:class:`~autoware_carla_scenario.TemporaryStopCondition`.

The pass condition requires the ego to be within ``s_margin`` of the stop
position on the correct OpenDRIVE road **and** remain below
``speed_threshold`` for at least ``stop_duration`` seconds.

Typical usage
-------------
Standalone::

    uv run scenario scenario=temporary_stop/temporary_stop

With pytest — see ``test/carla_scenario/test_examples.py``.
"""

from __future__ import annotations

import logging

import carla

from autoware_carla_scenario import (
    EGO_ROLE_NAME,
    BaseScenario,
    EgoConfig,
    Lanelet2Pose,
    SpawnTransform,
    TemporaryStopCondition,
    TimeoutCondition,
    find_actor_by_role_name,
    set_all_traffic_lights_state,
    snap_to_carla_road,
    to_opendrive,
)

from .configs import TemporaryStopConfig

logger = logging.getLogger(__name__)


class TemporaryStopScenario(BaseScenario):
    """Spawn the ego and verify it performs a temporary stop at the target position.

    The scenario:

    1. Snaps a Lanelet2 pose to the CARLA road surface for the ego spawn.
    2. Sets every traffic light to green so the ego proceeds normally.
    3. Registers a :class:`TemporaryStopCondition` as the pass condition,
       targeting the configured stop position.
    4. Registers a :class:`TimeoutCondition` as a fail-safe.

    The ego is expected to detect the stop line (or other regulatory element)
    on the lanelet and perform a temporary stop autonomously.
    """

    def __init__(
        self,
        ego_config: EgoConfig,
        config: TemporaryStopConfig | None = None,
        spawn_pose: Lanelet2Pose | None = None,
    ) -> None:
        super().__init__(ego_config)
        self._config = config or TemporaryStopConfig()
        self._spawn_pose = spawn_pose

    def setup(self) -> None:
        """Snap ego spawn to CARLA road, register temporary stop condition."""
        world = self.world
        cfg = self._config

        # --- Compute ego spawn from Lanelet2Pose via OpenDrivePose ---
        if self._spawn_pose is None:
            msg = "spawn_pose is required for TemporaryStopScenario"
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

        # --- Build stop position from config ---
        stop_pose = Lanelet2Pose(
            lanelet_id=cfg.stop_lanelet_id,
            s=cfg.stop_s,
        )
        logger.info(
            "Stop position: lanelet %d s=%.1f",
            cfg.stop_lanelet_id,
            cfg.stop_s,
        )

        # --- Pass condition: temporary stop at the target position ---
        self.register_pass_condition(
            TemporaryStopCondition(
                entity_name=EGO_ROLE_NAME,
                stop_positions=[stop_pose],
                s_margin=cfg.s_margin,
                speed_threshold=cfg.speed_threshold,
                stop_duration=cfg.stop_duration,
            )
        )

        # --- Fail-safe timeout ---
        self.register_fail_condition(TimeoutCondition(cfg.timeout_seconds))

    def is_done(self) -> bool:
        """Always ``False`` -- termination is driven by pass/fail conditions."""
        return False
