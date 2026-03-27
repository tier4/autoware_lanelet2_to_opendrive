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
    AndCondition,
    BaseScenario,
    ComparisonRule,
    EgoConfig,
    GroundProjectionConfig,
    Lanelet2Pose,
    SpeedCondition,
    StickyCondition,
    TemporaryStopCondition,
    TimeoutCondition,
    TrafficLightTarget,
    TrafficSignalAction,
    get_stop_line_poses_with_following,
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
        ground_projection: GroundProjectionConfig | None = None,
    ) -> None:
        super().__init__(
            ego_config, spawn_pose=spawn_pose, ground_projection=ground_projection
        )
        self._config = config or TemporaryStopConfig()

    def setup(self) -> None:
        """Snap ego spawn to CARLA road, register temporary stop condition."""
        world = self.world
        cfg = self._config

        # --- Compute ego spawn from Lanelet2Pose via OpenDrivePose ---
        self._setup_ego_spawn()
        assert self._spawn_pose is not None  # validated by _setup_ego_spawn

        # --- Set all traffic lights to green ---
        TrafficSignalAction(
            state=carla.TrafficLightState.Green,
            lanelet2_traffic_light_ids=TrafficLightTarget.ALL,
            label="set_all_green",
        ).execute(world)

        # --- Auto-detect stop line from spawn lanelet + following lanelets ---
        spawn_lanelet_id = self._spawn_pose.lanelet_id
        stop_poses = get_stop_line_poses_with_following(spawn_lanelet_id)
        if not stop_poses:
            msg = (
                f"No stop lines found on lanelet {spawn_lanelet_id} "
                "or its following lanelets."
            )
            raise ValueError(msg)
        for pose in stop_poses:
            od = to_opendrive(pose)
            logger.info(
                "Stop line detected: lanelet %d s=%.1f -> OpenDRIVE road='%s' s=%.1f",
                pose.lanelet_id,
                pose.s,
                od.road_id,
                od.s,
            )

        # --- Pass condition: temporary stop + restart ---
        # 1) Sticky: latches once ego has temporarily stopped at the position
        stopped = StickyCondition(
            TemporaryStopCondition(
                entity_name=EGO_ROLE_NAME,
                stop_positions=stop_poses,
                s_margin=cfg.s_margin,
                speed_threshold=cfg.speed_threshold,
                stop_duration=cfg.stop_duration,
                label="temporary_stop",
            )
        )
        # 2) Ego has restarted (speed exceeds threshold)
        restarted = SpeedCondition(
            entity_name=EGO_ROLE_NAME,
            value=cfg.restart_speed_kmh / 3.6,
            rule=ComparisonRule.GREATER_THAN_OR_EQUAL,
            label="ego_restart_speed",
        )
        self.register_pass_condition(AndCondition([stopped, restarted]))

        # --- Fail-safe timeout ---
        self.register_fail_condition(
            TimeoutCondition(cfg.timeout_seconds, label="scenario_timeout")
        )

    def is_done(self) -> bool:
        """Always ``False`` -- termination is driven by pass/fail conditions."""
        return False
