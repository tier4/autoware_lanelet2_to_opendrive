"""A minimal external scenario: verify the ego reaches its goal roads.

This module lives in a **separate installable package**, not inside
``autoware_carla_scenario``.  It only depends on the framework's public API,
which it imports from the top-level package.

Note: importing this module pulls in :class:`BaseScenario`, which imports CARLA
at module scope, so it requires the framework's runtime environment.  The
package ``__init__`` imports this module lazily (inside ``register()``) so that
merely discovering the entry point stays lightweight.  The CARLA world itself
is only touched inside :meth:`setup`, which runs on the live server.
"""

from __future__ import annotations

import logging

from autoware_carla_scenario import (
    EGO_ROLE_NAME,
    AndCondition,
    BaseScenario,
    EgoConfig,
    EntityLanePositionCondition,
    GroundProjectionConfig,
    Lanelet2Pose,
    StickyCondition,
    TimeoutCondition,
    to_opendrive,
)

from .configs import ReachGoalConfig

logger = logging.getLogger(__name__)


class ReachGoalScenario(BaseScenario):
    """Pass when the ego has visited every configured goal road."""

    def __init__(
        self,
        ego_config: EgoConfig,
        spawn_pose: Lanelet2Pose,
        config: ReachGoalConfig | None = None,
        ground_projection: GroundProjectionConfig | None = None,
    ) -> None:
        super().__init__(
            ego_config, spawn_pose=spawn_pose, ground_projection=ground_projection
        )
        self._config = config or ReachGoalConfig()

    def setup(self) -> None:
        """Snap the ego spawn and register the pass/fail conditions."""
        # 1. Snap the Lanelet2 spawn pose to the CARLA road surface.
        self._setup_ego_spawn()

        cfg = self._config

        # 2. Pass condition: the ego reaches each goal road (latched via Sticky).
        stickies = []
        for lanelet_id in cfg.goal_lanelet_ids:
            od = to_opendrive(Lanelet2Pose(lanelet_id=lanelet_id, s=0.0))
            logger.info(
                "Goal lanelet %d -> OpenDRIVE road '%s'", lanelet_id, od.road_id
            )
            stickies.append(
                StickyCondition(
                    EntityLanePositionCondition(
                        entity_name=EGO_ROLE_NAME,
                        road_id=od.road_id,
                    )
                )
            )
        self.register_pass_condition(AndCondition(stickies))

        # 3. Fail-safe timeout.
        self.register_fail_condition(
            TimeoutCondition(cfg.timeout_seconds, label="scenario_timeout")
        )

    def is_done(self) -> bool:
        """Termination is driven entirely by the pass/fail conditions."""
        return False
