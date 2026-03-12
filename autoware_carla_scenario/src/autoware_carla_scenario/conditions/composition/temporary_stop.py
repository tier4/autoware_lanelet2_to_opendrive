"""Temporary stop condition for detecting standstill at specified positions."""

from __future__ import annotations

from typing import TYPE_CHECKING, Optional, Sequence

from ...coordinate.poses import AnyPose, OpenDrivePose
from ...coordinate.transform import to_opendrive
from ..and_condition import AndCondition
from ..base import BaseCondition, ScenarioResult
from ..comparison import ComparisonRule, ScalarComparisonRule
from ..or_condition import OrCondition
from ..persistent import PersistentCondition
from .base import CompositionCondition
from .entity_lane_position import EntityLanePositionCondition
from .speed import SpeedCondition

if TYPE_CHECKING:
    import carla


class TemporaryStopCondition(CompositionCondition):
    """Pass when entity temporarily stops at any of the given positions.

    For each stop position, constructs a composite condition:
    ``PersistentCondition(AndCondition(EntityLanePositionCondition, SpeedCondition))``.

    When multiple stop positions are given, they are combined with
    :class:`OrCondition` — stopping at any one position is sufficient.

    An :class:`EntityExistenceCondition` guard ensures the entity is present
    before the inner conditions are evaluated.

    Args:
        entity_name: The ``role_name`` attribute of the actor to track.
        stop_positions: One or more poses where a stop is expected.
            Each pose is converted to :class:`OpenDrivePose` for road/s matching.
        s_margin: Arc-length margin (m) around each stop position.
            The entity must be within ``[s - s_margin, s + s_margin]``.
        speed_threshold: Maximum speed (m/s) considered as stopped.
        stop_duration: Minimum consecutive seconds the entity must remain
            stopped at the position.

    Raises:
        ValueError: If *stop_positions* is empty, *s_margin* is not positive,
            *speed_threshold* is negative, or *stop_duration* is not positive.
    """

    def __init__(
        self,
        entity_name: str,
        stop_positions: Sequence[AnyPose],
        s_margin: float = 5.0,
        speed_threshold: float = 0.1,
        stop_duration: float = 1.0,
    ) -> None:
        if not stop_positions:
            raise ValueError("stop_positions must not be empty")
        if s_margin <= 0:
            raise ValueError("s_margin must be positive")
        if speed_threshold < 0:
            raise ValueError("speed_threshold must be non-negative")
        if stop_duration <= 0:
            raise ValueError("stop_duration must be positive")

        super().__init__(entity_name=entity_name)
        self._entity_name = entity_name

        persistent_conditions: list[PersistentCondition] = []
        for pose in stop_positions:
            od_pose = self._to_od(pose)
            position_cond = EntityLanePositionCondition(
                entity_name=entity_name,
                road_id=od_pose.road_id,
                rules=[
                    ScalarComparisonRule(
                        field="s",
                        rule=ComparisonRule.GREATER_THAN_OR_EQUAL,
                        value=od_pose.s - s_margin,
                    ),
                    ScalarComparisonRule(
                        field="s",
                        rule=ComparisonRule.LESS_THAN_OR_EQUAL,
                        value=od_pose.s + s_margin,
                    ),
                ],
            )
            speed_cond = SpeedCondition(
                entity_name=entity_name,
                value=speed_threshold,
                rule=ComparisonRule.LESS_THAN_OR_EQUAL,
            )
            and_cond = AndCondition([position_cond, speed_cond])
            persistent = PersistentCondition(and_cond, duration=stop_duration)
            persistent_conditions.append(persistent)

        if len(persistent_conditions) == 1:
            self._inner: BaseCondition = persistent_conditions[0]
        else:
            self._inner = OrCondition(persistent_conditions)

    @staticmethod
    def _to_od(pose: AnyPose) -> OpenDrivePose:
        """Convert any pose to OpenDrivePose."""
        if isinstance(pose, OpenDrivePose):
            return pose
        return to_opendrive(pose)

    def _check(self, world: "carla.World", elapsed: float) -> Optional[ScenarioResult]:
        """Return a pass result if the entity has stopped at any target position.

        The entity is guaranteed to exist by the
        :class:`EntityExistenceCondition` guard.

        Args:
            world: The CARLA world instance.
            elapsed: Elapsed time in seconds since the scenario started.

        Returns:
            :class:`ScenarioResult` with ``passed=True`` if the entity has
            remained stopped at a target position for the required duration,
            ``None`` otherwise.
        """
        return self._inner.check(world, elapsed)
