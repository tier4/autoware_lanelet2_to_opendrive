"""Sticky condition that latches after the first passing result."""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Any, Optional

from .base import BaseCondition, ScenarioResult

if TYPE_CHECKING:
    import carla

logger = logging.getLogger(__name__)


class StickyCondition(BaseCondition):
    """Wrapper that latches a child condition after its first passing result.

    Once the wrapped condition returns a :class:`ScenarioResult` with
    ``passed=True``, the result is stored and returned on every subsequent
    call — the inner condition is never re-evaluated.

    This is useful for verifying that an entity *visited* a state at some point,
    even if it has since moved on.  Combine multiple sticky conditions with
    :class:`AndCondition` to assert that an entity traversed a specific
    sequence of states::

        pass_cond = AndCondition([
            StickyCondition(EntityLanePositionCondition("npc", road_id="10")),
            StickyCondition(EntityLanePositionCondition("npc", road_id="20")),
        ])

    Args:
        condition: The child condition to wrap.
    """

    def __init__(self, condition: BaseCondition, *, label: str | None = None) -> None:
        auto_label = f"{condition.label}_sticky"
        super().__init__(label=label if label is not None else auto_label)
        self._condition = condition
        self._latched_result: Optional[ScenarioResult] = None

    def get_details(self) -> dict[str, Any]:
        return {
            "wrapper": "sticky",
            "latched": self._latched_result is not None,
            "child": {
                "condition_type": type(self._condition).__name__,
                "label": self._condition.label,
                **self._condition.get_details(),
            },
        }

    def check(self, world: "carla.World", elapsed: float) -> Optional[ScenarioResult]:
        """Return the latched result if available, otherwise delegate to the child.

        Args:
            world: The CARLA world instance.
            elapsed: Elapsed time in seconds since the scenario started.

        Returns:
            The latched :class:`ScenarioResult` if the child previously passed,
            otherwise the child's current result (which may be ``None``).
        """
        if self._latched_result is not None:
            return self._latched_result

        result = self._condition.check(world, elapsed)
        if result is not None and result.passed:
            self._latched_result = result
            logger.info(
                "StickyCondition: latched — %s",
                result.message,
            )
        return result
