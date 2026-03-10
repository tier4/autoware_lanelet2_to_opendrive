"""Composite conditions that combine child conditions with AND/OR/Sticky logic."""

from __future__ import annotations

from typing import TYPE_CHECKING, List, Optional, Sequence

from .base import BaseCondition, ScenarioResult

if TYPE_CHECKING:
    import carla


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

    def __init__(self, condition: BaseCondition) -> None:
        self._condition = condition
        self._latched_result: Optional[ScenarioResult] = None

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
        return result


class AndCondition(BaseCondition):
    """Composite condition that requires ALL child conditions to be satisfied.

    Evaluation rules:

    - If any child returns ``passed=False``, return that failure immediately.
    - If any child returns ``None`` (not yet determined), return ``None``.
    - If all children return ``passed=True``, return a combined pass result.

    Args:
        conditions: Child conditions to evaluate.  Must contain at least 2.
    """

    def __init__(self, conditions: Sequence[BaseCondition]) -> None:
        if len(conditions) < 2:
            raise ValueError("AndCondition requires at least 2 conditions")
        self._conditions: Sequence[BaseCondition] = conditions

    def check(self, world: "carla.World", elapsed: float) -> Optional[ScenarioResult]:
        """Check all child conditions with AND logic.

        Args:
            world: The CARLA world instance.
            elapsed: Elapsed time in seconds since the scenario started.

        Returns:
            :class:`ScenarioResult` with ``passed=False`` if any child fails,
            ``passed=True`` if all children pass, or ``None`` if any child
            is still pending.
        """
        messages: List[str] = []
        for condition in self._conditions:
            result = condition.check(world, elapsed)
            if result is None:
                return None
            if not result.passed:
                return result
            messages.append(result.message)
        return ScenarioResult(
            passed=True,
            message=" AND ".join(messages),
            elapsed_seconds=elapsed,
        )


class OrCondition(BaseCondition):
    """Composite condition that requires ANY child condition to be satisfied.

    Evaluation rules:

    - If any child returns a non-``None`` result, return it immediately.
    - If all children return ``None``, return ``None``.

    Args:
        conditions: Child conditions to evaluate.  Must contain at least 2.
    """

    def __init__(self, conditions: Sequence[BaseCondition]) -> None:
        if len(conditions) < 2:
            raise ValueError("OrCondition requires at least 2 conditions")
        self._conditions: Sequence[BaseCondition] = conditions

    def check(self, world: "carla.World", elapsed: float) -> Optional[ScenarioResult]:
        """Check child conditions with OR logic.

        Args:
            world: The CARLA world instance.
            elapsed: Elapsed time in seconds since the scenario started.

        Returns:
            The first non-``None`` :class:`ScenarioResult` from any child,
            or ``None`` if all children are still pending.
        """
        for condition in self._conditions:
            result = condition.check(world, elapsed)
            if result is not None:
                return result
        return None
