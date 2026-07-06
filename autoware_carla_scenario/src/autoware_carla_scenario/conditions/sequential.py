"""Condition that requires its children to be satisfied in order."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Optional

from .base import BaseCondition, ScenarioResult

if TYPE_CHECKING:
    import carla


class SequentialCondition(BaseCondition):
    """Satisfied once every child condition has passed **in order**.

    Unlike :class:`AndCondition`, which is order-insensitive, this condition
    only evaluates the next child once all preceding children have passed. A
    child that would pass out of turn is therefore not counted until its
    predecessors have, so reaching waypoints (or completing steps) in the wrong
    order never satisfies the sequence.

    Progress latches: once a child has passed, the condition advances past it
    permanently, even if that child would no longer pass on its own.

    Args:
        conditions: The ordered child conditions.
        label: Human-readable identifier for this condition.
    """

    def __init__(self, conditions: list[BaseCondition], *, label: str) -> None:
        super().__init__(label=label)
        if not conditions:
            raise ValueError(
                f"{type(self).__name__}: at least one child condition is required"
            )
        self._conditions = list(conditions)
        self._index = 0

    def check(self, world: "carla.World", elapsed: float) -> Optional[ScenarioResult]:
        # Advance through as many consecutive satisfied children as possible.
        while self._index < len(self._conditions):
            result = self._conditions[self._index].check(world, elapsed)
            if result is not None and result.passed:
                self._index += 1
            else:
                break

        if self._index >= len(self._conditions):
            return ScenarioResult(
                passed=True,
                message=f"all {len(self._conditions)} steps satisfied in order",
                elapsed_seconds=elapsed,
            )
        return None

    def get_details(self) -> dict[str, Any]:
        return {
            "wrapper": "sequential",
            "completed": self._index,
            "total": len(self._conditions),
            "children": [c.to_summary_dict() for c in self._conditions],
        }
