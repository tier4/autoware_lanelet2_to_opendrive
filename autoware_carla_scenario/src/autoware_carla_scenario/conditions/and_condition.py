"""AND composite condition that requires all child conditions to pass."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, List, Optional, Sequence

from .base import BaseCondition, ScenarioResult

if TYPE_CHECKING:
    import carla


class AndCondition(BaseCondition):
    """Composite condition that requires ALL child conditions to be satisfied.

    Evaluation rules:

    - If any child returns ``passed=False``, return that failure immediately.
    - If any child returns ``None`` (not yet determined), return ``None``.
    - If all children return ``passed=True``, return a combined pass result.

    Args:
        conditions: Child conditions to evaluate.  Must contain at least 2.
    """

    def __init__(
        self, conditions: Sequence[BaseCondition], *, label: str | None = None
    ) -> None:
        if len(conditions) < 2:
            raise ValueError("AndCondition requires at least 2 conditions")
        auto_label = " AND ".join(c.label for c in conditions)
        super().__init__(label=label if label is not None else auto_label)
        self._conditions: Sequence[BaseCondition] = conditions

    def get_details(self) -> dict[str, Any]:
        return {
            "operator": "AND",
            "children": [
                {
                    "condition_type": type(c).__name__,
                    "label": c.label,
                    **c.get_details(),
                }
                for c in self._conditions
            ],
        }

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
