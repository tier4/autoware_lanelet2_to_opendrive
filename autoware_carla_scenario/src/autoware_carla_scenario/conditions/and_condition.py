"""AND composite condition that requires all child conditions to pass."""

from __future__ import annotations

from typing import Any, List, Optional, Sequence

from ..tick_snapshot import TickSnapshot
from .base import BaseCondition, ScenarioResult


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
            "children": [c.to_summary_dict() for c in self._conditions],
        }

    def check(self, snapshot: TickSnapshot) -> Optional[ScenarioResult]:
        """Check all child conditions with AND logic.

        Args:
            snapshot: Immutable snapshot of the current tick state.

        Returns:
            :class:`ScenarioResult` with ``passed=False`` if any child fails,
            ``passed=True`` if all children pass, or ``None`` if any child
            is still pending.
        """
        messages: List[str] = []
        for condition in self._conditions:
            result = condition.check(snapshot)
            if result is None:
                return None
            if not result.passed:
                return result
            messages.append(result.message)
        return ScenarioResult(
            passed=True,
            message=" AND ".join(messages),
            elapsed_seconds=snapshot.elapsed,
        )
