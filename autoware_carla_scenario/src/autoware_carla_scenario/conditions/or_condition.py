"""OR composite condition that requires any child condition to pass."""

from __future__ import annotations

from typing import Any, Optional, Sequence

from ..tick_snapshot import TickSnapshot
from .base import BaseCondition, ScenarioResult


class OrCondition(BaseCondition):
    """Composite condition that requires ANY child condition to be satisfied.

    Evaluation rules:

    - If any child returns a non-``None`` result, return it immediately.
    - If all children return ``None``, return ``None``.

    Args:
        conditions: Child conditions to evaluate.  Must contain at least 2.
    """

    def __init__(
        self, conditions: Sequence[BaseCondition], *, label: str | None = None
    ) -> None:
        if len(conditions) < 2:
            raise ValueError("OrCondition requires at least 2 conditions")
        auto_label = " OR ".join(c.label for c in conditions)
        super().__init__(label=label if label is not None else auto_label)
        self._conditions: Sequence[BaseCondition] = conditions

    def get_details(self) -> dict[str, Any]:
        return {
            "operator": "OR",
            "children": [c.to_summary_dict() for c in self._conditions],
        }

    def check(self, snapshot: TickSnapshot) -> Optional[ScenarioResult]:
        """Check child conditions with OR logic.

        Args:
            snapshot: Immutable snapshot of the current tick state.

        Returns:
            The first non-``None`` :class:`ScenarioResult` from any child,
            or ``None`` if all children are still pending.
        """
        for condition in self._conditions:
            result = condition.check(snapshot)
            if result is not None:
                return result
        return None
