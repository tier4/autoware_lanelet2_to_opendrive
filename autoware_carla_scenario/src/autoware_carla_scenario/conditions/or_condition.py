"""OR composite condition that requires any child condition to pass."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Optional, Sequence

from .base import BaseCondition, ScenarioResult

if TYPE_CHECKING:
    import carla


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
