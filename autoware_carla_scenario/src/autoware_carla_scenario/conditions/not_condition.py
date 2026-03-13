"""NOT condition that inverts the satisfied result of a child condition."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Optional

from .base import BaseCondition, ScenarioResult

if TYPE_CHECKING:
    import carla


class NotCondition(BaseCondition):
    """Condition that inverts the result of a child condition.

    When the child returns a satisfied result (``passed=True``), this
    condition flips it to ``passed=False`` and vice versa.  When the child
    returns ``None`` (not yet determined), this also returns ``None``.

    Args:
        condition: The child condition whose result is inverted.
    """

    def __init__(self, condition: BaseCondition, *, label: str | None = None) -> None:
        super().__init__(label=label if label is not None else condition.label)
        self._condition = condition

    def get_details(self) -> dict[str, Any]:
        return {
            "operator": "NOT",
            "child": self._condition.to_summary_dict(),
        }

    def check(self, world: "carla.World", elapsed: float) -> Optional[ScenarioResult]:
        """Check the child condition and invert the result.

        Args:
            world: The CARLA world instance.
            elapsed: Elapsed time in seconds since the scenario started.

        Returns:
            :class:`ScenarioResult` with inverted ``passed`` flag, or
            ``None`` if the child is still pending.
        """
        result = self._condition.check(world, elapsed)
        if result is None:
            return None
        return ScenarioResult(
            passed=not result.passed,
            message=f"NOT({result.message})",
            elapsed_seconds=result.elapsed_seconds,
        )
