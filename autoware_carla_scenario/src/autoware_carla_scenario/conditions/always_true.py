"""Condition that is always satisfied."""

from __future__ import annotations

from typing import TYPE_CHECKING, Optional

from .base import BaseCondition, ScenarioResult

if TYPE_CHECKING:
    import carla


class AlwaysTrueCondition(BaseCondition):
    """A condition that returns a successful result on every check.

    Useful as a trigger for actions that should fire unconditionally
    (subject to the action's own ``once`` flag).
    """

    def __init__(self, *, label: str = "always_true") -> None:
        super().__init__(label=label)

    def check(self, world: "carla.World", elapsed: float) -> Optional[ScenarioResult]:
        """Always returns a passing :class:`ScenarioResult`."""
        return ScenarioResult(
            passed=True,
            message="AlwaysTrueCondition",
            elapsed_seconds=elapsed,
        )
