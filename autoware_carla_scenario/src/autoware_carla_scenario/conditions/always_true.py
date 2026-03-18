"""Condition that is always satisfied."""

from __future__ import annotations

from typing import Optional

from ..tick_snapshot import TickSnapshot
from .base import BaseCondition, ScenarioResult


class AlwaysTrueCondition(BaseCondition):
    """A condition that returns a successful result on every check.

    Useful as a trigger for actions that should fire unconditionally
    (subject to the action's own ``once`` flag).
    """

    def __init__(self, *, label: str = "always_true") -> None:
        super().__init__(label=label)

    def check(self, snapshot: TickSnapshot) -> Optional[ScenarioResult]:
        """Always returns a passing :class:`ScenarioResult`."""
        return ScenarioResult(
            passed=True,
            message="AlwaysTrueCondition",
            elapsed_seconds=snapshot.elapsed,
        )
