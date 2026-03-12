"""Elapsed-time-based scenario pass condition."""

from __future__ import annotations

from typing import TYPE_CHECKING, Optional

from .base import BaseCondition, ScenarioResult
from .comparison import ComparisonRule, ScalarComparisonRule

if TYPE_CHECKING:
    import carla


class ElapsedTimeCondition(BaseCondition):
    """Pass condition that triggers when elapsed time satisfies a comparison.

    By default the condition fires when elapsed time **reaches or exceeds**
    *duration_seconds* (``GREATER_THAN_OR_EQUAL``), which is backward-compatible
    with the original behaviour.

    Unlike :class:`TimeoutCondition` (which returns a *failure*), this condition
    returns a *success* result, making it suitable for scenarios that should pass
    after a certain amount of time (e.g. "drive for 30 seconds without issues").

    Args:
        duration_seconds: Time threshold in seconds.  Must be positive.
        rule: Comparison operator applied to *elapsed* vs *duration_seconds*.
            Defaults to :attr:`ComparisonRule.GREATER_THAN_OR_EQUAL`.
        tolerance: Tolerance for :attr:`ComparisonRule.EQUAL_TO`.
            Defaults to ``1e-6``.
    """

    def __init__(
        self,
        duration_seconds: float,
        rule: ComparisonRule = ComparisonRule.GREATER_THAN_OR_EQUAL,
        tolerance: float = 1e-6,
    ) -> None:
        if duration_seconds <= 0:
            raise ValueError("duration_seconds must be positive")
        if tolerance < 0:
            raise ValueError("tolerance must be non-negative")
        self._comparison = ScalarComparisonRule(
            field="elapsed", rule=rule, value=duration_seconds, tolerance=tolerance
        )

    @property
    def duration_seconds(self) -> float:
        """The time threshold in seconds."""
        return self._comparison.value

    def check(self, world: "carla.World", elapsed: float) -> Optional[ScenarioResult]:
        """Return a success result if the elapsed time satisfies the rule.

        Args:
            world: The CARLA world instance (unused).
            elapsed: Elapsed time in seconds since the scenario started.

        Returns:
            ScenarioResult with passed=True if the condition is met, None otherwise.
        """
        if self._comparison.satisfied(elapsed):
            rule_text = self._comparison.rule.name.lower().replace("_", " ")
            return ScenarioResult(
                passed=True,
                message=(
                    f"Elapsed time {elapsed:.2f}s {rule_text}"
                    f" target duration {self.duration_seconds:.2f}s"
                ),
                elapsed_seconds=elapsed,
            )
        return None
