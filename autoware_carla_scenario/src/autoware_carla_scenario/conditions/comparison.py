"""Shared comparison rule for condition evaluation."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
from typing import Any


class ComparisonRule(Enum):
    """Comparison operator for condition evaluation.

    Attributes:
        GREATER_THAN: Value must be strictly greater than the threshold.
        LESS_THAN: Value must be strictly less than the threshold.
        EQUAL_TO: Value must be approximately equal (within tolerance).
        GREATER_THAN_OR_EQUAL: Value must be greater than or equal.
        LESS_THAN_OR_EQUAL: Value must be less than or equal.
    """

    GREATER_THAN = auto()
    LESS_THAN = auto()
    EQUAL_TO = auto()
    GREATER_THAN_OR_EQUAL = auto()
    LESS_THAN_OR_EQUAL = auto()


def compare(
    actual: float, rule: ComparisonRule, value: float, tolerance: float = 1e-6
) -> bool:
    """Return ``True`` if *actual* satisfies *rule* against *value*.

    Args:
        actual: The measured value.
        rule: The comparison operator.
        value: The threshold to compare against.
        tolerance: Tolerance for :attr:`ComparisonRule.EQUAL_TO`.

    Returns:
        Whether the comparison is satisfied.

    Raises:
        ValueError: If *rule* is not a known :class:`ComparisonRule` member.
    """
    if rule == ComparisonRule.GREATER_THAN:
        return actual > value
    if rule == ComparisonRule.LESS_THAN:
        return actual < value
    if rule == ComparisonRule.EQUAL_TO:
        return abs(actual - value) <= tolerance
    if rule == ComparisonRule.GREATER_THAN_OR_EQUAL:
        return actual >= value
    if rule == ComparisonRule.LESS_THAN_OR_EQUAL:
        return actual <= value
    raise ValueError(f"Unknown comparison rule: {rule}")


@dataclass(frozen=True)
class ScalarComparisonRule:
    """A comparison rule applied to a named scalar field (e.g. 's', 't').

    Attributes:
        field: Name of the scalar to compare ('s' or 't').
        rule: The comparison operator.
        value: The threshold value.
        tolerance: Tolerance for EQUAL_TO comparisons.
    """

    field: str
    rule: ComparisonRule
    value: float
    tolerance: float = 1e-6

    def satisfied(self, actual: float) -> bool:
        """Return True if *actual* satisfies this rule."""
        return compare(actual, self.rule, self.value, self.tolerance)

    def to_dict(self) -> dict[str, Any]:
        """Return a JSON-serialisable dict representation."""
        return {
            "field": self.field,
            "rule": self.rule.name,
            "value": self.value,
        }
