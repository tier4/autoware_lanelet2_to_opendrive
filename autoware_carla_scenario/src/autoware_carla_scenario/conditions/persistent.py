"""Persistent condition that requires a child to remain True for a duration."""

from __future__ import annotations

from typing import Any, Optional

from ..tick_snapshot import TickSnapshot
from .base import BaseCondition, ScenarioResult


class PersistentCondition(BaseCondition):
    """Requires child condition to remain True for *duration* consecutive seconds.

    Evaluation rules:

    - While the child returns ``passed=True``, a timer accumulates.
    - When the timer reaches *duration*, this condition returns ``passed=True``.
    - If the child returns ``None`` or ``passed=False``, the timer resets.

    Args:
        condition: The child condition to monitor.
        duration: Minimum consecutive seconds the child must pass before
            this condition passes.

    Raises:
        ValueError: If *duration* is not positive.
    """

    def __init__(
        self, condition: BaseCondition, duration: float, *, label: str | None = None
    ) -> None:
        if duration <= 0:
            raise ValueError("duration must be positive")
        super().__init__(label=label if label is not None else condition.label)
        self._condition = condition
        self._duration = duration
        self._true_start: Optional[float] = None

    def get_details(self) -> dict[str, Any]:
        return {
            "wrapper": "persistent",
            "required_duration_seconds": self._duration,
            "child": self._condition.to_summary_dict(),
        }

    def check(self, snapshot: TickSnapshot) -> Optional[ScenarioResult]:
        """Return a pass result when the child has been passing for *duration* seconds.

        Args:
            snapshot: Immutable snapshot of the current tick state.

        Returns:
            :class:`ScenarioResult` with ``passed=True`` if the child has been
            continuously passing for at least *duration* seconds, ``None``
            otherwise.
        """
        result = self._condition.check(snapshot)

        if result is not None and result.passed:
            if self._true_start is None:
                self._true_start = snapshot.elapsed
            if snapshot.elapsed - self._true_start >= self._duration:
                return ScenarioResult(
                    passed=True,
                    message=(
                        f"Child condition passed continuously for"
                        f" {snapshot.elapsed - self._true_start:.2f}s"
                        f" (threshold: {self._duration}s)"
                    ),
                    elapsed_seconds=snapshot.elapsed,
                )
            return None

        # Child returned None or failed — reset timer.
        self._true_start = None
        return None
