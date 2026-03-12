"""Persistent condition that requires a child to remain True for a duration."""

from __future__ import annotations

from typing import TYPE_CHECKING, Optional

from .base import BaseCondition, ScenarioResult

if TYPE_CHECKING:
    import carla


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

    def __init__(self, condition: BaseCondition, duration: float) -> None:
        if duration <= 0:
            raise ValueError("duration must be positive")
        self._condition = condition
        self._duration = duration
        self._true_start: Optional[float] = None

    def check(self, world: "carla.World", elapsed: float) -> Optional[ScenarioResult]:
        """Return a pass result when the child has been passing for *duration* seconds.

        Args:
            world: The CARLA world instance.
            elapsed: Elapsed time in seconds since the scenario started.

        Returns:
            :class:`ScenarioResult` with ``passed=True`` if the child has been
            continuously passing for at least *duration* seconds, ``None``
            otherwise.
        """
        result = self._condition.check(world, elapsed)

        if result is not None and result.passed:
            if self._true_start is None:
                self._true_start = elapsed
            if elapsed - self._true_start >= self._duration:
                return ScenarioResult(
                    passed=True,
                    message=(
                        f"Child condition passed continuously for"
                        f" {elapsed - self._true_start:.2f}s"
                        f" (threshold: {self._duration}s)"
                    ),
                    elapsed_seconds=elapsed,
                )
            return None

        # Child returned None or failed — reset timer.
        self._true_start = None
        return None
