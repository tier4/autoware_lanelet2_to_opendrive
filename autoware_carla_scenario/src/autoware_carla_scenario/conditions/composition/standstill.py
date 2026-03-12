"""Standstill condition composed from PersistentCondition + SpeedCondition."""

from __future__ import annotations

from typing import TYPE_CHECKING, Optional

from ..base import ScenarioResult
from ..comparison import ComparisonRule
from ..persistent import PersistentCondition
from .base import CompositionCondition
from .speed import SpeedCondition

if TYPE_CHECKING:
    import carla


class StandstillCondition(CompositionCondition):
    """Pass when entity speed stays below threshold for *duration* seconds.

    Composition of ``PersistentCondition(SpeedCondition(LESS_THAN_OR_EQUAL))``.

    An :class:`EntityExistenceCondition` guard ensures the entity is present
    before the inner condition is evaluated.

    Args:
        entity_name: The ``role_name`` attribute of the actor to track.
        duration: Minimum consecutive seconds the entity must be nearly
            stopped before the condition passes.
        speed_threshold: Maximum speed (m/s) considered as standstill.
            Defaults to 0.1 m/s.

    Raises:
        ValueError: If *duration* is not positive or *speed_threshold* is negative.
    """

    def __init__(
        self,
        entity_name: str,
        duration: float,
        speed_threshold: float = 0.1,
    ) -> None:
        if duration <= 0:
            raise ValueError("duration must be positive")
        if speed_threshold < 0:
            raise ValueError("speed_threshold must be non-negative")

        speed_cond = SpeedCondition(
            entity_name=entity_name,
            value=speed_threshold,
            rule=ComparisonRule.LESS_THAN_OR_EQUAL,
        )
        child = PersistentCondition(speed_cond, duration=duration)

        super().__init__(child=child, entity_name=entity_name)

        self._entity_name = entity_name
        self._duration = duration
        self._speed_threshold = speed_threshold

    def _check(self, world: "carla.World", elapsed: float) -> Optional[ScenarioResult]:
        """Return a pass result once the child :class:`PersistentCondition` fires.

        The entity is guaranteed to exist by the
        :class:`EntityExistenceCondition` guard, and the child
        ``PersistentCondition(SpeedCondition)`` has already passed when
        this method is called.

        Args:
            world: The CARLA world instance.
            elapsed: Elapsed time in seconds since the scenario started.

        Returns:
            :class:`ScenarioResult` with ``passed=True``.
        """
        return ScenarioResult(
            passed=True,
            message=(
                f"Entity '{self._entity_name}' has been standing still"
                f" for {self._duration:.2f}s"
                f" (threshold: {self._duration}s)"
            ),
            elapsed_seconds=elapsed,
        )
