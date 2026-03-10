"""Standstill scenario pass condition."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, Optional

from .base import BaseCondition, ScenarioResult

if TYPE_CHECKING:
    import carla


class StandstillCondition(BaseCondition):
    """Pass condition that triggers when an entity remains nearly stopped.

    The condition is met when the entity's speed stays below
    ``speed_threshold`` for at least ``duration`` consecutive seconds.

    Args:
        entity_name: The ``role_name`` attribute of the actor to track.
        duration: Minimum consecutive seconds the entity must be nearly
            stopped before the condition passes.
        speed_threshold: Maximum speed (m/s) considered as standstill.
            Defaults to 0.1 m/s.
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
        self._entity_name = entity_name
        self._duration = duration
        self._speed_threshold = speed_threshold
        self._standstill_start: Optional[float] = None

    def check(self, world: "carla.World", elapsed: float) -> Optional[ScenarioResult]:
        """Return a pass result if the entity has been standing still long enough.

        Args:
            world: The CARLA world instance.
            elapsed: Elapsed time in seconds since the scenario started.

        Returns:
            :class:`ScenarioResult` with ``passed=True`` if the entity has been
            nearly stopped for at least ``duration`` seconds, ``None`` otherwise.
        """
        actors = world.get_actors()
        entity = next(
            (a for a in actors if a.attributes.get("role_name") == self._entity_name),
            None,
        )
        if entity is None:
            return None

        velocity: carla.Vector3D = entity.get_velocity()
        speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)

        if speed <= self._speed_threshold:
            if self._standstill_start is None:
                self._standstill_start = elapsed
            standstill_elapsed = elapsed - self._standstill_start
            if standstill_elapsed >= self._duration:
                return ScenarioResult(
                    passed=True,
                    message=(
                        f"Entity '{self._entity_name}' has been standing still"
                        f" for {standstill_elapsed:.2f}s (threshold: {self._duration}s)"
                    ),
                    elapsed_seconds=elapsed,
                )
        else:
            self._standstill_start = None

        return None
