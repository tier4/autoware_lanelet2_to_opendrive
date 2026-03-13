"""Collision-based scenario fail condition using CARLA sensor.other.collision."""

from __future__ import annotations

import math
import threading
from typing import TYPE_CHECKING, Any, Optional

from ..constants import EGO_ROLE_NAME
from .base import BaseCondition, ScenarioResult

if TYPE_CHECKING:
    import carla


class CollisionCondition(BaseCondition):
    """Fail condition that triggers when the ego vehicle collides with any actor.

    Uses the CARLA ``sensor.other.collision`` sensor attached to the ego vehicle
    (identified by ``role_name == EGO_ROLE_NAME``).  The sensor is attached lazily on the
    first call to :meth:`check` so that it works even when the ego vehicle is
    spawned after the condition is registered.

    Args:
        min_impulse: Minimum collision impulse magnitude (in N·s) required to
            trigger the condition.  Collisions below this threshold are ignored.
            Defaults to ``0.0`` (all collisions trigger).
    """

    # Minimum interval between ego vehicle search attempts (seconds).
    _ATTACH_RETRY_INTERVAL: float = 1.0

    def __init__(self, min_impulse: float = 0.0, *, label: str) -> None:
        super().__init__(label=label)
        self._min_impulse = min_impulse
        self._collision_elapsed: Optional[float] = None
        self._sensor: Optional["carla.Actor"] = None
        self._lock = threading.Lock()
        self._collided = False
        self._other_type_id: Optional[str] = None
        self._last_attach_attempt: float = -math.inf
        self._cached_result: Optional[ScenarioResult] = None

    def get_details(self) -> dict[str, Any]:
        details: dict[str, Any] = {"min_impulse": self._min_impulse}
        if self._collided:
            details["other_actor_type"] = self._other_type_id or "unknown"
            if self._collision_elapsed is not None:
                details["collision_elapsed_seconds"] = self._collision_elapsed
        return details

    def _on_collision(self, event: "carla.CollisionEvent") -> None:
        """Callback invoked by CARLA when a collision event occurs."""
        impulse = event.normal_impulse
        magnitude = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
        if magnitude < self._min_impulse:
            return
        with self._lock:
            if not self._collided:
                self._collided = True
                self._other_type_id = event.other_actor.type_id

    def _try_attach_sensor(self, world: "carla.World", elapsed: float) -> None:
        """Search for the ego vehicle and attach a collision sensor to it.

        Rate-limited by ``_ATTACH_RETRY_INTERVAL`` to avoid calling the
        expensive ``world.get_actors()`` API on every tick during startup.
        Returns without doing anything if the ego is not yet available.
        """
        if elapsed - self._last_attach_attempt < self._ATTACH_RETRY_INTERVAL:
            return
        self._last_attach_attempt = elapsed

        import carla as _carla

        actors = world.get_actors().filter("vehicle.*")
        ego = next(
            (a for a in actors if a.attributes.get("role_name") == str(EGO_ROLE_NAME)),
            None,
        )
        if ego is None:
            return

        blueprint_library = world.get_blueprint_library()
        sensor_bp = blueprint_library.find("sensor.other.collision")
        self._sensor = world.spawn_actor(sensor_bp, _carla.Transform(), attach_to=ego)
        self._sensor.listen(self._on_collision)

    def check(self, world: "carla.World", elapsed: float) -> Optional[ScenarioResult]:
        """Return a failure result if the ego vehicle has collided.

        Args:
            world: The CARLA world instance.
            elapsed: Elapsed time in seconds since the scenario started.

        Returns:
            ScenarioResult with passed=False if a collision occurred, None otherwise.
        """
        # Fast path: return cached result after first collision without acquiring the lock.
        if self._cached_result is not None:
            return self._cached_result

        if self._sensor is None:
            self._try_attach_sensor(world, elapsed)

        with self._lock:
            if self._collided:
                other = self._other_type_id or "unknown"
                self._collision_elapsed = elapsed
                self._cached_result = ScenarioResult(
                    passed=False,
                    message=f"Ego vehicle collided with '{other}' at {elapsed:.2f}s",
                    elapsed_seconds=elapsed,
                )
                # Stop the listener — no further callbacks are needed.
                if self._sensor is not None:
                    self._sensor.stop()
                return self._cached_result
        return None
