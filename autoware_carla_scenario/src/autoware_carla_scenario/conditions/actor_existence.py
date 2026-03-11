"""Fail condition that triggers when a named actor no longer exists."""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Optional

from .base import BaseCondition, ScenarioResult, find_actor_by_role_name

if TYPE_CHECKING:
    import carla

logger = logging.getLogger(__name__)


class ActorExistenceCondition(BaseCondition):
    """Fail condition that triggers when a named actor disappears from the world.

    On every call to :meth:`check`, the condition searches for an actor with
    the given ``role_name``.  If the actor is **not found**, the condition
    returns a failure result — the actor has been destroyed (e.g. fell through
    the map, collision, or server-side cleanup).

    This condition is typically registered as a **fail condition** so that the
    scenario terminates immediately when the ego (or any critical actor)
    despawns unexpectedly.

    Args:
        entity_name: The ``role_name`` attribute of the actor to monitor.
    """

    def __init__(self, entity_name: str) -> None:
        self._entity_name = entity_name
        self._was_alive = False

    def check(self, world: "carla.World", elapsed: float) -> Optional[ScenarioResult]:
        """Return a fail result if the actor no longer exists.

        The condition only triggers after the actor has been seen at least
        once (i.e. it was alive and then disappeared).  This avoids a false
        positive during the brief window before the actor is first spawned.

        Args:
            world: The CARLA world instance.
            elapsed: Elapsed time in seconds since the scenario started.

        Returns:
            :class:`ScenarioResult` with ``passed=False`` if the actor
            previously existed but is now gone, ``None`` otherwise.
        """
        actor = find_actor_by_role_name(world, self._entity_name)

        if actor is not None:
            self._was_alive = True
            return None

        if not self._was_alive:
            # Actor hasn't spawned yet — don't trigger
            return None

        msg = (
            f"Actor '{self._entity_name}' disappeared from the world "
            f"at {elapsed:.2f}s"
        )
        logger.error(msg)

        return ScenarioResult(
            passed=False,
            message=msg,
            elapsed_seconds=elapsed,
        )
