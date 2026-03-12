"""Fail condition that triggers when a named actor does not exist."""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Optional

from .base import BaseCondition, ScenarioResult, find_actor_by_role_name

if TYPE_CHECKING:
    import carla

logger = logging.getLogger(__name__)


class EntityExistenceCondition(BaseCondition):
    """Fail condition that triggers whenever a named actor is absent from the world.

    On every call to :meth:`check`, the condition searches for an actor with
    the given ``role_name``.  If the actor is **not found** — whether it has
    never spawned or has since been destroyed — the condition returns a failure
    result immediately.

    This condition is typically registered as a **fail condition** so that the
    scenario terminates immediately when the ego (or any critical actor)
    is missing.

    Args:
        entity_name: The ``role_name`` attribute of the actor to monitor.
    """

    def __init__(self, entity_name: str) -> None:
        self._entity_name = entity_name

    def check(self, world: "carla.World", elapsed: float) -> Optional[ScenarioResult]:
        """Return a fail result if the actor does not exist.

        The condition triggers immediately whenever the actor cannot be found,
        including before it has ever been spawned.

        Args:
            world: The CARLA world instance.
            elapsed: Elapsed time in seconds since the scenario started.

        Returns:
            :class:`ScenarioResult` with ``passed=False`` if the actor
            is not found, ``None`` otherwise.
        """
        actor = find_actor_by_role_name(world, self._entity_name)

        if actor is not None:
            return None

        msg = (
            f"Actor '{self._entity_name}' not found in the world " f"at {elapsed:.2f}s"
        )
        logger.error(msg)

        return ScenarioResult(
            passed=False,
            message=msg,
            elapsed_seconds=elapsed,
        )
