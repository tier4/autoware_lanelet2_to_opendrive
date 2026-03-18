"""Fail condition that triggers when a named actor does not exist."""

from __future__ import annotations

import logging
from typing import Any, Optional, Union

from ..entity_role import EntityRole
from ..tick_snapshot import TickSnapshot
from .base import BaseCondition, ScenarioResult, find_actor_by_role_name

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
            Accepts both :class:`EntityRole` and plain ``str``.
    """

    def __init__(self, entity_name: Union[EntityRole, str], *, label: str) -> None:
        super().__init__(label=label)
        self._entity_name = entity_name

    def get_details(self) -> dict[str, Any]:
        return {"entity_name": str(self._entity_name)}

    def check(self, snapshot: TickSnapshot) -> Optional[ScenarioResult]:
        """Return a fail result if the actor does not exist.

        The condition triggers immediately whenever the actor cannot be found,
        including before it has ever been spawned.

        Args:
            snapshot: Immutable snapshot of the current tick state.

        Returns:
            :class:`ScenarioResult` with ``passed=False`` if the actor
            is not found, ``None`` otherwise.
        """
        actor = find_actor_by_role_name(snapshot.world, self._entity_name)

        if actor is not None:
            return None

        msg = (
            f"Actor '{self._entity_name}' not found in the world "
            f"at {snapshot.elapsed:.2f}s"
        )
        logger.error(msg)

        return ScenarioResult(
            passed=False,
            message=msg,
            elapsed_seconds=snapshot.elapsed,
        )
