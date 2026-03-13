"""Base class for composition conditions."""

from __future__ import annotations

from abc import abstractmethod
from typing import TYPE_CHECKING, Any, Optional, Union

from ...entity_role import EntityRole
from ..base import BaseCondition, ScenarioResult
from ..entity_existence import EntityExistenceCondition

if TYPE_CHECKING:
    import carla


class CompositionCondition(BaseCondition):
    """Base class for conditions composed from other conditions.

    :meth:`check` evaluates guards and prerequisites in order:

    1. **Entity existence guard** — when *entity_existence* is provided, the
       entity must be present in the world.  If the entity is absent the
       condition short-circuits and returns ``None``.
    2. **Child condition** — when *child* is provided it must have fired
       (returned ``ScenarioResult(passed=True)``).  If the child has not yet
       fired the condition short-circuits and returns ``None``.
    3. **Subclass logic** — :meth:`_check` is called only after both guards
       pass.

    Leaf composition conditions (e.g. :class:`SpeedCondition`) that have no
    child or entity guard pass ``None`` for both and implement all logic in
    :meth:`_check`.

    Args:
        child: An optional child condition that must be satisfied before
            :meth:`_check` is evaluated.  Typically an
            :class:`AndCondition` or :class:`OrCondition` combining
            multiple sub-conditions.
        entity_name: When provided, an :class:`EntityExistenceCondition`
            is constructed internally and used as a guard.  :meth:`check`
            returns ``None`` immediately if the entity is absent.
            Accepts both :class:`EntityRole` and plain ``str``.
    """

    def __init__(
        self,
        child: BaseCondition | None = None,
        entity_name: Union[EntityRole, str] | None = None,
        *,
        label: str,
    ) -> None:
        super().__init__(label=label)
        self._child = child
        self._entity_name = entity_name
        self._entity_existence: EntityExistenceCondition | None = (
            EntityExistenceCondition(entity_name, label=f"{label}_entity_exists")
            if entity_name is not None
            else None
        )

    def get_details(self) -> dict[str, Any]:
        details: dict[str, Any] = {}
        if self._entity_name is not None:
            details["entity_name"] = str(self._entity_name)
        if self._child is not None:
            details["child"] = {
                "condition_type": type(self._child).__name__,
                "label": self._child.label,
                **self._child.get_details(),
            }
        return details

    def check(self, world: "carla.World", elapsed: float) -> Optional[ScenarioResult]:
        """Evaluate entity existence, then child, then delegate to :meth:`_check`.

        Returns ``None`` immediately when the entity is absent or the
        child exists but has not yet fired.
        """
        if self._entity_existence is not None:
            existence_result = self._entity_existence.check(world, elapsed)
            if existence_result is not None:
                # Entity is absent — cannot evaluate further.
                return None

        if self._child is not None:
            result = self._child.check(world, elapsed)
            if result is None or not result.passed:
                return None

        return self._check(world, elapsed)

    @abstractmethod
    def _check(self, world: "carla.World", elapsed: float) -> Optional[ScenarioResult]:
        """Subclass-specific check logic, called after all guards pass."""
        ...
