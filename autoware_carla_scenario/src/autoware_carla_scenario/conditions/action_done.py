"""Condition that fires once a referenced action has executed."""

from __future__ import annotations

from typing import TYPE_CHECKING, Optional

from .base import BaseCondition, ScenarioResult

if TYPE_CHECKING:
    import carla

    from ..actions.base import BaseAction


class ActionDoneCondition(BaseCondition):
    """Satisfied once the referenced action has fired (``action.done``).

    This is the sequencing primitive used to gate one action on another: pass a
    previously constructed :class:`~autoware_carla_scenario.BaseAction` and use
    the resulting condition as the ``condition=`` trigger of a later action so
    that the later action only arms after the earlier one has executed.

    Note that ``done`` becomes true when the action *issues* its command (e.g.
    a :class:`TurnAction` sets the TrafficManager route), not when the resulting
    manoeuvre has physically finished. Combine with an observed condition (a
    lane-position or elapsed-time condition) when physical completion matters.

    Args:
        action: The action to observe. Only its ``done`` / ``label`` attributes
            are used, so any object exposing them is accepted.
        label: Human-readable identifier for this condition.
    """

    def __init__(self, action: "BaseAction", *, label: str) -> None:
        super().__init__(label=label)
        self._action = action

    def check(self, world: "carla.World", elapsed: float) -> Optional[ScenarioResult]:
        if self._action.done:
            return ScenarioResult(
                passed=True,
                message=f"action '{self._action.label}' has fired",
                elapsed_seconds=elapsed,
            )
        return None

    def get_details(self) -> dict[str, object]:
        return {"waits_for_action": self._action.label}
