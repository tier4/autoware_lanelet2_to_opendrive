"""Relative-position condition: is an actor at a longitudinal gap to another?

Companion to
:class:`~autoware_carla_scenario.actions.relative_position.RelativePositionAction`:
where the action *drives* an actor toward a moving longitudinal gap, this
condition *checks* whether the gap has been reached (within a tolerance), so the
goal can be registered as a scenario pass condition.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any, Optional, Union

from ...entity_role import EntityRole
from ...kinematics import Vector3
from ..base import ScenarioResult, find_actor_in_list
from .base import CompositionCondition

if TYPE_CHECKING:
    import carla

_NEAR_ZERO_THRESHOLD = 1e-6
"""Magnitude below which a reference forward vector is considered degenerate."""


class RelativePositionCondition(CompositionCondition):
    """Pass when an actor is within *tolerance* of a longitudinal gap goal.

    The signed longitudinal gap is measured along the reference actor's forward
    direction: ``gap = (entity_pos - reference_pos) · reference_forward``. A
    positive ``target_gap`` means *ahead of* the reference, negative *behind*.

    Args:
        entity_name: ``role_name`` of the controlled/observed vehicle.
        reference_entity_name: ``role_name`` of the reference vehicle.
        target_gap: Desired signed longitudinal gap in metres.
        tolerance: Half-width (metres) of the acceptance band around
            ``target_gap``.
        label: Human-readable identifier.
    """

    def __init__(
        self,
        entity_name: Union[EntityRole, str],
        reference_entity_name: Union[EntityRole, str],
        target_gap: float,
        *,
        tolerance: float = 2.0,
        label: str = "relative_position",
    ) -> None:
        super().__init__(entity_name=entity_name, label=label)
        self._reference_entity_name = reference_entity_name
        self._target_gap = target_gap
        self._tolerance = tolerance

    def get_details(self) -> dict[str, Any]:
        details = super().get_details()
        details.update(
            {
                "reference_entity_name": str(self._reference_entity_name),
                "target_gap": self._target_gap,
                "tolerance": self._tolerance,
            }
        )
        return details

    def _gap(self, entity: "carla.Actor", reference: "carla.Actor") -> Optional[float]:
        fwd_carla = reference.get_transform().get_forward_vector()
        fwd = Vector3(fwd_carla.x, fwd_carla.y, 0.0)
        fwd_mag = fwd.magnitude()
        if fwd_mag < _NEAR_ZERO_THRESHOLD:
            return None
        fwd_unit = fwd / fwd_mag
        ent_loc = entity.get_location()
        ref_loc = reference.get_location()
        displacement = Vector3(ent_loc.x - ref_loc.x, ent_loc.y - ref_loc.y, 0.0)
        return displacement.dot(fwd_unit)

    def _check(self, world: "carla.World", elapsed: float) -> Optional[ScenarioResult]:
        assert self._entity_name is not None
        actors: list[carla.Actor] = world.get_actors()
        entity = find_actor_in_list(actors, self._entity_name)
        reference = find_actor_in_list(actors, self._reference_entity_name)
        if entity is None or reference is None:
            return None

        gap = self._gap(entity, reference)
        if gap is None:
            return None

        if abs(gap - self._target_gap) <= self._tolerance:
            return ScenarioResult(
                passed=True,
                message=(
                    f"{self._entity_name} reached gap {gap:.1f} m "
                    f"(target {self._target_gap:.1f} m) to "
                    f"{self._reference_entity_name}"
                ),
                elapsed_seconds=elapsed,
            )
        return None
