"""Waypoint condition for scenario evaluation."""

from __future__ import annotations

from enum import Enum, auto
from typing import TYPE_CHECKING, Any, Optional, Union

from ...entity_role import EntityRole
from ..base import ScenarioResult, find_actor_by_role_name
from .base import CompositionCondition

if TYPE_CHECKING:
    import carla


class WaypointCheckType(Enum):
    """Type of check to perform on ``waypoint.next(distance)`` results.

    Attributes:
        IS_EMPTY: True when ``waypoint.next(distance)`` returns an empty list,
            indicating the vehicle has no further waypoints ahead (e.g. near
            the end of a road or before a dead-end).
    """

    IS_EMPTY = auto()


class WaypointCondition(CompositionCondition):
    """Condition that evaluates ``waypoint.next(distance)`` for a given entity.

    Retrieves the CARLA waypoint closest to the entity's current location,
    calls ``waypoint.next(distance)``, and applies the specified check.

    Args:
        entity_name: The ``role_name`` attribute of the actor to evaluate.
        distance: Look-ahead distance (metres) passed to ``waypoint.next()``.
        check_type: The check to apply to the resulting waypoint list.
        label: Human-readable label for this condition.
    """

    def __init__(
        self,
        entity_name: Union[EntityRole, str],
        distance: float,
        check_type: WaypointCheckType = WaypointCheckType.IS_EMPTY,
        *,
        label: str,
    ) -> None:
        if distance <= 0:
            raise ValueError("distance must be positive")
        super().__init__(entity_name=entity_name, label=label)
        self._distance = distance
        self._check_type = check_type
        self._carla_map: Optional[carla.Map] = None

    def get_details(self) -> dict[str, Any]:
        details = super().get_details()
        details.update(
            {
                "distance": self._distance,
                "check_type": self._check_type.name,
            }
        )
        return details

    def _check(self, world: "carla.World", elapsed: float) -> Optional[ScenarioResult]:
        """Return a pass result if the waypoint check is satisfied.

        Args:
            world: The CARLA world instance.
            elapsed: Elapsed time in seconds since the scenario started.

        Returns:
            :class:`ScenarioResult` with ``passed=True`` if the check is met,
            ``None`` otherwise.
        """
        assert self._entity_name is not None
        entity = find_actor_by_role_name(world, self._entity_name)
        if entity is None:
            return None

        if self._carla_map is None:
            self._carla_map = world.get_map()

        waypoint = self._carla_map.get_waypoint(entity.get_location())
        if waypoint is None:
            return None

        next_waypoints = waypoint.next(self._distance)

        if self._check_type == WaypointCheckType.IS_EMPTY:
            if not next_waypoints:
                return ScenarioResult(
                    passed=True,
                    message=(
                        f"Entity '{self._entity_name}' waypoint.next({self._distance})"
                        f" is empty at road={waypoint.road_id}"
                        f" lane={waypoint.lane_id} s={waypoint.s:.2f}"
                    ),
                    elapsed_seconds=elapsed,
                )

        return None
