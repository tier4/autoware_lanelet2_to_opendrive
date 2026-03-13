"""Entity-lane-position scenario pass condition."""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Optional, Union

from ...coordinate.poses import CarlaWorldPose
from ...coordinate.transform import project_onto_road, to_opendrive
from ...entity_role import EntityRole
from ..base import ScenarioResult, find_actor_by_role_name
from ..comparison import ScalarComparisonRule
from .base import CompositionCondition

if TYPE_CHECKING:
    import carla

logger = logging.getLogger(__name__)

_VALID_FIELDS = frozenset({"s", "t"})


class EntityLanePositionCondition(CompositionCondition):
    """Pass condition that triggers when a named entity is on a specified OpenDRIVE road and lane.

    On every call to :meth:`check`, the entity's CARLA world position is converted
    to an :class:`OpenDrivePose` using the coordinate transformation system.  The
    condition triggers when the resulting ``road_id`` and ``lane_id`` match the
    specified values, and all optional comparison *rules* on ``s`` / ``t`` are
    satisfied.

    When ``lane_id`` is ``None``, only the ``road_id`` is checked; the entity may
    be on any lane of that road.  When comparison *rules* are also specified,
    :func:`project_onto_road` is used to obtain accurate ``s``/``t`` values on
    the confirmed road.

    An :class:`EntityExistenceCondition` guard ensures the entity is present
    before the position check runs.

    .. note::
        :class:`~autoware_carla_scenario.coordinate.map_manager.MapManager` must be
        initialised before the first call to :meth:`check`, since the conversion from
        CARLA world coordinates to OpenDRIVE coordinates requires map data.

    Args:
        entity_name: The ``role_name`` attribute of the actor to track.
        road_id: The OpenDRIVE road ID that the entity must be on.
        lane_id: The OpenDRIVE lane ID that the entity must be on.
            If ``None``, any lane on the specified road is accepted.
        rules: Optional comparison rules applied to the ``s`` and/or ``t``
            coordinates of the resolved :class:`OpenDrivePose`.

    Raises:
        ValueError: If any rule has a ``field`` other than ``'s'`` or ``'t'``.
    """

    def __init__(
        self,
        entity_name: Union[EntityRole, str],
        road_id: str,
        lane_id: Optional[int] = None,
        rules: Optional[list[ScalarComparisonRule]] = None,
    ) -> None:
        super().__init__(entity_name=entity_name)
        self._road_id = road_id
        self._lane_id = lane_id
        self._rules: list[ScalarComparisonRule] = rules or []

        for rule in self._rules:
            if rule.field not in _VALID_FIELDS:
                raise ValueError(
                    f"ScalarComparisonRule field must be 's' or 't', got '{rule.field}'"
                )

    def _check(self, world: "carla.World", elapsed: float) -> Optional[ScenarioResult]:
        """Return a pass result if the named entity is on the specified road and lane.

        The entity is guaranteed to exist by the
        :class:`EntityExistenceCondition` guard.

        Args:
            world: The CARLA world instance.
            elapsed: Elapsed time in seconds since the scenario started.

        Returns:
            :class:`ScenarioResult` with ``passed=True`` if the entity is on the
            specified road and lane and all rules are satisfied, ``None`` otherwise.
        """
        assert self._entity_name is not None
        entity = find_actor_by_role_name(world, self._entity_name)
        if entity is None:
            return None

        loc = entity.get_location()
        carla_pose = CarlaWorldPose(x=loc.x, y=loc.y, z=loc.z)

        # Always use to_opendrive() first — it finds the nearest road and
        # therefore acts as the authoritative "is the entity on this road?" check.
        od_pose = to_opendrive(carla_pose)

        if od_pose.road_id != self._road_id:
            logger.debug(
                "EntityLanePositionCondition: '%s' on road='%s' lane=%d "
                "(want road='%s') at (%.1f, %.1f, %.1f) t=%.2fs",
                self._entity_name,
                od_pose.road_id,
                od_pose.lane_id,
                self._road_id,
                loc.x,
                loc.y,
                loc.z,
                elapsed,
            )
            return None

        if self._lane_id is not None and od_pose.lane_id != self._lane_id:
            logger.debug(
                "EntityLanePositionCondition: '%s' on road='%s' lane=%d "
                "(want lane=%d) at t=%.2fs",
                self._entity_name,
                od_pose.road_id,
                od_pose.lane_id,
                self._lane_id,
                elapsed,
            )
            return None

        # When lane_id is None and we have rules, use project_onto_road() for
        # accurate s/t on the specific road (to_opendrive() may pick a slightly
        # different nearest point when roads are close together).
        if self._lane_id is None and self._rules:
            od_pose = project_onto_road(carla_pose, self._road_id)

        # Evaluate s/t comparison rules.
        field_values = {"s": od_pose.s, "t": od_pose.t}
        for rule in self._rules:
            actual = field_values[rule.field]
            if not rule.satisfied(actual):
                logger.debug(
                    "EntityLanePositionCondition: '%s' rule %s %s %.3f "
                    "not satisfied (actual %.3f) at t=%.2fs",
                    self._entity_name,
                    rule.field,
                    rule.rule.name,
                    rule.value,
                    actual,
                    elapsed,
                )
                return None

        lane_desc = "(any lane)" if self._lane_id is None else f"lane {self._lane_id}"
        msg = (
            f"Entity '{self._entity_name}' is on road '{self._road_id}'"
            f" {lane_desc} (s={od_pose.s:.2f}, t={od_pose.t:.2f}) at {elapsed:.2f}s"
        )
        logger.info(
            "EntityLanePositionCondition: MATCHED — '%s' on road='%s' %s"
            " (s=%.2f, t=%.2f)",
            self._entity_name,
            self._road_id,
            lane_desc,
            od_pose.s,
            od_pose.t,
        )

        return ScenarioResult(
            passed=True,
            message=msg,
            elapsed_seconds=elapsed,
        )
