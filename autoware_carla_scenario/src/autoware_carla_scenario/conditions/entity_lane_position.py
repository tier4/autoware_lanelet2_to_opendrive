"""Entity-lane-position scenario pass condition."""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Optional

from ..coordinate.poses import CarlaWorldPose
from ..coordinate.transform import to_opendrive
from .base import BaseCondition, ScenarioResult, find_actor_by_role_name

if TYPE_CHECKING:
    import carla

logger = logging.getLogger(__name__)


class EntityLanePositionCondition(BaseCondition):
    """Pass condition that triggers when a named entity is on a specified OpenDRIVE road and lane.

    On every call to :meth:`check`, the entity's CARLA world position is converted
    to an :class:`OpenDrivePose` using the coordinate transformation system.  The
    condition triggers when the resulting ``road_id`` and ``lane_id`` match the
    specified values.

    When ``lane_id`` is ``None``, only the ``road_id`` is checked; the entity may
    be on any lane of that road.

    .. note::
        :class:`~autoware_carla_scenario.coordinate.map_manager.MapManager` must be
        initialised before the first call to :meth:`check`, since the conversion from
        CARLA world coordinates to OpenDRIVE coordinates requires map data.

    Args:
        entity_name: The ``role_name`` attribute of the actor to track.
        road_id: The OpenDRIVE road ID that the entity must be on.
        lane_id: The OpenDRIVE lane ID that the entity must be on.
            If ``None``, any lane on the specified road is accepted.
    """

    def __init__(
        self,
        entity_name: str,
        road_id: str,
        lane_id: Optional[int] = None,
    ) -> None:
        self._entity_name = entity_name
        self._road_id = road_id
        self._lane_id = lane_id

    def check(self, world: "carla.World", elapsed: float) -> Optional[ScenarioResult]:
        """Return a pass result if the named entity is on the specified road and lane.

        Args:
            world: The CARLA world instance.
            elapsed: Elapsed time in seconds since the scenario started.

        Returns:
            :class:`ScenarioResult` with ``passed=True`` if the entity is on the
            specified road and lane, ``None`` otherwise (entity not found or on a
            different road/lane).
        """
        entity = find_actor_by_role_name(world, self._entity_name)
        if entity is None:
            logger.debug(
                "EntityLanePositionCondition: actor '%s' not found",
                self._entity_name,
            )
            return None

        loc = entity.get_location()
        carla_pose = CarlaWorldPose(x=loc.x, y=loc.y, z=loc.z)
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

        lane_desc = "(any lane)" if self._lane_id is None else f"lane {self._lane_id}"
        msg = (
            f"Entity '{self._entity_name}' is on road '{self._road_id}'"
            f" {lane_desc} at {elapsed:.2f}s"
        )
        logger.info(
            "EntityLanePositionCondition: MATCHED — '%s' on road='%s' %s",
            self._entity_name,
            self._road_id,
            lane_desc,
        )

        return ScenarioResult(
            passed=True,
            message=msg,
            elapsed_seconds=elapsed,
        )
