"""Entity-in-area scenario pass condition."""

from __future__ import annotations

from typing import TYPE_CHECKING, List, Optional, Sequence, Union

import cv2
import numpy as np

from ...coordinate.poses import AnyPose, CarlaWorldPose
from ...coordinate.transform import to_carla_world
from ...entity_role import EntityRole
from ..base import ScenarioResult, find_actor_by_role_name
from .base import CompositionCondition

if TYPE_CHECKING:
    import carla


def _point_in_polygon_2d(
    x: float, y: float, polygon: List[CarlaWorldPose], include_boundary: bool = True
) -> bool:
    """Test whether (x, y) is inside the polygon.

    Uses :func:`cv2.pointPolygonTest` for a robust containment check.
    Only the x-y plane is considered (z is ignored).

    Args:
        x: X coordinate of the point to test.
        y: Y coordinate of the point to test.
        polygon: Polygon vertices as a list of CarlaWorldPose.
        include_boundary: If True, points on the polygon boundary are considered
            inside. If False, only strictly interior points return True.

    Returns:
        True if the point is inside the polygon (and on the boundary when
        ``include_boundary`` is True), False otherwise.
    """
    contour = np.array([[p.x, p.y] for p in polygon], dtype=np.float32)
    result = cv2.pointPolygonTest(contour, (x, y), measureDist=False)
    return result >= 0 if include_boundary else result > 0


class EntityInAreaCondition(CompositionCondition):
    """Pass condition that triggers when a named entity is inside a polygon area.

    The polygon boundary is defined by a list of poses in any supported coordinate
    system (``Lanelet2Pose``, ``OpenDrivePose``, or ``CarlaWorldPose``).  On every
    call to :meth:`check`, the vertices are converted to ``CarlaWorldPose`` so that
    future relative-coordinate systems can be applied dynamically.

    Only the x-y (horizontal) plane is considered for the containment test;
    the z coordinate is ignored.

    An :class:`EntityExistenceCondition` guard ensures the entity is present
    before the containment check runs.

    Args:
        entity_name: The ``role_name`` attribute of the actor to track.
        polygon: List of poses defining the closed polygon boundary.
            Must contain at least 3 vertices.
        include_boundary: If True (default), an entity on the polygon boundary
            is considered inside the area. If False, only strictly interior
            positions trigger the condition.
    """

    def __init__(
        self,
        entity_name: Union[EntityRole, str],
        polygon: Sequence[AnyPose],
        include_boundary: bool = True,
        *,
        label: str,
    ) -> None:
        if len(polygon) < 3:
            raise ValueError("polygon must have at least 3 vertices")
        super().__init__(entity_name=entity_name, label=label)
        self._polygon: Sequence[AnyPose] = polygon
        self._include_boundary = include_boundary

    def _resolve_polygon(self) -> List[CarlaWorldPose]:
        """Convert all polygon vertices to CarlaWorldPose.

        This conversion is performed on every check so that coordinate
        transformations remain dynamic (e.g. when relative poses are used).
        """
        result: List[CarlaWorldPose] = []
        for pose in self._polygon:
            if isinstance(pose, CarlaWorldPose):
                result.append(pose)
            else:
                result.append(to_carla_world(pose))
        return result

    def _check(self, world: "carla.World", elapsed: float) -> Optional[ScenarioResult]:
        """Return a pass result if the named entity is inside the polygon area.

        The entity is guaranteed to exist by the
        :class:`EntityExistenceCondition` guard.

        Args:
            world: The CARLA world instance.
            elapsed: Elapsed time in seconds since the scenario started.

        Returns:
            :class:`ScenarioResult` with ``passed=True`` if the entity is inside
            the area, ``None`` otherwise.
        """
        carla_polygon = self._resolve_polygon()

        assert self._entity_name is not None
        entity = find_actor_by_role_name(world, self._entity_name)
        if entity is None:
            return None

        loc = entity.get_location()
        if _point_in_polygon_2d(loc.x, loc.y, carla_polygon, self._include_boundary):
            return ScenarioResult(
                passed=True,
                message=(
                    f"Entity '{self._entity_name}' is inside the designated area"
                    f" at {elapsed:.2f}s"
                ),
                elapsed_seconds=elapsed,
            )
        return None
