"""Stop line pose computation from Lanelet2 map data."""

from __future__ import annotations

import logging

import lanelet2

from ..utils.stop_line import get_stop_line_linestrings
from .map_manager import MapManager
from .poses import Lanelet2Pose

logger = logging.getLogger(__name__)


def get_stop_line_poses(lanelet_id: int) -> list[Lanelet2Pose]:
    """Return Lanelet2Pose list for stop lines associated with the given lanelet.

    Uses :func:`~autoware_carla_scenario.utils.stop_line.get_stop_line_linestrings`
    to find stop line linestrings, then converts each centroid to a
    :class:`Lanelet2Pose` via arc-length projection.

    Requires :class:`MapManager` to be initialised.

    Args:
        lanelet_id: The Lanelet2 lanelet ID to search for stop lines.

    Returns:
        List of :class:`Lanelet2Pose` for each unique stop line found.
        Empty list if no stop lines are associated with the lanelet.

    Raises:
        ValueError: If the lanelet ID is not found in the map.
    """
    linestrings = get_stop_line_linestrings(lanelet_id)
    if not linestrings:
        return []

    mm = MapManager.get_instance()
    lanelet = mm.lanelet_map.laneletLayer[lanelet_id]

    poses: list[Lanelet2Pose] = []
    for ls in linestrings:
        points = list(ls)
        if not points:
            continue

        cx = sum(p.x for p in points) / len(points)
        cy = sum(p.y for p in points) / len(points)

        pt = lanelet2.core.BasicPoint2d(cx, cy)
        arc = lanelet2.geometry.toArcCoordinates(lanelet2.geometry.to2D(lanelet), pt)

        poses.append(Lanelet2Pose(lanelet_id=lanelet_id, s=arc.length, t=0.0))

    return poses
