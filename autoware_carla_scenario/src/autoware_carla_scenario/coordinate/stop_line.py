"""Stop line pose computation from Lanelet2 map data."""

from __future__ import annotations

import logging

import lanelet2

from ..utils.stop_line import (
    get_stop_line_linestrings,
    get_stop_line_linestrings_with_following,
)
from .map_manager import MapManager
from .poses import Lanelet2Pose

logger = logging.getLogger(__name__)


def _linestring_to_pose(ls: object, lanelet_id: int) -> Lanelet2Pose | None:
    """Project a linestring centroid onto a lanelet centerline."""
    mm = MapManager.get_instance()
    lanelet = mm.lanelet_map.laneletLayer[lanelet_id]

    points = list(ls)  # type: ignore[call-overload]
    if not points:
        return None

    cx = sum(p.x for p in points) / len(points)
    cy = sum(p.y for p in points) / len(points)

    pt = lanelet2.core.BasicPoint2d(cx, cy)
    centerline_2d = lanelet2.geometry.to2D(lanelet.centerline)
    arc = lanelet2.geometry.toArcCoordinates(centerline_2d, pt)

    return Lanelet2Pose(lanelet_id=lanelet_id, s=arc.length, t=0.0)


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

    poses: list[Lanelet2Pose] = []
    for ls in linestrings:
        pose = _linestring_to_pose(ls, lanelet_id)
        if pose is not None:
            poses.append(pose)

    return poses


def get_stop_line_poses_with_following(lanelet_id: int) -> list[Lanelet2Pose]:
    """Return stop line poses searching the lanelet and its successors.

    Searches the given lanelet first, then its immediate following lanelets
    via the routing graph. Each stop line centroid is projected onto the
    lanelet that owns the regulatory element.

    Args:
        lanelet_id: The starting Lanelet2 lanelet ID.

    Returns:
        List of :class:`Lanelet2Pose` for each unique stop line found.
        Empty list if no stop lines are found.

    Raises:
        ValueError: If the lanelet ID is not found in the map.
    """
    results = get_stop_line_linestrings_with_following(lanelet_id)
    if not results:
        return []

    poses: list[Lanelet2Pose] = []
    for owner_id, ls in results:
        pose = _linestring_to_pose(ls, owner_id)
        if pose is not None:
            poses.append(pose)

    return poses
