"""OpenDRIVE objects definitions for crosswalk and other road objects."""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from typing import List, Optional, TYPE_CHECKING

import lanelet2
import lxml.etree as ET
import numpy as np

from ..util import extract_points
from .geometry import ParamPoly3

if TYPE_CHECKING:
    from .road import Road

logger = logging.getLogger(__name__)

_NEAREST_ROAD_THRESHOLD_M = (
    50.0  # Max distance (m) to associate a crosswalk with a road
)
_SAMPLE_POINTS_PER_GEOMETRY = 10  # Number of sample points per geometry segment


@dataclass
class CornerLocal:
    """Local corner point for an OpenDRIVE object outline.

    Represents a vertex in the object-local coordinate system where:
    - u: distance along the object's heading direction
    - v: distance perpendicular to the heading direction
    - z: vertical offset
    """

    u: float
    v: float
    z: float = 0.0

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("cornerLocal")
        elem.set("u", str(self.u))
        elem.set("v", str(self.v))
        elem.set("z", str(self.z))
        return elem


@dataclass
class CrosswalkObject:
    """OpenDRIVE object representing a crosswalk.

    Corresponds to <object type="crosswalk"> in the OpenDRIVE specification.
    Contains position on road reference line and polygon outline in local coordinates.
    """

    id: int
    name: str
    s: float  # s-coordinate on road reference line
    t: float  # t-coordinate (lateral offset from reference line)
    z_offset: float  # vertical offset from road surface
    hdg: float  # heading angle (radians) relative to road direction
    pitch: float = 0.0
    roll: float = 0.0
    orientation: str = "none"
    width: float = 0.0
    length: float = 0.0
    corners: List[CornerLocal] = field(default_factory=list)

    def to_xml(self) -> ET.Element:
        """Convert to XML element.

        Returns:
            <object type="crosswalk"> element with <outline><cornerLocal> children.
        """
        elem = ET.Element("object")
        elem.set("type", "crosswalk")
        elem.set("id", str(self.id))
        elem.set("name", self.name)
        elem.set("s", str(self.s))
        elem.set("t", str(self.t))
        elem.set("zOffset", str(self.z_offset))
        elem.set("hdg", str(self.hdg))
        elem.set("pitch", str(self.pitch))
        elem.set("roll", str(self.roll))
        elem.set("orientation", self.orientation)
        elem.set("width", str(self.width))
        elem.set("length", str(self.length))

        if self.corners:
            outline_elem = ET.SubElement(elem, "outline")
            for corner in self.corners:
                outline_elem.append(corner.to_xml())

        return elem

    @staticmethod
    def construct_from_crosswalk_lanelet(
        lanelet: lanelet2.core.Lanelet,
        road: Road,
        object_id: int,
    ) -> Optional[CrosswalkObject]:
        """Construct a CrosswalkObject from a crosswalk lanelet and its nearest road.

        Args:
            lanelet: Crosswalk lanelet with subtype="crosswalk"
            road: The nearest road to associate this crosswalk with
            object_id: ID for the resulting object (typically lanelet.id)

        Returns:
            CrosswalkObject if construction succeeds, None on failure.
        """
        try:
            # Extract 2D boundary points with coordinate offset applied
            left_pts = extract_points(lanelet.leftBound, dimensions=2)
            right_pts = extract_points(lanelet.rightBound, dimensions=2)

            if len(left_pts) < 2 or len(right_pts) < 2:
                logger.warning(
                    f"Crosswalk lanelet {lanelet.id} has insufficient boundary points, skipping"
                )
                return None

            # 4 vertices in order: leftBound start, leftBound end,
            #                      rightBound end, rightBound start
            p0 = left_pts[0]  # left-start
            p1 = left_pts[-1]  # left-end
            p2 = right_pts[-1]  # right-end
            p3 = right_pts[0]  # right-start

            # Compute centroid of the quadrilateral
            centroid = np.mean([p0, p1, p2, p3], axis=0)

            # Project centroid onto road reference line to get (s, t, road_hdg)
            projection = _project_point_onto_road(centroid, road)
            if projection is None:
                logger.warning(
                    f"Could not project crosswalk {lanelet.id} centroid onto road {road.id}"
                )
                return None
            s, t, road_hdg_at_s = projection

            # Compute absolute elevation of crosswalk from 3D boundary points
            left_pts_3d = extract_points(lanelet.leftBound, dimensions=3)
            right_pts_3d = extract_points(lanelet.rightBound, dimensions=3)
            crosswalk_absolute_z = float(
                np.mean([left_pts_3d[:, 2].mean(), right_pts_3d[:, 2].mean()])
            )

            # Evaluate road surface elevation at position s using the elevation profile
            road_elevation_at_s = road.get_elevation_at_s(s)

            # zOffset = height relative to road surface (should be ~0.0 for on-road crosswalks)
            z_offset = crosswalk_absolute_z - road_elevation_at_s

            # Main crosswalk direction: along leftBound (road-crossing direction)
            cw_dir = left_pts[-1] - left_pts[0]
            cw_dir_len = float(np.linalg.norm(cw_dir))
            if cw_dir_len < 1e-6:
                cw_dir = np.array([1.0, 0.0])
            else:
                cw_dir = cw_dir / cw_dir_len

            cw_angle = math.atan2(float(cw_dir[1]), float(cw_dir[0]))

            # hdg is the angle of crosswalk direction relative to road direction
            hdg = cw_angle - road_hdg_at_s
            # Normalize to (-pi, pi)
            hdg = (hdg + math.pi) % (2 * math.pi) - math.pi

            # Width: distance between leftBound start and rightBound start (road-parallel)
            width = float(np.linalg.norm(p3 - p0))

            # Length: average of left and right bound lengths (crossing distance)
            left_len = float(np.linalg.norm(p1 - p0))
            right_len = float(np.linalg.norm(p2 - p3))
            length = (left_len + right_len) / 2.0

            # Generate cornerLocal polygon vertices in object-local coordinates
            corners = _compute_corner_locals(centroid, cw_dir, [p0, p1, p2, p3])

            return CrosswalkObject(
                id=object_id,
                name=f"crosswalk_{object_id}",
                s=s,
                t=t,
                z_offset=z_offset,
                hdg=hdg,
                width=width,
                length=length,
                corners=corners,
            )

        except Exception as e:
            logger.warning(
                f"Failed to construct CrosswalkObject from lanelet {lanelet.id}: {e}"
            )
            return None


def _sample_road_points(road: Road) -> List[tuple]:
    """Sample world-space points along the road reference line.

    Args:
        road: Road whose plan_view geometries to sample.

    Returns:
        List of (world_x, world_y, s, heading) tuples.
    """
    samples: List[tuple] = []
    if road.plan_view is None:
        return samples

    for geom in road.plan_view.geometries:
        seg_length = geom.length
        if seg_length <= 0:
            continue

        n_pts = _SAMPLE_POINTS_PER_GEOMETRY
        cos_hdg = math.cos(geom.hdg)
        sin_hdg = math.sin(geom.hdg)

        for i in range(n_pts):
            p = seg_length * i / (n_pts - 1)  # arc-length parameter

            if isinstance(geom, ParamPoly3):
                # ParamPoly3 geometry: evaluate polynomial at arc-length p
                local_u = geom.aU + geom.bU * p + geom.cU * p**2 + geom.dU * p**3
                local_v = geom.aV + geom.bV * p + geom.cV * p**2 + geom.dV * p**3
                wx = geom.x + local_u * cos_hdg - local_v * sin_hdg
                wy = geom.y + local_u * sin_hdg + local_v * cos_hdg

                # Tangent for local heading
                du = geom.bU + 2 * geom.cU * p + 3 * geom.dU * p**2
                dv = geom.bV + 2 * geom.cV * p + 3 * geom.dV * p**2
                tx = du * cos_hdg - dv * sin_hdg
                ty = du * sin_hdg + dv * cos_hdg
                local_hdg = math.atan2(ty, tx)
            else:
                # Line or other simple geometry: straight-line along heading
                wx = geom.x + p * cos_hdg
                wy = geom.y + p * sin_hdg
                local_hdg = geom.hdg

            samples.append((wx, wy, geom.s + p, local_hdg))

    return samples


def _project_point_onto_road(
    point: np.ndarray,
    road: Road,
) -> Optional[tuple]:
    """Project a 2D point onto the road reference line.

    Finds the closest sample point on the road reference line and returns
    the corresponding (s, t, heading) values.

    Args:
        point: 2D point (x, y) to project
        road: Road to project onto

    Returns:
        (s, t, road_hdg) tuple, or None if the road has no geometry.
    """
    samples = _sample_road_points(road)
    if not samples:
        return None

    px, py = float(point[0]), float(point[1])
    best_dist = float("inf")
    best_s = 0.0
    best_t = 0.0
    best_hdg = 0.0

    for wx, wy, s, hdg in samples:
        dist = math.hypot(px - wx, py - wy)
        if dist < best_dist:
            best_dist = dist
            dx = px - wx
            dy = py - wy
            cos_h = math.cos(hdg)
            sin_h = math.sin(hdg)
            # Signed lateral offset: positive = left side of road
            t = -dx * sin_h + dy * cos_h
            best_s = s
            best_t = t
            best_hdg = hdg

    return best_s, best_t, best_hdg


def _compute_corner_locals(
    centroid: np.ndarray,
    cw_dir: np.ndarray,
    vertices: List[np.ndarray],
) -> List[CornerLocal]:
    """Compute cornerLocal coordinates for crosswalk polygon vertices.

    Transforms world-space vertices into the object-local coordinate system
    (origin = centroid, u-axis = cw_dir).

    Args:
        centroid: 2D centroid of the crosswalk polygon
        cw_dir: Unit vector for the crosswalk heading direction (2D)
        vertices: List of 4 world-space 2D vertex positions

    Returns:
        List of 5 CornerLocal points (first repeated at end to close the polygon).
    """
    # Perpendicular direction (right-hand rule: rotate cw_dir 90° clockwise)
    perp_dir = np.array([-cw_dir[1], cw_dir[0]])

    corners: List[CornerLocal] = []
    for v in vertices:
        delta = v - centroid
        u = float(np.dot(delta, cw_dir))
        vv = float(np.dot(delta, perp_dir))
        corners.append(CornerLocal(u=u, v=vv, z=0.0))

    # Close the polygon by repeating the first corner
    if corners:
        corners.append(corners[0])

    return corners


@dataclass
class StopLineObject:
    """OpenDRIVE object representing a stop line.

    Corresponds to <object type="stopLine"> in OpenDRIVE specification.
    """

    id: int
    name: str
    s: float  # s-coordinate on road reference line
    t: float  # t-coordinate (lateral offset from reference line)
    z_offset: float  # vertical offset from road surface
    hdg: float  # heading angle (radians) relative to road direction
    pitch: float = 0.0
    roll: float = 0.0
    orientation: str = "none"
    width: float = 0.0  # stop_line length (the transversal distance crossing the lanes)
    length: float = 0.0  # thickness (usually 0.0)

    def to_xml(self) -> ET.Element:
        """Convert to XML element.

        Returns:
            <object type="stopLine"> element.
        """
        elem = ET.Element("object")
        elem.set("type", "stopLine")
        elem.set("id", str(self.id))
        elem.set("name", self.name)
        elem.set("s", str(self.s))
        elem.set("t", str(self.t))
        elem.set("zOffset", str(self.z_offset))
        elem.set("hdg", str(self.hdg))
        elem.set("pitch", str(self.pitch))
        elem.set("roll", str(self.roll))
        elem.set("orientation", self.orientation)
        elem.set("width", str(self.width))
        elem.set("length", str(self.length))
        return elem

    @staticmethod
    def construct_from_linestring(
        linestring: lanelet2.core.LineString3d,
        road: "Road",
        object_id: int,
    ) -> Optional["StopLineObject"]:
        """Construct a StopLineObject from a stop_line linestring and its nearest road.

        Args:
            linestring: LineString with type="stop_line"
            road: The nearest road to associate this stop line with
            object_id: ID for the resulting object (typically linestring.id)

        Returns:
            StopLineObject if construction succeeds, None on failure.
        """
        try:
            pts = extract_points(linestring, dimensions=2)
            if len(pts) < 2:
                logger.warning(
                    f"Stop line linestring {linestring.id} has fewer than 2 points, skipping"
                )
                return None

            # Centroid of all points
            centroid = np.mean(pts, axis=0)

            projection = _project_point_onto_road(centroid, road)
            if projection is None:
                logger.warning(
                    f"Could not project stop line {linestring.id} centroid onto road {road.id}"
                )
                return None
            s, t, road_hdg_at_s = projection

            # Compute z_offset from 3D points vs road elevation
            pts_3d = extract_points(linestring, dimensions=3)
            stop_line_absolute_z = float(np.mean(pts_3d[:, 2]))
            road_elevation_at_s = road.get_elevation_at_s(s)
            z_offset = stop_line_absolute_z - road_elevation_at_s

            # Heading: direction of the stop line (from first to last point)
            direction = pts[-1] - pts[0]
            stop_line_angle = math.atan2(float(direction[1]), float(direction[0]))
            hdg = (stop_line_angle - road_hdg_at_s + math.pi) % (2 * math.pi) - math.pi

            # Width = length of the stop line; length (thickness) = 0
            width = float(np.linalg.norm(pts[-1] - pts[0]))

            return StopLineObject(
                id=object_id,
                name=f"stop_line_{object_id}",
                s=s,
                t=t,
                z_offset=z_offset,
                hdg=hdg,
                width=width,
                length=0.0,
            )

        except Exception as e:
            logger.warning(
                f"Failed to construct StopLineObject from linestring {linestring.id}: {e}"
            )
            return None


def find_nearest_road_for_linestring(
    linestring: lanelet2.core.LineString3d,
    all_roads: List["Road"],
    threshold_m: float = _NEAREST_ROAD_THRESHOLD_M,
) -> Optional["Road"]:
    """Find the nearest road to a linestring's centroid.

    Args:
        linestring: LineString to find the nearest road for
        all_roads: List of all candidate roads
        threshold_m: Maximum allowed distance in meters

    Returns:
        Nearest Road within threshold_m, or None if no road is close enough.
    """
    pts = extract_points(linestring, dimensions=2)
    if len(pts) == 0:
        return None

    centroid = np.mean(pts, axis=0)

    best_road: Optional[Road] = None
    best_dist = float("inf")

    for road in all_roads:
        if road.plan_view is None:
            continue
        for wx, wy, _, _ in _sample_road_points(road):
            dist = math.hypot(float(centroid[0]) - wx, float(centroid[1]) - wy)
            if dist < best_dist:
                best_dist = dist
                best_road = road

    if best_dist > threshold_m:
        logger.warning(
            f"Stop line linestring {linestring.id}: nearest road is {best_dist:.1f}m away "
            f"(threshold={threshold_m}m), skipping"
        )
        return None

    return best_road


def find_nearest_road(
    lanelet: lanelet2.core.Lanelet,
    all_roads: List[Road],
    threshold_m: float = _NEAREST_ROAD_THRESHOLD_M,
) -> Optional[Road]:
    """Find the nearest road to a crosswalk lanelet's centroid.

    Args:
        lanelet: Crosswalk lanelet to find the nearest road for
        all_roads: List of all candidate roads
        threshold_m: Maximum allowed distance in meters

    Returns:
        Nearest Road within threshold_m, or None if no road is close enough.
    """
    left_pts = extract_points(lanelet.leftBound, dimensions=2)
    right_pts = extract_points(lanelet.rightBound, dimensions=2)

    if len(left_pts) == 0 or len(right_pts) == 0:
        return None

    p0 = left_pts[0]
    p1 = left_pts[-1]
    p2 = right_pts[-1]
    p3 = right_pts[0]
    centroid = np.mean([p0, p1, p2, p3], axis=0)

    best_road: Optional[Road] = None
    best_dist = float("inf")

    for road in all_roads:
        if road.plan_view is None:
            continue
        for wx, wy, _, _ in _sample_road_points(road):
            dist = math.hypot(float(centroid[0]) - wx, float(centroid[1]) - wy)
            if dist < best_dist:
                best_dist = dist
                best_road = road

    if best_dist > threshold_m:
        logger.warning(
            f"Crosswalk lanelet {lanelet.id}: nearest road is {best_dist:.1f}m away "
            f"(threshold={threshold_m}m), skipping"
        )
        return None

    return best_road
