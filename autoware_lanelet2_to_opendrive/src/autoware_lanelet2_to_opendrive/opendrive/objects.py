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
from .geometry import Arc, GeometryBase, ParamPoly3, evaluate_plan_view_world

if TYPE_CHECKING:
    from .road import Road

logger = logging.getLogger(__name__)

_NEAREST_ROAD_THRESHOLD_M = (
    50.0  # Max distance (m) to associate a crosswalk with a road
)
_SAMPLE_POINTS_PER_GEOMETRY = 10  # Number of sample points per geometry segment


@dataclass(frozen=True)
class _ProjectionCandidate:
    s: float
    t: float
    hdg: float
    distance_sq: float


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
            elif isinstance(geom, Arc):
                # Arc geometry: constant curvature — sample along the curve.
                # Treating an arc as its start tangent skews the projected
                # (s, t) of objects placed on curved roads (#504).
                ax, ay = evaluate_plan_view_world(
                    geom.x, geom.y, geom.hdg, p, arc_curvature=geom.curvature
                )
                wx = float(ax)
                wy = float(ay)
                local_hdg = geom.hdg + geom.curvature * p
            else:
                # Line or other simple geometry: straight-line along heading
                wx = geom.x + p * cos_hdg
                wy = geom.y + p * sin_hdg
                local_hdg = geom.hdg

            samples.append((wx, wy, geom.s + p, local_hdg))

    return samples


def _geometry_epsilon() -> float:
    from ..config import DEFAULT_CONFIG

    return DEFAULT_CONFIG.geometry.epsilon


def _parampoly3_coefficients(
    geom: ParamPoly3,
) -> tuple[float, float, float, float, float, float, float, float]:
    return (
        geom.aU,
        geom.bU,
        geom.cU,
        geom.dU,
        geom.aV,
        geom.bV,
        geom.cV,
        geom.dV,
    )


def _evaluate_geometry_at(
    geom: GeometryBase,
    p: float,
) -> tuple[float, float, float]:
    """Evaluate a planView geometry and tangent heading at local parameter p."""
    p = max(0.0, min(float(p), float(getattr(geom, "length", 0.0))))

    if isinstance(geom, ParamPoly3):
        wx, wy = evaluate_plan_view_world(
            geom.x,
            geom.y,
            geom.hdg,
            p,
            param_poly3_coeffs=_parampoly3_coefficients(geom),
        )
        du = geom.bU + 2.0 * geom.cU * p + 3.0 * geom.dU * p * p
        dv = geom.bV + 2.0 * geom.cV * p + 3.0 * geom.dV * p * p
        if math.hypot(du, dv) <= _geometry_epsilon():
            hdg = geom.hdg
        else:
            hdg = geom.hdg + math.atan2(dv, du)
        return float(wx), float(wy), float(hdg)

    if isinstance(geom, Arc):
        wx, wy = evaluate_plan_view_world(
            geom.x, geom.y, geom.hdg, p, arc_curvature=geom.curvature
        )
        return float(wx), float(wy), float(geom.hdg + geom.curvature * p)

    wx, wy = evaluate_plan_view_world(geom.x, geom.y, geom.hdg, p)
    return float(wx), float(wy), float(geom.hdg)


def _projection_from_geometry_at(
    point: np.ndarray,
    geom: GeometryBase,
    p: float,
) -> _ProjectionCandidate:
    wx, wy, hdg = _evaluate_geometry_at(geom, p)
    dx = float(point[0]) - wx
    dy = float(point[1]) - wy
    sin_h = math.sin(hdg)
    cos_h = math.cos(hdg)
    t = -dx * sin_h + dy * cos_h
    return _ProjectionCandidate(
        s=float(getattr(geom, "s", 0.0)) + p,
        t=t,
        hdg=hdg,
        distance_sq=dx * dx + dy * dy,
    )


def _project_point_onto_line_geometry(
    point: np.ndarray,
    geom: GeometryBase,
) -> Optional[_ProjectionCandidate]:
    length = float(getattr(geom, "length", 0.0))
    if length <= 0.0 or not math.isfinite(length):
        return None

    px = float(point[0])
    py = float(point[1])
    cos_h = math.cos(geom.hdg)
    sin_h = math.sin(geom.hdg)
    dx = px - geom.x
    dy = py - geom.y
    p = max(0.0, min(length, dx * cos_h + dy * sin_h))
    return _projection_from_geometry_at(point, geom, p)


def _project_point_onto_arc_geometry(
    point: np.ndarray,
    geom: Arc,
) -> Optional[_ProjectionCandidate]:
    length = float(geom.length)
    if length <= 0.0 or not math.isfinite(length):
        return None
    if abs(geom.curvature) <= _geometry_epsilon():
        return _project_point_onto_line_geometry(point, geom)

    px = float(point[0])
    py = float(point[1])
    dx = px - geom.x
    dy = py - geom.y
    cos_h = math.cos(geom.hdg)
    sin_h = math.sin(geom.hdg)

    # Transform the query point into the geometry-local UV frame.
    local_u = dx * cos_h + dy * sin_h
    local_v = -dx * sin_h + dy * cos_h

    radius_signed = 1.0 / geom.curvature
    vx = local_u
    vy = local_v - radius_signed
    if math.hypot(vx, vy) <= _geometry_epsilon():
        # The circle center has no unique nearest angle; fall back to the
        # nearer endpoint, which keeps s inside the road.
        start = _projection_from_geometry_at(point, geom, 0.0)
        end = _projection_from_geometry_at(point, geom, length)
        return start if start.distance_sq <= end.distance_sq else end

    theta = math.atan2(geom.curvature * vx, -geom.curvature * vy)
    period = 2.0 * math.pi
    p_candidates = [0.0, length]
    for k in range(-1, 2):
        p = (theta + k * period) / geom.curvature
        if -_geometry_epsilon() <= p <= length + _geometry_epsilon():
            p_candidates.append(max(0.0, min(length, p)))

    return min(
        (_projection_from_geometry_at(point, geom, p) for p in p_candidates),
        key=lambda candidate: candidate.distance_sq,
    )


def _project_point_onto_parampoly3_geometry(
    point: np.ndarray,
    geom: ParamPoly3,
) -> Optional[_ProjectionCandidate]:
    length = float(geom.length)
    if length <= 0.0 or not math.isfinite(length):
        return None

    cos_h = math.cos(geom.hdg)
    sin_h = math.sin(geom.hdg)
    dx = float(point[0]) - geom.x
    dy = float(point[1]) - geom.y
    query_u = dx * cos_h + dy * sin_h
    query_v = -dx * sin_h + dy * cos_h

    # Minimize squared distance in local UV coordinates. For cubic U/V curves,
    # d(distance^2)/dp is a quintic polynomial, so all stationary points are
    # recoverable from its real roots plus the segment endpoints.
    u_minus_q = np.array([geom.aU - query_u, geom.bU, geom.cU, geom.dU], dtype=float)
    v_minus_q = np.array([geom.aV - query_v, geom.bV, geom.cV, geom.dV], dtype=float)
    du = np.array([geom.bU, 2.0 * geom.cU, 3.0 * geom.dU], dtype=float)
    dv = np.array([geom.bV, 2.0 * geom.cV, 3.0 * geom.dV], dtype=float)
    derivative_poly = np.polynomial.polynomial.polyadd(
        np.polynomial.polynomial.polymul(u_minus_q, du),
        np.polynomial.polynomial.polymul(v_minus_q, dv),
    )

    from ..config import DEFAULT_CONFIG

    coeff_epsilon = DEFAULT_CONFIG.parampoly3.coefficient_epsilon
    while len(derivative_poly) > 1 and abs(derivative_poly[-1]) <= coeff_epsilon:
        derivative_poly = derivative_poly[:-1]

    candidates = [0.0, length]
    if len(derivative_poly) > 1:
        roots = np.roots(derivative_poly[::-1])
        for root in roots:
            if abs(float(np.imag(root))) > coeff_epsilon:
                continue
            p = float(np.real(root))
            if -coeff_epsilon <= p <= length + coeff_epsilon:
                candidates.append(max(0.0, min(length, p)))

    return min(
        (_projection_from_geometry_at(point, geom, p) for p in candidates),
        key=lambda candidate: candidate.distance_sq,
    )


def _project_point_onto_geometry(
    point: np.ndarray,
    geom: GeometryBase,
) -> Optional[_ProjectionCandidate]:
    if isinstance(geom, ParamPoly3):
        return _project_point_onto_parampoly3_geometry(point, geom)
    if isinstance(geom, Arc):
        return _project_point_onto_arc_geometry(point, geom)
    return _project_point_onto_line_geometry(point, geom)


def _project_point_onto_road(
    point: np.ndarray,
    road: Road,
) -> Optional[tuple]:
    """Project a 2D point onto the road reference line.

    Finds the closest point on the continuous road reference line and returns
    the corresponding (s, t, heading) values.

    Args:
        point: 2D point (x, y) to project
        road: Road to project onto

    Returns:
        (s, t, road_hdg) tuple, or None if the road has no geometry.
    """
    best = _project_point_onto_road_with_distance(point, road)
    if best is None:
        return None
    return best.s, best.t, best.hdg


def _project_point_onto_road_with_distance(
    point: np.ndarray,
    road: Road,
) -> Optional[_ProjectionCandidate]:
    """Project a 2D point onto a road and retain the squared distance."""
    if road.plan_view is None or not road.plan_view.geometries:
        return None

    best: Optional[_ProjectionCandidate] = None
    for geom in road.plan_view.geometries:
        candidate = _project_point_onto_geometry(point, geom)
        if candidate is None:
            continue
        if best is None or candidate.distance_sq < best.distance_sq:
            best = candidate

    if best is None:
        return None
    return best


def _road_length(road: Road) -> float:
    try:
        length = float(getattr(road, "length"))
        if math.isfinite(length) and length >= 0.0:
            return length
    except (AttributeError, TypeError, ValueError):
        pass

    if road.plan_view is None or not road.plan_view.geometries:
        return 0.0
    return max(float(geom.s) + float(geom.length) for geom in road.plan_view.geometries)


@dataclass(frozen=True)
class _StopLineRoadCandidate:
    road: Road
    projection: _ProjectionCandidate
    road_length: float

    @property
    def distance(self) -> float:
        return math.sqrt(self.projection.distance_sq)

    @property
    def distance_to_end(self) -> float:
        return abs(self.road_length - self.projection.s)

    @property
    def longitudinal_residual(self) -> float:
        residual_sq = max(
            0.0,
            self.projection.distance_sq - self.projection.t * self.projection.t,
        )
        return math.sqrt(residual_sq)


def _stop_line_road_candidates(
    centroid: np.ndarray,
    roads: List[Road],
    threshold_m: float,
) -> List[_StopLineRoadCandidate]:
    candidates: List[_StopLineRoadCandidate] = []
    seen_road_ids: set[int] = set()
    for road in roads:
        road_id = int(getattr(road, "id", -1))
        if road_id in seen_road_ids:
            continue
        seen_road_ids.add(road_id)
        projection = _project_point_onto_road_with_distance(centroid, road)
        if projection is None:
            continue
        candidate = _StopLineRoadCandidate(
            road=road,
            projection=projection,
            road_length=_road_length(road),
        )
        if candidate.distance <= threshold_m:
            candidates.append(candidate)
    return candidates


def find_best_road_for_stop_line(
    linestring: lanelet2.core.LineString3d,
    all_roads: List["Road"],
    *,
    related_roads: Optional[List["Road"]] = None,
    predecessor_roads: Optional[List["Road"]] = None,
    endpoint_tolerance: float = 0.5,
    longitudinal_tolerance: float = 0.05,
    threshold_m: float = _NEAREST_ROAD_THRESHOLD_M,
) -> Optional["Road"]:
    """Select the OpenDRIVE road that should own a stop-line object.

    Stop lines at the start boundary of junction/turn lanelets physically mark
    the end of the incoming road. Prefer such direct-predecessor roads when the
    projected stop-line centroid falls near their end; otherwise prefer roads
    mapped from lanelets that reference the regulatory element. The historical
    nearest-road search remains the fallback when no semantic/topological
    candidate is available.
    """
    pts = extract_points(linestring, dimensions=2)
    if len(pts) == 0:
        return None

    centroid = np.mean(pts, axis=0)

    fallback_road = find_nearest_road_for_linestring(
        linestring,
        all_roads,
        threshold_m=threshold_m,
    )
    fallback_candidate = None
    if fallback_road is not None:
        fallback_projection = _project_point_onto_road_with_distance(
            centroid,
            fallback_road,
        )
        if fallback_projection is not None:
            fallback_candidate = _StopLineRoadCandidate(
                road=fallback_road,
                projection=fallback_projection,
                road_length=_road_length(fallback_road),
            )

    predecessor_candidates = _stop_line_road_candidates(
        centroid,
        predecessor_roads or [],
        threshold_m,
    )
    incoming_candidates = [
        candidate
        for candidate in predecessor_candidates
        if candidate.distance_to_end <= endpoint_tolerance
    ]
    related_candidates = _stop_line_road_candidates(
        centroid,
        related_roads or [],
        threshold_m,
    )

    if fallback_candidate is None:
        semantic_candidates = incoming_candidates or related_candidates
        if semantic_candidates:
            return min(
                semantic_candidates,
                key=lambda candidate: (
                    candidate.longitudinal_residual,
                    candidate.distance,
                ),
            ).road
        return None

    # Preserve the historical nearest-road assignment unless that road can
    # represent the stop line only by clamping to an endpoint. This keeps
    # ordinary mid-road stop lines stable while allowing semantic/topological
    # ownership to repair junction-boundary outliers.
    if fallback_candidate.longitudinal_residual <= longitudinal_tolerance:
        return fallback_road

    improving_incoming = [
        candidate
        for candidate in incoming_candidates
        if candidate.longitudinal_residual + longitudinal_tolerance
        < fallback_candidate.longitudinal_residual
    ]
    if improving_incoming:
        return min(
            improving_incoming,
            key=lambda candidate: (
                candidate.longitudinal_residual,
                candidate.distance_to_end,
                candidate.distance,
            ),
        ).road

    improving_related = [
        candidate
        for candidate in related_candidates
        if candidate.longitudinal_residual + longitudinal_tolerance
        < fallback_candidate.longitudinal_residual
    ]
    if improving_related:
        return min(
            improving_related,
            key=lambda candidate: (
                candidate.longitudinal_residual,
                candidate.distance,
            ),
        ).road

    return fallback_road


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


def _normalize_angle(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


def _evaluate_road_reference_at_s(
    road: "Road",
    s: float,
) -> Optional[tuple[float, float, float]]:
    """Evaluate a road reference line at global road station s."""
    if road.plan_view is None or not road.plan_view.geometries:
        return None

    best_geom: Optional[GeometryBase] = None
    best_p = 0.0
    best_error = float("inf")
    for geom in road.plan_view.geometries:
        geom_s = float(getattr(geom, "s", 0.0))
        geom_length = float(getattr(geom, "length", 0.0))
        if geom_length <= 0.0 or not math.isfinite(geom_length):
            continue
        p = max(0.0, min(geom_length, float(s) - geom_s))
        error = abs((geom_s + p) - float(s))
        if error < best_error:
            best_error = error
            best_geom = geom
            best_p = p

    if best_geom is None:
        return None
    return _evaluate_geometry_at(best_geom, best_p)


def _compute_stop_line_outline_corners(
    anchor: np.ndarray,
    stop_line_dir: np.ndarray,
    p0: np.ndarray,
    p1: np.ndarray,
    width: float,
) -> List[CornerLocal]:
    """Compute a painted stop-line rectangle around the source LineString."""
    normal = np.array([-stop_line_dir[1], stop_line_dir[0]])
    half_width = 0.5 * max(0.0, float(width))
    vertices = [
        p0 - half_width * normal,
        p1 - half_width * normal,
        p1 + half_width * normal,
        p0 + half_width * normal,
    ]

    corners: List[CornerLocal] = []
    for vertex in vertices:
        delta = vertex - anchor
        corners.append(
            CornerLocal(
                u=float(np.dot(delta, stop_line_dir)),
                v=float(np.dot(delta, normal)),
                z=0.0,
            )
        )
    if corners:
        corners.append(corners[0])
    return corners


@dataclass
class StopLineObject:
    """OpenDRIVE object representing a stop line.

    Corresponds to <object type="stopLine"> in OpenDRIVE specification.
    When carla_format=True, outputs CARLA's Stencil_STOP format instead:
    <object type="-1" name="Stencil_STOP" orientation="-" zOffset="0.0">.
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
    width: float = (
        0.0  # thickness in v-direction (perpendicular to stop line = along road)
    )
    length: float = 0.0  # extent in u-direction (along stop line heading = across road)
    carla_format: bool = False  # If True, output CARLA Stencil_STOP format
    corners: List[CornerLocal] = field(default_factory=list)

    def to_xml(self) -> ET.Element:
        """Convert to XML element.

        Returns:
            <object type="stopLine"> element (standard), or
            <object type="-1" name="Stencil_STOP"> element (CARLA format).
        """
        elem = ET.Element("object")
        if self.carla_format:
            elem.set("type", "-1")
            elem.set("name", "Stencil_STOP")
        else:
            elem.set("type", "stopLine")
            elem.set("name", self.name)
        elem.set("id", str(self.id))
        elem.set("s", str(self.s))
        elem.set("t", str(self.t))
        z_offset = 0.0 if self.carla_format else self.z_offset
        elem.set("zOffset", str(z_offset))
        elem.set("hdg", str(self.hdg))
        elem.set("pitch", str(self.pitch))
        elem.set("roll", str(self.roll))
        orientation = "-" if self.carla_format else self.orientation
        elem.set("orientation", orientation)
        elem.set("width", str(self.width))
        elem.set("length", str(self.length))

        if not self.carla_format and self.corners:
            outline_elem = ET.SubElement(elem, "outline")
            for corner in self.corners:
                outline_elem.append(corner.to_xml())

        return elem

    @staticmethod
    def construct_from_linestring(
        linestring: lanelet2.core.LineString3d,
        road: "Road",
        object_id: int,
        width: float = 0.1,
        carla_format: bool = False,
        use_physical_outline: bool = False,
    ) -> Optional["StopLineObject"]:
        """Construct a StopLineObject from a stop_line linestring and its nearest road.

        Args:
            linestring: LineString with type="stop_line"
            road: The nearest road to associate this stop line with
            object_id: ID for the resulting object (typically linestring.id)
            width: Painted width of the stop line in v-direction (along road), meters
            carla_format: If True, create a CARLA Stencil_STOP formatted object
            use_physical_outline: If True, encode the source stop-line rectangle as
                a local outline when the projected anchor is endpoint-clamped.

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

            projection = _project_point_onto_road_with_distance(centroid, road)
            if projection is None:
                logger.warning(
                    f"Could not project stop line {linestring.id} centroid onto road {road.id}"
                )
                return None
            s, t, road_hdg_at_s = projection.s, projection.t, projection.hdg

            # Compute z_offset from 3D points vs road elevation
            pts_3d = extract_points(linestring, dimensions=3)
            stop_line_absolute_z = float(np.mean(pts_3d[:, 2]))
            road_elevation_at_s = road.get_elevation_at_s(s)
            z_offset = stop_line_absolute_z - road_elevation_at_s

            # Heading: direction of the stop line (from first to last point)
            direction = pts[-1] - pts[0]
            # length = span along heading (u-axis = across road)
            # width = painted thickness in v-direction (along road)
            length = float(np.linalg.norm(pts[-1] - pts[0]))
            stop_line_angle = math.atan2(float(direction[1]), float(direction[0]))
            hdg = _normalize_angle(stop_line_angle - road_hdg_at_s)
            corners: List[CornerLocal] = []

            longitudinal_residual = math.sqrt(
                max(0.0, projection.distance_sq - projection.t * projection.t)
            )
            from ..config import DEFAULT_CONFIG

            outline_threshold = max(
                float(width) / 2.0,
                DEFAULT_CONFIG.geometry.point_distance_threshold,
            )
            if (
                use_physical_outline
                and not carla_format
                and longitudinal_residual > outline_threshold
                and length > _geometry_epsilon()
            ):
                road_pose = _evaluate_road_reference_at_s(road, s)
                if road_pose is not None:
                    anchor_x, anchor_y, _ = road_pose
                    road_normal = np.array(
                        [-math.sin(road_hdg_at_s), math.cos(road_hdg_at_s)]
                    )
                    anchor = np.array([anchor_x, anchor_y]) + t * road_normal
                    stop_line_dir = direction / length
                    corners = _compute_stop_line_outline_corners(
                        anchor,
                        stop_line_dir,
                        pts[0],
                        pts[-1],
                        width,
                    )

            return StopLineObject(
                id=object_id,
                name=f"stop_line_{object_id}",
                s=s,
                t=t,
                z_offset=z_offset,
                hdg=hdg,
                width=width,
                length=length,
                carla_format=carla_format,
                corners=corners,
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
