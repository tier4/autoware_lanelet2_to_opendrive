"""Parking-lot conversion logic for the Lanelet2 → OpenDRIVE pipeline (P2-1).

This module emits a synthetic OpenDRIVE ``Road`` for each Lanelet2
``parking_lot`` Area, with two PARKING-typed lanes flanking the
reference line and one ``<object type="parkingSpace">`` per
``parking_space`` LineString belonging to that lot.

See ``docs/superpowers/specs/2026-04-28-p2-1-parking-lots-design.md``
for the design overview.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple, cast

import lanelet2
import lxml.etree as ET
import numpy as np

from ..conversion_config import ParkingLotConfig
from ..util import extract_points
from .enums import LaneType
from .geometry import GeometryBase, Line, PlanView
from .lane import Lane
from .lane_elements import LaneWidth
from .lane_section import LaneSection
from .lane_sections import Lanes
from .objects import _project_point_onto_road
from .road import Road

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Numerical constants used only inside this module.
# ---------------------------------------------------------------------------

# Tolerance used both for stall-length sanity checks and for the OBB
# eigenvalue-degeneracy test.  Values smaller than this are considered
# numerically zero in the parking context.
_GEOMETRY_EPSILON = 1e-6

# Relative eigenvalue ratio under which the long axis is considered
# undefined (e.g. perfect square lots).  In that case we fall back to
# world-x as the long axis to keep the synthetic road deterministic.
_OBB_DEGENERACY_RELATIVE_TOL = 1e-6


# ---------------------------------------------------------------------------
# OBB and polygon helpers
# ---------------------------------------------------------------------------


def _compute_obb(
    polygon_xy: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray, float, float]:
    """Compute a 2D oriented bounding box (OBB) for a polygon.

    Uses 2D PCA (``np.cov`` + ``np.linalg.eigh``) on the polygon
    vertices to derive the long-axis direction.  When the two
    principal eigenvalues are nearly equal (e.g. a perfect square),
    the long axis is ill-defined and we fall back to world-x as the
    long axis so callers get a deterministic frame.

    Args:
        polygon_xy: ``(N, 2)`` array of polygon vertex coordinates.

    Returns:
        Tuple of:
            * ``centre`` – ``(2,)`` centroid of the vertex cloud.
            * ``long_axis`` – ``(2,)`` unit vector along the long
              axis (or world-x in the degenerate case).
            * ``along_length`` – Extent of the polygon along
              ``long_axis``.
            * ``across_length`` – Extent of the polygon perpendicular
              to ``long_axis``.
    """
    centre = polygon_xy.mean(axis=0)
    centred = polygon_xy - centre

    if centred.shape[0] < 2:
        # Not enough points to define an OBB; fall back to world axes
        # so the caller can decide whether to skip the area.
        return centre, np.array([1.0, 0.0]), 0.0, 0.0

    cov = np.cov(centred.T)
    # eigh returns eigenvalues in ascending order
    eigvals, eigvecs = np.linalg.eigh(cov)
    largest = float(eigvals[-1])
    second = float(eigvals[-2])

    # Detect axis degeneracy: if the two principal eigenvalues are
    # nearly equal the long axis is ill-defined.  Fall back to
    # world-x so the synthetic road frame stays stable.
    if largest <= 0.0 or (largest - second) < _OBB_DEGENERACY_RELATIVE_TOL * largest:
        long_axis = np.array([1.0, 0.0])
    else:
        long_axis = eigvecs[:, -1]
        # ``np.linalg.eigh`` eigenvectors are only defined up to sign;
        # canonicalise the principal-axis direction so the synthetic-road
        # frame (start point, hdg, sign of projected ``t``) stays
        # deterministic across runs / numpy backends.  Prefer +x; if x is
        # numerically zero, prefer +y.
        if long_axis[0] < -_GEOMETRY_EPSILON or (
            abs(long_axis[0]) <= _GEOMETRY_EPSILON and long_axis[1] < 0.0
        ):
            long_axis = -long_axis

    short_axis = np.array([-long_axis[1], long_axis[0]])

    along = centred @ long_axis
    across = centred @ short_axis

    along_length = float(along.max() - along.min())
    across_length = float(across.max() - across.min())
    return centre, long_axis, along_length, across_length


def _polygon_area(xy: np.ndarray) -> float:
    """Compute the (unsigned) polygon area via the shoelace formula.

    Args:
        xy: ``(N, 2)`` polygon vertex coordinates.

    Returns:
        Polygon area in square metres (always non-negative).  Returns
        ``0.0`` for degenerate inputs with fewer than three points.
    """
    if xy.shape[0] < 3:
        return 0.0
    x = xy[:, 0]
    y = xy[:, 1]
    return float(0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))))


def _area_polygon_xy(area: lanelet2.core.Area) -> np.ndarray:
    """Concatenate the outer-bound LineStrings into a 2D polygon array.

    The outer bound of a Lanelet2 ``Area`` is a closed loop made of one
    or more ``LineString3d`` segments.  Shared endpoints between
    consecutive LineStrings are not deduplicated because the OBB is
    barely affected by an extra coincident vertex.
    """
    points: List[List[float]] = []
    try:
        outer = area.outerBound
    except Exception:  # pragma: no cover - defensive
        return np.empty((0, 2))

    for ls in outer:
        try:
            pts = extract_points(ls, dimensions=2)
        except Exception:
            continue
        if pts.size:
            points.extend(pts.tolist())

    if not points:
        return np.empty((0, 2))
    return np.asarray(points, dtype=float)


def _stall_centroid_and_length(
    stall: lanelet2.core.LineString3d,
) -> Optional[Tuple[np.ndarray, float, np.ndarray, np.ndarray]]:
    """Return ``(centroid, length, start, end)`` for a stall LineString.

    Returns ``None`` when the LineString is degenerate (fewer than two
    points or near-zero length).
    """
    try:
        pts = extract_points(stall, dimensions=2)
    except Exception:
        return None
    if pts.shape[0] < 2:
        return None

    diffs = np.diff(pts, axis=0)
    length = float(np.sqrt((diffs**2).sum(axis=1)).sum())
    if length < _GEOMETRY_EPSILON:
        return None

    centroid = pts.mean(axis=0)
    return centroid, length, pts[0], pts[-1]


def _point_in_polygon(point: np.ndarray, polygon_xy: np.ndarray) -> bool:
    """Ray-cast point-in-polygon test.

    The polygon is treated as a closed ring formed by ``polygon_xy``
    in order; vertex order (CW vs. CCW) is irrelevant.  Edge / vertex
    coincidence is treated as "inside" (the test returns ``True``)
    because the ambiguity does not matter for the
    threshold-against-zero comparison in the caller.
    """
    n = polygon_xy.shape[0]
    if n < 3:
        return False
    px = float(point[0])
    py = float(point[1])
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = float(polygon_xy[i, 0]), float(polygon_xy[i, 1])
        xj, yj = float(polygon_xy[j, 0]), float(polygon_xy[j, 1])
        if (yi > py) != (yj > py):
            x_intersect = (xj - xi) * (py - yi) / (yj - yi) + xi
            if px < x_intersect:
                inside = not inside
        j = i
    return inside


def _min_distance_to_polygon(point: np.ndarray, polygon_xy: np.ndarray) -> float:
    """Minimum 2D distance from ``point`` to ``polygon_xy``.

    Returns 0 when ``point`` lies inside the polygon and otherwise the
    minimum distance from ``point`` to any *edge* (point-to-segment) of
    the polygon.  This is more robust than vertex-only distance for
    large lots: a stall centroid inside a 100 m × 100 m parking_lot
    polygon is still within the default 30 m threshold even though the
    nearest vertex is ~70 m away.
    """
    if polygon_xy.size == 0:
        return float("inf")
    if _point_in_polygon(point, polygon_xy):
        return 0.0

    # Vectorised point-to-segment distance against every edge.
    a = polygon_xy
    b = np.roll(polygon_xy, -1, axis=0)
    ab = b - a
    ab_len_sq = (ab**2).sum(axis=1)
    # Avoid division by zero on degenerate (zero-length) edges.
    safe_len_sq = np.where(ab_len_sq > 0.0, ab_len_sq, 1.0)
    ap = point - a
    t = np.clip((ap * ab).sum(axis=1) / safe_len_sq, 0.0, 1.0)
    closest = a + (t[:, None] * ab)
    diffs = closest - point
    return float(np.sqrt((diffs**2).sum(axis=1)).min())


# ---------------------------------------------------------------------------
# Lanelet2 attribute filtering
# ---------------------------------------------------------------------------


def _attr_matches(attrs: object, expected: str) -> bool:
    """Return True when ``type`` *or* ``subtype`` equals ``expected``.

    Lanelet2's ``AttributeMap`` supports ``__contains__`` and
    ``__getitem__`` but not ``.get``; we keep the access pattern
    defensive so a missing ``type`` does not raise.
    """
    try:
        if "type" in attrs and attrs["type"] == expected:  # type: ignore[operator]
            return True
    except Exception:
        pass
    try:
        if "subtype" in attrs and attrs["subtype"] == expected:  # type: ignore[operator]
            return True
    except Exception:
        pass
    return False


def _filter_parking_lot_areas(
    area_layer: object,
) -> List[lanelet2.core.Area]:
    """Filter an ``areaLayer`` (or any iterable of Areas) to parking lots.

    Accepts both ``type="parking_lot"`` and ``subtype="parking_lot"``
    aliases (Autoware OSM exporters use either).
    """
    result: List[lanelet2.core.Area] = []
    try:
        iterable = list(area_layer)  # type: ignore[arg-type]
    except TypeError:
        return result
    for area in iterable:
        attrs = getattr(area, "attributes", None)
        if attrs is None:
            continue
        if _attr_matches(attrs, "parking_lot"):
            result.append(area)
    return result


def _filter_parking_space_linestrings(
    line_string_layer: object,
) -> List[lanelet2.core.LineString3d]:
    """Filter a ``lineStringLayer`` to ``parking_space`` LineStrings.

    Accepts both ``type="parking_space"`` and
    ``subtype="parking_space"`` aliases.
    """
    result: List[lanelet2.core.LineString3d] = []
    try:
        iterable = list(line_string_layer)  # type: ignore[arg-type]
    except TypeError:
        return result
    for ls in iterable:
        attrs = getattr(ls, "attributes", None)
        if attrs is None:
            continue
        if _attr_matches(attrs, "parking_space"):
            result.append(ls)
    return result


# ---------------------------------------------------------------------------
# ParkingSpaceObject – OpenDRIVE <object type='parkingSpace'>
# ---------------------------------------------------------------------------


@dataclass
class ParkingSpaceObject:
    """OpenDRIVE ``<object type='parkingSpace'>`` for a single stall.

    Mirrors the StopLineObject layout: ``hdg`` is interpreted as the
    angle of the stall LineString relative to the road reference-line
    tangent at ``s``.  Downstream consumers read it as a relative
    heading; no normalisation beyond ``(-π, π]`` is applied.
    """

    id: int
    name: str
    s: float
    t: float
    z_offset: float
    hdg: float
    length: float
    width: float
    pitch: float = 0.0
    roll: float = 0.0
    orientation: str = "none"

    def to_xml(self) -> ET.Element:
        """Render the parkingSpace as an ``<object>`` element."""
        elem = ET.Element("object")
        elem.set("type", "parkingSpace")
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
    def construct_from_stall_linestring(
        stall: lanelet2.core.LineString3d,
        road: Road,
        object_id: int,
        default_width: float,
    ) -> Optional["ParkingSpaceObject"]:
        """Construct a ParkingSpaceObject by projecting the stall onto a road.

        Returns ``None`` and logs a warning when the LineString is too
        short or when Frenet projection onto ``road`` fails.
        """
        ls_id = getattr(stall, "id", -1)
        info = _stall_centroid_and_length(stall)
        if info is None:
            logger.warning(
                "Parking stall LineString %s is degenerate, skipping",
                ls_id,
            )
            return None
        centroid, length, p_start, p_end = info

        projection = _project_point_onto_road(centroid, road)
        if projection is None:
            logger.warning(
                "Parking stall LineString %s could not be projected onto road %s, "
                "skipping",
                ls_id,
                road.id,
            )
            return None
        s, t, road_hdg = projection

        # Heading is relative to the road tangent (matches the
        # StopLineObject convention used elsewhere in the project).
        direction = p_end - p_start
        stall_angle = math.atan2(float(direction[1]), float(direction[0]))
        hdg = (stall_angle - road_hdg + math.pi) % (2 * math.pi) - math.pi

        # The OpenDRIVE object id is caller-controlled (sequential per
        # road) while the stall lanelet id is preserved in ``name`` for
        # traceability back to the source map.  Mirrors the convention
        # used by CrosswalkObject and StopLineObject.
        return ParkingSpaceObject(
            id=object_id,
            name=f"parking_space_{ls_id}",
            s=float(s),
            t=float(t),
            z_offset=0.0,
            hdg=float(hdg),
            length=float(length),
            width=float(default_width),
        )


# ---------------------------------------------------------------------------
# ParkingLot – intermediate representation
# ---------------------------------------------------------------------------


@dataclass
class ParkingLot:
    """Intermediate representation of a Lanelet2 parking lot Area + stalls."""

    area: lanelet2.core.Area
    stalls: List[lanelet2.core.LineString3d] = field(default_factory=list)

    @staticmethod
    def construct_all_from_lanelet_map(
        lanelet_map: lanelet2.core.LaneletMap,
        config: ParkingLotConfig,
    ) -> List["ParkingLot"]:
        """Discover parking lots and assign stalls to the nearest lot.

        Stalls beyond ``config.nearest_area_threshold_m`` from every
        parking-lot area are dropped with a warning.  Areas with no
        nearby stalls remain in the result with an empty ``stalls``
        list (the synthetic road is still emitted, with no objects).
        """
        if not config.enabled:
            return []

        area_layer = getattr(lanelet_map, "areaLayer", None)
        ls_layer = getattr(lanelet_map, "lineStringLayer", None)
        if area_layer is None:
            return []

        areas = _filter_parking_lot_areas(area_layer)
        if not areas:
            return []

        # Pre-compute the polygon coordinates for each area so the
        # nearest-area search does not pay the extract_points cost
        # per stall.
        lots: List[ParkingLot] = [ParkingLot(area=area) for area in areas]
        polygons: List[np.ndarray] = [_area_polygon_xy(a) for a in areas]

        stalls = (
            _filter_parking_space_linestrings(ls_layer) if ls_layer is not None else []
        )

        threshold = float(config.nearest_area_threshold_m)
        for stall in stalls:
            info = _stall_centroid_and_length(stall)
            if info is None:
                logger.warning(
                    "Parking stall LineString %s is degenerate, skipping",
                    getattr(stall, "id", -1),
                )
                continue
            centroid = info[0]

            # Find nearest lot by minimum distance to the polygon (0 if
            # the centroid lies inside, otherwise distance to nearest edge).
            best_idx = -1
            best_dist = float("inf")
            for idx, poly in enumerate(polygons):
                dist = _min_distance_to_polygon(centroid, poly)
                if dist < best_dist:
                    best_dist = dist
                    best_idx = idx

            if best_idx < 0 or best_dist > threshold:
                logger.warning(
                    "Parking stall LineString %s is %.1fm from any parking_lot "
                    "(threshold=%.1fm), skipping",
                    getattr(stall, "id", -1),
                    best_dist,
                    threshold,
                )
                continue

            lots[best_idx].stalls.append(stall)

        return lots

    def to_road_and_objects(
        self,
        road_id: int,
        config: ParkingLotConfig,
    ) -> Tuple[Optional[Road], List[ParkingSpaceObject]]:
        """Build a synthetic Road + ParkingSpaceObjects for this lot.

        Returns ``(None, [])`` when the area polygon is degenerate
        (below ``config.min_area_polygon_m2``).  An area with no
        stalls within range still produces a road; the caller may
        decide whether to keep it.
        """
        polygon_xy = _area_polygon_xy(self.area)
        area_value = _polygon_area(polygon_xy)
        area_id = getattr(self.area, "id", -1)

        if area_value < float(config.min_area_polygon_m2):
            logger.warning(
                "Parking lot area %s polygon area %.3f m^2 is below the "
                "minimum %.3f m^2, skipping",
                area_id,
                area_value,
                config.min_area_polygon_m2,
            )
            return None, []

        road = _build_synthetic_road(self, road_id, config, polygon_xy)
        if road is None:
            return None, []

        # Project each stall onto the synthetic road to derive (s, t, hdg).
        # OpenDRIVE 1.7 §3.6.1 requires ``<object id>`` to be unique within a
        # road, so we assign sequential IDs (1, 2, 3, ...) per lot rather
        # than reusing the stall lanelet ID — that keeps the road valid
        # even if other object kinds are attached to the same Road later.
        # The lanelet ID is preserved in the object ``name`` for traceability.
        objects_out: List[ParkingSpaceObject] = []
        for object_id, stall in enumerate(self.stalls, start=1):
            obj = ParkingSpaceObject.construct_from_stall_linestring(
                stall=stall,
                road=road,
                object_id=object_id,
                default_width=float(config.default_stall_width),
            )
            if obj is not None:
                objects_out.append(obj)

        # Attach the objects to the road so downstream XML serialisation
        # picks them up automatically.
        if objects_out:
            road.objects = list(objects_out)

        return road, objects_out


# ---------------------------------------------------------------------------
# Synthetic road construction
# ---------------------------------------------------------------------------


def _build_synthetic_road(
    parking_lot: ParkingLot,
    road_id: int,
    config: ParkingLotConfig,
    polygon_xy: np.ndarray,
) -> Optional[Road]:
    """Build a straight synthetic Road covering the parking lot OBB.

    The reference line runs along the OBB long axis from
    ``centre - (along/2) * long_axis`` to ``centre + (along/2) *
    long_axis`` with a single ``Line`` geometry.  Two PARKING-typed
    lanes (one left, one right) of constant half-width
    ``across_length / 2`` flank the reference line.
    """
    if polygon_xy.shape[0] < 2:
        logger.warning(
            "Parking lot area %s has too few polygon points, skipping",
            getattr(parking_lot.area, "id", -1),
        )
        return None

    centre, long_axis, along_length, across_length = _compute_obb(polygon_xy)
    if along_length < _GEOMETRY_EPSILON:
        logger.warning(
            "Parking lot area %s has near-zero along-axis length, skipping",
            getattr(parking_lot.area, "id", -1),
        )
        return None

    half_along = along_length / 2.0
    start = centre - half_along * long_axis
    hdg = math.atan2(float(long_axis[1]), float(long_axis[0]))

    # Reference-line geometry: a single straight Line segment.
    line = Line(
        s=0.0,
        x=float(start[0]),
        y=float(start[1]),
        hdg=hdg,
        length=float(along_length),
    )
    plan_view = PlanView(geometries=cast(List[GeometryBase], [line]))

    # Lane section: PARKING lanes on both sides, NONE centre lane.
    half_width = max(across_length / 2.0, 0.0)
    lane_section = LaneSection(s_offset=0.0)

    # Centre lane (id=0, type=NONE).  We bypass the ReferenceLine
    # wrapper because building a Splines fit for a synthetic straight
    # line would be overkill; ``LaneSection.center_lane`` is typed
    # ``Optional[Union[ReferenceLine, Lane]]`` precisely to support
    # this case, and ``get_all_lanes`` unwraps either form.
    center_lane = Lane(lane_id=0, lane_type=LaneType.NONE)
    lane_section.center_lane = center_lane

    if half_width > 0.0:
        right = Lane(lane_id=-1, lane_type=LaneType.PARKING)
        right._add_width(LaneWidth(s_offset=0.0, a=half_width))
        lane_section._add_right_lane(right)

        left = Lane(lane_id=1, lane_type=LaneType.PARKING)
        left._add_width(LaneWidth(s_offset=0.0, a=half_width))
        lane_section._add_left_lane(left)

    lanes = Lanes(lane_sections=[lane_section])

    road = Road(
        id=road_id,
        name=f"parking_lot_{getattr(parking_lot.area, 'id', road_id)}",
        length=float(along_length),
        junction=-1,
        plan_view=plan_view,
        lanes=lanes,
    )
    return road


# ---------------------------------------------------------------------------
# Public top-level entry point
# ---------------------------------------------------------------------------


def construct_parking_roads(
    lanelet_map: lanelet2.core.LaneletMap,
    starting_road_id: int,
    config: ParkingLotConfig,
) -> List[Road]:
    """Build synthetic parking-lot roads from a Lanelet2 map.

    Args:
        lanelet_map: The Lanelet2 map.
        starting_road_id: First road ID to assign.  Subsequent lots
            consume sequential IDs.
        config: ``ParkingLotConfig`` controlling thresholds and
            stall width defaults.

    Returns:
        A list of synthetic ``Road`` objects, one per parking lot
        whose polygon area is above ``config.min_area_polygon_m2``.
        Lots with no associated stalls still produce a road (with no
        ``<object>`` children); this is intentional per the design
        spec §8.
    """
    if not config.enabled:
        return []

    lots = ParkingLot.construct_all_from_lanelet_map(lanelet_map, config)
    if not lots:
        return []

    roads: List[Road] = []
    next_id = int(starting_road_id)
    for lot in lots:
        road, _objects = lot.to_road_and_objects(next_id, config)
        if road is None:
            continue
        roads.append(road)
        next_id += 1

    return roads


__all__ = [
    "ParkingLot",
    "ParkingSpaceObject",
    "construct_parking_roads",
]
