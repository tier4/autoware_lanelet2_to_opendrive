"""Junction-wide physical topology and geometry emission planning.

The converter's Lanelet2 routing graph is the semantic source of truth.  This
module may change the OpenDRIVE road decomposition, but only after proving that
the emitted connector groups preserve the same lane-level maneuver pairs.
"""

from __future__ import annotations

import copy
import math
from dataclasses import asdict, dataclass
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Set, Tuple

import lanelet2
import numpy as np

from .config import DEFAULT_CONFIG
from .opendrive.elevation import Elevation, ElevationProfile
from .opendrive.enums import ContactPoint, ElementType, LaneType, TrafficRule
from .opendrive.geometry import Arc, ParamPoly3, PlanView, evaluate_plan_view_world
from .opendrive.junction import Connection, Junction
from .opendrive.junction import LaneLink as JunctionLaneLink
from .opendrive.lane import Lane
from .opendrive.lane_elements import LaneLink, LaneWidth
from .opendrive.lane_section import LaneSection
from .opendrive.lane_sections import Lanes
from .opendrive.reference_geometry import (
    EmissionReferenceGeometry,
    RoadEmissionContext,
    StationMapping,
)
from .opendrive.reference_line import ReferenceLine
from .opendrive.road import (
    Road,
    _evaluate_elevation_profile,
    _evaluate_lane_width,
    _evaluate_planview_endpoint_with_heading,
)
from .opendrive.road_links import Predecessor, RoadLink, Successor
from .util import extract_points_3d


@dataclass(frozen=True)
class LogicalLane:
    """A source Lanelet2 lane represented in the logical junction graph."""

    lanelet_id: int
    road_id: int
    lane_id: int
    subtype: str


@dataclass(frozen=True)
class LogicalManeuver:
    """One allowed source lane-to-lane transition."""

    incoming: LogicalLane
    outgoing: LogicalLane

    @property
    def lanelet_pair(self) -> Tuple[int, int]:
        """Return the semantic Lanelet2 maneuver key."""
        return (self.incoming.lanelet_id, self.outgoing.lanelet_id)


@dataclass(frozen=True)
class EmittedLaneSegment:
    """One OpenDRIVE segment used by a logical lane trace."""

    road_id: int
    lane_id: int
    role: str


@dataclass(frozen=True)
class LaneTrace:
    """Trace from one logical lane into emitted OpenDRIVE segments."""

    lanelet_id: int
    emitted_segments: Tuple[EmittedLaneSegment, ...]


@dataclass(frozen=True)
class CutSection:
    """A full lane-group cross-section used as a connector endpoint."""

    road_id: int
    station_from_boundary: float
    reference_xyz: Tuple[float, float, float]
    heading: float
    lane_widths: Tuple[float, ...]


@dataclass(frozen=True)
class SurfaceValidation:
    """Finite-width checks for one connector geometry candidate."""

    c0: bool
    c1: bool
    finite_width_valid: bool
    center_self_intersection: bool
    boundary_self_intersection: bool
    boundary_reversal: bool
    max_abs_curvature: float
    max_kappa_half_width: float
    reference_length: float
    max_internal_heading_change: float


@dataclass
class ConnectingRoadGroup:
    """Canonical emitted connector for parallel logical maneuvers."""

    incoming_road_id: int
    outgoing_road_id: int
    connector_road_id: int
    maneuvers: Tuple[LogicalManeuver, ...]
    connector_lane_ids: Tuple[int, ...]
    replaced_connector_road_ids: Tuple[int, ...]
    incoming_cut: CutSection
    outgoing_cut: CutSection
    curve_points_xyz: np.ndarray
    curve_control_points_xyz: np.ndarray
    surface: SurfaceValidation

    @property
    def lane_links(self) -> Tuple[Tuple[int, int], ...]:
        """Incoming lane to connector lane assignments."""
        return tuple(
            (maneuver.incoming.lane_id, connector_lane_id)
            for maneuver, connector_lane_id in zip(
                self.maneuvers, self.connector_lane_ids
            )
        )


@dataclass
class JunctionEmissionPlan:
    """Production plan for one junction's emitted topology and geometry."""

    junction_id: int
    logical_incoming_lanes: Tuple[LogicalLane, ...]
    logical_outgoing_lanes: Tuple[LogicalLane, ...]
    logical_maneuvers: Tuple[LogicalManeuver, ...]
    forbidden_maneuvers: Tuple[Tuple[int, int], ...]
    connecting_road_groups: Tuple[ConnectingRoadGroup, ...]
    lane_traces: Tuple[LaneTrace, ...]
    missing_maneuvers: Tuple[Tuple[int, int], ...] = ()
    unintended_maneuvers: Tuple[Tuple[int, int], ...] = ()
    applied: bool = False

    @property
    def protected_connector_ids(self) -> Set[int]:
        """Connector IDs whose planned geometry must not be realigned."""
        return {group.connector_road_id for group in self.connecting_road_groups}

    def to_summary_dict(self) -> dict:
        """Serialize the stable, JSON-safe planning summary."""
        return {
            "junction_id": self.junction_id,
            "logical_maneuvers": [
                list(maneuver.lanelet_pair) for maneuver in self.logical_maneuvers
            ],
            "forbidden_maneuvers": [list(pair) for pair in self.forbidden_maneuvers],
            "missing_maneuvers": [list(pair) for pair in self.missing_maneuvers],
            "unintended_maneuvers": [list(pair) for pair in self.unintended_maneuvers],
            "applied": self.applied,
            "connecting_road_groups": [
                {
                    "incoming_road_id": group.incoming_road_id,
                    "outgoing_road_id": group.outgoing_road_id,
                    "connector_road_id": group.connector_road_id,
                    "lanelet_pairs": [
                        list(maneuver.lanelet_pair) for maneuver in group.maneuvers
                    ],
                    "lane_links": [list(pair) for pair in group.lane_links],
                    "replaced_connector_road_ids": list(
                        group.replaced_connector_road_ids
                    ),
                    "incoming_cut": asdict(group.incoming_cut),
                    "outgoing_cut": asdict(group.outgoing_cut),
                    "surface": asdict(group.surface),
                }
                for group in self.connecting_road_groups
            ],
        }


@dataclass(frozen=True)
class _CrossSection:
    reference_xyz: np.ndarray
    tangent: np.ndarray
    widths: np.ndarray
    lane_side: int = 1
    width_derivatives: Optional[np.ndarray] = None
    lane_offset: float = 0.0
    lane_offset_derivative: float = 0.0
    source_lane_offset: float = 0.0
    source_lane_offset_derivative: float = 0.0
    source_curvature: float = 0.0
    source_speed: float = 1.0


@dataclass(frozen=True)
class _CurveCandidate:
    points_xyz: np.ndarray
    control_points_xyz: np.ndarray
    validation: SurfaceValidation


def _normalize(vector: np.ndarray) -> Optional[np.ndarray]:
    norm = float(np.linalg.norm(vector))
    if not math.isfinite(norm) or norm <= DEFAULT_CONFIG.geometry.epsilon:
        return None
    return vector / norm


def _normalize_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def _heading_error(left: float, right: float) -> float:
    return abs(_normalize_angle(left - right))


def _orient_like(points: np.ndarray, reference: np.ndarray) -> np.ndarray:
    direct = float(
        np.linalg.norm(points[0, :2] - reference[0, :2])
        + np.linalg.norm(points[-1, :2] - reference[-1, :2])
    )
    reverse = float(
        np.linalg.norm(points[-1, :2] - reference[0, :2])
        + np.linalg.norm(points[0, :2] - reference[-1, :2])
    )
    return points[::-1].copy() if reverse < direct else points


def _polyline_state(
    points: np.ndarray,
    distance_from_boundary: float,
    *,
    from_end: bool,
) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    oriented = points[::-1] if from_end else points
    segments = np.diff(oriented[:, :2], axis=0)
    lengths = np.linalg.norm(segments, axis=1)
    valid = lengths > DEFAULT_CONFIG.geometry.epsilon
    if not np.any(valid):
        return None

    cumulative = np.concatenate(([0.0], np.cumsum(lengths)))
    distance = float(np.clip(distance_from_boundary, 0.0, float(cumulative[-1])))
    index = int(np.searchsorted(cumulative, distance, side="right") - 1)
    index = max(0, min(index, len(lengths) - 1))
    while lengths[index] <= DEFAULT_CONFIG.geometry.epsilon:
        index += 1
        if index >= len(lengths):
            return None
    ratio = (distance - float(cumulative[index])) / float(lengths[index])
    point = oriented[index] + ratio * (oriented[index + 1] - oriented[index])
    tangent = oriented[index + 1, :2] - oriented[index, :2]
    if from_end:
        tangent = -tangent
    tangent = _normalize(tangent)
    if tangent is None:
        return None
    return point, tangent


def _cross_2d(left: np.ndarray, right: np.ndarray) -> float:
    return float(left[0] * right[1] - left[1] * right[0])


def _ray_segment_distance(
    origin: np.ndarray,
    direction: np.ndarray,
    start: np.ndarray,
    end: np.ndarray,
) -> Optional[float]:
    segment = end - start
    matrix = np.column_stack((direction, -segment))
    determinant = float(np.linalg.det(matrix))
    epsilon = DEFAULT_CONFIG.geometry.epsilon
    if abs(determinant) <= epsilon:
        return None
    distance, ratio = np.linalg.solve(matrix, start - origin)
    if -epsilon <= ratio <= 1.0 + epsilon and distance >= -epsilon:
        return max(0.0, float(distance))
    return None


def _normal_cross_section(
    boundaries: Sequence[np.ndarray],
    distance_from_boundary: float,
    *,
    from_end: bool,
    lane_side: int,
) -> Optional[_CrossSection]:
    if len(boundaries) < 2:
        return None
    state = _polyline_state(
        boundaries[0],
        distance_from_boundary,
        from_end=from_end,
    )
    if state is None:
        return None
    reference, tangent = state
    left_normal = np.array([-tangent[1], tangent[0]], dtype=float)
    side_direction = left_normal if lane_side > 0 else -left_normal

    offsets = [0.0]
    for raw_boundary in boundaries[1:]:
        boundary = _orient_like(raw_boundary, boundaries[0])
        intersections = [
            distance
            for start, end in zip(boundary[:-1, :2], boundary[1:, :2])
            if (
                distance := _ray_segment_distance(
                    reference[:2],
                    side_direction,
                    start,
                    end,
                )
            )
            is not None
        ]
        if not intersections:
            return None
        offsets.append(min(intersections))

    widths = np.diff(np.asarray(offsets, dtype=float))
    if np.any(widths <= DEFAULT_CONFIG.geometry.point_distance_threshold):
        return None
    return _CrossSection(
        reference,
        tangent,
        widths,
        lane_side=lane_side,
    )


def _segments_intersect(
    a0: np.ndarray,
    a1: np.ndarray,
    b0: np.ndarray,
    b1: np.ndarray,
) -> bool:
    epsilon = DEFAULT_CONFIG.geometry.epsilon

    def orientation(p: np.ndarray, q: np.ndarray, r: np.ndarray) -> float:
        return _cross_2d(q - p, r - p)

    o1 = orientation(a0, a1, b0)
    o2 = orientation(a0, a1, b1)
    o3 = orientation(b0, b1, a0)
    o4 = orientation(b0, b1, a1)
    return o1 * o2 < -epsilon and o3 * o4 < -epsilon


def _polyline_self_intersects(points: np.ndarray) -> bool:
    starts = points[:-1]
    ends = points[1:]
    minimum = np.minimum(starts, ends)
    maximum = np.maximum(starts, ends)
    segment_count = len(starts)
    for left_index in range(segment_count):
        candidate_indices = np.arange(left_index + 2, segment_count)
        if left_index == 0 and len(candidate_indices):
            candidate_indices = candidate_indices[
                candidate_indices != segment_count - 1
            ]
        if not len(candidate_indices):
            continue
        overlap = np.all(
            maximum[candidate_indices] + DEFAULT_CONFIG.geometry.epsilon
            >= minimum[left_index],
            axis=1,
        ) & np.all(
            minimum[candidate_indices] - DEFAULT_CONFIG.geometry.epsilon
            <= maximum[left_index],
            axis=1,
        )
        for right_index in candidate_indices[overlap]:
            if _segments_intersect(
                points[left_index],
                points[left_index + 1],
                points[right_index],
                points[right_index + 1],
            ):
                return True
    return False


def _evaluate_lane_offset(road: Road, station: float) -> float:
    if road.lanes is None or not road.lanes.lane_offsets:
        return 0.0
    active = road.lanes.lane_offsets[0]
    for record in road.lanes.lane_offsets:
        if record["s"] <= station + DEFAULT_CONFIG.geometry.epsilon:
            active = record
        else:
            break
    delta = station - active["s"]
    return float(
        active["a"]
        + active["b"] * delta
        + active["c"] * delta * delta
        + active["d"] * delta * delta * delta
    )


def _evaluate_road_reference_pose(
    road: Road,
    station: float,
) -> Optional[Tuple[np.ndarray, float]]:
    station = min(max(0.0, float(station)), road.length)
    if road.emission_context is not None:
        pose = road.emission_context.evaluate(station)
        return np.asarray([pose.x, pose.y], dtype=float), float(pose.heading)
    if road.plan_view is None or not road.plan_view.geometries:
        return None

    geometry = road.plan_view.geometries[-1]
    for candidate in road.plan_view.geometries:
        if station <= candidate.s + candidate.length + DEFAULT_CONFIG.geometry.epsilon:
            geometry = candidate
            break
    parameter = min(max(0.0, station - geometry.s), geometry.length)
    if isinstance(geometry, Arc):
        xy = evaluate_plan_view_world(
            geometry.x,
            geometry.y,
            geometry.hdg,
            parameter,
            arc_curvature=geometry.curvature,
        )
        heading = geometry.hdg + geometry.curvature * parameter
    elif isinstance(geometry, ParamPoly3):
        parameter_scale = 1.0
        if geometry.pRange == "normalized":
            parameter_scale = 1.0 / geometry.length
            parameter *= parameter_scale
        xy = evaluate_plan_view_world(
            geometry.x,
            geometry.y,
            geometry.hdg,
            parameter,
            (
                geometry.aU,
                geometry.bU,
                geometry.cU,
                geometry.dU,
                geometry.aV,
                geometry.bV,
                geometry.cV,
                geometry.dV,
            ),
        )
        du = (
            geometry.bU
            + 2.0 * geometry.cU * parameter
            + 3.0 * geometry.dU * parameter * parameter
        ) * parameter_scale
        dv = (
            geometry.bV
            + 2.0 * geometry.cV * parameter
            + 3.0 * geometry.dV * parameter * parameter
        ) * parameter_scale
        heading = geometry.hdg + math.atan2(dv, du)
    else:
        xy = evaluate_plan_view_world(
            geometry.x,
            geometry.y,
            geometry.hdg,
            parameter,
        )
        heading = geometry.hdg
    return np.asarray(xy, dtype=float), float(heading)


def _single_lane_surface_is_valid(road: Road, lane: Lane) -> bool:
    constants = DEFAULT_CONFIG.junction_emission
    sample_count = max(
        constants.minimum_curve_samples,
        int(
            math.ceil(
                max(road.length, constants.curve_sample_interval)
                / constants.curve_sample_interval
            )
        )
        + 1,
    )
    stations = np.linspace(0.0, road.length, sample_count)
    inner_points = []
    outer_points = []
    side = 1.0 if lane.lane_id is not None and lane.lane_id > 0 else -1.0
    for station in stations:
        pose = _evaluate_road_reference_pose(road, float(station))
        width = _evaluate_lane_width(lane, float(station))
        if pose is None or width is None:
            return False
        reference, heading = pose
        lane_offset = _evaluate_lane_offset(road, float(station))
        normal = np.asarray([-math.sin(heading), math.cos(heading)], dtype=float)
        inner = reference + lane_offset * normal
        outer = inner + side * float(width) * normal
        if (
            not np.all(np.isfinite(inner))
            or not np.all(np.isfinite(outer))
            or width <= DEFAULT_CONFIG.geometry.point_distance_threshold
        ):
            return False
        inner_points.append(inner)
        outer_points.append(outer)

    inner = np.asarray(inner_points)
    outer = np.asarray(outer_points)
    polygon_boundary = np.vstack((inner, outer[::-1], inner[:1]))
    return (
        not _polyline_self_intersects(inner)
        and not _polyline_self_intersects(outer)
        and not _polyline_self_intersects(polygon_boundary)
    )


def _bezier_arc_length(control_points_xy: np.ndarray) -> float:
    order = DEFAULT_CONFIG.junction_emission.arc_length_quadrature_order
    nodes, weights = np.polynomial.legendre.leggauss(order)
    u = (nodes + 1.0) * 0.5
    p0, p1, p2, p3 = control_points_xy
    derivative = (
        3.0 * (1.0 - u)[:, None] ** 2 * (p1 - p0)
        + 6.0 * (1.0 - u)[:, None] * u[:, None] * (p2 - p1)
        + 3.0 * u[:, None] ** 2 * (p3 - p2)
    )
    return float(0.5 * np.sum(weights * np.linalg.norm(derivative, axis=1)))


def _bezier_candidate(
    start: _CrossSection,
    end: _CrossSection,
    start_scale: float,
    end_scale: float,
) -> _CurveCandidate:
    constants = DEFAULT_CONFIG.junction_emission
    p0 = start.reference_xyz[:2]
    p3 = end.reference_xyz[:2]
    chord_length = float(np.linalg.norm(p3 - p0))
    p1 = p0 + max(constants.minimum_tangent_extent, start_scale * chord_length) * (
        start.tangent
    )
    p2 = p3 - max(constants.minimum_tangent_extent, end_scale * chord_length) * (
        end.tangent
    )
    control_points_xy = np.asarray([p0, p1, p2, p3], dtype=float)

    sample_count = max(
        constants.minimum_curve_samples,
        int(
            math.ceil(
                max(chord_length, constants.curve_sample_interval)
                / constants.curve_sample_interval
            )
        )
        + 1,
    )
    u = np.linspace(0.0, 1.0, sample_count)[:, None]
    reference = (
        (1.0 - u) ** 3 * p0
        + 3.0 * (1.0 - u) ** 2 * u * p1
        + 3.0 * (1.0 - u) * u**2 * p2
        + u**3 * p3
    )
    first = (
        3.0 * (1.0 - u) ** 2 * (p1 - p0)
        + 6.0 * (1.0 - u) * u * (p2 - p1)
        + 3.0 * u**2 * (p3 - p2)
    )
    second = 6.0 * (1.0 - u) * (p2 - 2.0 * p1 + p0) + 6.0 * u * (p3 - 2.0 * p2 + p1)
    speed = np.linalg.norm(first, axis=1)
    safe_speed = np.maximum(speed, DEFAULT_CONFIG.geometry.epsilon)
    curvature = (
        np.abs(first[:, 0] * second[:, 1] - first[:, 1] * second[:, 0]) / safe_speed**3
    )
    normals = np.column_stack((-first[:, 1], first[:, 0])) / safe_speed[:, None]
    segment_lengths = np.linalg.norm(np.diff(reference, axis=0), axis=1)
    reference_length = float(np.sum(segment_lengths))

    smooth = 3.0 * u[:, 0] ** 2 - 2.0 * u[:, 0] ** 3
    widths = (
        start.widths[None, :] * (1.0 - smooth[:, None])
        + end.widths[None, :] * smooth[:, None]
    )
    parameter = u[:, 0]
    lane_offsets = (
        (2.0 * parameter**3 - 3.0 * parameter**2 + 1.0) * start.lane_offset
        + (parameter**3 - 2.0 * parameter**2 + parameter)
        * reference_length
        * start.lane_offset_derivative
        + (-2.0 * parameter**3 + 3.0 * parameter**2) * end.lane_offset
        + (parameter**3 - parameter**2) * reference_length * end.lane_offset_derivative
    )
    if start.lane_side != end.lane_side:
        raise ValueError("junction connector lane block changes side")
    offsets = lane_offsets[:, None] + start.lane_side * np.column_stack(
        (np.zeros(sample_count), np.cumsum(widths, axis=1))
    )
    boundaries = [
        reference + offsets[:, index, None] * normals
        for index in range(widths.shape[1] + 1)
    ]

    max_offset = np.max(np.abs(offsets), axis=1)
    max_ratio = float(np.max(curvature * max_offset))
    basic_finite_width = (
        bool(np.all(np.isfinite(reference)))
        and float(np.min(speed)) > DEFAULT_CONFIG.geometry.epsilon
        and max_ratio < 1.0
        and reference_length > DEFAULT_CONFIG.geometry.point_distance_threshold
    )
    center_self_intersection = (
        _polyline_self_intersects(reference) if basic_finite_width else False
    )
    boundary_self_intersection = (
        any(_polyline_self_intersects(boundary) for boundary in boundaries)
        if basic_finite_width
        else False
    )
    boundary_reversal = False
    for index in range(len(boundaries) - 1):
        lateral = boundaries[index + 1] - boundaries[index]
        signed = np.sum(lateral * (start.lane_side * normals), axis=1)
        if np.any(signed <= DEFAULT_CONFIG.geometry.point_distance_threshold):
            boundary_reversal = True
            break

    headings = np.arctan2(
        np.diff(reference, axis=0)[:, 1], np.diff(reference, axis=0)[:, 0]
    )
    internal_heading_change = max(
        (
            _heading_error(float(left), float(right))
            for left, right in zip(headings[:-1], headings[1:])
        ),
        default=0.0,
    )
    start_heading = math.atan2(first[0, 1], first[0, 0])
    end_heading = math.atan2(first[-1, 1], first[-1, 0])
    c1 = (
        _heading_error(start_heading, math.atan2(start.tangent[1], start.tangent[0]))
        <= constants.heading_tolerance
        and _heading_error(end_heading, math.atan2(end.tangent[1], end.tangent[0]))
        <= constants.heading_tolerance
    )
    finite_width_valid = (
        basic_finite_width
        and not center_self_intersection
        and not boundary_self_intersection
        and not boundary_reversal
        and max_ratio < 1.0
        and reference_length > DEFAULT_CONFIG.geometry.point_distance_threshold
    )

    z = start.reference_xyz[2] * (1.0 - u[:, 0]) + end.reference_xyz[2] * u[:, 0]
    points_xyz = np.column_stack((reference, z))
    z_controls = np.linspace(
        start.reference_xyz[2],
        end.reference_xyz[2],
        4,
    )
    control_points_xyz = np.column_stack((control_points_xy, z_controls))
    return _CurveCandidate(
        points_xyz=points_xyz,
        control_points_xyz=control_points_xyz,
        validation=SurfaceValidation(
            c0=True,
            c1=c1,
            finite_width_valid=finite_width_valid,
            center_self_intersection=center_self_intersection,
            boundary_self_intersection=boundary_self_intersection,
            boundary_reversal=boundary_reversal,
            max_abs_curvature=float(np.max(curvature)),
            max_kappa_half_width=max_ratio,
            reference_length=reference_length,
            max_internal_heading_change=internal_heading_change,
        ),
    )


def _best_curve_candidate(
    start: _CrossSection,
    end: _CrossSection,
) -> Optional[_CurveCandidate]:
    constants = DEFAULT_CONFIG.junction_emission
    best: Optional[Tuple[Tuple[float, ...], float, float, _CurveCandidate]] = None

    def evaluate(scales: np.ndarray) -> None:
        nonlocal best
        for start_scale in scales:
            for end_scale in scales:
                candidate = _bezier_candidate(
                    start,
                    end,
                    float(start_scale),
                    float(end_scale),
                )
                validation = candidate.validation
                key = (
                    0.0 if validation.finite_width_valid else 1.0,
                    0.0
                    if validation.max_kappa_half_width
                    <= constants.preferred_max_kappa_half_width
                    else 1.0,
                    validation.max_kappa_half_width,
                    validation.max_abs_curvature,
                    validation.reference_length,
                )
                record = (
                    key,
                    float(start_scale),
                    float(end_scale),
                    candidate,
                )
                if best is None or key < best[0]:
                    best = record

    coarse_scales = np.linspace(
        constants.control_scale_min,
        constants.control_scale_max,
        constants.control_scale_coarse_steps,
    )
    evaluate(coarse_scales)
    if best is None:
        return None

    coarse_step = (constants.control_scale_max - constants.control_scale_min) / max(
        1, constants.control_scale_coarse_steps - 1
    )
    start_scales = np.linspace(
        max(constants.control_scale_min, best[1] - coarse_step),
        min(constants.control_scale_max, best[1] + coarse_step),
        constants.control_scale_refine_steps,
    )
    end_scales = np.linspace(
        max(constants.control_scale_min, best[2] - coarse_step),
        min(constants.control_scale_max, best[2] + coarse_step),
        constants.control_scale_refine_steps,
    )
    for start_scale in start_scales:
        for end_scale in end_scales:
            candidate = _bezier_candidate(
                start,
                end,
                float(start_scale),
                float(end_scale),
            )
            validation = candidate.validation
            key = (
                0.0 if validation.finite_width_valid else 1.0,
                0.0
                if validation.max_kappa_half_width
                <= constants.preferred_max_kappa_half_width
                else 1.0,
                validation.max_kappa_half_width,
                validation.max_abs_curvature,
                validation.reference_length,
            )
            if best is None or key < best[0]:
                best = (
                    key,
                    float(start_scale),
                    float(end_scale),
                    candidate,
                )
    return best[3]


def search_junction_cutback(
    incoming_cross_section: Callable[[float], Optional[_CrossSection]],
    outgoing_cross_section: Callable[[float], Optional[_CrossSection]],
) -> Optional[Tuple[float, float, _CurveCandidate, _CrossSection, _CrossSection]]:
    """Find the smallest junction-wide cutback with a valid finite-width surface."""
    constants = DEFAULT_CONFIG.junction_emission
    first_regular_outgoing: Optional[float] = None
    for distance in np.arange(
        0.0,
        constants.max_cutback + constants.cutback_step / 2.0,
        constants.cutback_step,
    ):
        if outgoing_cross_section(float(distance)) is not None:
            first_regular_outgoing = float(distance)
            break
    if first_regular_outgoing is None:
        return None

    fallback: Optional[
        Tuple[float, float, _CurveCandidate, _CrossSection, _CrossSection]
    ] = None
    for total in np.arange(
        first_regular_outgoing,
        constants.max_total_cutback + constants.cutback_step / 2.0,
        constants.cutback_step,
    ):
        regular_candidates = []
        max_incoming = total - first_regular_outgoing
        for incoming_distance in np.arange(
            0.0,
            max_incoming + constants.cutback_step / 2.0,
            constants.cutback_step,
        ):
            outgoing_distance = float(total - incoming_distance)
            incoming = incoming_cross_section(float(incoming_distance))
            outgoing = outgoing_cross_section(outgoing_distance)
            if incoming is None or outgoing is None:
                continue
            if len(incoming.widths) != len(outgoing.widths):
                continue
            cross_section_heading_tolerance = (
                DEFAULT_CONFIG.geometry.terminal_micro_kink_support_heading_tolerance
            )
            if (
                _cross_section_boundary_heading_spread(incoming)
                > cross_section_heading_tolerance
                or _cross_section_boundary_heading_spread(outgoing)
                > cross_section_heading_tolerance
            ):
                continue
            displacement = outgoing.reference_xyz[:2] - incoming.reference_xyz[:2]
            if (
                float(np.dot(displacement, incoming.tangent))
                <= DEFAULT_CONFIG.geometry.point_distance_threshold
                or float(np.dot(displacement, outgoing.tangent))
                <= DEFAULT_CONFIG.geometry.point_distance_threshold
            ):
                continue
            candidate = _best_curve_candidate(incoming, outgoing)
            if candidate is None:
                continue
            validation = candidate.validation
            lane_center_heading_gap = _candidate_lane_center_heading_error(
                incoming,
                outgoing,
                candidate,
            )
            record = (
                float(incoming_distance),
                outgoing_distance,
                candidate,
                incoming,
                outgoing,
            )
            if (
                validation.finite_width_valid
                and validation.c1
                and lane_center_heading_gap <= constants.lane_center_heading_tolerance
            ):
                regular_candidates.append(record)
                if (
                    validation.max_kappa_half_width
                    <= constants.preferred_max_kappa_half_width
                ):
                    return record
        if regular_candidates and fallback is None:
            fallback = min(
                regular_candidates,
                key=lambda record: (
                    record[2].validation.max_kappa_half_width,
                    record[2].validation.max_abs_curvature,
                ),
            )
    return fallback


def _lane_subtype(lanelet: lanelet2.core.Lanelet) -> str:
    return str(lanelet.attributes["subtype"]) if "subtype" in lanelet.attributes else ""


def _source_boundaries_for_maneuvers(
    maneuvers: Sequence[LogicalManeuver],
    lanelet_by_id: Dict[int, lanelet2.core.Lanelet],
    *,
    incoming: bool,
) -> Optional[Tuple[List[np.ndarray], int]]:
    lanes = [
        maneuver.incoming if incoming else maneuver.outgoing for maneuver in maneuvers
    ]
    lane_ids = [lane.lane_id for lane in lanes]
    if not lane_ids or any(lane_id == 0 for lane_id in lane_ids):
        return None
    lane_side = 1 if lane_ids[0] > 0 else -1
    if any((lane_id > 0) != (lane_side > 0) for lane_id in lane_ids):
        return None
    ordered = sorted(lanes, key=lambda lane: abs(lane.lane_id))
    ordered_ids = [abs(lane.lane_id) for lane in ordered]
    if ordered_ids != list(range(ordered_ids[0], ordered_ids[0] + len(ordered_ids))):
        return None

    lanelets = [lanelet_by_id.get(lane.lanelet_id) for lane in ordered]
    if any(lanelet is None for lanelet in lanelets):
        return None
    typed_lanelets = [lanelet for lanelet in lanelets if lanelet is not None]
    first = typed_lanelets[0]
    reference = first.rightBound if lane_side > 0 else first.leftBound
    outer_boundaries = [
        lanelet.leftBound if lane_side > 0 else lanelet.rightBound
        for lanelet in typed_lanelets
    ]
    reference_points = extract_points_3d(reference)
    boundaries = [reference_points]
    boundaries.extend(
        _orient_like(extract_points_3d(boundary), reference_points)
        for boundary in outer_boundaries
    )
    return boundaries, lane_side


def _road_lanelet_maps(
    roads: Iterable[Road],
) -> Tuple[Dict[int, Dict[int, int]], Dict[int, Dict[int, int]]]:
    lanelet_to_lane = {road.id: road.get_lanelet_to_lane_mapping() for road in roads}
    lane_to_lanelet = {
        road_id: {lane_id: lanelet_id for lanelet_id, lane_id in mapping.items()}
        for road_id, mapping in lanelet_to_lane.items()
    }
    return lanelet_to_lane, lane_to_lanelet


def _external_direct_maneuvers(
    *,
    junction: Junction,
    lanelet_map: lanelet2.core.LaneletMap,
    routing_graph,
    roads_by_id: Dict[int, Road],
    lanelet_to_road_id: Dict[int, int],
    junction_lanelet_ids: Set[int],
) -> Dict[Tuple[int, int], List[LogicalManeuver]]:
    lanelet_to_lane, lane_to_lanelet = _road_lanelet_maps(roads_by_id.values())
    incoming_road_ids = {
        connection.incoming_road for connection in junction.connections
    }
    connecting_road_ids = {
        connection.connecting_road for connection in junction.connections
    }
    groups: Dict[Tuple[int, int], List[LogicalManeuver]] = {}

    for incoming_road_id in sorted(incoming_road_ids):
        for incoming_lane_id, incoming_lanelet_id in sorted(
            lane_to_lanelet.get(incoming_road_id, {}).items()
        ):
            if not lanelet_map.laneletLayer.exists(incoming_lanelet_id):
                continue
            incoming_lanelet = lanelet_map.laneletLayer.get(incoming_lanelet_id)
            for outgoing_lanelet in routing_graph.following(incoming_lanelet):
                if outgoing_lanelet.id in junction_lanelet_ids:
                    continue
                outgoing_road_id = lanelet_to_road_id.get(outgoing_lanelet.id)
                if (
                    outgoing_road_id is None
                    or outgoing_road_id == incoming_road_id
                    or outgoing_road_id in connecting_road_ids
                    or outgoing_road_id not in roads_by_id
                ):
                    continue
                outgoing_lane_id = lanelet_to_lane.get(outgoing_road_id, {}).get(
                    outgoing_lanelet.id
                )
                if outgoing_lane_id is None:
                    continue
                maneuver = LogicalManeuver(
                    incoming=LogicalLane(
                        lanelet_id=incoming_lanelet_id,
                        road_id=incoming_road_id,
                        lane_id=incoming_lane_id,
                        subtype=_lane_subtype(incoming_lanelet),
                    ),
                    outgoing=LogicalLane(
                        lanelet_id=outgoing_lanelet.id,
                        road_id=outgoing_road_id,
                        lane_id=outgoing_lane_id,
                        subtype=_lane_subtype(outgoing_lanelet),
                    ),
                )
                groups.setdefault((incoming_road_id, outgoing_road_id), []).append(
                    maneuver
                )
    return groups


def _logical_junction_maneuvers(
    *,
    junction: Junction,
    lanelet_map: lanelet2.core.LaneletMap,
    routing_graph,
    roads_by_id: Dict[int, Road],
    lanelet_to_road_id: Dict[int, int],
) -> List[LogicalManeuver]:
    lanelet_to_lane, lane_to_lanelet = _road_lanelet_maps(roads_by_id.values())
    maneuvers: Dict[Tuple[int, int], LogicalManeuver] = {}
    for incoming_road_id in sorted(
        {connection.incoming_road for connection in junction.connections}
    ):
        for incoming_lane_id, incoming_lanelet_id in sorted(
            lane_to_lanelet.get(incoming_road_id, {}).items()
        ):
            if not lanelet_map.laneletLayer.exists(incoming_lanelet_id):
                continue
            incoming_lanelet = lanelet_map.laneletLayer.get(incoming_lanelet_id)
            for outgoing_lanelet in routing_graph.following(incoming_lanelet):
                outgoing_road_id = lanelet_to_road_id.get(outgoing_lanelet.id)
                if (
                    outgoing_road_id is None
                    or outgoing_road_id == incoming_road_id
                    or outgoing_road_id not in roads_by_id
                ):
                    continue
                outgoing_lane_id = lanelet_to_lane.get(outgoing_road_id, {}).get(
                    outgoing_lanelet.id
                )
                if outgoing_lane_id is None:
                    continue
                maneuver = LogicalManeuver(
                    incoming=LogicalLane(
                        lanelet_id=incoming_lanelet_id,
                        road_id=incoming_road_id,
                        lane_id=incoming_lane_id,
                        subtype=_lane_subtype(incoming_lanelet),
                    ),
                    outgoing=LogicalLane(
                        lanelet_id=outgoing_lanelet.id,
                        road_id=outgoing_road_id,
                        lane_id=outgoing_lane_id,
                        subtype=_lane_subtype(outgoing_lanelet),
                    ),
                )
                maneuvers[maneuver.lanelet_pair] = maneuver
    return sorted(
        maneuvers.values(),
        key=lambda maneuver: (
            maneuver.incoming.road_id,
            maneuver.incoming.lane_id,
            maneuver.outgoing.road_id,
            maneuver.outgoing.lane_id,
        ),
    )


def _source_backed_connection_pairs(
    junction: Junction,
    roads_by_id: Dict[int, Road],
) -> Set[Tuple[int, int]]:
    _, lane_to_lanelet = _road_lanelet_maps(roads_by_id.values())
    pairs: Set[Tuple[int, int]] = set()
    for connection in junction.connections:
        connecting = roads_by_id.get(connection.connecting_road)
        if connecting is None or not connecting.get_lanelet_to_lane_mapping():
            continue
        for lane_link in connection.lane_links:
            incoming_lanelet_id = lane_to_lanelet.get(connection.incoming_road, {}).get(
                lane_link.from_lane
            )
            outgoing_lanelet_id = lane_to_lanelet.get(
                connection.connecting_road, {}
            ).get(lane_link.to_lane)
            if incoming_lanelet_id is not None and outgoing_lanelet_id is not None:
                pairs.add((incoming_lanelet_id, outgoing_lanelet_id))
    return pairs


def _connector_endpoints(road: Road) -> Optional[Tuple[int, int]]:
    if (
        road.link is None
        or road.link.predecessor is None
        or road.link.successor is None
        or road.link.predecessor.element_type is not ElementType.ROAD
        or road.link.successor.element_type is not ElementType.ROAD
    ):
        return None
    return (road.link.predecessor.element_id, road.link.successor.element_id)


def _is_source_less_connector(road: Road, junction_id: int) -> bool:
    return road.junction == junction_id and not road.get_lanelet_to_lane_mapping()


def _dummy_center_lane() -> ReferenceLine:
    center = ReferenceLine.__new__(ReferenceLine)
    center._lane = Lane(lane_id=0, lane_type=LaneType.NONE, level=False)
    return center


def _bezier_plan_view(control_points_xyz: np.ndarray) -> Tuple[PlanView, float]:
    if control_points_xyz.shape != (4, 3):
        raise ValueError("junction emission Bezier requires four 3D control points")
    control_points_xy = control_points_xyz[:, :2]
    length = _bezier_arc_length(control_points_xy)
    if length <= DEFAULT_CONFIG.geometry.point_distance_threshold:
        raise ValueError("junction emission curve produced no geometry")

    start = control_points_xy[0]
    tangent = control_points_xy[1] - start
    normalized_tangent = _normalize(tangent)
    if normalized_tangent is None:
        raise ValueError("junction emission curve has no start tangent")
    heading = math.atan2(normalized_tangent[1], normalized_tangent[0])
    cos_heading = math.cos(heading)
    sin_heading = math.sin(heading)
    rotation = np.array(
        [[cos_heading, sin_heading], [-sin_heading, cos_heading]],
        dtype=float,
    )
    local = (control_points_xy - start) @ rotation.T
    p0, p1, p2, p3 = local
    normalized_coefficients = (
        p0,
        3.0 * (p1 - p0),
        3.0 * (p2 - 2.0 * p1 + p0),
        p3 - 3.0 * p2 + 3.0 * p1 - p0,
    )
    a, b, c, d = normalized_coefficients
    geometry = ParamPoly3(
        s=0.0,
        x=float(start[0]),
        y=float(start[1]),
        hdg=heading,
        length=length,
        aU=float(a[0]),
        bU=float(b[0] / length),
        cU=float(c[0] / (length * length)),
        dU=float(d[0] / (length * length * length)),
        aV=float(a[1]),
        bV=float(b[1] / length),
        cV=float(c[1] / (length * length)),
        dV=float(d[1] / (length * length * length)),
        pRange="arcLength",
    )
    return PlanView(geometries=[geometry]), length


def _smooth_width_records(
    start_width: float,
    end_width: float,
    length: float,
    start_derivative: float = 0.0,
    end_derivative: float = 0.0,
) -> List[LaneWidth]:
    delta = end_width - start_width
    return [
        LaneWidth(
            s_offset=0.0,
            a=float(start_width),
            b=float(start_derivative),
            c=float(
                (3.0 * delta - (2.0 * start_derivative + end_derivative) * length)
                / (length * length)
            ),
            d=float(
                (-2.0 * delta + (start_derivative + end_derivative) * length)
                / (length * length * length)
            ),
        )
    ]


def _build_multi_lane_connector(
    *,
    road_id: int,
    junction_id: int,
    incoming_road_id: int,
    outgoing_road_id: int,
    maneuvers: Sequence[LogicalManeuver],
    connector_lane_ids: Sequence[int],
    curve_control_points_xyz: np.ndarray,
    start_widths: Sequence[float],
    end_widths: Sequence[float],
    start_width_derivatives: Optional[Sequence[float]] = None,
    end_width_derivatives: Optional[Sequence[float]] = None,
    start_lane_offset: float = 0.0,
    end_lane_offset: float = 0.0,
    start_lane_offset_derivative: float = 0.0,
    end_lane_offset_derivative: float = 0.0,
    traffic_rule: TrafficRule,
) -> Road:
    plan_view, length = _bezier_plan_view(curve_control_points_xyz)
    if start_width_derivatives is None:
        start_width_derivatives = [0.0] * len(start_widths)
    if end_width_derivatives is None:
        end_width_derivatives = [0.0] * len(end_widths)
    lane_section = LaneSection(s_offset=0.0)
    lane_section.center_lane = _dummy_center_lane()
    lane_offsets: List[Dict[str, float]] = []
    if any(
        abs(value) > DEFAULT_CONFIG.geometry.epsilon
        for value in (
            start_lane_offset,
            end_lane_offset,
            start_lane_offset_derivative,
            end_lane_offset_derivative,
        )
    ):
        lane_offset = _smooth_width_records(
            start_lane_offset,
            end_lane_offset,
            length,
            start_lane_offset_derivative,
            end_lane_offset_derivative,
        )[0]
        lane_offsets.append(
            {
                "s": lane_offset.s_offset,
                "a": lane_offset.a,
                "b": lane_offset.b,
                "c": lane_offset.c,
                "d": lane_offset.d,
            }
        )
    for (
        maneuver,
        connector_lane_id,
        start_width,
        end_width,
        start_derivative,
        end_derivative,
    ) in zip(
        maneuvers,
        connector_lane_ids,
        start_widths,
        end_widths,
        start_width_derivatives,
        end_width_derivatives,
    ):
        lane = Lane(
            lane_id=connector_lane_id,
            lane_type=LaneType.DRIVING,
            predecessor=LaneLink(id=maneuver.incoming.lane_id),
            successor=LaneLink(id=maneuver.outgoing.lane_id),
            rule=traffic_rule.value,
        )
        lane.widths = _smooth_width_records(
            start_width,
            end_width,
            length,
            start_derivative,
            end_derivative,
        )
        if connector_lane_id > 0:
            lane_section.left_lanes[connector_lane_id] = lane
        else:
            lane_section.right_lanes[connector_lane_id] = lane

    z_start = float(curve_control_points_xyz[0, 2])
    z_end = float(curve_control_points_xyz[-1, 2])
    elevation = ElevationProfile(
        elevations=[
            Elevation(
                s=0.0,
                a=z_start,
                b=(z_end - z_start) / length,
                c=0.0,
                d=0.0,
            )
        ]
    )
    return Road(
        id=road_id,
        length=length,
        junction=junction_id,
        rule=traffic_rule,
        plan_view=plan_view,
        elevation_profile=elevation,
        lanes=Lanes(
            lane_sections=[lane_section],
            lane_offsets=lane_offsets,
        ),
        link=RoadLink(
            predecessor=Predecessor(
                element_type=ElementType.ROAD,
                element_id=incoming_road_id,
                contact_point=ContactPoint.END,
            ),
            successor=Successor(
                element_type=ElementType.ROAD,
                element_id=outgoing_road_id,
                contact_point=ContactPoint.START,
            ),
        ),
        reference_start_xyz=tuple(
            float(value) for value in curve_control_points_xyz[0]
        ),
        reference_end_xyz=tuple(float(value) for value in curve_control_points_xyz[-1]),
    )


def _connector_lane_ids_for_contiguous_maneuvers(
    incoming_ids: Sequence[int],
    outgoing_ids: Sequence[int],
) -> Optional[Tuple[int, ...]]:
    """Return canonical connector IDs for a contiguous same-side lane block."""
    if len(incoming_ids) < 2 or len(incoming_ids) != len(outgoing_ids):
        return None
    incoming_side = 1 if incoming_ids[0] > 0 else -1
    outgoing_side = 1 if outgoing_ids[0] > 0 else -1
    if incoming_side != outgoing_side:
        return None
    if any((1 if lane_id > 0 else -1) != incoming_side for lane_id in incoming_ids):
        return None
    if any((1 if lane_id > 0 else -1) != outgoing_side for lane_id in outgoing_ids):
        return None

    incoming_abs = [abs(lane_id) for lane_id in incoming_ids]
    outgoing_abs = [abs(lane_id) for lane_id in outgoing_ids]
    if incoming_abs != list(range(1, len(incoming_abs) + 1)):
        return None
    if outgoing_abs != list(
        range(outgoing_abs[0], outgoing_abs[0] + len(outgoing_abs))
    ):
        return None
    return tuple(incoming_side * index for index in range(1, len(incoming_ids) + 1))


def canonicalize_junction_emission(
    *,
    lanelet_map: lanelet2.core.LaneletMap,
    routing_graph,
    regular_roads: List[Road],
    connecting_roads: List[Road],
    junctions: List[Junction],
    lanelet_to_road_id: Dict[int, int],
    junction_lanelet_ids: Set[int],
    traffic_rule: TrafficRule,
    starting_road_id: int,
) -> Tuple[List[Road], List[JunctionEmissionPlan], int]:
    """Canonicalize feasible parallel direct continuations junction by junction."""
    all_roads = regular_roads + connecting_roads
    roads_by_id = {road.id: road for road in all_roads}
    lanelet_by_id = {lanelet.id: lanelet for lanelet in lanelet_map.laneletLayer}
    next_road_id = starting_road_id
    plans: List[JunctionEmissionPlan] = []

    for junction in junctions:
        junction_logical_maneuvers = _logical_junction_maneuvers(
            junction=junction,
            lanelet_map=lanelet_map,
            routing_graph=routing_graph,
            roads_by_id=roads_by_id,
            lanelet_to_road_id=lanelet_to_road_id,
        )
        direct_groups = _external_direct_maneuvers(
            junction=junction,
            lanelet_map=lanelet_map,
            routing_graph=routing_graph,
            roads_by_id=roads_by_id,
            lanelet_to_road_id=lanelet_to_road_id,
            junction_lanelet_ids=junction_lanelet_ids,
        )
        planned_groups: List[ConnectingRoadGroup] = []

        for (incoming_road_id, outgoing_road_id), raw_maneuvers in sorted(
            direct_groups.items()
        ):
            maneuvers = sorted(
                {
                    maneuver.lanelet_pair: maneuver for maneuver in raw_maneuvers
                }.values(),
                key=lambda maneuver: abs(maneuver.incoming.lane_id),
            )
            if len(maneuvers) < 2:
                continue
            incoming_ids = [maneuver.incoming.lane_id for maneuver in maneuvers]
            outgoing_ids = [maneuver.outgoing.lane_id for maneuver in maneuvers]
            connector_lane_ids = _connector_lane_ids_for_contiguous_maneuvers(
                incoming_ids,
                outgoing_ids,
            )
            if connector_lane_ids is None:
                continue

            incoming_boundaries = _source_boundaries_for_maneuvers(
                maneuvers,
                lanelet_by_id,
                incoming=True,
            )
            outgoing_boundaries = _source_boundaries_for_maneuvers(
                maneuvers,
                lanelet_by_id,
                incoming=False,
            )
            if incoming_boundaries is None or outgoing_boundaries is None:
                continue
            incoming_lines, incoming_side = incoming_boundaries
            outgoing_lines, outgoing_side = outgoing_boundaries
            if incoming_side != outgoing_side:
                continue

            result = search_junction_cutback(
                lambda distance: _normal_cross_section(
                    incoming_lines,
                    distance,
                    from_end=True,
                    lane_side=incoming_side,
                ),
                lambda distance: _normal_cross_section(
                    outgoing_lines,
                    distance,
                    from_end=False,
                    lane_side=outgoing_side,
                ),
            )
            if result is None:
                continue
            incoming_distance, outgoing_distance, curve, incoming, outgoing = result

            replaced_ids = tuple(
                sorted(
                    road.id
                    for road in connecting_roads
                    if _is_source_less_connector(road, junction.id)
                    and _connector_endpoints(road)
                    == (incoming_road_id, outgoing_road_id)
                )
            )
            connector_road_id = replaced_ids[0] if replaced_ids else next_road_id
            if not replaced_ids:
                next_road_id += 1

            group = ConnectingRoadGroup(
                incoming_road_id=incoming_road_id,
                outgoing_road_id=outgoing_road_id,
                connector_road_id=connector_road_id,
                maneuvers=tuple(maneuvers),
                connector_lane_ids=connector_lane_ids,
                replaced_connector_road_ids=replaced_ids,
                incoming_cut=CutSection(
                    road_id=incoming_road_id,
                    station_from_boundary=incoming_distance,
                    reference_xyz=tuple(
                        float(value) for value in incoming.reference_xyz
                    ),
                    heading=math.atan2(incoming.tangent[1], incoming.tangent[0]),
                    lane_widths=tuple(float(value) for value in incoming.widths),
                ),
                outgoing_cut=CutSection(
                    road_id=outgoing_road_id,
                    station_from_boundary=outgoing_distance,
                    reference_xyz=tuple(
                        float(value) for value in outgoing.reference_xyz
                    ),
                    heading=math.atan2(outgoing.tangent[1], outgoing.tangent[0]),
                    lane_widths=tuple(float(value) for value in outgoing.widths),
                ),
                curve_points_xyz=curve.points_xyz,
                curve_control_points_xyz=curve.control_points_xyz,
                surface=curve.validation,
            )
            planned_groups.append(group)

        if not planned_groups:
            continue

        logical_pairs = {
            maneuver.lanelet_pair for maneuver in junction_logical_maneuvers
        }
        emitted_pairs = _source_backed_connection_pairs(junction, roads_by_id)
        emitted_pairs.update(
            maneuver.lanelet_pair
            for group in planned_groups
            for maneuver in group.maneuvers
        )
        missing_maneuvers = tuple(sorted(logical_pairs - emitted_pairs))
        unintended_maneuvers = tuple(sorted(emitted_pairs - logical_pairs))
        if missing_maneuvers or unintended_maneuvers:
            continue

        removed_ids = {
            road_id
            for group in planned_groups
            for road_id in group.replaced_connector_road_ids
        }
        connecting_roads[:] = [
            road for road in connecting_roads if road.id not in removed_ids
        ]
        junction.connections = [
            connection
            for connection in junction.connections
            if connection.connecting_road not in removed_ids
        ]

        for group in planned_groups:
            connector = _build_multi_lane_connector(
                road_id=group.connector_road_id,
                junction_id=junction.id,
                incoming_road_id=group.incoming_road_id,
                outgoing_road_id=group.outgoing_road_id,
                maneuvers=group.maneuvers,
                connector_lane_ids=group.connector_lane_ids,
                curve_control_points_xyz=group.curve_control_points_xyz,
                start_widths=group.incoming_cut.lane_widths,
                end_widths=group.outgoing_cut.lane_widths,
                traffic_rule=traffic_rule,
            )
            connecting_roads.append(connector)
            roads_by_id[connector.id] = connector
            junction.connections.append(
                Connection(
                    id=0,
                    incoming_road=group.incoming_road_id,
                    connecting_road=group.connector_road_id,
                    contact_point=ContactPoint.START,
                    lane_links=[
                        JunctionLaneLink(
                            from_lane=maneuver.incoming.lane_id,
                            to_lane=connector_lane_id,
                        )
                        for maneuver, connector_lane_id in zip(
                            group.maneuvers,
                            group.connector_lane_ids,
                        )
                    ],
                )
            )
        for connection_id, connection in enumerate(
            sorted(
                junction.connections,
                key=lambda connection: (
                    connection.incoming_road,
                    connection.connecting_road,
                ),
            )
        ):
            connection.id = connection_id
        junction.connections.sort(key=lambda connection: connection.id)

        traces = tuple(
            LaneTrace(
                lanelet_id=maneuver.incoming.lanelet_id,
                emitted_segments=(
                    EmittedLaneSegment(
                        road_id=maneuver.incoming.road_id,
                        lane_id=maneuver.incoming.lane_id,
                        role="source",
                    ),
                    EmittedLaneSegment(
                        road_id=group.connector_road_id,
                        lane_id=connector_lane_id,
                        role="junction_connector",
                    ),
                ),
            )
            for group in planned_groups
            for maneuver, connector_lane_id in zip(
                group.maneuvers,
                group.connector_lane_ids,
            )
        )
        plans.append(
            JunctionEmissionPlan(
                junction_id=junction.id,
                logical_incoming_lanes=tuple(
                    dict.fromkeys(
                        maneuver.incoming for maneuver in junction_logical_maneuvers
                    )
                ),
                logical_outgoing_lanes=tuple(
                    dict.fromkeys(
                        maneuver.outgoing for maneuver in junction_logical_maneuvers
                    )
                ),
                logical_maneuvers=tuple(junction_logical_maneuvers),
                forbidden_maneuvers=tuple(
                    sorted(
                        {
                            (
                                incoming.lanelet_id,
                                outgoing.lanelet_id,
                            )
                            for incoming in dict.fromkeys(
                                maneuver.incoming
                                for maneuver in junction_logical_maneuvers
                            )
                            for outgoing in dict.fromkeys(
                                maneuver.outgoing
                                for maneuver in junction_logical_maneuvers
                            )
                        }
                        - logical_pairs
                    )
                ),
                connecting_road_groups=tuple(planned_groups),
                lane_traces=traces,
                missing_maneuvers=missing_maneuvers,
                unintended_maneuvers=unintended_maneuvers,
            )
        )

    return connecting_roads, plans, next_road_id


def _lane_by_id(road: Road, lane_id: int) -> Optional[Lane]:
    if road.lanes is None or not road.lanes.lane_sections:
        return None
    section = road.lanes.lane_sections[0]
    return (
        section.left_lanes.get(lane_id)
        if lane_id > 0
        else section.right_lanes.get(lane_id)
    )


def apply_planned_topology_links(
    plans: Sequence[JunctionEmissionPlan],
    roads: Sequence[Road],
) -> None:
    """Apply canonical road/lane links after the legacy setup pass."""
    roads_by_id = {road.id: road for road in roads}
    for plan in plans:
        for group in plan.connecting_road_groups:
            incoming = roads_by_id[group.incoming_road_id]
            outgoing = roads_by_id[group.outgoing_road_id]
            connector = roads_by_id[group.connector_road_id]
            incoming.add_successor(
                element_id=plan.junction_id,
                element_type=ElementType.JUNCTION,
            )
            outgoing.add_predecessor(
                element_id=plan.junction_id,
                element_type=ElementType.JUNCTION,
            )
            connector.link = RoadLink(
                predecessor=Predecessor(
                    element_type=ElementType.ROAD,
                    element_id=incoming.id,
                    contact_point=ContactPoint.END,
                ),
                successor=Successor(
                    element_type=ElementType.ROAD,
                    element_id=outgoing.id,
                    contact_point=ContactPoint.START,
                ),
            )
            for maneuver, connector_lane_id in zip(
                group.maneuvers,
                group.connector_lane_ids,
            ):
                incoming_lane = _lane_by_id(incoming, maneuver.incoming.lane_id)
                connector_lane = _lane_by_id(connector, connector_lane_id)
                outgoing_lane = _lane_by_id(outgoing, maneuver.outgoing.lane_id)
                if incoming_lane is not None:
                    incoming_lane.successor = LaneLink(id=connector_lane_id)
                if connector_lane is not None:
                    connector_lane.predecessor = LaneLink(id=maneuver.incoming.lane_id)
                    connector_lane.successor = LaneLink(id=maneuver.outgoing.lane_id)
                if outgoing_lane is not None:
                    outgoing_lane.predecessor = LaneLink(id=connector_lane_id)


def _slice_points(
    points: np.ndarray,
    start_station: float,
    end_station: float,
) -> np.ndarray:
    segment_lengths = np.linalg.norm(np.diff(points[:, :2], axis=0), axis=1)
    stations = np.concatenate(([0.0], np.cumsum(segment_lengths)))
    start = float(np.clip(start_station, 0.0, stations[-1]))
    end = float(np.clip(end_station, start, stations[-1]))
    if end - start <= DEFAULT_CONFIG.geometry.point_distance_threshold:
        raise ValueError("junction cutback would collapse a source-backed road")

    def interpolate(station: float) -> np.ndarray:
        index = int(np.searchsorted(stations, station, side="right") - 1)
        index = max(0, min(index, len(segment_lengths) - 1))
        ratio = (station - float(stations[index])) / float(segment_lengths[index])
        return points[index] + ratio * (points[index + 1] - points[index])

    retained = [
        points[index] for index, station in enumerate(stations) if start < station < end
    ]
    return np.asarray([interpolate(start), *retained, interpolate(end)], dtype=float)


def _translate_width_polynomial(record: LaneWidth, delta: float) -> LaneWidth:
    return LaneWidth(
        s_offset=0.0,
        a=record.a
        + record.b * delta
        + record.c * delta * delta
        + record.d * delta * delta * delta,
        b=record.b + 2.0 * record.c * delta + 3.0 * record.d * delta * delta,
        c=record.c + 3.0 * record.d * delta,
        d=record.d,
    )


def _evaluate_lane_width_derivative(lane: Lane, station: float) -> float:
    if not lane.widths:
        raise ValueError(f"Lane {lane.lane_id} has no width records")
    active = lane.widths[0]
    for record in lane.widths:
        if record.s_offset <= station + DEFAULT_CONFIG.geometry.epsilon:
            active = record
        else:
            break
    delta = station - active.s_offset
    return float(active.b + 2.0 * active.c * delta + 3.0 * active.d * delta * delta)


def _lane_width_states(
    road: Road,
    lane_ids: Sequence[int],
    station: float,
) -> Tuple[List[float], List[float]]:
    widths: List[float] = []
    derivatives: List[float] = []
    for lane_id in lane_ids:
        lane = _lane_by_id(road, lane_id)
        if lane is None:
            raise ValueError(f"Road {road.id} has no lane {lane_id}")
        width = _evaluate_lane_width(lane, station)
        if width is None:
            raise ValueError(f"Road {road.id} lane {lane_id} has no width")
        widths.append(width)
        derivatives.append(_evaluate_lane_width_derivative(lane, station))
    return widths, derivatives


def _emitted_lane_block_cross_section(
    road: Road,
    lane_ids: Sequence[int],
    *,
    at_start: bool,
    distance_from_boundary: float = 0.0,
) -> _CrossSection:
    """Evaluate a contiguous lane block against the final emitted road model."""
    if not lane_ids:
        raise ValueError(f"Road {road.id} has no selected lane block")
    side = 1 if lane_ids[0] > 0 else -1
    if any((1 if lane_id > 0 else -1) != side for lane_id in lane_ids):
        raise ValueError(f"Road {road.id} lane block crosses the reference line")
    ordered_ids = sorted(lane_ids, key=abs)
    absolute_ids = [abs(lane_id) for lane_id in ordered_ids]
    if absolute_ids != list(
        range(absolute_ids[0], absolute_ids[0] + len(absolute_ids))
    ):
        raise ValueError(f"Road {road.id} lane block is not contiguous")

    distance = max(0.0, float(distance_from_boundary))
    station = (
        min(road.length, distance) if at_start else max(0.0, road.length - distance)
    )
    if road.emission_context is not None:
        pose = road.emission_context.evaluate(station)
        reference_xy = np.asarray([pose.x, pose.y], dtype=float)
        heading = float(pose.heading)
    elif distance <= DEFAULT_CONFIG.geometry.epsilon:
        endpoint = _evaluate_planview_endpoint_with_heading(
            road.plan_view,
            at_start=at_start,
        )
        if endpoint is None:
            raise ValueError(f"Road {road.id} has no emitted endpoint geometry")
        reference_xy = np.asarray(endpoint[:2], dtype=float)
        heading = float(endpoint[2])
    else:
        raise ValueError(
            f"Road {road.id} has no emission context for lane-block cutback"
        )
    tangent = np.asarray([math.cos(heading), math.sin(heading)], dtype=float)
    normal = np.asarray([-tangent[1], tangent[0]], dtype=float)

    first_absolute_id = absolute_ids[0]
    inner_lane_ids = [side * lane_index for lane_index in range(1, first_absolute_id)]
    inner_widths, inner_derivatives = _lane_width_states(
        road,
        inner_lane_ids,
        station,
    )
    selected_widths, selected_derivatives = _lane_width_states(
        road,
        ordered_ids,
        station,
    )
    signed_offset = side * sum(inner_widths)
    signed_offset_derivative = side * sum(inner_derivatives)
    source_curvature, source_speed = _geometry_curvature_and_speed(road, station)

    z = _evaluate_elevation_profile(road.elevation_profile, station)
    reference_xyz = np.asarray(
        [
            reference_xy[0] + signed_offset * normal[0],
            reference_xy[1] + signed_offset * normal[1],
            z,
        ],
        dtype=float,
    )
    return _CrossSection(
        reference_xyz=reference_xyz,
        tangent=tangent,
        widths=np.asarray(selected_widths, dtype=float),
        lane_side=side,
        width_derivatives=np.asarray(selected_derivatives, dtype=float),
        lane_offset=0.0,
        lane_offset_derivative=signed_offset_derivative,
        source_lane_offset=signed_offset,
        source_lane_offset_derivative=signed_offset_derivative,
        source_curvature=source_curvature,
        source_speed=source_speed,
    )


def _geometry_curvature_and_speed(
    road: Road,
    station: float,
) -> Tuple[float, float]:
    if road.plan_view is None or not road.plan_view.geometries:
        return 0.0, 1.0
    station = min(max(0.0, float(station)), road.length)
    geometry = road.plan_view.geometries[-1]
    for candidate in road.plan_view.geometries:
        if station <= candidate.s + candidate.length + DEFAULT_CONFIG.geometry.epsilon:
            geometry = candidate
            break
    if isinstance(geometry, Arc):
        return float(geometry.curvature), 1.0
    if not isinstance(geometry, ParamPoly3):
        return 0.0, 1.0
    parameter = min(max(0.0, station - geometry.s), geometry.length)
    parameter_scale = 1.0
    if geometry.pRange == "normalized":
        parameter_scale = 1.0 / geometry.length
        parameter *= parameter_scale
    du = (
        geometry.bU
        + 2.0 * geometry.cU * parameter
        + 3.0 * geometry.dU * (parameter * parameter)
    ) * parameter_scale
    dv = (
        geometry.bV
        + 2.0 * geometry.cV * parameter
        + 3.0 * geometry.dV * (parameter * parameter)
    ) * parameter_scale
    ddu = (2.0 * geometry.cU + 6.0 * geometry.dU * parameter) * parameter_scale**2
    ddv = (2.0 * geometry.cV + 6.0 * geometry.dV * parameter) * parameter_scale**2
    speed_sq = du * du + dv * dv
    if speed_sq <= DEFAULT_CONFIG.geometry.epsilon**2:
        return 0.0, 1.0
    return (
        float((du * ddv - dv * ddu) / (speed_sq**1.5)),
        math.sqrt(speed_sq),
    )


def _geometry_endpoint_curvature(
    road: Road,
    *,
    at_start: bool,
) -> float:
    curvature, _ = _geometry_curvature_and_speed(
        road,
        0.0 if at_start else road.length,
    )
    return curvature


def _geometry_endpoint_speed(
    road: Road,
    *,
    at_start: bool,
) -> float:
    _, speed = _geometry_curvature_and_speed(
        road,
        0.0 if at_start else road.length,
    )
    return speed


def _transform_width_derivatives_for_curvature(
    widths: Sequence[float],
    source_derivatives: Sequence[float],
    *,
    lane_side: int,
    source_curvature: float,
    target_curvature: float,
    source_speed: float = 1.0,
    target_speed: float = 1.0,
) -> List[float]:
    _, transformed = _transform_lane_block_derivatives_for_curvature(
        widths,
        source_derivatives,
        lane_side=lane_side,
        lane_offset=0.0,
        lane_offset_derivative=0.0,
        source_curvature=source_curvature,
        target_curvature=target_curvature,
        source_speed=source_speed,
        target_speed=target_speed,
    )
    return transformed


def _transform_lane_block_derivatives_for_curvature(
    widths: Sequence[float],
    source_derivatives: Sequence[float],
    *,
    lane_side: int,
    lane_offset: float,
    lane_offset_derivative: float,
    target_lane_offset: Optional[float] = None,
    source_curvature: float,
    target_curvature: float,
    source_speed: float = 1.0,
    target_speed: float = 1.0,
) -> Tuple[float, List[float]]:
    cumulative_widths = np.concatenate(([0.0], np.cumsum(widths)))
    cumulative_derivatives = np.concatenate(([0.0], np.cumsum(source_derivatives)))
    side = 1.0 if lane_side > 0 else -1.0
    source_signed_offsets = lane_offset + side * cumulative_widths
    if target_lane_offset is None:
        target_lane_offset = lane_offset
    target_signed_offsets = target_lane_offset + side * cumulative_widths
    signed_derivatives = lane_offset_derivative + side * cumulative_derivatives
    target_signed_derivatives = []
    for source_offset, target_offset, derivative in zip(
        source_signed_offsets,
        target_signed_offsets,
        signed_derivatives,
    ):
        source_factor = 1.0 - source_curvature * float(source_offset)
        target_factor = 1.0 - target_curvature * float(target_offset)
        if (
            source_factor <= DEFAULT_CONFIG.geometry.point_distance_threshold
            or target_factor <= DEFAULT_CONFIG.geometry.point_distance_threshold
        ):
            raise ValueError("lane boundary Jacobian is not positive at junction cut")
        target_signed_derivatives.append(
            target_speed
            * target_factor
            * float(derivative)
            / (source_speed * source_factor)
        )
    target_lane_offset_derivative = float(target_signed_derivatives[0])
    target_cumulative_width_derivatives = [
        side * (float(value) - target_lane_offset_derivative)
        for value in target_signed_derivatives
    ]
    return target_lane_offset_derivative, [
        float(right - left)
        for left, right in zip(
            target_cumulative_width_derivatives[:-1],
            target_cumulative_width_derivatives[1:],
        )
    ]


def _bezier_endpoint_curvature(
    control_points_xyz: np.ndarray,
    *,
    at_start: bool,
) -> float:
    p0, p1, p2, p3 = control_points_xyz[:, :2]
    if at_start:
        first = 3.0 * (p1 - p0)
        second = 6.0 * (p2 - 2.0 * p1 + p0)
    else:
        first = 3.0 * (p3 - p2)
        second = 6.0 * (p3 - 2.0 * p2 + p1)
    speed_sq = float(np.dot(first, first))
    if speed_sq <= DEFAULT_CONFIG.geometry.epsilon**2:
        return 0.0
    return _cross_2d(first, second) / (speed_sq**1.5)


def _bezier_endpoint_speed(
    control_points_xyz: np.ndarray,
    *,
    at_start: bool,
) -> float:
    p0, p1, p2, p3 = control_points_xyz[:, :2]
    derivative = 3.0 * ((p1 - p0) if at_start else (p3 - p2))
    length = _bezier_arc_length(control_points_xyz[:, :2])
    return max(
        DEFAULT_CONFIG.geometry.epsilon,
        float(np.linalg.norm(derivative)) / length,
    )


def _lane_center_heading_error_after_curvature_transform(
    cross_section: _CrossSection,
    *,
    target_curvature: float,
    target_speed: float,
) -> float:
    source_derivatives = (
        np.zeros(len(cross_section.widths), dtype=float)
        if cross_section.width_derivatives is None
        else np.asarray(cross_section.width_derivatives, dtype=float)
    )
    try:
        target_lane_offset_derivative, target_width_derivatives = (
            _transform_lane_block_derivatives_for_curvature(
                cross_section.widths,
                source_derivatives,
                lane_side=cross_section.lane_side,
                lane_offset=cross_section.source_lane_offset,
                lane_offset_derivative=cross_section.source_lane_offset_derivative,
                target_lane_offset=cross_section.lane_offset,
                source_curvature=cross_section.source_curvature,
                target_curvature=target_curvature,
                source_speed=cross_section.source_speed,
                target_speed=target_speed,
            )
        )
    except ValueError:
        return math.inf
    cumulative_widths = np.concatenate(([0.0], np.cumsum(cross_section.widths)))
    source_cumulative_derivatives = np.concatenate(
        ([0.0], np.cumsum(source_derivatives))
    )
    target_cumulative_derivatives = np.concatenate(
        ([0.0], np.cumsum(target_width_derivatives))
    )
    side = 1.0 if cross_section.lane_side > 0 else -1.0
    max_error = 0.0
    for lane_index, width in enumerate(cross_section.widths):
        center_width = cumulative_widths[lane_index] + 0.5 * float(width)
        source_offset = cross_section.source_lane_offset + side * center_width
        target_offset = cross_section.lane_offset + side * center_width
        source_offset_derivative = (
            cross_section.source_lane_offset_derivative
            + side
            * (
                source_cumulative_derivatives[lane_index]
                + 0.5 * source_derivatives[lane_index]
            )
        )
        target_offset_derivative = target_lane_offset_derivative + side * (
            target_cumulative_derivatives[lane_index]
            + 0.5 * target_width_derivatives[lane_index]
        )
        source_heading = math.atan2(
            float(source_offset_derivative),
            cross_section.source_speed
            * (1.0 - cross_section.source_curvature * source_offset),
        )
        target_heading = math.atan2(
            float(target_offset_derivative),
            target_speed * (1.0 - target_curvature * target_offset),
        )
        max_error = max(max_error, _heading_error(source_heading, target_heading))
    return max_error


def _cross_section_boundary_heading_spread(
    cross_section: _CrossSection,
) -> float:
    source_derivatives = (
        np.zeros(len(cross_section.widths), dtype=float)
        if cross_section.width_derivatives is None
        else np.asarray(cross_section.width_derivatives, dtype=float)
    )
    cumulative_widths = np.concatenate(([0.0], np.cumsum(cross_section.widths)))
    cumulative_derivatives = np.concatenate(([0.0], np.cumsum(source_derivatives)))
    side = 1.0 if cross_section.lane_side > 0 else -1.0
    headings = []
    for width, width_derivative in zip(
        cumulative_widths,
        cumulative_derivatives,
    ):
        offset = cross_section.source_lane_offset + side * float(width)
        offset_derivative = cross_section.source_lane_offset_derivative + side * float(
            width_derivative
        )
        headings.append(
            math.atan2(
                offset_derivative,
                cross_section.source_speed
                * (1.0 - cross_section.source_curvature * offset),
            )
        )
    return max(
        (
            _heading_error(first, second)
            for index, first in enumerate(headings)
            for second in headings[index + 1 :]
        ),
        default=0.0,
    )


def _candidate_lane_center_heading_error(
    start: _CrossSection,
    end: _CrossSection,
    candidate: _CurveCandidate,
) -> float:
    return max(
        _lane_center_heading_error_after_curvature_transform(
            start,
            target_curvature=_bezier_endpoint_curvature(
                candidate.control_points_xyz,
                at_start=True,
            ),
            target_speed=_bezier_endpoint_speed(
                candidate.control_points_xyz,
                at_start=True,
            ),
        ),
        _lane_center_heading_error_after_curvature_transform(
            end,
            target_curvature=_bezier_endpoint_curvature(
                candidate.control_points_xyz,
                at_start=False,
            ),
            target_speed=_bezier_endpoint_speed(
                candidate.control_points_xyz,
                at_start=False,
            ),
        ),
    )


def _trim_lane_records(lane: Lane, start_trim: float, new_length: float) -> None:
    if lane.widths:
        ordered = sorted(lane.widths, key=lambda record: record.s_offset)
        active = ordered[0]
        for record in ordered:
            if record.s_offset <= start_trim + DEFAULT_CONFIG.geometry.epsilon:
                active = record
            else:
                break
        translated = _translate_width_polynomial(
            active,
            max(0.0, start_trim - active.s_offset),
        )
        new_widths = [translated]
        new_widths.extend(
            copy.deepcopy(record)
            for record in ordered
            if start_trim + DEFAULT_CONFIG.geometry.epsilon
            < record.s_offset
            < start_trim + new_length - DEFAULT_CONFIG.geometry.epsilon
        )
        for record in new_widths[1:]:
            record.s_offset -= start_trim
        lane.widths = new_widths

    for records_name in ("road_marks", "borders", "heights", "speeds", "accesses"):
        records = getattr(lane, records_name)
        if not records:
            continue
        ordered_records = sorted(records, key=lambda record: record.s_offset)
        active_record = ordered_records[0]
        for record in ordered_records:
            if record.s_offset <= start_trim + DEFAULT_CONFIG.geometry.epsilon:
                active_record = record
            else:
                break
        first = copy.deepcopy(active_record)
        first.s_offset = 0.0
        shifted = [first]
        for record in ordered_records:
            if (
                start_trim + DEFAULT_CONFIG.geometry.epsilon
                < record.s_offset
                < start_trim + new_length - DEFAULT_CONFIG.geometry.epsilon
            ):
                copied = copy.deepcopy(record)
                copied.s_offset -= start_trim
                shifted.append(copied)
        setattr(lane, records_name, shifted)


def _trim_source_backed_road(
    road: Road,
    *,
    start_trim: float,
    end_trim: float,
) -> None:
    context = road.emission_context
    if context is None:
        raise ValueError(f"Road {road.id} has no emission context for cutback")
    old_length = context.length
    end_station = old_length - end_trim
    points = _slice_points(
        context.emission_geometry.source_points_3d,
        start_trim,
        end_station,
    )
    start_heading = (
        context.evaluate(0.0).heading
        if start_trim <= DEFAULT_CONFIG.geometry.epsilon
        else None
    )
    end_heading = (
        context.evaluate(old_length).heading
        if end_trim <= DEFAULT_CONFIG.geometry.epsilon
        else None
    )
    emission_geometry = EmissionReferenceGeometry(
        points,
        start_heading=start_heading,
        end_heading=end_heading,
    )
    sliced_context = RoadEmissionContext(
        topology_geometry=context.topology_geometry,
        emission_geometry=emission_geometry,
        station_mapping=StationMapping.from_lengths(
            context.topology_geometry.length,
            emission_geometry.source_length,
            emission_geometry.length,
        ),
    )
    road.plan_view = sliced_context.to_plan_view()
    road.elevation_profile = sliced_context.to_elevation_profile()
    road.length = sliced_context.length
    road.elevation_offset = sliced_context.elevation_offset
    road.emission_context = sliced_context
    road.reference_start_xyz = tuple(float(value) for value in points[0])
    road.reference_end_xyz = tuple(float(value) for value in points[-1])
    if road.lanes is not None:
        for section in road.lanes.lane_sections:
            for lane in list(section.left_lanes.values()) + list(
                section.right_lanes.values()
            ):
                _trim_lane_records(lane, start_trim, road.length)


def apply_junction_emission_plans(
    plans: Sequence[JunctionEmissionPlan],
    roads: List[Road],
) -> None:
    """Apply planned cut stations and connector geometry to emitted road copies."""
    roads_by_id = {road.id: road for road in roads}
    incoming_cuts: Dict[int, float] = {}
    outgoing_cuts: Dict[int, float] = {}
    for plan in plans:
        if plan.missing_maneuvers or plan.unintended_maneuvers:
            continue
        for group in plan.connecting_road_groups:
            incoming_road = roads_by_id[group.incoming_road_id]
            outgoing_road = roads_by_id[group.outgoing_road_id]
            incoming_lane_ids = [
                maneuver.incoming.lane_id for maneuver in group.maneuvers
            ]
            outgoing_lane_ids = [
                maneuver.outgoing.lane_id for maneuver in group.maneuvers
            ]
            result = search_junction_cutback(
                lambda distance: _emitted_lane_block_cross_section(
                    incoming_road,
                    incoming_lane_ids,
                    at_start=False,
                    distance_from_boundary=distance,
                ),
                lambda distance: _emitted_lane_block_cross_section(
                    outgoing_road,
                    outgoing_lane_ids,
                    at_start=True,
                    distance_from_boundary=distance,
                ),
            )
            if result is None:
                raise ValueError(
                    "final emitted lane blocks do not admit a valid cutback "
                    f"for roads {incoming_road.id}->{outgoing_road.id}"
                )
            incoming_distance, outgoing_distance, curve, incoming, outgoing = result
            group.incoming_cut = CutSection(
                road_id=incoming_road.id,
                station_from_boundary=incoming_distance,
                reference_xyz=tuple(float(value) for value in incoming.reference_xyz),
                heading=math.atan2(incoming.tangent[1], incoming.tangent[0]),
                lane_widths=tuple(float(value) for value in incoming.widths),
            )
            group.outgoing_cut = CutSection(
                road_id=outgoing_road.id,
                station_from_boundary=outgoing_distance,
                reference_xyz=tuple(float(value) for value in outgoing.reference_xyz),
                heading=math.atan2(outgoing.tangent[1], outgoing.tangent[0]),
                lane_widths=tuple(float(value) for value in outgoing.widths),
            )
            group.curve_points_xyz = curve.points_xyz
            group.curve_control_points_xyz = curve.control_points_xyz
            group.surface = curve.validation
            incoming_cuts[group.incoming_road_id] = max(
                incoming_cuts.get(group.incoming_road_id, 0.0),
                incoming_distance,
            )
            outgoing_cuts[group.outgoing_road_id] = max(
                outgoing_cuts.get(group.outgoing_road_id, 0.0),
                outgoing_distance,
            )

    for road_id in sorted(set(incoming_cuts) | set(outgoing_cuts)):
        road = roads_by_id[road_id]
        _trim_source_backed_road(
            road,
            start_trim=outgoing_cuts.get(road_id, 0.0),
            end_trim=incoming_cuts.get(road_id, 0.0),
        )

    for plan in plans:
        if plan.missing_maneuvers or plan.unintended_maneuvers:
            continue
        for group in plan.connecting_road_groups:
            incoming_road = roads_by_id[group.incoming_road_id]
            outgoing_road = roads_by_id[group.outgoing_road_id]
            incoming_lane_ids = [
                maneuver.incoming.lane_id for maneuver in group.maneuvers
            ]
            outgoing_lane_ids = [
                maneuver.outgoing.lane_id for maneuver in group.maneuvers
            ]
            start = _emitted_lane_block_cross_section(
                incoming_road,
                incoming_lane_ids,
                at_start=False,
            )
            end = _emitted_lane_block_cross_section(
                outgoing_road,
                outgoing_lane_ids,
                at_start=True,
            )
            curve = _best_curve_candidate(start, end)
            if (
                curve is None
                or not curve.validation.finite_width_valid
                or not curve.validation.c1
                or _candidate_lane_center_heading_error(start, end, curve)
                > DEFAULT_CONFIG.junction_emission.lane_center_heading_tolerance
            ):
                raise ValueError(
                    "final emitted lane-block endpoints do not admit a valid "
                    f"connector for roads {incoming_road.id}->{outgoing_road.id}"
                )
            start_widths, start_width_derivatives = _lane_width_states(
                incoming_road,
                incoming_lane_ids,
                incoming_road.length,
            )
            end_widths, end_width_derivatives = _lane_width_states(
                outgoing_road,
                outgoing_lane_ids,
                0.0,
            )
            lane_side = 1 if group.connector_lane_ids[0] > 0 else -1
            (
                start_lane_offset_derivative,
                start_width_derivatives,
            ) = _transform_lane_block_derivatives_for_curvature(
                start_widths,
                start_width_derivatives,
                lane_side=lane_side,
                lane_offset=start.source_lane_offset,
                lane_offset_derivative=start.source_lane_offset_derivative,
                target_lane_offset=start.lane_offset,
                source_curvature=_geometry_endpoint_curvature(
                    incoming_road,
                    at_start=False,
                ),
                target_curvature=_bezier_endpoint_curvature(
                    curve.control_points_xyz,
                    at_start=True,
                ),
                source_speed=_geometry_endpoint_speed(
                    incoming_road,
                    at_start=False,
                ),
                target_speed=_bezier_endpoint_speed(
                    curve.control_points_xyz,
                    at_start=True,
                ),
            )
            (
                end_lane_offset_derivative,
                end_width_derivatives,
            ) = _transform_lane_block_derivatives_for_curvature(
                end_widths,
                end_width_derivatives,
                lane_side=lane_side,
                lane_offset=end.source_lane_offset,
                lane_offset_derivative=end.source_lane_offset_derivative,
                target_lane_offset=end.lane_offset,
                source_curvature=_geometry_endpoint_curvature(
                    outgoing_road,
                    at_start=True,
                ),
                target_curvature=_bezier_endpoint_curvature(
                    curve.control_points_xyz,
                    at_start=False,
                ),
                source_speed=_geometry_endpoint_speed(
                    outgoing_road,
                    at_start=True,
                ),
                target_speed=_bezier_endpoint_speed(
                    curve.control_points_xyz,
                    at_start=False,
                ),
            )
            connector = _build_multi_lane_connector(
                road_id=group.connector_road_id,
                junction_id=plan.junction_id,
                incoming_road_id=group.incoming_road_id,
                outgoing_road_id=group.outgoing_road_id,
                maneuvers=group.maneuvers,
                connector_lane_ids=group.connector_lane_ids,
                curve_control_points_xyz=curve.control_points_xyz,
                start_widths=start_widths,
                end_widths=end_widths,
                start_width_derivatives=start_width_derivatives,
                end_width_derivatives=end_width_derivatives,
                start_lane_offset=start.lane_offset,
                end_lane_offset=end.lane_offset,
                start_lane_offset_derivative=start_lane_offset_derivative,
                end_lane_offset_derivative=end_lane_offset_derivative,
                traffic_rule=roads_by_id[group.incoming_road_id].rule
                or TrafficRule.RHT,
            )
            existing = roads_by_id[group.connector_road_id]
            connector.signals = existing.signals
            connector.objects = existing.objects
            index = roads.index(existing)
            roads[index] = connector
            roads_by_id[connector.id] = connector
            group.curve_points_xyz = curve.points_xyz
            group.curve_control_points_xyz = curve.control_points_xyz
            group.surface = curve.validation
            group.incoming_cut = CutSection(
                road_id=incoming_road.id,
                station_from_boundary=group.incoming_cut.station_from_boundary,
                reference_xyz=tuple(float(value) for value in start.reference_xyz),
                heading=math.atan2(start.tangent[1], start.tangent[0]),
                lane_widths=tuple(float(value) for value in start.widths),
            )
            group.outgoing_cut = CutSection(
                road_id=outgoing_road.id,
                station_from_boundary=group.outgoing_cut.station_from_boundary,
                reference_xyz=tuple(float(value) for value in end.reference_xyz),
                heading=math.atan2(end.tangent[1], end.tangent[0]),
                lane_widths=tuple(float(value) for value in end.widths),
            )
        plan.applied = True


def _single_source_backed_lane(road: Road) -> Optional[Lane]:
    if road.lanes is None or len(road.lanes.lane_sections) != 1:
        return None
    section = road.lanes.lane_sections[0]
    lanes = list(section.left_lanes.values()) + list(section.right_lanes.values())
    if len(lanes) != 1:
        return None
    lane = lanes[0]
    if (
        lane.lane_id is None
        or abs(lane.lane_id) != 1
        or lane.lanelet_id is None
        or lane.predecessor is None
        or lane.successor is None
    ):
        return None
    return lane


def _rebuild_single_lane_connector(
    road: Road,
    lane: Lane,
    incoming_road: Road,
    outgoing_road: Road,
) -> Optional[Road]:
    if lane.lane_id is None or lane.predecessor is None or lane.successor is None:
        return None
    try:
        start = _emitted_lane_block_cross_section(
            incoming_road,
            (lane.predecessor.id,),
            at_start=False,
        )
        end = _emitted_lane_block_cross_section(
            outgoing_road,
            (lane.successor.id,),
            at_start=True,
        )
    except ValueError:
        return None
    if start.lane_side != end.lane_side or start.lane_side != (
        1 if lane.lane_id > 0 else -1
    ):
        return None

    curve = _best_curve_candidate(start, end)
    if (
        curve is None
        or not curve.validation.finite_width_valid
        or not curve.validation.c1
        or _candidate_lane_center_heading_error(start, end, curve)
        > DEFAULT_CONFIG.junction_emission.lane_center_heading_tolerance
    ):
        return None

    start_derivatives = (
        np.zeros(1, dtype=float)
        if start.width_derivatives is None
        else np.asarray(start.width_derivatives, dtype=float)
    )
    end_derivatives = (
        np.zeros(1, dtype=float)
        if end.width_derivatives is None
        else np.asarray(end.width_derivatives, dtype=float)
    )
    try:
        start_lane_offset_derivative, transformed_start_derivatives = (
            _transform_lane_block_derivatives_for_curvature(
                start.widths,
                start_derivatives,
                lane_side=start.lane_side,
                lane_offset=start.source_lane_offset,
                lane_offset_derivative=start.source_lane_offset_derivative,
                target_lane_offset=start.lane_offset,
                source_curvature=start.source_curvature,
                target_curvature=_bezier_endpoint_curvature(
                    curve.control_points_xyz,
                    at_start=True,
                ),
                source_speed=start.source_speed,
                target_speed=_bezier_endpoint_speed(
                    curve.control_points_xyz,
                    at_start=True,
                ),
            )
        )
        end_lane_offset_derivative, transformed_end_derivatives = (
            _transform_lane_block_derivatives_for_curvature(
                end.widths,
                end_derivatives,
                lane_side=end.lane_side,
                lane_offset=end.source_lane_offset,
                lane_offset_derivative=end.source_lane_offset_derivative,
                target_lane_offset=end.lane_offset,
                source_curvature=end.source_curvature,
                target_curvature=_bezier_endpoint_curvature(
                    curve.control_points_xyz,
                    at_start=False,
                ),
                source_speed=end.source_speed,
                target_speed=_bezier_endpoint_speed(
                    curve.control_points_xyz,
                    at_start=False,
                ),
            )
        )
    except ValueError:
        return None

    source_lanelet_id = int(lane.lanelet_id)
    maneuver = LogicalManeuver(
        incoming=LogicalLane(
            lanelet_id=source_lanelet_id,
            road_id=incoming_road.id,
            lane_id=lane.predecessor.id,
            subtype=lane.lane_type.value,
        ),
        outgoing=LogicalLane(
            lanelet_id=source_lanelet_id,
            road_id=outgoing_road.id,
            lane_id=lane.successor.id,
            subtype=lane.lane_type.value,
        ),
    )
    generated = _build_multi_lane_connector(
        road_id=road.id,
        junction_id=road.junction,
        incoming_road_id=incoming_road.id,
        outgoing_road_id=outgoing_road.id,
        maneuvers=(maneuver,),
        connector_lane_ids=(lane.lane_id,),
        curve_control_points_xyz=curve.control_points_xyz,
        start_widths=start.widths,
        end_widths=end.widths,
        start_width_derivatives=transformed_start_derivatives,
        end_width_derivatives=transformed_end_derivatives,
        start_lane_offset=start.lane_offset,
        end_lane_offset=end.lane_offset,
        start_lane_offset_derivative=start_lane_offset_derivative,
        end_lane_offset_derivative=end_lane_offset_derivative,
        traffic_rule=road.rule or TrafficRule.RHT,
    )

    replacement = copy.deepcopy(road)
    replacement.length = generated.length
    replacement.plan_view = generated.plan_view
    replacement.elevation_profile = generated.elevation_profile
    replacement.reference_start_xyz = generated.reference_start_xyz
    replacement.reference_end_xyz = generated.reference_end_xyz
    replacement.emission_context = None
    if replacement.lanes is None or generated.lanes is None:
        return None
    replacement.lanes.lane_offsets = generated.lanes.lane_offsets
    replacement_lane = _lane_by_id(replacement, lane.lane_id)
    generated_lane = _lane_by_id(generated, lane.lane_id)
    if replacement_lane is None or generated_lane is None:
        return None
    replacement_lane.widths = generated_lane.widths
    if not _single_lane_surface_is_valid(replacement, replacement_lane):
        return None
    return replacement


def repair_invalid_sibling_connecting_road_surfaces(
    plans: Sequence[JunctionEmissionPlan],
    roads: List[Road],
) -> Tuple[int, ...]:
    """Repair invalid source-backed branch surfaces beside planned continuations.

    A junction-wide plan may trim an incoming road while preserving a
    source-backed one-lane branch that leaves the same road. Post-freeze
    endpoint alignment keeps that branch's topology but can retain an invalid
    finite-width spline. Rebuild only such invalid sibling connectors from the
    final incoming/outgoing cross-sections.
    """
    roads_by_id = {road.id: road for road in roads}
    repaired_ids: List[int] = []
    for plan in plans:
        if (
            plan.missing_maneuvers
            or plan.unintended_maneuvers
            or not plan.connecting_road_groups
        ):
            continue
        planned_incoming_ids = {
            group.incoming_road_id for group in plan.connecting_road_groups
        }
        protected_ids = plan.protected_connector_ids
        for road in list(roads):
            if road.junction != plan.junction_id or road.id in protected_ids:
                continue
            lane = _single_source_backed_lane(road)
            if lane is None or road.link is None:
                continue
            predecessor = road.link.predecessor
            successor = road.link.successor
            if (
                predecessor is None
                or successor is None
                or predecessor.element_type != ElementType.ROAD
                or successor.element_type != ElementType.ROAD
                or predecessor.contact_point != ContactPoint.END
                or successor.contact_point != ContactPoint.START
                or predecessor.element_id not in planned_incoming_ids
            ):
                continue
            is_logical_sibling = any(
                maneuver.incoming.road_id == predecessor.element_id
                and maneuver.outgoing.road_id == road.id
                and lane.predecessor is not None
                and maneuver.incoming.lane_id == lane.predecessor.id
                and maneuver.outgoing.lane_id == lane.lane_id
                for maneuver in plan.logical_maneuvers
            )
            if not is_logical_sibling or _single_lane_surface_is_valid(road, lane):
                continue
            incoming_road = roads_by_id.get(predecessor.element_id)
            outgoing_road = roads_by_id.get(successor.element_id)
            if incoming_road is None or outgoing_road is None:
                continue
            replacement = _rebuild_single_lane_connector(
                road,
                lane,
                incoming_road,
                outgoing_road,
            )
            if replacement is None:
                continue
            index = roads.index(road)
            roads[index] = replacement
            roads_by_id[replacement.id] = replacement
            repaired_ids.append(replacement.id)
    return tuple(sorted(repaired_ids))


def build_emitted_traceability(
    lanelet_to_road_and_lane: Dict[int, Tuple[int, int]],
    plans: Sequence[JunctionEmissionPlan],
) -> Dict[int, List[dict]]:
    """Build a backward-compatible multi-segment lanelet trace sidecar field."""
    traces: Dict[int, List[dict]] = {
        lanelet_id: [
            {
                "road_id": road_lane[0],
                "lane_id": road_lane[1],
                "role": "source",
            }
        ]
        for lanelet_id, road_lane in lanelet_to_road_and_lane.items()
    }
    for plan in plans:
        for lane_trace in plan.lane_traces:
            trace = traces.setdefault(lane_trace.lanelet_id, [])
            existing = {
                (segment["road_id"], segment["lane_id"], segment["role"])
                for segment in trace
            }
            for segment in lane_trace.emitted_segments:
                key = (segment.road_id, segment.lane_id, segment.role)
                if key not in existing:
                    trace.append(
                        {
                            "road_id": segment.road_id,
                            "lane_id": segment.lane_id,
                            "role": segment.role,
                        }
                    )
                    existing.add(key)
    return traces
