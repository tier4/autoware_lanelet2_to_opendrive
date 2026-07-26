"""Internal reference-geometry primitives for topology/emission separation.

This module intentionally does not replace :mod:`reference_line`.  It provides
small internal building blocks for experimenting with a split between:

* topology geometry: the current smooth spline used for connectivity decisions
* emission geometry: the physical geometry intended for OpenDRIVE planView
* station mapping: a monotonic correspondence between the two station domains
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Iterable, Optional

import numpy as np

from ..conversion_config import WidthEstimationConfig, WidthReference
from ..config import DEFAULT_CONFIG
from ..spline import Splines


@dataclass(frozen=True)
class ReferencePose:
    """Reference-line pose in world XY coordinates."""

    x: float
    y: float
    heading: float

    @property
    def xy(self) -> np.ndarray:
        """Return ``(x, y)`` as a NumPy vector."""
        return np.array([self.x, self.y], dtype=float)


@dataclass(frozen=True)
class ProjectionResult:
    """Continuous projection result onto a reference geometry."""

    s: float
    t: float
    heading: float
    distance: float


@dataclass(frozen=True)
class ReprojectedPoint:
    """Point projected from topology/source context onto emission geometry."""

    topology: ProjectionResult
    emission: ProjectionResult
    source_station_hint: float


@dataclass(frozen=True)
class ReferenceDomainCoverage:
    """How a source polyline covers an emission reference station domain."""

    reference_start: float
    reference_end: float
    projected_min_station: float
    projected_max_station: float
    start_overhang: float
    end_overhang: float
    domain_start_gap: float
    domain_end_gap: float
    source_arc_length: float
    source_arc_inside_domain: float
    source_arc_outside_domain: float
    source_coverage_ratio: float
    domain_coverage_ratio: float


@dataclass(frozen=True)
class ValidationResult:
    """Validation summary for a reference geometry or station mapping."""

    valid: bool
    errors: tuple[str, ...] = ()


def _as_xy_array(points: Iterable[Iterable[float]]) -> np.ndarray:
    array = np.asarray(list(points), dtype=float)
    if array.ndim != 2 or array.shape[1] not in (2, 3):
        raise ValueError("points must be an (N, 2) or (N, 3) array")
    if len(array) < 2:
        raise ValueError("at least two points are required")
    if not np.all(np.isfinite(array)):
        raise ValueError("points contain non-finite values")
    return array


def _clean_polyline(
    points: np.ndarray,
    *,
    min_segment_length: float = DEFAULT_CONFIG.geometry.point_distance_threshold,
) -> np.ndarray:
    cleaned: list[np.ndarray] = []
    for point in points:
        if (
            not cleaned
            or float(np.linalg.norm(point[:2] - cleaned[-1][:2])) > min_segment_length
        ):
            cleaned.append(point)
    if len(cleaned) < 2:
        raise ValueError("source polyline has fewer than two distinct points")
    return np.asarray(cleaned, dtype=float)


def _heading_from_vector(vector: np.ndarray) -> float:
    norm = float(np.linalg.norm(vector))
    if norm <= DEFAULT_CONFIG.geometry.epsilon:
        raise ValueError("zero-length vector has no heading")
    return float(math.atan2(vector[1], vector[0]))


def _left_normal(heading: float) -> np.ndarray:
    return np.array([-math.sin(heading), math.cos(heading)], dtype=float)


def _polyline_stations(points_xy: np.ndarray) -> np.ndarray:
    if len(points_xy) < 2:
        raise ValueError("at least two points are required")
    segment_lengths = np.linalg.norm(np.diff(points_xy[:, :2], axis=0), axis=1)
    if np.any(segment_lengths <= DEFAULT_CONFIG.geometry.epsilon):
        raise ValueError("polyline contains zero-length segments")
    return np.concatenate(([0.0], np.cumsum(segment_lengths)))


def _interpolate_polyline(
    points: np.ndarray, stations: np.ndarray, s: float
) -> np.ndarray:
    s_clamped = float(np.clip(s, float(stations[0]), float(stations[-1])))
    idx = int(np.searchsorted(stations, s_clamped, side="right") - 1)
    idx = max(0, min(idx, len(points) - 2))
    seg_len = float(stations[idx + 1] - stations[idx])
    if seg_len <= DEFAULT_CONFIG.geometry.epsilon:
        return points[idx].copy()
    ratio = (s_clamped - float(stations[idx])) / seg_len
    return points[idx] + ratio * (points[idx + 1] - points[idx])


def _orient_polyline_like(
    points: np.ndarray, reference_points: np.ndarray
) -> np.ndarray:
    direct_distance = float(
        np.linalg.norm(points[0, :2] - reference_points[0, :2])
        + np.linalg.norm(points[-1, :2] - reference_points[-1, :2])
    )
    reversed_distance = float(
        np.linalg.norm(points[-1, :2] - reference_points[0, :2])
        + np.linalg.norm(points[0, :2] - reference_points[-1, :2])
    )
    if reversed_distance + DEFAULT_CONFIG.geometry.epsilon < direct_distance:
        return points[::-1].copy()
    return points


def _cross_2d(a: np.ndarray, b: np.ndarray) -> float:
    return float(a[0] * b[1] - a[1] * b[0])


def _line_segment_intersection_station(
    origin: np.ndarray,
    direction: np.ndarray,
    segment_start: np.ndarray,
    segment_end: np.ndarray,
) -> list[float]:
    """Return signed distances where a directed cross-section hits a segment."""
    segment = segment_end - segment_start
    epsilon = DEFAULT_CONFIG.geometry.epsilon
    determinant = _cross_2d(direction, -segment)
    delta = segment_start - origin

    if abs(determinant) > epsilon:
        distance, ratio = np.linalg.solve(
            np.column_stack((direction, -segment)),
            delta,
        )
        if -epsilon <= ratio <= 1.0 + epsilon:
            return [float(distance)]
        return []

    if abs(_cross_2d(delta, direction)) > epsilon:
        return []

    segment_length = float(np.linalg.norm(segment))
    if segment_length <= epsilon:
        return []
    return [
        float(np.dot(segment_start - origin, direction)),
        float(np.dot(segment_end - origin, direction)),
    ]


def _unique_sorted_values(values: list[float]) -> list[float]:
    epsilon = DEFAULT_CONFIG.geometry.epsilon
    sorted_values = sorted(float(value) for value in values if math.isfinite(value))
    unique: list[float] = []
    for value in sorted_values:
        if not unique or abs(value - unique[-1]) > epsilon:
            unique.append(value)
    return unique


def _polygon_cross_section_width(
    polygon_points: np.ndarray,
    origin: np.ndarray,
    side_direction: np.ndarray,
) -> Optional[float]:
    """Measure the first polygon interval on the expected lane side."""
    intersections: list[float] = []
    for idx in range(len(polygon_points)):
        start = polygon_points[idx, :2]
        end = polygon_points[(idx + 1) % len(polygon_points), :2]
        intersections.extend(
            _line_segment_intersection_station(origin, side_direction, start, end)
        )

    distances = _unique_sorted_values(intersections)
    epsilon = DEFAULT_CONFIG.geometry.epsilon
    if len(distances) == 1 and distances[0] >= -epsilon:
        return 0.0

    candidates: list[tuple[float, float]] = []
    for start, end in zip(distances[0::2], distances[1::2]):
        width = float(end - start)
        if width < -epsilon:
            continue
        midpoint = 0.5 * (start + end)
        if midpoint >= -epsilon:
            candidates.append((max(0.0, midpoint), max(0.0, width)))

    if not candidates:
        return None
    _midpoint, width = min(candidates)
    return width


def _polyline_cross_section_distances(
    points: np.ndarray,
    origin: np.ndarray,
    side_direction: np.ndarray,
) -> list[float]:
    distances: list[float] = []
    for idx in range(len(points) - 1):
        distances.extend(
            _line_segment_intersection_station(
                origin,
                side_direction,
                points[idx, :2],
                points[idx + 1, :2],
            )
        )
    return _unique_sorted_values(distances)


def _boundary_cross_section_width(
    anchor_points: np.ndarray,
    other_points: np.ndarray,
    origin: np.ndarray,
    side_direction: np.ndarray,
) -> Optional[float]:
    """Measure lane width from inner-boundary to first outer/cap hit."""
    epsilon = DEFAULT_CONFIG.geometry.epsilon
    anchor_distances = _polyline_cross_section_distances(
        anchor_points,
        origin,
        side_direction,
    )
    positive_anchor_distances = [
        distance for distance in anchor_distances if distance >= -epsilon
    ]
    if not positive_anchor_distances:
        return None
    inner_distance = min(positive_anchor_distances)

    outer_segments = [
        other_points,
        np.asarray([anchor_points[0], other_points[0]], dtype=float),
        np.asarray([anchor_points[-1], other_points[-1]], dtype=float),
    ]
    outer_distances: list[float] = []
    for segment_points in outer_segments:
        outer_distances.extend(
            _polyline_cross_section_distances(
                segment_points,
                origin,
                side_direction,
            )
        )

    outward_candidates = [
        distance
        for distance in _unique_sorted_values(outer_distances)
        if distance > inner_distance + epsilon
    ]
    if outward_candidates:
        return float(min(outward_candidates) - inner_distance)

    touching_candidates = [
        distance
        for distance in _unique_sorted_values(outer_distances)
        if abs(distance - inner_distance) <= epsilon
    ]
    if not touching_candidates:
        return None
    return 0.0


def _lateral_endpoint_cap_width(
    anchor_point: np.ndarray,
    other_point: np.ndarray,
    heading: float,
    side_direction: np.ndarray,
) -> Optional[float]:
    cap_vector = other_point[:2] - anchor_point[:2]
    lateral = float(np.dot(cap_vector, side_direction))
    tangent = np.array([math.cos(heading), math.sin(heading)], dtype=float)
    longitudinal = float(np.dot(cap_vector, tangent))
    if lateral <= DEFAULT_CONFIG.geometry.epsilon:
        return None
    if lateral >= abs(longitudinal):
        return lateral
    return None


def _piecewise_linear_width_segments(
    stations: np.ndarray,
    widths: np.ndarray,
) -> list[tuple[float, float, float, float, float]]:
    segments: list[tuple[float, float, float, float, float]] = []
    for idx in range(len(stations) - 1):
        length = float(stations[idx + 1] - stations[idx])
        if length <= DEFAULT_CONFIG.geometry.epsilon:
            continue
        a = float(widths[idx])
        b = float((widths[idx + 1] - widths[idx]) / length)
        segments.append((float(stations[idx]), a, b, 0.0, 0.0))
    return segments


def _refine_piecewise_linear_width_samples(
    stations: np.ndarray,
    widths: np.ndarray,
    measure_width: Callable[[float], float],
    *,
    max_samples: int,
) -> tuple[np.ndarray, np.ndarray]:
    """Insert local width samples where linear width emission would drift."""
    refined_stations = [float(station) for station in stations]
    refined_widths = {
        float(station): float(width) for station, width in zip(stations, widths)
    }
    tolerance = DEFAULT_CONFIG.geometry.emission_width_refinement_tolerance
    min_interval = DEFAULT_CONFIG.geometry.emission_width_refinement_min_interval

    for _ in range(DEFAULT_CONFIG.geometry.emission_width_refinement_max_iterations):
        additions: list[tuple[float, float, float]] = []
        for start, end in zip(refined_stations, refined_stations[1:]):
            if len(refined_stations) + len(additions) >= max_samples:
                break
            if end - start <= min_interval:
                continue
            midpoint = 0.5 * (start + end)
            midpoint_width = float(measure_width(midpoint))
            linear_width = 0.5 * (refined_widths[start] + refined_widths[end])
            error = abs(midpoint_width - linear_width)
            if error > tolerance:
                additions.append((error, midpoint, midpoint_width))

        if not additions:
            break

        remaining = max_samples - len(refined_stations)
        for _error, station, width in sorted(additions, reverse=True)[:remaining]:
            refined_widths[station] = width
            refined_stations.append(station)
        refined_stations = _unique_sorted_values(refined_stations)

    refined_width_array = np.asarray(
        [refined_widths[station] for station in refined_stations],
        dtype=float,
    )
    return np.asarray(refined_stations, dtype=float), refined_width_array


class _PiecewiseLinearWidthAdapter:
    """Width adapter that emits local linear OpenDRIVE width records."""

    def __init__(self, stations: np.ndarray, widths: np.ndarray):
        self._stations = np.asarray(stations, dtype=float)
        self._widths = np.asarray(widths, dtype=float)
        if len(self._stations) != len(self._widths):
            raise ValueError("width stations and widths must have the same length")
        if len(self._stations) < 2:
            raise ValueError("at least two width samples are required")
        if not np.all(np.isfinite(self._stations)) or not np.all(
            np.isfinite(self._widths)
        ):
            raise ValueError("width samples contain non-finite values")
        if np.any(np.diff(self._stations) <= DEFAULT_CONFIG.geometry.epsilon):
            raise ValueError("width stations must be strictly increasing")
        self.total_length = float(self._stations[-1])

    def evaluate(self, s: float, derivative: int = 0) -> np.ndarray:
        s_clamped = float(np.clip(s, float(self._stations[0]), self.total_length))
        idx = int(np.searchsorted(self._stations, s_clamped, side="right") - 1)
        idx = max(0, min(idx, len(self._stations) - 2))
        segment_length = float(self._stations[idx + 1] - self._stations[idx])
        slope = float((self._widths[idx + 1] - self._widths[idx]) / segment_length)
        if derivative == 1:
            return np.array([1.0, slope, 0.0], dtype=float)
        width = float(self._widths[idx] + slope * (s_clamped - self._stations[idx]))
        return np.array([s_clamped, width, 0.0], dtype=float)

    def evaluate_arc_length(self, s: float, derivative: int = 0) -> np.ndarray:
        return self.evaluate(s, derivative)

    def get_width_at_arc_length(self, s: float) -> float:
        return float(self.evaluate(s)[1])

    def get_polynomial_segments(self) -> list[tuple[float, float, float, float, float]]:
        return _piecewise_linear_width_segments(self._stations, self._widths)


def _with_endpoint_overrides(
    points: np.ndarray,
    start_xyz_override: Optional[tuple[float, float, float]],
    end_xyz_override: Optional[tuple[float, float, float]],
) -> np.ndarray:
    if start_xyz_override is None and end_xyz_override is None:
        return points
    overridden = points.copy()
    if start_xyz_override is not None:
        override = np.asarray(start_xyz_override, dtype=float)
        if override.shape != (3,):
            raise ValueError(
                f"start_xyz_override must have shape (3,), got {override.shape}"
            )
        overridden[0] = override
    if end_xyz_override is not None:
        override = np.asarray(end_xyz_override, dtype=float)
        if override.shape != (3,):
            raise ValueError(
                f"end_xyz_override must have shape (3,), got {override.shape}"
            )
        overridden[-1] = override
    return overridden


class TopologyReferenceGeometry:
    """Wrapper for the existing smooth spline used by topology code.

    The wrapper makes topology-domain use explicit without changing the
    underlying :class:`Splines` object or any existing topology algorithms.
    """

    def __init__(self, spline: Splines):
        self._spline = spline

    @property
    def spline(self) -> Splines:
        """Return the wrapped topology spline."""
        return self._spline

    @property
    def length(self) -> float:
        """Topology station length."""
        return float(self._spline.total_length)

    def evaluate(self, s: float) -> ReferencePose:
        """Evaluate topology geometry at station ``s``."""
        s_clamped = float(np.clip(s, 0.0, self.length))
        point = self._spline.evaluate(s_clamped)[:2]
        derivative = self._spline.evaluate(s_clamped, derivative=1)[:2]
        norm = float(np.linalg.norm(derivative))
        if norm <= DEFAULT_CONFIG.geometry.epsilon:
            step = min(0.01, max(self.length / 1000.0, 1e-6))
            p0 = self._spline.evaluate(max(0.0, s_clamped - step))[:2]
            p1 = self._spline.evaluate(min(self.length, s_clamped + step))[:2]
            derivative = p1 - p0
        heading = _heading_from_vector(derivative)
        return ReferencePose(float(point[0]), float(point[1]), heading)

    def project(self, point_xy: Iterable[float]) -> ProjectionResult:
        """Project an XY point onto the topology spline."""
        point = np.asarray(list(point_xy), dtype=float)[:2]
        s, t = self._spline.cartesian_to_frenet(float(point[0]), float(point[1]))
        pose = self.evaluate(s)
        closest = pose.xy
        return ProjectionResult(
            s=float(s),
            t=float(t),
            heading=pose.heading,
            distance=float(np.linalg.norm(point - closest)),
        )


class EmissionReferenceGeometry:
    """Piecewise-linear emission reference geometry from source Lanelet2 points.

    This first implementation preserves the source reference boundary exactly
    as a polyline.  Callers depend only on this class' geometry API, so a future
    ParamPoly3 or adaptive implementation can replace the internals without
    leaking into topology code.
    """

    def __init__(self, source_points: Iterable[Iterable[float]]):
        points = _as_xy_array(source_points)
        if points.shape[1] == 2:
            points = np.column_stack((points, np.zeros(len(points), dtype=float)))
        self._points_3d = _clean_polyline(points)
        self._points = self._points_3d[:, :2]
        self._segments = np.diff(self._points, axis=0)
        self._segment_lengths = np.linalg.norm(self._segments, axis=1)
        if np.any(self._segment_lengths <= DEFAULT_CONFIG.geometry.epsilon):
            raise ValueError("source polyline contains zero-length segments")
        self._stations = np.concatenate(([0.0], np.cumsum(self._segment_lengths)))
        self._length = float(self._stations[-1])
        if self._length <= DEFAULT_CONFIG.geometry.epsilon:
            raise ValueError("source polyline has zero length")

    @classmethod
    def from_source_boundary(
        cls, source_points: Iterable[Iterable[float]]
    ) -> "EmissionReferenceGeometry":
        """Build emission geometry from the selected Lanelet2 reference boundary."""
        return cls(source_points)

    @property
    def source_points(self) -> np.ndarray:
        """Cleaned source boundary points."""
        return self._points.copy()

    @property
    def source_points_3d(self) -> np.ndarray:
        """Cleaned source boundary points with elevation."""
        return self._points_3d.copy()

    @property
    def source_stations(self) -> np.ndarray:
        """Monotonic source arc-length station values."""
        return self._stations.copy()

    @property
    def length(self) -> float:
        """Emission geometry length."""
        return self._length

    @property
    def source_length(self) -> float:
        """Source boundary arc length."""
        return self._length

    @property
    def min_segment_length(self) -> float:
        """Shortest retained source segment length."""
        return float(np.min(self._segment_lengths))

    def evaluate(self, s: float) -> ReferencePose:
        """Evaluate emission geometry at station ``s``."""
        s_clamped = float(np.clip(s, 0.0, self._length))
        idx = int(np.searchsorted(self._stations, s_clamped, side="right") - 1)
        idx = max(0, min(idx, len(self._segment_lengths) - 1))
        seg_len = float(self._segment_lengths[idx])
        ratio = (s_clamped - float(self._stations[idx])) / seg_len
        point = self._points[idx] + ratio * self._segments[idx]
        heading = _heading_from_vector(self._segments[idx])
        return ReferencePose(float(point[0]), float(point[1]), heading)

    def point_at_lateral_offset(self, s: float, t: float) -> np.ndarray:
        """Evaluate a world XY point at station ``s`` and lateral offset ``t``."""
        pose = self.evaluate(s)
        return pose.xy + float(t) * _left_normal(pose.heading)

    def project(
        self,
        point_xy: Iterable[float],
        *,
        preferred_s: Optional[float] = None,
        search_radius: Optional[float] = None,
    ) -> ProjectionResult:
        """Project an XY point onto the emission geometry.

        ``preferred_s`` and ``search_radius`` restrict the search to a local
        station window.  This is the mechanism used by station correspondence
        to avoid self-near geometries jumping to another branch.
        """
        point = np.asarray(list(point_xy), dtype=float)[:2]
        if point.shape != (2,) or not np.all(np.isfinite(point)):
            raise ValueError("point_xy must be a finite XY point")

        candidate_indices = range(len(self._segment_lengths))
        if preferred_s is not None and search_radius is not None:
            low = float(preferred_s) - float(search_radius)
            high = float(preferred_s) + float(search_radius)
            local_indices = [
                i
                for i in candidate_indices
                if float(self._stations[i + 1]) >= low
                and float(self._stations[i]) <= high
            ]
            if local_indices:
                candidate_indices = local_indices

        best: tuple[float, float, float, float] | None = None
        for idx in candidate_indices:
            start = self._points[idx]
            segment = self._segments[idx]
            seg_len = float(self._segment_lengths[idx])
            ratio = float(np.dot(point - start, segment) / (seg_len * seg_len))
            ratio = float(np.clip(ratio, 0.0, 1.0))
            closest = start + ratio * segment
            heading = _heading_from_vector(segment)
            lateral = float(np.dot(point - closest, _left_normal(heading)))
            s = float(self._stations[idx]) + ratio * seg_len
            distance = float(np.linalg.norm(point - closest))
            station_penalty = (
                abs(s - float(preferred_s)) if preferred_s is not None else 0.0
            )
            key = (distance, station_penalty, s, lateral)
            if best is None or key < best:
                best = key
                best_projection = ProjectionResult(
                    s=s,
                    t=lateral,
                    heading=heading,
                    distance=distance,
                )

        if best is None:
            raise ValueError("no projection candidates available")
        return best_projection

    def project_signed(
        self,
        point_xy: Iterable[float],
        *,
        preferred_s: Optional[float] = None,
        search_radius: Optional[float] = None,
    ) -> ProjectionResult:
        """Project onto the local tangent line without clamping station.

        This is intended for fidelity/coverage analysis, not for OpenDRIVE
        object placement.  A point upstream of the first segment can therefore
        return ``s < 0``; a point downstream of the final segment can return
        ``s > length``.  ``preferred_s`` should be supplied by callers that have
        source-order context so self-near or parallel geometry cannot jump to a
        globally nearer branch.
        """
        point = np.asarray(list(point_xy), dtype=float)[:2]
        if point.shape != (2,) or not np.all(np.isfinite(point)):
            raise ValueError("point_xy must be a finite XY point")

        candidate_indices = range(len(self._segment_lengths))
        if preferred_s is not None and search_radius is not None:
            low = float(preferred_s) - float(search_radius)
            high = float(preferred_s) + float(search_radius)
            local_indices = [
                i
                for i in candidate_indices
                if float(self._stations[i + 1]) >= low
                and float(self._stations[i]) <= high
            ]
            if local_indices:
                candidate_indices = local_indices

        best: tuple[float, float, float, float] | None = None
        best_projection: ProjectionResult | None = None
        for idx in candidate_indices:
            start = self._points[idx]
            segment = self._segments[idx]
            seg_len = float(self._segment_lengths[idx])
            ratio = float(np.dot(point - start, segment) / (seg_len * seg_len))
            closest = start + ratio * segment
            heading = _heading_from_vector(segment)
            lateral = float(np.dot(point - closest, _left_normal(heading)))
            s = float(self._stations[idx]) + ratio * seg_len
            distance = float(np.linalg.norm(point - closest))
            station_penalty = (
                abs(s - float(preferred_s)) if preferred_s is not None else 0.0
            )
            key = (distance, station_penalty, abs(ratio - 0.5), s)
            if best is None or key < best:
                best = key
                best_projection = ProjectionResult(
                    s=s,
                    t=lateral,
                    heading=heading,
                    distance=distance,
                )

        if best_projection is None:
            raise ValueError("no projection candidates available")
        return best_projection

    def validate(self) -> ValidationResult:
        """Validate finiteness and monotonic station ordering."""
        errors: list[str] = []
        if not np.all(np.isfinite(self._points)):
            errors.append("non-finite source points")
        if not np.all(np.isfinite(self._stations)):
            errors.append("non-finite stations")
        if np.any(np.diff(self._stations) <= DEFAULT_CONFIG.geometry.epsilon):
            errors.append("non-monotonic stations")
        if self._length <= DEFAULT_CONFIG.geometry.epsilon:
            errors.append("zero length")
        return ValidationResult(valid=not errors, errors=tuple(errors))


class StationMapping:
    """Monotonic station correspondence between topology/source/emission."""

    def __init__(
        self,
        topology_stations: Iterable[float],
        source_stations: Iterable[float],
        emission_stations: Iterable[float],
    ):
        self._topology = np.asarray(list(topology_stations), dtype=float)
        self._source = np.asarray(list(source_stations), dtype=float)
        self._emission = np.asarray(list(emission_stations), dtype=float)
        self._validate_or_raise()

    @classmethod
    def from_lengths(
        cls, topology_length: float, source_length: float, emission_length: float
    ) -> "StationMapping":
        """Build a normalized monotonic station mapping from total lengths."""
        return cls(
            [0.0, float(topology_length)],
            [0.0, float(source_length)],
            [0.0, float(emission_length)],
        )

    @property
    def topology_length(self) -> float:
        return float(self._topology[-1])

    @property
    def source_length(self) -> float:
        return float(self._source[-1])

    @property
    def emission_length(self) -> float:
        return float(self._emission[-1])

    def topology_to_source(self, s_topology: float) -> float:
        return self._interp(s_topology, self._topology, self._source)

    def source_to_topology(self, u_source: float) -> float:
        return self._interp(u_source, self._source, self._topology)

    def source_to_emission(self, u_source: float) -> float:
        return self._interp(u_source, self._source, self._emission)

    def emission_to_source(self, s_emission: float) -> float:
        return self._interp(s_emission, self._emission, self._source)

    def topology_to_emission(self, s_topology: float) -> float:
        return self.source_to_emission(self.topology_to_source(s_topology))

    def emission_to_topology(self, s_emission: float) -> float:
        return self.source_to_topology(self.emission_to_source(s_emission))

    def validate(self) -> ValidationResult:
        errors = self._validation_errors()
        return ValidationResult(valid=not errors, errors=tuple(errors))

    def _validate_or_raise(self) -> None:
        errors = self._validation_errors()
        if errors:
            raise ValueError("; ".join(errors))

    def _validation_errors(self) -> list[str]:
        errors: list[str] = []
        lengths = {len(self._topology), len(self._source), len(self._emission)}
        if len(lengths) != 1:
            errors.append("station arrays must have the same length")
        if len(self._topology) < 2:
            errors.append("at least two station breakpoints are required")
        for name, stations in (
            ("topology", self._topology),
            ("source", self._source),
            ("emission", self._emission),
        ):
            if not np.all(np.isfinite(stations)):
                errors.append(f"{name} stations contain non-finite values")
            if len(stations) >= 2:
                if abs(float(stations[0])) > DEFAULT_CONFIG.geometry.epsilon:
                    errors.append(f"{name} stations must start at 0")
                if np.any(np.diff(stations) <= DEFAULT_CONFIG.geometry.epsilon):
                    errors.append(f"{name} stations must be strictly increasing")
        return errors

    @staticmethod
    def _interp(value: float, x: np.ndarray, y: np.ndarray) -> float:
        clipped = float(np.clip(value, float(x[0]), float(x[-1])))
        return float(np.interp(clipped, x, y))


@dataclass(frozen=True)
class RoadEmissionContext:
    """Atomic emission-domain context for one source-backed OpenDRIVE road."""

    topology_geometry: TopologyReferenceGeometry
    emission_geometry: EmissionReferenceGeometry
    station_mapping: StationMapping

    @classmethod
    def from_lanelet_groups(
        cls,
        lanelet_map,
        lanelet_group,
        *,
        traffic_rule: Optional[str] = None,
        routing_graph=None,
        topology_spline: Optional[Splines] = None,
        start_xyz_override: Optional[tuple[float, float, float]] = None,
        end_xyz_override: Optional[tuple[float, float, float]] = None,
    ) -> "RoadEmissionContext":
        """Build emission geometry from the same boundary ReferenceLine selects."""
        if not lanelet_group:
            raise ValueError("Lanelet group cannot be empty")

        from ..util import extract_points_3d, sort_adjacent_groups
        from .reference_line import ReferenceLine

        sorted_lanelets = sort_adjacent_groups(
            lanelet_map,
            lanelet_group,
            routing_graph,
        )
        traffic_rule_normalized = (traffic_rule or "RHT").upper()
        if traffic_rule_normalized not in ("RHT", "LHT"):
            raise ValueError(
                f"Invalid traffic_rule: '{traffic_rule}'. Must be 'RHT' or 'LHT'."
            )

        if traffic_rule_normalized == "RHT":
            reference_lanelet = sorted_lanelets[0]
            boundary = reference_lanelet.leftBound
        else:
            reference_lanelet = sorted_lanelets[-1]
            boundary = reference_lanelet.rightBound

        source_points_3d = extract_points_3d(boundary)

        if traffic_rule_normalized == "LHT":
            centerline_points = np.array(
                [[point.x, point.y] for point in reference_lanelet.centerline],
                dtype=float,
            )
            if len(centerline_points) >= 2:
                centerline_direction = centerline_points[-1] - centerline_points[0]
                boundary_direction = source_points_3d[-1, :2] - source_points_3d[0, :2]
                centerline_norm = np.linalg.norm(centerline_direction)
                boundary_norm = np.linalg.norm(boundary_direction)
                if (
                    centerline_norm > DEFAULT_CONFIG.geometry.epsilon
                    and boundary_norm > DEFAULT_CONFIG.geometry.epsilon
                    and np.dot(
                        centerline_direction / centerline_norm,
                        boundary_direction / boundary_norm,
                    )
                    < 0.0
                ):
                    source_points_3d = source_points_3d[::-1]

        source_points_3d = _with_endpoint_overrides(
            source_points_3d,
            start_xyz_override,
            end_xyz_override,
        )
        emission_geometry = EmissionReferenceGeometry.from_source_boundary(
            source_points_3d
        )

        if topology_spline is None:
            topology_reference = ReferenceLine.construct_from_lanelet_groups(
                lanelet_map,
                lanelet_group,
                traffic_rule=traffic_rule,
                routing_graph=routing_graph,
                start_xyz_override=start_xyz_override,
                end_xyz_override=end_xyz_override,
            )
            topology_spline = topology_reference.centerline_2d

        topology_geometry = TopologyReferenceGeometry(topology_spline)
        source_stations = emission_geometry.source_stations
        topology_stations = np.linspace(
            0.0,
            topology_geometry.length,
            len(source_stations),
        )
        station_mapping = StationMapping(
            topology_stations=topology_stations,
            source_stations=source_stations,
            emission_stations=source_stations,
        )
        return cls(
            topology_geometry=topology_geometry,
            emission_geometry=emission_geometry,
            station_mapping=station_mapping,
        )

    @property
    def length(self) -> float:
        return self.emission_geometry.length

    @property
    def elevation_offset(self) -> float:
        return float(self.emission_geometry.source_points_3d[0, 2])

    def evaluate(self, s: float) -> ReferencePose:
        return self.emission_geometry.evaluate(s)

    def project(
        self,
        point_xy: Iterable[float],
        *,
        preferred_s: Optional[float] = None,
        search_radius: Optional[float] = None,
    ) -> ProjectionResult:
        return self.emission_geometry.project(
            point_xy,
            preferred_s=preferred_s,
            search_radius=search_radius,
        )

    def topology_to_emission_station(self, s_topology: float) -> float:
        return self.station_mapping.topology_to_emission(s_topology)

    def source_to_emission_station(self, u_source: float) -> float:
        return self.station_mapping.source_to_emission(u_source)

    def to_plan_view(self):
        """Create an OpenDRIVE planView that follows the emission polyline."""
        from .geometry import Line, PlanView

        points = self.emission_geometry.source_points
        stations = self.emission_geometry.source_stations
        geometries = []
        for idx in range(len(points) - 1):
            segment = points[idx + 1] - points[idx]
            length = float(stations[idx + 1] - stations[idx])
            if length <= DEFAULT_CONFIG.geometry.epsilon:
                continue
            geometries.append(
                Line(
                    s=float(stations[idx]),
                    x=float(points[idx, 0]),
                    y=float(points[idx, 1]),
                    hdg=_heading_from_vector(segment),
                    length=length,
                )
            )
        if not geometries:
            raise ValueError("emission geometry produced no planView segments")
        return PlanView(geometries=geometries)

    def to_elevation_profile(self):
        """Create absolute elevation segments aligned with emission stations."""
        from .elevation import Elevation, ElevationProfile

        points = self.emission_geometry.source_points_3d
        stations = self.emission_geometry.source_stations
        elevations = []
        for idx in range(len(points) - 1):
            length = float(stations[idx + 1] - stations[idx])
            if length <= DEFAULT_CONFIG.geometry.epsilon:
                continue
            dz_ds = float((points[idx + 1, 2] - points[idx, 2]) / length)
            elevations.append(
                Elevation(
                    s=float(stations[idx]),
                    a=float(points[idx, 2]),
                    b=dz_ds,
                    c=0.0,
                    d=0.0,
                )
            )
        if not elevations:
            raise ValueError("emission geometry produced no elevation segments")
        return ElevationProfile(elevations=elevations)

    def validate(self) -> ValidationResult:
        errors = list(self.emission_geometry.validate().errors)
        errors.extend(self.station_mapping.validate().errors)
        try:
            plan_view = self.to_plan_view()
            if not plan_view.geometries:
                errors.append("empty planView")
            for geometry in plan_view.geometries:
                values = (
                    geometry.s,
                    geometry.x,
                    geometry.y,
                    geometry.hdg,
                    geometry.length,
                )
                if not all(math.isfinite(float(value)) for value in values):
                    errors.append("non-finite planView geometry")
                if geometry.length <= DEFAULT_CONFIG.geometry.epsilon:
                    errors.append("zero-length planView geometry")
        except ValueError as exc:
            errors.append(str(exc))
        try:
            elevation_profile = self.to_elevation_profile()
            if not elevation_profile.elevations:
                errors.append("empty elevationProfile")
            for elevation in elevation_profile.elevations:
                values = (
                    elevation.s,
                    elevation.a,
                    elevation.b,
                    elevation.c,
                    elevation.d,
                )
                if not all(math.isfinite(float(value)) for value in values):
                    errors.append("non-finite elevation")
        except ValueError as exc:
            errors.append(str(exc))
        return ValidationResult(valid=not errors, errors=tuple(errors))


def estimate_lanelet_width_with_emission_geometry(
    lanelet,
    emission_geometry: EmissionReferenceGeometry,
    config: WidthEstimationConfig,
    anchor_start_override: Optional[tuple[float, float, float]] = None,
    anchor_end_override: Optional[tuple[float, float, float]] = None,
):
    """Fit lane width from local cross-sections of the source lanelet polygon."""
    if config.reference == WidthReference.CENTER_LINE:
        from ..centerline import estimate_lanelet_width_as_spline

        return estimate_lanelet_width_as_spline(lanelet, config)

    from ..centerline import _calculate_optimal_num_samples
    from ..util import extract_points_3d

    left_points = extract_points_3d(lanelet.leftBound)
    right_points = extract_points_3d(lanelet.rightBound)
    reference_points = emission_geometry.source_points_3d

    if config.reference == WidthReference.LEFT_BOUND:
        anchor_points = left_points
        other_points = right_points
        side_sign = -1.0
    elif config.reference == WidthReference.RIGHT_BOUND:
        anchor_points = right_points
        other_points = left_points
        side_sign = 1.0
    else:
        raise ValueError(f"Unsupported width reference: {config.reference}")

    anchor_points = _orient_polyline_like(anchor_points, reference_points)
    anchor_points = _with_endpoint_overrides(
        anchor_points,
        anchor_start_override,
        anchor_end_override,
    )
    other_points = _orient_polyline_like(other_points, anchor_points)

    anchor_stations = _polyline_stations(anchor_points[:, :2])
    other_stations = _polyline_stations(other_points[:, :2])
    num_samples = _calculate_optimal_num_samples(emission_geometry.length, config)
    base_stations = np.linspace(0.0, emission_geometry.length, num_samples)
    projected_vertex_stations = [
        emission_geometry.project(point[:2]).s
        for point in np.vstack((anchor_points, other_points))
    ]
    width_stations = np.asarray(
        _unique_sorted_values(
            [
                0.0,
                emission_geometry.length,
                *base_stations.tolist(),
                *emission_geometry.source_stations.tolist(),
                *projected_vertex_stations,
            ]
        ),
        dtype=float,
    )

    widths = []
    polygon_points = np.vstack((anchor_points[:, :2], other_points[::-1, :2]))

    def measure_width(s_emission: float) -> float:
        pose = emission_geometry.evaluate(float(s_emission))
        side_direction = side_sign * _left_normal(pose.heading)
        width = _polygon_cross_section_width(
            polygon_points,
            pose.xy,
            side_direction,
        )
        if width is None:
            width = _boundary_cross_section_width(
                anchor_points[:, :2],
                other_points[:, :2],
                pose.xy,
                side_direction,
            )
        if width is None:
            if abs(float(s_emission)) <= DEFAULT_CONFIG.geometry.epsilon:
                width = _lateral_endpoint_cap_width(
                    anchor_points[0, :2],
                    other_points[0, :2],
                    pose.heading,
                    side_direction,
                )
            elif (
                abs(float(s_emission) - emission_geometry.length)
                <= DEFAULT_CONFIG.geometry.epsilon
            ):
                width = _lateral_endpoint_cap_width(
                    anchor_points[-1, :2],
                    other_points[-1, :2],
                    pose.heading,
                    side_direction,
                )
        if width is None:
            t_norm = float(s_emission / emission_geometry.length)
            anchor_s = float(t_norm * anchor_stations[-1])
            other_s = float(t_norm * other_stations[-1])
            anchor_pos = _interpolate_polyline(
                anchor_points,
                anchor_stations,
                anchor_s,
            )
            other_pos = _interpolate_polyline(
                other_points,
                other_stations,
                other_s,
            )
            width = float(np.linalg.norm(anchor_pos[:2] - other_pos[:2]))
        return float(width)

    for s_emission in width_stations:
        widths.append(measure_width(float(s_emission)))

    widths_array = np.asarray(widths, dtype=float)
    widths_array[np.abs(widths_array) <= DEFAULT_CONFIG.geometry.epsilon] = 0.0
    widths_array = np.maximum(widths_array, 0.0)

    if config.adaptive_sampling:
        max_refined_samples = (
            max(
                len(width_stations),
                config.max_samples,
            )
            * DEFAULT_CONFIG.geometry.emission_width_refinement_sample_multiplier
        )
        width_stations, widths_array = _refine_piecewise_linear_width_samples(
            width_stations,
            widths_array,
            measure_width,
            max_samples=max_refined_samples,
        )
        widths_array[np.abs(widths_array) <= DEFAULT_CONFIG.geometry.epsilon] = 0.0
        widths_array = np.maximum(widths_array, 0.0)

    return _PiecewiseLinearWidthAdapter(width_stations, widths_array)


def measure_reference_domain_coverage(
    source_points: Iterable[Iterable[float]],
    emission_geometry: EmissionReferenceGeometry,
    *,
    preferred_stations: Optional[Iterable[float]] = None,
    search_radius: Optional[float] = None,
) -> ReferenceDomainCoverage:
    """Measure source coverage relative to an emission reference s-domain.

    The projection is local and signed: points may project to ``s < 0`` or
    ``s > road.length``.  When callers do not supply explicit station hints, the
    source polyline's own normalized arc length is used as the local hint.
    """
    points = _as_xy_array(source_points)
    if points.shape[1] == 2:
        points = np.column_stack((points, np.zeros(len(points), dtype=float)))
    points = _clean_polyline(points)
    stations = _polyline_stations(points[:, :2])
    source_length = float(stations[-1])

    if preferred_stations is None:
        hints = stations / source_length * emission_geometry.length
    else:
        hints = np.asarray(list(preferred_stations), dtype=float)
        if hints.shape != stations.shape:
            raise ValueError("preferred_stations must match source point count")
        if not np.all(np.isfinite(hints)):
            raise ValueError("preferred_stations contain non-finite values")

    if search_radius is None:
        local_segment = source_length / max(1, len(points) - 1)
        search_radius = max(5.0, 2.0 * local_segment)

    projected_stations = np.asarray(
        [
            emission_geometry.project_signed(
                point[:2],
                preferred_s=float(hint),
                search_radius=search_radius,
            ).s
            for point, hint in zip(points, hints)
        ],
        dtype=float,
    )
    projected_min = float(np.min(projected_stations))
    projected_max = float(np.max(projected_stations))
    reference_start = 0.0
    reference_end = emission_geometry.length

    inside_arc = 0.0
    for idx in range(len(points) - 1):
        segment_length = float(stations[idx + 1] - stations[idx])
        s0 = float(projected_stations[idx])
        s1 = float(projected_stations[idx + 1])
        span = abs(s1 - s0)
        if span <= DEFAULT_CONFIG.geometry.epsilon:
            if reference_start <= s0 <= reference_end:
                inside_arc += segment_length
            continue

        overlap_start = max(min(s0, s1), reference_start)
        overlap_end = min(max(s0, s1), reference_end)
        if overlap_end > overlap_start:
            inside_arc += segment_length * (overlap_end - overlap_start) / span

    inside_arc = min(source_length, max(0.0, inside_arc))
    outside_arc = max(0.0, source_length - inside_arc)
    covered_station_start = max(projected_min, reference_start)
    covered_station_end = min(projected_max, reference_end)
    covered_station_span = max(0.0, covered_station_end - covered_station_start)

    return ReferenceDomainCoverage(
        reference_start=reference_start,
        reference_end=reference_end,
        projected_min_station=projected_min,
        projected_max_station=projected_max,
        start_overhang=max(0.0, reference_start - projected_min),
        end_overhang=max(0.0, projected_max - reference_end),
        domain_start_gap=max(0.0, projected_min - reference_start),
        domain_end_gap=max(0.0, reference_end - projected_max),
        source_arc_length=source_length,
        source_arc_inside_domain=inside_arc,
        source_arc_outside_domain=outside_arc,
        source_coverage_ratio=inside_arc / source_length if source_length > 0 else 0.0,
        domain_coverage_ratio=(
            covered_station_span / reference_end
            if reference_end > DEFAULT_CONFIG.geometry.epsilon
            else 0.0
        ),
    )


def reproject_source_point_to_emission(
    point_xy: Iterable[float],
    *,
    topology_geometry: TopologyReferenceGeometry,
    emission_geometry: EmissionReferenceGeometry,
    station_mapping: StationMapping,
    search_radius: float = 5.0,
) -> ReprojectedPoint:
    """Reproject a physical source point onto emission geometry.

    The topology projection provides only a local station hint.  The final
    ``s/t`` is computed by continuous projection onto the emission geometry so
    physical signal/object geometry is not tied to the topology spline's
    lateral fitting error.
    """
    topology_projection = topology_geometry.project(point_xy)
    source_hint = station_mapping.topology_to_source(topology_projection.s)
    emission_hint = station_mapping.source_to_emission(source_hint)
    emission_projection = emission_geometry.project(
        point_xy,
        preferred_s=emission_hint,
        search_radius=search_radius,
    )
    return ReprojectedPoint(
        topology=topology_projection,
        emission=emission_projection,
        source_station_hint=source_hint,
    )
