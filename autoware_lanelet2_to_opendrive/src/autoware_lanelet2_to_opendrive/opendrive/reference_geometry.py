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
from typing import Iterable, Optional

import numpy as np

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
class ValidationResult:
    """Validation summary for a reference geometry or station mapping."""

    valid: bool
    errors: tuple[str, ...] = ()


def _as_xy_array(points: Iterable[Iterable[float]]) -> np.ndarray:
    array = np.asarray(list(points), dtype=float)
    if array.ndim != 2 or array.shape[1] not in (2, 3):
        raise ValueError("points must be an (N, 2) or (N, 3) array")
    xy = array[:, :2]
    if len(xy) < 2:
        raise ValueError("at least two points are required")
    if not np.all(np.isfinite(xy)):
        raise ValueError("points contain non-finite values")
    return xy


def _clean_polyline(
    points: np.ndarray,
    *,
    min_segment_length: float = DEFAULT_CONFIG.geometry.point_distance_threshold,
) -> np.ndarray:
    cleaned: list[np.ndarray] = []
    for point in points:
        if (
            not cleaned
            or float(np.linalg.norm(point - cleaned[-1])) > min_segment_length
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
        xy = _as_xy_array(source_points)
        self._points = _clean_polyline(xy)
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
