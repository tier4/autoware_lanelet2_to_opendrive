"""Geometric lanelet -> (road_id, lane_id) mapping with SHA256-based caching.

Builds a deterministic mapping by comparing OpenDRIVE road reference line
polylines against Lanelet2 boundary polylines.  In RHT the reference line
corresponds to a lanelet's **left** boundary; in LHT it corresponds to a
lanelet's **right** boundary.  Once the *reference lanelet* for a road is
identified, adjacent lanelets are discovered via shared linestring IDs and
assigned lane IDs that match the road's lane structure.

The mapping is cached as ``<stem>.mapping.json`` next to the source XODR
file and invalidated when either the XODR or OSM file content changes
(SHA256 check).
"""

from __future__ import annotations

import hashlib
import json
import logging
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Optional

import numpy as np
from tqdm import tqdm

from .config import DEFAULT_CONFIG
from .opendrive.enums import TrafficRule
from .opendrive.geometry import (
    Arc,
    GeometryBase,
    ParamPoly3,
    evaluate_plan_view_world,
    param_for_offset,
)
from .util import CONVERTIBLE_LANELET_SUBTYPES, filter_lanelets_by_subtype

if TYPE_CHECKING:
    import lanelet2.core

    from .opendrive.road import Road as ConverterRoad

logger = logging.getLogger(__name__)


class MappingMismatchError(Exception):
    """Raised when conversion mapping and geometric mapping disagree."""


#: Mapping cache files are stored next to the source XODR file.

#: Maximum mean distance (m) between a road reference line and a lanelet
#: boundary for them to be considered a match.  Defined in
#: :class:`~.config.GeoMappingConstants`, which documents the window it has
#: to sit in and why the current value sits at the top of it.
_MATCH_THRESHOLD: float = DEFAULT_CONFIG.geo_mapping.match_threshold

#: Weight for endpoint proximity penalty added to candidate distance.
#: The penalty = (start_dist + end_dist) * weight, where start_dist and
#: end_dist are the minimum distances from the reference line endpoints
#: to either endpoint of the candidate boundary.  This helps disambiguate
#: geometrically similar junction lanelets that connect different roads.
_ENDPOINT_WEIGHT: float = 0.1

#: Margin (m) by which a road reference line's bounding box is grown before
#: it is tested against a lanelet boundary's bounding box.  Boundaries that
#: fail this test are dropped without measuring a distance.  It is a filter
#: only: it has to stay comfortably above ``_MATCH_THRESHOLD`` (and above the
#: rescue pass's ``rescue_threshold_factor`` multiple of it) so that no pair
#: whose mean distance could fall under either threshold is ever discarded.
_CANDIDATE_BBOX_MARGIN: float = 10.0

#: Total geometry length (m) below which a connecting road
#: (``junction != -1``) is treated as a synthetic divergence/merge stub
#: from the issue #291 pass and excluded from geometric matching. Real
#: ``turn_direction`` connecting roads are at least several metres long;
#: synthetic stubs default to 0.01 m. The threshold of 0.5 m matches
#: ``ParamPoly3Constants.min_segment_length`` — the project's existing
#: floor for non-degenerate segments.
_SYNTHETIC_CONNECTOR_MAX_LENGTH: float = 0.5


# ---------------------------------------------------------------------------
# Stop line mapping dataclasses
# ---------------------------------------------------------------------------


@dataclass
class StopLineMappingEntry:
    """Stop line mapping entry recorded during conversion."""

    road_id: int
    signal_types: list[int]

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict) -> "StopLineMappingEntry":
        return cls(**data)


@dataclass
class SkippedStopLineEntry:
    """Skipped stop line entry recorded during conversion."""

    reason: str

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict) -> "SkippedStopLineEntry":
        return cls(**data)


# ---------------------------------------------------------------------------
# Internal dataclass for 3-phase matching
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ReferenceMatchDistance:
    """Positional error of one road's reference line against its source boundary.

    Attributes:
        max_m: Symmetric Hausdorff-style point-to-segment distance (m) -- the
            primary figure.  A mean over sample points shrinks as the road
            gets longer, so a road that is correct everywhere but one spot
            reports a small error purely because of the correct part; the
            maximum does not.
        mean_m: Symmetric mean point-to-segment distance (m), kept as a
            secondary figure: it says how much of the road is off, where
            ``max_m`` says how far the worst of it is.
        boundary_length_m: Length (m) of the lanelet boundary this was
            measured against, so exceedance counts can be normalised per
            kilometre and compared across maps of different size.  ``None``
            for records restored from a mapping written before it existed.
    """

    max_m: float
    mean_m: float
    boundary_length_m: float | None = None

    def to_dict(self, decimals: int) -> dict:
        """Serialize to a JSON-compatible dictionary, rounded to *decimals*."""
        result: dict = {
            "max_m": round(self.max_m, decimals),
            "mean_m": round(self.mean_m, decimals),
        }
        if self.boundary_length_m is not None:
            result["boundary_length_m"] = round(self.boundary_length_m, decimals)
        return result

    @classmethod
    def from_value(cls, value: object) -> "ReferenceMatchDistance":
        """Deserialize one stored value, accepting the legacy plain float.

        Mappings written before this record existed stored a bare float per
        lanelet, and that float was the *mean*.  There is no way to recover
        the maximum from it, so it is reused for both fields: the summary then
        reports what the old file actually measured rather than dropping the
        record or inventing a worse figure.
        """
        if isinstance(value, dict):
            return cls(
                max_m=float(value["max_m"]),
                mean_m=float(value["mean_m"]),
                boundary_length_m=(
                    float(value["boundary_length_m"])
                    if value.get("boundary_length_m") is not None
                    else None
                ),
            )
        mean = float(value)  # type: ignore[arg-type]
        return cls(max_m=mean, mean_m=mean, boundary_length_m=None)


@dataclass
class _RoadCandidates:
    """Per-road candidate list for the 3-phase matching algorithm."""

    road: "ConverterRoad"
    road_id: int
    lane_ids: list[int]
    is_rht: bool
    ref_line: np.ndarray
    candidates: list[tuple[float, int]]  # [(ranking_dist, lanelet_id), ...] ascending
    raw_dists: dict[int, float] = field(default_factory=dict)  # lid -> raw distance
    walk_lane_ids: list[int] = field(default_factory=list)  # Geometric walk order


# ---------------------------------------------------------------------------
# Dataclass
# ---------------------------------------------------------------------------


@dataclass
class GeoRoadLaneletMapping:
    """Mapping from Lanelet2 lanelet IDs to OpenDRIVE (road_id, lane_id)."""

    xodr_sha256: str
    osm_sha256: str
    lanelet_to_road_and_lane: dict[int, tuple[int, int]] = field(default_factory=dict)
    preprocessing_log: dict | None = None
    stop_line_mapping: dict[int, StopLineMappingEntry] | None = None
    skipped_stop_lines: dict[int, SkippedStopLineEntry] | None = None
    #: Road IDs of synthetic divergence/merge connecting roads (#291) that
    #: were deliberately excluded from geometric lanelet matching because
    #: they have no source lanelet. Recorded so consumers can tell them
    #: apart from genuine mapping failures (#493).
    skipped_synthetic_roads: list[int] | None = None
    traffic_light_config: dict | None = None
    #: Symmetric mean point-to-segment distance (m) between a road's reference
    #: line and the boundary of the lanelet that reference line was generated
    #: from, keyed by that lanelet's id. This is the conversion's positional
    #: error measured against its own Lanelet2 input, so accuracy can be
    #: reported without converting the same map twice. Keyed by lanelet id
    #: because those are input-side and stable, unlike road ids, which are
    #: reassigned whenever preprocessing changes the lanelet grouping.
    #:
    #: Every value comes from the same metric (``_REPORT_METRIC_NAME``),
    #: recomputed rather than taken from whichever matching fallback level
    #: happened to win — those levels use two different metrics and mixing
    #: them makes level-3 matches look optimistically small.
    reference_match_distances: dict[int, ReferenceMatchDistance] | None = None
    #: Subset of the ``reference_match_distances`` keys whose road was
    #: recovered by the rescue pass. Those roads are matched with a relaxed
    #: threshold (``rescue_threshold_factor`` x ``match_threshold``), so their
    #: distance may legitimately exceed the matching tolerance and a report
    #: must not claim otherwise.
    rescued_reference_lanelets: list[int] | None = None
    _road_lane_to_lanelet: dict[tuple[int, int], int] = field(
        default_factory=dict,
        init=False,
        repr=False,
    )

    def __post_init__(self) -> None:
        """Build the reverse index."""
        self._rebuild_reverse_index()

    @property
    def road_lane_to_lanelet(self) -> dict[tuple[int, int], int]:
        """Reverse mapping: (road_id, lane_id) -> lanelet_id."""
        return self._road_lane_to_lanelet

    def _rebuild_reverse_index(self) -> None:
        self._road_lane_to_lanelet = {
            v: k for k, v in self.lanelet_to_road_and_lane.items()
        }

    def to_dict(self) -> dict:
        """Serialize to a JSON-compatible dictionary."""
        result: dict = {
            "xodr_sha256": self.xodr_sha256,
            "osm_sha256": self.osm_sha256,
            "lanelet_to_road_and_lane": {
                str(k): list(v) for k, v in self.lanelet_to_road_and_lane.items()
            },
        }
        if self.stop_line_mapping is not None:
            result["stop_line_mapping"] = {
                str(k): v.to_dict() for k, v in self.stop_line_mapping.items()
            }
        if self.skipped_stop_lines is not None:
            result["skipped_stop_lines"] = {
                str(k): v.to_dict() for k, v in self.skipped_stop_lines.items()
            }
        if self.skipped_synthetic_roads is not None:
            result["skipped_synthetic_roads"] = list(self.skipped_synthetic_roads)
        if self.preprocessing_log is not None:
            result["preprocessing_log"] = self.preprocessing_log
        if self.traffic_light_config is not None:
            result["traffic_light_config"] = self.traffic_light_config
        if self.reference_match_distances is not None:
            decimals = DEFAULT_CONFIG.geo_mapping.distance_report_decimals
            result["reference_match_distances"] = {
                str(k): v.to_dict(decimals)
                for k, v in self.reference_match_distances.items()
            }
        if self.rescued_reference_lanelets is not None:
            result["rescued_reference_lanelets"] = list(self.rescued_reference_lanelets)
        return result

    @classmethod
    def from_dict(cls, data: dict) -> "GeoRoadLaneletMapping":
        """Deserialize from a JSON-compatible dictionary."""
        stop_line_mapping: dict[int, StopLineMappingEntry] | None = None
        skipped_stop_lines: dict[int, SkippedStopLineEntry] | None = None

        raw_slm = data.get("stop_line_mapping")
        if raw_slm is not None:
            stop_line_mapping = {
                int(k): StopLineMappingEntry.from_dict(v) for k, v in raw_slm.items()
            }

        raw_ssl = data.get("skipped_stop_lines")
        if raw_ssl is not None:
            skipped_stop_lines = {
                int(k): SkippedStopLineEntry.from_dict(v) for k, v in raw_ssl.items()
            }

        raw_rmd = data.get("reference_match_distances")
        reference_match_distances: dict[int, ReferenceMatchDistance] | None = None
        if raw_rmd is not None:
            reference_match_distances = {
                int(k): ReferenceMatchDistance.from_value(v) for k, v in raw_rmd.items()
            }

        return cls(
            xodr_sha256=data["xodr_sha256"],
            osm_sha256=data["osm_sha256"],
            lanelet_to_road_and_lane={
                int(k): (v[0], v[1])
                for k, v in data["lanelet_to_road_and_lane"].items()
            },
            preprocessing_log=data.get("preprocessing_log"),
            stop_line_mapping=stop_line_mapping,
            skipped_stop_lines=skipped_stop_lines,
            skipped_synthetic_roads=data.get("skipped_synthetic_roads"),
            traffic_light_config=data.get("traffic_light_config"),
            reference_match_distances=reference_match_distances,
            rescued_reference_lanelets=data.get("rescued_reference_lanelets"),
        )


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------


def _bbox(pts: np.ndarray) -> tuple[float, float, float, float]:
    """Return (min_x, min_y, max_x, max_y) bounding box."""
    return (
        float(pts[:, 0].min()),
        float(pts[:, 1].min()),
        float(pts[:, 0].max()),
        float(pts[:, 1].max()),
    )


def _bboxes_overlap(
    a: tuple[float, float, float, float],
    b: tuple[float, float, float, float],
    margin: float = 10.0,
) -> bool:
    """Check whether two bounding boxes overlap (with a margin)."""
    return not (
        a[0] > b[2] + margin
        or a[2] < b[0] - margin
        or a[1] > b[3] + margin
        or a[3] < b[1] - margin
    )


@dataclass(frozen=True)
class _BoundaryGrid:
    """Uniform grid over lanelet boundary bounding boxes.

    Answers "which boundaries could be near this road reference line?" without
    walking every boundary.  Built in the same style as
    :func:`~.util.build_lanelet_intersection_adjacency`: the cell size is the
    largest bounding-box extent in the input, which caps each boundary at 2x2
    cells and keeps the grid at O(N) entries.

    Attributes:
        cell_size: Edge length (m) of a grid cell.
        cells: Cell coordinate -> positions of the boundaries touching it.
        lids: Lanelet IDs in the iteration order of the source dict.
        boxes: Bounding boxes parallel to ``lids``.
    """

    cell_size: float
    cells: dict[tuple[int, int], list[int]]
    lids: list[int]
    boxes: list[tuple[float, float, float, float]]


def _build_boundary_grid(
    boundaries: dict[int, np.ndarray],
    bboxes: dict[int, tuple[float, float, float, float]],
) -> _BoundaryGrid:
    """Bucket boundary bounding boxes into a uniform grid.

    ``lids`` keeps the iteration order of ``boundaries`` so that a caller can
    replay the original scan order from the positions the grid returns.

    Args:
        boundaries: Lanelet ID -> boundary polyline
        bboxes: Lanelet ID -> bounding box of that polyline

    Returns:
        The populated grid.
    """
    lids = list(boundaries)
    boxes = [bboxes[lid] for lid in lids]

    max_extent = 0.0
    for box in boxes:
        max_extent = max(max_extent, box[2] - box[0], box[3] - box[1])
    cell_size = max(max_extent, DEFAULT_CONFIG.geometry.spatial_grid_min_cell_size)

    cells: dict[tuple[int, int], list[int]] = {}
    for position, box in enumerate(boxes):
        for cell_x in range(int(box[0] // cell_size), int(box[2] // cell_size) + 1):
            for cell_y in range(int(box[1] // cell_size), int(box[3] // cell_size) + 1):
                cells.setdefault((cell_x, cell_y), []).append(position)

    return _BoundaryGrid(cell_size=cell_size, cells=cells, lids=lids, boxes=boxes)


def _grid_candidate_positions(
    grid: _BoundaryGrid,
    query: tuple[float, float, float, float],
    margin: float,
) -> list[int]:
    """Positions of the boundaries that may overlap ``query`` grown by ``margin``.

    Every boundary whose box overlaps the grown query box shares at least one
    cell with it — the overlap contains a point, that point sits in some cell,
    and both the boundary and the swept cell range cover that cell.  The result
    is therefore a superset of the exact test and the filter has no false
    negatives; callers still confirm each position with
    :func:`_bboxes_overlap`.

    Args:
        grid: Grid to query
        query: ``(min_x, min_y, max_x, max_y)`` of the query box
        margin: Distance (m) by which ``query`` is grown before the lookup

    Returns:
        Positions into ``grid.lids``, ascending, so that iterating them
        reproduces the source dict's order.
    """
    cell_size = grid.cell_size
    min_x = int((query[0] - margin) // cell_size)
    max_x = int((query[2] + margin) // cell_size)
    min_y = int((query[1] - margin) // cell_size)
    max_y = int((query[3] + margin) // cell_size)

    positions: set[int] = set()
    for cell_x in range(min_x, max_x + 1):
        for cell_y in range(min_y, max_y + 1):
            bucket = grid.cells.get((cell_x, cell_y))
            if bucket is not None:
                positions.update(bucket)
    return sorted(positions)


def _directed_mean_distance(from_line: np.ndarray, to_line: np.ndarray) -> float:
    """Mean nearest-point distance from *from_line* to *to_line*.

    For each point in *from_line* the distance to the closest point in
    *to_line* is computed; the result is the mean of those distances.
    Direction-agnostic (works regardless of polyline traversal order).
    """
    # (N, 1, 2) - (1, M, 2) -> (N, M)
    diffs = from_line[:, np.newaxis, :] - to_line[np.newaxis, :, :]
    dists_sq = np.sum(diffs * diffs, axis=2)
    return float(np.mean(np.sqrt(np.min(dists_sq, axis=1))))


def _symmetric_mean_distance(line_a: np.ndarray, line_b: np.ndarray) -> float:
    """Symmetric mean distance: max of both directed mean distances.

    Using the maximum ensures that a short polyline that only overlaps
    part of a long one gets a high (=bad) score, preventing a long
    opposing-lane boundary from being preferred over a shorter same-
    direction boundary that covers the same road section.
    """
    return max(
        _directed_mean_distance(line_a, line_b),
        _directed_mean_distance(line_b, line_a),
    )


def _directed_polyline_distances(
    from_points: np.ndarray, to_line: np.ndarray
) -> np.ndarray:
    """Point-to-segment distance from every point of *from_points* to *to_line*.

    Unlike :func:`_directed_mean_distance`, the nearest point is taken on the
    *segments* of ``to_line``, not on its vertices.  The two agree only when
    ``to_line`` is sampled far more densely than ``from_points``; otherwise the
    vertex form reports the sampling gap as if it were a geometric error.  A
    reference line sampled every 0.5 m that lies exactly on a boundary with 5 m
    vertex spacing measures ~1.25 m against the vertices and 0 m against the
    segments.

    This is used for *reporting only*.  Matching keeps
    :func:`_directed_mean_distance` so that no match decision changes.
    """
    seg_start = to_line[:-1]  # (M-1, 2)
    seg_vec = to_line[1:] - seg_start  # (M-1, 2)
    seg_len_sq = np.sum(seg_vec * seg_vec, axis=1)  # (M-1,)
    # Degenerate (zero-length) segments collapse to their start point: the
    # projection parameter is forced to 0 rather than dividing by zero.
    safe_len_sq = np.where(seg_len_sq > 0.0, seg_len_sq, 1.0)

    # (N, 1, 2) - (1, M-1, 2) -> (N, M-1, 2)
    rel = from_points[:, np.newaxis, :] - seg_start[np.newaxis, :, :]
    t = np.sum(rel * seg_vec[np.newaxis, :, :], axis=2) / safe_len_sq[np.newaxis, :]
    t = np.clip(t, 0.0, 1.0)
    closest = (
        seg_start[np.newaxis, :, :] + t[:, :, np.newaxis] * seg_vec[np.newaxis, :, :]
    )
    diffs = from_points[:, np.newaxis, :] - closest
    dists_sq = np.sum(diffs * diffs, axis=2)
    return np.sqrt(np.min(dists_sq, axis=1))


def _directed_mean_polyline_distance(
    from_points: np.ndarray, to_line: np.ndarray
) -> float:
    """Mean point-to-segment distance from *from_points* to *to_line*."""
    if len(to_line) < 2:
        return _directed_mean_distance(from_points, to_line)
    return float(np.mean(_directed_polyline_distances(from_points, to_line)))


def _directed_max_polyline_distance(
    from_points: np.ndarray, to_line: np.ndarray
) -> float:
    """Worst point-to-segment distance from *from_points* to *to_line*."""
    if len(to_line) < 2:
        return _directed_mean_distance(from_points, to_line)
    return float(np.max(_directed_polyline_distances(from_points, to_line)))


def _symmetric_mean_polyline_distance(line_a: np.ndarray, line_b: np.ndarray) -> float:
    """Symmetric point-to-segment distance: max of both directed means.

    The reporting counterpart of :func:`_symmetric_mean_distance`; see
    :func:`_directed_polyline_distances` for why the vertex-based form is
    unsuitable for an accuracy figure.

    Averaging over the sample points makes this figure fall as the road gets
    longer: a single localised excursion is divided by the whole length, so it
    is kept only as a secondary value.  Use
    :func:`_symmetric_max_polyline_distance` for the headline figure.
    """
    return max(
        _directed_mean_polyline_distance(line_a, line_b),
        _directed_mean_polyline_distance(line_b, line_a),
    )


def _symmetric_max_polyline_distance(line_a: np.ndarray, line_b: np.ndarray) -> float:
    """Symmetric Hausdorff-style point-to-segment distance.

    The maximum over both directed worst-case distances: the smallest ``d``
    such that every point of either polyline lies within ``d`` of the other.
    This is the primary accuracy figure, because it is the only one that
    survives the road's length — a mean dilutes a localised deviation in
    proportion to how much correct geometry surrounds it, so a road with one
    bad metre reports a small error simply for being long.
    """
    return max(
        _directed_max_polyline_distance(line_a, line_b),
        _directed_max_polyline_distance(line_b, line_a),
    )


def _polyline_length(pts: np.ndarray) -> float:
    """Total length (m) of a polyline."""
    if len(pts) < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))


#: Name of the metric behind every ``max_m`` in
#: :attr:`GeoRoadLaneletMapping.reference_match_distances`.  The matching
#: fallback levels use two different metrics (symmetric for levels 1-2,
#: one-directional for level 3), so the reported figures are *not* taken from
#: whichever level won — they are all recomputed with this single metric, and
#: the name is carried in the summary so it cannot be mistaken for another.
_REPORT_METRIC_NAME: str = "symmetric_point_to_segment_max"


def _polyline_direction(pts: np.ndarray) -> np.ndarray:
    """Return the unit direction vector of a polyline (first to last point).

    Returns the zero vector when the polyline has negligible length.
    """
    d = pts[-1] - pts[0]
    norm = float(np.linalg.norm(d))
    if norm < 1e-10:
        return np.zeros(2)
    return d / norm


def _same_direction(line_a: np.ndarray, line_b: np.ndarray) -> bool:
    """Return True if two polylines run in roughly the same direction.

    Uses the dot product of overall direction vectors; a non-negative
    value (angle <= 90 degrees) is considered "same direction".
    """
    return bool(np.dot(_polyline_direction(line_a), _polyline_direction(line_b)) >= 0.0)


# ---------------------------------------------------------------------------
# Reference-line sampling from converter Road objects
# ---------------------------------------------------------------------------


def _evaluate_geometry_world(geom: GeometryBase, p: float) -> tuple[float, float]:
    """Evaluate a single planView geometry at local arc-length ``p``.

    Dispatches to :func:`evaluate_plan_view_world` with the geometry's own
    analytic model — ``ParamPoly3`` -> cubic coefficients, ``Arc`` ->
    constant curvature, anything else (``Line``) -> straight line. This
    keeps non-``ParamPoly3`` primitives, emitted only when arc-primitive
    detection (#466) is enabled, from being mis-sampled as straight
    chords between their endpoints (#495).
    """
    if isinstance(geom, ParamPoly3):
        # `p` arrives as travelled distance; map it through the geometry's
        # declared pRange before evaluating the polynomial.
        p = param_for_offset(geom, p)
        coeffs = (
            geom.aU,
            geom.bU,
            geom.cU,
            geom.dU,
            geom.aV,
            geom.bV,
            geom.cV,
            geom.dV,
        )
        return evaluate_plan_view_world(
            geom.x, geom.y, geom.hdg, p, param_poly3_coeffs=coeffs
        )
    if isinstance(geom, Arc):
        return evaluate_plan_view_world(
            geom.x, geom.y, geom.hdg, p, arc_curvature=geom.curvature
        )
    # Line geometry (and any other primitive): straight along the heading.
    return evaluate_plan_view_world(geom.x, geom.y, geom.hdg, p)


#: Target arc-length spacing (m) between reference-line sample points.
#: The reference line is sampled at this fixed density regardless of how the
#: planView is split into <line>/<arc>/<paramPoly3> segments, so the geometric
#: matching distance does not depend on the segmentation (#499).
_REFERENCE_LINE_SAMPLE_SPACING: float = 0.5


def _sample_reference_line_from_road(
    road: "ConverterRoad", sample_spacing: float = _REFERENCE_LINE_SAMPLE_SPACING
) -> np.ndarray:
    """Sample a converter Road's planView into a uniform reference polyline.

    Sample points are spaced uniformly by arc-length across the whole
    planView at a fixed density of one point per ``sample_spacing`` metres,
    independent of how the planView is split into ``<line>``, ``<arc>`` and
    ``<paramPoly3>`` segments. Each geometry is evaluated with its own
    analytic model, so arcs are traced along their curve, not their chord.

    Both the sample count and the sample positions depend only on the
    reference *curve* — its total length and shape — not on its
    segmentation. The sampled polyline, and therefore the geometric matching
    distances computed from it, are invariant to the planView segmentation,
    so enabling arc-primitive detection (#466) cannot change which lanelet a
    road maps to (#499).

    Args:
        road: Converter Road object with ``plan_view`` containing geometries.
        sample_spacing: Target arc-length spacing (m) between sample points.

    Returns:
        NumPy array of shape ``(N, 2)`` with global (x, y) coordinates.
    """
    if road.plan_view is None or not road.plan_view.geometries:
        return np.empty((0, 2))

    geometries = road.plan_view.geometries

    # Cumulative arc-length at the start of each geometry; the final entry
    # is the total planView length.
    segment_starts = np.concatenate([[0.0], np.cumsum([g.length for g in geometries])])
    total_length = float(segment_starts[-1])
    if total_length <= 0.0:
        return np.empty((0, 2))

    # Number of evenly spaced intervals along the whole planView. Derived
    # from the total length only, so two planViews tracing the same curve
    # with different segmentations yield the same sample points.
    num_intervals = max(1, round(total_length / sample_spacing))
    points: list[list[float]] = []
    for k in range(num_intervals + 1):
        s = total_length * k / num_intervals
        # Locate the geometry that contains global arc-length ``s``.
        idx = int(np.searchsorted(segment_starts, s, side="right")) - 1
        idx = min(max(idx, 0), len(geometries) - 1)
        x, y = _evaluate_geometry_world(geometries[idx], s - segment_starts[idx])
        points.append([x, y])

    return np.array(points)


# ---------------------------------------------------------------------------
# Lane-ID extraction from converter Road objects
# ---------------------------------------------------------------------------


def _get_driving_lane_ids_from_road(road: "ConverterRoad") -> list[int]:
    """Return sorted non-zero lane IDs from a converter Road's first lane section.

    Sorted by ``abs(id)`` so that the innermost lane (closest to the
    reference line) comes first.
    """
    if road.lanes is None or not road.lanes.lane_sections:
        return []
    section = road.lanes.lane_sections[0]
    ids = list(section.left_lanes.keys()) + list(section.right_lanes.keys())
    return sorted(ids, key=abs)


# ---------------------------------------------------------------------------
# SHA256 helper
# ---------------------------------------------------------------------------


def _sha256_of_file(path: Path) -> str:
    """Compute SHA256 hex digest of a file."""
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(8192), b""):
            h.update(chunk)
    return h.hexdigest()


# ---------------------------------------------------------------------------
# Parse XODR XML to converter-compatible Road objects
# ---------------------------------------------------------------------------


def parse_roads_from_xodr(
    xodr_path: Path, xodr_root: object | None = None
) -> list["ConverterRoad"]:
    """Parse XODR XML and construct converter-compatible Road objects.

    Reads ``<road>`` elements from the XODR file, extracting ``<planView>``
    geometries (``<paramPoly3>``) and lane IDs from ``<laneSection>``.
    Returns a list of ``Road`` objects usable by :func:`build_mapping`.

    Args:
        xodr_path: Path to the OpenDRIVE file (unused when *xodr_root* given).
        xodr_root: Pre-parsed XML root element.  Avoids re-parsing the file.

    Returns:
        List of converter ``Road`` objects with ``plan_view`` and ``lanes``
        populated from the XODR XML.
    """
    import lxml.etree as ET

    from .opendrive.enums import LaneType
    from .opendrive.geometry import Arc, Line, ParamPoly3, PlanView
    from .opendrive.lane import Lane
    from .opendrive.lane_section import LaneSection
    from .opendrive.lane_sections import Lanes
    from .opendrive.reference_line import ReferenceLine
    from .opendrive.road import Road

    if xodr_root is not None:
        root = xodr_root
    else:
        root = ET.parse(str(xodr_path)).getroot()
    roads: list[Road] = []

    for road_elem in root.findall(".//road"):
        road_id = int(road_elem.get("id", "0"))
        junction = int(road_elem.get("junction", "-1"))

        # Parse planView geometries (line, arc and paramPoly3).
        geometries: list[GeometryBase] = []
        plan_view_elem = road_elem.find("planView")
        if plan_view_elem is not None:
            for geom_elem in plan_view_elem.findall("geometry"):
                s = float(geom_elem.get("s", "0"))
                x = float(geom_elem.get("x", "0"))
                y = float(geom_elem.get("y", "0"))
                hdg = float(geom_elem.get("hdg", "0"))
                length = float(geom_elem.get("length", "0"))
                pp3_elem = geom_elem.find("paramPoly3")
                arc_elem = geom_elem.find("arc")
                if pp3_elem is not None:
                    geometries.append(
                        ParamPoly3(
                            s=s,
                            x=x,
                            y=y,
                            hdg=hdg,
                            length=length,
                            aU=float(pp3_elem.get("aU", "0")),
                            bU=float(pp3_elem.get("bU", "0")),
                            cU=float(pp3_elem.get("cU", "0")),
                            dU=float(pp3_elem.get("dU", "0")),
                            aV=float(pp3_elem.get("aV", "0")),
                            bV=float(pp3_elem.get("bV", "0")),
                            cV=float(pp3_elem.get("cV", "0")),
                            dV=float(pp3_elem.get("dV", "0")),
                            pRange=pp3_elem.get("pRange", "arcLength"),
                        )
                    )
                elif arc_elem is not None:
                    geometries.append(
                        Arc(
                            s=s,
                            x=x,
                            y=y,
                            hdg=hdg,
                            length=length,
                            curvature=float(arc_elem.get("curvature", "0")),
                        )
                    )
                elif geom_elem.find("line") is not None:
                    geometries.append(Line(s=s, x=x, y=y, hdg=hdg, length=length))
                else:
                    logger.warning(
                        "parse_roads_from_xodr: road %d has an unsupported "
                        "planView geometry; skipped",
                        road_id,
                    )

        if not geometries:
            continue

        plan_view = PlanView(geometries=geometries)

        # Parse lane IDs from first laneSection
        lane_section = LaneSection(s_offset=0.0)
        lanes_elem = road_elem.find("lanes")
        if lanes_elem is not None:
            first_section = lanes_elem.find("laneSection")
            if first_section is not None:
                # Parse left lanes
                left_elem = first_section.find("left")
                if left_elem is not None:
                    for lane_elem in left_elem.findall("lane"):
                        lid = int(lane_elem.get("id", "0"))
                        if lid > 0:
                            lane = Lane(lane_id=lid, lane_type=LaneType.DRIVING)
                            lane_section.left_lanes[lid] = lane

                # Parse right lanes
                right_elem = first_section.find("right")
                if right_elem is not None:
                    for lane_elem in right_elem.findall("lane"):
                        lid = int(lane_elem.get("id", "0"))
                        if lid < 0:
                            lane = Lane(lane_id=lid, lane_type=LaneType.DRIVING)
                            lane_section.right_lanes[lid] = lane

                # Set a dummy center lane
                dummy_ref = ReferenceLine.__new__(ReferenceLine)
                dummy_ref._lane = Lane(
                    lane_id=0,
                    lane_type=LaneType.NONE,
                    level=False,
                )
                lane_section.center_lane = dummy_ref

        lanes_obj = Lanes(lane_sections=[lane_section])

        road = Road(
            id=road_id,
            junction=junction,
            plan_view=plan_view,
            lanes=lanes_obj,
        )
        roads.append(road)

    return roads


# ---------------------------------------------------------------------------
# Phase 1 & 2 helpers for 3-phase matching
# ---------------------------------------------------------------------------


def _synthetic_connector_road_ids(roads: list["ConverterRoad"]) -> set[int]:
    """Road IDs of synthetic divergence/merge connecting roads (#291, #493).

    A connecting road (``junction != -1``) whose total planView length is
    below ``_SYNTHETIC_CONNECTOR_MAX_LENGTH`` is a synthetic stub from the
    divergence/merge synthesis pass — it has no backing lanelet. Real
    ``turn_direction`` connecting roads are at least several metres long.

    The check depends only on ``junction`` and planView length, so callers
    can identify these roads with a lightweight pass — without running the
    full geometric mapping.
    """
    skipped: set[int] = set()
    for road in roads:
        if road.junction != -1 and road.plan_view is not None:
            total_length = sum(g.length for g in road.plan_view.geometries)
            if total_length < _SYNTHETIC_CONNECTOR_MAX_LENGTH:
                skipped.add(road.id)
    return skipped


def _compute_all_candidates(
    roads: list["ConverterRoad"],
    lanelet_left: dict[int, np.ndarray],
    lanelet_right: dict[int, np.ndarray],
    lanelet_left_bbox: dict[int, tuple[float, float, float, float]],
    lanelet_right_bbox: dict[int, tuple[float, float, float, float]],
) -> tuple[list[_RoadCandidates], dict[int, dict], set[int]]:
    """Phase 1: compute candidate lanelet lists for every road.

    For each road, applies bbox -> direction -> distance filtering against
    **all** lanelets (no ``matched_lanelets`` exclusion) and returns candidates
    sorted by ascending distance.

    Returns:
        Tuple of (candidate list, diagnostic dict keyed by road_id for
        roads with zero candidates, set of road IDs deliberately skipped as
        synthetic divergence/merge connectors — see ``#493``).
    """
    all_rc: list[_RoadCandidates] = []
    no_candidate_diag: dict[int, dict] = {}

    # Synthetic divergence/merge connecting roads (#291) have no backing
    # lanelets, so geometric matching would either drop them silently or
    # claim a regular lanelet from the divergence-point neighbourhood —
    # poisoning Phase 2 conflict resolution and the Phase 3 adjacency walk.
    # Exclude them up front and report them separately (#493).
    skipped_synthetic = _synthetic_connector_road_ids(roads)

    # Uniform grids over the two boundary populations, built on first use so
    # that a map with a single traffic rule never pays for the unused side.
    grids: dict[bool, Optional[_BoundaryGrid]] = {True: None, False: None}

    for road in tqdm(roads, desc="Computing road candidates", unit="road"):
        if road.id in skipped_synthetic:
            continue

        ref_line = _sample_reference_line_from_road(road)
        if len(ref_line) < 2:
            continue

        lane_ids = _get_driving_lane_ids_from_road(road)
        if not lane_ids:
            continue

        # Determine RHT/LHT from road's traffic rule when available.
        # The traffic rule determines which lanelet boundary the reference
        # line was built from:
        #   RHT → left boundary  →  search lanelet_left
        #   LHT → right boundary →  search lanelet_right
        if road.rule is not None:
            is_rht = road.rule == TrafficRule.RHT
        else:
            is_rht = all(lid < 0 for lid in lane_ids)

        boundaries = lanelet_left if is_rht else lanelet_right
        grid = grids[is_rht]
        if grid is None:
            grid = _build_boundary_grid(
                boundaries, lanelet_left_bbox if is_rht else lanelet_right_bbox
            )
            grids[is_rht] = grid
        ref_bbox = _bbox(ref_line)

        # Grid pre-filter.  ``_grid_candidate_positions`` returns a superset of
        # the boundaries whose box overlaps ``ref_bbox``, in ascending position
        # order, and the exact test below trims it to precisely the set the
        # earlier exhaustive scan kept — in the same order, which matters
        # because the nearest-rejected diagnostic breaks ties by first-seen.
        # Every boundary is bbox-tested exactly once instead of once per
        # fallback level, so the skip count is also hoisted out of the loop.
        near: list[tuple[int, np.ndarray]] = []
        for position in _grid_candidate_positions(
            grid, ref_bbox, _CANDIDATE_BBOX_MARGIN
        ):
            if _bboxes_overlap(
                ref_bbox, grid.boxes[position], margin=_CANDIDATE_BBOX_MARGIN
            ):
                near_lid = grid.lids[position]
                near.append((near_lid, boundaries[near_lid]))
        n_bbox_skip = len(boundaries) - len(near)

        # Progressive fallback search with 3 levels:
        #   1. Symmetric distance + direction check  (strictest)
        #   2. Symmetric distance, no direction check (curved/long roads)
        #   3. Directed distance, no direction check  (length mismatch)
        #
        # Symmetric distance is preferred because it penalises partial
        # overlaps (e.g. an adjacent lane boundary covering part of a long
        # reference line).  Directed distance is only used as a last resort
        # for roads where the reference line is much longer than any single
        # lanelet boundary.
        _FALLBACK_LEVELS: list[tuple[bool, str]] = [
            (True, "symmetric"),
            (False, "symmetric"),
            (False, "directed"),
        ]

        candidates: list[tuple[float, int]] = []
        raw_dists: dict[int, float] = {}
        best_rejected_dist: float = float("inf")
        best_rejected_lid: Optional[int] = None
        n_dir_skip = 0

        # Pre-compute reference line endpoints for endpoint penalty
        ref_start = ref_line[0]
        ref_end = ref_line[-1]

        # A boundary that survives into more than one fallback level would
        # otherwise be measured against the same reference line again.  The
        # directed distance is also the first half of the symmetric one
        # (``_symmetric_mean_distance(boundary, ref_line)`` is the max of the
        # two directions), so both levels share a single measurement.
        directed_cache: dict[int, float] = {}
        symmetric_cache: dict[int, float] = {}

        def _distance(lid: int, boundary: np.ndarray, metric: str) -> float:
            directed = directed_cache.get(lid)
            if directed is None:
                directed = _directed_mean_distance(boundary, ref_line)
                directed_cache[lid] = directed
            if metric != "symmetric":
                return directed
            symmetric = symmetric_cache.get(lid)
            if symmetric is None:
                symmetric = max(directed, _directed_mean_distance(ref_line, boundary))
                symmetric_cache[lid] = symmetric
            return symmetric

        for require_dir, metric in _FALLBACK_LEVELS:
            candidates.clear()
            raw_dists.clear()
            best_rejected_dist = float("inf")
            best_rejected_lid = None
            n_dir_skip = 0

            for lid, boundary in near:
                if require_dir and not _same_direction(ref_line, boundary):
                    n_dir_skip += 1
                    continue
                dist = _distance(lid, boundary, metric)
                if dist <= _MATCH_THRESHOLD:
                    # Endpoint proximity penalty for candidate ranking.
                    # Correct matches have aligned start/end points;
                    # wrong junction lanelets connect different roads and
                    # have divergent endpoints.
                    ep_start = min(
                        float(np.linalg.norm(ref_start - boundary[0])),
                        float(np.linalg.norm(ref_start - boundary[-1])),
                    )
                    ep_end = min(
                        float(np.linalg.norm(ref_end - boundary[0])),
                        float(np.linalg.norm(ref_end - boundary[-1])),
                    )
                    ranking_dist = dist + (ep_start + ep_end) * _ENDPOINT_WEIGHT
                    candidates.append((ranking_dist, lid))
                    raw_dists[lid] = dist
                elif dist < best_rejected_dist:
                    best_rejected_dist = dist
                    best_rejected_lid = lid

            if candidates:
                if metric != "symmetric" or not require_dir:
                    logger.debug(
                        "Road %d: found %d candidates at fallback level "
                        "(dir=%s, metric=%s)",
                        road.id,
                        len(candidates),
                        require_dir,
                        metric,
                    )
                break  # found candidates, no need to fall back further

        candidates.sort()  # ascending by distance

        if candidates:
            # Compute geometric walk order for lane IDs.
            # RHT walks right (leftmost → rightmost):
            #   positive IDs descending + negative IDs ascending by abs
            # LHT walks left (rightmost → leftmost):
            #   negative IDs descending by abs + positive IDs ascending
            positive = sorted([lid for lid in lane_ids if lid > 0])
            negative = sorted([lid for lid in lane_ids if lid < 0], key=abs)
            if is_rht:
                walk_lane_ids = list(reversed(positive)) + negative
            else:
                walk_lane_ids = list(reversed(negative)) + positive

            all_rc.append(
                _RoadCandidates(
                    road=road,
                    road_id=road.id,
                    lane_ids=lane_ids,
                    is_rht=is_rht,
                    ref_line=ref_line,
                    candidates=candidates,
                    raw_dists=dict(raw_dists),
                    walk_lane_ids=walk_lane_ids,
                )
            )
        else:
            no_candidate_diag[road.id] = {
                "lane_ids": lane_ids,
                "is_rht": is_rht,
                "rule": str(road.rule) if road.rule else None,
                "ref_pts": len(ref_line),
                "n_bbox_skip": n_bbox_skip,
                "n_dir_skip": n_dir_skip,
                "nearest_dist": round(best_rejected_dist, 3)
                if best_rejected_lid is not None
                else None,
                "nearest_lid": best_rejected_lid,
            }

    return all_rc, no_candidate_diag, skipped_synthetic


def _resolve_conflicts(
    all_rc: list[_RoadCandidates],
) -> dict[int, int]:
    """Phase 2: iteratively resolve conflicts when multiple roads claim the same lanelet.

    For two-way conflicts, uses cost-aware tie-breaking: picks the resolution
    that minimises the total distance of the two involved roads (current
    assignment plus the loser's next-best alternative).  This prevents
    suboptimal greedy choices that lead to unnecessary drops or swaps.

    Returns a mapping ``rc_index -> candidates_list_index`` indicating which
    candidate each road should use as its reference lanelet.  Roads that
    cannot be assigned any candidate are excluded from the result.
    """
    # assignment[rc_idx] = index into all_rc[rc_idx].candidates
    assignment: dict[int, int] = {i: 0 for i in range(len(all_rc))}

    # Both loops below run for an unknown number of rounds, so the bars count
    # rounds rather than showing a percentage.  Without them this phase is
    # silent, and on a large map that silence is indistinguishable from a hang.
    rounds = tqdm(desc="Resolving candidate conflicts", unit="round")

    while True:
        rounds.update()
        # Build claims: lanelet_id -> [(rc_idx, distance), ...]
        claims: dict[int, list[tuple[int, float]]] = {}
        for rc_idx, cand_idx in assignment.items():
            rc = all_rc[rc_idx]
            if cand_idx >= len(rc.candidates):
                continue
            dist, lid = rc.candidates[cand_idx]
            claims.setdefault(lid, []).append((rc_idx, dist))

        had_conflict = False
        for lid, claimants in claims.items():
            if len(claimants) <= 1:
                continue

            # Sort by (distance, road_id) for deterministic greedy order
            claimants.sort(key=lambda x: (x[1], all_rc[x[0]].road_id))

            if len(claimants) == 2:
                # 2-way conflict with cost-aware tie-breaking.
                winner_idx, winner_dist = claimants[0]
                loser_idx, loser_dist = claimants[1]

                next_loser = assignment[loser_idx] + 1
                loser_exhausted = next_loser >= len(all_rc[loser_idx].candidates)
                next_winner = assignment[winner_idx] + 1
                winner_exhausted = next_winner >= len(all_rc[winner_idx].candidates)

                if loser_exhausted and not winner_exhausted:
                    # Save the loser: advance the winner instead
                    logger.debug(
                        "Conflict on lanelet %d: save-the-drowning — "
                        "advance winner road %d (loser road %d exhausted)",
                        lid,
                        all_rc[winner_idx].road_id,
                        all_rc[loser_idx].road_id,
                    )
                    assignment[winner_idx] += 1
                    had_conflict = True
                    continue

                if winner_exhausted and not loser_exhausted:
                    # Winner exhausted but loser has alternatives — advance loser
                    logger.debug(
                        "Conflict on lanelet %d: advance loser road %d "
                        "(winner road %d exhausted)",
                        lid,
                        all_rc[loser_idx].road_id,
                        all_rc[winner_idx].road_id,
                    )
                    assignment[loser_idx] += 1
                    had_conflict = True
                    continue

                if loser_exhausted and winner_exhausted:
                    # Both exhausted — loser is dropped
                    logger.debug(
                        "Conflict on lanelet %d: both exhausted — "
                        "drop loser road %d",
                        lid,
                        all_rc[loser_idx].road_id,
                    )
                    assignment[loser_idx] += 1
                    had_conflict = True
                    continue

                # Both have alternatives — compare total cost
                loser_next_dist = all_rc[loser_idx].candidates[next_loser][0]
                winner_next_dist = all_rc[winner_idx].candidates[next_winner][0]
                cost_advance_loser = winner_dist + loser_next_dist
                cost_advance_winner = loser_dist + winner_next_dist

                if cost_advance_winner < cost_advance_loser:
                    # Advancing the winner yields lower total cost
                    logger.debug(
                        "Conflict on lanelet %d: cost-aware — "
                        "advance winner road %d "
                        "(cost_adv_winner=%.3f < cost_adv_loser=%.3f)",
                        lid,
                        all_rc[winner_idx].road_id,
                        cost_advance_winner,
                        cost_advance_loser,
                    )
                    assignment[winner_idx] += 1
                else:
                    # Default: advance the loser
                    logger.debug(
                        "Conflict on lanelet %d: cost-aware — "
                        "advance loser road %d "
                        "(cost_adv_loser=%.3f <= cost_adv_winner=%.3f)",
                        lid,
                        all_rc[loser_idx].road_id,
                        cost_advance_loser,
                        cost_advance_winner,
                    )
                    assignment[loser_idx] += 1
                had_conflict = True
            else:
                # Multi-way conflict: greedy distance-based resolution
                for rc_idx, _ in claimants[1:]:
                    assignment[rc_idx] += 1
                    had_conflict = True

        if not had_conflict:
            break

    rounds.close()

    # Remove entries where the candidate index is out of range
    valid = {
        rc_idx: cand_idx
        for rc_idx, cand_idx in assignment.items()
        if cand_idx < len(all_rc[rc_idx].candidates)
    }

    # --- Swap detection: fix pairwise assignment swaps that greedy missed ---
    # Two roads A, B may each hold the other's ideal lanelet.  When swapping
    # reduces the total raw distance, apply the swap.  Raw (geometric)
    # distances are used instead of ranking distances (which include the
    # endpoint penalty) for more accurate swap benefit calculation.
    #
    # Only a road whose own candidate set contains A's lanelet can be A's
    # swap partner, so the pairs worth testing are found through an index
    # rather than by walking every later road: the exhaustive form is
    # quadratic in the road count and restarts after every swap, which cost
    # 22 of the 26 minutes the mapping phase took on a 25k-lanelet map.
    #
    # ``raw_dists`` holds exactly the road's candidate lanelets and never
    # changes here, so the index is built once.  ``order`` fixes each road's
    # position for the whole search — ``valid`` only ever has existing keys
    # reassigned below, never added or removed — and the per-lanelet position
    # lists come out ascending, so the pairs are still visited in the
    # ``i < j`` order of the exhaustive scan and the same swap is found first.
    order = list(valid)
    holders: dict[int, list[int]] = {}
    for position, rc_idx in enumerate(order):
        for cand_lid in all_rc[rc_idx].raw_dists:
            holders.setdefault(cand_lid, []).append(position)

    swaps = tqdm(desc="Fixing pairwise swaps", unit="pass")
    swap_found = True
    while swap_found:
        swaps.update()
        swap_found = False
        for i, rc_a in enumerate(order):
            _, lid_a = all_rc[rc_a].candidates[valid[rc_a]]
            raw_dists_a = all_rc[rc_a].raw_dists
            raw_a = raw_dists_a.get(lid_a, float("inf"))
            for j in holders.get(lid_a, ()):
                if j <= i:
                    continue
                rc_b = order[j]
                _, lid_b = all_rc[rc_b].candidates[valid[rc_b]]
                raw_dists_b = all_rc[rc_b].raw_dists
                raw_b = raw_dists_b.get(lid_b, float("inf"))

                # Does A have lid_b?  B has lid_a by construction of the index.
                raw_a_new = raw_dists_a.get(lid_b)
                if raw_a_new is None:
                    continue
                raw_b_new = raw_dists_b[lid_a]

                if raw_a_new + raw_b_new < raw_a + raw_b:
                    # Find the candidate indices for the swapped lanelets
                    ci_a_new = next(
                        ci
                        for ci, (_, cand_lid) in enumerate(all_rc[rc_a].candidates)
                        if cand_lid == lid_b
                    )
                    ci_b_new = next(
                        ci
                        for ci, (_, cand_lid) in enumerate(all_rc[rc_b].candidates)
                        if cand_lid == lid_a
                    )
                    valid[rc_a] = ci_a_new
                    valid[rc_b] = ci_b_new
                    swap_found = True
                    logger.debug(
                        "Swap fix: roads %d<->%d (lanelets %d<->%d, "
                        "raw cost %.3f->%.3f)",
                        all_rc[rc_a].road_id,
                        all_rc[rc_b].road_id,
                        lid_a,
                        lid_b,
                        raw_a + raw_b,
                        raw_a_new + raw_b_new,
                    )
                    break
            if swap_found:
                break

    swaps.close()

    return valid


# ---------------------------------------------------------------------------
# Build mapping
# ---------------------------------------------------------------------------


def build_mapping(
    lanelet_map: "lanelet2.core.LaneletMap",
    roads: list["ConverterRoad"],
    mgrs_offset: tuple[float, float],
    xodr_sha256: str,
    osm_sha256: str,
) -> GeoRoadLaneletMapping:
    """Build lanelet -> (road_id, lane_id) mapping by boundary shape comparison.

    Uses the converter's own ``Road`` objects (with ``plan_view`` and ``lanes``)
    to sample reference-line polylines and extract lane IDs.  This avoids the
    need to re-parse the XODR with an external library like pyxodr.

    Algorithm
    ---------
    1. For each road, determine RHT/LHT from its lane IDs
       (negative-only = RHT, positive-only = LHT).
    2. RHT: road reference line was generated from a lanelet's **left**
       boundary.  LHT: from a lanelet's **right** boundary.
    3. Compare the road's reference line polyline against the candidate
       lanelet boundaries.  Candidates are the lanelets the conversion can
       map — ``CONVERTIBLE_LANELET_SUBTYPES`` plus anything carrying
       ``turn_direction`` — so that subtypes the converter never emits
       (crosswalks, for instance) cannot win a road.  The candidate with the
       smallest mean nearest-point distance (below ``_MATCH_THRESHOLD``) is
       the *reference lanelet*.
    4. From the reference lanelet, walk adjacent lanelets (via shared
       boundary linestring IDs) and assign lane IDs from the road's
       lane structure.

    Args:
        lanelet_map: Lanelet2 map.
        roads: List of converter ``Road`` objects (from ``opendrive.road``).
        mgrs_offset: ``(offset_x, offset_y)`` subtracted from lanelet coords.
        xodr_sha256: SHA256 of the XODR file.
        osm_sha256: SHA256 of the OSM file.

    Returns:
        :class:`GeoRoadLaneletMapping` built by geometric comparison.
    """
    offset_x, offset_y = mgrs_offset

    # -- Pre-compute lanelet boundary data --------------------------------

    lanelet_left: dict[int, np.ndarray] = {}  # lanelet_id -> (N,2)
    lanelet_right: dict[int, np.ndarray] = {}
    lanelet_left_bbox: dict[int, tuple[float, float, float, float]] = {}
    lanelet_right_bbox: dict[int, tuple[float, float, float, float]] = {}

    # Adjacency index: boundary linestring id -> [lanelet_ids]
    left_bound_to_lanelets: dict[int, list[int]] = {}
    right_bound_to_lanelets: dict[int, list[int]] = {}

    # Restrict the candidate pool to the lanelets the conversion can actually
    # place on a road, so both mappings search the same population.  Regular
    # roads come from ``CONVERTIBLE_LANELET_SUBTYPES`` (see
    # ``Road.construct_from_lanelet_map``) and connecting roads come from the
    # lanelets carrying ``turn_direction`` (see
    # ``Junction.construct_from_lanelet_map``), so the union of the two is the
    # exact domain of the conversion-time mapping.  Without this filter a
    # crosswalk — which has no conversion-time entry — could still win a road
    # as its geometric match and be reported as a mismatch.
    #
    # ``filter_lanelets_by_subtype`` returns a set, so the ids are collected
    # first and the layer is re-walked in its own order: downstream nearest
    # neighbour searches break ties by iteration order, and the mapping is
    # documented as deterministic.
    all_lanelets = list(lanelet_map.laneletLayer)
    convertible_ids = {
        ll.id
        for ll in filter_lanelets_by_subtype(all_lanelets, CONVERTIBLE_LANELET_SUBTYPES)
    }
    lanelets = [
        ll
        for ll in all_lanelets
        if ll.id in convertible_ids or "turn_direction" in ll.attributes
    ]
    n_excluded = len(all_lanelets) - len(lanelets)
    if n_excluded:
        logger.info(
            "Geometric mapping candidate pool: %d of %d lanelets "
            "(%d excluded as non-convertible subtypes)",
            len(lanelets),
            len(all_lanelets),
            n_excluded,
        )

    for ll in tqdm(lanelets, desc="Pre-computing lanelet boundaries", unit="lanelet"):
        lid = ll.id
        lp = np.array([(p.x - offset_x, p.y - offset_y) for p in ll.leftBound])
        rp = np.array([(p.x - offset_x, p.y - offset_y) for p in ll.rightBound])
        if len(lp) >= 2:
            lanelet_left[lid] = lp
            lanelet_left_bbox[lid] = _bbox(lp)
        if len(rp) >= 2:
            lanelet_right[lid] = rp
            lanelet_right_bbox[lid] = _bbox(rp)
        left_bound_to_lanelets.setdefault(ll.leftBound.id, []).append(lid)
        right_bound_to_lanelets.setdefault(ll.rightBound.id, []).append(lid)

    # -- Adjacency helpers ------------------------------------------------

    #: Maximum mean distance (m) between two boundary polylines for them to be
    #: considered geometrically adjacent when linestring IDs do not match.
    _GEO_ADJACENCY_THRESHOLD: float = 1.5

    def _right_neighbor(lanelet_id: int) -> Optional[int]:
        """Lanelet whose left boundary matches this lanelet's right boundary.

        First tries shared linestring ID; falls back to geometric proximity.
        """
        ll = lanelet_map.laneletLayer[lanelet_id]
        for cand in left_bound_to_lanelets.get(ll.rightBound.id, []):
            if cand != lanelet_id:
                return cand

        # Geometric fallback: find a lanelet whose left boundary is close
        # to this lanelet's right boundary.
        if lanelet_id not in lanelet_right:
            return None
        my_right = lanelet_right[lanelet_id]
        my_bbox = lanelet_right_bbox[lanelet_id]
        best_cand: Optional[int] = None
        best_dist = _GEO_ADJACENCY_THRESHOLD
        for cand, cand_left in lanelet_left.items():
            if cand == lanelet_id or cand in matched_lanelets:
                continue
            if not _bboxes_overlap(my_bbox, lanelet_left_bbox[cand], margin=3.0):
                continue
            dist = _directed_mean_distance(my_right, cand_left)
            if dist < best_dist:
                best_dist = dist
                best_cand = cand
        return best_cand

    def _left_neighbor(lanelet_id: int) -> Optional[int]:
        """Lanelet whose right boundary matches this lanelet's left boundary.

        First tries shared linestring ID; falls back to geometric proximity.
        """
        ll = lanelet_map.laneletLayer[lanelet_id]
        for cand in right_bound_to_lanelets.get(ll.leftBound.id, []):
            if cand != lanelet_id:
                return cand

        # Geometric fallback: find a lanelet whose right boundary is close
        # to this lanelet's left boundary.
        if lanelet_id not in lanelet_left:
            return None
        my_left = lanelet_left[lanelet_id]
        my_bbox = lanelet_left_bbox[lanelet_id]
        best_cand: Optional[int] = None
        best_dist = _GEO_ADJACENCY_THRESHOLD
        for cand, cand_right in lanelet_right.items():
            if cand == lanelet_id or cand in matched_lanelets:
                continue
            if not _bboxes_overlap(my_bbox, lanelet_right_bbox[cand], margin=3.0):
                continue
            dist = _directed_mean_distance(my_left, cand_right)
            if dist < best_dist:
                best_dist = dist
                best_cand = cand
        return best_cand

    # -- 3-phase matching --------------------------------------------------

    mapping: dict[int, tuple[int, int]] = {}
    matched_lanelets: set[int] = set()

    # The adjacency walks below check ``resolved_references`` but not
    # ``matched_lanelets``, so a later road can silently claim a lanelet an
    # earlier road already committed.  Whether the first or the second claim is
    # the correct one is not decidable here, so the walks are left alone and
    # the collisions are recorded for reporting instead.
    overwrites: list[tuple[int, tuple[int, int], tuple[int, int]]] = []

    def _commit(walk: list[tuple[int, int, int]]) -> None:
        """Write walk results into ``mapping``, recording any re-assignment."""
        for lid, rid, lane_id in walk:
            previous = mapping.get(lid)
            if previous is not None and previous != (rid, lane_id):
                overwrites.append((lid, previous, (rid, lane_id)))
            mapping[lid] = (rid, lane_id)
            matched_lanelets.add(lid)

    # Phase 1: compute candidate lists for every road (no exclusion)
    all_rc, no_candidate_diag, skipped_synthetic = _compute_all_candidates(
        roads,
        lanelet_left,
        lanelet_right,
        lanelet_left_bbox,
        lanelet_right_bbox,
    )

    # Phase 2: resolve conflicts iteratively
    assignment = _resolve_conflicts(all_rc)

    # Positional accuracy of every accepted match, for reporting only.
    #
    # The figures are recomputed rather than read out of ``raw_dists``, for
    # three reasons: ``raw_dists`` holds whichever metric the winning fallback
    # level used (symmetric for levels 1-2, one-directional for level 3), which
    # would mix two scales in one summary; it measures against boundary
    # *vertices*, which charges the boundary's vertex spacing as if it were
    # error; and the Phase-3 pre-walk may move the reference to a lanelet that
    # ``raw_dists`` has no entry for at all.
    reference_match_distances: dict[int, ReferenceMatchDistance] = {}
    rescued_reference_lanelets: set[int] = set()

    def _record_reference_accuracy(
        lanelet_id: int, rc: _RoadCandidates, *, rescued: bool
    ) -> None:
        """Measure ``rc``'s reference line against the boundary actually used."""
        boundaries = lanelet_left if rc.is_rht else lanelet_right
        boundary = boundaries.get(lanelet_id)
        if boundary is None:
            return
        reference_match_distances[lanelet_id] = ReferenceMatchDistance(
            max_m=_symmetric_max_polyline_distance(rc.ref_line, boundary),
            mean_m=_symmetric_mean_polyline_distance(rc.ref_line, boundary),
            boundary_length_m=_polyline_length(boundary),
        )
        if rescued:
            rescued_reference_lanelets.add(lanelet_id)

    # Build resolved_references: lanelet_id -> rc_idx
    resolved_references: dict[int, int] = {}
    for rc_idx, cand_idx in assignment.items():
        _, lid = all_rc[rc_idx].candidates[cand_idx]
        resolved_references[lid] = rc_idx

    # Phase 3: walk adjacency in distance order (closest first)
    sorted_assignments = sorted(
        assignment.items(),
        key=lambda item: all_rc[item[0]].candidates[item[1]][0],
    )

    for rc_idx, cand_idx in tqdm(
        sorted_assignments, desc="Matching roads to lanelets", unit="road"
    ):
        rc = all_rc[rc_idx]
        _, best_lid = rc.candidates[cand_idx]
        road_id = rc.road_id

        # Walk from the Phase-1/2 reference lanelet.
        # If the walk is incomplete (fewer lanes than expected) for a
        # multi-lane road, retry with a pre-walk that walks in the
        # opposite direction to find the true edge lanelet.  This
        # corrects for spline fitting error causing Phase 1 to pick a
        # non-edge lanelet (e.g. 2nd instead of 1st).
        current = best_lid
        walk_result: list[tuple[int, int, int]] = []  # [(lid, road_id, lane_id)]

        for lane_id in rc.walk_lane_ids:
            walk_result.append((current, int(road_id), lane_id))
            next_ll = _right_neighbor(current) if rc.is_rht else _left_neighbor(current)
            if next_ll is None:
                break
            if (
                next_ll in resolved_references
                and resolved_references[next_ll] != rc_idx
            ):
                break
            current = next_ll

        # Fallback pre-walk: only when initial walk is incomplete
        if len(walk_result) < len(rc.walk_lane_ids) and len(rc.walk_lane_ids) > 1:
            max_prewalk = len(rc.walk_lane_ids) - 1
            edge = best_lid
            for _ in range(max_prewalk):
                neighbor = _left_neighbor(edge) if rc.is_rht else _right_neighbor(edge)
                if neighbor is None:
                    break
                if neighbor in matched_lanelets:
                    break
                if (
                    neighbor in resolved_references
                    and resolved_references[neighbor] != rc_idx
                ):
                    break
                edge = neighbor

            if edge != best_lid:
                # Trial walk from edge to verify coverage
                trial = edge
                trial_count = 1
                for _ in range(len(rc.walk_lane_ids) - 1):
                    next_ll = (
                        _right_neighbor(trial) if rc.is_rht else _left_neighbor(trial)
                    )
                    if next_ll is None:
                        break
                    if (
                        next_ll in resolved_references
                        and resolved_references[next_ll] != rc_idx
                    ):
                        break
                    trial = next_ll
                    trial_count += 1

                if trial_count > len(walk_result):
                    # Pre-walk yields better coverage — redo walk
                    logger.debug(
                        "Road %d: pre-walk corrected reference from "
                        "lanelet %d to %d (coverage %d→%d)",
                        road_id,
                        best_lid,
                        edge,
                        len(walk_result),
                        trial_count,
                    )
                    walk_result.clear()
                    current = edge
                    for lane_id in rc.walk_lane_ids:
                        walk_result.append((current, int(road_id), lane_id))
                        next_ll = (
                            _right_neighbor(current)
                            if rc.is_rht
                            else _left_neighbor(current)
                        )
                        if next_ll is None:
                            break
                        if (
                            next_ll in resolved_references
                            and resolved_references[next_ll] != rc_idx
                        ):
                            break
                        current = next_ll

        # Commit walk results
        _commit(walk_result)

        # Record accuracy under the lanelet the walk actually started from:
        # the pre-walk above may have corrected the reference away from
        # ``best_lid``, and the distance belongs to the lanelet that was used.
        if walk_result:
            _record_reference_accuracy(walk_result[0][0], rc, rescued=False)

    # -- Rescue pass: attempt to assign dropped roads -----------------------
    # Roads that had candidates in Phase 1 but were dropped in Phase 2
    # (conflict resolution) get a second chance with a relaxed search
    # against currently unmatched lanelets.
    _RESCUE_THRESHOLD: float = (
        _MATCH_THRESHOLD * DEFAULT_CONFIG.geo_mapping.rescue_threshold_factor
    )
    assigned_rc_indices = set(assignment.keys())
    rescued_road_ids: set[int] = set()
    dropped_rc_indices = [
        rc_idx for rc_idx in range(len(all_rc)) if rc_idx not in assigned_rc_indices
    ]
    # Same grid pre-filter as Phase 1, for the same reason: the exhaustive
    # form walks every boundary on the map once per dropped road.  The grid's
    # default bbox margin below is ``_CANDIDATE_BBOX_MARGIN``, so querying it
    # with that margin returns a superset of what the exact test keeps, in the
    # boundary dict's own order — the nearest match is therefore unchanged,
    # ties included.  Built on first use so a map with no dropped roads, or
    # with a single traffic rule, never pays for it.
    rescue_grids: dict[bool, Optional[_BoundaryGrid]] = {True: None, False: None}
    for rc_idx in tqdm(dropped_rc_indices, desc="Rescuing dropped roads", unit="road"):
        rc = all_rc[rc_idx]
        boundaries = lanelet_left if rc.is_rht else lanelet_right
        bboxes = lanelet_left_bbox if rc.is_rht else lanelet_right_bbox
        grid = rescue_grids[rc.is_rht]
        if grid is None:
            grid = _build_boundary_grid(boundaries, bboxes)
            rescue_grids[rc.is_rht] = grid
        ref_bbox = _bbox(rc.ref_line)
        best_lid: Optional[int] = None
        best_dist = _RESCUE_THRESHOLD
        for position in _grid_candidate_positions(
            grid, ref_bbox, _CANDIDATE_BBOX_MARGIN
        ):
            lid = grid.lids[position]
            if lid in matched_lanelets:
                continue
            if not _bboxes_overlap(ref_bbox, grid.boxes[position]):
                continue
            dist = _symmetric_mean_distance(boundaries[lid], rc.ref_line)
            if dist < best_dist:
                best_dist = dist
                best_lid = lid

        if best_lid is not None:
            # Assign reference lanelet and walk
            current = best_lid
            walk_result_rescue: list[tuple[int, int, int]] = []
            for lane_id in rc.walk_lane_ids:
                walk_result_rescue.append((current, int(rc.road_id), lane_id))
                next_ll = (
                    _right_neighbor(current) if rc.is_rht else _left_neighbor(current)
                )
                if next_ll is None or next_ll in matched_lanelets:
                    break
                current = next_ll
            _commit(walk_result_rescue)
            # Rescued roads carry positional error like any other match — and
            # more of it, since they are accepted up to the relaxed rescue
            # threshold.  Leaving them out understates ``max`` and ``p95``.
            _record_reference_accuracy(best_lid, rc, rescued=True)
            rescued_road_ids.add(rc.road_id)
            logger.info(
                "Rescue: road %d recovered via lanelet %d (dist=%.3f, lanes=%d/%d)",
                rc.road_id,
                best_lid,
                best_dist,
                len(walk_result_rescue),
                len(rc.walk_lane_ids),
            )

    # -- Diagnostic summary ------------------------------------------------
    all_road_ids = {road.id for road in roads if road.plan_view and road.lanes}
    rc_road_ids = {rc.road_id for rc in all_rc}
    assigned_road_ids = {all_rc[rc_idx].road_id for rc_idx in assignment}

    # Synthetic divergence/merge connectors are deliberately excluded from
    # matching (#493) — they have no source lanelet, so they are reported
    # separately rather than counted as 0-candidate matching failures.
    roads_no_candidates = all_road_ids - rc_road_ids - skipped_synthetic
    roads_dropped_phase2 = rc_road_ids - assigned_road_ids
    # One pass over ``mapping`` instead of one per assigned road: the inner
    # scan below used to walk all ~18k entries for each of the ~10k roads,
    # 175M comparisons for a diagnostic line, and was the 95 seconds between
    # the last rescue and "Built lanelet-to-road mapping" on a 25k-lanelet map.
    lanes_by_road: dict[int, set[int]] = {}
    for mapped_road_id, mapped_lane_id in mapping.values():
        lanes_by_road.setdefault(mapped_road_id, set()).add(mapped_lane_id)

    roads_not_fully_mapped: list[tuple[int, tuple[int, ...]]] = []
    for rc_idx, cand_idx in assignment.items():
        rc = all_rc[rc_idx]
        mapped_lanes = lanes_by_road.get(rc.road_id, set())
        expected_lanes = set(rc.walk_lane_ids)
        if mapped_lanes != expected_lanes:
            missing_lanes = expected_lanes - mapped_lanes
            roads_not_fully_mapped.append(
                (rc.road_id, tuple(sorted(missing_lanes, key=abs)))
            )

    logger.info(
        "Built lanelet-to-road mapping: %d lanelets -> %d unique roads",
        len(mapping),
        len({v[0] for v in mapping.values()}),
    )
    if overwrites:
        msg = (
            f"  [Diag] Phase 3: {len(overwrites)} lanelets were re-assigned "
            f"by a later road (the earlier assignment was dropped); "
            f"{len({lid for lid, _, _ in overwrites})} distinct lanelets"
        )
        tqdm.write(msg)
        logger.warning(msg)
        for lid, previous, current in overwrites[:10]:
            tqdm.write(f"    lanelet {lid}: {previous} -> {current}")
    if roads_no_candidates:
        msg = (
            f"  [Diag] Phase 1: {len(roads_no_candidates)} roads had 0 "
            f"candidates (threshold={_MATCH_THRESHOLD}m): "
            f"road IDs = {sorted(roads_no_candidates)[:30]}"
        )
        tqdm.write(msg)
        logger.warning(msg)
        # Print detailed diagnostics for roads without candidates
        for rid in sorted(roads_no_candidates)[:10]:
            diag = no_candidate_diag.get(rid)
            if diag:
                tqdm.write(
                    f"    road {rid}: rule={diag['rule']}, "
                    f"is_rht={diag['is_rht']}, "
                    f"lanes={diag['lane_ids']}, "
                    f"ref_pts={diag['ref_pts']}, "
                    f"bbox_skip={diag['n_bbox_skip']}, "
                    f"dir_skip={diag['n_dir_skip']}, "
                    f"nearest_dist={diag['nearest_dist']}m "
                    f"(lanelet {diag['nearest_lid']})"
                )
    if roads_dropped_phase2:
        unrecovered = roads_dropped_phase2 - rescued_road_ids
        rescued = roads_dropped_phase2 & rescued_road_ids
        if unrecovered:
            msg = (
                f"  [Diag] Phase 2: {len(unrecovered)} roads dropped "
                f"in conflict resolution (unrecovered): "
                f"road IDs = {sorted(unrecovered)[:30]}"
            )
            tqdm.write(msg)
            logger.warning(msg)
        if rescued:
            msg = (
                f"  [Diag] Phase 2: {len(rescued)} roads dropped "
                f"in conflict resolution but rescued: "
                f"road IDs = {sorted(rescued)[:30]}"
            )
            tqdm.write(msg)
            logger.info(msg)
    if roads_not_fully_mapped:
        msg = (
            f"  [Diag] Phase 3: {len(roads_not_fully_mapped)} roads not "
            f"fully mapped (walk stopped early): "
            f"{sorted(roads_not_fully_mapped)[:30]}"
        )
        tqdm.write(msg)
        logger.warning(msg)

    return GeoRoadLaneletMapping(
        xodr_sha256=xodr_sha256,
        osm_sha256=osm_sha256,
        lanelet_to_road_and_lane=mapping,
        skipped_synthetic_roads=sorted(skipped_synthetic) or None,
        reference_match_distances=reference_match_distances or None,
        rescued_reference_lanelets=sorted(rescued_reference_lanelets) or None,
    )


# ---------------------------------------------------------------------------
# Cache path
# ---------------------------------------------------------------------------


def _cache_path_for(xodr_path: Path) -> Path:
    """Return the cache file path next to the XODR file."""
    return xodr_path.parent / f"{xodr_path.stem}.mapping.json"


def _preprocessed_osm_path_for(xodr_path: Path) -> Path:
    """Return the preprocessed OSM sidecar path next to the XODR file."""
    return xodr_path.parent / f"{xodr_path.stem}.preprocessed.osm"


# ---------------------------------------------------------------------------
# Accuracy reporting
# ---------------------------------------------------------------------------


def summarise_reference_match_accuracy(
    mapping: GeoRoadLaneletMapping,
    *,
    warn_threshold: float | None = None,
) -> dict:
    """Summarise how closely the emitted roads follow their source lanelets.

    :func:`build_mapping` measures, for every road it matches, the point-to-
    segment distance between that road's reference line and the boundary of
    the lanelet the reference line was generated from. That figure is the
    conversion's positional error against its own Lanelet2 input, and this
    turns it into something reportable.

    The distinction matters: ``_MATCH_THRESHOLD`` asks "is this the same
    road?", not "is this road accurate?". A road sitting 3.4 m from its source
    boundary -- most of a lane width -- passes the tolerance without comment.

    The headline is a **count**, not a distribution statistic: how many roads
    exceed each level of
    :attr:`~.config.GeoMappingConstants.exceedance_thresholds_m`, what
    fraction of all matched roads that is, and how many that is per kilometre
    of matched boundary. ``median_m`` and ``p95_m`` are kept but are
    *scale-dependent* -- they are named in ``scale_dependent_keys`` -- because
    they describe the mix of roads in this particular map: adding
    well-converted roads improves them without anything being fixed, and two
    maps cannot be compared by them. ``per_km`` is the figure that survives a
    change of map size.

    Roads recovered by the rescue pass are included, and their distances may
    exceed ``_MATCH_THRESHOLD`` because that pass uses a relaxed threshold;
    ``above_match_threshold`` and ``rescued_count`` make that visible instead
    of letting a summary claim every match is within tolerance.

    Because the distances are keyed by lanelet id, this works from a single
    conversion. Comparing two ``.xodr`` outputs instead requires converting
    the same map twice and cannot be keyed on road ids at all, since those are
    reassigned whenever preprocessing changes the lanelet grouping.

    Args:
        mapping: Mapping carrying ``reference_match_distances``.
        warn_threshold: Distances at or above this many metres are listed
            individually. Defaults to ``warn_threshold_factor`` times
            ``_MATCH_THRESHOLD``, i.e. the point beyond which a match is
            closer to the tolerance limit than to its own source geometry.

    Returns:
        Dictionary with the headline ``exceedances`` (one entry per configured
        level, each with ``label``, ``threshold_m``, ``count``, ``fraction``
        and ``per_km``), ``max_m`` with its ``worst_lanelet_id``,
        ``matched_boundary_length_m``, ``count``, the secondary
        ``median_m``/``p95_m``/``median_mean_m`` named by
        ``scale_dependent_keys``, plus ``match_threshold_m``,
        ``rescue_threshold_m``, ``warn_threshold_m``, ``metric``,
        ``rescued_count``, ``above_match_threshold`` and ``worst`` -- a list of
        ``{"lanelet_id", "distance_m", "mean_distance_m", "rescued"}`` at or
        above the warn threshold, worst first. Returns ``{}`` when no distances
        were recorded.
    """
    distances = mapping.reference_match_distances
    if not distances:
        return {}

    geo_config = DEFAULT_CONFIG.geo_mapping
    if warn_threshold is None:
        warn_threshold = _MATCH_THRESHOLD * geo_config.warn_threshold_factor

    rescue_threshold = _MATCH_THRESHOLD * geo_config.rescue_threshold_factor
    rescued = set(mapping.rescued_reference_lanelets or ())
    values = np.array([d.max_m for d in distances.values()], dtype=float)
    mean_values = np.array([d.mean_m for d in distances.values()], dtype=float)

    # Boundary lengths are absent in mappings written before they were
    # recorded; the density is then unavailable rather than wrong.
    lengths = [
        d.boundary_length_m
        for d in distances.values()
        if d.boundary_length_m is not None
    ]
    total_length = float(sum(lengths)) if lengths else None

    exceedances = []
    for label, threshold in geo_config.exceedance_thresholds_m:
        count = int((values > threshold).sum())
        exceedances.append(
            {
                "label": label,
                "threshold_m": float(threshold),
                "count": count,
                "fraction": count / float(values.size),
                "per_km": (
                    count * geo_config.exceedance_density_base_m / total_length
                    if total_length
                    else None
                ),
            }
        )

    worst_lanelet_id = max(distances, key=lambda lid: distances[lid].max_m)
    worst = sorted(
        (
            {
                "lanelet_id": lid,
                "distance_m": d.max_m,
                "mean_distance_m": d.mean_m,
                "rescued": lid in rescued,
            }
            for lid, d in distances.items()
            if d.max_m >= warn_threshold
        ),
        key=lambda item: -item["distance_m"],
    )

    return {
        "count": int(values.size),
        # Headline: size-independent, count-based.
        "exceedances": exceedances,
        "max_m": float(values.max()),
        "worst_lanelet_id": worst_lanelet_id,
        "matched_boundary_length_m": total_length,
        # Secondary: these depend on the mix of roads in *this* map.
        "median_m": float(np.median(values)),
        "p95_m": float(np.percentile(values, 95)),
        "median_mean_m": float(np.median(mean_values)),
        "scale_dependent_keys": ["median_m", "p95_m", "median_mean_m"],
        "match_threshold_m": _MATCH_THRESHOLD,
        "rescue_threshold_m": rescue_threshold,
        "warn_threshold_m": warn_threshold,
        "metric": _REPORT_METRIC_NAME,
        "rescued_count": sum(1 for lid in distances if lid in rescued),
        "above_match_threshold": int((values > _MATCH_THRESHOLD).sum()),
        "worst": worst,
    }


# ---------------------------------------------------------------------------
# Cross-validation
# ---------------------------------------------------------------------------


def validate_mapping_consistency(
    conversion_mapping: dict[int, tuple[int, int]],
    geo_mapping: GeoRoadLaneletMapping,
    preprocessing_log: dict | None = None,
    strict: bool = True,
) -> bool:
    """Validate that conversion-time mapping matches geometric mapping.

    Compares each entry in the conversion-time mapping against the geometric
    mapping.  A lanelet ID that maps to a different ``(road_id, lane_id)``
    pair — or that is present in only one of the two mappings — is a mismatch.

    The geometric mapping is a *cross-check*, not the conversion output: it
    re-derives the mapping from raw geometry and is subject to its own
    matching-threshold and adjacency-walk heuristics.  A disagreement
    therefore says the two derivations differ, not necessarily that the
    emitted XODR is wrong, which is why ``strict`` exists.

    Args:
        conversion_mapping: Mapping produced during conversion
            (lanelet_id -> (road_id, lane_id)).
        geo_mapping: Mapping produced by geometric boundary comparison.
        preprocessing_log: Optional preprocessing log dict (from
            ``PreprocessingLog.to_dict()``) to annotate mismatches with
            preprocessing context.
        strict: When ``True`` (the default) a mismatch raises
            :class:`MappingMismatchError`.  When ``False`` the mismatch is
            logged as a warning and reported through the return value.

    Returns:
        ``True`` when the two mappings agree, ``False`` when they do not and
        ``strict`` is ``False``.

    Raises:
        MappingMismatchError: When at least one entry differs between the two
            mappings and ``strict`` is ``True``.
    """
    # Build a set of merge-produced IDs and a lookup from the log.
    # Uses PreprocessingLog typed methods instead of raw dict parsing.
    merge_output_ids: set[int] = set()
    merge_sources: dict[int, list[int]] = {}
    if preprocessing_log:
        from .preprocess_lanelet import PreprocessingLog

        log_obj = PreprocessingLog.from_dict(preprocessing_log)
        merge_output_ids = log_obj.get_merge_output_ids()
        for mid in merge_output_ids:
            src = log_obj.get_merge_source_for(mid)
            if src is not None:
                merge_sources[mid] = src

    mismatches: list[str] = []
    geo = geo_mapping.lanelet_to_road_and_lane

    for lanelet_id, conv_value in conversion_mapping.items():
        geo_value = geo.get(lanelet_id)
        if geo_value is None:
            msg = f"  lanelet {lanelet_id}: conversion={conv_value}, geo=<missing>"
            if lanelet_id in merge_output_ids:
                src = merge_sources.get(lanelet_id, [])
                msg += f" [merge output from {src}]"
            mismatches.append(msg)
        elif conv_value != geo_value:
            msg = f"  lanelet {lanelet_id}: conversion={conv_value}, geo={geo_value}"
            if lanelet_id in merge_output_ids:
                src = merge_sources.get(lanelet_id, [])
                msg += f" [merge output from {src}]"
            mismatches.append(msg)

    # Also check for entries only in geo mapping
    for lanelet_id, geo_value in geo.items():
        if lanelet_id not in conversion_mapping:
            msg = f"  lanelet {lanelet_id}: conversion=<missing>, geo={geo_value}"
            if lanelet_id in merge_output_ids:
                src = merge_sources.get(lanelet_id, [])
                msg += f" [merge output from {src}]"
            mismatches.append(msg)

    if mismatches:
        detail = "\n".join(mismatches[:20])
        total = len(mismatches)
        summary = (
            f"Mapping mismatch: {total} entries differ between conversion-time "
            f"and geometric mappings:\n{detail}"
            + (f"\n  ... and {total - 20} more" if total > 20 else "")
        )
        if strict:
            raise MappingMismatchError(summary)
        logger.warning(
            "%s\nCross-validation is non-strict, so the conversion output is "
            "kept.  Set strict_mapping_validation=true to fail instead.",
            summary,
        )
        tqdm.write(
            f"  Mapping cross-validation: {total} mismatching entries "
            f"(non-fatal; see log for details)"
        )
        return False

    logger.info(
        "Cross-validation passed: conversion and geometric mappings agree "
        "(%d entries)",
        len(conversion_mapping),
    )
    return True


# ---------------------------------------------------------------------------
# Save mapping JSON
# ---------------------------------------------------------------------------


def save_mapping_json(
    mapping: GeoRoadLaneletMapping,
    xodr_path: Path,
) -> Path:
    """Save mapping to a JSON file next to the XODR file.

    Args:
        mapping: The mapping to save.
        xodr_path: Path to the XODR file (used to derive the JSON path).

    Returns:
        Path to the saved JSON file.
    """
    cache_file = _cache_path_for(xodr_path)
    cache_file.write_text(
        json.dumps(mapping.to_dict(), indent=2) + "\n",
        encoding="utf-8",
    )
    logger.info(
        "Saved mapping JSON to %s (%d entries)",
        cache_file,
        len(mapping.lanelet_to_road_and_lane),
    )
    return cache_file


# ---------------------------------------------------------------------------
# Validate and save (single entry-point for post-conversion)
# ---------------------------------------------------------------------------


def validate_and_save_mapping(
    lanelet_to_road_and_lane: dict[int, tuple[int, int]],
    lanelet_map: "lanelet2.core.LaneletMap",
    roads: list["ConverterRoad"],
    xodr_path: Path,
    osm_path: Path,
    mgrs_offset: tuple[float, float],
    preprocessing_log: dict | None = None,
    stop_line_mapping: dict[int, StopLineMappingEntry] | None = None,
    skipped_stop_lines: dict[int, SkippedStopLineEntry] | None = None,
    traffic_light_config: dict | None = None,
    strict: bool = True,
    report_accuracy: bool = True,
) -> Path:
    """Save mapping JSON and cross-validate against geometric mapping.

    This is the single entry-point called at the end of conversion to:

    1. Compute SHA256 checksums of the XODR and OSM files.
    2. Build a :class:`GeoRoadLaneletMapping` from the conversion-time
       mapping — recording the synthetic divergence/merge connecting roads
       (#493), identified by a lightweight pass independent of the
       geometric mapping — and save it as ``.mapping.json`` next to the
       XODR file.
    3. Build a geometric mapping from the converter's own ``Road`` objects
       via :func:`build_mapping`, and cross-validate the two mappings.

    Args:
        lanelet_to_road_and_lane: Mapping produced during conversion.
        lanelet_map: The Lanelet2 map used for conversion.
        roads: List of converter ``Road`` objects (from ``opendrive.road``).
        xodr_path: Path to the generated XODR file.
        osm_path: Path to the source OSM file.
        mgrs_offset: ``(offset_x, offset_y)`` coordinate offset.
        preprocessing_log: Optional preprocessing log dict to embed in the
            mapping JSON and annotate cross-validation mismatches.
        stop_line_mapping: Optional mapping of linestring ID to stop line
            conversion info (road_id, signal_types).
        skipped_stop_lines: Optional mapping of linestring ID to skip reason.
        traffic_light_config: Optional dict of TrafficLightConfig fields
            (offset_x, offset_y, offset_z, hdg_offset) persisted for the
            ``analyze`` command to reverse the spawn offset.
        strict: When ``True`` a cross-validation mismatch raises
            :class:`MappingMismatchError`.  When ``False`` the mismatch is
            logged as a warning and the already-written XODR and
            ``.mapping.json`` are kept — the conversion itself succeeded, and
            the geometric mapping is only a heuristic cross-check.

    Returns:
        Path to the saved ``.mapping.json`` file.

    Raises:
        MappingMismatchError: When the conversion-time mapping disagrees with
            the geometric mapping and ``strict`` is ``True``.
    """
    # 1. SHA256
    xodr_sha256 = _sha256_of_file(xodr_path)
    osm_sha256 = _sha256_of_file(osm_path)

    # 2. Save mapping JSON. The synthetic divergence/merge connectors
    #    (#493) are identified by a lightweight pass over ``roads``, so the
    #    JSON artifact is still written — and stays available for
    #    post-mortem debugging — even if the geometric mapping below raises.
    conv_mapping = GeoRoadLaneletMapping(
        xodr_sha256=xodr_sha256,
        osm_sha256=osm_sha256,
        lanelet_to_road_and_lane=lanelet_to_road_and_lane,
        preprocessing_log=preprocessing_log,
        stop_line_mapping=stop_line_mapping,
        skipped_stop_lines=skipped_stop_lines,
        skipped_synthetic_roads=sorted(_synthetic_connector_road_ids(roads)) or None,
        traffic_light_config=traffic_light_config,
    )
    json_path = save_mapping_json(conv_mapping, xodr_path)
    logger.info("Mapping JSON saved to %s", json_path)

    # 3. Cross-validate with geometric mapping using converter Roads
    geo_mapping = build_mapping(
        lanelet_map, roads, mgrs_offset, xodr_sha256, osm_sha256
    )
    passed = validate_mapping_consistency(
        lanelet_to_road_and_lane, geo_mapping, preprocessing_log, strict=strict
    )
    if passed:
        logger.info("Cross-validation passed successfully!")

    # 4. Report positional accuracy against the Lanelet2 input. The distances
    #    come from the matching above, which already holds both polylines, so
    #    this costs one extra measurement per matched road and no extra
    #    conversion. Re-save the JSON so the figures outlive the log.
    accuracy = summarise_reference_match_accuracy(geo_mapping)
    if accuracy:
        # The headline is the exceedance counts, not median/p95: those two
        # fall as soon as the map gains well-converted roads, so they are
        # logged after the counts and only as a description of this map.
        logger.info(
            "Geometric accuracy vs Lanelet2 input (%s): %s over %d matched "
            "road(s)%s; worst %.3f m at lanelet %d (matching tolerance %.1f m; "
            "%d rescued under the relaxed %.2f m threshold)",
            accuracy["metric"],
            "; ".join(
                "{} > {:.2f} m: {} ({:.1%}{})".format(
                    e["label"],
                    e["threshold_m"],
                    e["count"],
                    e["fraction"],
                    "" if e["per_km"] is None else f", {e['per_km']:.2f}/km",
                )
                for e in accuracy["exceedances"]
            ),
            accuracy["count"],
            (
                ""
                if accuracy["matched_boundary_length_m"] is None
                else f" spanning {accuracy['matched_boundary_length_m'] / 1000.0:.2f} km"
            ),
            accuracy["max_m"],
            accuracy["worst_lanelet_id"],
            accuracy["match_threshold_m"],
            accuracy["rescued_count"],
            accuracy["rescue_threshold_m"],
        )
        logger.info(
            "  map-dependent (not comparable between maps): median=%.3f m  "
            "p95=%.3f m  median of per-road means=%.3f m",
            accuracy["median_m"],
            accuracy["p95_m"],
            accuracy["median_mean_m"],
        )
        worst = accuracy["worst"]
        if worst:
            # A reported distance can exceed the matching tolerance: rescued
            # roads are accepted up to the relaxed rescue threshold, and the
            # reported metric is not the one every fallback level matched on.
            # Claiming "no match failed" for all of them would be wrong.
            logger.warning(
                "%d road(s) sit >= %.2f m from their source lanelet boundary; "
                "%d of those exceed the %.1f m matching tolerance (matching "
                "used a looser metric, or the relaxed %.2f m rescue "
                "threshold). Worst: %s",
                len(worst),
                accuracy["warn_threshold_m"],
                accuracy["above_match_threshold"],
                accuracy["match_threshold_m"],
                accuracy["rescue_threshold_m"],
                ", ".join(
                    f"lanelet {w['lanelet_id']} at {w['distance_m']:.2f} m"
                    + (" (rescued)" if w["rescued"] else "")
                    for w in worst[:10]
                ),
            )
        conv_mapping.reference_match_distances = geo_mapping.reference_match_distances
        conv_mapping.rescued_reference_lanelets = geo_mapping.rescued_reference_lanelets
        save_mapping_json(conv_mapping, xodr_path)

    return json_path


# ---------------------------------------------------------------------------
# Load or build
# ---------------------------------------------------------------------------


def load_or_build_mapping(
    xodr_path: Path,
    osm_path: Path,
    lanelet_map,
    roads: list["ConverterRoad"],
    mgrs_offset: tuple[float, float],
) -> GeoRoadLaneletMapping:
    """Load cached mapping or build a fresh one.

    The mapping is stored as ``<xodr_stem>.mapping.json`` next to the XODR file.
    """
    tqdm.write("Computing SHA256 checksums for map files...")
    xodr_sha256 = _sha256_of_file(xodr_path)
    osm_sha256 = _sha256_of_file(osm_path)
    cache_file = _cache_path_for(xodr_path)

    # Try loading from cache
    if cache_file.exists():
        try:
            data = json.loads(cache_file.read_text(encoding="utf-8"))
            if (
                data.get("xodr_sha256") == xodr_sha256
                and data.get("osm_sha256") == osm_sha256
            ):
                mapping = GeoRoadLaneletMapping.from_dict(data)
                tqdm.write(
                    f"Loaded cached mapping from {cache_file} "
                    f"({len(mapping.lanelet_to_road_and_lane)} entries)"
                )
                logger.info(
                    "Loaded cached lanelet-to-road mapping from %s (%d entries)",
                    cache_file,
                    len(mapping.lanelet_to_road_and_lane),
                )
                return mapping
            tqdm.write("Cache invalidated (SHA256 mismatch); rebuilding mapping...")
            logger.info("Cache invalidated (SHA256 mismatch); rebuilding mapping")
        except (json.JSONDecodeError, KeyError, TypeError):
            logger.warning(
                "Failed to load cached mapping from %s; rebuilding",
                cache_file,
            )

    # Build fresh mapping
    mapping = build_mapping(lanelet_map, roads, mgrs_offset, xodr_sha256, osm_sha256)

    # Save to cache
    try:
        tqdm.write(f"Saving mapping to {cache_file}...")
        cache_file.write_text(
            json.dumps(mapping.to_dict(), indent=2) + "\n",
            encoding="utf-8",
        )
        tqdm.write(f"Saved mapping ({len(mapping.lanelet_to_road_and_lane)} entries)")
        logger.info("Saved lanelet-to-road mapping to %s", cache_file)
    except OSError:
        logger.warning("Could not write mapping cache to %s", cache_file, exc_info=True)

    return mapping
