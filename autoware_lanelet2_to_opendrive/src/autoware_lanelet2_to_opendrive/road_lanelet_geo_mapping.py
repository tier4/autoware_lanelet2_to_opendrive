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
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Optional

import numpy as np
from tqdm import tqdm

from .opendrive.enums import TrafficRule

if TYPE_CHECKING:
    from .opendrive.road import Road as ConverterRoad

logger = logging.getLogger(__name__)


class MappingMismatchError(Exception):
    """Raised when conversion mapping and geometric mapping disagree."""


#: Mapping cache files are stored next to the source XODR file.

#: Maximum mean distance (m) between a road reference line and a lanelet
#: boundary for them to be considered a match.  This must be larger than
#: the spline fitting error (max_avg_error=2.0 m in config) but smaller
#: than a typical lane width (~3.5 m) so that we never confuse adjacent
#: boundaries.
_MATCH_THRESHOLD: float = 3.5

#: Weight for endpoint proximity penalty added to candidate distance.
#: The penalty = (start_dist + end_dist) * weight, where start_dist and
#: end_dist are the minimum distances from the reference line endpoints
#: to either endpoint of the candidate boundary.  This helps disambiguate
#: geometrically similar junction lanelets that connect different roads.
_ENDPOINT_WEIGHT: float = 0.1


# ---------------------------------------------------------------------------
# Stop line mapping dataclasses
# ---------------------------------------------------------------------------


@dataclass
class StopLineMappingEntry:
    """Stop line mapping entry recorded during conversion."""

    road_id: int
    signal_types: list[int]

    def to_dict(self) -> dict:
        return {"road_id": self.road_id, "signal_types": self.signal_types}

    @classmethod
    def from_dict(cls, data: dict) -> "StopLineMappingEntry":
        return cls(road_id=data["road_id"], signal_types=data["signal_types"])


@dataclass
class SkippedStopLineEntry:
    """Skipped stop line entry recorded during conversion."""

    reason: str

    def to_dict(self) -> dict:
        return {"reason": self.reason}

    @classmethod
    def from_dict(cls, data: dict) -> "SkippedStopLineEntry":
        return cls(reason=data["reason"])


# ---------------------------------------------------------------------------
# Internal dataclass for 3-phase matching
# ---------------------------------------------------------------------------


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
        if self.preprocessing_log is not None:
            result["preprocessing_log"] = self.preprocessing_log
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


def _sample_reference_line_from_road(
    road: "ConverterRoad", num_samples_per_segment: int = 10
) -> np.ndarray:
    """Sample 2D reference line points from a converter Road's ParamPoly3 geometries.

    Evaluates the parametric cubic polynomial at evenly-spaced parameter values
    within each geometry segment and transforms local (u, v) to global (x, y).

    Args:
        road: Converter Road object with ``plan_view`` containing geometries.
        num_samples_per_segment: Number of sample points per geometry segment.

    Returns:
        NumPy array of shape ``(N, 2)`` with global (x, y) coordinates.
    """
    if road.plan_view is None or not road.plan_view.geometries:
        return np.empty((0, 2))

    points: list[list[float]] = []
    for geom in road.plan_view.geometries:
        cos_hdg = np.cos(geom.hdg)
        sin_hdg = np.sin(geom.hdg)
        for i in range(num_samples_per_segment):
            p = geom.length * i / num_samples_per_segment
            u = geom.aU + geom.bU * p + geom.cU * p**2 + geom.dU * p**3
            v = geom.aV + geom.bV * p + geom.cV * p**2 + geom.dV * p**3
            x = geom.x + cos_hdg * u - sin_hdg * v
            y = geom.y + sin_hdg * u + cos_hdg * v
            points.append([x, y])

    # Add endpoint of last segment
    if road.plan_view.geometries:
        last = road.plan_view.geometries[-1]
        p = last.length
        cos_hdg = np.cos(last.hdg)
        sin_hdg = np.sin(last.hdg)
        u = last.aU + last.bU * p + last.cU * p**2 + last.dU * p**3
        v = last.aV + last.bV * p + last.cV * p**2 + last.dV * p**3
        x = last.x + cos_hdg * u - sin_hdg * v
        y = last.y + sin_hdg * u + cos_hdg * v
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


def parse_roads_from_xodr(xodr_path: Path) -> list["ConverterRoad"]:
    """Parse XODR XML and construct converter-compatible Road objects.

    Reads ``<road>`` elements from the XODR file, extracting ``<planView>``
    geometries (``<paramPoly3>``) and lane IDs from ``<laneSection>``.
    Returns a list of ``Road`` objects usable by :func:`build_mapping`.

    This utility allows callers that only have an XODR file (not converter
    Road objects) to build the geometric mapping without pyxodr.

    Args:
        xodr_path: Path to the OpenDRIVE file.

    Returns:
        List of converter ``Road`` objects with ``plan_view`` and ``lanes``
        populated from the XODR XML.
    """
    import lxml.etree as ET

    from .opendrive.enums import LaneType
    from .opendrive.geometry import ParamPoly3, PlanView
    from .opendrive.lane import Lane
    from .opendrive.lane_section import LaneSection
    from .opendrive.lane_sections import Lanes
    from .opendrive.reference_line import ReferenceLine
    from .opendrive.road import Road

    tree = ET.parse(str(xodr_path))
    root = tree.getroot()
    roads: list[Road] = []

    for road_elem in root.findall(".//road"):
        road_id = int(road_elem.get("id", "0"))
        junction = int(road_elem.get("junction", "-1"))

        # Parse planView geometries
        geometries = []
        plan_view_elem = road_elem.find("planView")
        if plan_view_elem is not None:
            for geom_elem in plan_view_elem.findall("geometry"):
                pp3_elem = geom_elem.find("paramPoly3")
                if pp3_elem is not None:
                    geometries.append(
                        ParamPoly3(
                            s=float(geom_elem.get("s", "0")),
                            x=float(geom_elem.get("x", "0")),
                            y=float(geom_elem.get("y", "0")),
                            hdg=float(geom_elem.get("hdg", "0")),
                            length=float(geom_elem.get("length", "0")),
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


def _compute_all_candidates(
    roads: list["ConverterRoad"],
    lanelet_left: dict[int, np.ndarray],
    lanelet_right: dict[int, np.ndarray],
    lanelet_left_bbox: dict[int, tuple[float, float, float, float]],
    lanelet_right_bbox: dict[int, tuple[float, float, float, float]],
) -> tuple[list[_RoadCandidates], dict[int, dict]]:
    """Phase 1: compute candidate lanelet lists for every road.

    For each road, applies bbox -> direction -> distance filtering against
    **all** lanelets (no ``matched_lanelets`` exclusion) and returns candidates
    sorted by ascending distance.

    Returns:
        Tuple of (candidate list, diagnostic dict keyed by road_id for
        roads with zero candidates).
    """
    all_rc: list[_RoadCandidates] = []
    no_candidate_diag: dict[int, dict] = {}

    for road in roads:
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
        bboxes = lanelet_left_bbox if is_rht else lanelet_right_bbox
        ref_bbox = _bbox(ref_line)

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
        n_bbox_skip = 0
        n_dir_skip = 0

        # Pre-compute reference line endpoints for endpoint penalty
        ref_start = ref_line[0]
        ref_end = ref_line[-1]

        for require_dir, metric in _FALLBACK_LEVELS:
            candidates.clear()
            raw_dists.clear()
            best_rejected_dist = float("inf")
            best_rejected_lid = None
            n_bbox_skip = 0
            n_dir_skip = 0

            for lid, boundary in boundaries.items():
                if not _bboxes_overlap(ref_bbox, bboxes[lid]):
                    n_bbox_skip += 1
                    continue
                if require_dir and not _same_direction(ref_line, boundary):
                    n_dir_skip += 1
                    continue
                if metric == "symmetric":
                    dist = _symmetric_mean_distance(boundary, ref_line)
                else:
                    dist = _directed_mean_distance(boundary, ref_line)
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

    return all_rc, no_candidate_diag


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

    while True:
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
    swap_found = True
    while swap_found:
        swap_found = False
        items = list(valid.items())
        for i in range(len(items)):
            rc_a, cand_a = items[i]
            _, lid_a = all_rc[rc_a].candidates[cand_a]
            raw_a = all_rc[rc_a].raw_dists.get(lid_a, float("inf"))
            for j in range(i + 1, len(items)):
                rc_b, cand_b = items[j]
                _, lid_b = all_rc[rc_b].candidates[cand_b]
                raw_b = all_rc[rc_b].raw_dists.get(lid_b, float("inf"))

                # Does A have lid_b? Does B have lid_a?
                raw_a_new = all_rc[rc_a].raw_dists.get(lid_b)
                if raw_a_new is None:
                    continue
                raw_b_new = all_rc[rc_b].raw_dists.get(lid_a)
                if raw_b_new is None:
                    continue

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

    return valid


# ---------------------------------------------------------------------------
# Build mapping
# ---------------------------------------------------------------------------


def build_mapping(
    lanelet_map,
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
    3. Compare the road's reference line polyline against all candidate
       lanelet boundaries.  The one with the smallest mean nearest-point
       distance (below ``_MATCH_THRESHOLD``) is the *reference lanelet*.
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

    lanelets = list(lanelet_map.laneletLayer)
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

    # Phase 1: compute candidate lists for every road (no exclusion)
    all_rc, no_candidate_diag = _compute_all_candidates(
        roads,
        lanelet_left,
        lanelet_right,
        lanelet_left_bbox,
        lanelet_right_bbox,
    )

    # Phase 2: resolve conflicts iteratively
    assignment = _resolve_conflicts(all_rc)

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
        for lid, rid, lane_id in walk_result:
            mapping[lid] = (rid, lane_id)
            matched_lanelets.add(lid)

    # -- Rescue pass: attempt to assign dropped roads -----------------------
    # Roads that had candidates in Phase 1 but were dropped in Phase 2
    # (conflict resolution) get a second chance with a relaxed search
    # against currently unmatched lanelets.
    _RESCUE_THRESHOLD: float = _MATCH_THRESHOLD * 1.5
    assigned_rc_indices = set(assignment.keys())
    rescued_road_ids: set[int] = set()
    for rc_idx in range(len(all_rc)):
        if rc_idx in assigned_rc_indices:
            continue
        rc = all_rc[rc_idx]
        boundaries = lanelet_left if rc.is_rht else lanelet_right
        bboxes = lanelet_left_bbox if rc.is_rht else lanelet_right_bbox
        ref_bbox = _bbox(rc.ref_line)
        best_lid: Optional[int] = None
        best_dist = _RESCUE_THRESHOLD
        for lid, boundary in boundaries.items():
            if lid in matched_lanelets:
                continue
            if not _bboxes_overlap(ref_bbox, bboxes[lid]):
                continue
            dist = _symmetric_mean_distance(boundary, rc.ref_line)
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
            for lid, rid, lane_id in walk_result_rescue:
                mapping[lid] = (rid, lane_id)
                matched_lanelets.add(lid)
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

    roads_no_candidates = all_road_ids - rc_road_ids
    roads_dropped_phase2 = rc_road_ids - assigned_road_ids
    roads_not_fully_mapped: list[tuple[int, tuple[int, ...]]] = []
    for rc_idx, cand_idx in assignment.items():
        rc = all_rc[rc_idx]
        mapped_lanes = {v[1] for k, v in mapping.items() if v[0] == rc.road_id}
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
# Cross-validation
# ---------------------------------------------------------------------------


def validate_mapping_consistency(
    conversion_mapping: dict[int, tuple[int, int]],
    geo_mapping: GeoRoadLaneletMapping,
    preprocessing_log: dict | None = None,
) -> None:
    """Validate that conversion-time mapping matches geometric mapping.

    Compares each entry in the conversion-time mapping against the geometric
    mapping.  Raises :class:`MappingMismatchError` if any lanelet ID maps to a
    different ``(road_id, lane_id)`` pair.

    Args:
        conversion_mapping: Mapping produced during conversion
            (lanelet_id -> (road_id, lane_id)).
        geo_mapping: Mapping produced by geometric boundary comparison.
        preprocessing_log: Optional preprocessing log dict (from
            ``PreprocessingLog.to_dict()``) to annotate mismatches with
            preprocessing context.

    Raises:
        MappingMismatchError: When at least one entry differs between the two
            mappings.
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
        raise MappingMismatchError(
            f"Mapping mismatch: {total} entries differ between conversion-time "
            f"and geometric mappings:\n{detail}"
            + (f"\n  ... and {total - 20} more" if total > 20 else "")
        )

    logger.info(
        "Cross-validation passed: conversion and geometric mappings agree "
        "(%d entries)",
        len(conversion_mapping),
    )


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
    lanelet_map,
    roads: list["ConverterRoad"],
    xodr_path: Path,
    osm_path: Path,
    mgrs_offset: tuple[float, float],
    preprocessing_log: dict | None = None,
    stop_line_mapping: dict[int, StopLineMappingEntry] | None = None,
    skipped_stop_lines: dict[int, SkippedStopLineEntry] | None = None,
) -> Path:
    """Save mapping JSON and cross-validate against geometric mapping.

    This is the single entry-point called at the end of conversion to:

    1. Compute SHA256 checksums of the XODR and OSM files.
    2. Build a :class:`GeoRoadLaneletMapping` from the conversion-time mapping
       and save it as ``.mapping.json`` next to the XODR file.
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

    Returns:
        Path to the saved ``.mapping.json`` file.

    Raises:
        MappingMismatchError: When the conversion-time mapping disagrees with
            the geometric mapping.
    """
    # 1. SHA256
    xodr_sha256 = _sha256_of_file(xodr_path)
    osm_sha256 = _sha256_of_file(osm_path)

    # 2. Save mapping JSON
    conv_mapping = GeoRoadLaneletMapping(
        xodr_sha256=xodr_sha256,
        osm_sha256=osm_sha256,
        lanelet_to_road_and_lane=lanelet_to_road_and_lane,
        preprocessing_log=preprocessing_log,
        stop_line_mapping=stop_line_mapping,
        skipped_stop_lines=skipped_stop_lines,
    )
    json_path = save_mapping_json(conv_mapping, xodr_path)
    logger.info("Mapping JSON saved to %s", json_path)

    # 3. Cross-validate with geometric mapping using converter Roads
    geo_mapping = build_mapping(
        lanelet_map, roads, mgrs_offset, xodr_sha256, osm_sha256
    )
    validate_mapping_consistency(
        lanelet_to_road_and_lane, geo_mapping, preprocessing_log
    )
    logger.info("Cross-validation passed successfully!")

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
