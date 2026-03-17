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
from typing import Optional

import numpy as np
from tqdm import tqdm

logger = logging.getLogger(__name__)


class MappingMismatchError(Exception):
    """Raised when conversion mapping and geometric mapping disagree."""


#: Mapping cache files are stored next to the source XODR file.

#: Maximum mean distance (m) between a road reference line and a lanelet
#: boundary for them to be considered a match.  This should be larger than
#: the spline fitting error (~1 m) but smaller than a typical lane width
#: (~3 m) so that we never confuse adjacent boundaries.
_MATCH_THRESHOLD: float = 2.0


# ---------------------------------------------------------------------------
# Dataclass
# ---------------------------------------------------------------------------


@dataclass
class GeoRoadLaneletMapping:
    """Mapping from Lanelet2 lanelet IDs to OpenDRIVE (road_id, lane_id)."""

    xodr_sha256: str
    osm_sha256: str
    lanelet_to_road_and_lane: dict[int, tuple[int, int]] = field(default_factory=dict)
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
        return {
            "version": 2,
            "xodr_sha256": self.xodr_sha256,
            "osm_sha256": self.osm_sha256,
            "lanelet_to_road_and_lane": {
                str(k): list(v) for k, v in self.lanelet_to_road_and_lane.items()
            },
        }

    @classmethod
    def from_dict(cls, data: dict) -> "GeoRoadLaneletMapping":
        """Deserialize from a JSON-compatible dictionary."""
        return cls(
            xodr_sha256=data["xodr_sha256"],
            osm_sha256=data["osm_sha256"],
            lanelet_to_road_and_lane={
                int(k): (v[0], v[1])
                for k, v in data["lanelet_to_road_and_lane"].items()
            },
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


# ---------------------------------------------------------------------------
# Lane-ID extraction
# ---------------------------------------------------------------------------


def _get_driving_lane_ids(road) -> list[int]:
    """Return sorted non-zero lane IDs from the first lane section.

    Sorted by ``abs(id)`` so that the innermost lane (closest to the
    reference line) comes first.
    """
    if not road.lane_sections:
        return []
    section = road.lane_sections[0]
    ids = [lane.id for lane in section.lanes if lane.id != 0]
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
# Build mapping
# ---------------------------------------------------------------------------


def build_mapping(
    lanelet_map,
    road_network,
    mgrs_offset: tuple[float, float],
    xodr_sha256: str,
    osm_sha256: str,
) -> GeoRoadLaneletMapping:
    """Build lanelet -> (road_id, lane_id) mapping by boundary shape comparison.

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

    # -- Match roads to lanelets ------------------------------------------

    mapping: dict[int, tuple[int, int]] = {}
    matched_lanelets: set[int] = set()

    roads = road_network.road_ids_to_object
    for road_id, road in tqdm(
        roads.items(), desc="Matching roads to lanelets", unit="road"
    ):
        ref_line: np.ndarray = road.reference_line
        if len(ref_line) < 2:
            continue

        lane_ids = _get_driving_lane_ids(road)
        if not lane_ids:
            continue

        # Determine RHT / LHT from lane IDs
        is_rht = all(lid < 0 for lid in lane_ids)

        # RHT reference line = leftmost lanelet's LEFT boundary
        # LHT reference line = rightmost lanelet's RIGHT boundary
        boundaries = lanelet_left if is_rht else lanelet_right
        bboxes = lanelet_left_bbox if is_rht else lanelet_right_bbox

        ref_bbox = _bbox(ref_line)

        best_lid: Optional[int] = None
        best_dist = float("inf")

        for lid, boundary in boundaries.items():
            if lid in matched_lanelets:
                continue
            if not _bboxes_overlap(ref_bbox, bboxes[lid]):
                continue
            dist = _symmetric_mean_distance(ref_line, boundary)
            if dist < best_dist:
                best_dist = dist
                best_lid = lid

        if best_lid is None or best_dist > _MATCH_THRESHOLD:
            logger.debug(
                "No matching lanelet for road %s (best_dist=%.2f)",
                road_id,
                best_dist,
            )
            continue

        # -- Assign lane IDs by walking adjacency -------------------------

        current = best_lid
        for lane_id in lane_ids:
            mapping[current] = (int(road_id), lane_id)
            matched_lanelets.add(current)

            # Advance: RHT walks right, LHT walks left
            next_ll = _right_neighbor(current) if is_rht else _left_neighbor(current)
            if next_ll is None:
                break
            current = next_ll

    logger.info(
        "Built lanelet-to-road mapping: %d lanelets -> %d unique roads",
        len(mapping),
        len({v[0] for v in mapping.values()}),
    )

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


# ---------------------------------------------------------------------------
# Cross-validation
# ---------------------------------------------------------------------------


def validate_mapping_consistency(
    conversion_mapping: dict[int, tuple[int, int]],
    geo_mapping: GeoRoadLaneletMapping,
) -> None:
    """Validate that conversion-time mapping matches geometric mapping.

    Compares each entry in the conversion-time mapping against the geometric
    mapping.  Raises :class:`MappingMismatchError` if any lanelet ID maps to a
    different ``(road_id, lane_id)`` pair.

    Args:
        conversion_mapping: Mapping produced during conversion
            (lanelet_id -> (road_id, lane_id)).
        geo_mapping: Mapping produced by geometric boundary comparison.

    Raises:
        MappingMismatchError: When at least one entry differs between the two
            mappings.
    """
    mismatches: list[str] = []
    geo = geo_mapping.lanelet_to_road_and_lane

    for lanelet_id, conv_value in conversion_mapping.items():
        geo_value = geo.get(lanelet_id)
        if geo_value is None:
            mismatches.append(
                f"  lanelet {lanelet_id}: conversion={conv_value}, " f"geo=<missing>"
            )
        elif conv_value != geo_value:
            mismatches.append(
                f"  lanelet {lanelet_id}: conversion={conv_value}, " f"geo={geo_value}"
            )

    # Also check for entries only in geo mapping
    for lanelet_id, geo_value in geo.items():
        if lanelet_id not in conversion_mapping:
            mismatches.append(
                f"  lanelet {lanelet_id}: conversion=<missing>, " f"geo={geo_value}"
            )

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
    xodr_path: Path,
    osm_path: Path,
    mgrs_offset: tuple[float, float],
) -> Path:
    """Save mapping JSON and cross-validate against geometric mapping.

    This is the single entry-point called at the end of conversion to:

    1. Compute SHA256 checksums of the XODR and OSM files.
    2. Build a :class:`GeoRoadLaneletMapping` from the conversion-time mapping
       and save it as ``.mapping.json`` next to the XODR file.
    3. Re-parse the XODR with ``pyxodr``, build the geometric mapping via
       :func:`build_mapping`, and cross-validate the two mappings.

    Args:
        lanelet_to_road_and_lane: Mapping produced during conversion.
        lanelet_map: The Lanelet2 map used for conversion.
        xodr_path: Path to the generated XODR file.
        osm_path: Path to the source OSM file.
        mgrs_offset: ``(offset_x, offset_y)`` coordinate offset.

    Returns:
        Path to the saved ``.mapping.json`` file.

    Raises:
        MappingMismatchError: When the conversion-time mapping disagrees with
            the geometric mapping.
    """
    from pyxodr.road_objects.network import RoadNetwork

    # 1. SHA256
    xodr_sha256 = _sha256_of_file(xodr_path)
    osm_sha256 = _sha256_of_file(osm_path)

    # 2. Save mapping JSON
    conv_mapping = GeoRoadLaneletMapping(
        xodr_sha256=xodr_sha256,
        osm_sha256=osm_sha256,
        lanelet_to_road_and_lane=lanelet_to_road_and_lane,
    )
    json_path = save_mapping_json(conv_mapping, xodr_path)
    logger.info("Mapping JSON saved to %s", json_path)

    # 3. Cross-validate with geometric mapping
    road_network = RoadNetwork(str(xodr_path))
    geo_mapping = build_mapping(
        lanelet_map, road_network, mgrs_offset, xodr_sha256, osm_sha256
    )
    validate_mapping_consistency(lanelet_to_road_and_lane, geo_mapping)
    logger.info("Cross-validation passed successfully!")

    return json_path


# ---------------------------------------------------------------------------
# Load or build
# ---------------------------------------------------------------------------


def load_or_build_mapping(
    xodr_path: Path,
    osm_path: Path,
    lanelet_map,
    road_network,
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
    mapping = build_mapping(
        lanelet_map, road_network, mgrs_offset, xodr_sha256, osm_sha256
    )

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
