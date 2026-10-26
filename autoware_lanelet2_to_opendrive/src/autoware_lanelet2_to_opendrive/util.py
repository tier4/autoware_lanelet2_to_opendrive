"""Utility functions for lanelet2 to OpenDRIVE conversion."""

import logging
from typing import (
    Callable,
    FrozenSet,
    Set,
    List,
    Tuple,
    Union,
    Dict,
    Optional,
    Iterable,
    Literal,
    Hashable,
    Mapping,
)
from enum import Enum
from dataclasses import dataclass
import lanelet2
from lanelet2.routing import RoutingGraph, RoutingCostDistance
from lanelet2.geometry import intersects2d
import numpy as np

from .config import DEFAULT_CONFIG, CoordinateOffset

logger = logging.getLogger(__name__)


def extract_points(
    boundary: lanelet2.core.LineString3d,
    dimensions: Literal[2, 3] = 3,
    offset: CoordinateOffset = CoordinateOffset(),
) -> np.ndarray:
    """Extract points from a Lanelet2 boundary with coordinate offset applied.

    This function extracts coordinates from a Lanelet2 LineString3d
    and applies the given coordinate offset (subtracting offset values).

    Args:
        boundary: Lanelet2 LineString3d to extract points from
        dimensions: Number of dimensions to extract (2 or 3)
        offset: Coordinate offset to subtract from the extracted points.
            Defaults to a zero offset (no-op) for callers that do not need
            offset handling.

    Returns:
        numpy array of shape (N, dimensions) with coordinates,
        with coordinate offset applied

    Raises:
        ValueError: If dimensions is not 2 or 3
    """
    if dimensions == 3:
        points = np.array([[p.x, p.y, p.z] for p in boundary])
        if offset.is_active:
            points[:, 0] -= offset.x
            points[:, 1] -= offset.y
            points[:, 2] -= offset.z
    elif dimensions == 2:
        points = np.array([[p.x, p.y] for p in boundary])
        if offset.is_active:
            points[:, 0] -= offset.x
            points[:, 1] -= offset.y
    else:
        raise ValueError(f"Dimensions must be 2 or 3, got {dimensions}")
    return points


def extract_points_3d(
    boundary: lanelet2.core.LineString3d,
    offset: CoordinateOffset = CoordinateOffset(),
) -> np.ndarray:
    """Extract 3D points from a Lanelet2 boundary with coordinate offset applied.

    This function extracts X, Y, Z coordinates from a Lanelet2 LineString3d
    and applies the given coordinate offset (subtracting offset values).

    Args:
        boundary: Lanelet2 LineString3d to extract points from
        offset: Coordinate offset to subtract from the extracted points.
            Defaults to a zero offset (no-op) for callers that do not need
            offset handling.

    Returns:
        numpy array of shape (N, 3) with [x, y, z] coordinates,
        with coordinate offset applied
    """
    return extract_points(boundary, dimensions=3, offset=offset)


def extract_points_2d(
    boundary: lanelet2.core.LineString3d,
    offset: CoordinateOffset = CoordinateOffset(),
) -> np.ndarray:
    """Extract 2D points from a Lanelet2 boundary with coordinate offset applied.

    This function extracts X, Y coordinates from a Lanelet2 LineString3d
    and applies the given coordinate offset (subtracting offset values).

    Args:
        boundary: Lanelet2 LineString3d to extract points from
        offset: Coordinate offset to subtract from the extracted points.
            Defaults to a zero offset (no-op) for callers that do not need
            offset handling.

    Returns:
        numpy array of shape (N, 2) with [x, y] coordinates,
        with coordinate offset applied
    """
    return extract_points(boundary, dimensions=2, offset=offset)


# Type aliases for common lanelet collection types
LaneletInput = Union[
    Set[lanelet2.core.Lanelet],
    List[lanelet2.core.Lanelet],
    lanelet2.core.LaneletLayer,
    Iterable[lanelet2.core.Lanelet],
]


def to_lanelet_set(lanelets: LaneletInput) -> Set[lanelet2.core.Lanelet]:
    """Convert various lanelet collection types to a set.

    Args:
        lanelets: Collection of lanelets in any supported format

    Returns:
        Set of lanelets

    Raises:
        TypeError: If the input cannot be converted to a set of lanelets
    """
    if isinstance(lanelets, set):
        return lanelets
    try:
        return set(lanelets)
    except TypeError as e:
        raise TypeError(f"Cannot convert {type(lanelets)} to set of lanelets") from e


def to_lanelet_list(lanelets: LaneletInput) -> List[lanelet2.core.Lanelet]:
    """Convert various lanelet collection types to a list.

    Args:
        lanelets: Collection of lanelets in any supported format

    Returns:
        List of lanelets

    Raises:
        TypeError: If the input cannot be converted to a list of lanelets
    """
    if isinstance(lanelets, list):
        return lanelets
    try:
        return list(lanelets)
    except TypeError as e:
        raise TypeError(f"Cannot convert {type(lanelets)} to list of lanelets") from e


def create_routing_graph(lanelet_map: lanelet2.core.LaneletMap) -> RoutingGraph:
    """Create a routing graph with standard traffic rules.

    Args:
        lanelet_map: The lanelet2 map to create a routing graph for

    Returns:
        RoutingGraph configured with Germany location and Vehicle participant
    """
    traffic_rules = lanelet2.traffic_rules.create(
        lanelet2.traffic_rules.Locations.Germany,
        lanelet2.traffic_rules.Participants.Vehicle,
    )
    return RoutingGraph(lanelet_map, traffic_rules, [RoutingCostDistance(0.0)])


@dataclass
class RoadLaneletMapping:
    """
    Mapping between OpenDRIVE Roads and Lanelet2 lanelets.

    This class provides bidirectional mapping to easily convert between
    OpenDRIVE road IDs and Lanelet2 lanelet IDs.

    Attributes:
        road_to_lanelets: Maps OpenDRIVE road ID to list of Lanelet2 lanelet IDs
        lanelet_to_road: Maps Lanelet2 lanelet ID to OpenDRIVE road ID
    """

    road_to_lanelets: Dict[int, List[int]]
    lanelet_to_road: Dict[int, int]

    def get_lanelets_for_road(self, road_id: int) -> List[int]:
        """Get all lanelet IDs that belong to a specific road.

        Args:
            road_id: OpenDRIVE road ID

        Returns:
            List of Lanelet2 lanelet IDs, or empty list if road not found
        """
        return self.road_to_lanelets.get(road_id, [])

    def get_road_for_lanelet(self, lanelet_id: int) -> Optional[int]:
        """Get the road ID that contains a specific lanelet.

        Args:
            lanelet_id: Lanelet2 lanelet ID

        Returns:
            OpenDRIVE road ID, or None if lanelet not found
        """
        return self.lanelet_to_road.get(lanelet_id)


class ConnectionDirection(Enum):
    """Enum for specifying connection direction in lanelet relationships."""

    FOLLOWING = "following"
    PREVIOUS = "previous"


def find_lanelets_without_next(
    lanelet_map: lanelet2.core.LaneletMap,
) -> Set[lanelet2.core.Lanelet]:
    """Find all lanelets that have no following lanelets.

    Args:
        lanelet_map: The lanelet2 map to analyze

    Returns:
        Set of lanelets that have no successors
    """
    _, lanelets_without_next = find_terminal_lanelets(lanelet_map)
    return lanelets_without_next


def find_lanelets_without_previous(
    lanelet_map: lanelet2.core.LaneletMap,
) -> Set[lanelet2.core.Lanelet]:
    """Find all lanelets that have no preceding lanelets.

    Args:
        lanelet_map: The lanelet2 map to analyze

    Returns:
        Set of lanelets that have no predecessors
    """
    lanelets_without_previous, _ = find_terminal_lanelets(lanelet_map)
    return lanelets_without_previous


def find_terminal_lanelets(
    lanelet_map: lanelet2.core.LaneletMap,
) -> tuple[Set[lanelet2.core.Lanelet], Set[lanelet2.core.Lanelet]]:
    """Find all terminal lanelets (start and end points) in the map.

    Args:
        lanelet_map: The lanelet2 map to analyze

    Returns:
        Tuple of (lanelets_without_previous, lanelets_without_next)
    """
    # Create routing graph once for efficiency
    routing_graph = create_routing_graph(lanelet_map)

    lanelets_without_previous = set()
    lanelets_without_next = set()

    for lanelet in lanelet_map.laneletLayer:
        # Check for previous lanelets
        if not routing_graph.previous(lanelet):
            lanelets_without_previous.add(lanelet)

        # Check for following lanelets
        if not routing_graph.following(lanelet):
            lanelets_without_next.add(lanelet)

    return lanelets_without_previous, lanelets_without_next


def find_connecting_lanelet_groups(
    lanelet_map: lanelet2.core.LaneletMap,
    lanelet_group: LaneletInput,
    direction: ConnectionDirection,
    routing_graph: Optional[RoutingGraph] = None,
) -> List[Set[lanelet2.core.Lanelet]]:
    """Find and group the connecting lanelets of a given lanelet group.

    Args:
        lanelet_map: The lanelet2 map to analyze
        lanelet_group: Set of lanelets to find connections for
        direction: ConnectionDirection enum value to specify which connections to find
        routing_graph: Optional pre-built routing graph. If None, creates a new one.

    Returns:
        List of sets, where each set contains lanelets that are adjacent to each other
    """
    # Use provided routing graph or create a new one
    if routing_graph is None:
        routing_graph = create_routing_graph(lanelet_map)

    # Collect all connecting lanelets
    connecting_lanelets = set()
    for lanelet in lanelet_group:
        if direction == ConnectionDirection.FOLLOWING:
            connections = routing_graph.following(lanelet)
        elif direction == ConnectionDirection.PREVIOUS:
            connections = routing_graph.previous(lanelet)
        else:
            # This should never happen with enum, but provides safety
            raise ValueError(f"Invalid direction: {direction}")

        for connected_ll in connections:
            connecting_lanelets.add(connected_ll)

    # No connections at all means there is nothing to group. Returning early is
    # required for correctness as well as speed: find_adjacent_groups treats an
    # empty target set as "group every lanelet in the map", so forwarding an
    # empty set here would report the whole map as connecting groups and make
    # every dead-end lanelet group pay for a full-map traversal.
    if not connecting_lanelets:
        return []

    # Group the connecting lanelets by their adjacency
    groups = find_adjacent_groups(lanelet_map, connecting_lanelets, routing_graph)

    return groups


def find_adjacent_groups(
    lanelet_map: lanelet2.core.LaneletMap,
    target_lanelets: Set[lanelet2.core.Lanelet],
    routing_graph: Optional[RoutingGraph] = None,
) -> List[Set[lanelet2.core.Lanelet]]:
    """Find groups of laterally adjacent lanelets.

    Groups lanelets that are connected to each other by left/right adjacency
    into separate groups. Does not consider longitudinal (following/previous) connections.

    Args:
        lanelet_map: The lanelet2 map to analyze
        target_lanelets: Set of lanelets to group. If empty, groups all lanelets in the map.
                        If not empty, groups only the target lanelets into adjacent groups.
        routing_graph: Optional pre-built routing graph. If None, creates a new one.

    Returns:
        List of sets, where each set contains lanelets that are laterally adjacent to each other
    """
    # If target_lanelets is empty, use all lanelets from the map.
    # The full-map set is materialised only in that branch: on large maps it
    # is tens of thousands of elements, and every caller that passes a
    # non-empty ``target_lanelets`` (junction / divergence grouping, called
    # once per site) never reads it.
    if not target_lanelets:
        lanelets_to_group = set(lanelet_map.laneletLayer)
    else:
        # Use only the target lanelets without adding adjacent ones from other groups
        # The adjacency relationships will be found in the DFS below
        lanelets_to_group = target_lanelets.copy()

    # Use provided routing graph or create a new one
    if routing_graph is None:
        routing_graph = create_routing_graph(lanelet_map)

    groups = []
    visited = set()

    def get_adjacent_lanelets(
        lanelet: lanelet2.core.Lanelet,
    ) -> Set[lanelet2.core.Lanelet]:
        """Get laterally adjacent lanelets (left/right only, not following/previous)."""
        adjacent: Set[lanelet2.core.Lanelet] = set()

        # Add left adjacent lanelets.
        # Check both left() (lane-change allowed) and adjacentLeft() (no lane-change)
        # to capture all physically shared boundaries.
        for get_left in (routing_graph.left, routing_graph.adjacentLeft):
            left_ll = get_left(lanelet)
            if left_ll and left_ll in lanelets_to_group:
                adjacent.add(left_ll)

        # Add right adjacent lanelets.
        # Check both right() (lane-change allowed) and adjacentRight() (no lane-change)
        # to capture all physically shared boundaries.
        for get_right in (routing_graph.right, routing_graph.adjacentRight):
            right_ll = get_right(lanelet)
            if right_ll and right_ll in lanelets_to_group:
                adjacent.add(right_ll)

        return adjacent

    def dfs_group(
        start_lanelet: lanelet2.core.Lanelet, current_group: Set[lanelet2.core.Lanelet]
    ) -> None:
        """Depth-first search to find all connected lanelets."""
        if start_lanelet in visited:
            return

        visited.add(start_lanelet)
        current_group.add(start_lanelet)

        # Recursively add all adjacent lanelets
        for adjacent_ll in get_adjacent_lanelets(start_lanelet):
            if adjacent_ll not in visited:
                dfs_group(adjacent_ll, current_group)

    # Group lanelets using DFS
    for lanelet in lanelets_to_group:
        if lanelet not in visited:
            current_group: Set[lanelet2.core.Lanelet] = set()
            dfs_group(lanelet, current_group)
            if current_group:
                groups.append(current_group)

    return groups


def _split_group_by_signature(
    lanelet_map: lanelet2.core.LaneletMap,
    group: Set[lanelet2.core.Lanelet],
    sigs: Mapping[int, Hashable],
    routing_graph: RoutingGraph,
) -> List[Set[lanelet2.core.Lanelet]]:
    """Cut ``group`` into laterally contiguous runs of equal signature.

    Non-adjacent input (``sort_adjacent_groups`` raises) falls back to one
    lateral component per signature bucket, so every emitted group is
    itself adjacency-connected.
    """
    try:
        sorted_lls = sort_adjacent_groups(lanelet_map, group, routing_graph)
    except ValueError:
        buckets: Dict[Hashable, Set[lanelet2.core.Lanelet]] = {}
        for ll in group:
            buckets.setdefault(sigs[ll.id], set()).add(ll)
        parts: List[Set[lanelet2.core.Lanelet]] = []
        for bucket in buckets.values():
            parts.extend(find_adjacent_groups(lanelet_map, bucket, routing_graph))
        return parts

    parts = []
    run: List[lanelet2.core.Lanelet] = []
    run_sig: Optional[Hashable] = None
    for lanelet in sorted_lls:
        sig = sigs[lanelet.id]
        if run and sig != run_sig:
            parts.append(set(run))
            run = []
        run.append(lanelet)
        run_sig = sig
    if run:
        parts.append(set(run))
    return parts


def split_groups_by_divergent_connections(
    lanelet_map: lanelet2.core.LaneletMap,
    groups: List[Set[lanelet2.core.Lanelet]],
    routing_graph: Optional[RoutingGraph] = None,
) -> List[Set[lanelet2.core.Lanelet]]:
    """Refine lateral lanelet groups so each group's lanelets share the same
    downstream group target.

    The default :func:`find_adjacent_groups` groups lanelets purely by
    lateral adjacency, even when their longitudinal followings belong to
    different downstream groups. That works fine in typical Manhattan-
    style segments, but it produces silent lane-link drops in OpenDRIVE
    when a road's lanes fan out to multiple successor roads (e.g.
    nishishinjuku roads 17, 79, 82 — see
    ``specs/2026-05-22-road-successor-mis-merge-design.md``).

    Two lanelets ``A`` and ``B`` in the same lateral group split into
    different roads when the set of group ids covered by
    ``routing_graph.following(A)`` differs from the same set for ``B``.

    Predecessor-side asymmetry is intentionally not split here: the
    existing divergence/merge synthesis pass (``apply_divergence_synthesis``,
    issue #291) already handles the N->1 merge case by synthesising a
    junction, and splitting on both sides causes a cascade of unnecessary
    splits in routing-graph-only-divergent cases that the synthesis pass
    handles cleanly.

    Splits are computed iteratively: a split changes the lanelet->group
    mapping, which can change neighbouring groups' signatures and cause
    further splits. Each iteration that splits strictly increases the
    number of groups, and the maximum number of groups is the number of
    distinct lanelets in the input, so we cap the loop at
    ``len(all_lanelets) + 1`` — a provable upper bound that guarantees
    convergence without an externally-tunable knob that could silently
    degrade behaviour.

    Each partition within a group is materialised as a contiguous lateral
    run via :func:`sort_adjacent_groups`. Non-contiguous inputs fall back
    to (signature, lateral-connected-component) partitioning so every
    emitted group is itself laterally adjacency-connected — otherwise
    :func:`sort_adjacent_groups`, which is invoked unconditionally on the
    refined groups by ``LaneSection.construct_from_lanelet_groups``,
    would fail later and drop the whole road in
    ``Road.construct_from_lanelet_map``.

    Args:
        lanelet_map: Map containing the lanelets, needed by
            :func:`sort_adjacent_groups`.
        groups: Lateral groups from :func:`find_adjacent_groups`.
        routing_graph: Optional pre-built routing graph; created on demand.

    Returns:
        Refined groups. Length is >= ``len(groups)``; the partition of
        lanelets is preserved.
    """
    if routing_graph is None:
        routing_graph = create_routing_graph(lanelet_map)

    current_groups: List[Set[lanelet2.core.Lanelet]] = [set(g) for g in groups]

    # Provable upper bound: every iteration that splits strictly grows the
    # group count, and the absolute maximum is one group per lanelet.
    total_lanelets = sum(len(g) for g in current_groups)
    iteration_cap = total_lanelets + 1

    for _ in range(iteration_cap):
        ll_to_gid: Dict[int, int] = {}
        for gid, group in enumerate(current_groups):
            for lanelet in group:
                ll_to_gid[lanelet.id] = gid

        def signature(lanelet: lanelet2.core.Lanelet) -> frozenset:
            return frozenset(
                ll_to_gid[s.id]
                for s in routing_graph.following(lanelet)
                if s.id in ll_to_gid
            )

        new_groups: List[Set[lanelet2.core.Lanelet]] = []
        any_split = False
        for group in current_groups:
            sigs = {ll.id: signature(ll) for ll in group}
            distinct = set(sigs.values())
            if len(distinct) <= 1:
                new_groups.append(group)
                continue

            any_split = True
            new_groups.extend(
                _split_group_by_signature(lanelet_map, group, sigs, routing_graph)
            )

        if not any_split:
            return new_groups
        current_groups = new_groups

    # Unreachable under the provable bound above; treat as an internal
    # invariant violation rather than silently returning a non-fixed-point
    # grouping (which would reintroduce the lane-link drop this helper
    # exists to prevent).
    raise RuntimeError(
        "split_groups_by_divergent_connections did not converge within "
        f"{iteration_cap} iterations on {total_lanelets} lanelets; this "
        "indicates a cycle in the lateral grouping refinement, which "
        "should be impossible because each split strictly increases the "
        "group count"
    )


#: Lanelet subtypes that become OpenDRIVE lanes on a regular (non-junction)
#: road.  This is the candidate pool for :meth:`Road.construct_from_lanelet_map`
#: and therefore the set of lanelets that can appear in the conversion-time
#: ``lanelet_id -> (road_id, lane_id)`` mapping.  The geometric cross-validation
#: pass in :mod:`road_lanelet_geo_mapping` shares this list so that both sides
#: search the same population; a subtype present in only one of them produces
#: a spurious mismatch.  See ``Lane.construct_from_lanelet`` for the
#: subtype -> ``LaneType`` mapping.
CONVERTIBLE_LANELET_SUBTYPES: List[str] = [
    "road",
    "highway",
    "walkway",
    "pedestrian_lane",
    "road_shoulder",
    "bicycle_lane",
]

#: Lanelet subtypes that carry vehicle traffic and are therefore vertices of
#: the ``Participants.Vehicle`` routing graph used for lateral grouping.
VEHICLE_LANELET_SUBTYPES: FrozenSet[str] = frozenset({"road", "highway"})

#: Non-vehicle lanelet subtypes that belong to the driving road beside them
#: and become extra OpenDRIVE lanes of that road rather than roads of their
#: own.
#:
#: These subtypes are not vertices of the ``Participants.Vehicle`` routing
#: graph, so ``routing_graph.left``/``right``/``adjacentLeft``/
#: ``adjacentRight`` always return ``None`` for them and
#: :func:`find_adjacent_groups` can never put them in the same lateral group
#: as the road they belong to.  The Lanelet2 map specification requires such
#: a lanelet to *share its boundary linestring* with that road lanelet, so
#: the shared linestring — not the routing graph — is the signal used to
#: group them (see :func:`build_boundary_lateral_adjacency` and
#: :func:`attach_boundary_adjacent_lanelets`).
#:
#: ``walkway`` is deliberately NOT in this list.  Measured on the two
#: reference maps, 100% of ``road_shoulder`` / ``pedestrian_lane`` /
#: ``bicycle_lane`` lanelets share a boundary linestring with a road
#: lanelet, whereas 0% of ``walkway`` lanelets do (nishishinjuku 8/8,
#: odaiba 180/180 detached).  Walkways are genuinely standalone footpaths
#: and keep their existing one-lane-road handling.
BOUNDARY_ATTACHED_LANELET_SUBTYPES: List[str] = [
    "road_shoulder",
    "pedestrian_lane",
    "bicycle_lane",
]

#: Subtypes that may take part in a shared-boundary lateral edge.  Anything
#: outside this set (``walkway``, ``crosswalk``, unknown subtypes) is never
#: attached and never used as an attachment anchor.
_BOUNDARY_ATTACHABLE_SUBTYPES: FrozenSet[str] = VEHICLE_LANELET_SUBTYPES | frozenset(
    BOUNDARY_ATTACHED_LANELET_SUBTYPES
)


def lanelet_subtype(lanelet: lanelet2.core.Lanelet) -> str:
    """Return a lanelet's ``subtype`` attribute, or ``""`` when absent."""
    attributes = lanelet.attributes
    if attributes and "subtype" in attributes:
        return str(attributes["subtype"])
    return ""


def is_boundary_attached_lanelet(lanelet: lanelet2.core.Lanelet) -> bool:
    """Return True when ``lanelet`` is a non-vehicle lane of a driving road.

    See :data:`BOUNDARY_ATTACHED_LANELET_SUBTYPES`.
    """
    return lanelet_subtype(lanelet) in BOUNDARY_ATTACHED_LANELET_SUBTYPES


def build_boundary_lateral_adjacency(
    lanelets: LaneletInput,
) -> Tuple[Dict[int, Set[int]], Dict[int, Set[int]]]:
    """Derive lateral adjacency from *shared boundary linestrings*.

    This is the routing-graph-independent counterpart of the left/right
    lookups in :func:`find_adjacent_groups`.  Two lanelets are laterally
    adjacent when one's ``rightBound`` **is** the other's ``leftBound`` —
    the same linestring, in the same orientation.  That is exactly
    ``lanelet2.geometry.leftOf``, which is used here as the authoritative
    check; a linestring-id index only narrows the candidate pairs so the
    pass stays O(n) on maps with tens of thousands of lanelets.

    Only pairs in which **at least one** side is a
    :data:`BOUNDARY_ATTACHED_LANELET_SUBTYPES` lanelet are reported, and
    both sides must be in :data:`_BOUNDARY_ATTACHABLE_SUBTYPES`.  Road ↔
    road adjacency therefore stays entirely with the routing graph, whose
    lane-change semantics this function does not model; and anti-parallel
    neighbours (two lanelets sharing a linestring in opposite orientation,
    e.g. opposing carriageways that share their centre line) are rejected
    by ``leftOf``, so they are never merged into one road.

    Args:
        lanelets: Collection of lanelets to index.

    Returns:
        ``(left_of, right_of)``.  ``left_of[i]`` is the set of ids of
        lanelets immediately to the LEFT of lanelet ``i``; ``right_of[i]``
        the ids immediately to its RIGHT.  Ids absent from a map have no
        neighbour on that side.
    """
    from lanelet2.geometry import leftOf

    lanelet_list = to_lanelet_list(lanelets)

    # Candidate index: a pair can only be adjacent when one lanelet's
    # leftBound and the other's rightBound are the same linestring id.
    by_left_bound: Dict[int, List[lanelet2.core.Lanelet]] = {}
    by_right_bound: Dict[int, List[lanelet2.core.Lanelet]] = {}
    for lanelet in lanelet_list:
        if lanelet_subtype(lanelet) not in _BOUNDARY_ATTACHABLE_SUBTYPES:
            continue
        by_left_bound.setdefault(lanelet.leftBound.id, []).append(lanelet)
        by_right_bound.setdefault(lanelet.rightBound.id, []).append(lanelet)

    left_of: Dict[int, Set[int]] = {}
    right_of: Dict[int, Set[int]] = {}
    for linestring_id, right_side in by_left_bound.items():
        left_side = by_right_bound.get(linestring_id)
        if not left_side:
            continue
        for right_lanelet in right_side:
            for left_lanelet in left_side:
                if left_lanelet.id == right_lanelet.id:
                    continue
                if not (
                    is_boundary_attached_lanelet(left_lanelet)
                    or is_boundary_attached_lanelet(right_lanelet)
                ):
                    continue
                # Authoritative orientation-aware check: rejects the case
                # where the shared linestring is referenced inverted by one
                # of the two lanelets (anti-parallel neighbours).
                if not leftOf(left_lanelet, right_lanelet):
                    continue
                left_of.setdefault(right_lanelet.id, set()).add(left_lanelet.id)
                right_of.setdefault(left_lanelet.id, set()).add(right_lanelet.id)

    return left_of, right_of


#: Subtypes whose longitudinal succession is derived from shared boundary
#: geometry rather than from the ``Participants.Vehicle`` routing graph.
#:
#: These are exactly the lanelets that carry no vehicle traffic and are
#: therefore not vertices of that graph, so ``routing_graph.previous`` and
#: ``routing_graph.following`` return empty for them and their OpenDRIVE
#: lanes are emitted with no ``<link>`` at all.  Measured on the two
#: reference maps, that was 11 of 11 non-vehicle lanes on nishishinjuku and
#: 4,405 of 4,405 on odaiba.  See
#: :func:`build_boundary_longitudinal_adjacency`.
#:
#: ``walkway`` is included for completeness -- a walkway that did share a
#: boundary with another walkway would be linked -- but neither reference
#: map contains such a pair: all 8 nishishinjuku and all 180 odaiba
#: walkways share no linestring and no boundary endpoint with any lanelet.
_LONGITUDINAL_GEOMETRY_SUBTYPES: FrozenSet[str] = frozenset(
    BOUNDARY_ATTACHED_LANELET_SUBTYPES
) | frozenset({"walkway"})


def build_boundary_longitudinal_adjacency(
    lanelets: LaneletInput,
) -> Tuple[Dict[int, Set[int]], Dict[int, Set[int]]]:
    """Derive longitudinal succession from *shared boundary endpoints*.

    This is the longitudinal counterpart of
    :func:`build_boundary_lateral_adjacency`, and exists for the same
    reason: the non-vehicle subtypes are not vertices of a
    ``Participants.Vehicle`` routing graph, so ``routing_graph.previous`` /
    ``routing_graph.following`` return nothing for them and their lanes are
    emitted with no ``<link>``.  A ``Participants.Pedestrian`` graph is not
    an alternative -- on nishishinjuku it yields no edge for any subtype,
    walkway and crosswalk included.

    Lanelet ``a`` is followed by ``b`` when ``a``'s end cross-section *is*
    ``b``'s start cross-section: same left boundary point and same right
    boundary point.  That is exactly ``lanelet2.geometry.follows``, which is
    used here as the authoritative check; a boundary-point-id index only
    narrows the candidate pairs so the pass stays O(n) on maps with tens of
    thousands of lanelets.

    Both sides must be in :data:`_LONGITUDINAL_GEOMETRY_SUBTYPES`.  Vehicle
    lanelets are therefore never an endpoint of an edge reported here, and
    the routing graph remains the sole source of succession for them --
    which is what keeps this pass from changing any vehicle route.

    Args:
        lanelets: Collection of lanelets to index.

    Returns:
        ``(previous_of, following_of)``.  ``previous_of[i]`` is the set of
        ids of lanelets that end where lanelet ``i`` begins;
        ``following_of[i]`` the ids that begin where ``i`` ends.  Ids absent
        from a map have no neighbour on that side.
    """
    from lanelet2.geometry import follows

    lanelet_list = [
        lanelet
        for lanelet in to_lanelet_list(lanelets)
        if lanelet_subtype(lanelet) in _LONGITUDINAL_GEOMETRY_SUBTYPES
    ]

    # Candidate index: a pair can only be in succession when the preceding
    # lanelet's end cross-section and the following one's start
    # cross-section are built from the same two boundary points.
    def _cross_section(
        lanelet: lanelet2.core.Lanelet, at_end: bool
    ) -> Optional[Tuple[int, int]]:
        left, right = lanelet.leftBound, lanelet.rightBound
        if len(left) == 0 or len(right) == 0:
            return None
        index = -1 if at_end else 0
        return (left[index].id, right[index].id)

    by_start: Dict[Tuple[int, int], List[lanelet2.core.Lanelet]] = {}
    for lanelet in lanelet_list:
        start = _cross_section(lanelet, at_end=False)
        if start is not None:
            by_start.setdefault(start, []).append(lanelet)

    previous_of: Dict[int, Set[int]] = {}
    following_of: Dict[int, Set[int]] = {}
    for lanelet in lanelet_list:
        end = _cross_section(lanelet, at_end=True)
        if end is None:
            continue
        for candidate in by_start.get(end, ()):
            if candidate.id == lanelet.id:
                continue
            # Authoritative orientation-aware check.  A lanelet whose own
            # start and end cross-sections coincide (a degenerate loop)
            # would otherwise report itself as its own neighbour through a
            # third lanelet.
            if not follows(lanelet, candidate):
                continue
            following_of.setdefault(lanelet.id, set()).add(candidate.id)
            previous_of.setdefault(candidate.id, set()).add(lanelet.id)

    return previous_of, following_of


#: ``(previous_of, following_of)`` as returned by
#: :func:`build_boundary_longitudinal_adjacency`.
LongitudinalSuccession = Tuple[Dict[int, Set[int]], Dict[int, Set[int]]]


def resolve_lane_link_neighbours(
    lanelet: lanelet2.core.Lanelet,
    routing_lookup: Callable[[lanelet2.core.Lanelet], Iterable[lanelet2.core.Lanelet]],
    succession: Optional[LongitudinalSuccession],
    direction: ConnectionDirection,
    lanelet_map: lanelet2.core.LaneletMap,
) -> List[lanelet2.core.Lanelet]:
    """Pick the source of a lane's predecessors or successors.

    Vehicle lanelets keep the routing graph untouched.  A lanelet in
    :data:`_LONGITUDINAL_GEOMETRY_SUBTYPES` is not a vertex of a
    ``Participants.Vehicle`` graph, so ``routing_lookup`` returns nothing
    for it and its OpenDRIVE lane was emitted with no ``<link>`` at all;
    for those, the succession derived from shared boundary geometry is used
    instead.

    This function only chooses the neighbour list.  Every consistency check
    the caller applies afterwards -- the road-link guard of issue #202, the
    junction branches, and the target-lane validation -- runs unchanged on
    the result, which is what keeps the emitted links schema-valid.

    Args:
        lanelet: The lanelet whose neighbours are wanted.
        routing_lookup: ``routing_graph.previous`` or ``.following``.
        succession: Precomputed geometric succession, or ``None`` to keep
            the routing graph for every lanelet.
        direction: Which side of ``succession`` to read.
        lanelet_map: Map the neighbour ids are resolved against.

    Returns:
        The neighbouring lanelets, ordered by id when they come from
        ``succession`` so the emitted link is reproducible run to run.
    """
    if succession is None or (
        lanelet_subtype(lanelet) not in _LONGITUDINAL_GEOMETRY_SUBTYPES
    ):
        return list(routing_lookup(lanelet))

    previous_of, following_of = succession
    side = previous_of if direction is ConnectionDirection.PREVIOUS else following_of
    neighbours = []
    for neighbour_id in sorted(side.get(lanelet.id, ())):
        try:
            neighbours.append(lanelet_map.laneletLayer.get(neighbour_id))
        except Exception:
            continue
    return neighbours


@dataclass(frozen=True)
class BoundaryAttachmentResult:
    """Outcome of :func:`attach_boundary_adjacent_lanelets`."""

    #: The input groups, each possibly grown by attached lanelets.
    groups: List[Set[lanelet2.core.Lanelet]]
    #: Attachable lanelets that reached no group and keep their previous
    #: standalone-road handling.
    unattached: Set[lanelet2.core.Lanelet]
    #: Ids of lanelets that bordered more than one group and were resolved
    #: by the deterministic tie-break.
    ambiguous_ids: List[int]


def attach_boundary_adjacent_lanelets(
    groups: List[Set[lanelet2.core.Lanelet]],
    attachable: Set[lanelet2.core.Lanelet],
) -> BoundaryAttachmentResult:
    """Fold non-vehicle lanelets into the lateral group they border.

    ``groups`` are the vehicle-routing lateral groups (already refined by
    :func:`split_groups_by_divergent_connections`).  Every lanelet in
    ``attachable`` that shares a boundary linestring with a member of some
    group — directly, or through a chain of other attachable lanelets, e.g.
    ``road | bicycle_lane | pedestrian_lane`` — joins that group and
    becomes an extra lane of the resulting OpenDRIVE road.

    Deterministic rules, in order:

    1. **A lanelet joins exactly one group; groups are never merged.**
       A lanelet that borders two different groups (e.g. a shoulder
       between two carriageways that the routing graph keeps apart) would
       otherwise fuse them into a single road whose reference line spans
       both.  Instead it joins one of them and is reported as ambiguous.
    2. **Nearest group wins**, measured in shared-boundary hops, so a
       lanelet directly touching a road never attaches through a longer
       chain.
    3. **Ties break on the group's smallest lanelet id**, which is
       independent of set iteration order, so the result is reproducible
       run to run.
    4. **A lanelet that reaches no group stays unattached** and is
       returned to the caller, which keeps its existing behaviour (its own
       single-lane road).  Nothing is dropped.

    Args:
        groups: Lateral groups of vehicle lanelets.  Mutated in place by
            adding attached lanelets; also returned in the result.
        attachable: Candidate lanelets, normally the
            :data:`BOUNDARY_ATTACHED_LANELET_SUBTYPES` lanelets of the map.

    Returns:
        A :class:`BoundaryAttachmentResult`.
    """
    if not attachable:
        return BoundaryAttachmentResult(
            groups=groups, unattached=set(), ambiguous_ids=[]
        )

    grouped_lanelets = [lanelet for group in groups for lanelet in group]
    left_of, right_of = build_boundary_lateral_adjacency(
        grouped_lanelets + list(attachable)
    )
    neighbours: Dict[int, Set[int]] = {}
    for side in (left_of, right_of):
        for lanelet_id, others in side.items():
            neighbours.setdefault(lanelet_id, set()).update(others)

    # Group ordering key: the smallest member id.  Groups are disjoint, so
    # this is unique and independent of iteration order.
    group_key: Dict[int, int] = {}
    assigned_group: Dict[int, int] = {}
    for gid, group in enumerate(groups):
        member_ids = [lanelet.id for lanelet in group]
        group_key[gid] = min(member_ids) if member_ids else gid
        for lanelet_id in member_ids:
            assigned_group[lanelet_id] = gid

    pending: Dict[int, lanelet2.core.Lanelet] = {
        lanelet.id: lanelet
        for lanelet in attachable
        if lanelet.id not in assigned_group
    }
    ambiguous_ids: List[int] = []

    # Breadth-first rounds.  Every decision in a round reads the assignment
    # snapshot from the end of the previous round, so rule 2 (nearest group
    # wins) holds and the outcome does not depend on iteration order.
    while pending:
        round_assignment: Dict[int, int] = {}
        for lanelet_id in sorted(pending):
            candidates = {
                assigned_group[neighbour_id]
                for neighbour_id in neighbours.get(lanelet_id, ())
                if neighbour_id in assigned_group
            }
            if not candidates:
                continue
            if len(candidates) > 1:
                ambiguous_ids.append(lanelet_id)
            round_assignment[lanelet_id] = min(
                candidates, key=lambda gid: group_key[gid]
            )
        if not round_assignment:
            break
        for lanelet_id, gid in round_assignment.items():
            assigned_group[lanelet_id] = gid
            groups[gid].add(pending.pop(lanelet_id))

    return BoundaryAttachmentResult(
        groups=groups,
        unattached=set(pending.values()),
        ambiguous_ids=sorted(ambiguous_ids),
    )


def split_junction_groups_by_connections(
    lanelet_map: lanelet2.core.LaneletMap,
    groups: List[Set[lanelet2.core.Lanelet]],
    routing_graph: Optional[RoutingGraph] = None,
    lanelet_to_road_id: Optional[Dict[int, int]] = None,
) -> List[Set[lanelet2.core.Lanelet]]:
    """Refine one junction's lateral lanelet groups so that the lanelets of
    each group share the same incoming and outgoing roads.

    A connecting road carries one road-level ``<predecessor>`` and one
    ``<successor>``, and its lane links are read against those roads.
    Laterally adjacent connecting lanelets that come from, or lead to,
    different roads therefore have to become separate connecting roads --
    the junction counterpart of :func:`split_groups_by_divergent_connections`.

    A lanelet's signature is the pair (predecessor keys, successor keys).
    A neighbour outside the junction is keyed by the regular road it belongs
    to (``lanelet_to_road_id``), a neighbour inside the junction by the group
    it currently belongs to, so chained connecting roads split only where
    their neighbouring groups differ.  Neighbours missing from
    ``lanelet_to_road_id`` cannot be told apart and share one key, so they
    never cause a split on their own; a lanelet with no neighbour on one side
    has an empty key set there and forms its own group.  Unlike the
    regular-road splitter, the predecessor side is split too: no synthesis
    pass covers connecting roads, and a connecting road's single
    ``<predecessor>`` has to hold for every lane's link.  Splits are iterated
    to a fixed point with the same provable bound as
    :func:`split_groups_by_divergent_connections`.

    Args:
        lanelet_map: Map containing the lanelets.
        groups: Lateral groups of one junction from :func:`find_adjacent_groups`.
        routing_graph: Optional pre-built routing graph; created on demand.
        lanelet_to_road_id: ``lanelet_id -> road_id`` of the regular roads
            built before the junction phase.  Without it only the groups
            inside the junction can split.

    Returns:
        Refined groups. Length is >= ``len(groups)``; the partition of
        lanelets is preserved.
    """
    if routing_graph is None:
        routing_graph = create_routing_graph(lanelet_map)
    road_of = lanelet_to_road_id or {}

    current_groups: List[Set[lanelet2.core.Lanelet]] = [set(g) for g in groups]
    total_lanelets = sum(len(g) for g in current_groups)
    iteration_cap = total_lanelets + 1

    for _ in range(iteration_cap):
        ll_to_gid: Dict[int, int] = {}
        for gid, group in enumerate(current_groups):
            for lanelet in group:
                ll_to_gid[lanelet.id] = gid

        def key(neighbour: lanelet2.core.Lanelet) -> tuple[str, int]:
            if neighbour.id in ll_to_gid:
                return ("group", ll_to_gid[neighbour.id])
            if neighbour.id in road_of:
                return ("road", road_of[neighbour.id])
            return ("outside", -1)

        def signature(lanelet: lanelet2.core.Lanelet) -> tuple[frozenset, frozenset]:
            return (
                frozenset(key(n) for n in routing_graph.previous(lanelet)),
                frozenset(key(n) for n in routing_graph.following(lanelet)),
            )

        new_groups: List[Set[lanelet2.core.Lanelet]] = []
        any_split = False
        for group in current_groups:
            sigs = {ll.id: signature(ll) for ll in group}
            if len(set(sigs.values())) <= 1:
                new_groups.append(group)
                continue
            any_split = True
            new_groups.extend(
                _split_group_by_signature(lanelet_map, group, sigs, routing_graph)
            )

        if not any_split:
            return new_groups
        current_groups = new_groups

    raise RuntimeError(
        "split_junction_groups_by_connections did not converge within "
        f"{iteration_cap} iterations on {total_lanelets} lanelets; each split "
        "strictly increases the group count, so this should be impossible"
    )


#: Lanelet subtypes that become OpenDRIVE lanes on a regular (non-junction)
#: road.  This is the candidate pool for :meth:`Road.construct_from_lanelet_map`
#: and therefore the set of lanelets that can appear in the conversion-time
#: ``lanelet_id -> (road_id, lane_id)`` mapping.  The geometric cross-validation
#: pass in :mod:`road_lanelet_geo_mapping` shares this list so that both sides
#: search the same population; a subtype present in only one of them produces
#: a spurious mismatch.  See ``Lane.construct_from_lanelet`` for the
#: subtype -> ``LaneType`` mapping.
CONVERTIBLE_LANELET_SUBTYPES: List[str] = [
    "road",
    "highway",
    "walkway",
    "pedestrian_lane",
    "road_shoulder",
    "bicycle_lane",
]


def filter_lanelets_by_subtype(
    lanelets: LaneletInput,
    subtypes: List[str],
) -> Set[lanelet2.core.Lanelet]:
    """Filter lanelets by their subtype attribute.

    Args:
        lanelets: Collection of lanelets to filter. Can be a set, list, or LaneletLayer.
        subtypes: List of subtypes to filter by. Lanelets with any of these
                  subtypes will be returned.

    Returns:
        Set of lanelets that match the specified subtype(s).
        If subtypes is empty, returns empty set.
        If a lanelet doesn't have a subtype attribute, it is not included.

    Examples:
        # Filter for road lanelets only
        road_lanelets = filter_lanelets_by_subtype(lanelet_map.laneletLayer, ["road"])

        # Filter for multiple subtypes
        main_lanelets = filter_lanelets_by_subtype(
            lanelet_map.laneletLayer,
            ["road", "highway"]
        )

        # Filter from a specific set of lanelets
        terminal_roads = filter_lanelets_by_subtype(terminal_lanelets, ["road"])
    """
    if not subtypes:
        return set()

    # Convert input to a set for consistent processing
    lanelet_set = to_lanelet_set(lanelets)

    # Prepare the set of subtypes to check
    target_subtypes = set(subtypes)

    # Filter lanelets by subtype
    filtered_lanelets = set()
    for lanelet in lanelet_set:
        # Check if the lanelet has a subtype attribute
        if lanelet.attributes and "subtype" in lanelet.attributes:
            lanelet_subtype = lanelet.attributes["subtype"]
            if lanelet_subtype in target_subtypes:
                filtered_lanelets.add(lanelet)

    return filtered_lanelets


def check_lanelet_groups_intersect(
    group1: LaneletInput,
    group2: LaneletInput,
) -> bool:
    """Check if two lanelet groups have any intersecting lanelets.

    Args:
        group1: First group of lanelets
        group2: Second group of lanelets

    Returns:
        True if any lanelet in group1 intersects with any lanelet in group2
    """
    for lanelet_1 in group1:
        for lanelet_2 in group2:
            if intersects2d(lanelet_1, lanelet_2):
                return True
    return False


def lanelet_bounds_2d(
    lanelet: lanelet2.core.Lanelet,
) -> Optional[Tuple[float, float, float, float]]:
    """Compute the 2D axis-aligned bounding box of a lanelet.

    Raw map coordinates are used (no coordinate offset), so the box lives in
    the same frame as the geometry that ``intersects2d`` operates on.

    The lanelet polygon used by ``intersects2d`` is built from the points of
    the left and right bounds, so every point of that polygon lies inside the
    box returned here. A pair of lanelets whose boxes are disjoint therefore
    cannot intersect.

    Args:
        lanelet: Lanelet to measure

    Returns:
        ``(min_x, min_y, max_x, max_y)``, or None if the lanelet has no
        boundary points at all
    """
    xs: List[float] = []
    ys: List[float] = []
    for boundary in (lanelet.leftBound, lanelet.rightBound):
        for point in boundary:
            xs.append(point.x)
            ys.append(point.y)

    if not xs:
        return None

    return (min(xs), min(ys), max(xs), max(ys))


def _bounds_overlap(
    box_1: Tuple[float, float, float, float],
    box_2: Tuple[float, float, float, float],
) -> bool:
    """Check whether two axis-aligned boxes overlap (touching counts as overlap)."""
    return not (
        box_1[2] < box_2[0]
        or box_2[2] < box_1[0]
        or box_1[3] < box_2[1]
        or box_2[3] < box_1[1]
    )


def build_lanelet_intersection_adjacency(
    lanelets: List[lanelet2.core.Lanelet],
) -> List[Set[int]]:
    """Compute which lanelets in a list intersect each other.

    The naive way to answer this is to call ``intersects2d`` on every pair,
    which is O(N^2) geometric tests. Instead the lanelets are bucketed into a
    uniform grid over their bounding boxes and only pairs that share a cell
    *and* whose boxes actually overlap are tested. Because a lanelet polygon
    is contained in its bounding box, a discarded pair can never intersect,
    so this filter produces no false negatives and the resulting adjacency is
    identical to the exhaustive one.

    The cell size is the largest bounding-box extent in the input, which caps
    each lanelet at 2x2 cells and therefore keeps the grid at O(N) entries.

    Args:
        lanelets: Lanelets to relate, indexed by their position in the list

    Returns:
        List parallel to ``lanelets``; entry ``i`` holds the indices of the
        lanelets that intersect ``lanelets[i]``. The relation is stored
        symmetrically and never contains ``i`` itself.
    """
    count = len(lanelets)
    adjacency: List[Set[int]] = [set() for _ in range(count)]
    if count < 2:
        return adjacency

    boxes = [lanelet_bounds_2d(lanelet) for lanelet in lanelets]

    # Candidate pairs are keyed by (lower index, higher index) so that a pair
    # found through several cells is only tested once.
    candidates: Set[Tuple[int, int]] = set()

    # A lanelet without boundary points has no box to filter on, so it stays a
    # candidate against everything rather than being silently dropped.
    unbounded = [index for index, box in enumerate(boxes) if box is None]
    for index in unbounded:
        for other in range(count):
            if other != index:
                candidates.add((min(index, other), max(index, other)))

    max_extent = 0.0
    for box in boxes:
        if box is not None:
            max_extent = max(max_extent, box[2] - box[0], box[3] - box[1])
    cell_size = max(max_extent, DEFAULT_CONFIG.geometry.spatial_grid_min_cell_size)

    grid: Dict[Tuple[int, int], List[int]] = {}
    for index, box in enumerate(boxes):
        if box is None:
            continue
        for cell_x in range(int(box[0] // cell_size), int(box[2] // cell_size) + 1):
            for cell_y in range(int(box[1] // cell_size), int(box[3] // cell_size) + 1):
                grid.setdefault((cell_x, cell_y), []).append(index)

    for bucket in grid.values():
        for position, index in enumerate(bucket):
            # Buckets are filled in ascending index order, so everything after
            # `position` has a larger index and the pair is already ordered.
            for other in bucket[position + 1 :]:
                if _bounds_overlap(boxes[index], boxes[other]):
                    candidates.add((index, other))

    for index, other in candidates:
        if intersects2d(lanelets[index], lanelets[other]):
            adjacency[index].add(other)
            adjacency[other].add(index)

    return adjacency


def sort_adjacent_groups(
    lanelet_map: lanelet2.core.LaneletMap,
    target_lanelets: LaneletInput,
    routing_graph: Optional[RoutingGraph] = None,
) -> List[lanelet2.core.Lanelet]:
    """Sort lanelets from left to right by following their adjacent relationships.

    Lateral neighbours come from the ``Participants.Vehicle`` routing graph.
    When that graph yields nothing — which it always does for the
    non-vehicle subtypes in :data:`BOUNDARY_ATTACHED_LANELET_SUBTYPES`,
    because they are not vertices of it — the shared-boundary relation from
    :func:`build_boundary_lateral_adjacency` is consulted instead, so a
    group grown by :func:`attach_boundary_adjacent_lanelets` can still be
    ordered.  The fallback index is built only when the set actually
    contains such a lanelet, so pure vehicle groups are untouched.

    Args:
        lanelet_map: The lanelet2 map containing the lanelets
        target_lanelets: Set of lanelets to sort
        routing_graph: Optional pre-built routing graph. If None, creates a new one.

    Returns:
        List of lanelets sorted from left to right

    Raises:
        ValueError: If target_lanelets contains non-adjacent lanelets
    """
    if not target_lanelets:
        return []

    # Convert input to set for consistent processing
    lanelet_set = to_lanelet_set(target_lanelets)

    # Use provided routing graph or create a new one
    if routing_graph is None:
        routing_graph = create_routing_graph(lanelet_map)

    # Shared-boundary fallback, built only when it can possibly matter.
    boundary_left: Dict[int, Set[int]] = {}
    boundary_right: Dict[int, Set[int]] = {}
    lanelets_by_id: Dict[int, lanelet2.core.Lanelet] = {}
    if any(is_boundary_attached_lanelet(lanelet) for lanelet in lanelet_set):
        boundary_left, boundary_right = build_boundary_lateral_adjacency(lanelet_set)
        lanelets_by_id = {lanelet.id: lanelet for lanelet in lanelet_set}

    def _boundary_neighbor(
        side: Dict[int, Set[int]],
        lanelet: lanelet2.core.Lanelet,
        candidates: Set[lanelet2.core.Lanelet],
    ) -> Optional[lanelet2.core.Lanelet]:
        """Return the shared-boundary neighbour of ``lanelet`` on ``side``.

        Ties (a lanelet with several neighbours on one side, e.g. where a
        long shoulder abuts two consecutive road lanelets) resolve to the
        smallest lanelet id so the ordering is reproducible.
        """
        best: Optional[lanelet2.core.Lanelet] = None
        for neighbour_id in sorted(side.get(lanelet.id, ())):
            neighbour = lanelets_by_id.get(neighbour_id)
            if neighbour is not None and neighbour in candidates:
                best = neighbour
                break
        return best

    def _get_left_neighbor(
        lanelet: lanelet2.core.Lanelet,
    ) -> Optional[lanelet2.core.Lanelet]:
        """Return the left neighbor in the target set (lane-change or adjacent)."""
        for get_left in (routing_graph.left, routing_graph.adjacentLeft):
            neighbor = get_left(lanelet)
            if neighbor and neighbor in lanelet_set:
                return neighbor
        return _boundary_neighbor(boundary_left, lanelet, lanelet_set)

    def _get_right_neighbor(
        lanelet: lanelet2.core.Lanelet,
        remaining: Set[lanelet2.core.Lanelet],
    ) -> Optional[lanelet2.core.Lanelet]:
        """Return the right neighbor in the remaining set (lane-change or adjacent)."""
        for get_right in (routing_graph.right, routing_graph.adjacentRight):
            neighbor = get_right(lanelet)
            if neighbor and neighbor in remaining:
                return neighbor
        return _boundary_neighbor(boundary_right, lanelet, remaining)

    # Find the leftmost lanelet by traversing left until no more left neighbors
    def find_leftmost_lanelet(
        start_lanelet: lanelet2.core.Lanelet,
    ) -> lanelet2.core.Lanelet:
        current = start_lanelet
        # ``seen`` guards against a cyclic lateral relation.  The routing
        # graph cannot produce one, but the shared-boundary fallback is a
        # purely geometric relation on arbitrary map data, so a ring of
        # lanelets would otherwise spin here forever.
        seen = {current.id}
        while True:
            left_neighbor = _get_left_neighbor(current)
            if left_neighbor is None or left_neighbor.id in seen:
                break
            seen.add(left_neighbor.id)
            current = left_neighbor
        return current

    # Start with any lanelet from the set
    start_lanelet = next(iter(lanelet_set))
    leftmost = find_leftmost_lanelet(start_lanelet)

    # Build sorted list from left to right
    sorted_lanelets = []
    remaining_lanelets = lanelet_set.copy()
    current = leftmost

    while current and current in remaining_lanelets:
        sorted_lanelets.append(current)
        remaining_lanelets.remove(current)

        # Move to the right neighbor
        right_neighbor = _get_right_neighbor(current, remaining_lanelets)
        if right_neighbor:
            current = right_neighbor
        else:
            break

    # Check if all lanelets were processed (i.e., all are adjacent)
    if remaining_lanelets:
        remaining_ids = [ll.id for ll in remaining_lanelets]
        raise ValueError(
            f"Target lanelets contain non-adjacent lanelets. "
            f"Non-adjacent lanelet IDs: {remaining_ids}"
        )

    return sorted_lanelets


# Mapping from element_type property names to lanelet2 subtype attribute values.
# When lanelet2 loads a regulatory element as a generic RegulatoryElement
# instead of the specific subclass (e.g. TrafficLight), the property-based
# detection (hasattr) fails.  This mapping allows fallback detection via the
# ``subtype`` attribute stored in the OSM data.
_ELEMENT_TYPE_TO_SUBTYPE: Dict[str, str] = {
    "trafficLights": "traffic_light",
    "speedLimits": "speed_limit",
}


class TrafficLightAdapter:
    """Adapter that wraps a generic RegulatoryElement to provide the TrafficLight interface.

    When lanelet2 fails to instantiate a ``TrafficLight`` subclass (returning a
    plain ``RegulatoryElement`` instead), downstream code that accesses
    ``.trafficLights`` and ``.stopLine`` breaks.  This adapter reads the raw
    ``parameters`` dict and exposes the same properties.

    Mapping (standard lanelet2 TrafficLight → parameter keys):
        * ``trafficLights`` → ``parameters["refers"]``
        * ``stopLine``      → ``parameters["ref_line"]`` (first element or None)
    """

    def __init__(self, reg_elem: lanelet2.core.RegulatoryElement) -> None:
        self._reg_elem = reg_elem

    # --- Delegate common attributes to the wrapped element ---
    @property
    def id(self) -> int:
        return self._reg_elem.id

    @property
    def attributes(self):  # noqa: ANN201
        return self._reg_elem.attributes

    @property
    def parameters(self):  # noqa: ANN201
        return self._reg_elem.parameters

    # --- TrafficLight-specific properties ---
    @property
    def trafficLights(self) -> list:  # noqa: N802
        """Return the ``refers`` linestrings (traffic light positions)."""
        params = self._reg_elem.parameters
        if "refers" in params:
            return list(params["refers"])
        return []

    @property
    def stopLine(self):  # noqa: ANN201, N802
        """Return the first ``ref_line`` linestring, or None."""
        params = self._reg_elem.parameters
        if "ref_line" in params:
            ref_lines = list(params["ref_line"])
            return ref_lines[0] if ref_lines else None
        return None


def _matches_element_type(
    reg_elem: lanelet2.core.RegulatoryElement,
    element_type: str,
) -> bool:
    """Check whether *reg_elem* matches *element_type*.

    First tries the native property (e.g. ``reg_elem.trafficLights``).
    If the element is a generic ``RegulatoryElement`` (property missing),
    falls back to checking ``reg_elem.attributes['subtype']``.
    """
    # Fast path: native subclass exposes the property directly.
    if hasattr(reg_elem, element_type) and getattr(reg_elem, element_type):
        return True

    # Slow path: check subtype attribute for generic RegulatoryElement.
    expected_subtype = _ELEMENT_TYPE_TO_SUBTYPE.get(element_type)
    if expected_subtype is None:
        return False

    try:
        return dict(reg_elem.attributes).get("subtype") == expected_subtype
    except Exception:
        return False


def _maybe_wrap(
    reg_elem: lanelet2.core.RegulatoryElement,
    element_type: str,
) -> lanelet2.core.RegulatoryElement:
    """Wrap a generic RegulatoryElement in an adapter if needed.

    If the element is already the correct subclass (e.g. TrafficLight),
    return it unchanged.  Otherwise wrap it so that downstream code can
    access ``.trafficLights`` / ``.stopLine`` transparently.
    """
    if element_type == "trafficLights" and not hasattr(reg_elem, "trafficLights"):
        return TrafficLightAdapter(reg_elem)  # type: ignore[return-value]
    return reg_elem


def filter_regulatory_element_by_type(
    lanelet_map: lanelet2.core.LaneletMap,
    element_type: str,
) -> Dict[int, tuple[lanelet2.core.RegulatoryElement, Set[int]]]:
    """
    Filter regulatory elements of specified type from lanelet map.

    Detection uses the native subclass property when available (e.g.
    ``TrafficLight.trafficLights``).  When lanelet2 loads an element as a
    generic ``RegulatoryElement``, the function falls back to checking the
    ``subtype`` attribute in the OSM data and wraps the element in an
    adapter so that downstream code can use the same API.

    Args:
        lanelet_map: Lanelet2 map containing regulatory elements
        element_type: Type of regulatory element to filter (e.g., "trafficLights", "speedLimits")

    Returns:
        Dictionary mapping regulatory element ID to (regulatory element, set of lanelet IDs)
    """
    element_map: Dict[int, tuple[lanelet2.core.RegulatoryElement, Set[int]]] = {}

    for lanelet in lanelet_map.laneletLayer:
        # Get all regulatory elements of the specified type for this lanelet
        for reg_elem in lanelet.regulatoryElements:
            if _matches_element_type(reg_elem, element_type):
                reg_elem_id = reg_elem.id
                if reg_elem_id not in element_map:
                    wrapped = _maybe_wrap(reg_elem, element_type)
                    element_map[reg_elem_id] = (wrapped, set())
                element_map[reg_elem_id][1].add(lanelet.id)

    return element_map
