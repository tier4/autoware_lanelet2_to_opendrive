"""Utility functions for lanelet2 to OpenDRIVE conversion."""

import logging
from typing import Set, List, Union, Dict, Optional, Iterable, Literal
from enum import Enum
from dataclasses import dataclass
import lanelet2
from lanelet2.routing import RoutingGraph, RoutingCostDistance
from lanelet2.geometry import intersects2d
import numpy as np

from .config import COORDINATE_OFFSET

logger = logging.getLogger(__name__)


def extract_points(
    boundary: lanelet2.core.LineString3d,
    dimensions: Literal[2, 3] = 3,
) -> np.ndarray:
    """Extract points from a Lanelet2 boundary with coordinate offset applied.

    This function extracts coordinates from a Lanelet2 LineString3d
    and applies the global coordinate offset (subtracting offset values).

    Args:
        boundary: Lanelet2 LineString3d to extract points from
        dimensions: Number of dimensions to extract (2 or 3)

    Returns:
        numpy array of shape (N, dimensions) with coordinates,
        with coordinate offset applied

    Raises:
        ValueError: If dimensions is not 2 or 3
    """
    if dimensions == 3:
        points = np.array([[p.x, p.y, p.z] for p in boundary])
        if COORDINATE_OFFSET.is_active:
            points[:, 0] -= COORDINATE_OFFSET.x
            points[:, 1] -= COORDINATE_OFFSET.y
            points[:, 2] -= COORDINATE_OFFSET.z
    elif dimensions == 2:
        points = np.array([[p.x, p.y] for p in boundary])
        if COORDINATE_OFFSET.is_active:
            points[:, 0] -= COORDINATE_OFFSET.x
            points[:, 1] -= COORDINATE_OFFSET.y
    else:
        raise ValueError(f"Dimensions must be 2 or 3, got {dimensions}")
    return points


def extract_points_3d(boundary: lanelet2.core.LineString3d) -> np.ndarray:
    """Extract 3D points from a Lanelet2 boundary with coordinate offset applied.

    This function extracts X, Y, Z coordinates from a Lanelet2 LineString3d
    and applies the global coordinate offset (subtracting offset values).

    Args:
        boundary: Lanelet2 LineString3d to extract points from

    Returns:
        numpy array of shape (N, 3) with [x, y, z] coordinates,
        with coordinate offset applied
    """
    return extract_points(boundary, dimensions=3)


def extract_points_2d(boundary: lanelet2.core.LineString3d) -> np.ndarray:
    """Extract 2D points from a Lanelet2 boundary with coordinate offset applied.

    This function extracts X, Y coordinates from a Lanelet2 LineString3d
    and applies the global coordinate offset (subtracting offset values).

    Args:
        boundary: Lanelet2 LineString3d to extract points from

    Returns:
        numpy array of shape (N, 2) with [x, y] coordinates,
        with coordinate offset applied
    """
    return extract_points(boundary, dimensions=2)


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
    all_lanelets_in_map = set(lanelet_map.laneletLayer)

    # If target_lanelets is empty, use all lanelets from the map
    if not target_lanelets:
        lanelets_to_group = all_lanelets_in_map
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


def split_groups_by_divergent_connections(
    lanelet_map: lanelet2.core.LaneletMap,
    groups: List[Set[lanelet2.core.Lanelet]],
    routing_graph: Optional[RoutingGraph] = None,
    max_iterations: int = 20,
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
    further splits. We iterate to a fixed point (or ``max_iterations``,
    which only guards against pathological cycles — every iteration that
    splits strictly increases the number of groups, so termination is
    structurally guaranteed up to ``len(all_lanelets)``).

    Each partition within a group is materialised as a contiguous lateral
    run via :func:`sort_adjacent_groups`. Non-contiguous patterns fall
    back to set-based partitioning (and downstream geometry code already
    tolerates non-adjacent groups via ``sorted_lanelet_ids=None``).

    Args:
        lanelet_map: Map containing the lanelets, needed by
            :func:`sort_adjacent_groups`.
        groups: Lateral groups from :func:`find_adjacent_groups`.
        routing_graph: Optional pre-built routing graph; created on demand.
        max_iterations: Safety bound for the fixed-point loop.

    Returns:
        Refined groups. Length is >= ``len(groups)``; the partition of
        lanelets is preserved.
    """
    if routing_graph is None:
        routing_graph = create_routing_graph(lanelet_map)

    current_groups: List[Set[lanelet2.core.Lanelet]] = [set(g) for g in groups]

    for _ in range(max_iterations):
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
            try:
                sorted_lls = sort_adjacent_groups(lanelet_map, group, routing_graph)
            except ValueError:
                # Non-adjacent input — fall back to set-based partitioning.
                buckets: Dict[frozenset, Set[lanelet2.core.Lanelet]] = {}
                for ll in group:
                    buckets.setdefault(sigs[ll.id], set()).add(ll)
                new_groups.extend(buckets.values())
                continue

            run: List[lanelet2.core.Lanelet] = []
            run_sig: Optional[frozenset] = None
            for lanelet in sorted_lls:
                sig = sigs[lanelet.id]
                if run and sig != run_sig:
                    new_groups.append(set(run))
                    run = []
                run.append(lanelet)
                run_sig = sig
            if run:
                new_groups.append(set(run))

        if not any_split:
            return new_groups
        current_groups = new_groups

    logger.warning(
        "split_groups_by_divergent_connections: hit max_iterations=%d; "
        "returning latest grouping anyway",
        max_iterations,
    )
    return current_groups


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


def sort_adjacent_groups(
    lanelet_map: lanelet2.core.LaneletMap,
    target_lanelets: LaneletInput,
    routing_graph: Optional[RoutingGraph] = None,
) -> List[lanelet2.core.Lanelet]:
    """Sort lanelets from left to right by following their adjacent relationships.

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

    def _get_left_neighbor(
        lanelet: lanelet2.core.Lanelet,
    ) -> Optional[lanelet2.core.Lanelet]:
        """Return the left neighbor in the target set (lane-change or adjacent)."""
        for get_left in (routing_graph.left, routing_graph.adjacentLeft):
            neighbor = get_left(lanelet)
            if neighbor and neighbor in lanelet_set:
                return neighbor
        return None

    def _get_right_neighbor(
        lanelet: lanelet2.core.Lanelet,
        remaining: Set[lanelet2.core.Lanelet],
    ) -> Optional[lanelet2.core.Lanelet]:
        """Return the right neighbor in the remaining set (lane-change or adjacent)."""
        for get_right in (routing_graph.right, routing_graph.adjacentRight):
            neighbor = get_right(lanelet)
            if neighbor and neighbor in remaining:
                return neighbor
        return None

    # Find the leftmost lanelet by traversing left until no more left neighbors
    def find_leftmost_lanelet(
        start_lanelet: lanelet2.core.Lanelet,
    ) -> lanelet2.core.Lanelet:
        current = start_lanelet
        while True:
            left_neighbor = _get_left_neighbor(current)
            if left_neighbor:
                current = left_neighbor
            else:
                break
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
