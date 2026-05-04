"""OpenDRIVE junction definitions."""

import logging
from collections import defaultdict
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Dict, Iterable, List, Optional, Sequence, Set

import lxml.etree as ET
import lanelet2

from .enums import ContactPoint

if TYPE_CHECKING:
    from .road import Road

log = logging.getLogger(__name__)


@dataclass
class LaneLink:
    """Lane link within a junction connection.

    Defines how a lane from the incoming road connects to a lane
    on the connecting road.
    """

    from_lane: int  # Lane ID on the incoming road
    to_lane: int  # Lane ID on the connecting road

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("laneLink")
        elem.set("from", str(self.from_lane))
        elem.set("to", str(self.to_lane))
        return elem


@dataclass(frozen=True)
class Priority:
    """OpenDRIVE <junction><priority high low/> element.

    `high` and `low` are connecting road IDs inside the junction.
    Frozen so the type is hashable and can be set-deduplicated.
    """

    high: int
    low: int

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("priority")
        elem.set("high", str(self.high))
        elem.set("low", str(self.low))
        return elem


@dataclass(frozen=True)
class _RightOfWayRecord:
    """One right_of_way RE reduced to plain integers.

    Used by the pure helper `_build_priorities_from_records` so the
    algorithm can be unit-tested without constructing native lanelet2 REs.
    """

    re_id: int
    row_lanelet_ids: tuple[int, ...]
    yield_lanelet_ids: tuple[int, ...]


def _build_priorities_from_records(
    records: Iterable[_RightOfWayRecord],
    lanelet_to_road_id: Dict[int, int],
    lanelet_to_junction_id: Dict[int, int],
) -> Dict[int, List["Priority"]]:
    """Expand each record into per-junction Priority pairs.

    See spec section 7 for the algorithm. Pure (no lanelet2 dependency)
    so unit tests can drive each scenario with plain dicts and tuples.
    """
    junction_priorities: Dict[int, Set["Priority"]] = defaultdict(set)
    sources: Dict[int, Dict["Priority", List[int]]] = defaultdict(
        lambda: defaultdict(list)
    )

    for record in records:
        row_roads, row_jid = _resolve_lanelet_ids(
            record.row_lanelet_ids, lanelet_to_road_id, lanelet_to_junction_id
        )
        yield_roads, yield_jid = _resolve_lanelet_ids(
            record.yield_lanelet_ids, lanelet_to_road_id, lanelet_to_junction_id
        )

        if row_jid is None or yield_jid is None:
            log.warning(
                "RE %d: cannot determine owning junction; skipped",
                record.re_id,
            )
            continue
        if row_jid != yield_jid:
            log.warning(
                "RE %d: row/yield lanelets span junctions {%d, %d}; skipped",
                record.re_id,
                row_jid,
                yield_jid,
            )
            continue
        if not row_roads or not yield_roads:
            log.warning(
                "RE %d: row/yield road set empty after resolution; skipped",
                record.re_id,
            )
            continue

        jid = row_jid
        for high in row_roads:
            for low in yield_roads:
                if high == low:
                    log.debug(
                        "RE %d: self-priority on road %d; skipped",
                        record.re_id,
                        high,
                    )
                    continue
                p = Priority(high=high, low=low)
                junction_priorities[jid].add(p)
                sources[jid][p].append(record.re_id)

    _warn_on_conflicts(junction_priorities, sources)

    return {
        jid: sorted(prio_set, key=lambda p: (p.high, p.low))
        for jid, prio_set in junction_priorities.items()
    }


def _resolve_lanelet_ids(
    lanelet_ids: Sequence[int],
    lanelet_to_road_id: Dict[int, int],
    lanelet_to_junction_id: Dict[int, int],
) -> tuple[Set[int], Optional[int]]:
    """Return (set of resolved road IDs, owning junction ID or None).

    Returns junction None when zero lanelets or multiple distinct junctions
    are referenced. Lanelets without a road mapping are silently dropped
    (DEBUG-logged at the call site if needed).
    """
    road_ids: Set[int] = set()
    junction_ids: Set[int] = set()
    for lid in lanelet_ids:
        rid = lanelet_to_road_id.get(lid)
        jid = lanelet_to_junction_id.get(lid)
        if rid is not None:
            road_ids.add(rid)
        if jid is not None:
            junction_ids.add(jid)
    if len(junction_ids) != 1:
        return road_ids, None
    return road_ids, next(iter(junction_ids))


def _warn_on_conflicts(
    junction_priorities: Dict[int, Set["Priority"]],
    sources: Dict[int, Dict["Priority", List[int]]],
) -> None:
    """Log a WARNING for every (high, low) where its reverse also exists."""
    seen: Set[tuple[int, int, int]] = set()  # (jid, min, max)
    for jid, prio_set in junction_priorities.items():
        for p in prio_set:
            rev = Priority(high=p.low, low=p.high)
            if rev not in prio_set:
                continue
            key = (jid, min(p.high, p.low), max(p.high, p.low))
            if key in seen:
                continue
            seen.add(key)
            log.warning(
                "Conflicting priority %d<->%d in junction %d "
                "(REs %s vs %s); both pairs emitted.",
                p.high,
                p.low,
                jid,
                sources[jid][p],
                sources[jid][rev],
            )


def _extract_right_of_way_records(
    lanelet_map: lanelet2.core.LaneletMap,
) -> List[_RightOfWayRecord]:
    """Walk regulatoryElementLayer, filter by subtype, extract id sets.

    Skips REs missing a `right_of_way` or `yield` parameter list (logged
    at DEBUG). Per-RE exceptions are caught and logged at WARNING so a
    single malformed RE does not abort conversion.
    """
    records: List[_RightOfWayRecord] = []
    for re in lanelet_map.regulatoryElementLayer:
        try:
            if dict(re.attributes).get("subtype") != "right_of_way":
                continue
            params = re.parameters
            row_lanelets = list(params.get("right_of_way", []))
            yield_lanelets = list(params.get("yield", []))
            if not row_lanelets or not yield_lanelets:
                log.debug(
                    "RE %d: incomplete right_of_way (row=%d, yield=%d); skipped",
                    re.id,
                    len(row_lanelets),
                    len(yield_lanelets),
                )
                continue
            records.append(
                _RightOfWayRecord(
                    re_id=re.id,
                    row_lanelet_ids=tuple(ll.id for ll in row_lanelets),
                    yield_lanelet_ids=tuple(ll.id for ll in yield_lanelets),
                )
            )
        except Exception:
            log.warning(
                "Failed to parse regulatory element %d; skipped",
                getattr(re, "id", -1),
                exc_info=True,
            )
    return records


@dataclass
class Connection:
    """Connection within a junction.

    Defines a connection between an incoming road and a connecting road
    through the junction.
    """

    id: int  # Unique ID within this junction
    incoming_road: int  # ID of the incoming road
    connecting_road: int  # ID of the connecting road
    contact_point: ContactPoint  # Which end of connecting road is used
    lane_links: List[LaneLink] = field(default_factory=list)

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("connection")
        elem.set("id", str(self.id))
        elem.set("incomingRoad", str(self.incoming_road))
        elem.set("connectingRoad", str(self.connecting_road))
        elem.set("contactPoint", self.contact_point.value)

        for lane_link in self.lane_links:
            elem.append(lane_link.to_xml())

        return elem

    def add_lane_link(self, from_lane: int, to_lane: int) -> None:
        """Add a lane link to this connection.

        Args:
            from_lane: Lane ID on the incoming road
            to_lane: Lane ID on the connecting road
        """
        self.lane_links.append(LaneLink(from_lane=from_lane, to_lane=to_lane))


@dataclass
class Junction:
    """Junction definition.

    A junction represents an intersection or complex road connection
    where multiple roads meet.
    """

    id: int
    name: Optional[str] = None
    connections: List[Connection] = field(default_factory=list)
    priorities: List[Priority] = field(default_factory=list)
    controller_ids: List[int] = field(default_factory=list)

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("junction")
        elem.set("id", str(self.id))

        if self.name:
            elem.set("name", self.name)

        for connection in self.connections:
            elem.append(connection.to_xml())

        # OpenDRIVE 1.4 t_junction XSD requires connection* -> priority*
        # -> controller* ordering.
        for priority in self.priorities:
            elem.append(priority.to_xml())

        for controller_id in self.controller_ids:
            controller_elem = ET.SubElement(elem, "controller")
            controller_elem.set("id", str(controller_id))

        return elem

    def add_connection(
        self,
        connection_id: int,
        incoming_road: int,
        connecting_road: int,
        contact_point: ContactPoint,
    ) -> Connection:
        """Add a connection to this junction.

        Args:
            connection_id: Unique ID for this connection within the junction
            incoming_road: ID of the incoming road
            connecting_road: ID of the connecting road
            contact_point: Which end of connecting road is used

        Returns:
            The created Connection object
        """
        connection = Connection(
            id=connection_id,
            incoming_road=incoming_road,
            connecting_road=connecting_road,
            contact_point=contact_point,
        )
        self.connections.append(connection)
        return connection

    @staticmethod
    def construct_from_lanelet_groups(
        junction_id: int,
        lanelet_group: List[lanelet2.core.Lanelet],
        name: Optional[str] = None,
    ) -> "Junction":
        """Construct a Junction from a group of lanelets.

        This method creates a Junction instance from a single lanelet group
        as returned by find_junction_groups(). The connections are not
        automatically created - they should be added separately using
        add_connection() based on the road network structure.

        Args:
            junction_id: Unique ID for this junction
            lanelet_group: List of lanelets that form this junction
                          (one group from find_junction_groups result)
            name: Optional name for this junction. If not provided,
                  will be generated from the lanelet IDs

        Returns:
            Junction instance with the specified ID and name, but no connections yet

        Example:
            >>> from autoware_lanelet2_to_opendrive.junction import find_junction_groups
            >>> junction_groups = find_junction_groups(junction_lanelets)
            >>> junctions = []
            >>> for i, group in enumerate(junction_groups):
            >>>     junction = Junction.construct_from_lanelet_groups(
            >>>         junction_id=i,
            >>>         lanelet_group=group,
            >>>         name=f"Junction_{i}"
            >>>     )
            >>>     junctions.append(junction)
        """
        # Generate a name if not provided
        if name is None:
            if lanelet_group:
                lanelet_ids = sorted([ll.id for ll in lanelet_group])
                name = f"junction_{lanelet_ids[0]}"
            else:
                name = f"junction_{junction_id}"

        # Create the junction without connections
        junction = Junction(id=junction_id, name=name, connections=[])

        return junction

    @staticmethod
    def construct_from_lanelet_map(
        lanelet_map: lanelet2.core.LaneletMap,
        starting_junction_id: int = 0,
    ) -> List["Junction"]:
        """Construct a list of Junctions from a lanelet2 map.

        This method automatically extracts all junction lanelets from the map,
        groups them into separate junctions based on their spatial relationships,
        and creates Junction instances for each group.

        Args:
            lanelet_map: The lanelet2 map to extract junctions from
            starting_junction_id: The starting ID for junction numbering (default: 0)

        Returns:
            List of Junction instances, one for each detected junction group.
            The connections are not automatically created - they should be added
            separately using add_connection() based on the road network structure.

        Example:
            >>> from autoware_lanelet2_to_opendrive.opendrive.junction import Junction
            >>> junctions = Junction.construct_from_lanelet_map(lanelet_map)
            >>> print(f"Found {len(junctions)} junctions")
            >>> for junction in junctions:
            ...     print(f"Junction {junction.id}: {junction.name}")
        """
        # Import here to avoid circular dependency
        from ..junction import _filter_lanelets_inside_junction, find_junction_groups

        # Get all lanelets from the map
        all_lanelets = list(lanelet_map.laneletLayer)

        # Filter lanelets that are inside junctions (have turn_direction attribute)
        junction_lanelets = _filter_lanelets_inside_junction(all_lanelets)

        # Group junction lanelets into separate junction groups
        junction_groups = find_junction_groups(junction_lanelets)

        # Create Junction instances from each group
        junctions = []
        for i, group in enumerate(junction_groups):
            junction_id = starting_junction_id + i
            junction = Junction.construct_from_lanelet_groups(
                junction_id=junction_id, lanelet_group=group
            )
            junctions.append(junction)

        return junctions

    @staticmethod
    def build_connections_from_roads(
        lanelet_map: lanelet2.core.LaneletMap,
        junction_lanelet_group: List[lanelet2.core.Lanelet],
        junction_id: int,
        lanelet_to_road_id: dict[int, int],
        connecting_road_ids: List[int],
        roads: Optional[List] = None,
    ) -> List[Connection]:
        """Build junction connections from road topology.

        Analyzes the routing graph to find how incoming roads connect to
        connecting roads (roads inside the junction), and creates appropriate
        Connection and LaneLink elements.

        Args:
            lanelet_map: The lanelet2 map for routing graph analysis
            junction_lanelet_group: List of lanelets in this junction
            junction_id: ID of this junction
            lanelet_to_road_id: Mapping from lanelet ID to road ID for ALL lanelets
            connecting_road_ids: List of road IDs that are inside this junction
            roads: Optional list of all Road objects for lane ID lookup.
                   If provided, lane links will be created for all driving lanes.

        Returns:
            List of Connection objects for this junction

        Note:
            This method uses the routing graph to find predecessor lanelets
            for each junction lanelet, determines which roads they belong to,
            and creates connections with appropriate lane links.
        """
        from lanelet2.routing import RoutingGraph, RoutingCostDistance
        import lanelet2

        # Create routing graph
        traffic_rules = lanelet2.traffic_rules.create(
            lanelet2.traffic_rules.Locations.Germany,
            lanelet2.traffic_rules.Participants.Vehicle,
        )
        routing_graph = RoutingGraph(
            lanelet_map, traffic_rules, [RoutingCostDistance(0.0)]
        )

        # Build road_id to Road mapping for lane ID lookup
        road_id_to_road: dict[int, "Road"] = {}
        if roads is not None:
            road_id_to_road = {road.id: road for road in roads}

        # Track connections: (incoming_road_id, connecting_road_id) -> Connection
        connection_map: dict[tuple[int, int], Connection] = {}
        connection_id_counter = 0

        # Track actual lane connections from routing graph
        # Issue #132 fix: Store actual lanelet connections to create correct lane links
        connection_lane_links: dict[tuple[int, int], List[tuple[int, int]]] = {}

        junction_lanelet_ids = {ll.id for ll in junction_lanelet_group}

        # For each lanelet in this junction
        for junction_lanelet in junction_lanelet_group:
            # Get the connecting road ID for this junction lanelet
            if junction_lanelet.id not in lanelet_to_road_id:
                continue

            connecting_road_id = lanelet_to_road_id[junction_lanelet.id]

            # Get the connecting road and its lanelet-to-lane mapping
            connecting_road = road_id_to_road.get(connecting_road_id)
            if connecting_road is None:
                continue

            connecting_lane_mapping = connecting_road.get_lanelet_to_lane_mapping()
            connecting_lane_id = connecting_lane_mapping.get(junction_lanelet.id)

            if connecting_lane_id is None:
                continue

            # Find predecessor lanelets (incoming to junction)
            previous_lanelets = routing_graph.previous(junction_lanelet)

            for prev_lanelet in previous_lanelets:
                # Skip if predecessor is also in the junction
                if prev_lanelet.id in junction_lanelet_ids:
                    continue

                # Get the incoming road ID
                if prev_lanelet.id not in lanelet_to_road_id:
                    continue

                incoming_road_id = lanelet_to_road_id[prev_lanelet.id]

                # Get the incoming road and its lanelet-to-lane mapping
                incoming_road = road_id_to_road.get(incoming_road_id)
                if incoming_road is None:
                    continue

                incoming_lane_mapping = incoming_road.get_lanelet_to_lane_mapping()
                incoming_lane_id = incoming_lane_mapping.get(prev_lanelet.id)

                if incoming_lane_id is None:
                    continue

                # Create or get connection for this (incoming, connecting) pair
                connection_key = (incoming_road_id, connecting_road_id)

                if connection_key not in connection_map:
                    # Create new connection
                    # ContactPoint determination: we connect to the START of the connecting road
                    # This is a simplification - proper implementation would check geometry
                    connection = Connection(
                        id=connection_id_counter,
                        incoming_road=incoming_road_id,
                        connecting_road=connecting_road_id,
                        contact_point=ContactPoint.START,
                        lane_links=[],
                    )
                    connection_map[connection_key] = connection
                    connection_id_counter += 1
                    connection_lane_links[connection_key] = []

                # Store the actual lane connection from routing graph
                lane_link = (incoming_lane_id, connecting_lane_id)
                if lane_link not in connection_lane_links[connection_key]:
                    connection_lane_links[connection_key].append(lane_link)

        # After building all connections, add lane links based on actual routing graph
        for connection_key, connection in connection_map.items():
            incoming_road_id, connecting_road_id = connection_key

            # Issue #132 fix: Use actual lane connections from routing graph
            actual_lane_links = connection_lane_links.get(connection_key, [])

            if actual_lane_links:
                # Add lane links based on actual routing graph connections
                for from_lane, to_lane in actual_lane_links:
                    # Check if this lane link already exists
                    lane_link_exists = any(
                        ll.from_lane == from_lane and ll.to_lane == to_lane
                        for ll in connection.lane_links
                    )
                    if not lane_link_exists:
                        connection.add_lane_link(from_lane=from_lane, to_lane=to_lane)
            else:
                # Fallback: If no actual connections were found, use old logic
                # This preserves backward compatibility for edge cases

                # Get driving lane IDs from both roads
                incoming_lane_ids = Junction._get_driving_lane_ids(
                    road_id_to_road.get(incoming_road_id)
                )
                connecting_lane_ids = Junction._get_driving_lane_ids(
                    road_id_to_road.get(connecting_road_id)
                )

                # Issue #125 fix: Create lane links for ALL incoming lanes
                # If connecting road has fewer lanes, map multiple incoming lanes to same
                # connecting lane
                if incoming_lane_ids and connecting_lane_ids:
                    # Sort lane IDs (negative for right, positive for left)
                    incoming_sorted = sorted(incoming_lane_ids)
                    connecting_sorted = sorted(connecting_lane_ids)

                    # Map each incoming lane to a connecting lane
                    for incoming_lane in incoming_sorted:
                        # Try to find exact match first
                        if incoming_lane in connecting_lane_ids:
                            to_lane = incoming_lane
                        else:
                            # No exact match - map to closest lane in connecting road
                            # For right lanes (negative IDs): more negative = further
                            # from center
                            # For left lanes (positive IDs): more positive = further
                            # from center
                            to_lane = Junction._find_closest_lane(
                                incoming_lane, connecting_sorted
                            )

                        # Check if this lane link already exists
                        lane_link_exists = any(
                            ll.from_lane == incoming_lane and ll.to_lane == to_lane
                            for ll in connection.lane_links
                        )
                        if not lane_link_exists:
                            connection.add_lane_link(
                                from_lane=incoming_lane, to_lane=to_lane
                            )
                else:
                    # If no lane links were created (roads not found or no lanes),
                    # fall back to default -1 to -1 link
                    if not connection.lane_links:
                        connection.add_lane_link(from_lane=-1, to_lane=-1)

        return list(connection_map.values())

    @staticmethod
    def build_priorities_from_regulatory_elements(
        lanelet_map: lanelet2.core.LaneletMap,
        junctions: List["Junction"],
        junction_lanelet_groups: List[List[lanelet2.core.Lanelet]],
        lanelet_to_road_id: Dict[int, int],
    ) -> Dict[int, List[Priority]]:
        """Walk right_of_way REs and emit per-junction <priority> pairs.

        Args:
            lanelet_map: Source lanelet2 map.
            junctions: All junctions (parallel to junction_lanelet_groups).
            junction_lanelet_groups: Lanelet groups for each junction
                (same index = same junction).
            lanelet_to_road_id: Existing lanelet -> road ID mapping
                (covers regular and connecting roads).

        Returns:
            dict[junction_id -> List[Priority]] (sorted, deduplicated).
        """
        if len(junctions) != len(junction_lanelet_groups):
            raise ValueError(
                "junctions and junction_lanelet_groups must have equal length "
                f"(got {len(junctions)} vs {len(junction_lanelet_groups)})"
            )

        lanelet_to_junction_id: Dict[int, int] = {}
        for junction, group in zip(junctions, junction_lanelet_groups):
            for ll in group:
                lanelet_to_junction_id[ll.id] = junction.id

        records = _extract_right_of_way_records(lanelet_map)
        result = _build_priorities_from_records(
            records, lanelet_to_road_id, lanelet_to_junction_id
        )

        if not result:
            log.info("No <priority> emitted (no valid right_of_way REs)")
        return result

    @staticmethod
    def _find_closest_lane(incoming_lane: int, connecting_lanes: list[int]) -> int:
        """Find the closest lane in connecting road for a given incoming lane.

        Args:
            incoming_lane: Lane ID from incoming road
            connecting_lanes: Sorted list of lane IDs in connecting road

        Returns:
            Closest lane ID in connecting road
        """
        if not connecting_lanes:
            return -1  # Fallback to default lane

        # For right lanes (negative IDs), find the closest (least negative if incoming
        # is more negative)
        # For left lanes (positive IDs), find the closest (least positive if incoming
        # is more positive)
        closest = min(connecting_lanes, key=lambda x: abs(x - incoming_lane))
        return closest

    @staticmethod
    def _get_driving_lane_ids(road: Optional["Road"]) -> set[int]:
        """Get all driving lane IDs from a road.

        Args:
            road: The Road object to extract lane IDs from

        Returns:
            Set of driving lane IDs (negative for right lanes, positive for left)
        """
        from .lane import LaneType

        lane_ids: set[int] = set()

        if road is None or road.lanes is None:
            return lane_ids

        for lane_section in road.lanes.lane_sections:
            # Check right lanes (negative IDs)
            for lane_id, lane in lane_section.right_lanes.items():
                if lane.lane_type == LaneType.DRIVING:
                    lane_ids.add(lane_id)
            # Check left lanes (positive IDs)
            for lane_id, lane in lane_section.left_lanes.items():
                if lane.lane_type == LaneType.DRIVING:
                    lane_ids.add(lane_id)

        return lane_ids
