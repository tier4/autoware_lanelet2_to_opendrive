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
        row_roads, row_jid, row_unmapped = _resolve_lanelet_ids(
            record.row_lanelet_ids, lanelet_to_road_id, lanelet_to_junction_id
        )
        yield_roads, yield_jid, yield_unmapped = _resolve_lanelet_ids(
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
        if row_unmapped or yield_unmapped:
            # Reject the whole RE rather than emit a partial Cartesian product
            # built from the lanelets that happened to resolve. A partial
            # priority set is harder to detect downstream than a missing one.
            log.warning(
                "RE %d: lanelets without road mapping (row=%s, yield=%s); skipped",
                record.re_id,
                row_unmapped,
                yield_unmapped,
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
) -> tuple[Set[int], Optional[int], List[int]]:
    """Return (resolved road IDs, owning junction ID or None, unmapped lanelets).

    Returns junction None when zero lanelets or multiple distinct junctions
    are referenced. The third return value lists lanelet IDs with no road
    mapping so the caller can reject the RE rather than emit a partial
    Cartesian product.
    """
    road_ids: Set[int] = set()
    junction_ids: Set[int] = set()
    unmapped: List[int] = []
    for lid in lanelet_ids:
        rid = lanelet_to_road_id.get(lid)
        jid = lanelet_to_junction_id.get(lid)
        if rid is None:
            unmapped.append(lid)
        else:
            road_ids.add(rid)
        if jid is not None:
            junction_ids.add(jid)
    if len(junction_ids) != 1:
        return road_ids, None, unmapped
    return road_ids, next(iter(junction_ids)), unmapped


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

    Uses ``RightOfWay.rightOfWayLanelets()`` / ``yieldLanelets()`` to obtain
    strong-ref ``Lanelet`` lists. The generic ``RegulatoryElement.parameters``
    map exposes ``ConstWeakLanelet`` items for which lanelet2's Python
    bindings have no by-value to-Python converter, so iterating it raises
    ``TypeError`` — the typed accessors avoid that path.

    Skips REs missing either side (logged at DEBUG). Map-data exceptions
    on individual REs are caught and logged at WARNING so a single
    malformed RE does not abort conversion; programming errors
    (AttributeError, TypeError) are not caught — they indicate a code-level
    bug that must surface, not be hidden.
    """
    records: List[_RightOfWayRecord] = []
    for re in lanelet_map.regulatoryElementLayer:
        attrs = re.attributes
        if "subtype" not in attrs or attrs["subtype"] != "right_of_way":
            continue
        if not isinstance(re, lanelet2.core.RightOfWay):
            # Subtype says right_of_way but the lanelet2 factory did not
            # construct a typed RightOfWay (e.g. malformed parameters at
            # load time). The generic .parameters map cannot be read for
            # lanelet roles, so there is nothing further we can extract.
            log.warning(
                "RE %d: subtype=right_of_way but not RightOfWay-typed (%s); " "skipped",
                re.id,
                type(re).__name__,
            )
            continue
        try:
            row_lanelets = list(re.rightOfWayLanelets())
            yield_lanelets = list(re.yieldLanelets())
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
        except (RuntimeError, ValueError, KeyError) as exc:
            log.warning(
                "Failed to parse regulatory element %d (%s); skipped",
                getattr(re, "id", -1),
                exc,
            )
    return records


def _enumerate_lane_pairs(
    junction_to_predecessors: Dict[int, List[int]],
    lanelet_to_road_id: Dict[int, int],
    road_id_to_lanelet_to_lane: Dict[int, Dict[int, int]],
) -> Dict[tuple[int, int], List[tuple[int, int]]]:
    """Enumerate lane-id pairs for each (incoming_road, connecting_road) pair.

    Pure helper for #439: drops the 1:1 lane-pair assumption and returns
    one ``(incoming_lane_id, connecting_lane_id)`` entry per direct
    lane-level predecessor edge implied by ``junction_to_predecessors``.

    Every direct longitudinal predecessor is treated as an incoming edge,
    including predecessors that are themselves connecting lanelets of the
    same junction.  A chained connecting road's incoming road is the
    upstream connecting road; dropping those edges previously left such
    roads out of the junction ``<connection>`` table entirely (#492).

    Args:
        junction_to_predecessors: ``junction_lanelet_id -> [predecessor
            lanelet ids]`` from the routing graph (direct longitudinal
            predecessors only, lane changes excluded).
        lanelet_to_road_id: Global ``lanelet_id -> road_id`` mapping.
        road_id_to_lanelet_to_lane: ``road_id -> (lanelet_id -> lane_id)``.

    Returns:
        ``(incoming_road_id, connecting_road_id) -> [(incoming_lane_id,
        connecting_lane_id), ...]``. Pairs are deduplicated and sorted
        by ``(incoming_lane_id, connecting_lane_id)`` so the emitted XML
        is deterministic.
    """
    pairs: Dict[tuple[int, int], Set[tuple[int, int]]] = defaultdict(set)

    for connecting_lanelet_id, predecessor_ids in junction_to_predecessors.items():
        connecting_road_id = lanelet_to_road_id.get(connecting_lanelet_id)
        if connecting_road_id is None:
            continue
        connecting_lane_id = road_id_to_lanelet_to_lane.get(connecting_road_id, {}).get(
            connecting_lanelet_id
        )
        if connecting_lane_id is None:
            continue

        for prev_id in predecessor_ids:
            incoming_road_id = lanelet_to_road_id.get(prev_id)
            if incoming_road_id is None:
                continue
            incoming_lane_id = road_id_to_lanelet_to_lane.get(incoming_road_id, {}).get(
                prev_id
            )
            if incoming_lane_id is None:
                continue

            pairs[(incoming_road_id, connecting_road_id)].add(
                (incoming_lane_id, connecting_lane_id)
            )

    return {key: sorted(value) for key, value in pairs.items()}


def _build_connections_from_lane_pairs(
    lane_pairs_per_connection: Dict[tuple[int, int], List[tuple[int, int]]],
) -> List["Connection"]:
    """Build :class:`Connection` objects from the helper's lane-pair map.

    Pure (no lanelet2 dependency) so the construction loop — connection
    ID assignment, lane-link emission order, and per-connection laneLink
    fan-out — is exercised by the regression suite without needing a
    real lanelet2 routing graph (#439). Connection IDs are assigned in
    sorted ``(incoming_road, connecting_road)`` order so the emitted
    XML is deterministic across runs.
    """
    connections: List[Connection] = []
    for connection_id, connection_key in enumerate(
        sorted(lane_pairs_per_connection.keys())
    ):
        incoming_road_id, connecting_road_id = connection_key
        connection = Connection(
            id=connection_id,
            incoming_road=incoming_road_id,
            connecting_road=connecting_road_id,
            contact_point=ContactPoint.START,
            lane_links=[],
        )
        for from_lane, to_lane in lane_pairs_per_connection[connection_key]:
            connection.add_lane_link(from_lane=from_lane, to_lane=to_lane)
        connections.append(connection)
    return connections


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
    controller_ids: List[int] = field(default_factory=list)
    priorities: List[Priority] = field(default_factory=list)

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
                   Required to emit ``<laneLink>`` elements; when omitted
                   no connections are returned.

        Returns:
            List of Connection objects for this junction. Each Connection
            carries one ``<laneLink>`` per direct lane-level
            predecessor edge implied by the routing graph (#439: N:M
            lane links for multi-lane merges/splits).
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

        # Walk the routing graph once and collect direct longitudinal
        # predecessor lanelet ids per connecting (junction) lanelet.  The
        # tuple-of-int view feeds the pure ``_enumerate_lane_pairs`` helper
        # so the N:M aggregation logic stays unit-testable without lanelet2
        # fixtures (#439 regression coverage).
        junction_to_predecessors: Dict[int, List[int]] = {}
        for junction_lanelet in junction_lanelet_group:
            junction_to_predecessors[junction_lanelet.id] = [
                prev.id for prev in routing_graph.previous(junction_lanelet)
            ]

        # Lane mapping is needed only for roads that actually appear as a
        # connecting road in this junction or as an incoming road feeding
        # one of its lanelets.  ``Road.get_lanelet_to_lane_mapping()``
        # walks every lane section, so for large maps it would otherwise
        # be O(total_roads) per junction even though most roads never
        # participate.  In-junction predecessors are kept here: a chained
        # connecting road's incoming road is itself a connecting road of
        # this junction (#492).
        participating_road_ids: Set[int] = set()
        for connecting_lid, prev_ids in junction_to_predecessors.items():
            connecting_rid = lanelet_to_road_id.get(connecting_lid)
            if connecting_rid is not None:
                participating_road_ids.add(connecting_rid)
            for prev_id in prev_ids:
                incoming_rid = lanelet_to_road_id.get(prev_id)
                if incoming_rid is not None:
                    participating_road_ids.add(incoming_rid)

        road_id_to_lanelet_to_lane: Dict[int, Dict[int, int]] = {
            rid: road_id_to_road[rid].get_lanelet_to_lane_mapping()
            for rid in participating_road_ids
            if rid in road_id_to_road
        }

        # #439: enumerate every (incoming_lane, connecting_lane) pair the
        # lane-level routing graph implies for each (incoming_road,
        # connecting_road) — multi-lane merges/splits no longer collapse to
        # a single laneLink.
        lane_pairs_per_connection = _enumerate_lane_pairs(
            junction_to_predecessors=junction_to_predecessors,
            lanelet_to_road_id=lanelet_to_road_id,
            road_id_to_lanelet_to_lane=road_id_to_lanelet_to_lane,
        )

        # ContactPoint is fixed to START of the connecting road;
        # geometry-aware contact-point selection is tracked separately
        # and out of scope for #439.
        connections = _build_connections_from_lane_pairs(lane_pairs_per_connection)

        # #492: every connecting road must appear as the connectingRoad of
        # a <connection>; one left out is topologically unreachable — no
        # road-level <link> and referenced by no connection.  Warn so a
        # regression, or a genuinely disconnected turn lanelet in the
        # source map, is visible instead of silently emitting orphan
        # junction geometry.
        uncovered = sorted(
            set(connecting_road_ids) - {c.connecting_road for c in connections}
        )
        if uncovered:
            # The connecting lanelet has no routing-graph predecessor, so no
            # incoming road can be resolved.  This is typically a source-map
            # issue — the turn lanelet is not joined to any incoming lane —
            # rather than a converter fault.  Warn rather than fail: the road
            # is still emitted, just unreachable.
            log.warning(
                "Junction %d: %d connecting road(s) have no <connection> "
                "entry; their connecting lanelet has no routing-graph "
                "predecessor (disconnected turn lanelet in the source map): "
                "%s",
                junction_id,
                len(uncovered),
                uncovered,
            )

        return connections

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
