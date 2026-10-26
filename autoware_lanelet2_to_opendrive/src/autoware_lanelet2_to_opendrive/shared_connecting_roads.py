"""Give every junction ``<connection>`` a connecting road of its own.

A junction connector that the source map feeds from *two* external roads --
a merge that happens inside the junction mouth -- is discovered twice by
``Junction.build_connections_from_roads``: ``_enumerate_lane_pairs`` keys
its result on ``(incoming_road, connecting_road)``, so each incoming road
gets its own ``<connection>`` row naming the same ``connectingRoad``.

OpenDRIVE cannot write that on one connecting road. A connecting road
carries a single road-level ``<link><predecessor>``, and
``Road.set_connecting_road_links`` fills it from the first predecessor
lanelet it resolves, so one of the incoming roads is named there and the
others are not. Two ASAM rules break at once:

``junctions.connection.one_connection_element``
    "Each connecting road shall be represented by exactly one element."
    Reported once per reused ``connectingRoad``.

``junctions.connection.start_along_linkage``
    ``contactPoint="start"`` asserts that the connecting road's
    ``<predecessor>`` *is* the connection's ``incomingRoad``. It holds for
    whichever incoming road won the link and fails for the others.

Both are the same defect seen from two sides. Merging the rows into one
``<connection>`` is not available -- ``<connection>`` carries exactly one
``incomingRoad`` and the rows differ precisely in it -- so the extra rows
get a connecting road of their own: a zero-length stub from the incoming
road into the head of the shared connector, the same primitive the
divergence (#291) and direct-junction-completion passes already use.

Chaining the stub into the shared connector rather than copying the
connector keeps the vehicle on the connector's own lanelet-backed lanes.
A copy would carry no entry in the ``lanelet -> (road, lane)`` mapping, so
every consumer that resolves lanes through it -- the mapping JSON, the
CARLA lane correspondence, the semantic-equivalence measurement -- would
see the branch bypass the connector's lanelet instead of traversing it.
The stub is synthetic and unmapped by construction, exactly like the other
zero-length connectors, and contracts away to the transition it stands
for. It is also below ``_SYNTHETIC_CONNECTOR_MAX_LENGTH``, so the
geometric road<->lanelet matching skips it rather than letting it compete
with the connector it feeds.

Runs last, after signals, objects and lane links are all in place, so the
stubs carry geometry and links only.
"""

from __future__ import annotations

import logging
from collections import defaultdict
from typing import Dict, List, Optional, Tuple

from autoware_lanelet2_to_opendrive.divergence import (
    _lane_anchor_xyz,
    _lane_width_at,
    _lane_width_at_end,
    _make_zero_length_connecting_road,
    _taper_stub_width,
)
from autoware_lanelet2_to_opendrive.opendrive.enums import (
    ContactPoint,
    ElementType,
    TrafficRule,
)
from autoware_lanelet2_to_opendrive.opendrive.junction import Connection, Junction
from autoware_lanelet2_to_opendrive.opendrive.junction import (
    LaneLink as JunctionLaneLink,
)
from autoware_lanelet2_to_opendrive.opendrive.road import (
    Road,
    _evaluate_planview_endpoint_with_heading,
)

log = logging.getLogger(__name__)

XYZ = Tuple[float, float, float]


def _linked_road_id(road: Road, contact_point: ContactPoint) -> Optional[int]:
    """Return the road id the junction-side end of ``road`` links to.

    ``contact_point`` is the connection's, i.e. the end of the connecting
    road the incoming road meets: ``START`` means traffic enters at s=0, so
    the incoming road is the connecting road's ``<predecessor>``; ``END``
    means it enters at s=length against the connecting road's direction, so
    it is the ``<successor>``.  ``None`` when that link is unset or names a
    junction rather than a road.
    """
    if road.link is None:
        return None
    element = (
        road.link.predecessor
        if contact_point == ContactPoint.START
        else road.link.successor
    )
    if element is None or element.element_type != ElementType.ROAD:
        return None
    return element.element_id


def _choose_keeper(road: Road, connections: List[Connection]) -> Connection:
    """Pick the connection that keeps the shared connecting road.

    The one whose ``incomingRoad`` the connecting road's own link already
    names, so the existing road-level link -- and the lane links derived
    from it -- stay untouched.  When no connection matches (the link is
    unset, or names a road that is not an incoming road of any of them),
    the lowest ``incomingRoad`` keeps it, which makes the choice
    deterministic across runs.
    """
    for connection in connections:
        linked = _linked_road_id(road, connection.contact_point)
        if linked is not None and linked == connection.incoming_road:
            return connection
    return min(connections, key=lambda c: (c.incoming_road, c.id))


def _connector_anchor(
    connector: Road,
    lane_id: int,
    *,
    at_start: bool,
    traffic_rule: TrafficRule,
) -> Optional[XYZ]:
    """Where the stub meets the shared connector, on its own lane edge."""
    anchor = _lane_anchor_xyz(
        connector, lane_id, at_start=at_start, traffic_rule=traffic_rule
    )
    if anchor is not None:
        return anchor
    return connector.reference_start_xyz if at_start else connector.reference_end_xyz


def _incoming_anchor(
    incoming: Road,
    lane_id: int,
    *,
    traffic_rule: TrafficRule,
) -> Optional[XYZ]:
    """Where the stub leaves the incoming road, on its own lane edge."""
    anchor = _lane_anchor_xyz(
        incoming, lane_id, at_start=False, traffic_rule=traffic_rule
    )
    if anchor is not None:
        return anchor
    return incoming.reference_end_xyz


def split_shared_connecting_roads(
    *,
    junctions: List[Junction],
    all_roads: List[Road],
    starting_road_id: int,
    traffic_rule: TrafficRule,
    min_segment_length: float,
) -> Tuple[List[Road], int]:
    """Give each ``<connection>`` sharing a connecting road one of its own.

    Args:
        junctions: every junction, with its connections already built.
        all_roads: regular + connecting + synthetic roads. Not mutated;
            the stubs are returned for the caller to append.
        starting_road_id: first free road id.
        traffic_rule: RHT / LHT -- sets the stub lane's id.
        min_segment_length: minimum planView length; downstream consumers
            reject zero-length roads.

    Returns:
        ``(new_roads, next_id)`` -- the stubs to append and the first id
        left unallocated.
    """
    road_by_id: Dict[int, Road] = {road.id: road for road in all_roads}
    stub_lane_id = 1 if traffic_rule == TrafficRule.LHT else -1
    new_roads: List[Road] = []
    next_id = starting_road_id

    for junction in junctions:
        grouped: Dict[int, List[Connection]] = defaultdict(list)
        for connection in junction.connections:
            grouped[connection.connecting_road].append(connection)

        for connecting_road_id, connections in sorted(grouped.items()):
            if len(connections) < 2:
                continue
            connector = road_by_id.get(connecting_road_id)
            if connector is None:
                # No <road> to chain into; leave the duplicate rows as they
                # are rather than drop them, so the orphan stays visible to
                # the checker instead of disappearing here.
                log.warning(
                    "Junction %d: connecting road %d is shared by %d "
                    "connections but has no <road> element; not split",
                    junction.id,
                    connecting_road_id,
                    len(connections),
                )
                continue

            keeper = _choose_keeper(connector, connections)
            for connection in connections:
                if connection is keeper:
                    continue
                stub = _make_stub(
                    connection=connection,
                    connector=connector,
                    junction_id=junction.id,
                    road_id=next_id,
                    road_by_id=road_by_id,
                    traffic_rule=traffic_rule,
                    min_segment_length=min_segment_length,
                )
                if stub is None:
                    continue
                new_roads.append(stub)
                road_by_id[next_id] = stub
                log.debug(
                    "Junction %d: connecting road %d is also entered from "
                    "road %d; chained through stub %d",
                    junction.id,
                    connecting_road_id,
                    connection.incoming_road,
                    next_id,
                )
                connection.connecting_road = next_id
                connection.lane_links = [
                    JunctionLaneLink(from_lane=link.from_lane, to_lane=stub_lane_id)
                    for link in connection.lane_links
                ][:1]
                next_id += 1

            # The keeper's own link may still be unset (or name a road that
            # is not an incoming road here); make it agree with the row that
            # kept the connector so `start_along_linkage` holds for it too.
            if _linked_road_id(connector, keeper.contact_point) != keeper.incoming_road:
                if keeper.contact_point == ContactPoint.START:
                    connector.add_predecessor(
                        element_id=keeper.incoming_road,
                        element_type=ElementType.ROAD,
                        contact_point=ContactPoint.END,
                    )
                else:
                    connector.add_successor(
                        element_id=keeper.incoming_road,
                        element_type=ElementType.ROAD,
                        contact_point=ContactPoint.END,
                    )

    if new_roads:
        log.info(
            "shared connecting roads: chained %d stub(s) so every "
            "<connection> names a connecting road of its own",
            len(new_roads),
        )
    return new_roads, next_id


def _make_stub(
    *,
    connection: Connection,
    connector: Road,
    junction_id: int,
    road_id: int,
    road_by_id: Dict[int, Road],
    traffic_rule: TrafficRule,
    min_segment_length: float,
) -> Optional[Road]:
    """Build the zero-length connector from ``connection``'s incoming road.

    Runs from the incoming road's lane edge to the end of the shared
    connector that the connection's ``contactPoint`` names, so each
    ``<connection>`` row that used to share the connector ends up with a
    connecting road of its own and the vehicle still drives onto the
    connector's own lanelet-backed lane.
    """
    incoming = road_by_id.get(connection.incoming_road)
    if incoming is None or not connection.lane_links:
        return None
    lane_link = connection.lane_links[0]

    at_start = connection.contact_point == ContactPoint.START
    incoming_xyz = _incoming_anchor(
        incoming, lane_link.from_lane, traffic_rule=traffic_rule
    )
    connector_xyz = _connector_anchor(
        connector, lane_link.to_lane, at_start=at_start, traffic_rule=traffic_rule
    )
    if incoming_xyz is None or connector_xyz is None:
        log.warning(
            "Junction %d: no endpoint geometry for the stub from road %d "
            "into connecting road %d; connection left sharing it",
            junction_id,
            connection.incoming_road,
            connector.id,
        )
        return None

    # Both neighbour tangents: the stub is a curve meeting each of them.
    start_heading = None
    if incoming.plan_view is not None:
        endpoint = _evaluate_planview_endpoint_with_heading(
            incoming.plan_view, at_start=False
        )
        if endpoint is not None:
            start_heading = endpoint[2]

    end_heading = None
    if connector.plan_view is not None:
        endpoint = _evaluate_planview_endpoint_with_heading(
            connector.plan_view, at_start=at_start
        )
        if endpoint is not None:
            end_heading = endpoint[2]

    incoming_width = _lane_width_at_end(incoming, lane_link.from_lane)
    connector_width = _lane_width_at(connector, lane_link.to_lane, at_start=at_start)

    # The two anchors are typically within a centimetre of each other, at
    # which scale the vector between them is numerical noise rather than a
    # direction. ``_make_zero_length_connecting_road`` owns that decision:
    # below ``min_segment_length`` it ignores the vector and takes
    # ``fallback_heading`` -- the incoming road's own end tangent -- so the
    # stub runs along the lane it continues instead of across it.
    stub = _make_zero_length_connecting_road(
        road_id=road_id,
        junction_id=junction_id,
        incoming_road_id=connection.incoming_road,
        outgoing_road_id=connector.id,
        incoming_contact=ContactPoint.END,
        outgoing_contact=ContactPoint.START if at_start else ContactPoint.END,
        start_xyz=incoming_xyz,
        end_xyz=connector_xyz,
        min_segment_length=min_segment_length,
        traffic_rule=traffic_rule,
        from_lane=lane_link.from_lane,
        to_lane=lane_link.to_lane,
        start_heading=start_heading,
        end_heading=end_heading,
        lane_width=incoming_width if incoming_width is not None else 3.5,
    )
    _taper_stub_width(stub, incoming_width, connector_width)
    return stub
