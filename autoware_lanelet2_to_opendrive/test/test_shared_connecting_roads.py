"""A junction connector fed by two external roads gets one road per feeder.

``_enumerate_lane_pairs`` keys its result on ``(incoming_road,
connecting_road)``, so a connector the source map feeds from two external
roads -- a merge inside the junction mouth -- produces two ``<connection>``
rows naming the same ``connectingRoad``. That breaks two ASAM rules at
once, and both are views of the same defect:

  * ``junctions.connection.one_connection_element`` -- "each connecting
    road shall be represented by exactly one element";
  * ``junctions.connection.start_along_linkage`` -- ``contactPoint="start"``
    asserts the connecting road's ``<predecessor>`` *is* the connection's
    ``incomingRoad``, which can hold for only one of the rows because a
    road carries one ``<predecessor>``.

Merging the rows is not available (``<connection>`` has one
``incomingRoad`` and the rows differ in exactly that), so each extra row
gets a connecting road of its own: a zero-length stub chained into the
head of the shared connector.
"""

from typing import List, Optional

from autoware_lanelet2_to_opendrive.opendrive.enums import (
    ContactPoint,
    ElementType,
    LaneType,
    TrafficRule,
)
from autoware_lanelet2_to_opendrive.opendrive.junction import Connection, Junction
from autoware_lanelet2_to_opendrive.opendrive.junction import (
    LaneLink as JunctionLaneLink,
)
from autoware_lanelet2_to_opendrive.opendrive.lane import Lane
from autoware_lanelet2_to_opendrive.opendrive.lane_elements import LaneLink
from autoware_lanelet2_to_opendrive.opendrive.lane_section import LaneSection
from autoware_lanelet2_to_opendrive.opendrive.lane_sections import Lanes
from autoware_lanelet2_to_opendrive.opendrive.road import Road
from autoware_lanelet2_to_opendrive.shared_connecting_roads import (
    split_shared_connecting_roads,
)

# The ids mirror ``junction_312`` of the Nishi-Shinjuku map, where this was
# reported: roads 162 and 204 both feed connector 677, whose own
# ``<predecessor>`` names 204.
MIN_SEGMENT_LENGTH = 0.01


def _one_lane_road(
    road_id: int,
    *,
    junction: int = -1,
    lane_id: int = 1,
    start_xyz=(0.0, 0.0, 0.0),
    end_xyz=(10.0, 0.0, 0.0),
    predecessor: Optional[int] = None,
    successor: Optional[int] = None,
    lane_predecessor: Optional[int] = None,
) -> Road:
    """A one-lane road with the links and endpoints the pass reads."""
    lane = Lane(lane_id=lane_id, lane_type=LaneType.DRIVING)
    if lane_predecessor is not None:
        lane.predecessor = LaneLink(id=lane_predecessor)
    lane_section = LaneSection(s_offset=0.0)
    if lane_id > 0:
        lane_section.left_lanes[lane_id] = lane
    else:
        lane_section.right_lanes[lane_id] = lane

    road = Road(
        id=road_id,
        name=f"Road_{road_id}",
        junction=junction,
        rule=TrafficRule.LHT,
        lanes=Lanes(lane_sections=[lane_section]),
        reference_start_xyz=start_xyz,
        reference_end_xyz=end_xyz,
        length=10.0,
    )
    if predecessor is not None:
        road.add_predecessor(
            element_id=predecessor,
            element_type=ElementType.ROAD,
            contact_point=ContactPoint.END,
        )
    if successor is not None:
        road.add_successor(
            element_id=successor,
            element_type=ElementType.ROAD,
            contact_point=ContactPoint.START,
        )
    return road


def _scenario(*, connector_predecessor: Optional[int] = 204):
    """Connector 677 shared by incoming roads 162 and 204."""
    incoming_162 = _one_lane_road(
        162, start_xyz=(0.0, 0.0, 0.0), end_xyz=(10.0, 0.0, 0.0), successor=677
    )
    incoming_204 = _one_lane_road(
        204, start_xyz=(10.0, -10.0, 0.0), end_xyz=(10.0, 0.0, 0.0), successor=677
    )
    connector = _one_lane_road(
        677,
        junction=1021,
        start_xyz=(10.0, 0.0, 0.0),
        end_xyz=(20.0, 0.0, 0.0),
        predecessor=connector_predecessor,
        successor=12,
        lane_predecessor=1,
    )
    outgoing = _one_lane_road(12, start_xyz=(20.0, 0.0, 0.0), end_xyz=(30.0, 0.0, 0.0))
    return [incoming_162, incoming_204, connector, outgoing], connector


def _connection(
    connection_id: int,
    incoming_road: int,
    connecting_road: int,
    *,
    contact_point: ContactPoint = ContactPoint.START,
    from_lane: int = 1,
    to_lane: int = 1,
) -> Connection:
    return Connection(
        id=connection_id,
        incoming_road=incoming_road,
        connecting_road=connecting_road,
        contact_point=contact_point,
        lane_links=[JunctionLaneLink(from_lane=from_lane, to_lane=to_lane)],
    )


def _split(junction: Junction, roads: List[Road], starting_road_id: int = 800):
    return split_shared_connecting_roads(
        junctions=[junction],
        all_roads=roads,
        starting_road_id=starting_road_id,
        traffic_rule=TrafficRule.LHT,
        min_segment_length=MIN_SEGMENT_LENGTH,
    )


def _connecting_road_ids(junction: Junction) -> List[int]:
    return [c.connecting_road for c in junction.connections]


def test_shared_connector_gets_a_stub_per_extra_incoming_road() -> None:
    roads, connector = _scenario()
    junction = Junction(
        id=1021,
        connections=[
            _connection(0, incoming_road=162, connecting_road=677),
            _connection(1, incoming_road=204, connecting_road=677),
        ],
    )

    new_roads, next_id = _split(junction, roads)

    assert len(new_roads) == 1
    assert next_id == 801
    stub = new_roads[0]
    assert stub.id == 800

    # one_connection_element: no connecting road id is reused any more.
    ids = _connecting_road_ids(junction)
    assert sorted(ids) == [677, 800]

    # start_along_linkage: every contactPoint="start" row names a connecting
    # road whose <predecessor> is that row's incomingRoad.
    by_connecting = {c.connecting_road: c for c in junction.connections}
    for road in (connector, stub):
        connection = by_connecting[road.id]
        assert road.link is not None
        assert road.link.predecessor is not None
        assert road.link.predecessor.element_type == ElementType.ROAD
        assert road.link.predecessor.element_id == connection.incoming_road

    # 204 kept the connector, because the connector's link already named it.
    assert by_connecting[677].incoming_road == 204
    assert by_connecting[800].incoming_road == 162


def test_stub_is_chained_into_the_shared_connector() -> None:
    """Traffic still drives onto the connector's own lanelet-backed lane.

    A copy of the connector would carry no ``lanelet -> (road, lane)``
    mapping entry, so the branch would appear to bypass the connector's
    lanelet entirely.
    """
    roads, connector = _scenario()
    junction = Junction(
        id=1021,
        connections=[
            _connection(0, incoming_road=162, connecting_road=677),
            _connection(1, incoming_road=204, connecting_road=677),
        ],
    )

    new_roads, _ = _split(junction, roads)
    stub = new_roads[0]

    assert stub.junction == junction.id
    assert stub.link.successor is not None
    assert stub.link.successor.element_type == ElementType.ROAD
    assert stub.link.successor.element_id == connector.id
    assert stub.link.successor.contact_point == ContactPoint.START
    # Short enough that the geometric road<->lanelet matching skips it.
    assert stub.length <= 1.0
    # The junction row points at the stub's own lane, not the connector's.
    row = next(c for c in junction.connections if c.connecting_road == stub.id)
    assert [(link.from_lane, link.to_lane) for link in row.lane_links] == [(1, 1)]
    stub_lane = stub.lanes.lane_sections[0].left_lanes[1]
    assert stub_lane.predecessor.id == 1
    assert stub_lane.successor.id == 1


def test_unshared_connecting_road_is_left_alone() -> None:
    roads, connector = _scenario()
    junction = Junction(
        id=1021,
        connections=[_connection(0, incoming_road=204, connecting_road=677)],
    )

    new_roads, next_id = _split(junction, roads)

    assert new_roads == []
    assert next_id == 800
    assert _connecting_road_ids(junction) == [677]
    assert connector.link.predecessor.element_id == 204


def test_keeper_is_deterministic_when_no_row_matches_the_link() -> None:
    """With the link unset, the lowest incomingRoad keeps the connector.

    Its link is then repaired to name that road rather than left dangling.
    """
    roads, connector = _scenario(connector_predecessor=None)
    junction = Junction(
        id=1021,
        connections=[
            _connection(0, incoming_road=204, connecting_road=677),
            _connection(1, incoming_road=162, connecting_road=677),
        ],
    )

    new_roads, _ = _split(junction, roads)

    assert len(new_roads) == 1
    by_connecting = {c.connecting_road: c for c in junction.connections}
    assert by_connecting[677].incoming_road == 162
    assert connector.link.predecessor.element_id == 162
    assert by_connecting[800].incoming_road == 204
    assert new_roads[0].link.predecessor.element_id == 204


def test_end_contact_point_uses_the_successor_link() -> None:
    """``contactPoint="end"`` means the incoming road is the successor."""
    roads, connector = _scenario(connector_predecessor=12)
    connector.add_successor(
        element_id=204,
        element_type=ElementType.ROAD,
        contact_point=ContactPoint.END,
    )
    junction = Junction(
        id=1021,
        connections=[
            _connection(
                0,
                incoming_road=204,
                connecting_road=677,
                contact_point=ContactPoint.END,
            ),
            _connection(
                1,
                incoming_road=162,
                connecting_road=677,
                contact_point=ContactPoint.END,
            ),
        ],
    )

    new_roads, _ = _split(junction, roads)

    assert len(new_roads) == 1
    stub = new_roads[0]
    by_connecting = {c.connecting_road: c for c in junction.connections}
    # The connector's <successor> already named 204, so that row kept it.
    assert by_connecting[677].incoming_road == 204
    assert connector.link.successor.element_id == 204
    assert by_connecting[800].incoming_road == 162
    # The stub runs out of 162 and meets the connector at its END.
    assert stub.link.predecessor.element_id == 162
    assert stub.link.successor.element_id == 677
    assert stub.link.successor.contact_point == ContactPoint.END


def test_three_way_share_yields_two_stubs() -> None:
    """N rows on one connecting road become N distinct connecting roads."""
    roads, connector = _scenario()
    roads.append(
        _one_lane_road(
            311, start_xyz=(10.0, 10.0, 0.0), end_xyz=(10.0, 0.0, 0.0), successor=677
        )
    )
    junction = Junction(
        id=1021,
        connections=[
            _connection(0, incoming_road=162, connecting_road=677),
            _connection(1, incoming_road=204, connecting_road=677),
            _connection(2, incoming_road=311, connecting_road=677),
        ],
    )

    new_roads, next_id = _split(junction, roads)

    assert len(new_roads) == 2
    assert next_id == 802
    assert len(set(_connecting_road_ids(junction))) == 3
    by_id = {r.id: r for r in roads + new_roads}
    for connection in junction.connections:
        road = by_id[connection.connecting_road]
        assert road.link.predecessor.element_id == connection.incoming_road


def _one_connection_element_errors(junction_xml) -> List[str]:
    """Re-implement ``junctions.connection.one_connection_element``.

    ASAM: "Each connecting road shall be represented by exactly one
    element." One issue per connecting road id used by more than one
    ``<connection>``, exactly as the bundle's checker counts it.
    """
    seen: dict = {}
    for connection in junction_xml.findall("connection"):
        seen.setdefault(connection.get("connectingRoad"), []).append(connection)
    return sorted(rid for rid, rows in seen.items() if len(rows) > 1)


def _start_along_linkage_errors(junction_xml, roads_xml) -> List[str]:
    """Re-implement ``junctions.connection.start_along_linkage``.

    ASAM: ``contactPoint="start"`` says the connecting road runs along the
    linkage named in the element, i.e. its ``<predecessor>`` is the
    connection's ``incomingRoad``. One issue per offending ``<connection>``.
    """
    road_by_id = {road.get("id"): road for road in roads_xml}
    offenders = []
    for connection in junction_xml.findall("connection"):
        if connection.get("contactPoint") != "start":
            continue
        road = road_by_id.get(connection.get("connectingRoad"))
        if road is None:
            continue
        link = road.find("link")
        predecessor = None if link is None else link.find("predecessor")
        if predecessor is None or predecessor.get("elementType") != "road":
            continue
        if predecessor.get("elementId") != connection.get("incomingRoad"):
            offenders.append(connection.get("id"))
    return offenders


def test_emitted_xml_satisfies_both_asam_rules() -> None:
    """The two findings, evaluated on the XML the converter actually writes.

    The pre-split assertions pin the defect, so the test would still fail
    if the split silently stopped happening.
    """
    roads, connector = _scenario()
    junction = Junction(
        id=1021,
        connections=[
            _connection(0, incoming_road=162, connecting_road=677),
            _connection(1, incoming_road=204, connecting_road=677),
        ],
    )

    before_roads = [road.to_xml() for road in roads]
    assert _one_connection_element_errors(junction.to_xml()) == ["677"]
    assert _start_along_linkage_errors(junction.to_xml(), before_roads) == ["0"]

    new_roads, _ = _split(junction, roads)

    after_roads = [road.to_xml() for road in roads + new_roads]
    assert _one_connection_element_errors(junction.to_xml()) == []
    assert _start_along_linkage_errors(junction.to_xml(), after_roads) == []
