"""Tests for N:M ``<laneLink>`` enumeration in ``Junction`` (#439).

Drives the pure ``_enumerate_lane_pairs`` and
``_build_connections_from_lane_pairs`` helpers with synthetic inputs so
multi-lane merge/split scenarios — and the Connection construction loop
that turns helper output into XML-ready objects — can be exercised
without constructing real lanelet2 fixtures.
"""

import lxml.etree as ET

from autoware_lanelet2_to_opendrive.opendrive.enums import ContactPoint
from autoware_lanelet2_to_opendrive.opendrive.junction import (
    _build_connections_from_lane_pairs,
    _enumerate_lane_pairs,
)


def _make_inputs(
    *,
    junction_lanelets,
    incoming_lanelets,
    edges,
    incoming_road_id=10,
    connecting_road_id=20,
):
    """Build the three dict inputs ``_enumerate_lane_pairs`` consumes.

    ``junction_lanelets`` and ``incoming_lanelets`` are lists of
    ``(lanelet_id, lane_id)``; ``edges`` is a list of
    ``(connecting_lanelet_id, [predecessor_lanelet_ids])`` tuples.
    """
    junction_to_predecessors = {
        connecting_lid: list(prev_ids) for connecting_lid, prev_ids in edges
    }
    lanelet_to_road_id = {lid: connecting_road_id for lid, _ in junction_lanelets}
    lanelet_to_road_id.update({lid: incoming_road_id for lid, _ in incoming_lanelets})
    road_id_to_lanelet_to_lane = {
        connecting_road_id: {lid: lane for lid, lane in junction_lanelets},
        incoming_road_id: {lid: lane for lid, lane in incoming_lanelets},
    }
    return (
        junction_to_predecessors,
        lanelet_to_road_id,
        road_id_to_lanelet_to_lane,
    )


def test_two_into_two_emits_two_lane_links():
    """2-lane approach feeding 2-lane connecting road -> two laneLinks.

    Direct regression for #439: prior to the refactor, a single Connection
    might collapse to one ``<laneLink>``; the helper now returns one entry
    per lane-level edge.
    """
    inputs = _make_inputs(
        junction_lanelets=[(101, -2), (102, -1)],
        incoming_lanelets=[(201, -2), (202, -1)],
        edges=[(101, [201]), (102, [202])],
    )

    pairs = _enumerate_lane_pairs(*inputs)

    assert pairs == {(10, 20): [(-2, -2), (-1, -1)]}


def test_two_into_one_merge_emits_two_lane_links():
    """Two incoming lanes merging into a single connecting lane."""
    inputs = _make_inputs(
        junction_lanelets=[(101, -1)],
        incoming_lanelets=[(201, -2), (202, -1)],
        edges=[(101, [201, 202])],
    )

    pairs = _enumerate_lane_pairs(*inputs)

    assert pairs == {(10, 20): [(-2, -1), (-1, -1)]}


def test_one_into_two_split_emits_two_lane_links():
    """Single incoming lane splitting into two connecting lanes."""
    inputs = _make_inputs(
        junction_lanelets=[(101, -2), (102, -1)],
        incoming_lanelets=[(201, -1)],
        edges=[(101, [201]), (102, [201])],
    )

    pairs = _enumerate_lane_pairs(*inputs)

    assert pairs == {(10, 20): [(-1, -2), (-1, -1)]}


def test_chained_connector_is_not_advertised_from_the_external_road():
    """Only the head of an in-junction chain is reachable from outside.

    Regular road 10 -> connecting road 20 (lanelet 201 -> 101) ->
    connecting road 21 (lanelet 101 -> 102).  Road 21 is entered from road
    20, never straight from road 10, so it must not appear in the
    ``<connection>`` table: a consumer that took ``10 -> 21`` would skip
    the whole length of connector 20.  Resolving ``incomingRoad``
    transitively to road 10 (the earlier #492/#500 behaviour) advertised
    exactly that jump; road 21 stays reachable through its own
    ``<link><predecessor>``, which points at road 20.
    """
    junction_to_predecessors = {101: [201], 102: [101]}
    lanelet_to_road_id = {101: 20, 102: 21, 201: 10}
    road_id_to_lanelet_to_lane = {
        20: {101: -1},
        21: {102: -1},
        10: {201: -1},
    }

    pairs = _enumerate_lane_pairs(
        junction_to_predecessors,
        lanelet_to_road_id,
        road_id_to_lanelet_to_lane,
    )

    assert pairs == {(10, 20): [(-1, -1)]}


def test_deep_chain_advertises_only_its_head():
    """No stage of a multi-hop chain but the first is advertised.

    Chain: regular road 10 (lanelet 201) -> connecting road 20 (101)
    -> connecting road 21 (102) -> connecting road 22 (103).  Roads 21
    and 22 are reached by traversing the chain, so the junction table
    holds the single entry for its head.
    """
    junction_to_predecessors = {101: [201], 102: [101], 103: [102]}
    lanelet_to_road_id = {101: 20, 102: 21, 103: 22, 201: 10}
    road_id_to_lanelet_to_lane = {
        20: {101: -1},
        21: {102: -1},
        22: {103: -1},
        10: {201: -1},
    }

    pairs = _enumerate_lane_pairs(
        junction_to_predecessors,
        lanelet_to_road_id,
        road_id_to_lanelet_to_lane,
    )

    assert pairs == {(10, 20): [(-1, -1)]}


def test_chain_does_not_suppress_the_other_branches_of_its_head():
    """A connector fed both directly and through a chain keeps both roles.

    Road 10 feeds connectors 20 (lanelet 101) and 21 (lanelet 102)
    directly, and 20 continues into 22 (lanelet 103).  Dropping the
    chained stage must not disturb the two direct entries — a junction
    without chains is emitted exactly as before.
    """
    junction_to_predecessors = {101: [201], 102: [201], 103: [101]}
    lanelet_to_road_id = {101: 20, 102: 21, 103: 22, 201: 10}
    road_id_to_lanelet_to_lane = {
        20: {101: -1},
        21: {102: -1},
        22: {103: -1},
        10: {201: -1},
    }

    pairs = _enumerate_lane_pairs(
        junction_to_predecessors,
        lanelet_to_road_id,
        road_id_to_lanelet_to_lane,
    )

    assert pairs == {
        (10, 20): [(-1, -1)],
        (10, 21): [(-1, -1)],
    }


def test_connector_with_both_external_and_chained_predecessors_keeps_external():
    """Only the chained predecessor edge is dropped, not the whole connection.

    Connecting lanelet 102 is reached from external lanelet 202 *and*
    from in-junction lanelet 101.  The external edge must survive; the
    in-junction one must not add a second, earlier ``incomingRoad``.
    """
    junction_to_predecessors = {101: [201], 102: [101, 202]}
    lanelet_to_road_id = {101: 20, 102: 21, 201: 10, 202: 11}
    road_id_to_lanelet_to_lane = {
        20: {101: -1},
        21: {102: -1},
        10: {201: -1},
        11: {202: -1},
    }

    pairs = _enumerate_lane_pairs(
        junction_to_predecessors,
        lanelet_to_road_id,
        road_id_to_lanelet_to_lane,
    )

    assert pairs == {
        (10, 20): [(-1, -1)],
        (11, 21): [(-1, -1)],
    }


def test_missing_lane_mapping_drops_pair():
    """A lanelet without a lane mapping is dropped, not silently faked."""
    junction_to_predecessors = {101: [201]}
    lanelet_to_road_id = {101: 20, 201: 10}
    # Connecting lanelet 101 has no lane mapping -> the pair must be dropped.
    road_id_to_lanelet_to_lane = {
        20: {},
        10: {201: -1},
    }

    pairs = _enumerate_lane_pairs(
        junction_to_predecessors,
        lanelet_to_road_id,
        road_id_to_lanelet_to_lane,
    )

    assert pairs == {}


def test_duplicate_edges_are_deduplicated():
    """Repeating the same edge yields a single laneLink entry."""
    inputs = _make_inputs(
        junction_lanelets=[(101, -1)],
        incoming_lanelets=[(201, -1)],
        edges=[(101, [201, 201])],
    )

    pairs = _enumerate_lane_pairs(*inputs)

    assert pairs == {(10, 20): [(-1, -1)]}


def test_separate_incoming_roads_get_separate_keys():
    """Two incoming roads feeding the same connecting road -> two keys."""
    junction_to_predecessors = {101: [201, 301]}
    lanelet_to_road_id = {101: 20, 201: 10, 301: 11}
    road_id_to_lanelet_to_lane = {
        20: {101: -1},
        10: {201: -1},
        11: {301: -1},
    }

    pairs = _enumerate_lane_pairs(
        junction_to_predecessors,
        lanelet_to_road_id,
        road_id_to_lanelet_to_lane,
    )

    assert pairs == {
        (10, 20): [(-1, -1)],
        (11, 20): [(-1, -1)],
    }


# --- _build_connections_from_lane_pairs --------------------------------------
# Drives the Connection construction loop directly so a regression in the
# end-to-end emission path (e.g. dropping pairs after the first or
# assigning unstable IDs) trips the regression suite — independent of
# whether ``_enumerate_lane_pairs`` is correct.


def test_build_connections_emits_one_connection_with_multi_lanelinks():
    """Single (incoming, connecting) pair with N>1 lane links -> one Connection.

    Direct guard against the original 1:1 collapse: the Connection must
    carry one ``LaneLink`` per pair, in the helper's order.
    """
    pairs = {(10, 20): [(-2, -2), (-1, -1)]}

    connections = _build_connections_from_lane_pairs(pairs)

    assert len(connections) == 1
    conn = connections[0]
    assert conn.id == 0
    assert conn.incoming_road == 10
    assert conn.connecting_road == 20
    assert conn.contact_point == ContactPoint.START
    assert [(ll.from_lane, ll.to_lane) for ll in conn.lane_links] == [
        (-2, -2),
        (-1, -1),
    ]


def test_build_connections_assigns_sorted_deterministic_ids():
    """Connection IDs assigned in sorted (incoming, connecting) order."""
    pairs = {
        (11, 20): [(-1, -1)],
        (10, 20): [(-1, -1)],
        (10, 21): [(-1, -1)],
    }

    connections = _build_connections_from_lane_pairs(pairs)

    # Sorted: (10, 20) < (10, 21) < (11, 20)
    assert [(c.id, c.incoming_road, c.connecting_road) for c in connections] == [
        (0, 10, 20),
        (1, 10, 21),
        (2, 11, 20),
    ]


def test_build_connections_empty_input_returns_empty_list():
    assert _build_connections_from_lane_pairs({}) == []


def test_connection_xml_emits_one_lanelink_per_pair():
    """End-to-end: helper output -> Connection -> ``<laneLink>`` XML.

    Pins the user-visible XML shape so a regression in either the
    construction loop or ``Connection.to_xml`` would fail this test.
    """
    pairs = {(10, 20): [(-2, -2), (-1, -1)]}

    connections = _build_connections_from_lane_pairs(pairs)

    elem = connections[0].to_xml()
    lane_links = elem.findall("laneLink")

    assert len(lane_links) == 2
    assert [(ll.get("from"), ll.get("to")) for ll in lane_links] == [
        ("-2", "-2"),
        ("-1", "-1"),
    ]
    assert elem.get("incomingRoad") == "10"
    assert elem.get("connectingRoad") == "20"
    # Round-trip the bytes too so any XML namespace / serialization
    # regression also surfaces.
    rendered = ET.tostring(elem, encoding="unicode")
    assert rendered.count("<laneLink") == 2
