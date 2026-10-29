"""Merging junctions that claim the same road end.

A ``<connection>`` row claims one end of its incoming road. That end carries
a single ``<link>`` slot, so two junctions claiming it can never both be read
and one of them holds rows nothing will ever consult.

The counting tests here are **fault-injection** tests: each one seeds an exact
number of contested road ends into a network that has none, and asserts the
pass merges away exactly that many junctions. "More than zero" would also pass
against a pass that merged everything in sight, which is why the numbers are
pinned rather than bounded. Every one of them carries a control -- a junction
whose road ends nothing else claims -- and asserts it comes through untouched.
"""

from __future__ import annotations

from typing import List, Tuple

import pytest

from autoware_lanelet2_to_opendrive.junction_claims import (
    merge_contested_junction_claims,
)
from autoware_lanelet2_to_opendrive.opendrive.enums import ContactPoint, ElementType
from autoware_lanelet2_to_opendrive.opendrive.junction import (
    Connection,
    Junction,
    LaneLink,
)
from autoware_lanelet2_to_opendrive.opendrive.road import Road
from autoware_lanelet2_to_opendrive.opendrive.road_links import (
    Predecessor,
    RoadLink,
    Successor,
)


def _incoming(road_id: int, junction_id: int) -> Road:
    """A plain road whose END is a junction, the way the converter emits it."""
    return Road(
        id=road_id,
        junction=-1,
        link=RoadLink(
            successor=Successor(
                element_type=ElementType.JUNCTION, element_id=junction_id
            )
        ),
    )


def _connector(road_id: int, junction_id: int, incoming_id: int) -> Road:
    """A connecting road of ``junction_id`` fed by ``incoming_id``."""
    return Road(
        id=road_id,
        junction=junction_id,
        link=RoadLink(
            predecessor=Predecessor(
                element_type=ElementType.ROAD,
                element_id=incoming_id,
                contact_point=ContactPoint.END,
            )
        ),
    )


def _junction(junction_id: int, rows: List[Tuple[int, int]]) -> Junction:
    """``rows`` is ``[(incoming road id, connecting road id), ...]``."""
    return Junction(
        id=junction_id,
        connections=[
            Connection(
                id=index,
                incoming_road=incoming,
                connecting_road=connecting,
                contact_point=ContactPoint.START,
                lane_links=[LaneLink(from_lane=1, to_lane=1)],
            )
            for index, (incoming, connecting) in enumerate(rows)
        ],
    )


def _network(faults: int):
    """A network of ``faults`` contested road ends plus one clean control.

    Road ``100 + i`` ends at junction ``200 + i``. For each seeded fault a
    *second* junction ``300 + i`` is given a row for the same road end, which
    is precisely the state the three link-writing passes can leave behind.
    Road 999 / junction 900 is the control: nothing else claims its end.
    """
    roads = [_incoming(999, 900), _connector(950, 900, 999)]
    junctions = [_junction(900, [(999, 950)])]
    for i in range(faults):
        incoming_id = 100 + i
        first_id, second_id = 200 + i, 300 + i
        roads.append(_incoming(incoming_id, first_id))
        roads.append(_connector(500 + i, first_id, incoming_id))
        roads.append(_connector(600 + i, second_id, incoming_id))
        junctions.append(_junction(first_id, [(incoming_id, 500 + i)]))
        junctions.append(_junction(second_id, [(incoming_id, 600 + i)]))
    return roads, junctions


def _contested_ends(junctions: List[Junction]) -> int:
    """Road ends more than one junction holds a ``<connection>`` row for."""
    claims: dict = {}
    for junction in junctions:
        for connection in junction.connections:
            end = "end" if connection.contact_point == ContactPoint.START else "start"
            claims.setdefault((connection.incoming_road, end), set()).add(junction.id)
    return sum(1 for claimants in claims.values() if len(claimants) > 1)


def _lane_edges(junctions: List[Junction]):
    """Every ``(incoming road, from lane, connecting road, to lane)`` carried."""
    return {
        (c.incoming_road, link.from_lane, c.connecting_road, link.to_lane)
        for junction in junctions
        for c in junction.connections
        for link in c.lane_links
    }


# --------------------------------------------------------------------------
# Fault injection: the merge count tracks the seeded fault count exactly.
# --------------------------------------------------------------------------


def test_a_network_with_no_contested_end_is_left_alone() -> None:
    """The zero case, and the baseline the seeded cases are measured against."""
    roads, junctions = _network(faults=0)
    assert _contested_ends(junctions) == 0

    before = _lane_edges(junctions)
    assert merge_contested_junction_claims(all_roads=roads, junctions=junctions) == 0
    assert [j.id for j in junctions] == [900]
    assert _lane_edges(junctions) == before


@pytest.mark.parametrize("faults", [1, 2, 3, 7])
def test_merge_count_equals_the_number_of_seeded_contested_ends(faults: int) -> None:
    roads, junctions = _network(faults)
    assert _contested_ends(junctions) == faults

    merged = merge_contested_junction_claims(all_roads=roads, junctions=junctions)

    assert merged == faults
    assert _contested_ends(junctions) == 0
    # Control: the uncontested junction survives with its row intact.
    control = [j for j in junctions if j.id == 900]
    assert len(control) == 1
    assert [(c.incoming_road, c.connecting_road) for c in control[0].connections] == [
        (999, 950)
    ]


@pytest.mark.parametrize("faults", [1, 2, 3, 7])
def test_no_lane_edge_is_lost_to_the_merge(faults: int) -> None:
    """The merge re-homes maneuvers; it must not drop any."""
    roads, junctions = _network(faults)
    before = _lane_edges(junctions)

    merge_contested_junction_claims(all_roads=roads, junctions=junctions)

    assert _lane_edges(junctions) == before


def test_the_surviving_junction_is_the_lowest_id() -> None:
    """Deterministic, so the emitted ids do not depend on write order."""
    roads, junctions = _network(faults=1)

    merge_contested_junction_claims(all_roads=roads, junctions=junctions)

    assert sorted(j.id for j in junctions) == [200, 900]


def test_roads_and_links_follow_the_junction_that_survives() -> None:
    roads, junctions = _network(faults=1)

    merge_contested_junction_claims(all_roads=roads, junctions=junctions)

    by_id = {road.id: road for road in roads}
    # The merged-away junction's connecting road is re-homed...
    assert by_id[600].junction == 200
    # ...and no road link names a junction that is gone.
    surviving = {j.id for j in junctions}
    for road in roads:
        if road.link is None:
            continue
        for link in (road.link.predecessor, road.link.successor):
            if link is not None and link.element_type == ElementType.JUNCTION:
                assert link.element_id in surviving


def test_a_chain_of_shared_ends_collapses_to_one_junction() -> None:
    """Three junctions linked by two shared ends are one place, not two pairs."""
    roads = [_incoming(10, 20), _incoming(11, 22)]
    junctions = [
        _junction(20, [(10, 50)]),
        _junction(21, [(10, 51), (11, 52)]),
        _junction(22, [(11, 53)]),
    ]

    merged = merge_contested_junction_claims(all_roads=roads, junctions=junctions)

    assert merged == 2
    assert [j.id for j in junctions] == [20]
    assert sorted(c.connecting_road for c in junctions[0].connections) == [
        50,
        51,
        52,
        53,
    ]
    assert [c.id for c in junctions[0].connections] == [0, 1, 2, 3]


def test_rows_for_the_same_transition_are_unioned_not_duplicated() -> None:
    """Two junctions can describe one maneuver; the merge keeps one row."""
    roads = [_incoming(10, 20)]
    junctions = [
        Junction(
            id=20,
            connections=[
                Connection(
                    id=0,
                    incoming_road=10,
                    connecting_road=50,
                    contact_point=ContactPoint.START,
                    lane_links=[LaneLink(from_lane=1, to_lane=1)],
                )
            ],
        ),
        Junction(
            id=21,
            connections=[
                Connection(
                    id=0,
                    incoming_road=10,
                    connecting_road=50,
                    contact_point=ContactPoint.START,
                    lane_links=[
                        LaneLink(from_lane=1, to_lane=1),
                        LaneLink(from_lane=2, to_lane=2),
                    ],
                )
            ],
        ),
    ]

    merge_contested_junction_claims(all_roads=roads, junctions=junctions)

    assert len(junctions) == 1
    assert len(junctions[0].connections) == 1
    links = junctions[0].connections[0].lane_links
    assert sorted((link.from_lane, link.to_lane) for link in links) == [(1, 1), (2, 2)]


def test_the_two_ends_of_one_road_are_different_claims() -> None:
    """A road feeding one junction at each end is not a conflict."""
    roads = [
        Road(
            id=10,
            junction=-1,
            link=RoadLink(
                predecessor=Predecessor(
                    element_type=ElementType.JUNCTION, element_id=20
                ),
                successor=Successor(element_type=ElementType.JUNCTION, element_id=21),
            ),
        )
    ]
    junctions = [
        _junction(21, [(10, 50)]),  # claims road 10's END (contactPoint=start)
        Junction(
            id=20,
            connections=[
                Connection(
                    id=0,
                    incoming_road=10,
                    connecting_road=51,
                    contact_point=ContactPoint.END,  # claims road 10's START
                    lane_links=[LaneLink(from_lane=1, to_lane=1)],
                )
            ],
        ),
    ]

    assert merge_contested_junction_claims(all_roads=roads, junctions=junctions) == 0
    assert sorted(j.id for j in junctions) == [20, 21]
