"""Tests for allocating junction IDs above the road ID range."""

from typing import List, NamedTuple, Union

import pytest

from autoware_lanelet2_to_opendrive.conversion_config import (
    ConversionConfig,
    OriginSpec,
)
from autoware_lanelet2_to_opendrive.main import convert_lanelet2_to_opendrive
from autoware_lanelet2_to_opendrive.opendrive.enums import ContactPoint, ElementType
from autoware_lanelet2_to_opendrive.opendrive.junction import (
    Junction,
    junction_id_base,
    shift_junction_ids,
)
from autoware_lanelet2_to_opendrive.opendrive.road import Road
from autoware_lanelet2_to_opendrive.opendrive.road_links import (
    Predecessor,
    RoadLink,
    Successor,
)


@pytest.mark.parametrize(
    "max_road_id,offset,expected",
    [
        (-1, 1000, 1000),  # no roads
        (692, 1000, 1000),  # Nishishinjuku: highest road ID below the offset
        (999, 1000, 1000),
        (1000, 1000, 2000),  # road 1000 would collide with junction 1000
        (4739, 1000, 5000),
        (4739, 100_000, 100_000),
        (5, 1, 6),
    ],
)
def test_junction_id_base_is_first_multiple_above_road_ids(
    max_road_id, offset, expected
):
    assert junction_id_base(max_road_id, offset) == expected


def test_junction_id_base_rejects_non_positive_offset():
    with pytest.raises(ValueError, match="positive"):
        junction_id_base(10, 0)


def test_conversion_config_rejects_non_positive_junction_id_offset():
    with pytest.raises(ValueError, match="junction_id_offset"):
        ConversionConfig(junction_id_offset=0)


class _References(NamedTuple):
    """Two junctions, the roads referring to them and the links naming them."""

    junctions: List[Junction]
    roads: List[Road]
    into_junction: Successor  # incoming road -> junction 11000
    from_junction: Predecessor  # outgoing road <- junction 1000
    road_typed: List[Union[Predecessor, Successor]]  # must not move


def _junction_references() -> _References:
    into_junction = Successor(ElementType.JUNCTION, 11000)
    from_junction = Predecessor(ElementType.JUNCTION, 1000)
    from_road = Predecessor(ElementType.ROAD, 3, ContactPoint.END)
    synthetic_in = Predecessor(ElementType.ROAD, 0, ContactPoint.END)
    synthetic_out = Successor(ElementType.ROAD, 3, ContactPoint.START)
    roads = [
        Road(id=0, link=RoadLink(predecessor=from_road, successor=into_junction)),
        Road(id=1, junction=1000),
        Road(
            id=2,
            junction=11000,
            link=RoadLink(predecessor=synthetic_in, successor=synthetic_out),
        ),
        Road(id=3, link=RoadLink(predecessor=from_junction)),
    ]
    return _References(
        junctions=[Junction(id=1000), Junction(id=11000)],
        roads=roads,
        into_junction=into_junction,
        from_junction=from_junction,
        road_typed=[from_road, synthetic_in, synthetic_out],
    )


def test_shift_junction_ids_moves_ids_and_every_reference():
    refs = _junction_references()

    shift_junction_ids(refs.junctions, refs.roads, 4000)

    assert [junction.id for junction in refs.junctions] == [5000, 15000]
    assert [road.junction for road in refs.roads] == [-1, 5000, 15000, -1]
    assert refs.into_junction.element_id == 15000
    assert refs.from_junction.element_id == 5000
    assert [link.element_id for link in refs.road_typed] == [3, 0, 3]


def test_shift_junction_ids_by_zero_changes_nothing():
    refs = _junction_references()

    shift_junction_ids(refs.junctions, refs.roads, 0)

    assert [junction.id for junction in refs.junctions] == [1000, 11000]
    assert [road.junction for road in refs.roads] == [-1, 1000, 11000, -1]
    assert refs.into_junction.element_id == 11000
    assert refs.from_junction.element_id == 1000


def test_converted_junction_ids_lie_above_every_road_id(lanelet_map):
    """An offset below the highest road ID moves the junction IDs above the roads."""
    config = ConversionConfig(
        origin=OriginSpec(mgrs_code="54SUE"),  # the header's geoReference needs one
        junction_id_offset=1,
        traffic_rule="LHT",
    )
    opendrive, *_ = convert_lanelet2_to_opendrive(lanelet_map, config)
    road_ids = {road.id for road in opendrive.roads}
    junction_ids = {junction.id for junction in opendrive.junctions}

    assert junction_ids
    assert min(junction_ids) == max(road_ids) + 1
    assert not junction_ids & road_ids
    for road in opendrive.roads:
        if road.junction != -1:
            assert road.junction in junction_ids
        if road.link is None:
            continue
        for element in (road.link.predecessor, road.link.successor):
            if element is not None and element.element_type == ElementType.JUNCTION:
                assert element.element_id in junction_ids
