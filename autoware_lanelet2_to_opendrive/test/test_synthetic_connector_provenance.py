"""A synthetic connector must be known by provenance, not by being short.

`_synthetic_connector_road_ids` answers "which connecting roads have no
backing lanelet", and the geo-mapping phase skips those: a road with no
lanelet must not compete for one. It answers by testing
``planView length < 0.5 m``, and its docstring says why that works today --
"Real ``turn_direction`` connecting roads are at least several metres long".

Measured on nishishinjuku, that holds with room to spare: every synthetic stub
is 0.010 m and the shortest real connector is 2.598 m. But it is a proxy, and
it fails for a synthetic road longer than the threshold -- which is exactly
what the junction carve produces, since giving connectors real extent is the
whole point of carving.

The provenance also has to survive the round trip through the emitted file:
the conversion cross-validates its own mapping against one built from
``parse_roads_from_xodr``, so a signal that exists only in memory makes the
two disagree instead of agreeing.
"""

from __future__ import annotations

from pathlib import Path

import lxml.etree as ET

from autoware_lanelet2_to_opendrive.divergence import (
    _make_zero_length_connecting_road,
)
from autoware_lanelet2_to_opendrive.opendrive.enums import ContactPoint, TrafficRule
from autoware_lanelet2_to_opendrive.road_lanelet_geo_mapping import (
    _synthetic_connector_road_ids,
    parse_roads_from_xodr,
)

CARVE_LENGTH = 1.0
"""Longer than the 0.5 m threshold, which is the point of the test."""


def _synthetic_connector(road_id: int, length: float):
    """Build one through the production builder, not by hand."""
    return _make_zero_length_connecting_road(
        road_id=road_id,
        junction_id=11000,
        incoming_road_id=10,
        outgoing_road_id=20,
        incoming_contact=ContactPoint.END,
        outgoing_contact=ContactPoint.START,
        start_xyz=(0.0, 0.0, 0.0),
        end_xyz=(length, 0.0, 0.0),
        min_segment_length=length,
        traffic_rule=TrafficRule.RHT,
        from_lane=-1,
        to_lane=-1,
        heading=0.0,
        lane_width=3.5,
    )


def test_a_short_synthetic_connector_is_recognised() -> None:
    """The present behaviour, kept as the guard that nothing regressed."""
    road = _synthetic_connector(road_id=100, length=0.01)
    assert _synthetic_connector_road_ids([road]) == {100}


def test_a_synthetic_connector_longer_than_the_threshold_is_recognised() -> None:
    """A synthetic road with no lanelet must be skipped whatever its length.

    Without this, a carved connector enters the geo-mapping phase, claims the
    lanelet whose stretch it occupies, and Phase 2 drops either it or the road
    that legitimately owns that lanelet.
    """
    road = _synthetic_connector(road_id=100, length=CARVE_LENGTH)
    assert road.length == CARVE_LENGTH, "the fixture must exceed the threshold"

    assert _synthetic_connector_road_ids([road]) == {100}


def test_a_real_connecting_road_is_not_recognised() -> None:
    """Non-vacuity: the set must not simply contain every connecting road."""
    xodr = (
        "<OpenDRIVE>"
        "<road id='200' name='Road_200' junction='1000'><planView>"
        "<geometry s='0.0' x='0.0' y='0.0' hdg='0.0' length='12.0'>"
        "<line/></geometry></planView></road>"
        "</OpenDRIVE>"
    )
    roads = parse_roads_from_xodr(Path("unused.xodr"), xodr_root=ET.fromstring(xodr))
    assert _synthetic_connector_road_ids(roads) == set()


def test_the_provenance_survives_the_round_trip_through_the_file() -> None:
    """Re-parsed from the emitted XODR, it must still be synthetic.

    The conversion builds its mapping from the in-memory roads and
    cross-validates it against one built from the re-parsed file. A provenance
    signal held only in memory makes those two disagree.
    """
    road = _synthetic_connector(road_id=100, length=CARVE_LENGTH)

    document = ET.Element("OpenDRIVE")
    document.append(road.to_xml())
    reparsed = parse_roads_from_xodr(Path("unused.xodr"), xodr_root=document)

    assert [r.id for r in reparsed] == [100], "the road must survive the round trip"
    assert _synthetic_connector_road_ids(reparsed) == {100}
