"""Tests for OpenDRIVE <junction><priority/> emission from right_of_way REs."""

from __future__ import annotations

from autoware_lanelet2_to_opendrive.opendrive.junction import Priority


def test_priority_to_xml_emits_high_and_low_attributes():
    p = Priority(high=10, low=20)
    elem = p.to_xml()

    assert elem.tag == "priority"
    assert elem.get("high") == "10"
    assert elem.get("low") == "20"
    assert list(elem) == []  # no children


def test_priority_is_hashable_and_equal_by_value():
    a = Priority(high=10, low=20)
    b = Priority(high=10, low=20)
    c = Priority(high=20, low=10)

    assert a == b
    assert hash(a) == hash(b)
    assert a != c
    assert {a, b, c} == {a, c}  # set deduplicates a vs b


def test_junction_to_xml_emits_priority_between_connection_and_controller():
    """OpenDRIVE 1.4 t_junction XSD requires connection* -> priority* -> controller*."""
    from autoware_lanelet2_to_opendrive.opendrive.enums import ContactPoint
    from autoware_lanelet2_to_opendrive.opendrive.junction import (
        Connection,
        Junction,
    )

    junction = Junction(id=1000, name="J")
    junction.connections.append(
        Connection(
            id=0,
            incoming_road=1,
            connecting_road=10,
            contact_point=ContactPoint.START,
        )
    )
    junction.priorities.append(Priority(high=10, low=20))
    junction.controller_ids.append(99)

    elem = junction.to_xml()
    child_tags = [child.tag for child in elem]

    assert child_tags == ["connection", "priority", "controller"]


def test_junction_to_xml_with_no_priorities_unchanged():
    from autoware_lanelet2_to_opendrive.opendrive.enums import ContactPoint
    from autoware_lanelet2_to_opendrive.opendrive.junction import (
        Connection,
        Junction,
    )

    junction = Junction(id=1000)
    junction.connections.append(
        Connection(
            id=0,
            incoming_road=1,
            connecting_road=10,
            contact_point=ContactPoint.START,
        )
    )
    junction.controller_ids.append(99)

    elem = junction.to_xml()
    child_tags = [child.tag for child in elem]

    assert child_tags == ["connection", "controller"]  # no priority slot


def test_build_priorities_simple_n_to_1():
    """ROW={l1, l2}, yield={l3} in junction J -> 2 priorities (R1>R3, R2>R3)."""
    from autoware_lanelet2_to_opendrive.opendrive.junction import (
        _RightOfWayRecord,
        _build_priorities_from_records,
    )

    records = [
        _RightOfWayRecord(
            re_id=42,
            row_lanelet_ids=(101, 102),
            yield_lanelet_ids=(103,),
        )
    ]
    lanelet_to_road_id = {101: 1001, 102: 1002, 103: 1003}
    lanelet_to_junction_id = {101: 9, 102: 9, 103: 9}

    result = _build_priorities_from_records(
        records, lanelet_to_road_id, lanelet_to_junction_id
    )

    assert result == {
        9: [Priority(high=1001, low=1003), Priority(high=1002, low=1003)],
    }
