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
