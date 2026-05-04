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
