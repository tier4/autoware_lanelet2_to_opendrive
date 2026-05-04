"""Tests for OpenDRIVE <junction><priority/> emission from right_of_way REs."""

from __future__ import annotations

import logging

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


def test_build_priorities_n_to_m_cartesian():
    """ROW={l1,l2}, yield={l3,l4} -> 4 priorities."""
    from autoware_lanelet2_to_opendrive.opendrive.junction import (
        _RightOfWayRecord,
        _build_priorities_from_records,
    )

    records = [
        _RightOfWayRecord(
            re_id=1,
            row_lanelet_ids=(101, 102),
            yield_lanelet_ids=(103, 104),
        )
    ]
    lanelet_to_road_id = {101: 11, 102: 12, 103: 13, 104: 14}
    lanelet_to_junction_id = {101: 9, 102: 9, 103: 9, 104: 9}

    result = _build_priorities_from_records(
        records, lanelet_to_road_id, lanelet_to_junction_id
    )

    assert result == {
        9: [
            Priority(high=11, low=13),
            Priority(high=11, low=14),
            Priority(high=12, low=13),
            Priority(high=12, low=14),
        ],
    }


def test_build_priorities_dedup_across_res():
    """Two REs claim the same (high, low) -> one priority."""
    from autoware_lanelet2_to_opendrive.opendrive.junction import (
        _RightOfWayRecord,
        _build_priorities_from_records,
    )

    records = [
        _RightOfWayRecord(re_id=1, row_lanelet_ids=(101,), yield_lanelet_ids=(103,)),
        _RightOfWayRecord(re_id=2, row_lanelet_ids=(101,), yield_lanelet_ids=(103,)),
    ]
    lanelet_to_road_id = {101: 11, 103: 13}
    lanelet_to_junction_id = {101: 9, 103: 9}

    result = _build_priorities_from_records(
        records, lanelet_to_road_id, lanelet_to_junction_id
    )

    assert result == {9: [Priority(high=11, low=13)]}


def test_build_priorities_dedup_via_road_merge():
    """l1 and l2 merged into the same connecting road -> one priority."""
    from autoware_lanelet2_to_opendrive.opendrive.junction import (
        _RightOfWayRecord,
        _build_priorities_from_records,
    )

    records = [
        _RightOfWayRecord(re_id=1, row_lanelet_ids=(101, 102), yield_lanelet_ids=(103,))
    ]
    # l1 and l2 both belong to road 11.
    lanelet_to_road_id = {101: 11, 102: 11, 103: 13}
    lanelet_to_junction_id = {101: 9, 102: 9, 103: 9}

    result = _build_priorities_from_records(
        records, lanelet_to_road_id, lanelet_to_junction_id
    )

    assert result == {9: [Priority(high=11, low=13)]}


def test_build_priorities_self_priority_skipped(caplog):
    """ROW and yield both resolve to the same road -> no priority emitted."""
    from autoware_lanelet2_to_opendrive.opendrive.junction import (
        _RightOfWayRecord,
        _build_priorities_from_records,
    )

    records = [
        _RightOfWayRecord(re_id=42, row_lanelet_ids=(101,), yield_lanelet_ids=(102,))
    ]
    lanelet_to_road_id = {101: 11, 102: 11}  # both lanelets on road 11
    lanelet_to_junction_id = {101: 9, 102: 9}

    with caplog.at_level(
        logging.DEBUG,
        logger="autoware_lanelet2_to_opendrive.opendrive.junction",
    ):
        result = _build_priorities_from_records(
            records, lanelet_to_road_id, lanelet_to_junction_id
        )

    assert result == {}
    assert any("self-priority on road 11" in rec.message for rec in caplog.records)


def test_build_priorities_cross_junction_warns(caplog):
    """ROW lanelets in junction A, yield lanelets in junction B -> skip RE with WARNING."""
    from autoware_lanelet2_to_opendrive.opendrive.junction import (
        _RightOfWayRecord,
        _build_priorities_from_records,
    )

    records = [
        _RightOfWayRecord(re_id=42, row_lanelet_ids=(101,), yield_lanelet_ids=(102,))
    ]
    lanelet_to_road_id = {101: 11, 102: 22}
    lanelet_to_junction_id = {101: 9, 102: 8}  # different junctions

    with caplog.at_level(
        logging.WARNING,
        logger="autoware_lanelet2_to_opendrive.opendrive.junction",
    ):
        result = _build_priorities_from_records(
            records, lanelet_to_road_id, lanelet_to_junction_id
        )

    # row_jid resolves to {9}, yield_jid resolves to {8}; row+yield checked separately.
    # Both row_jid and yield_jid are non-None, but they differ, so we expect
    # the "span junctions" warning.
    assert result == {}
    assert any(
        "span junctions" in rec.message and rec.levelno == logging.WARNING
        for rec in caplog.records
    )


def test_build_priorities_no_junction_lanelet_warns(caplog):
    """Lanelets that aren't in any junction -> WARNING + skip RE."""
    from autoware_lanelet2_to_opendrive.opendrive.junction import (
        _RightOfWayRecord,
        _build_priorities_from_records,
    )

    records = [
        _RightOfWayRecord(re_id=42, row_lanelet_ids=(101,), yield_lanelet_ids=(102,))
    ]
    lanelet_to_road_id = {101: 11, 102: 22}
    lanelet_to_junction_id: dict[int, int] = {}  # neither lanelet has a junction

    with caplog.at_level(
        logging.WARNING,
        logger="autoware_lanelet2_to_opendrive.opendrive.junction",
    ):
        result = _build_priorities_from_records(
            records, lanelet_to_road_id, lanelet_to_junction_id
        )

    assert result == {}
    assert any(
        "cannot determine owning junction" in rec.message for rec in caplog.records
    )


def test_build_priorities_conflict_both_emitted(caplog):
    """RE1: A>B, RE2: B>A -> both <priority> emitted, ONE warning per pair."""
    from autoware_lanelet2_to_opendrive.opendrive.junction import (
        _RightOfWayRecord,
        _build_priorities_from_records,
    )

    records = [
        _RightOfWayRecord(re_id=1, row_lanelet_ids=(101,), yield_lanelet_ids=(102,)),
        _RightOfWayRecord(re_id=2, row_lanelet_ids=(102,), yield_lanelet_ids=(101,)),
    ]
    lanelet_to_road_id = {101: 11, 102: 22}
    lanelet_to_junction_id = {101: 9, 102: 9}

    with caplog.at_level(
        logging.WARNING,
        logger="autoware_lanelet2_to_opendrive.opendrive.junction",
    ):
        result = _build_priorities_from_records(
            records, lanelet_to_road_id, lanelet_to_junction_id
        )

    assert result == {
        9: [Priority(high=11, low=22), Priority(high=22, low=11)],
    }
    conflict_warnings = [
        rec for rec in caplog.records if "Conflicting priority" in rec.message
    ]
    assert len(conflict_warnings) == 1, conflict_warnings
    msg = conflict_warnings[0].message
    assert "junction 9" in msg
    assert "REs [1] vs [2]" in msg or "REs [2] vs [1]" in msg


def test_extract_right_of_way_records_from_nishishinjuku(lanelet_map):
    """Confirm the lanelet2 walk yields exactly 85 right_of_way records.

    Uses the shared `lanelet_map` fixture from conftest.py
    (loads nishishinjuku.osm).
    """
    from autoware_lanelet2_to_opendrive.opendrive.junction import (
        _extract_right_of_way_records,
    )

    records = list(_extract_right_of_way_records(lanelet_map))

    # nishishinjuku.osm carries 85 right_of_way REs (verified by grep).
    assert len(records) == 85

    # Every record has at least one lanelet on each side, otherwise the
    # extractor should have skipped it.
    for r in records:
        assert r.row_lanelet_ids
        assert r.yield_lanelet_ids
        assert isinstance(r.re_id, int)
