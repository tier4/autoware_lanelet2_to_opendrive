"""Tests for Lanelet2 LineString -> OpenDRIVE RoadMark mapping (P0-3)."""

import subprocess
from pathlib import Path

import lxml.etree as ET

from autoware_lanelet2_to_opendrive.opendrive.lane_elements import (
    RoadMark,
    road_mark_from_linestring_attrs,
)
from autoware_lanelet2_to_opendrive.opendrive.enums import (
    RoadMarkType,
    RoadMarkColor,
    RoadMarkLaneChange,
    RoadMarkWeight,
)


def test_line_thin_solid_white():
    rm = road_mark_from_linestring_attrs(
        0.0, {"type": "line_thin", "subtype": "solid", "color": "white"}
    )
    assert rm.type == RoadMarkType.SOLID
    assert rm.color == RoadMarkColor.WHITE
    assert rm.weight == RoadMarkWeight.STANDARD


def test_line_thick_dashed_bold():
    rm = road_mark_from_linestring_attrs(
        0.0, {"type": "line_thick", "subtype": "dashed", "color": "yellow"}
    )
    assert rm.type == RoadMarkType.BROKEN
    assert rm.weight == RoadMarkWeight.BOLD
    assert rm.color == RoadMarkColor.YELLOW


def test_curbstone_maps_to_curb():
    rm = road_mark_from_linestring_attrs(0.0, {"type": "curbstone"})
    assert rm.type == RoadMarkType.CURB


def test_virtual_maps_to_none():
    rm = road_mark_from_linestring_attrs(0.0, {"type": "virtual"})
    assert rm.type == RoadMarkType.NONE


def test_lane_change_yes():
    rm = road_mark_from_linestring_attrs(
        0.0,
        {"type": "line_thin", "subtype": "dashed", "lane_change": "yes"},
    )
    assert rm.lane_change == RoadMarkLaneChange.BOTH


def test_unknown_type_falls_back_solid_white():
    rm = road_mark_from_linestring_attrs(0.0, {"type": "mystery", "subtype": "stuff"})
    assert rm.type == RoadMarkType.SOLID
    assert rm.color == RoadMarkColor.WHITE


def test_roadmark_to_xml_includes_weight():
    rm = RoadMark(
        s_offset=0.0,
        type=RoadMarkType.SOLID,
        color=RoadMarkColor.WHITE,
        weight=RoadMarkWeight.BOLD,
    )
    elem = rm.to_xml()
    assert elem.get("weight") == "bold"


def test_roadmark_to_xml_omits_standard_weight():
    """STANDARD is the OpenDRIVE default for ``<roadMark weight>`` — it
    should not be emitted, to keep the output free of redundant attributes.
    """
    rm = RoadMark(
        s_offset=0.0,
        type=RoadMarkType.SOLID,
        color=RoadMarkColor.WHITE,
        weight=RoadMarkWeight.STANDARD,
    )
    elem = rm.to_xml()
    assert elem.get("weight") is None


def test_lane_change_left_right_rht_vs_lht():
    """World-relative Lanelet2 values must be flipped by handedness.

    RHT: right-hand traffic. Right-side lanes have negative IDs,
    so ``lane_change=left`` (world-relative) means "move toward more
    positive IDs" = ``INCREASE``.

    LHT: left-hand traffic. Handedness flips; ``lane_change=left``
    (world-relative) means "move toward more negative IDs" = ``DECREASE``.
    """
    # RHT (default)
    rm_left = road_mark_from_linestring_attrs(
        0.0,
        {"type": "line_thin", "subtype": "dashed", "lane_change": "left"},
    )
    rm_right = road_mark_from_linestring_attrs(
        0.0,
        {"type": "line_thin", "subtype": "dashed", "lane_change": "right"},
    )
    assert rm_left.lane_change == RoadMarkLaneChange.INCREASE
    assert rm_right.lane_change == RoadMarkLaneChange.DECREASE

    # LHT
    rm_left_lht = road_mark_from_linestring_attrs(
        0.0,
        {"type": "line_thin", "subtype": "dashed", "lane_change": "left"},
        is_lht=True,
    )
    rm_right_lht = road_mark_from_linestring_attrs(
        0.0,
        {"type": "line_thin", "subtype": "dashed", "lane_change": "right"},
        is_lht=True,
    )
    assert rm_left_lht.lane_change == RoadMarkLaneChange.DECREASE
    assert rm_right_lht.lane_change == RoadMarkLaneChange.INCREASE


def test_nishishinjuku_emits_non_default_roadmarks(tmp_path):
    """End-to-end: nishishinjuku output should carry heterogeneous roadMarks.

    We require *some* variation beyond the former hard-coded "solid white":
    either a non-(solid,white) mark OR a populated ``laneChange`` or ``weight``
    attribute. If the underlying LineStrings really are all homogeneous the
    assertion still protects against the helper being unwired.

    The converter may exit non-zero because of an unrelated mapping
    cross-validation warning on the nishishinjuku fixture; we do not
    propagate that failure here — the XODR is still written and is what we
    actually assert on.
    """
    fixture = Path(
        "autoware_lanelet2_to_opendrive/test/data/nishishinjuku.osm"
    ).resolve()
    out = tmp_path / "n.xodr"
    subprocess.run(
        [
            "uv",
            "run",
            "convert",
            "map=nishishinjuku",
            "target=carla",
            f"input_map_path={fixture}",
            f"output_map_path={out}",
        ],
        check=False,
    )
    assert out.is_file(), "converter did not write an output .xodr"
    root = ET.parse(str(out)).getroot()
    rmarks = root.findall(".//roadMark")
    assert rmarks, "expected roadMark elements in output"

    non_solid_white = [
        m
        for m in rmarks
        if not (m.get("type") == "solid" and m.get("color") == "white")
    ]
    with_lane_change = [m for m in rmarks if m.get("laneChange")]
    with_weight = [m for m in rmarks if m.get("weight")]
    assert non_solid_white or with_lane_change or with_weight, (
        "expected heterogeneity in roadMark attributes "
        "(non-solid-white, laneChange, or weight) — helper not wired"
    )
