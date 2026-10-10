"""Tests for main conversion functionality and RoadLaneletMapping."""

import subprocess
from pathlib import Path

import lxml.etree as ET

from autoware_lanelet2_to_opendrive.util import RoadLaneletMapping


def test_road_lanelet_mapping_creation():
    """Test RoadLaneletMapping creation."""
    road_to_lanelets = {
        0: [100, 101, 102],
        1: [200, 201],
        2: [300],
    }
    lanelet_to_road = {
        100: 0,
        101: 0,
        102: 0,
        200: 1,
        201: 1,
        300: 2,
    }

    mapping = RoadLaneletMapping(
        road_to_lanelets=road_to_lanelets, lanelet_to_road=lanelet_to_road
    )

    assert mapping.road_to_lanelets == road_to_lanelets
    assert mapping.lanelet_to_road == lanelet_to_road


def test_get_lanelets_for_road():
    """Test getting lanelets for a specific road."""
    mapping = RoadLaneletMapping(
        road_to_lanelets={0: [100, 101, 102], 1: [200, 201]},
        lanelet_to_road={100: 0, 101: 0, 102: 0, 200: 1, 201: 1},
    )

    # Test existing road
    lanelets = mapping.get_lanelets_for_road(0)
    assert lanelets == [100, 101, 102]

    lanelets = mapping.get_lanelets_for_road(1)
    assert lanelets == [200, 201]

    # Test non-existing road
    lanelets = mapping.get_lanelets_for_road(999)
    assert lanelets == []


def test_get_road_for_lanelet():
    """Test getting road for a specific lanelet."""
    mapping = RoadLaneletMapping(
        road_to_lanelets={0: [100, 101, 102], 1: [200, 201]},
        lanelet_to_road={100: 0, 101: 0, 102: 0, 200: 1, 201: 1},
    )

    # Test existing lanelets
    assert mapping.get_road_for_lanelet(100) == 0
    assert mapping.get_road_for_lanelet(101) == 0
    assert mapping.get_road_for_lanelet(102) == 0
    assert mapping.get_road_for_lanelet(200) == 1
    assert mapping.get_road_for_lanelet(201) == 1

    # Test non-existing lanelet
    assert mapping.get_road_for_lanelet(999) is None


def test_mapping_consistency():
    """Test that forward and reverse mappings are consistent."""
    mapping = RoadLaneletMapping(
        road_to_lanelets={0: [100, 101, 102], 1: [200, 201], 2: [300, 301, 302, 303]},
        lanelet_to_road={
            100: 0,
            101: 0,
            102: 0,
            200: 1,
            201: 1,
            300: 2,
            301: 2,
            302: 2,
            303: 2,
        },
    )

    # Verify all lanelets in road_to_lanelets have corresponding entries in lanelet_to_road
    for road_id, lanelet_ids in mapping.road_to_lanelets.items():
        for lanelet_id in lanelet_ids:
            assert mapping.get_road_for_lanelet(lanelet_id) == road_id

    # Verify all lanelets in lanelet_to_road appear in road_to_lanelets
    for lanelet_id, road_id in mapping.lanelet_to_road.items():
        assert lanelet_id in mapping.get_lanelets_for_road(road_id)


def test_empty_mapping():
    """Test empty mapping."""
    mapping = RoadLaneletMapping(road_to_lanelets={}, lanelet_to_road={})

    assert mapping.get_lanelets_for_road(0) == []
    assert mapping.get_road_for_lanelet(100) is None


def test_single_road_multiple_lanelets():
    """Test mapping with single road containing multiple lanelets."""
    lanelet_ids = [100, 101, 102, 103, 104]
    road_id = 0

    road_to_lanelets = {road_id: lanelet_ids}
    lanelet_to_road = {lid: road_id for lid in lanelet_ids}

    mapping = RoadLaneletMapping(
        road_to_lanelets=road_to_lanelets, lanelet_to_road=lanelet_to_road
    )

    assert mapping.get_lanelets_for_road(road_id) == lanelet_ids
    for lanelet_id in lanelet_ids:
        assert mapping.get_road_for_lanelet(lanelet_id) == road_id


def test_issue_291_diverging_roads_have_no_lane_drop(nishishinjuku_xodr):
    """Diverging lanes must not silently drop their road->road successor link.

    Originally tracked under #291: a single 1->N divergence (Road 185 ->
    186/187/188 in the previously generated XODR) was emitted as one
    multi-lane road with a single road-level successor, silently
    discarding the lanes whose actual routing-graph successors lived on
    other downstream roads.

    The first fix (#291) wrapped the divergence in a synthetic junction.
    The follow-up fix (``specs/2026-05-22-road-successor-mis-merge-design.md``,
    Option A) supersedes that by splitting the upstream group along the
    successor boundary so each split road has one unambiguous downstream
    road and a complete set of ``<lane><link><successor/></link></lane>``
    entries. The contract this regression asserts is therefore the
    user-observable property — no silent lane drops — rather than a
    specific synthesis-vs-split mechanism:

    - Every regular road whose road-level ``<successor>`` points at
      another regular road must give every driving lane a
      ``<lane><link><successor/></link></lane>`` whose target lane id
      exists in the downstream road. ``odrviewer`` logs "No connection
      from rid X lid Y -> rid Z" whenever this fails and ego visibly
      snaps lanes.
    """
    xodr_path = nishishinjuku_xodr
    tree = ET.parse(str(xodr_path))

    def driving_lane_ids(road: ET._Element) -> set[int]:
        return {
            int(lane.get("id"))
            for lane in road.findall("lanes/laneSection//lane")
            if lane.get("type") == "driving"
        }

    def lane_successor_id(road: ET._Element, lane_id: int) -> int | None:
        for lane in road.findall("lanes/laneSection//lane"):
            if int(lane.get("id")) != lane_id:
                continue
            succ = lane.find("link/successor")
            return int(succ.get("id")) if succ is not None else None
        return None

    roads_by_id = {int(r.get("id")): r for r in tree.findall(".//road")}
    lane_drops: list[tuple[int, int, int, str]] = []
    for rid, road in roads_by_id.items():
        succ = road.find("link/successor")
        if succ is None or succ.get("elementType") != "road":
            continue
        sid = int(succ.get("elementId"))
        downstream = roads_by_id.get(sid)
        if downstream is None:
            continue
        dst_lanes = driving_lane_ids(downstream)
        for lid in driving_lane_ids(road):
            target = lane_successor_id(road, lid)
            if target is None:
                lane_drops.append((rid, lid, sid, "NO_LINK"))
            elif target not in dst_lanes:
                lane_drops.append((rid, lid, sid, f"BAD_TARGET({target})"))

    assert not lane_drops, (
        "Diverging lanes silently dropped their road->road successor link "
        "(odrviewer would log 'No connection from rid X lid Y -> rid Z'):\n  "
        + "\n  ".join(
            f"road {rid} lane {lid} -> road {sid}: {reason}"
            for rid, lid, sid, reason in lane_drops
        )
    )


# ---------------------------------------------------------------------------
# End-to-end conversion tests (CLI)
# ---------------------------------------------------------------------------


def test_convert_map_with_parking_lot_emits_parking_road(tmp_path: Path) -> None:
    """Converting a Lanelet2 map containing a parking lot must succeed
    and produce at least one road with a parking lane (P2-1)."""
    fixture = (Path(__file__).parent / "data" / "parking_lot_mini.osm").resolve()
    out = tmp_path / "parking_lot_mini.xodr"

    subprocess.run(
        [
            "uv",
            "run",
            "convert",
            "map=example_mgrs_offset",
            "target=carla",
            f"input_map_path={fixture}",
            f"output_map_path={out}",
        ],
        check=True,
    )

    root = ET.parse(out).getroot()
    parking_lanes = root.findall(".//lane[@type='parking']")
    assert parking_lanes, (
        "conversion of a map with a parking lot should emit at least "
        "one lane[type='parking']"
    )
