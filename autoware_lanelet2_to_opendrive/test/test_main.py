"""Tests for main conversion functionality and RoadLaneletMapping."""

import os
import subprocess
import tempfile
from pathlib import Path

import lxml.etree as ET
import pytest

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


def _nishishinjuku_xodr_for_issue_291() -> Path:
    """Build (or reuse) the Nishishinjuku XODR for the Road 185 regression test.

    Mirrors the on-demand build pattern in ``test_junction_endpoint_fidelity``:
    invoke ``uv run convert`` on the bundled OSM fixture and parse the result.
    Skips when the converter or fixture isn't available so the test is
    sandbox-friendly.
    """
    fixture = Path(
        "autoware_lanelet2_to_opendrive/test/data/nishishinjuku.osm"
    ).resolve()
    if not fixture.is_file():
        pytest.skip(f"{fixture} not available; cannot build XODR")

    worker_id = os.environ.get("PYTEST_XDIST_WORKER", "main")
    xodr_path = (
        Path(tempfile.gettempdir()) / f"nishishinjuku_issue_291_{worker_id}.xodr"
    )

    cmd = [
        "uv",
        "run",
        "convert",
        "map=nishishinjuku",
        "target=carla",
        f"input_map_path={fixture}",
        f"output_map_path={xodr_path}",
    ]
    try:
        subprocess.run(cmd, check=True)
    except FileNotFoundError as exc:
        pytest.skip(f"converter unavailable: {exc}")

    if not xodr_path.is_file():
        pytest.fail(f"converter exited successfully but {xodr_path} was not produced")
    return xodr_path


def test_issue_291_road_185_successor_is_synthetic_junction():
    """Road 185 (Nishishinjuku) diverges into 186/187/188 — successor must be a junction."""
    xodr_path = _nishishinjuku_xodr_for_issue_291()
    tree = ET.parse(str(xodr_path))

    road_185 = tree.find(".//road[@id='185']")
    assert road_185 is not None, "Road 185 missing from generated XODR"

    succ = road_185.find("link/successor")
    assert succ is not None, "Road 185 has no <successor> element"
    assert succ.get("elementType") == "junction", (
        f"Road 185 <successor> elementType is {succ.get('elementType')!r}, "
        "expected 'junction' (#291)"
    )

    junction_id = succ.get("elementId")
    junction = tree.find(f".//junction[@id='{junction_id}']")
    assert junction is not None, f"Junction {junction_id} not present"
    connections = junction.findall("connection")
    assert (
        len(connections) == 3
    ), f"Expected 3 connections (one per Road 185 lane), got {len(connections)}"

    sources = {c.find("laneLink").get("from") for c in connections}
    assert sources == {"-1", "-2", "-3"}

    # Verify the lane-to-road mapping: each connecting road must terminate at
    # the expected candidate (-1 -> 186, -2 -> 187, -3 -> 188) (#291 review).
    expected_targets = {("-1", "186"), ("-2", "187"), ("-3", "188")}
    actual_targets = set()
    for connection in connections:
        connecting_road_id = connection.get("connectingRoad")
        lane_link = connection.find("laneLink")
        from_lane = lane_link.get("from")
        cr = tree.find(f".//road[@id='{connecting_road_id}']")
        assert cr is not None, f"Connecting road {connecting_road_id} missing"
        succ = cr.find("link/successor")
        assert (
            succ is not None
        ), f"Connecting road {connecting_road_id} has no <successor>"
        actual_targets.add((from_lane, succ.get("elementId")))
    assert (
        actual_targets == expected_targets
    ), f"Lane-to-road mapping wrong: {actual_targets}, expected {expected_targets}"
