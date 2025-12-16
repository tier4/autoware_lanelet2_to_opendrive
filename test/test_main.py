"""Tests for main conversion functionality and RoadLaneletMapping."""

from autoware_lanelet2_to_opendrive.main import RoadLaneletMapping


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
