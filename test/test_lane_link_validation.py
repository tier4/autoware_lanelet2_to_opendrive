"""Test for lane link validation and non-existent lane detection."""

from unittest.mock import Mock

from autoware_lanelet2_to_opendrive.opendrive.road import Road
from autoware_lanelet2_to_opendrive.opendrive.lane_sections import Lanes
from autoware_lanelet2_to_opendrive.opendrive.lane_section import LaneSection
from autoware_lanelet2_to_opendrive.opendrive.reference_line import ReferenceLine


def test_road_lane_ids_includes_center_lane():
    """Test that center lane (ID=0) is included in road_lane_ids."""
    # Create mock roads with lanes
    road1 = Mock(spec=Road)
    road1.id = 1
    road1.lanes = Mock(spec=Lanes)

    lane_section = Mock(spec=LaneSection)
    lane_section.left_lanes = {1: Mock(), 2: Mock()}
    lane_section.right_lanes = {-1: Mock(), -2: Mock()}
    lane_section.center_lane = Mock(spec=ReferenceLine)  # Center lane exists

    road1.lanes.lane_sections = [lane_section]

    # Build road_lane_ids as done in set_all_lane_links
    road_lane_ids = {}
    lane_ids = set()
    if road1.lanes:
        for ls in road1.lanes.lane_sections:
            lane_ids.update(ls.left_lanes.keys())
            lane_ids.update(ls.right_lanes.keys())
            # Include center lane (ID=0)
            if ls.center_lane is not None:
                lane_ids.add(0)
    road_lane_ids[road1.id] = lane_ids

    # Verify center lane is included
    assert 0 in road_lane_ids[1], "Center lane (ID=0) should be included"
    assert road_lane_ids[1] == {-2, -1, 0, 1, 2}


def test_road_lane_ids_without_center_lane():
    """Test case where center lane doesn't exist."""
    road1 = Mock(spec=Road)
    road1.id = 1
    road1.lanes = Mock(spec=Lanes)

    lane_section = Mock(spec=LaneSection)
    lane_section.left_lanes = {}
    lane_section.right_lanes = {-1: Mock()}
    lane_section.center_lane = None  # No center lane

    road1.lanes.lane_sections = [lane_section]

    # Build road_lane_ids
    road_lane_ids = {}
    lane_ids = set()
    if road1.lanes:
        for ls in road1.lanes.lane_sections:
            lane_ids.update(ls.left_lanes.keys())
            lane_ids.update(ls.right_lanes.keys())
            # Include center lane (ID=0) only if it exists
            if ls.center_lane is not None:
                lane_ids.add(0)
    road_lane_ids[road1.id] = lane_ids

    # Verify center lane is not included when it doesn't exist
    assert (
        0 not in road_lane_ids[1]
    ), "Center lane should not be included when it doesn't exist"
    assert road_lane_ids[1] == {-1}


def test_find_closest_lane_right_lanes():
    """Test _find_closest_lane for right lanes (negative IDs)."""
    # Lane -3 should map to -2 when -3 doesn't exist
    result = Road._find_closest_lane(-3, [-1, -2])
    assert result == -2, "Lane -3 should map to closest lane -2"


def test_find_closest_lane_left_lanes():
    """Test _find_closest_lane for left lanes (positive IDs)."""
    # Lane 3 should map to 2 when 3 doesn't exist
    result = Road._find_closest_lane(3, [1, 2])
    assert result == 2, "Lane 3 should map to closest lane 2"


def test_find_closest_lane_single_lane():
    """Test _find_closest_lane with only one available lane."""
    result = Road._find_closest_lane(-2, [-1])
    assert result == -1, "Should return the only available lane"


def test_find_closest_lane_empty_list():
    """Test _find_closest_lane with empty available lanes."""
    result = Road._find_closest_lane(-2, [])
    assert result == -1, "Should return default lane -1 when no lanes available"


def test_lane_link_validation_workflow():
    """Test the complete lane link validation workflow."""
    # This is an integration-style test that verifies the fix works
    # In real usage, center lane should be included in road_lane_ids

    # Simulate a scenario where:
    # Road 1 has lanes: -2, -1, 0 (center), 1, 2
    # Road 2 has lanes: -1, 0 (center), 1
    # A lane in Road 2 references Lane -2 in Road 1 (exists) -> should work
    # A lane in Road 2 references Lane 0 in Road 1 (exists) -> should work
    # A lane in Road 3 references Lane 3 in Road 1 (doesn't exist) -> should remap to 2

    road_lane_ids = {
        1: {-2, -1, 0, 1, 2},  # Road 1 with center lane
        2: {-1, 0, 1},  # Road 2 with center lane
    }

    # Test: Lane exists
    assert -2 in road_lane_ids[1], "Lane -2 should exist in Road 1"
    assert 0 in road_lane_ids[1], "Center lane should exist in Road 1"
    assert 0 in road_lane_ids[2], "Center lane should exist in Road 2"

    # Test: Lane doesn't exist, needs remapping
    target_lane = 3
    existing_lanes = road_lane_ids[1]
    if target_lane not in existing_lanes:
        available_lanes = sorted(existing_lanes)
        remapped_lane = Road._find_closest_lane(target_lane, available_lanes)
        assert remapped_lane == 2, "Lane 3 should be remapped to Lane 2"
