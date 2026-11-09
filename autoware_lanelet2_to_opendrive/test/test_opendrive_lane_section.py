"""Tests for OpenDRIVE lane section functions."""

from autoware_lanelet2_to_opendrive.opendrive.lane_section import LaneSection
from autoware_lanelet2_to_opendrive.opendrive.lane import Lane
from autoware_lanelet2_to_opendrive.opendrive.reference_line import ReferenceLine


def test_construct_lane_section_from_two_lanes(lanelet_map):
    """Test constructing a LaneSection from two adjacent lanelets."""

    # Use two adjacent lanelets
    lanelet_group = [
        lanelet_map.laneletLayer.get(3002094),
        lanelet_map.laneletLayer.get(3002093),
    ]

    lane_section = LaneSection.construct_from_lanelet_groups(
        lanelet_map, lanelet_group, s_offset=0.0
    )

    # Check that lane section was created
    assert lane_section is not None
    assert lane_section.s_offset == 0.0

    # Check center lane (reference line)
    assert lane_section.center_lane is not None
    assert isinstance(lane_section.center_lane, ReferenceLine)
    assert lane_section.center_lane.lane_id == 0

    # Check left and right lanes
    # With 2 lanes, all are right lanes (reference line is leftmost left boundary)
    assert len(lane_section.left_lanes) == 0
    assert len(lane_section.right_lanes) == 2

    # Check lane IDs (negative IDs from left to right: -1, -2)
    assert -1 in lane_section.right_lanes
    assert -2 in lane_section.right_lanes

    # Check that lanes are Lane instances
    assert isinstance(lane_section.right_lanes[-1], Lane)
    assert isinstance(lane_section.right_lanes[-2], Lane)


def test_get_all_lanes(lanelet_map):
    """Test getting all lanes from a LaneSection."""

    lanelet_group = [
        lanelet_map.laneletLayer.get(3002094),
        lanelet_map.laneletLayer.get(3002093),
    ]

    lane_section = LaneSection.construct_from_lanelet_groups(lanelet_map, lanelet_group)

    all_lanes = lane_section.get_all_lanes()

    # Should have 3 lanes total (0 left + 1 center + 2 right)
    assert len(all_lanes) == 3

    # Check order: left lanes, center, right lanes (but no left lanes in new logic)
    lane_ids = [lane.lane_id for lane in all_lanes]
    assert lane_ids == [0, -1, -2]

    # Verify center lane type is "none"
    xml_element = lane_section.to_xml()
    center_section = xml_element.find("center")
    assert center_section is not None
    center_lane = center_section.find("lane")
    assert center_lane is not None
    assert center_lane.get("type") == "none"


def test_single_lane_section_has_one_right_lane_and_no_lane_offset(lanelet_map):
    """A one-lanelet section gets lane -1 and emits no ``<laneOffset>``.

    The old name claimed the section was built *with* a laneOffset. It never
    was: the reference line is the lanelet's own left boundary, so the centre
    lane sits on it and the offset is zero by construction. Nothing in the
    converter has ever produced a non-zero one.
    """

    # Use a single lanelet
    single_lanelet = lanelet_map.laneletLayer.get(3002094)
    lanelet_group = [single_lanelet]

    lane_section = LaneSection.construct_from_lanelet_groups(
        lanelet_map, lanelet_group, s_offset=0.0
    )

    # Check that lane section was created
    assert lane_section is not None
    assert lane_section.s_offset == 0.0

    # Check center lane (reference line)
    assert lane_section.center_lane is not None
    assert isinstance(lane_section.center_lane, ReferenceLine)
    assert lane_section.center_lane.lane_id == 0

    # For single lane: should have 0 left lanes and 1 right lane
    assert len(lane_section.left_lanes) == 0
    assert len(lane_section.right_lanes) == 1

    # Check lane ID for single lane
    assert -1 in lane_section.right_lanes

    # No <laneOffset>: the reference line is the lanelet's left boundary, so
    # the centre lane lies on it. (In OpenDRIVE <laneOffset> is a child of
    # <lanes>, not of <laneSection>, so a section could not carry one anyway.)
    xml_element = lane_section.to_xml()
    assert xml_element.find("laneOffset") is None
    assert xml_element.find("lanes") is None


def test_lane_section_rht_explicit(lanelet_map):
    """Test LaneSection construction with explicit RHT produces right lanes with negative IDs."""
    lanelet_group = [
        lanelet_map.laneletLayer.get(3002094),
        lanelet_map.laneletLayer.get(3002093),
    ]

    lane_section = LaneSection.construct_from_lanelet_groups(
        lanelet_map, lanelet_group, s_offset=0.0, traffic_rule="RHT"
    )

    # RHT should produce right lanes with negative IDs
    assert len(lane_section.left_lanes) == 0
    assert len(lane_section.right_lanes) == 2

    # Check lane IDs are negative (-1, -2 from left to right)
    assert -1 in lane_section.right_lanes
    assert -2 in lane_section.right_lanes

    # Verify lanes are ordered correctly
    all_lanes = lane_section.get_all_lanes()
    lane_ids = [lane.lane_id for lane in all_lanes]
    assert lane_ids == [0, -1, -2]


def test_lane_section_lht(lanelet_map):
    """Test LaneSection construction with LHT produces left lanes with positive IDs."""
    lanelet_group = [
        lanelet_map.laneletLayer.get(3002094),
        lanelet_map.laneletLayer.get(3002093),
    ]

    lane_section = LaneSection.construct_from_lanelet_groups(
        lanelet_map, lanelet_group, s_offset=0.0, traffic_rule="LHT"
    )

    # LHT should produce left lanes with positive IDs (+1, +2, ...)
    assert len(lane_section.right_lanes) == 0
    assert len(lane_section.left_lanes) == 2

    # Check lane IDs are positive (+1, +2 from innermost to outermost)
    assert 1 in lane_section.left_lanes
    assert 2 in lane_section.left_lanes

    # Verify lanes are ordered correctly: left lanes first (ascending), then center
    all_lanes = lane_section.get_all_lanes()
    lane_ids = [lane.lane_id for lane in all_lanes]
    assert lane_ids == [1, 2, 0]


def test_lane_section_traffic_rule_default(lanelet_map):
    """Test LaneSection construction without traffic_rule defaults to RHT."""
    lanelet_group = [
        lanelet_map.laneletLayer.get(3002094),
        lanelet_map.laneletLayer.get(3002093),
    ]

    # Omit traffic_rule parameter
    lane_section = LaneSection.construct_from_lanelet_groups(
        lanelet_map, lanelet_group, s_offset=0.0
    )

    # Should default to RHT behavior (right lanes with negative IDs)
    assert len(lane_section.left_lanes) == 0
    assert len(lane_section.right_lanes) == 2
    assert -1 in lane_section.right_lanes
    assert -2 in lane_section.right_lanes


def test_lane_section_invalid_traffic_rule(lanelet_map):
    """Test LaneSection construction with invalid traffic_rule raises ValueError."""
    import pytest

    lanelet_group = [
        lanelet_map.laneletLayer.get(3002094),
        lanelet_map.laneletLayer.get(3002093),
    ]

    # Test invalid traffic_rule
    with pytest.raises(
        ValueError, match="Invalid traffic_rule.*Must be 'RHT' or 'LHT'"
    ):
        LaneSection.construct_from_lanelet_groups(
            lanelet_map, lanelet_group, s_offset=0.0, traffic_rule="INVALID"
        )
