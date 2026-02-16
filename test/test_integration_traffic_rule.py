"""Integration tests for RHT/LHT traffic rule support."""

import pytest
from autoware_lanelet2_to_opendrive.opendrive.road import Road
from autoware_lanelet2_to_opendrive.opendrive.reference_line import ReferenceLine


def test_full_conversion_rht(lanelet_map):
    """Test full conversion pipeline with RHT, verify XML structure."""
    lanelet_group = [
        lanelet_map.laneletLayer.get(3002094),
        lanelet_map.laneletLayer.get(3002093),
    ]

    road = Road.construct_from_lanelet_groups(
        lanelet_map, lanelet_group, road_id=1, s_offset=0.0, traffic_rule="RHT"
    )

    # Convert to XML
    road_xml = road.to_xml()

    # Verify road structure
    assert road_xml.tag == "road"
    assert road_xml.get("id") == "1"

    # Verify lane sections exist
    lanes_elem = road_xml.find("lanes")
    assert lanes_elem is not None

    lane_section_elems = lanes_elem.findall("laneSection")
    assert len(lane_section_elems) > 0

    # Check first lane section
    first_section = lane_section_elems[0]

    # RHT should have right lanes only
    right_elem = first_section.find("right")
    assert right_elem is not None
    right_lane_elems = right_elem.findall("lane")
    assert len(right_lane_elems) == 2

    # Check lane IDs are negative
    right_lane_ids = [int(lane.get("id")) for lane in right_lane_elems]
    assert all(lane_id < 0 for lane_id in right_lane_ids)
    assert -1 in right_lane_ids
    assert -2 in right_lane_ids

    # RHT should have no left lanes
    left_elem = first_section.find("left")
    if left_elem is not None:
        left_lane_elems = left_elem.findall("lane")
        assert len(left_lane_elems) == 0


def test_full_conversion_lht(lanelet_map):
    """Test full conversion pipeline with LHT, verify XML structure."""
    lanelet_group = [
        lanelet_map.laneletLayer.get(3002094),
        lanelet_map.laneletLayer.get(3002093),
    ]

    road = Road.construct_from_lanelet_groups(
        lanelet_map, lanelet_group, road_id=1, s_offset=0.0, traffic_rule="LHT"
    )

    # Convert to XML
    road_xml = road.to_xml()

    # Verify road structure
    assert road_xml.tag == "road"
    assert road_xml.get("id") == "1"

    # Verify lane sections exist
    lanes_elem = road_xml.find("lanes")
    assert lanes_elem is not None

    lane_section_elems = lanes_elem.findall("laneSection")
    assert len(lane_section_elems) > 0

    # Check first lane section
    first_section = lane_section_elems[0]

    # LHT should have right lanes only
    right_elem = first_section.find("right")
    assert right_elem is not None
    right_lane_elems = right_elem.findall("lane")
    assert len(right_lane_elems) == 2

    # Check lane IDs are negative
    right_lane_ids = [int(lane.get("id")) for lane in right_lane_elems]
    assert all(lane_id < 0 for lane_id in right_lane_ids)
    assert -1 in right_lane_ids
    assert -2 in right_lane_ids

    # LHT should have no left lanes
    left_elem = first_section.find("left")
    if left_elem is not None:
        left_lane_elems = left_elem.findall("lane")
        assert len(left_lane_elems) == 0


def test_reference_line_geometry_rht(lanelet_map):
    """Test RHT reference line uses leftmost left boundary."""
    lanelet_group = [
        lanelet_map.laneletLayer.get(3002094),
        lanelet_map.laneletLayer.get(3002093),
    ]

    reference_line = ReferenceLine.construct_from_lanelet_groups(
        lanelet_map, lanelet_group, traffic_rule="RHT"
    )

    # Verify reference line was created
    assert reference_line is not None
    assert reference_line.centerline_2d is not None

    # Verify reference line has valid geometry
    assert reference_line.centerline_2d.total_length > 0

    # Verify elevation offset is set
    assert reference_line.elevation_offset is not None
    assert isinstance(reference_line.elevation_offset, float)


def test_reference_line_geometry_lht(lanelet_map):
    """Test LHT reference line uses rightmost right boundary."""
    lanelet_group = [
        lanelet_map.laneletLayer.get(3002094),
        lanelet_map.laneletLayer.get(3002093),
    ]

    reference_line = ReferenceLine.construct_from_lanelet_groups(
        lanelet_map, lanelet_group, traffic_rule="LHT"
    )

    # Verify reference line was created
    assert reference_line is not None
    assert reference_line.centerline_2d is not None

    # Verify reference line has valid geometry
    assert reference_line.centerline_2d.total_length > 0

    # Verify elevation offset is set
    assert reference_line.elevation_offset is not None
    assert isinstance(reference_line.elevation_offset, float)


def test_reference_line_invalid_traffic_rule(lanelet_map):
    """Test ReferenceLine construction with invalid traffic_rule raises ValueError."""
    lanelet_group = [
        lanelet_map.laneletLayer.get(3002094),
        lanelet_map.laneletLayer.get(3002093),
    ]

    # Test invalid traffic_rule
    with pytest.raises(
        ValueError, match="Invalid traffic_rule.*Must be 'RHT' or 'LHT'"
    ):
        ReferenceLine.construct_from_lanelet_groups(
            lanelet_map, lanelet_group, traffic_rule="INVALID"
        )


def test_reference_line_case_insensitive(lanelet_map):
    """Test traffic_rule is case-insensitive."""
    lanelet_group = [
        lanelet_map.laneletLayer.get(3002094),
        lanelet_map.laneletLayer.get(3002093),
    ]

    # Test lowercase
    reference_line_lower = ReferenceLine.construct_from_lanelet_groups(
        lanelet_map, lanelet_group, traffic_rule="rht"
    )
    assert reference_line_lower is not None

    # Test mixed case
    reference_line_mixed = ReferenceLine.construct_from_lanelet_groups(
        lanelet_map, lanelet_group, traffic_rule="RhT"
    )
    assert reference_line_mixed is not None
