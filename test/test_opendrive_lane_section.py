"""Tests for OpenDRIVE lane section functions."""

from pathlib import Path
import lanelet2
from autoware_lanelet2_extension_python.projection import MGRSProjector
from autoware_lanelet2_to_opendrive.opendrive.lane_section import LaneSection
from autoware_lanelet2_to_opendrive.opendrive.lane import Lane
from autoware_lanelet2_to_opendrive.opendrive.reference_line import ReferenceLine


def load_test_map():
    """Load the test lanelet2 map."""
    test_data_path = Path(__file__).parent / "data" / "lanelet2_map.osm"
    projector = MGRSProjector(
        lanelet2.io.Origin(35.23, 139.16)
    )  # MGRS origin for Tokyo area (54SUE)
    return lanelet2.io.load(str(test_data_path), projector)


def test_construct_lane_section_from_two_lanes():
    """Test constructing a LaneSection from two adjacent lanelets."""
    lanelet_map = load_test_map()

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
    # With 2 lanes (even), we should have 1 left and 1 right
    assert len(lane_section.left_lanes) == 1
    assert len(lane_section.right_lanes) == 1

    # Check lane IDs
    assert 1 in lane_section.left_lanes
    assert -1 in lane_section.right_lanes

    # Check that lanes are Lane instances
    assert isinstance(lane_section.left_lanes[1], Lane)
    assert isinstance(lane_section.right_lanes[-1], Lane)


def test_lane_section_to_standard():
    """Test converting LaneSection to scenariogeneration standard."""
    lanelet_map = load_test_map()

    lanelet_group = [
        lanelet_map.laneletLayer.get(3002094),
        lanelet_map.laneletLayer.get(3002093),
    ]

    lane_section = LaneSection.construct_from_lanelet_groups(lanelet_map, lanelet_group)

    # Convert to standard lane section
    standard_section = lane_section.to_standard_lane_section()

    # Check that conversion succeeded
    assert standard_section is not None

    # Check s_offset
    assert standard_section.s == 0.0


def test_get_all_lanes():
    """Test getting all lanes from a LaneSection."""
    lanelet_map = load_test_map()

    lanelet_group = [
        lanelet_map.laneletLayer.get(3002094),
        lanelet_map.laneletLayer.get(3002093),
    ]

    lane_section = LaneSection.construct_from_lanelet_groups(lanelet_map, lanelet_group)

    all_lanes = lane_section.get_all_lanes()

    # Should have 3 lanes total (1 left + 1 center + 1 right)
    assert len(all_lanes) == 3

    # Check order: left lanes, center, right lanes
    lane_ids = [lane.lane_id for lane in all_lanes]
    assert lane_ids == [1, 0, -1]

    # Verify center lane type is "none"
    xml_element = lane_section.to_xml()
    center_section = xml_element.find("center")
    assert center_section is not None
    center_lane = center_section.find("lane")
    assert center_lane is not None
    assert center_lane.get("type") == "none"


def test_single_lane_section_with_lane_offset():
    """Test constructing a LaneSection from a single lanelet with laneOffset."""
    lanelet_map = load_test_map()

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

    # Check that laneOffset was set
    assert lane_section.lane_offset is not None
    assert lane_section.lane_offset["s"] == 0.0
    assert lane_section.lane_offset["b"] == 0.0
    assert lane_section.lane_offset["c"] == 0.0
    assert lane_section.lane_offset["d"] == 0.0
    # Width should be calculated from the lanelet width, so check it's a reasonable value
    assert 1.0 <= lane_section.lane_offset["a"] <= 3.0  # Reasonable lane offset range

    # Test XML output includes laneOffset
    xml_element = lane_section.to_xml()
    offset_element = xml_element.find("laneOffset")
    assert offset_element is not None
    assert offset_element.get("s") == "0.0"
    assert offset_element.get("b") == "0.0"
    assert offset_element.get("c") == "0.0"
    assert offset_element.get("d") == "0.0"
    # Check that 'a' value is reasonable (calculated from lanelet width)
    offset_a = float(offset_element.get("a"))
    assert 1.0 <= offset_a <= 3.0
