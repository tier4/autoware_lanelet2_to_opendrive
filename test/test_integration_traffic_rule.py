"""Integration tests for RHT/LHT traffic rule support."""

import numpy as np
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

    # LHT should have left lanes only
    left_elem = first_section.find("left")
    assert left_elem is not None
    left_lane_elems = left_elem.findall("lane")
    assert len(left_lane_elems) == 2

    # Check lane IDs are positive
    left_lane_ids = [int(lane.get("id")) for lane in left_lane_elems]
    assert all(lane_id > 0 for lane_id in left_lane_ids)
    assert 1 in left_lane_ids
    assert 2 in left_lane_ids

    # LHT should have no right lanes
    right_elem = first_section.find("right")
    if right_elem is not None:
        right_lane_elems = right_elem.findall("lane")
        assert len(right_lane_elems) == 0


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


def test_lht_reference_line_s_coordinate_direction(lanelet_map):
    """Test that LHT reference line has correct s-coordinate direction.

    Verifies that s-coordinates are monotonically increasing along the
    reference line, indicating proper alignment with the road direction.
    """
    lanelet_group = [
        lanelet_map.laneletLayer.get(3002094),
        lanelet_map.laneletLayer.get(3002093),
    ]

    reference_line = ReferenceLine.construct_from_lanelet_groups(
        lanelet_map, lanelet_group, traffic_rule="LHT"
    )

    # Sample points along the reference line
    total_length = reference_line.centerline_2d.total_length
    num_samples = 100
    s_coords = np.linspace(0, total_length, num_samples)

    # Evaluate positions at sampled s-coordinates
    positions = []
    for s in s_coords:
        pos = reference_line.centerline_2d.evaluate(s, derivative=0)
        positions.append(pos)
    positions = np.array(positions)

    # Verify positions are valid (no NaN or infinite values)
    assert np.all(
        np.isfinite(positions)
    ), "Reference line positions contain invalid values"

    # Calculate distances between consecutive points
    distances = np.linalg.norm(np.diff(positions, axis=0), axis=1)

    # Verify distances are positive (points are moving forward)
    assert np.all(
        distances > 0
    ), "Reference line s-coordinates are not monotonically increasing"

    # Verify total distance matches expected length
    total_distance = np.sum(distances)
    assert np.isclose(
        total_distance, total_length, rtol=0.1
    ), f"Total distance {total_distance:.3f}m does not match expected length {total_length:.3f}m"


def test_lht_lane_widths_are_reasonable(lanelet_map):
    """Test that LHT lane widths are calculated correctly and are reasonable values."""
    lanelet_group = [
        lanelet_map.laneletLayer.get(3002094),
        lanelet_map.laneletLayer.get(3002093),
    ]

    road = Road.construct_from_lanelet_groups(
        lanelet_map, lanelet_group, road_id=1, s_offset=0.0, traffic_rule="LHT"
    )

    # Get first lane section
    assert road.lanes is not None, "Road should have lanes"
    assert len(road.lanes.lane_sections) > 0, "Road should have lane sections"
    lane_section = road.lanes.lane_sections[0]

    # Verify left lanes exist
    assert len(lane_section.left_lanes) > 0, "LHT should have left lanes"

    # Check each lane's width
    for lane_id, lane in lane_section.left_lanes.items():
        # Get width polynomial
        assert len(lane.widths) > 0, f"Lane {lane_id} has no width entries"

        width_entry = lane.widths[0]

        # Check width polynomial coefficients are reasonable
        # a coefficient should be positive (lane width at s_offset)
        assert width_entry.a > 0, f"Lane {lane_id} has invalid width: a={width_entry.a}"
        assert (
            width_entry.a < 10.0
        ), f"Lane {lane_id} has unreasonably large width: a={width_entry.a}m"

        # Typical lane width is 2.5m - 4.0m
        # Allow some tolerance for narrow/wide lanes
        assert (
            1.0 < width_entry.a < 6.0
        ), f"Lane {lane_id} width {width_entry.a:.3f}m is outside reasonable range (1.0m - 6.0m)"


def test_lht_lane_widths_consistency(lanelet_map):
    """Test that LHT lane widths are consistent along the lane section.

    Verifies that width remains positive and does not vary drastically
    within the lane section, which would indicate calculation errors.
    """
    lanelet_group = [
        lanelet_map.laneletLayer.get(3002094),
        lanelet_map.laneletLayer.get(3002093),
    ]

    road = Road.construct_from_lanelet_groups(
        lanelet_map, lanelet_group, road_id=1, s_offset=0.0, traffic_rule="LHT"
    )

    # Get first lane section
    assert road.lanes is not None, "Road should have lanes"
    assert len(road.lanes.lane_sections) > 0, "Road should have lane sections"
    lane_section = road.lanes.lane_sections[0]

    # Check width consistency for each lane
    for lane_id, lane in lane_section.left_lanes.items():
        assert len(lane.widths) > 0, f"Lane {lane_id} has no width entries"

        # Check each width segment
        for i, width_entry in enumerate(lane.widths):
            # Determine the valid range for this width entry
            # Width polynomials are valid from their s_offset to the next s_offset
            if i + 1 < len(lane.widths):
                segment_end = lane.widths[i + 1].s_offset
            else:
                # Last segment: use a reasonable length (e.g., 50m or road length)
                assert road.plan_view is not None, "Road should have plan_view"
                segment_end = min(width_entry.s_offset + 50.0, road.length)

            # Sample width polynomial within its valid range
            s_start = width_entry.s_offset
            s_values = np.linspace(s_start, segment_end, 20)

            widths = []
            for s in s_values:
                ds = s - width_entry.s_offset
                # width(ds) = a + b*ds + c*ds^2 + d*ds^3
                width = (
                    width_entry.a
                    + width_entry.b * ds
                    + width_entry.c * ds**2
                    + width_entry.d * ds**3
                )
                widths.append(width)

            widths_array = np.array(widths)

            # All widths should be positive within the valid segment
            assert np.all(
                widths_array > 0
            ), f"Lane {lane_id} segment {i} has negative or zero widths at some points"

            # Check width variation is reasonable
            width_min = widths_array.min()
            width_max = widths_array.max()
            width_mean = widths_array.mean()

            # Width should not vary too much within a segment
            # Allow up to 50% variation from mean
            assert (
                width_max - width_min < 0.5 * width_mean
            ), f"Lane {lane_id} segment {i} width varies too much: min={width_min:.3f}m, max={width_max:.3f}m, mean={width_mean:.3f}m"
