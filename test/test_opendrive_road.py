"""Tests for OpenDRIVE lane section functions."""

from autoware_lanelet2_to_opendrive.opendrive.road import Road


def test_construct_road_from_two_lanes(lanelet_map):
    """Test constructing a Road from two adjacent lanelets."""

    # Use two adjacent lanelets
    lanelet_group = [
        lanelet_map.laneletLayer.get(3002094),
        lanelet_map.laneletLayer.get(3002093),
    ]

    road = Road.construct_from_lanelet_groups(  # noqa F841
        lanelet_map, lanelet_group, road_id=0, s_offset=0.0
    )

    # from lxml import etree
    # print("")
    # print(etree.tostring(road.to_xml(), pretty_print=True).decode())


def test_elevation_profile_extraction(lanelet_map):
    """Test that elevation profile is extracted from lanelets."""

    # Use two adjacent lanelets
    lanelet_group = [
        lanelet_map.laneletLayer.get(3002094),
        lanelet_map.laneletLayer.get(3002093),
    ]

    road = Road.construct_from_lanelet_groups(
        lanelet_map, lanelet_group, road_id=0, s_offset=0.0
    )

    # Verify elevation profile is not None
    assert road.elevation_profile is not None, "Elevation profile should not be None"

    # Verify elevation profile has elevations
    assert (
        len(road.elevation_profile.elevations) > 0
    ), "Elevation profile should contain at least one elevation segment"

    # Verify first elevation starts at s=0
    first_elevation = road.elevation_profile.elevations[0]
    assert first_elevation.s == 0.0, "First elevation should start at s=0"

    # Verify all elevations have valid polynomial coefficients
    for elevation in road.elevation_profile.elevations:
        assert isinstance(
            elevation.a, (int, float)
        ), "Coefficient 'a' should be numeric"
        assert isinstance(
            elevation.b, (int, float)
        ), "Coefficient 'b' should be numeric"
        assert isinstance(
            elevation.c, (int, float)
        ), "Coefficient 'c' should be numeric"
        assert isinstance(
            elevation.d, (int, float)
        ), "Coefficient 'd' should be numeric"


def test_elevation_profile_xml_output(lanelet_map):
    """Test that elevation profile can be converted to XML."""

    # Use two adjacent lanelets
    lanelet_group = [
        lanelet_map.laneletLayer.get(3002094),
        lanelet_map.laneletLayer.get(3002093),
    ]

    road = Road.construct_from_lanelet_groups(
        lanelet_map, lanelet_group, road_id=0, s_offset=0.0
    )

    # Convert road to XML
    road_xml = road.to_xml()

    # Find elevationProfile element
    elevation_profile_elem = road_xml.find("elevationProfile")
    assert (
        elevation_profile_elem is not None
    ), "Road XML should contain elevationProfile element"

    # Find elevation elements
    elevation_elems = elevation_profile_elem.findall("elevation")
    assert (
        len(elevation_elems) > 0
    ), "elevationProfile should contain at least one elevation element"

    # Verify XML attributes
    first_elevation_elem = elevation_elems[0]
    assert (
        "s" in first_elevation_elem.attrib
    ), "Elevation element should have 's' attribute"
    assert (
        "a" in first_elevation_elem.attrib
    ), "Elevation element should have 'a' attribute"
    assert (
        "b" in first_elevation_elem.attrib
    ), "Elevation element should have 'b' attribute"
    assert (
        "c" in first_elevation_elem.attrib
    ), "Elevation element should have 'c' attribute"
    assert (
        "d" in first_elevation_elem.attrib
    ), "Elevation element should have 'd' attribute"


def test_road_construction_rht(lanelet_map):
    """Test Road construction with RHT creates correct lane structure."""
    lanelet_group = [
        lanelet_map.laneletLayer.get(3002094),
        lanelet_map.laneletLayer.get(3002093),
    ]

    road = Road.construct_from_lanelet_groups(
        lanelet_map, lanelet_group, road_id=0, s_offset=0.0, traffic_rule="RHT"
    )

    # Verify road was created
    assert road is not None
    assert road.id == 0

    # Verify road has lanes object
    assert road.lanes is not None
    assert len(road.lanes.lane_sections) > 0

    # Check first lane section has correct structure for RHT
    lane_section = road.lanes.lane_sections[0]
    assert len(lane_section.left_lanes) == 0
    assert len(lane_section.right_lanes) == 2

    # Check lane IDs are negative for RHT
    assert -1 in lane_section.right_lanes
    assert -2 in lane_section.right_lanes


def test_road_construction_lht(lanelet_map):
    """Test Road construction with LHT creates correct lane structure."""
    lanelet_group = [
        lanelet_map.laneletLayer.get(3002094),
        lanelet_map.laneletLayer.get(3002093),
    ]

    road = Road.construct_from_lanelet_groups(
        lanelet_map, lanelet_group, road_id=0, s_offset=0.0, traffic_rule="LHT"
    )

    # Verify road was created
    assert road is not None
    assert road.id == 0

    # Verify road has lanes object
    assert road.lanes is not None
    assert len(road.lanes.lane_sections) > 0

    # Check first lane section has correct structure for LHT (same as RHT)
    lane_section = road.lanes.lane_sections[0]
    assert len(lane_section.left_lanes) == 0
    assert len(lane_section.right_lanes) == 2

    # Check lane IDs are negative for LHT (same structure as RHT)
    assert -1 in lane_section.right_lanes
    assert -2 in lane_section.right_lanes
