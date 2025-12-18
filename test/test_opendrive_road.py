"""Tests for OpenDRIVE lane section functions."""

from pathlib import Path
import lanelet2
from autoware_lanelet2_extension_python.projection import MGRSProjector
from autoware_lanelet2_to_opendrive.opendrive.road import Road


def load_test_map():
    """Load the test lanelet2 map."""
    test_data_path = Path(__file__).parent / "data" / "lanelet2_map.osm"
    projector = MGRSProjector(
        lanelet2.io.Origin(35.23, 139.16)
    )  # MGRS origin for Tokyo area (54SUE)
    return lanelet2.io.load(str(test_data_path), projector)


def test_construct_road_from_two_lanes():
    """Test constructing a Road from two adjacent lanelets."""
    lanelet_map = load_test_map()

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


def test_elevation_profile_extraction():
    """Test that elevation profile is extracted from lanelets."""
    lanelet_map = load_test_map()

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


def test_elevation_profile_xml_output():
    """Test that elevation profile can be converted to XML."""
    lanelet_map = load_test_map()

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
