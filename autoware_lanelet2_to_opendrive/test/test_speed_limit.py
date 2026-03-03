"""Tests for speed limit extraction and conversion."""

import lanelet2
from autoware_lanelet2_to_opendrive.opendrive.road import Road
from autoware_lanelet2_to_opendrive.opendrive.opendrive_dataclass import (
    LaneSpeed,
    SpeedUnit,
    RoadType,
    RoadTypeSpeed,
    RoadTypeDefinition,
)


def test_lane_speed_xml():
    """Test LaneSpeed XML generation."""
    lane_speed = LaneSpeed(s_offset=0.0, max=50.0, unit=SpeedUnit.KMH)
    xml = lane_speed.to_xml()

    assert xml.tag == "speed"
    assert xml.get("sOffset") == "0.0"
    assert xml.get("max") == "50"
    assert xml.get("unit") == "km/h"


def test_road_type_speed_xml():
    """Test RoadTypeSpeed XML generation."""
    road_type_speed = RoadTypeSpeed(max=60.0, unit=SpeedUnit.KMH)
    xml = road_type_speed.to_xml()

    assert xml.tag == "speed"
    assert xml.get("max") == "60"
    assert xml.get("unit") == "km/h"


def test_road_type_definition_xml():
    """Test RoadTypeDefinition XML generation."""
    speed = RoadTypeSpeed(max=50.0, unit=SpeedUnit.KMH)
    road_type_def = RoadTypeDefinition(s=0.0, type=RoadType.TOWN, speed=speed)
    xml = road_type_def.to_xml()

    assert xml.tag == "type"
    assert xml.get("s") == "0.0"
    assert xml.get("type") == "town"

    # Check that speed element is present
    speed_elem = xml.find("speed")
    assert speed_elem is not None
    assert speed_elem.get("max") == "50"
    assert speed_elem.get("unit") == "km/h"


def test_road_type_definition_without_speed():
    """Test RoadTypeDefinition XML generation without speed."""
    road_type_def = RoadTypeDefinition(s=0.0, type=RoadType.UNKNOWN, speed=None)
    xml = road_type_def.to_xml()

    assert xml.tag == "type"
    assert xml.get("s") == "0.0"
    assert xml.get("type") == "unknown"

    # Check that speed element is not present
    speed_elem = xml.find("speed")
    assert speed_elem is None


def test_extract_road_types_with_speed_limit():
    """Test _extract_road_types_from_lanelets with speed limit."""
    # Create a mock lanelet with speed_limit attribute
    left_points = [
        lanelet2.core.Point3d(1, 0, 0, 0),
        lanelet2.core.Point3d(2, 10, 0, 0),
    ]
    right_points = [
        lanelet2.core.Point3d(3, 0, 2, 0),
        lanelet2.core.Point3d(4, 10, 2, 0),
    ]

    left_bound = lanelet2.core.LineString3d(1, left_points)
    right_bound = lanelet2.core.LineString3d(2, right_points)
    lanelet = lanelet2.core.Lanelet(100, left_bound, right_bound)

    # Add speed_limit attribute
    lanelet.attributes["speed_limit"] = "50"
    lanelet.attributes["location"] = "urban"

    # Extract road types
    road_types = Road._extract_road_types_from_lanelets([lanelet])

    assert road_types is not None
    assert len(road_types) == 1

    road_type = road_types[0]
    assert road_type.s == 0.0
    assert road_type.type == RoadType.TOWN
    assert road_type.speed is not None
    assert road_type.speed.max == 50.0
    assert road_type.speed.unit == SpeedUnit.KMH


def test_extract_road_types_without_speed_limit():
    """Test _extract_road_types_from_lanelets without speed limit."""
    # Create a mock lanelet without speed_limit attribute
    left_points = [
        lanelet2.core.Point3d(1, 0, 0, 0),
        lanelet2.core.Point3d(2, 10, 0, 0),
    ]
    right_points = [
        lanelet2.core.Point3d(3, 0, 2, 0),
        lanelet2.core.Point3d(4, 10, 2, 0),
    ]

    left_bound = lanelet2.core.LineString3d(1, left_points)
    right_bound = lanelet2.core.LineString3d(2, right_points)
    lanelet = lanelet2.core.Lanelet(100, left_bound, right_bound)

    lanelet.attributes["location"] = "urban"

    # Extract road types
    road_types = Road._extract_road_types_from_lanelets([lanelet])

    assert road_types is not None
    assert len(road_types) == 1

    road_type = road_types[0]
    assert road_type.s == 0.0
    assert road_type.type == RoadType.TOWN
    assert road_type.speed is None


def test_extract_road_types_infer_from_speed():
    """Test road type inference from speed limit."""
    # Create a mock lanelet with various speed limits
    test_cases = [
        (5, RoadType.LOW_SPEED),
        (50, RoadType.TOWN),
        (80, RoadType.RURAL),
        (120, RoadType.MOTORWAY),
    ]

    for speed_limit, expected_type in test_cases:
        left_points = [
            lanelet2.core.Point3d(1, 0, 0, 0),
            lanelet2.core.Point3d(2, 10, 0, 0),
        ]
        right_points = [
            lanelet2.core.Point3d(3, 0, 2, 0),
            lanelet2.core.Point3d(4, 10, 2, 0),
        ]

        left_bound = lanelet2.core.LineString3d(1, left_points)
        right_bound = lanelet2.core.LineString3d(2, right_points)
        lanelet = lanelet2.core.Lanelet(100, left_bound, right_bound)

        lanelet.attributes["speed_limit"] = str(speed_limit)
        # No location attribute - should infer from speed

        road_types = Road._extract_road_types_from_lanelets([lanelet])

        assert road_types is not None
        assert len(road_types) == 1
        assert road_types[0].type == expected_type
        assert road_types[0].speed.max == speed_limit
