"""Integration test for speed limit in OpenDRIVE output."""

import lanelet2
import lxml.etree as ET
from autoware_lanelet2_to_opendrive.opendrive.road import Road
from autoware_lanelet2_to_opendrive.opendrive.lane import Lane
from autoware_lanelet2_to_opendrive.opendrive.opendrive_dataclass import (
    OpenDRIVE,
    Header,
    RoadType,
)


def test_road_xml_includes_speed_limit():
    """Test that Road.to_xml() includes speed limit in road type."""
    # Create a mock road with road types
    from autoware_lanelet2_to_opendrive.opendrive.lane_elements import (
        RoadTypeDefinition,
        RoadTypeSpeed,
        SpeedUnit,
    )
    from autoware_lanelet2_to_opendrive.opendrive.geometry import PlanView, Line

    speed = RoadTypeSpeed(max=50.0, unit=SpeedUnit.KMH)
    road_type_def = RoadTypeDefinition(s=0.0, type=RoadType.TOWN, speed=speed)

    # Create simple geometry
    line = Line(s=0.0, x=0.0, y=0.0, hdg=0.0, length=100.0)
    plan_view = PlanView(geometries=[line])

    road = Road(
        id=1,
        name="Test Road",
        length=100.0,
        junction=-1,
        plan_view=plan_view,
        road_types=[road_type_def],
    )

    xml = road.to_xml()

    # Convert to string for easier verification
    xml_str = ET.tostring(xml, encoding="unicode")

    # Check that road type is present
    assert '<type s="0.0" type="town">' in xml_str

    # Check that speed element is present
    assert '<speed max="50" unit="km/h"/>' in xml_str


def test_lane_xml_includes_speed_limit():
    """Test that Lane.to_xml() includes lane speed limit."""
    from autoware_lanelet2_to_opendrive.opendrive.opendrive_dataclass import (
        LaneType,
        LaneSpeed,
        SpeedUnit,
    )

    lane = Lane(lane_id=-1, lane_type=LaneType.DRIVING)

    # Add speed limit
    lane._add_speed(LaneSpeed(s_offset=0.0, max=60.0, unit=SpeedUnit.KMH))

    xml = lane.to_xml()
    xml_str = ET.tostring(xml, encoding="unicode")

    # Check that speed element is present
    assert '<speed sOffset="0.0" max="60" unit="km/h"/>' in xml_str


def test_construct_from_lanelet_with_speed_limit():
    """Test that constructing a Lane from a lanelet with speed_limit extracts it."""
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
    lanelet.attributes["subtype"] = "road"

    # Create a minimal lanelet map
    lanelet_map = lanelet2.core.LaneletMap()
    lanelet_map.add(lanelet)

    # Construct Lane from lanelet
    lane = Lane.construct_from_lanelet(lanelet_map, lanelet)

    # Check that speed limit was extracted
    assert len(lane.speeds) == 1
    assert lane.speeds[0].s_offset == 0.0
    assert lane.speeds[0].max == 50.0

    # Verify XML output contains speed
    xml = lane.to_xml()
    xml_str = ET.tostring(xml, encoding="unicode")
    assert '<speed sOffset="0.0" max="50" unit="km/h"/>' in xml_str


def test_opendrive_xml_with_speed_limits():
    """Test that complete OpenDRIVE XML includes speed limits."""
    from autoware_lanelet2_to_opendrive.opendrive.lane_elements import (
        RoadTypeDefinition,
        RoadTypeSpeed,
        SpeedUnit,
    )
    from autoware_lanelet2_to_opendrive.opendrive.geometry import PlanView, Line
    from autoware_lanelet2_to_opendrive.opendrive.opendrive_dataclass import (
        export_to_xml,
    )

    # Create road with speed limit
    speed = RoadTypeSpeed(max=50.0, unit=SpeedUnit.KMH)
    road_type_def = RoadTypeDefinition(s=0.0, type=RoadType.TOWN, speed=speed)

    line = Line(s=0.0, x=0.0, y=0.0, hdg=0.0, length=100.0)
    plan_view = PlanView(geometries=[line])

    road = Road(
        id=1,
        name="Test Road",
        length=100.0,
        junction=-1,
        plan_view=plan_view,
        road_types=[road_type_def],
    )

    # Create OpenDRIVE with header
    header = Header(
        rev_major="1",
        rev_minor="4",
        name="Test Map",
        version="1.0",
        date="2026-01-01T00:00:00",
        north="0.0",
        south="0.0",
        east="0.0",
        west="0.0",
        geo_reference="+proj=utm +zone=54 +datum=WGS84",
    )

    opendrive = OpenDRIVE(header=header, roads=[road], junctions=[], controllers=[])

    # Export to XML (returns string)
    xml_str = export_to_xml(opendrive)

    # Check that speed limit is present in the output
    assert '<type s="0.0" type="town">' in xml_str
    assert '<speed max="50" unit="km/h"/>' in xml_str
