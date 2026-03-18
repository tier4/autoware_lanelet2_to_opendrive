"""Tests for OpenDRIVE signal module."""

import lxml.etree as ET
import pytest
from autoware_lanelet2_to_opendrive.opendrive.signal import (
    Dependency,
    PositionInertial,
    Reference,
    Signal,
    SignalType,
    SignalUserData,
    Validity,
)


def test_validity_to_xml():
    """Test Validity XML conversion."""
    validity = Validity(from_lane=0, to_lane=0)
    xml = validity.to_xml()

    assert xml.tag == "validity"
    assert xml.get("fromLane") == "0"
    assert xml.get("toLane") == "0"


def test_signal_user_data_to_xml():
    """Test SignalUserData XML conversion."""
    user_data = SignalUserData(
        data={"vectorSignal": {"signalId": "{630a1ee4-4a24-4263-aaf7-df023be1ff6e}"}}
    )
    xml = user_data.to_xml()

    assert xml.tag == "userData"
    vector_signal = xml.find("vectorSignal")
    assert vector_signal is not None
    assert vector_signal.get("signalId") == "{630a1ee4-4a24-4263-aaf7-df023be1ff6e}"


def test_signal_creation():
    """Test Signal object creation."""
    signal = Signal(
        id=362,
        name="Signal_3Light_Post01",
        s=35.840466582261428,
        t=-4.4797897637266599,
        z_offset=-0.69904327392578125,
        h_offset=-3.5448458390874293e-8,
        roll=0.0,
        pitch=0.0,
        orientation="-",
        dynamic="yes",
        country="OpenDRIVE",
        type=SignalType.TRAFFIC_LIGHT_3_LIGHTS,
        subtype=-1,
        value=-1.0,
        text="",
        height=1.1595988571643829,
        width=0.52492320205637566,
    )

    assert signal.id == 362
    assert signal.name == "Signal_3Light_Post01"
    assert signal.dynamic == "yes"
    assert signal.type == 1000001


def test_signal_to_xml():
    """Test Signal XML conversion."""
    signal = Signal(
        id=362,
        name="Signal_3Light_Post01",
        s=35.840466582261428,
        t=-4.4797897637266599,
        z_offset=-0.69904327392578125,
        orientation="-",
        dynamic="yes",
        country="OpenDRIVE",
        type=1000001,
        subtype=-1,
        height=1.1595988571643829,
        width=0.52492320205637566,
        validities=[Validity(from_lane=0, to_lane=0)],
    )

    xml = signal.to_xml()

    assert xml.tag == "signal"
    assert xml.get("id") == "362"
    assert xml.get("name") == "Signal_3Light_Post01"
    assert xml.get("orientation") == "-"
    assert xml.get("dynamic") == "yes"
    assert xml.get("country") == "OpenDRIVE"
    assert xml.get("type") == "1000001"
    assert xml.get("subtype") == "-1"

    # Check validity element
    validity_elem = xml.find("validity")
    assert validity_elem is not None
    assert validity_elem.get("fromLane") == "0"
    assert validity_elem.get("toLane") == "0"


def test_signal_xml_formatting():
    """Test that Signal XML uses scientific notation for floats."""
    signal = Signal(
        id=1,
        name="Test",
        s=35.840466582261428,
        t=-4.4797897637266599,
        orientation="-",
        dynamic="yes",
        country="OpenDRIVE",
        type=1000001,
        subtype=-1,
    )

    xml = signal.to_xml()
    xml_string = ET.tostring(xml, encoding="unicode")

    # Check that scientific notation is used (contains 'e')
    assert "e" in xml_string.lower()


def test_signal_repr():
    """Test Signal string representation."""
    signal = Signal(
        id=100,
        name="TestSignal",
        s=50.5,
        t=-3.2,
        orientation="-",
        dynamic="yes",
        country="OpenDRIVE",
        type=1000001,
        subtype=-1,
    )

    repr_str = repr(signal)
    assert "100" in repr_str
    assert "TestSignal" in repr_str
    assert "1000001" in repr_str


def test_complete_signal_xml_output():
    """Test complete XML output matches expected OpenDRIVE format."""
    signal = Signal(
        id=362,
        name="Signal_3Light_Post01",
        s=35.840466582261428,
        t=-4.4797897637266599,
        z_offset=-0.69904327392578125,
        h_offset=-3.5448458390874293e-8,
        roll=0.0,
        pitch=0.0,
        orientation="-",
        dynamic="yes",
        country="OpenDRIVE",
        type=1000001,
        subtype=-1,
        value=-1.0,
        text="",
        height=1.1595988571643829,
        width=0.52492320205637566,
        validities=[Validity(from_lane=0, to_lane=0)],
    )

    xml = signal.to_xml()
    xml_string = ET.tostring(xml, encoding="unicode", pretty_print=True)

    # Verify all required attributes are present
    assert 'id="362"' in xml_string
    assert 'name="Signal_3Light_Post01"' in xml_string
    assert 'orientation="-"' in xml_string
    assert 'dynamic="yes"' in xml_string
    assert 'country="OpenDRIVE"' in xml_string
    assert 'type="1000001"' in xml_string
    assert 'subtype="-1"' in xml_string

    # Verify validity sub-element
    assert "<validity" in xml_string
    assert 'fromLane="0"' in xml_string
    assert 'toLane="0"' in xml_string


def test_construct_from_lanelet2_traffic_signal_basic():
    """Test basic construction from lanelet2 traffic signal."""

    # Create a mock traffic light object
    class MockPoint:
        def __init__(self, x, y, z):
            self.x = x
            self.y = y
            self.z = z

    class MockLineString:
        def __init__(self, ls_id, points):
            self.id = ls_id
            self.points = points

        def __len__(self):
            return len(self.points)

        def __getitem__(self, index):
            return self.points[index]

    class MockTrafficLight:
        def __init__(self, traffic_light_id, geometry):
            self.id = traffic_light_id
            self.trafficLights = geometry
            self.attributes = {}

    # Create mock geometry
    point = MockPoint(100.0, 200.0, 5.0)
    linestring = MockLineString(1234, [point])
    traffic_light = MockTrafficLight(12345, [linestring])

    # Convert to Signal
    signal = Signal.construct_from_lanelet2_traffic_signal(
        traffic_light=traffic_light, signal_id=100, s=50.0, t=-4.5, lane_ids=[-1]
    )

    # Verify signal properties
    assert signal.id == 100
    assert signal.name == "TrafficLight_12345_1234"
    assert signal.s == 50.0
    assert signal.t == -4.5
    assert signal.z_offset == 5.0
    assert signal.orientation == "-"
    assert signal.dynamic == "yes"
    assert signal.country == "OpenDRIVE"
    assert signal.type == SignalType.TRAFFIC_LIGHT_3_LIGHTS
    assert signal.subtype == -1
    assert len(signal.validities) == 1
    assert signal.validities[0].from_lane == -1
    assert signal.validities[0].to_lane == -1


def test_construct_from_lanelet2_traffic_signal_with_attributes():
    """Test construction with traffic light attributes."""

    # Create mock objects
    class MockPoint:
        def __init__(self, x, y, z):
            self.x = x
            self.y = y
            self.z = z

    class MockLineString:
        def __init__(self, ls_id, points):
            self.id = ls_id
            self.points = points

        def __len__(self):
            return len(self.points)

        def __getitem__(self, index):
            return self.points[index]

    class MockTrafficLight:
        def __init__(self, traffic_light_id, geometry, attrs):
            self.id = traffic_light_id
            self.trafficLights = geometry
            self.attributes = attrs

    # Test pedestrian traffic light
    point = MockPoint(100.0, 200.0, 3.0)
    linestring = MockLineString(5001, [point])
    traffic_light = MockTrafficLight(
        67890, [linestring], {"subtype": "pedestrian_light"}
    )

    signal = Signal.construct_from_lanelet2_traffic_signal(
        traffic_light=traffic_light, signal_id=200, s=75.0, t=3.0, lane_ids=[1]
    )

    assert signal.type == SignalType.TRAFFIC_LIGHT_PEDESTRIAN
    assert signal.orientation == "+"  # Positive t means left side


def test_construct_from_lanelet2_traffic_signal_arrow():
    """Test construction with arrow traffic light."""

    class MockPoint:
        def __init__(self, x, y, z):
            self.x = x
            self.y = y
            self.z = z

    class MockLineString:
        def __init__(self, ls_id, points):
            self.id = ls_id
            self.points = points

        def __len__(self):
            return len(self.points)

        def __getitem__(self, index):
            return self.points[index]

    class MockTrafficLight:
        def __init__(self, traffic_light_id, geometry, attrs):
            self.id = traffic_light_id
            self.trafficLights = geometry
            self.attributes = attrs

    point = MockPoint(150.0, 250.0, 4.5)
    linestring = MockLineString(5002, [point])
    traffic_light = MockTrafficLight(11111, [linestring], {"type": "arrow_light"})

    signal = Signal.construct_from_lanelet2_traffic_signal(
        traffic_light=traffic_light, signal_id=300, s=100.0, t=-5.0
    )

    assert signal.type == SignalType.TRAFFIC_LIGHT_ARROW


def test_construct_from_lanelet2_traffic_signal_multiple_lanes():
    """Test construction with multiple lane validities."""

    class MockPoint:
        def __init__(self, x, y, z):
            self.x = x
            self.y = y
            self.z = z

    class MockLineString:
        def __init__(self, ls_id, points):
            self.id = ls_id
            self.points = points

        def __len__(self):
            return len(self.points)

        def __getitem__(self, index):
            return self.points[index]

    class MockTrafficLight:
        def __init__(self, traffic_light_id, geometry):
            self.id = traffic_light_id
            self.trafficLights = geometry
            self.attributes = {}

    point = MockPoint(100.0, 200.0, 5.0)
    linestring = MockLineString(5003, [point])
    traffic_light = MockTrafficLight(22222, [linestring])

    # Test with multiple lanes
    signal = Signal.construct_from_lanelet2_traffic_signal(
        traffic_light=traffic_light,
        signal_id=400,
        s=120.0,
        t=-4.0,
        lane_ids=[-3, -2, -1],
    )

    assert len(signal.validities) == 1
    assert signal.validities[0].from_lane == -3
    assert signal.validities[0].to_lane == -1


def test_construct_from_lanelet2_traffic_signal_no_geometry():
    """Test error handling when traffic light has no geometry."""

    class MockTrafficLight:
        def __init__(self, traffic_light_id):
            self.id = traffic_light_id
            self.trafficLights = []
            self.attributes = {}

    traffic_light = MockTrafficLight(99999)

    with pytest.raises(ValueError, match="has no geometry"):
        Signal.construct_from_lanelet2_traffic_signal(
            traffic_light=traffic_light, signal_id=500, s=50.0, t=-4.0
        )


def test_construct_from_lanelet2_traffic_signal_with_road_elevation():
    """Test z_offset calculation with road elevation."""

    # Create a mock traffic light object
    class MockPoint:
        def __init__(self, x, y, z):
            self.x = x
            self.y = y
            self.z = z

    class MockLineString:
        def __init__(self, ls_id, points):
            self.id = ls_id
            self.points = points

        def __len__(self):
            return len(self.points)

        def __getitem__(self, index):
            return self.points[index]

    class MockTrafficLight:
        def __init__(self, traffic_light_id, geometry):
            self.id = traffic_light_id
            self.trafficLights = geometry
            self.attributes = {}

    # Create mock geometry
    # Signal absolute height is 45.0m, road elevation is 40.0m
    # Expected z_offset should be 45.0 - 40.0 = 5.0m
    point = MockPoint(100.0, 200.0, 45.0)
    linestring = MockLineString(5004, [point])
    traffic_light = MockTrafficLight(12345, [linestring])

    # Convert to Signal with road elevation
    signal = Signal.construct_from_lanelet2_traffic_signal(
        traffic_light=traffic_light,
        signal_id=100,
        s=50.0,
        t=-4.5,
        lane_ids=[-1],
        road_elevation_at_s=40.0,  # Road surface is at 40m elevation
    )

    # Verify z_offset is relative to road surface
    assert (
        signal.z_offset == 5.0
    ), f"Expected z_offset=5.0 (45.0 - 40.0), got {signal.z_offset}"


def test_construct_from_lanelet2_traffic_signal_empty_linestring():
    """Test error handling when linestring is empty."""

    class MockLineString:
        def __init__(self):
            self.points = []

        def __len__(self):
            return 0

    class MockTrafficLight:
        def __init__(self, traffic_light_id):
            self.id = traffic_light_id
            self.trafficLights = [MockLineString()]
            self.attributes = {}

    traffic_light = MockTrafficLight(88888)

    with pytest.raises(ValueError, match="linestring.*is empty"):
        Signal.construct_from_lanelet2_traffic_signal(
            traffic_light=traffic_light, signal_id=600, s=50.0, t=-4.0
        )


def test_construct_from_lanelet2_traffic_signal_with_coordinate_offset():
    """Test z_offset calculation with coordinate offset enabled."""
    from autoware_lanelet2_to_opendrive.config import COORDINATE_OFFSET

    # Set up coordinate offset (UTM to local)
    original_x = COORDINATE_OFFSET.x
    original_y = COORDINATE_OFFSET.y
    original_z = COORDINATE_OFFSET.z
    COORDINATE_OFFSET.x = 0.0
    COORDINATE_OFFSET.y = 0.0
    COORDINATE_OFFSET.z = 40.0

    try:
        # Create mock traffic light at UTM height 45.0
        class MockPoint:
            def __init__(self, x, y, z):
                self.x = x
                self.y = y
                self.z = z

        class MockLineString:
            def __init__(self, ls_id, points):
                self.id = ls_id
                self.points = points

            def __len__(self):
                return len(self.points)

            def __getitem__(self, index):
                return self.points[index]

        class MockTrafficLight:
            def __init__(self, traffic_light_id, geometry):
                self.id = traffic_light_id
                self.trafficLights = geometry
                self.attributes = {}

        point = MockPoint(100.0, 200.0, 45.0)
        linestring = MockLineString(5005, [point])
        traffic_light = MockTrafficLight(12345, [linestring])

        # Road elevation: 3.0m in local coordinates
        signal = Signal.construct_from_lanelet2_traffic_signal(
            traffic_light=traffic_light,
            signal_id=100,
            s=50.0,
            t=-4.5,
            lane_ids=[-1],
            road_elevation_at_s=3.0,
        )

        # Expected: (45.0 - 40.0) - 3.0 = 2.0m relative height
        assert signal.z_offset == 2.0
    finally:
        # Restore original offset
        COORDINATE_OFFSET.x = original_x
        COORDINATE_OFFSET.y = original_y
        COORDINATE_OFFSET.z = original_z


def test_signal_to_signal_reference_xml():
    """Test Signal conversion to signalReference XML."""
    signal = Signal(
        id=59,
        name="TrafficLight_3002245",
        s=44.13,
        t=6.51,  # Original signal lateral offset
        orientation="+",
        dynamic="yes",
        country="OpenDRIVE",
        type=1000001,
        subtype=-1,
        validities=[Validity(from_lane=-1, to_lane=-1)],
    )

    xml = signal.to_signal_reference_xml()

    assert xml.tag == "signalReference"
    assert xml.get("id") == "59"
    assert xml.get("s") == "4.4130000000000003e+01"  # Scientific notation
    assert xml.get("t") == "0.0000000000000000e+00"  # Always 0.0
    assert xml.get("orientation") == "+"

    # Should NOT have signal-specific attributes
    assert xml.get("name") is None
    assert xml.get("dynamic") is None
    assert xml.get("type") is None

    # Should have validity
    validity_elem = xml.find("validity")
    assert validity_elem is not None
    assert validity_elem.get("fromLane") == "-1"
    assert validity_elem.get("toLane") == "-1"


def test_signal_reference_with_multiple_validities():
    """Test signalReference with multiple validity elements."""
    signal = Signal(
        id=100,
        name="Signal1",
        s=50.0,
        t=-3.5,
        orientation="-",
        dynamic="yes",
        country="OpenDRIVE",
        type=1000001,
        subtype=-1,
        validities=[
            Validity(from_lane=-3, to_lane=-1),
            Validity(from_lane=1, to_lane=3),
        ],
    )

    xml = signal.to_signal_reference_xml()
    validity_elems = xml.findall("validity")
    assert len(validity_elems) == 2


def test_signal_reference_without_validities():
    """Test signalReference when signal has no validities."""
    signal = Signal(
        id=200,
        name="Signal2",
        s=75.0,
        t=-4.0,
        orientation="-",
        dynamic="yes",
        country="OpenDRIVE",
        type=1000001,
        subtype=-1,
        validities=None,
    )

    xml = signal.to_signal_reference_xml()
    validity_elems = xml.findall("validity")
    assert len(validity_elems) == 0


# ---------------------------------------------------------------------------
# Tests for Dependency and Reference dataclasses
# ---------------------------------------------------------------------------


def test_dependency_to_xml():
    """Test Dependency XML conversion."""
    dep = Dependency(id=102, type="trafficLight")
    xml = dep.to_xml()

    assert xml.tag == "dependency"
    assert xml.get("id") == "102"
    assert xml.get("type") == "trafficLight"


def test_reference_to_xml():
    """Test Reference XML conversion."""
    ref = Reference(id=203, element_type="signal", type="stopLine")
    xml = ref.to_xml()

    assert xml.tag == "reference"
    assert xml.get("id") == "203"
    assert xml.get("elementType") == "signal"
    assert xml.get("type") == "stopLine"


def test_signal_type_stop_line():
    """Test that STOP_LINE type constant is defined."""
    assert SignalType.STOP_LINE == 294


def test_stop_line_signal_with_dependencies():
    """Test Signal of type 294 (stop line) with traffic light dependencies."""
    signal = Signal(
        id=203,
        name="StopLine_100",
        s=3.0,
        t=0.0,
        orientation="-",
        dynamic="no",
        country="OpenDRIVE",
        type=SignalType.STOP_LINE,
        subtype=-1,
        dependencies=[
            Dependency(id=102, type="trafficLight"),
            Dependency(id=103, type="trafficLight"),
        ],
    )

    xml = signal.to_xml()
    xml_string = ET.tostring(xml, encoding="unicode", pretty_print=True)

    assert xml.get("type") == "294"
    assert xml.get("dynamic") == "no"

    dep_elems = xml.findall("dependency")
    assert len(dep_elems) == 2
    assert dep_elems[0].get("id") == "102"
    assert dep_elems[0].get("type") == "trafficLight"
    assert dep_elems[1].get("id") == "103"
    assert dep_elems[1].get("type") == "trafficLight"

    assert "<dependency" in xml_string
    assert 'type="trafficLight"' in xml_string


def test_traffic_light_signal_with_stop_line_reference():
    """Test traffic light Signal with a stop line reference."""
    signal = Signal(
        id=102,
        name="TrafficLight_1000",
        s=3.0,
        t=-4.5,
        orientation="-",
        dynamic="yes",
        country="OpenDRIVE",
        type=SignalType.TRAFFIC_LIGHT_3_LIGHTS,
        subtype=-1,
        references=[
            Reference(id=203, element_type="signal", type="stopLine"),
        ],
    )

    xml = signal.to_xml()
    xml_string = ET.tostring(xml, encoding="unicode", pretty_print=True)

    assert xml.get("type") == "1000001"

    ref_elems = xml.findall("reference")
    assert len(ref_elems) == 1
    assert ref_elems[0].get("id") == "203"
    assert ref_elems[0].get("elementType") == "signal"
    assert ref_elems[0].get("type") == "stopLine"

    assert "<reference" in xml_string
    assert 'elementType="signal"' in xml_string
    assert 'type="stopLine"' in xml_string


def test_signal_dependency_ordering_after_validity():
    """Test that dependencies appear after validity elements in XML output."""
    signal = Signal(
        id=203,
        name="StopLine_200",
        s=5.0,
        t=-2.0,
        orientation="-",
        dynamic="no",
        country="OpenDRIVE",
        type=SignalType.STOP_LINE,
        subtype=-1,
        validities=[Validity(from_lane=-1, to_lane=-1)],
        dependencies=[Dependency(id=50, type="trafficLight")],
    )

    xml = signal.to_xml()
    children = list(xml)

    # validity must appear before dependency
    tags = [child.tag for child in children]
    assert "validity" in tags
    assert "dependency" in tags
    assert tags.index("validity") < tags.index("dependency")


def test_signal_no_dependencies_no_elements():
    """Test that signals without dependencies/references have no such elements."""
    signal = Signal(
        id=10,
        name="TL_10",
        s=1.0,
        t=-3.0,
        orientation="-",
        dynamic="yes",
        country="OpenDRIVE",
        type=SignalType.TRAFFIC_LIGHT_3_LIGHTS,
        subtype=-1,
    )

    xml = signal.to_xml()
    assert xml.find("dependency") is None
    assert xml.find("reference") is None


# ---------------------------------------------------------------------------
# Tests for StopSign signal (type 206)
# ---------------------------------------------------------------------------


def test_signal_type_stop_sign():
    """Test that STOP_SIGN type constant is defined as 206."""
    assert SignalType.STOP_SIGN == 206


def test_stop_sign_signal_creation():
    """Test StopSign signal creation with type 206."""
    signal = Signal(
        id=500,
        name="StopSign_12345",
        s=10.0,
        t=-2.5,
        z_offset=0.0,
        orientation="-",
        dynamic="no",
        country="OpenDRIVE",
        type=SignalType.STOP_SIGN,
        subtype=-1,
        value=-1.0,
        height=0.0,
        width=3.5,
    )

    assert signal.id == 500
    assert signal.name == "StopSign_12345"
    assert signal.type == 206
    assert signal.dynamic == "no"
    assert signal.orientation == "-"


def test_stop_sign_signal_to_xml():
    """Test StopSign signal XML output has correct type attribute."""
    signal = Signal(
        id=501,
        name="StopSign_67890",
        s=25.0,
        t=-1.0,
        orientation="-",
        dynamic="no",
        country="OpenDRIVE",
        type=SignalType.STOP_SIGN,
        subtype=-1,
        validities=[Validity(from_lane=-1, to_lane=-1)],
    )

    xml = signal.to_xml()
    xml_string = ET.tostring(xml, encoding="unicode", pretty_print=True)

    assert xml.get("type") == "206"
    assert xml.get("dynamic") == "no"
    assert xml.get("name") == "StopSign_67890"

    # Should have validity but no dependencies (standalone stop sign)
    validity_elem = xml.find("validity")
    assert validity_elem is not None
    assert xml.find("dependency") is None
    assert xml.find("reference") is None

    assert 'type="206"' in xml_string


def test_stop_sign_signal_has_no_dependencies():
    """Test that StopSign signals have no traffic light dependencies."""
    signal = Signal(
        id=502,
        name="StopSign_100",
        s=5.0,
        t=-3.0,
        orientation="-",
        dynamic="no",
        country="OpenDRIVE",
        type=SignalType.STOP_SIGN,
        subtype=-1,
    )

    assert signal.dependencies is None
    assert signal.references is None

    xml = signal.to_xml()
    assert len(xml.findall("dependency")) == 0
    assert len(xml.findall("reference")) == 0


# ---------------------------------------------------------------------------
# Tests for YieldSign signal (type 205)
# ---------------------------------------------------------------------------


def test_signal_type_yield_sign():
    """Test that YIELD_SIGN type constant is defined as 205."""
    assert SignalType.YIELD_SIGN == 205


def test_yield_sign_signal_creation():
    """Test YieldSign signal creation with type 205 and XML output."""
    signal = Signal(
        id=600,
        name="YieldSign_1409",
        s=12.0,
        t=-2.0,
        z_offset=0.0,
        orientation="-",
        dynamic="no",
        country="OpenDRIVE",
        type=SignalType.YIELD_SIGN,
        subtype=-1,
        value=-1.0,
        height=0.0,
        width=3.5,
    )

    assert signal.id == 600
    assert signal.name == "YieldSign_1409"
    assert signal.type == 205
    assert signal.dynamic == "no"

    xml = signal.to_xml()
    xml_string = ET.tostring(xml, encoding="unicode", pretty_print=True)

    assert xml.get("type") == "205"
    assert xml.get("name") == "YieldSign_1409"
    assert 'type="205"' in xml_string
    # Standalone yield sign has no dependencies
    assert xml.find("dependency") is None


def test_road_marking_stop_line_with_yield_dependency():
    """Test StopLine (type 294) with yieldSign dependency."""
    signal = Signal(
        id=601,
        name="StopLine_1409",
        s=12.0,
        t=-2.0,
        z_offset=0.0,
        orientation="-",
        dynamic="no",
        country="OpenDRIVE",
        type=SignalType.STOP_LINE,
        subtype=-1,
        value=-1.0,
        height=0.0,
        width=3.5,
        dependencies=[Dependency(id=600, type="yieldSign")],
    )

    assert signal.type == 294
    assert len(signal.dependencies) == 1
    assert signal.dependencies[0].id == 600
    assert signal.dependencies[0].type == "yieldSign"

    xml = signal.to_xml()
    xml_string = ET.tostring(xml, encoding="unicode", pretty_print=True)

    assert xml.get("type") == "294"
    dep_elems = xml.findall("dependency")
    assert len(dep_elems) == 1
    assert dep_elems[0].get("id") == "600"
    assert dep_elems[0].get("type") == "yieldSign"
    assert 'type="yieldSign"' in xml_string


# ---------------------------------------------------------------------------
# Tests for PositionInertial dataclass
# ---------------------------------------------------------------------------


def test_position_inertial_to_xml():
    """Test PositionInertial XML conversion."""
    pos = PositionInertial(x=553.09, y=362.05, z=20.10, hdg=-1.65)
    xml = pos.to_xml()

    assert xml.tag == "positionInertial"
    # Verify scientific notation formatting
    assert "e" in xml.get("x").lower()
    assert "e" in xml.get("y").lower()
    assert "e" in xml.get("z").lower()
    assert "e" in xml.get("hdg").lower()
    assert xml.get("pitch") is not None
    assert xml.get("roll") is not None


def test_position_inertial_defaults():
    """Test PositionInertial default values."""
    pos = PositionInertial(x=1.0, y=2.0, z=3.0)
    assert pos.hdg == 0.0
    assert pos.pitch == 0.0
    assert pos.roll == 0.0


def test_signal_with_position_inertial_xml():
    """Test Signal with positionInertial generates correct XML child element."""
    pos = PositionInertial(x=100.0, y=200.0, z=10.0, hdg=1.57)
    signal = Signal(
        id=58,
        name="TrafficLight_3002245_1234",
        s=50.0,
        t=-4.5,
        orientation="-",
        dynamic="yes",
        country="OpenDRIVE",
        type=SignalType.TRAFFIC_LIGHT_3_LIGHTS,
        subtype=-1,
        validities=[Validity(from_lane=-1, to_lane=-1)],
        position_inertial=pos,
    )

    xml = signal.to_xml()
    xml_string = ET.tostring(xml, encoding="unicode", pretty_print=True)

    # positionInertial must be present
    pi_elem = xml.find("positionInertial")
    assert pi_elem is not None
    assert "e" in pi_elem.get("x").lower()

    # Ordering: validity < positionInertial < userData
    tags = [child.tag for child in xml]
    assert "validity" in tags
    assert "positionInertial" in tags
    assert tags.index("validity") < tags.index("positionInertial")

    assert "<positionInertial" in xml_string


def test_signal_without_position_inertial_xml():
    """Test Signal without positionInertial has no such element."""
    signal = Signal(
        id=10,
        name="TL_10",
        s=1.0,
        t=-3.0,
        orientation="-",
        dynamic="yes",
        country="OpenDRIVE",
        type=SignalType.TRAFFIC_LIGHT_3_LIGHTS,
        subtype=-1,
    )

    xml = signal.to_xml()
    assert xml.find("positionInertial") is None


def test_centroid_z_offset_multiple_points():
    """Test z_offset is computed from centroid of all points, not just first."""

    class MockPoint:
        def __init__(self, x, y, z):
            self.x = x
            self.y = y
            self.z = z

    class MockLineString:
        def __init__(self, ls_id, points):
            self.id = ls_id
            self.points = points

        def __len__(self):
            return len(self.points)

        def __getitem__(self, index):
            return self.points[index]

    class MockTrafficLight:
        def __init__(self, traffic_light_id, geometry):
            self.id = traffic_light_id
            self.trafficLights = geometry
            self.attributes = {}

    # Three bulbs at z = 4.0, 5.0, 6.0 → centroid z = 5.0
    points = [
        MockPoint(0.0, 0.0, 4.0),
        MockPoint(1.0, 0.0, 5.0),
        MockPoint(2.0, 0.0, 6.0),
    ]
    linestring = MockLineString(7001, points)
    traffic_light = MockTrafficLight(33333, [linestring])

    signal = Signal.construct_from_lanelet2_traffic_signal(
        traffic_light=traffic_light, signal_id=700, s=10.0, t=-2.0
    )

    assert signal.z_offset == 5.0, f"Expected centroid z=5.0, got {signal.z_offset}"


def test_multiple_linestrings_different_names():
    """Test that different linestrings produce different signal names."""

    class MockPoint:
        def __init__(self, x, y, z):
            self.x = x
            self.y = y
            self.z = z

    class MockLineString:
        def __init__(self, ls_id, points):
            self.id = ls_id
            self.points = points

        def __len__(self):
            return len(self.points)

        def __getitem__(self, index):
            return self.points[index]

    class MockTrafficLight:
        def __init__(self, traffic_light_id, geometry):
            self.id = traffic_light_id
            self.trafficLights = geometry
            self.attributes = {}

    p1 = MockPoint(0.0, 0.0, 5.0)
    p2 = MockPoint(1.0, 0.0, 6.0)
    ls1 = MockLineString(1001, [p1])
    ls2 = MockLineString(1002, [p2])
    traffic_light = MockTrafficLight(44444, [ls1, ls2])

    sig1 = Signal.construct_from_lanelet2_traffic_signal(
        traffic_light=traffic_light,
        light_linestring=ls1,
        signal_id=800,
        s=10.0,
        t=-2.0,
    )
    sig2 = Signal.construct_from_lanelet2_traffic_signal(
        traffic_light=traffic_light,
        light_linestring=ls2,
        signal_id=801,
        s=10.0,
        t=-2.0,
    )

    assert sig1.name == "TrafficLight_44444_1001"
    assert sig2.name == "TrafficLight_44444_1002"
    assert sig1.name != sig2.name
