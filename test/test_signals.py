"""Tests for OpenDRIVE signal module."""

import lxml.etree as ET
import pytest
from autoware_lanelet2_to_opendrive.opendrive.signal import (
    Signal,
    Validity,
    SignalUserData,
    SignalType,
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
        def __init__(self, points):
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
    linestring = MockLineString([point])
    traffic_light = MockTrafficLight(12345, [linestring])

    # Convert to Signal
    signal = Signal.construct_from_lanelet2_traffic_signal(
        traffic_light=traffic_light, signal_id=100, s=50.0, t=-4.5, lane_ids=[-1]
    )

    # Verify signal properties
    assert signal.id == 100
    assert signal.name == "TrafficLight_12345"
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
        def __init__(self, points):
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
    linestring = MockLineString([point])
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
        def __init__(self, points):
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
    linestring = MockLineString([point])
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
        def __init__(self, points):
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
    linestring = MockLineString([point])
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
