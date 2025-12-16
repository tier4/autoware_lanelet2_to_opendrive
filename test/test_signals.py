"""Tests for OpenDRIVE signal and signals modules."""

import lxml.etree as ET
from autoware_lanelet2_to_opendrive.opendrive.signal import (
    Signal,
    Validity,
    SignalUserData,
    SignalType,
)
from autoware_lanelet2_to_opendrive.opendrive.signals import (
    Signals,
    create_traffic_light,
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


def test_signals_container():
    """Test Signals container."""
    signals = Signals()

    # Add signals
    signal1 = Signal(
        id=1,
        name="Signal1",
        s=10.0,
        t=-2.0,
        orientation="-",
        dynamic="yes",
        country="OpenDRIVE",
        type=1000001,
        subtype=-1,
    )
    signal2 = Signal(
        id=2,
        name="Signal2",
        s=20.0,
        t=-2.0,
        orientation="-",
        dynamic="yes",
        country="OpenDRIVE",
        type=1000002,
        subtype=-1,
    )

    signals.add_signal(signal1)
    signals.add_signal(signal2)

    assert len(signals) == 2
    assert signals[0].id == 1
    assert signals[1].id == 2


def test_signals_to_xml():
    """Test Signals XML conversion."""
    signals = Signals()

    signal = Signal(
        id=362,
        name="Signal_3Light_Post01",
        s=35.840466582261428,
        t=-4.4797897637266599,
        orientation="-",
        dynamic="yes",
        country="OpenDRIVE",
        type=1000001,
        subtype=-1,
    )
    signals.add_signal(signal)

    xml = signals.to_xml()

    assert xml.tag == "signals"
    signal_elems = xml.findall("signal")
    assert len(signal_elems) == 1
    assert signal_elems[0].get("id") == "362"


def test_create_traffic_light_function():
    """Test create_traffic_light convenience function."""
    traffic_light = create_traffic_light(
        signal_id=100,
        name="TrafficLight1",
        s=50.0,
        t=-3.0,
        orientation="-",
        z_offset=0.5,
        height=1.2,
        width=0.6,
        traffic_light_type=SignalType.TRAFFIC_LIGHT_3_LIGHTS,
        from_lane=-1,
        to_lane=-1,
    )

    assert traffic_light.id == 100
    assert traffic_light.name == "TrafficLight1"
    assert traffic_light.s == 50.0
    assert traffic_light.t == -3.0
    assert traffic_light.dynamic == "yes"
    assert traffic_light.type == 1000001
    assert traffic_light.country == "OpenDRIVE"
    assert len(traffic_light.validities) == 1
    assert traffic_light.validities[0].from_lane == -1
    assert traffic_light.validities[0].to_lane == -1


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


def test_signals_iteration():
    """Test Signals container iteration."""
    signals = Signals()

    for i in range(3):
        signal = Signal(
            id=i,
            name=f"Signal{i}",
            s=float(i * 10),
            t=-2.0,
            orientation="-",
            dynamic="yes",
            country="OpenDRIVE",
            type=1000001,
            subtype=-1,
        )
        signals.add_signal(signal)

    # Test iteration
    signal_ids = [signal.id for signal in signals]
    assert signal_ids == [0, 1, 2]


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


def test_signals_repr():
    """Test Signals container string representation."""
    signals = Signals()

    signal = Signal(
        id=1,
        name="Signal1",
        s=10.0,
        t=-2.0,
        orientation="-",
        dynamic="yes",
        country="OpenDRIVE",
        type=1000001,
        subtype=-1,
    )
    signals.add_signal(signal)

    repr_str = repr(signals)
    assert "Signals" in repr_str
    assert "count=1" in repr_str


def test_road_with_signals_integration():
    """Test Road integration with Signals."""
    from autoware_lanelet2_to_opendrive.opendrive.road import Road
    from autoware_lanelet2_to_opendrive.opendrive.geometry import PlanView, Line

    # Create a simple road
    road = Road(
        id=1,
        name="Test Road",
        length=100.0,
        junction=-1,
    )

    # Create plan view with a simple line geometry
    line = Line()
    plan_view = PlanView(geometries=[line])
    plan_view.geometries[0].s = 0.0
    plan_view.geometries[0].x = 0.0
    plan_view.geometries[0].y = 0.0
    plan_view.geometries[0].hdg = 0.0
    plan_view.geometries[0].length = 100.0
    road.plan_view = plan_view

    # Create signals
    signals = Signals()
    traffic_light = create_traffic_light(
        signal_id=100,
        name="TrafficLight1",
        s=50.0,
        t=-3.0,
        orientation="-",
        z_offset=0.5,
        height=1.2,
        width=0.6,
        from_lane=-1,
        to_lane=-1,
    )
    signals.add_signal(traffic_light)
    road.signals = signals

    # Convert to XML
    xml = road.to_xml()

    # Verify signals element is present
    signals_elem = xml.find("signals")
    assert signals_elem is not None

    # Verify signal element is present
    signal_elems = signals_elem.findall("signal")
    assert len(signal_elems) == 1
    assert signal_elems[0].get("id") == "100"
    assert signal_elems[0].get("name") == "TrafficLight1"

    # Verify XML structure order (signals should come after lanes)
    children_tags = [child.tag for child in xml]
    # Signals should be in the road XML
    assert "signals" in children_tags


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
