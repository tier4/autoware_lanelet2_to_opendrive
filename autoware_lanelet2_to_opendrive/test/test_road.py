"""Tests for Road class XML generation with signals and signalReferences."""

from autoware_lanelet2_to_opendrive.opendrive.road import Road
from autoware_lanelet2_to_opendrive.opendrive.signal import Signal, Validity


def test_road_signals_with_references():
    """Test Road.to_xml() generates both signals and signalReferences."""
    signal1 = Signal(
        id=1,
        name="Signal1",
        s=10.0,
        t=-3.0,
        orientation="-",
        dynamic="yes",
        country="OpenDRIVE",
        type=1000001,
        subtype=-1,
        validities=[Validity(from_lane=-1, to_lane=-1)],
    )

    signal2 = Signal(
        id=2,
        name="Signal2",
        s=50.0,
        t=2.5,
        orientation="+",
        dynamic="yes",
        country="OpenDRIVE",
        type=1000001,
        subtype=-1,
        validities=[Validity(from_lane=1, to_lane=1)],
    )

    road = Road(id=0, length=100.0, signals=[signal1, signal2])

    xml = road.to_xml()
    signals_elem = xml.find("signals")
    assert signals_elem is not None

    # Should have 2 signal elements and 2 signalReference elements
    signal_elems = signals_elem.findall("signal")
    signal_ref_elems = signals_elem.findall("signalReference")

    assert len(signal_elems) == 2
    assert len(signal_ref_elems) == 2

    # Verify signal IDs match
    signal_ids = {elem.get("id") for elem in signal_elems}
    ref_ids = {elem.get("id") for elem in signal_ref_elems}
    assert signal_ids == ref_ids == {"1", "2"}

    # Verify all signalReferences have t="0.0"
    for ref_elem in signal_ref_elems:
        t_value = ref_elem.get("t")
        assert "0.0" in t_value or t_value == "0.0000000000000000e+00"
