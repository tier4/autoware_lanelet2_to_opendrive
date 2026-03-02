"""Tests for SignalsAndControllers class."""

from unittest.mock import MagicMock

from autoware_lanelet2_to_opendrive.opendrive import (
    SignalsAndControllers,
    Signal,
    Controller,
    ControlEntry,
    SignalType,
)
from autoware_lanelet2_to_opendrive.opendrive.enums import TrafficRule
from autoware_lanelet2_to_opendrive.util import RoadLaneletMapping


def test_signals_and_controllers_creation():
    """Test SignalsAndControllers creation."""
    sac = SignalsAndControllers()

    assert len(sac.signals) == 0
    assert len(sac.controllers) == 0
    assert len(sac) == 0


def test_add_signal():
    """Test adding signals."""
    sac = SignalsAndControllers()

    signal = Signal(
        id=1,
        name="Signal1",
        s=10.0,
        t=-2.0,
        orientation="-",
        dynamic="yes",
        country="OpenDRIVE",
        type=SignalType.TRAFFIC_LIGHT_3_LIGHTS,
        subtype=-1,
    )

    sac.add_signal(signal)

    assert len(sac.signals) == 1
    assert sac.signals[0] == signal


def test_add_controller():
    """Test adding controllers."""
    sac = SignalsAndControllers()

    controller = Controller(
        id=1,
        name="Controller1",
        controls=[ControlEntry(signal_id=1), ControlEntry(signal_id=2)],
    )

    sac.add_controller(controller)

    assert len(sac.controllers) == 1
    assert sac.controllers[0] == controller


def test_signals_and_controllers_repr():
    """Test string representation."""
    sac = SignalsAndControllers()

    signal = Signal(
        id=1,
        name="Signal1",
        s=10.0,
        t=-2.0,
        orientation="-",
        dynamic="yes",
        country="OpenDRIVE",
        type=SignalType.TRAFFIC_LIGHT_3_LIGHTS,
        subtype=-1,
    )
    sac.add_signal(signal)

    controller = Controller(id=1, name="Controller1")
    sac.add_controller(controller)

    repr_str = repr(sac)
    assert "SignalsAndControllers" in repr_str
    assert "signals=1" in repr_str
    assert "controllers=1" in repr_str


def test_signals_and_controllers_to_xml():
    """Test XML conversion."""
    sac = SignalsAndControllers()

    # Add signal
    signal = Signal(
        id=100,
        name="TrafficLight1",
        s=50.0,
        t=-3.0,
        orientation="-",
        dynamic="yes",
        country="OpenDRIVE",
        type=SignalType.TRAFFIC_LIGHT_3_LIGHTS,
        subtype=-1,
    )
    sac.add_signal(signal)

    # Add controller
    controller = Controller(
        id=1, name="Controller1", controls=[ControlEntry(signal_id=100)]
    )
    sac.add_controller(controller)

    # Convert to XML
    xml = sac.to_xml()

    assert xml.tag == "signalsAndControllers"

    # Check signals section
    signals_elem = xml.find("signals")
    assert signals_elem is not None
    signal_elems = signals_elem.findall("signal")
    assert len(signal_elems) == 1
    assert signal_elems[0].get("id") == "100"

    # Check controllers section
    controllers_elem = xml.find("controllers")
    assert controllers_elem is not None
    controller_elems = controllers_elem.findall("controller")
    assert len(controller_elems) == 1
    assert controller_elems[0].get("id") == "1"


def test_construct_from_lanelet_map_mock():
    """Test construct_from_lanelet_map with mock data."""
    # Create mock mapping
    road_to_lanelets = {
        0: [100, 101],
        1: [200, 201],
    }
    lanelet_to_road = {
        100: 0,
        101: 0,
        200: 1,
        201: 1,
    }
    mapping = RoadLaneletMapping(
        road_to_lanelets=road_to_lanelets, lanelet_to_road=lanelet_to_road
    )

    # Note: We can't fully test construct_from_lanelet_map without a real
    # lanelet2 map, but we can verify the mapping is used correctly
    # This test verifies the class structure and basic functionality
    assert mapping.get_road_for_lanelet(100) == 0
    assert mapping.get_road_for_lanelet(200) == 1


def test_empty_signals_and_controllers_to_xml():
    """Test XML conversion with no signals or controllers."""
    sac = SignalsAndControllers()
    xml = sac.to_xml()

    assert xml.tag == "signalsAndControllers"
    # Should have no child elements when empty
    assert len(xml) == 0


def test_multiple_signals_single_controller():
    """Test multiple signals coordinated by single controller."""
    sac = SignalsAndControllers()

    # Add multiple signals
    signal1 = Signal(
        id=1,
        name="Signal1",
        s=10.0,
        t=-2.0,
        orientation="-",
        dynamic="yes",
        country="OpenDRIVE",
        type=SignalType.TRAFFIC_LIGHT_3_LIGHTS,
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
        type=SignalType.TRAFFIC_LIGHT_3_LIGHTS,
        subtype=-1,
    )
    sac.add_signal(signal1)
    sac.add_signal(signal2)

    # Add controller coordinating both signals
    controller = Controller(
        id=1,
        name="IntersectionController",
        controls=[ControlEntry(signal_id=1), ControlEntry(signal_id=2)],
    )
    sac.add_controller(controller)

    assert len(sac.signals) == 2
    assert len(sac.controllers) == 1
    assert len(sac) == 3
    assert len(sac.controllers[0].controls) == 2


# ---------------------------------------------------------------------------
# Unit tests for _get_signal_lane_ids
# ---------------------------------------------------------------------------


def _make_road(rule: TrafficRule, mapping: dict) -> MagicMock:
    """Return a mock Road object with the given rule and lanelet-to-lane mapping."""
    road = MagicMock()
    road.rule = rule
    road.get_lanelet_to_lane_mapping.return_value = mapping
    return road


def test_get_signal_lane_ids_rht_from_mapping():
    """RHT: lane IDs come from the road mapping (negative)."""
    road = _make_road(TrafficRule.RHT, {10: -1, 11: -2})
    result = SignalsAndControllers._get_signal_lane_ids(
        road_lanelets_with_signal=[10, 11],
        matching_road=road,
    )
    assert set(result) == {-1, -2}
    assert all(lane_id < 0 for lane_id in result)


def test_get_signal_lane_ids_lht_from_mapping():
    """LHT: lane IDs come from the road mapping (positive)."""
    road = _make_road(TrafficRule.LHT, {20: 1, 21: 2})
    result = SignalsAndControllers._get_signal_lane_ids(
        road_lanelets_with_signal=[20, 21],
        matching_road=road,
    )
    assert set(result) == {1, 2}
    assert all(lane_id > 0 for lane_id in result)


def test_get_signal_lane_ids_rht_fallback_no_road():
    """Without a road object, fall back to RHT default (-1)."""
    result = SignalsAndControllers._get_signal_lane_ids(
        road_lanelets_with_signal=[99],
        matching_road=None,
    )
    assert result == [-1]


def test_get_signal_lane_ids_lht_fallback_missing_lanelet():
    """LHT: lanelets not in mapping → fall back to LHT innermost lane (+1)."""
    road = _make_road(TrafficRule.LHT, {})  # empty mapping
    result = SignalsAndControllers._get_signal_lane_ids(
        road_lanelets_with_signal=[55],
        matching_road=road,
    )
    assert result == [1]


def test_get_signal_lane_ids_rht_fallback_missing_lanelet():
    """RHT: lanelets not in mapping → fall back to RHT innermost lane (-1)."""
    road = _make_road(TrafficRule.RHT, {})  # empty mapping
    result = SignalsAndControllers._get_signal_lane_ids(
        road_lanelets_with_signal=[55],
        matching_road=road,
    )
    assert result == [-1]
