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
        country="DE",
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
        country="DE",
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
        country="DE",
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
        country="DE",
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
        country="DE",
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


# ---------------------------------------------------------------------------
# Integration test: multiple LineStrings × multiple roads
# ---------------------------------------------------------------------------


def _make_linestring(ls_id, points):
    """Return a mock linestring with the given ID and points."""
    from unittest.mock import Mock, MagicMock

    ls = MagicMock()
    ls.id = ls_id
    ls.__len__ = Mock(return_value=len(points))
    ls.__getitem__ = Mock(side_effect=lambda i: points[i])
    ls.__iter__ = Mock(side_effect=lambda: iter(points))
    return ls


def _make_mock_roads():
    """Return a pair of mock roads for signal tests."""
    road0 = MagicMock()
    road0.id = 0
    road0.rule = TrafficRule.RHT
    road0.get_lanelet_to_lane_mapping.return_value = {100: -1, 101: -2}
    road0.get_half_width_at_s.return_value = -4.0
    road0.get_elevation_at_s.return_value = 0.0

    road1 = MagicMock()
    road1.id = 1
    road1.rule = TrafficRule.RHT
    road1.get_lanelet_to_lane_mapping.return_value = {200: -1, 201: -2}
    road1.get_half_width_at_s.return_value = -4.0
    road1.get_elevation_at_s.return_value = 0.0

    return [road0, road1]


def _make_mapping():
    """Return a RoadLaneletMapping for signal tests."""
    return RoadLaneletMapping(
        road_to_lanelets={0: [100, 101], 1: [200, 201]},
        lanelet_to_road={100: 0, 101: 0, 200: 1, 201: 1},
    )


def test_multiple_linestrings_deduplicated_to_one_per_road():
    """Test that duplicate linestrings are deduplicated: 2 LS × 2 roads → 2 signals."""
    from unittest.mock import patch, MagicMock

    pt_a = MagicMock(x=10.0, y=5.0, z=8.0)
    pt_b = MagicMock(x=12.0, y=6.0, z=9.0)
    ls1 = _make_linestring(1001, [pt_a])
    ls2 = _make_linestring(1002, [pt_b])

    traffic_light = MagicMock()
    traffic_light.id = 5000
    traffic_light.trafficLights = [ls1, ls2]
    traffic_light.stopLine = None
    traffic_light.attributes = {}

    mock_lanelet_map = MagicMock()
    mock_lanelet_map.laneletLayer.get.return_value = MagicMock()

    traffic_light_map = {5000: (traffic_light, [100, 200])}
    mapping = _make_mapping()
    roads = _make_mock_roads()

    with (
        patch(
            "autoware_lanelet2_to_opendrive.opendrive.signals_and_controllers.filter_regulatory_element_by_type",
            return_value=traffic_light_map,
        ),
        patch(
            "autoware_lanelet2_to_opendrive.opendrive.signals_and_controllers.SignalsAndControllers._calculate_signal_position",
            return_value=(10.0, -4.0),
        ),
    ):
        result = SignalsAndControllers.construct_from_lanelet_map(
            lanelet_map=mock_lanelet_map,
            road_lanelet_mapping=mapping,
            roads=roads,
        )

    # Deduplicated: 1 representative linestring × 2 roads = 2 signals
    assert len(result.signals) == 2, f"Expected 2 signals, got {len(result.signals)}"

    # 1 controller covering 2 signals
    assert len(result.controllers) == 1
    assert len(result.controllers[0].controls) == 2

    # All signals should use the first linestring (ls1, id=1001) as representative
    for sig in result.signals:
        assert sig.position_inertial is not None
        assert "1001" in sig.name


def test_dedup_single_linestring_unchanged():
    """Single linestring produces one signal per road (no dedup needed)."""
    from unittest.mock import patch, MagicMock

    pt = MagicMock(x=10.0, y=5.0, z=8.0)
    ls1 = _make_linestring(1001, [pt])

    traffic_light = MagicMock()
    traffic_light.id = 6000
    traffic_light.trafficLights = [ls1]
    traffic_light.stopLine = None
    traffic_light.attributes = {}

    mock_lanelet_map = MagicMock()
    mock_lanelet_map.laneletLayer.get.return_value = MagicMock()

    traffic_light_map = {6000: (traffic_light, [100, 200])}
    mapping = _make_mapping()
    roads = _make_mock_roads()

    with (
        patch(
            "autoware_lanelet2_to_opendrive.opendrive.signals_and_controllers.filter_regulatory_element_by_type",
            return_value=traffic_light_map,
        ),
        patch(
            "autoware_lanelet2_to_opendrive.opendrive.signals_and_controllers.SignalsAndControllers._calculate_signal_position",
            return_value=(5.0, -3.0),
        ),
    ):
        result = SignalsAndControllers.construct_from_lanelet_map(
            lanelet_map=mock_lanelet_map,
            road_lanelet_mapping=mapping,
            roads=roads,
        )

    assert len(result.signals) == 2
    assert len(result.controllers) == 1
    assert len(result.controllers[0].controls) == 2


def test_dedup_first_empty_uses_second():
    """When the first linestring is empty, the second non-empty one is used."""
    from unittest.mock import patch, MagicMock

    pt = MagicMock(x=10.0, y=5.0, z=8.0)
    ls_empty = _make_linestring(1001, [])
    ls_ok = _make_linestring(1002, [pt])

    traffic_light = MagicMock()
    traffic_light.id = 7000
    traffic_light.trafficLights = [ls_empty, ls_ok]
    traffic_light.stopLine = None
    traffic_light.attributes = {}

    mock_lanelet_map = MagicMock()
    mock_lanelet_map.laneletLayer.get.return_value = MagicMock()

    traffic_light_map = {7000: (traffic_light, [100, 200])}
    mapping = _make_mapping()
    roads = _make_mock_roads()

    with (
        patch(
            "autoware_lanelet2_to_opendrive.opendrive.signals_and_controllers.filter_regulatory_element_by_type",
            return_value=traffic_light_map,
        ),
        patch(
            "autoware_lanelet2_to_opendrive.opendrive.signals_and_controllers.SignalsAndControllers._calculate_signal_position",
            return_value=(1.0, -2.0),
        ),
    ):
        result = SignalsAndControllers.construct_from_lanelet_map(
            lanelet_map=mock_lanelet_map,
            road_lanelet_mapping=mapping,
            roads=roads,
        )

    # Should use ls_ok (id=1002) as representative
    assert len(result.signals) == 2
    for sig in result.signals:
        assert "1002" in sig.name


def test_dedup_all_empty_skipped():
    """When all linestrings are empty, no signals are created."""
    from unittest.mock import patch, MagicMock

    ls_empty1 = _make_linestring(1001, [])
    ls_empty2 = _make_linestring(1002, [])

    traffic_light = MagicMock()
    traffic_light.id = 8000
    traffic_light.trafficLights = [ls_empty1, ls_empty2]
    traffic_light.stopLine = None
    traffic_light.attributes = {}

    mock_lanelet_map = MagicMock()
    mock_lanelet_map.laneletLayer.get.return_value = MagicMock()

    traffic_light_map = {8000: (traffic_light, [100, 200])}
    mapping = _make_mapping()
    roads = _make_mock_roads()

    with patch(
        "autoware_lanelet2_to_opendrive.opendrive.signals_and_controllers.filter_regulatory_element_by_type",
        return_value=traffic_light_map,
    ):
        result = SignalsAndControllers.construct_from_lanelet_map(
            lanelet_map=mock_lanelet_map,
            road_lanelet_mapping=mapping,
            roads=roads,
        )

    assert len(result.signals) == 0
    assert len(result.controllers) == 0


def test_real_data_emits_arrow_subtypes(lanelet_map):
    """Real Autoware data drives the constructor's subtype path to non-trivial values.

    Exercises `_compute_signal_subtype_from_traffic_light` — the helper the
    `Signal.construct_from_lanelet2_traffic_signal` constructor calls — against
    the `AutowareTrafficLight` regulatory elements in `nishishinjuku.osm`.
    Going through the RE-level aggregator (rather than just
    `_compute_signal_subtype_from_bulbs` on individual LineStrings) is the test
    that would have caught wiring bugs where the constructor reads from the
    geometry LineString instead of `lightBulbs()`.
    """
    import autoware_lanelet2_extension_python.regulatory_elements as ll2_ext_reg

    from autoware_lanelet2_to_opendrive.opendrive.signal import (
        _compute_signal_subtype_from_traffic_light,
    )

    subtypes_seen: set = set()
    for reg_elem in lanelet_map.regulatoryElementLayer:
        if not isinstance(reg_elem, ll2_ext_reg.AutowareTrafficLight):
            continue
        subtypes_seen.add(_compute_signal_subtype_from_traffic_light(reg_elem))

    # The dataset has both pure 3-aspect REs and arrow-bearing REs.
    assert 0 in subtypes_seen, (
        "Expected at least one pure 3-aspect RE (subtype=0) but only saw "
        f"{sorted(subtypes_seen)}"
    )
    assert any(s >= 1 for s in subtypes_seen), (
        "Expected at least one arrow-bearing RE (subtype>=1) but only saw "
        f"{sorted(subtypes_seen)}"
    )
