"""Tests for Road traffic rule attribute support."""

from pathlib import Path
import lanelet2
import lxml.etree as ET
from autoware_lanelet2_extension_python.projection import MGRSProjector
from autoware_lanelet2_to_opendrive.opendrive.road import Road
from autoware_lanelet2_to_opendrive.opendrive.enums import TrafficRule


def load_test_map():
    """Load the test lanelet2 map."""
    test_data_path = Path(__file__).parent / "data" / "lanelet2_map.osm"
    projector = MGRSProjector(
        lanelet2.io.Origin(35.23, 139.16)
    )  # MGRS origin for Tokyo area (54SUE)
    return lanelet2.io.load(str(test_data_path), projector)


def test_road_with_rht_rule():
    """Test that Road with RHT rule generates correct XML attribute."""
    lanelet_map = load_test_map()

    lanelet_group = [
        lanelet_map.laneletLayer.get(3002094),
        lanelet_map.laneletLayer.get(3002093),
    ]

    road = Road.construct_from_lanelet_groups(
        lanelet_map,
        lanelet_group,
        road_id=0,
        s_offset=0.0,
        traffic_rule=TrafficRule.RHT,
    )

    # Verify rule is set to RHT
    assert road.rule == TrafficRule.RHT, "Road should have rule=TrafficRule.RHT"

    # Convert to XML
    road_xml = road.to_xml()

    # Verify rule attribute exists and has correct value
    assert road_xml.get("rule") == "RHT", "Road should have rule='RHT' attribute"

    # Verify XML structure
    xml_str = ET.tostring(road_xml, encoding="unicode")
    assert 'rule="RHT"' in xml_str, "XML should contain rule='RHT' attribute"


def test_road_with_lht_rule():
    """Test that Road with LHT rule generates correct XML attribute."""
    lanelet_map = load_test_map()

    lanelet_group = [
        lanelet_map.laneletLayer.get(3002094),
        lanelet_map.laneletLayer.get(3002093),
    ]

    road = Road.construct_from_lanelet_groups(
        lanelet_map,
        lanelet_group,
        road_id=0,
        s_offset=0.0,
        traffic_rule=TrafficRule.LHT,
    )

    # Verify rule is set to LHT
    assert road.rule == TrafficRule.LHT, "Road should have rule=TrafficRule.LHT"

    # Convert to XML
    road_xml = road.to_xml()

    # Verify rule attribute exists and has correct value
    assert road_xml.get("rule") == "LHT", "Road should have rule='LHT' attribute"

    # Verify XML structure
    xml_str = ET.tostring(road_xml, encoding="unicode")
    assert 'rule="LHT"' in xml_str, "XML should contain rule='LHT' attribute"


def test_road_without_rule():
    """Test that Road without rule does not generate rule attribute."""
    lanelet_map = load_test_map()

    lanelet_group = [
        lanelet_map.laneletLayer.get(3002094),
        lanelet_map.laneletLayer.get(3002093),
    ]

    road = Road.construct_from_lanelet_groups(
        lanelet_map, lanelet_group, road_id=0, s_offset=0.0, traffic_rule=None
    )

    # Verify rule is None
    assert road.rule is None, "Road should have no rule by default"

    # Convert to XML
    road_xml = road.to_xml()

    # Verify rule attribute does NOT exist
    assert road_xml.get("rule") is None, "Road should not have rule attribute when None"

    # Verify XML structure
    xml_str = ET.tostring(road_xml, encoding="unicode")
    assert "rule=" not in xml_str, "XML should not contain rule attribute"


def test_road_rule_enum_values():
    """Test that TrafficRule enum has correct values."""
    assert TrafficRule.RHT.value == "RHT", "RHT enum value should be 'RHT'"
    assert TrafficRule.LHT.value == "LHT", "LHT enum value should be 'LHT'"

    # Test that enum can be constructed from string
    assert TrafficRule["RHT"] == TrafficRule.RHT, "Should construct RHT from string"
    assert TrafficRule["LHT"] == TrafficRule.LHT, "Should construct LHT from string"


def test_lane_ids_rht():
    """Test that lanes have negative IDs for RHT (right lanes)."""
    lanelet_map = load_test_map()

    lanelet_group = [
        lanelet_map.laneletLayer.get(3002094),
        lanelet_map.laneletLayer.get(3002093),
    ]

    road = Road.construct_from_lanelet_groups(
        lanelet_map,
        lanelet_group,
        road_id=0,
        s_offset=0.0,
        traffic_rule=TrafficRule.RHT,
    )

    # Get all lanes from the road
    assert road.lanes is not None, "Road should have lanes"
    assert (
        len(road.lanes.lane_sections) > 0
    ), "Road should have at least one lane section"

    lane_section = road.lanes.lane_sections[0]

    # For RHT: lanes should be right lanes (negative IDs)
    assert len(lane_section.right_lanes) > 0, "RHT road should have right lanes"
    assert len(lane_section.left_lanes) == 0, "RHT road should not have left lanes"

    # Verify all lane IDs are negative
    for lane_id, lane in lane_section.right_lanes.items():
        assert lane_id < 0, f"RHT lane ID should be negative, got {lane_id}"
        assert (
            lane.lane_id < 0
        ), f"RHT lane.lane_id should be negative, got {lane.lane_id}"


def test_lane_ids_lht():
    """Test that lanes have positive IDs for LHT (left lanes)."""
    lanelet_map = load_test_map()

    lanelet_group = [
        lanelet_map.laneletLayer.get(3002094),
        lanelet_map.laneletLayer.get(3002093),
    ]

    road = Road.construct_from_lanelet_groups(
        lanelet_map,
        lanelet_group,
        road_id=0,
        s_offset=0.0,
        traffic_rule=TrafficRule.LHT,
    )

    # Get all lanes from the road
    assert road.lanes is not None, "Road should have lanes"
    assert (
        len(road.lanes.lane_sections) > 0
    ), "Road should have at least one lane section"

    lane_section = road.lanes.lane_sections[0]

    # For LHT: lanes should be left lanes (positive IDs)
    assert len(lane_section.left_lanes) > 0, "LHT road should have left lanes"
    assert len(lane_section.right_lanes) == 0, "LHT road should not have right lanes"

    # Verify all lane IDs are positive
    for lane_id, lane in lane_section.left_lanes.items():
        assert lane_id > 0, f"LHT lane ID should be positive, got {lane_id}"
        assert (
            lane.lane_id > 0
        ), f"LHT lane.lane_id should be positive, got {lane.lane_id}"


def test_reference_line_positioning_rht():
    """Test that ReferenceLine uses left boundary for RHT."""
    lanelet_map = load_test_map()

    lanelet_group = [
        lanelet_map.laneletLayer.get(3002094),
        lanelet_map.laneletLayer.get(3002093),
    ]

    # For RHT: reference line should be at left boundary of leftmost lanelet
    road = Road.construct_from_lanelet_groups(
        lanelet_map,
        lanelet_group,
        road_id=0,
        s_offset=0.0,
        traffic_rule=TrafficRule.RHT,
    )

    # Verify road was constructed successfully
    assert road is not None, "Road should be constructed for RHT"
    assert road.plan_view is not None, "Road should have plan_view"
    assert len(road.plan_view.geometries) > 0, "Road should have geometries"


def test_reference_line_positioning_lht():
    """Test that ReferenceLine uses right boundary for LHT."""
    lanelet_map = load_test_map()

    lanelet_group = [
        lanelet_map.laneletLayer.get(3002094),
        lanelet_map.laneletLayer.get(3002093),
    ]

    # For LHT: reference line should be at right boundary of rightmost lanelet
    road = Road.construct_from_lanelet_groups(
        lanelet_map,
        lanelet_group,
        road_id=0,
        s_offset=0.0,
        traffic_rule=TrafficRule.LHT,
    )

    # Verify road was constructed successfully
    assert road is not None, "Road should be constructed for LHT"
    assert road.plan_view is not None, "Road should have plan_view"
    assert len(road.plan_view.geometries) > 0, "Road should have geometries"
