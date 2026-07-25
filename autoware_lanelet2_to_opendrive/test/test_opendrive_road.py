"""Tests for OpenDRIVE lane section functions."""

import math
from pathlib import Path

import lanelet2
import lxml.etree as ET
import pytest

from autoware_lanelet2_to_opendrive.conversion_config import ParamPoly3Config
from autoware_lanelet2_to_opendrive.opendrive.road import Road
from autoware_lanelet2_to_opendrive.road_lanelet_geo_mapping import (
    parse_roads_from_xodr,
)


def _make_short_valid_lanelet(
    *,
    length: float = 0.3,
    width: float = 2.0,
) -> tuple[lanelet2.core.LaneletMap, lanelet2.core.Lanelet]:
    """Build a valid road lanelet whose reference line is shorter than 0.5 m."""
    left_points = [
        lanelet2.core.Point3d(lanelet2.core.getId(), 0.0, 0.0, 0.0),
        lanelet2.core.Point3d(lanelet2.core.getId(), length, 0.0, 0.0),
    ]
    right_points = [
        lanelet2.core.Point3d(lanelet2.core.getId(), 0.0, width, 0.0),
        lanelet2.core.Point3d(lanelet2.core.getId(), length, width, 0.0),
    ]
    left_bound = lanelet2.core.LineString3d(lanelet2.core.getId(), left_points)
    right_bound = lanelet2.core.LineString3d(lanelet2.core.getId(), right_points)
    lanelet = lanelet2.core.Lanelet(lanelet2.core.getId(), left_bound, right_bound)
    lanelet.attributes["subtype"] = "road"
    lanelet.attributes["one_way"] = "yes"

    lanelet_map = lanelet2.core.LaneletMap()
    lanelet_map.add(lanelet)
    return lanelet_map, lanelet


def test_construct_road_from_two_lanes(lanelet_map):
    """Test constructing a Road from two adjacent lanelets."""

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


def test_short_valid_lanelet_emits_non_empty_planview() -> None:
    """Sub-minimum but valid lanelets must produce schema-valid road geometry."""
    lanelet_map, lanelet = _make_short_valid_lanelet(length=0.3, width=2.0)
    parampoly3_config = ParamPoly3Config(
        min_segment_length=0.5,
        default_segment_length=1.0,
        max_segments=100,
        min_segments=1,
        enabled=True,
    )

    road = Road.construct_from_lanelet_groups(
        lanelet_map,
        [lanelet],
        road_id=7,
        s_offset=0.0,
        traffic_rule="RHT",
        parampoly3_config=parampoly3_config,
    )

    assert road.length > 0.0
    assert road.length < parampoly3_config.min_segment_length
    assert road.plan_view is not None
    assert len(road.plan_view.geometries) == 1

    geometry = road.plan_view.geometries[0]
    assert geometry.length == pytest.approx(road.length)
    assert geometry.length > 0.0
    assert all(
        math.isfinite(value)
        for value in (
            geometry.s,
            geometry.x,
            geometry.y,
            geometry.hdg,
            geometry.length,
        )
    )

    assert road.elevation_profile is not None
    assert len(road.elevation_profile.elevations) >= 1

    assert road.lanes is not None
    lane_section = road.lanes.lane_sections[0]
    lane = lane_section.right_lanes[-1]
    assert lane.lanelet_id == lanelet.id
    assert lane.widths
    assert lane.widths[0].a == pytest.approx(2.0, abs=1e-2)

    root = ET.Element("OpenDRIVE")
    root.append(road.to_xml())
    parsed_roads = parse_roads_from_xodr(Path("unused.xodr"), xodr_root=root)
    assert [parsed_road.id for parsed_road in parsed_roads] == [7]


def test_elevation_profile_extraction(lanelet_map):
    """Test that elevation profile is extracted from lanelets."""

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


def test_elevation_profile_xml_output(lanelet_map):
    """Test that elevation profile can be converted to XML."""

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


def test_road_construction_rht(lanelet_map):
    """Test Road construction with RHT creates correct lane structure."""
    lanelet_group = [
        lanelet_map.laneletLayer.get(3002094),
        lanelet_map.laneletLayer.get(3002093),
    ]

    road = Road.construct_from_lanelet_groups(
        lanelet_map, lanelet_group, road_id=0, s_offset=0.0, traffic_rule="RHT"
    )

    # Verify road was created
    assert road is not None
    assert road.id == 0

    # Verify road has lanes object
    assert road.lanes is not None
    assert len(road.lanes.lane_sections) > 0

    # Check first lane section has correct structure for RHT
    lane_section = road.lanes.lane_sections[0]
    assert len(lane_section.left_lanes) == 0
    assert len(lane_section.right_lanes) == 2

    # Check lane IDs are negative for RHT
    assert -1 in lane_section.right_lanes
    assert -2 in lane_section.right_lanes


def test_road_construction_lht(lanelet_map):
    """Test Road construction with LHT creates correct lane structure."""
    lanelet_group = [
        lanelet_map.laneletLayer.get(3002094),
        lanelet_map.laneletLayer.get(3002093),
    ]

    road = Road.construct_from_lanelet_groups(
        lanelet_map, lanelet_group, road_id=0, s_offset=0.0, traffic_rule="LHT"
    )

    # Verify road was created
    assert road is not None
    assert road.id == 0

    # Verify road has lanes object
    assert road.lanes is not None
    assert len(road.lanes.lane_sections) > 0

    # Check first lane section has correct structure for LHT
    # LHT: driving lanes are in the left section with positive IDs (+1, +2, ...)
    lane_section = road.lanes.lane_sections[0]
    assert len(lane_section.right_lanes) == 0
    assert len(lane_section.left_lanes) == 2

    # Check lane IDs are positive for LHT
    assert 1 in lane_section.left_lanes
    assert 2 in lane_section.left_lanes
