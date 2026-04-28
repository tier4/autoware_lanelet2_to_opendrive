"""Tests for get_lane_outer_edge_t_at_s and get_reference_line_tangent_at_s."""

from __future__ import annotations

import math

import numpy as np
import pytest

from autoware_lanelet2_to_opendrive.cubic_spline_1d import CubicSpline1D
from autoware_lanelet2_to_opendrive.opendrive.enums import LaneType
from autoware_lanelet2_to_opendrive.opendrive.lane import Lane
from autoware_lanelet2_to_opendrive.opendrive.lane_elements import LaneWidth
from autoware_lanelet2_to_opendrive.opendrive.lane_section import LaneSection
from autoware_lanelet2_to_opendrive.opendrive.lane_sections import Lanes
from autoware_lanelet2_to_opendrive.opendrive.reference_line import ReferenceLine
from autoware_lanelet2_to_opendrive.opendrive.road import (
    Road,
    get_lane_outer_edge_t_at_s,
    get_reference_line_tangent_at_s,
)
from autoware_lanelet2_to_opendrive.spline import Splines


def _make_road_with_constant_widths(widths: list[float], length: float = 10.0) -> Road:
    """Build a synthetic RHT Road with given negative-side lane widths."""
    points = np.array([[i * length / 10, 0.0] for i in range(11)])
    spline = Splines(points, num_control_points=11)
    height = CubicSpline1D(np.array([0.0, length]), np.array([0.0, 0.0]))
    ref = ReferenceLine(centerline_2d=spline, height_spline=height)

    section = LaneSection(s_offset=0.0)
    section._set_center_lane(ref)
    for i, width in enumerate(widths, start=1):
        lane = Lane(lane_id=-i, lane_type=LaneType.DRIVING, level=False)
        lane.widths = [LaneWidth(s_offset=0.0, a=width, b=0.0, c=0.0, d=0.0)]
        section._add_right_lane(lane)

    lanes = Lanes(lane_sections=[section])
    return Road(
        id=0,
        name="test",
        length=length,
        junction=-1,
        rule=None,
        plan_view=None,
        elevation_profile=None,
        lanes=lanes,
        elevation_offset=0.0,
        road_types=[],
    )


def test_get_lane_outer_edge_t_rht_first_lane() -> None:
    road = _make_road_with_constant_widths([3.0])
    t = get_lane_outer_edge_t_at_s(road, lane_id=-1, s=5.0)
    assert t == pytest.approx(-3.0, abs=1e-9)


def test_get_lane_outer_edge_t_rht_second_lane_accumulates() -> None:
    road = _make_road_with_constant_widths([3.0, 2.5])
    t = get_lane_outer_edge_t_at_s(road, lane_id=-2, s=5.0)
    assert t == pytest.approx(-(3.0 + 2.5), abs=1e-9)


def test_get_reference_line_tangent_horizontal() -> None:
    road = _make_road_with_constant_widths([3.0])
    theta = get_reference_line_tangent_at_s(road, s=5.0)
    assert theta == pytest.approx(0.0, abs=1e-6)


def test_get_reference_line_tangent_diagonal() -> None:
    points = np.array([[i, i] for i in range(11)])  # 45° line y=x
    spline = Splines(points, num_control_points=11)
    height = CubicSpline1D(np.array([0.0, spline.total_length]), np.array([0.0, 0.0]))
    ref = ReferenceLine(centerline_2d=spline, height_spline=height)
    section = LaneSection(s_offset=0.0)
    section._set_center_lane(ref)
    lanes = Lanes(lane_sections=[section])
    road = Road(
        id=0,
        name="test",
        length=spline.total_length,
        junction=-1,
        rule=None,
        plan_view=None,
        elevation_profile=None,
        lanes=lanes,
        elevation_offset=0.0,
        road_types=[],
    )
    theta = get_reference_line_tangent_at_s(road, s=spline.total_length / 2)
    assert theta == pytest.approx(math.pi / 4, abs=1e-3)


import lanelet2  # noqa: E402
from lanelet2.core import (  # noqa: E402
    Lanelet,
    LineString3d,
    Point3d,
    getId,
)


def _make_lanelet_for_road(
    map_obj: lanelet2.core.LaneletMap,
    x_start: float,
    x_end: float,
    left_y: float,
    right_y: float,
    n: int = 5,
) -> Lanelet:
    left_pts = [
        Point3d(getId(), x_start + i * (x_end - x_start) / (n - 1), left_y, 0.0)
        for i in range(n)
    ]
    right_pts = [
        Point3d(getId(), x_start + i * (x_end - x_start) / (n - 1), right_y, 0.0)
        for i in range(n)
    ]
    left_ls = LineString3d(getId(), left_pts)
    right_ls = LineString3d(getId(), right_pts)
    lanelet = Lanelet(getId(), left_ls, right_ls)
    lanelet.attributes["subtype"] = "road"
    map_obj.add(lanelet)
    return lanelet


def test_road_construct_with_regular_at_start_uses_border() -> None:
    """When regular_road_at_start is supplied, the road emits <border> lanes."""
    lanelet_map = lanelet2.core.LaneletMap()
    upstream_lanelet = _make_lanelet_for_road(
        lanelet_map, 0.0, 10.0, left_y=0.0, right_y=-3.0
    )
    connecting_lanelet = _make_lanelet_for_road(
        lanelet_map, 10.0, 20.0, left_y=0.0, right_y=-3.0
    )

    upstream_road = Road.construct_from_lanelet_groups(
        lanelet_map=lanelet_map,
        lanelet_group=[upstream_lanelet],
        road_id=0,
        traffic_rule="RHT",
    )

    connecting_road = Road.construct_from_lanelet_groups(
        lanelet_map=lanelet_map,
        lanelet_group=[connecting_lanelet],
        road_id=1,
        traffic_rule="RHT",
        regular_road_at_start=upstream_road,
    )

    lane = connecting_road.lanes.lane_sections[0].right_lanes[-1]
    assert len(lane.borders) >= 1, "BORDER mode should populate borders"
    assert lane.widths == [], "BORDER mode should leave widths empty"


def test_road_construct_without_regular_road_uses_width() -> None:
    """When neither regular_road_at_start nor _at_end is supplied, WIDTH mode is used."""
    lanelet_map = lanelet2.core.LaneletMap()
    lanelet = _make_lanelet_for_road(lanelet_map, 0.0, 10.0, left_y=0.0, right_y=-3.0)

    road = Road.construct_from_lanelet_groups(
        lanelet_map=lanelet_map,
        lanelet_group=[lanelet],
        road_id=0,
        traffic_rule="RHT",
    )
    lane = road.lanes.lane_sections[0].right_lanes[-1]
    assert len(lane.widths) >= 1
    assert lane.borders == []
