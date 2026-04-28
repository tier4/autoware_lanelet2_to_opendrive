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
