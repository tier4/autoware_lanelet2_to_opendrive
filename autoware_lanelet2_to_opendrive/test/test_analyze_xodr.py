"""Tests for analyze_xodr helpers."""

from __future__ import annotations

import math

import pytest

from autoware_lanelet2_to_opendrive.analyze_xodr import _evaluate_road_xy
from autoware_lanelet2_to_opendrive.opendrive.geometry import Arc, Line, PlanView
from autoware_lanelet2_to_opendrive.opendrive.road import Road


class TestEvaluateRoadXy:
    """``_evaluate_road_xy`` must evaluate line and arc geometry, not only
    paramPoly3 (#502).

    The analyze/QC path reconstructs roads from the XODR; once
    ``parse_roads_from_xodr`` keeps ``<arc>`` and ``<line>`` segments, the
    Frenet-to-world evaluator has to handle them too — previously it read
    paramPoly3 coefficients directly and would fail on other primitives.
    """

    def test_arc_geometry_evaluated_along_its_curve(self) -> None:
        # Quarter circle of radius 10 m, starting at the origin heading +x.
        radius = 10.0
        curvature = 1.0 / radius
        length = radius * math.pi / 2.0
        road = Road(
            id=1,
            plan_view=PlanView(
                geometries=[
                    Arc(
                        s=0.0,
                        x=0.0,
                        y=0.0,
                        hdg=0.0,
                        length=length,
                        curvature=curvature,
                    )
                ]
            ),
        )

        # A point partway along the arc lies on the analytic circle.
        p = length / 3.0
        assert _evaluate_road_xy(road, p) == pytest.approx(
            (
                math.sin(curvature * p) / curvature,
                (1.0 - math.cos(curvature * p)) / curvature,
            ),
            abs=1e-6,
        )
        # The quarter circle ends exactly at (radius, radius).
        assert _evaluate_road_xy(road, length) == pytest.approx(
            (radius, radius), abs=1e-6
        )

    def test_line_geometry_with_lateral_offset(self) -> None:
        # A 20 m line heading +x from the origin.
        road = Road(
            id=2,
            plan_view=PlanView(
                geometries=[Line(s=0.0, x=0.0, y=0.0, hdg=0.0, length=20.0)]
            ),
        )

        assert _evaluate_road_xy(road, 5.0) == pytest.approx((5.0, 0.0), abs=1e-6)
        # Positive t is to the left of travel; heading +x -> left is +y.
        assert _evaluate_road_xy(road, 5.0, 3.0) == pytest.approx((5.0, 3.0), abs=1e-6)
