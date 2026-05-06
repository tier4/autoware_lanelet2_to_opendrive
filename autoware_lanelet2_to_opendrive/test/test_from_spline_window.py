"""Unit tests for the from_spline_window family of helpers (#466)."""

import numpy as np
import pytest

from autoware_lanelet2_to_opendrive.opendrive.geometry import (
    Arc,
    Line,
)
from autoware_lanelet2_to_opendrive.spline import Splines


def _straight_spline(length: float = 100.0, n: int = 50) -> Splines:
    """Build a Splines that fits a straight line along +x."""
    pts = np.column_stack([np.linspace(0.0, length, n), np.zeros(n), np.zeros(n)])
    return Splines(pts)


def _circle_spline(
    radius: float = 50.0, arc: float = np.pi / 2.0, n: int = 80
) -> Splines:
    """Build a Splines that fits a circular arc starting at (radius, 0) heading +y."""
    theta = np.linspace(0.0, arc, n)
    # Centred at origin, parameterised so heading at theta=0 is +y.
    pts = np.column_stack([radius * np.cos(theta), radius * np.sin(theta), np.zeros(n)])
    return Splines(pts)


class TestLineFromSplineWindow:
    def test_records_start_position_and_heading(self):
        spline = _straight_spline()
        line = Line.from_spline_window(spline, s_start=10.0, s_end=40.0)
        assert line.s == pytest.approx(10.0)
        assert line.length == pytest.approx(30.0)
        assert line.x == pytest.approx(10.0, abs=1e-6)
        assert line.y == pytest.approx(0.0, abs=1e-6)
        assert line.hdg == pytest.approx(0.0, abs=1e-6)

    def test_emits_line_xml(self):
        spline = _straight_spline()
        line = Line.from_spline_window(spline, s_start=0.0, s_end=10.0)
        xml = line.to_xml()
        assert xml.find("line") is not None
        assert xml.find("arc") is None
        assert xml.find("paramPoly3") is None


class TestArcFromSplineWindow:
    def test_records_start_position_heading_and_curvature(self):
        spline = _circle_spline(radius=50.0, arc=np.pi / 2.0)
        # s_start=0 of the arc corresponds to (50, 0) heading +y.
        arc = Arc.from_spline_window(
            spline, s_start=0.0, s_end=10.0, curvature=1.0 / 50.0
        )
        assert arc.curvature == pytest.approx(1.0 / 50.0)
        assert arc.length == pytest.approx(10.0)
        assert arc.x == pytest.approx(50.0, abs=1e-3)
        assert arc.y == pytest.approx(0.0, abs=1e-3)
        # Spline parametrised with theta=0 at +x axis → tangent at s=0 is +y.
        # Tolerance is relaxed to 2e-2 rad to accommodate the discrete-point
        # spline approximation of a perfect circle.
        assert arc.hdg == pytest.approx(np.pi / 2.0, abs=2e-2)

    def test_emits_arc_xml(self):
        spline = _circle_spline()
        arc = Arc.from_spline_window(spline, s_start=0.0, s_end=5.0, curvature=0.02)
        xml = arc.to_xml()
        arc_elem = xml.find("arc")
        assert arc_elem is not None
        assert float(arc_elem.get("curvature")) == pytest.approx(0.02)
