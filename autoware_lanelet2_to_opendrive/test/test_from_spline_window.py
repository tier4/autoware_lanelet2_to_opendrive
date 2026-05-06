"""Unit tests for the from_spline_window family of helpers (#466)."""

import numpy as np
import pytest

from autoware_lanelet2_to_opendrive.opendrive.geometry import (
    Arc,
    Line,
    ParamPoly3,
)
from autoware_lanelet2_to_opendrive.spline import Splines


def _straight_spline(length: float = 100.0, n: int = 50) -> Splines:
    """Build a Splines that fits a straight line along +x."""
    pts = np.column_stack(
        [np.linspace(0.0, length, n), np.zeros(n), np.zeros(n)]
    )
    return Splines(pts)


def _circle_spline(radius: float = 50.0, arc: float = np.pi / 2.0, n: int = 80) -> Splines:
    """Build a Splines that fits a circular arc starting at (radius, 0) heading +y."""
    theta = np.linspace(0.0, arc, n)
    # Centred at origin, parameterised so heading at theta=0 is +y.
    pts = np.column_stack(
        [radius * np.cos(theta), radius * np.sin(theta), np.zeros(n)]
    )
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
