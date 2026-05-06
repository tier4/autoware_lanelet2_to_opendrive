"""Unit tests for opendrive.geometry_classifier (#466)."""

import numpy as np
import pytest

from autoware_lanelet2_to_opendrive.opendrive.geometry_classifier import (
    ArcRun,
    ClassifiedSegment,
    LineRun,
    ParamPoly3Run,
)


class TestClassifiedSegment:
    def test_line_run_fields(self):
        run = LineRun(s_start=0.0, s_end=10.0)
        assert run.s_start == 0.0 and run.s_end == 10.0
        assert isinstance(run, ClassifiedSegment)

    def test_arc_run_fields(self):
        run = ArcRun(s_start=10.0, s_end=20.0, curvature=0.05)
        assert run.curvature == 0.05
        assert isinstance(run, ClassifiedSegment)

    def test_param_poly3_run_fields(self):
        run = ParamPoly3Run(s_start=20.0, s_end=30.0)
        assert run.s_end == 30.0
        assert isinstance(run, ClassifiedSegment)


def _build_straight(length: float = 100.0, n: int = 50):
    from autoware_lanelet2_to_opendrive.spline import Splines

    pts = np.column_stack([np.linspace(0.0, length, n), np.zeros(n), np.zeros(n)])
    return Splines(pts)


def _build_circle(radius: float, arc_rad: float, n: int = 80):
    from autoware_lanelet2_to_opendrive.spline import Splines

    theta = np.linspace(0.0, arc_rad, n)
    pts = np.column_stack([radius * np.cos(theta), radius * np.sin(theta), np.zeros(n)])
    return Splines(pts)


class TestGrowLine:
    def test_extends_to_total_length_for_pure_line(self):
        from autoware_lanelet2_to_opendrive.opendrive.geometry_classifier import (
            _grow_line,
        )
        from autoware_lanelet2_to_opendrive.conversion_config import (
            ArcSpiralConfig,
        )
        from autoware_lanelet2_to_opendrive.config import DEFAULT_CONFIG

        spline = _build_straight(length=100.0)
        end = _grow_line(
            spline,
            s_start=0.0,
            s_max=spline.total_length,
            config=ArcSpiralConfig(enabled=True),
            constants=DEFAULT_CONFIG.arcspiral,
        )
        assert end == pytest.approx(spline.total_length, abs=0.51)

    def test_returns_s_start_for_curve(self):
        from autoware_lanelet2_to_opendrive.opendrive.geometry_classifier import (
            _grow_line,
        )
        from autoware_lanelet2_to_opendrive.conversion_config import (
            ArcSpiralConfig,
        )
        from autoware_lanelet2_to_opendrive.config import DEFAULT_CONFIG

        spline = _build_circle(radius=20.0, arc_rad=np.pi / 2.0)
        end = _grow_line(
            spline,
            s_start=0.0,
            s_max=spline.total_length,
            config=ArcSpiralConfig(enabled=True),
            constants=DEFAULT_CONFIG.arcspiral,
        )
        # Curve is too tight to qualify as a line at all.
        assert end - 0.0 < 1.0
