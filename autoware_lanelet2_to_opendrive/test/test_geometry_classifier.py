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
