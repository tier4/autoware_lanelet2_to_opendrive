"""Tests for ParamPoly3 geometry class."""

import sys

import numpy as np
import pytest
from autoware_lanelet2_to_opendrive.spline import Splines
from autoware_lanelet2_to_opendrive.opendrive.geometry import ParamPoly3
from autoware_lanelet2_to_opendrive.opendrive.xml_utils import replace_subnormal


class TestParamPoly3FromSpline:
    """Tests for ParamPoly3.from_spline method."""

    def test_from_spline_basic(self):
        """Test basic ParamPoly3.from_spline functionality."""
        # Create test points for a simple curve
        points = np.array(
            [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 1.0, 0.0], [3.0, 1.0, 0.0]]
        )

        # Create Splines object
        spline = Splines(points, num_control_points=8)

        # Create ParamPoly3 list from spline
        param_polys = ParamPoly3.from_spline(spline, num_segments=5)

        # Verify basic properties
        assert len(param_polys) == 5
        assert all(isinstance(poly, ParamPoly3) for poly in param_polys)
        assert all(poly.pRange == "arcLength" for poly in param_polys)

        # Verify total length matches spline
        total_poly_length = sum(poly.length for poly in param_polys)
        assert abs(total_poly_length - spline.total_length) < 1e-10

        # Verify segments are continuous (each starts where previous ends)
        for i in range(1, len(param_polys)):
            prev_end_s = param_polys[i - 1].s + param_polys[i - 1].length
            assert abs(param_polys[i].s - prev_end_s) < 1e-10

    def test_from_spline_straight_line(self):
        """Test ParamPoly3.from_spline with straight line."""
        # Create straight line points
        points = np.array(
            [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [3.0, 0.0, 0.0]]
        )

        # Create Splines object with explicit tangents for straight line
        start_vel = np.array([1.0, 0.0, 0.0])
        end_vel = np.array([1.0, 0.0, 0.0])
        spline = Splines(points, start_vel=start_vel, end_vel=end_vel)

        # Create ParamPoly3 list from spline
        param_polys = ParamPoly3.from_spline(spline, num_segments=3)

        # Verify basic properties
        assert len(param_polys) == 3

        # For a straight horizontal line, V coefficients should be small
        for poly in param_polys:
            assert abs(poly.aV) < 0.1  # V offset should be small
            assert abs(poly.bV) < 0.1  # V velocity should be small
            assert abs(poly.cV) < 0.1  # V should stay small
            assert abs(poly.dV) < 0.1

    def test_from_spline_single_segment(self):
        """Test ParamPoly3.from_spline with single segment."""
        points = np.array([[0.0, 0.0, 0.0], [1.0, 1.0, 0.0]])

        spline = Splines(points, num_control_points=4)
        param_polys = ParamPoly3.from_spline(spline, num_segments=1)

        assert len(param_polys) == 1
        poly = param_polys[0]

        # Verify segment covers entire spline
        assert abs(poly.s - 0.0) < 1e-10
        assert abs(poly.length - spline.total_length) < 1e-6

    def test_from_spline_zero_length_error(self):
        """Test ParamPoly3.from_spline with zero length spline raises error."""
        # Create points that are all the same (zero length)
        points = np.array([[1.0, 1.0, 0.0], [1.0, 1.0, 0.0]])

        try:
            spline = Splines(points, num_control_points=4)
            # This might work but have zero length
            if spline.total_length <= 0:
                with pytest.raises(ValueError, match="Spline has zero length"):
                    ParamPoly3.from_spline(spline, num_segments=3)
        except Exception:
            # If spline creation fails, that's also acceptable for this degenerate case
            pass

    def test_from_spline_xml_output(self):
        """Test that ParamPoly3 from spline can be converted to XML."""
        points = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]])

        spline = Splines(points, num_control_points=4)
        param_polys = ParamPoly3.from_spline(spline, num_segments=2)

        # Verify XML conversion works
        for poly in param_polys:
            xml_elem = poly.to_xml()
            assert xml_elem.tag == "geometry"

            # Check that paramPoly3 child element exists
            poly3_elem = xml_elem.find("paramPoly3")
            assert poly3_elem is not None
            assert poly3_elem.get("pRange") == "arcLength"


class TestParamPoly3DynamicSegments:
    """Tests for dynamic segment calculation in ParamPoly3.from_spline."""

    def test_dynamic_segments_short_road(self):
        """Test that short roads get appropriate segment count."""
        # Create a very short road (0.53m - the problematic case from issue)
        points = np.array(
            [
                [0.0, 0.0, 0.0],
                [0.53, 0.0, 0.0],
            ]
        )

        spline = Splines(points, num_control_points=4)

        # Use dynamic calculation (num_segments=None is default)
        param_polys = ParamPoly3.from_spline(spline)

        # Should create only 1 segment (0.53m > 0.5m minimum)
        assert len(param_polys) == 1
        assert param_polys[0].length >= 0.5
        assert param_polys[0].length == pytest.approx(spline.total_length, rel=1e-3)

    def test_dynamic_segments_medium_road(self):
        """Test that medium roads get appropriate segment count."""
        # Create a medium-length road (10m)
        points = np.array(
            [
                [0.0, 0.0, 0.0],
                [5.0, 0.0, 0.0],
                [10.0, 0.0, 0.0],
            ]
        )

        spline = Splines(points, num_control_points=6)
        param_polys = ParamPoly3.from_spline(spline)

        # Should create ~10 segments (10m / 1m target = 10)
        assert 8 <= len(param_polys) <= 12  # Allow some tolerance

        # All segments should be above minimum length
        for poly in param_polys:
            assert poly.length >= 0.5

    def test_dynamic_segments_long_road(self):
        """Test that long roads don't create excessive segments."""
        # Create a long road (200m)
        points = np.linspace([0, 0, 0], [200, 0, 0], 100)

        spline = Splines(points, num_control_points=20)
        param_polys = ParamPoly3.from_spline(spline)

        # Should be capped at max_segments (100)
        assert len(param_polys) <= 100

    def test_manual_override_still_works(self):
        """Test that manual num_segments still works (backward compatibility)."""
        points = np.array(
            [
                [0.0, 0.0, 0.0],
                [10.0, 0.0, 0.0],
            ]
        )

        spline = Splines(points, num_control_points=6)

        # Manually specify 5 segments
        param_polys = ParamPoly3.from_spline(spline, num_segments=5)

        # Should respect manual override
        assert len(param_polys) == 5

    def test_segment_length_never_below_minimum(self):
        """Test that no segment is ever below minimum length."""
        # Create various road lengths
        test_lengths = [0.53, 1.0, 5.0, 10.0, 50.0, 150.0]

        for length in test_lengths:
            points = np.array(
                [
                    [0.0, 0.0, 0.0],
                    [length, 0.0, 0.0],
                ]
            )

            spline = Splines(points, num_control_points=4)
            param_polys = ParamPoly3.from_spline(spline)

            # Verify all segments meet minimum length
            for poly in param_polys:
                assert (
                    poly.length >= 0.5
                ), f"Segment length {poly.length:.6f}m below 0.5m for road length {length}m"

    def test_coefficient_normalization(self):
        """Test that very small coefficients are normalized to zero."""
        points = np.array(
            [
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],  # Straight line
            ]
        )

        spline = Splines(points, num_control_points=4)
        param_polys = ParamPoly3.from_spline(spline)

        # For a straight horizontal line, V coefficients should be very small
        for poly in param_polys:
            # After normalization, small values should be exactly zero
            assert abs(poly.aV) < 1e-6
            assert abs(poly.bV) < 1e-6

    def test_validation_rejects_invalid_segments(self):
        """Test that validation catches invalid segments."""
        # Create an invalid segment (length too short)
        invalid_segment = ParamPoly3(
            s=0.0,
            x=0.0,
            y=0.0,
            hdg=0.0,
            length=0.01,  # Below 0.5m minimum
            aU=0,
            bU=1,
            cU=0,
            dU=0,
            aV=0,
            bV=0,
            cV=0,
            dV=0,
        )

        is_valid, error_msg = ParamPoly3._validate_segment(invalid_segment)

        assert not is_valid
        assert "below minimum" in error_msg.lower()

    def test_calculate_optimal_num_segments(self):
        """Test the _calculate_optimal_num_segments helper method."""
        # Short road: should return 1
        assert ParamPoly3._calculate_optimal_num_segments(0.53) == 1

        # Medium road: should return ~10
        result = ParamPoly3._calculate_optimal_num_segments(10.0)
        assert 8 <= result <= 12

        # Long road: should be capped at 100
        assert ParamPoly3._calculate_optimal_num_segments(200.0) == 100

        # Zero length: should return min_segments
        assert ParamPoly3._calculate_optimal_num_segments(0.0) == 1


class TestReplaceSubnormal:
    """Tests for replace_subnormal and subnormal handling in to_xml."""

    def test_subnormal_replaced_with_zero(self):
        """Test that subnormal values are replaced with 0.0."""
        assert replace_subnormal(5e-324) == 0.0
        assert replace_subnormal(1e-310) == 0.0
        assert replace_subnormal(-5e-324) == 0.0
        assert replace_subnormal(-1e-310) == 0.0

    def test_zero_unchanged(self):
        """Test that zero remains zero."""
        assert replace_subnormal(0.0) == 0.0
        assert replace_subnormal(-0.0) == 0.0

    def test_normal_values_unchanged(self):
        """Test that normal (non-subnormal) values are not modified."""
        min_normal = sys.float_info.min  # ~2.2250738585072014e-308
        assert replace_subnormal(min_normal) == min_normal
        assert replace_subnormal(1.0) == 1.0
        assert replace_subnormal(-1.0) == -1.0
        assert replace_subnormal(1e-100) == 1e-100

    def test_boundary_at_min_normal(self):
        """Test the boundary between subnormal and normal values."""
        min_normal = sys.float_info.min
        just_below = min_normal / 2.0
        assert replace_subnormal(just_below) == 0.0
        assert replace_subnormal(min_normal) == min_normal

    def test_to_xml_replaces_subnormal_coefficients(self):
        """Test that to_xml outputs 0.0 for subnormal coefficients."""
        subnormal_val = 5e-324
        segment = ParamPoly3(
            s=0.0,
            x=0.0,
            y=0.0,
            hdg=0.0,
            length=1.0,
            aU=subnormal_val,
            bU=1.0,
            cU=-subnormal_val,
            dU=0.0,
            aV=subnormal_val,
            bV=0.0,
            cV=0.0,
            dV=-subnormal_val,
        )

        xml_elem = segment.to_xml()
        poly3_elem = xml_elem.find("paramPoly3")

        assert poly3_elem.get("aU") == "0.0"
        assert poly3_elem.get("bU") == "1.0"
        assert poly3_elem.get("cU") == "0.0"
        assert poly3_elem.get("dU") == "0.0"
        assert poly3_elem.get("aV") == "0.0"
        assert poly3_elem.get("bV") == "0.0"
        assert poly3_elem.get("cV") == "0.0"
        assert poly3_elem.get("dV") == "0.0"

    def test_to_xml_preserves_normal_coefficients(self):
        """Test that to_xml preserves normal coefficient values."""
        segment = ParamPoly3(
            s=0.0,
            x=0.0,
            y=0.0,
            hdg=0.0,
            length=1.0,
            aU=0.0,
            bU=1.0,
            cU=-0.5,
            dU=0.1,
            aV=0.0,
            bV=0.01,
            cV=-0.02,
            dV=0.003,
        )

        xml_elem = segment.to_xml()
        poly3_elem = xml_elem.find("paramPoly3")

        assert poly3_elem.get("aU") == "0.0"
        assert poly3_elem.get("bU") == "1.0"
        assert poly3_elem.get("cU") == "-0.5"
        assert poly3_elem.get("dU") == "0.1"
        assert poly3_elem.get("aV") == "0.0"
        assert poly3_elem.get("bV") == "0.01"
        assert poly3_elem.get("cV") == "-0.02"
        assert poly3_elem.get("dV") == "0.003"


class TestEvaluatePlanViewWorldArc:
    """Tests for arc evaluation in evaluate_plan_view_world (#466)."""

    def test_unit_circle_quarter_arc(self):
        """κ=1, length=π/2, start at origin heading +x → end at (1, 1)."""
        from autoware_lanelet2_to_opendrive.opendrive.geometry import (
            evaluate_plan_view_world,
        )

        wx, wy = evaluate_plan_view_world(
            x=0.0, y=0.0, hdg=0.0, p=np.pi / 2.0, arc_curvature=1.0
        )
        assert wx == pytest.approx(1.0, abs=1e-9)
        assert wy == pytest.approx(1.0, abs=1e-9)

    def test_negative_curvature_quarter_arc(self):
        """κ=-1, length=π/2, start at origin heading +x → end at (1, -1)."""
        from autoware_lanelet2_to_opendrive.opendrive.geometry import (
            evaluate_plan_view_world,
        )

        wx, wy = evaluate_plan_view_world(
            x=0.0, y=0.0, hdg=0.0, p=np.pi / 2.0, arc_curvature=-1.0
        )
        assert wx == pytest.approx(1.0, abs=1e-9)
        assert wy == pytest.approx(-1.0, abs=1e-9)

    def test_zero_curvature_falls_back_to_line(self):
        """κ below epsilon must match the straight-line fallback path."""
        from autoware_lanelet2_to_opendrive.opendrive.geometry import (
            evaluate_plan_view_world,
        )

        wx_arc, wy_arc = evaluate_plan_view_world(
            x=10.0, y=20.0, hdg=np.pi / 4.0, p=5.0, arc_curvature=0.0
        )
        wx_line, wy_line = evaluate_plan_view_world(
            x=10.0, y=20.0, hdg=np.pi / 4.0, p=5.0
        )
        assert wx_arc == pytest.approx(wx_line, abs=1e-12)
        assert wy_arc == pytest.approx(wy_line, abs=1e-12)

    def test_translated_rotated_arc(self):
        """Origin and heading offsets compose correctly with arc evaluation."""
        from autoware_lanelet2_to_opendrive.opendrive.geometry import (
            evaluate_plan_view_world,
        )

        # κ=1, p=π → 180° turn, end is 2 units to the "left" of start tangent.
        wx, wy = evaluate_plan_view_world(
            x=5.0, y=-3.0, hdg=np.pi / 2.0, p=np.pi, arc_curvature=1.0
        )
        # heading +y, turn left → end at (5 - 2, -3)
        assert wx == pytest.approx(3.0, abs=1e-9)
        assert wy == pytest.approx(-3.0, abs=1e-9)

    def test_rejects_both_arc_and_paramPoly3(self):
        """Mutually exclusive kwargs must raise ValueError."""
        from autoware_lanelet2_to_opendrive.opendrive.geometry import (
            evaluate_plan_view_world,
        )

        with pytest.raises(ValueError):
            evaluate_plan_view_world(
                x=0.0,
                y=0.0,
                hdg=0.0,
                p=1.0,
                param_poly3_coeffs=(0, 1, 0, 0, 0, 0, 0, 0),
                arc_curvature=0.5,
            )
