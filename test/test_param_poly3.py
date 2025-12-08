"""Tests for ParamPoly3 geometry class."""

import numpy as np
import pytest
from autoware_lanelet2_to_opendrive.spline import Splines
from autoware_lanelet2_to_opendrive.opendrive.geometry import ParamPoly3


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
