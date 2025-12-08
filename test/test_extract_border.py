"""Tests for extract_border_from_spline function."""

import pytest
from autoware_lanelet2_to_opendrive.centerline import extract_border_from_spline
from autoware_lanelet2_to_opendrive.spline import Splines


class TestExtractBorderFromSpline:
    """Tests for extract_border_from_spline function."""

    def create_mock_lanelet(self):
        """Create a mock lanelet-like object for testing."""

        class MockPoint:
            def __init__(self, x, y, z=0.0):
                self.x = x
                self.y = y
                self.z = z

        class MockLanelet:
            def __init__(self):
                # Create left and right boundaries
                self.leftBound = [
                    MockPoint(0.0, 1.0, 0.0),
                    MockPoint(1.0, 1.0, 0.0),
                    MockPoint(2.0, 1.0, 0.0),
                    MockPoint(3.0, 1.0, 0.0),
                ]
                self.rightBound = [
                    MockPoint(0.0, -1.0, 0.0),
                    MockPoint(1.0, -1.0, 0.0),
                    MockPoint(2.0, -1.0, 0.0),
                    MockPoint(3.0, -1.0, 0.0),
                ]

        return MockLanelet()

    def test_extract_left_border(self):
        """Test extracting left border from lanelet."""
        lanelet = self.create_mock_lanelet()

        spline = extract_border_from_spline(lanelet, "left", num_control_points=6)

        assert isinstance(spline, Splines)
        assert spline.total_length > 0

        # Check start and end positions
        start_pos = spline.evaluate(0.0)
        end_pos = spline.evaluate(spline.total_length)

        # Should match left boundary points
        assert abs(start_pos[0] - 0.0) < 1e-6
        assert abs(start_pos[1] - 1.0) < 1e-6
        assert abs(end_pos[0] - 3.0) < 1e-2
        assert abs(end_pos[1] - 1.0) < 1e-2

    def test_extract_right_border(self):
        """Test extracting right border from lanelet."""
        lanelet = self.create_mock_lanelet()

        spline = extract_border_from_spline(lanelet, "right", num_control_points=6)

        assert isinstance(spline, Splines)
        assert spline.total_length > 0

        # Check start and end positions
        start_pos = spline.evaluate(0.0)
        end_pos = spline.evaluate(spline.total_length)

        # Should match right boundary points
        assert abs(start_pos[0] - 0.0) < 1e-6
        assert abs(start_pos[1] + 1.0) < 1e-6  # +1.0 because y = -1.0
        assert abs(end_pos[0] - 3.0) < 1e-2
        assert abs(end_pos[1] + 1.0) < 1e-2

    def test_invalid_border_raises_error(self):
        """Test that invalid border specification raises ValueError."""
        lanelet = self.create_mock_lanelet()

        with pytest.raises(
            ValueError, match="Invalid border: center. Must be 'left' or 'right'"
        ):
            extract_border_from_spline(lanelet, "center")

    def test_insufficient_points_raises_error(self):
        """Test that insufficient points in boundary raises ValueError."""

        class MockPoint:
            def __init__(self, x, y, z=0.0):
                self.x = x
                self.y = y
                self.z = z

        class MockLanelet:
            def __init__(self):
                self.leftBound = [MockPoint(0.0, 0.0, 0.0)]  # Only one point
                self.rightBound = [MockPoint(0.0, -1.0, 0.0)]

        lanelet = MockLanelet()

        with pytest.raises(
            ValueError, match="Lanelet must have at least 2 points in its left boundary"
        ):
            extract_border_from_spline(lanelet, "left")

    def test_different_num_control_points(self):
        """Test with different numbers of control points."""
        lanelet = self.create_mock_lanelet()

        # Test with fewer control points (smoother)
        spline_smooth = extract_border_from_spline(
            lanelet, "left", num_control_points=4
        )

        # Test with more control points (more detailed)
        spline_detailed = extract_border_from_spline(
            lanelet, "left", num_control_points=12
        )

        # Both should be valid
        assert isinstance(spline_smooth, Splines)
        assert isinstance(spline_detailed, Splines)
        assert spline_smooth.total_length > 0
        assert spline_detailed.total_length > 0

    def test_coordinate_validation(self):
        """Test coordinate validation for extracted border."""
        lanelet = self.create_mock_lanelet()

        spline = extract_border_from_spline(lanelet, "left", num_control_points=8)

        # Test specific positions along the border
        pos_0 = spline.evaluate(0.0)
        pos_mid = spline.evaluate(spline.total_length * 0.5)
        pos_end = spline.evaluate(spline.total_length)

        # For this straight left boundary at y=1.0
        assert abs(pos_0[1] - 1.0) < 1e-6
        assert (
            abs(pos_mid[1] - 1.0) < 0.2
        )  # Should stay reasonably close to y=1.0 (B-spline approximation)
        assert abs(pos_end[1] - 1.0) < 1e-2

        # X coordinates should progress from 0 to 3
        assert abs(pos_0[0] - 0.0) < 1e-6
        assert 0.0 <= pos_mid[0] <= 3.0
        assert abs(pos_end[0] - 3.0) < 1e-2
