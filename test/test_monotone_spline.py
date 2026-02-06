"""Unit tests for MonotoneSpline1D class."""

import numpy as np
import pytest
from autoware_lanelet2_to_opendrive.monotone_spline_1d import MonotoneSpline1D
from autoware_lanelet2_to_opendrive.cubic_spline_1d import CubicSpline1D


def test_monotone_spline_basic_interpolation():
    """Test basic interpolation: endpoints should match input data."""
    arc_lengths = np.array([0.0, 10.0, 20.0, 30.0])
    values = np.array([3.0, 3.5, 3.2, 4.0])

    spline = MonotoneSpline1D(arc_lengths, values)

    # Check endpoints
    assert abs(spline.evaluate(0.0) - 3.0) < 1e-10, "Start point should match"
    assert abs(spline.evaluate(30.0) - 4.0) < 1e-10, "End point should match"

    # Check intermediate points
    assert (
        abs(spline.evaluate(10.0) - 3.5) < 1e-10
    ), "First intermediate point should match"
    assert (
        abs(spline.evaluate(20.0) - 3.2) < 1e-10
    ), "Second intermediate point should match"


def test_monotone_spline_prevents_overshoot():
    """Test that monotone spline prevents overshoot beyond input range."""
    # Create data that would cause cubic spline to overshoot
    arc_lengths = np.array([0.0, 1.0, 2.0, 3.0, 4.0])
    values = np.array([1.0, 2.0, 1.5, 3.0, 2.5])

    spline = MonotoneSpline1D(arc_lengths, values)

    # Sample densely between input points
    test_points = np.linspace(0.0, 4.0, 100)
    interpolated_values = [spline.evaluate(s) for s in test_points]

    # All interpolated values should be within input range
    min_val = np.min(values)
    max_val = np.max(values)

    assert all(
        min_val - 1e-10 <= v <= max_val + 1e-10 for v in interpolated_values
    ), "All interpolated values should be within input range"


def test_monotone_spline_monotonicity():
    """Test that monotone spline preserves monotonicity."""
    # Monotonically increasing data
    arc_lengths = np.array([0.0, 1.0, 2.0, 3.0, 4.0])
    values = np.array([1.0, 1.5, 2.0, 2.5, 3.0])

    spline = MonotoneSpline1D(arc_lengths, values)

    # Sample densely
    test_points = np.linspace(0.0, 4.0, 100)
    interpolated_values = [spline.evaluate(s) for s in test_points]

    # Check monotonicity (allowing small numerical errors)
    for i in range(len(interpolated_values) - 1):
        assert (
            interpolated_values[i + 1] >= interpolated_values[i] - 1e-10
        ), f"Monotonicity violated at index {i}"


def test_monotone_spline_constant_values():
    """Test monotone spline with constant width."""
    arc_lengths = np.array([0.0, 10.0, 20.0, 30.0])
    values = np.array([3.5, 3.5, 3.5, 3.5])

    spline = MonotoneSpline1D(arc_lengths, values)

    # Sample at various points
    test_points = np.linspace(0.0, 30.0, 50)
    for s in test_points:
        assert (
            abs(spline.evaluate(s) - 3.5) < 1e-10
        ), f"Constant value not preserved at s={s}"


def test_monotone_spline_derivatives():
    """Test derivative evaluation."""
    arc_lengths = np.array([0.0, 10.0, 20.0, 30.0])
    values = np.array([3.0, 4.0, 3.5, 5.0])

    spline = MonotoneSpline1D(arc_lengths, values)

    # Test first derivative exists and is reasonable
    deriv_at_start = spline.evaluate(0.0, derivative=1)
    deriv_at_mid = spline.evaluate(15.0, derivative=1)

    # Derivatives should be finite
    assert np.isfinite(deriv_at_start), "First derivative at start should be finite"
    assert np.isfinite(deriv_at_mid), "First derivative at midpoint should be finite"

    # Test second derivative
    deriv2_at_mid = spline.evaluate(15.0, derivative=2)
    assert np.isfinite(deriv2_at_mid), "Second derivative should be finite"

    # PCHIP doesn't support derivatives higher than 2
    with pytest.raises(ValueError):
        spline.evaluate(15.0, derivative=3)


def test_monotone_spline_polynomial_coefficients():
    """Test extraction of polynomial coefficients."""
    arc_lengths = np.array([0.0, 10.0, 20.0])
    values = np.array([3.0, 4.0, 3.5])

    spline = MonotoneSpline1D(arc_lengths, values)

    # Should have 2 segments
    assert spline.num_segments == 2

    # Get coefficients for first segment
    a, b, c, d = spline.get_polynomial_coefficients(0)

    # Coefficients should be finite
    assert all(
        np.isfinite(coef) for coef in [a, b, c, d]
    ), "Coefficients should be finite"

    # Polynomial should match spline value at segment start
    s0 = arc_lengths[0]
    poly_value = a  # At s0, (s-s0) = 0, so poly = a
    spline_value = spline.evaluate(s0)
    assert (
        abs(poly_value - spline_value) < 1e-6
    ), "Polynomial should match spline at segment start"


def test_monotone_vs_cubic_comparison():
    """Compare monotone and cubic splines on overshoot-prone data."""
    # Data that causes cubic spline to overshoot
    arc_lengths = np.array([0.0, 1.0, 2.0, 3.0])
    values = np.array([2.0, 3.0, 1.5, 2.5])  # Has a local minimum

    monotone_spline = MonotoneSpline1D(arc_lengths, values)
    cubic_spline = CubicSpline1D(arc_lengths, values)

    # Sample densely
    test_points = np.linspace(0.0, 3.0, 100)

    monotone_values = [monotone_spline.evaluate(s) for s in test_points]
    cubic_values = [cubic_spline.evaluate(s) for s in test_points]

    # Monotone spline should stay within range
    min_val = np.min(values)
    max_val = np.max(values)

    monotone_within_range = all(
        min_val - 1e-6 <= v <= max_val + 1e-6 for v in monotone_values
    )
    assert monotone_within_range, "Monotone spline should stay within input range"

    # Cubic spline might overshoot (this demonstrates the problem)
    cubic_min = np.min(cubic_values)
    cubic_max = np.max(cubic_values)

    # Document that cubic can overshoot
    # (This is informational, not a failure condition)
    if cubic_min < min_val - 1e-6 or cubic_max > max_val + 1e-6:
        print(
            f"Cubic spline overshot: input range [{min_val:.3f}, {max_val:.3f}], "
            f"cubic range [{cubic_min:.3f}, {cubic_max:.3f}]"
        )


def test_monotone_spline_get_segments():
    """Test get_segments() method."""
    arc_lengths = np.array([0.0, 10.0, 20.0])
    values = np.array([3.0, 4.0, 3.5])

    spline = MonotoneSpline1D(arc_lengths, values)

    segments = spline.get_segments()

    # Should have 2 segments
    assert len(segments) == 2

    # Each segment should have (s_offset, a, b, c, d)
    for seg in segments:
        assert len(seg) == 5
        s_offset, a, b, c, d = seg
        assert all(np.isfinite(v) for v in [s_offset, a, b, c, d])


def test_monotone_spline_validation():
    """Test input validation."""
    # Mismatched lengths
    with pytest.raises(ValueError, match="must have the same length"):
        MonotoneSpline1D(np.array([0.0, 1.0]), np.array([1.0, 2.0, 3.0]))

    # Too few points
    with pytest.raises(ValueError, match="At least 2 points"):
        MonotoneSpline1D(np.array([0.0]), np.array([1.0]))

    # Non-monotonic arc lengths
    with pytest.raises(ValueError, match="monotonically increasing"):
        MonotoneSpline1D(np.array([0.0, 2.0, 1.0]), np.array([1.0, 2.0, 3.0]))


def test_monotone_spline_positive_preservation():
    """Test that positive input values remain positive in output."""
    arc_lengths = np.array([0.0, 10.0, 20.0, 30.0])
    values = np.array([1.0, 0.5, 0.8, 0.3])  # All positive

    spline = MonotoneSpline1D(arc_lengths, values)

    # Sample very densely to check for any negative values
    test_points = np.linspace(0.0, 30.0, 500)
    interpolated_values = [spline.evaluate(s) for s in test_points]

    # All values should remain positive
    assert all(
        v > -1e-10 for v in interpolated_values
    ), "All values should remain positive"

    # Minimum should be close to input minimum (shape preservation)
    min_interpolated = np.min(interpolated_values)
    min_input = np.min(values)
    assert (
        abs(min_interpolated - min_input) < 0.1
    ), "Minimum interpolated value should be close to input minimum"
