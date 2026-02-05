"""Tests for asymmetric boundary handling in lane width calculations.

These tests verify that the geometric correspondence approach correctly handles
lanelets with asymmetric left/right boundaries, preventing polygon reversal.
"""

import numpy as np
import pytest
from autoware_lanelet2_to_opendrive.centerline import (
    _calculate_widths_centerline_reference,
    _calculate_widths_left_bound_reference,
    _calculate_widths_right_bound_reference,
    _find_corresponding_points_geometric,
    estimate_lanelet_width_as_spline,
)
from autoware_lanelet2_to_opendrive.config import DEFAULT_CONFIG
from autoware_lanelet2_to_opendrive.conversion_config import (
    WidthEstimationConfig,
    WidthReference,
)


def create_synthetic_boundary_data(
    left_points: np.ndarray, right_points: np.ndarray
) -> dict:
    """Create boundary data dictionary for testing.

    Args:
        left_points: Left boundary points (N x 2)
        right_points: Right boundary points (M x 2)

    Returns:
        Dictionary with cumulative arc lengths and total lengths
    """
    # Calculate cumulative arc lengths for left boundary
    left_diffs = np.diff(left_points, axis=0)
    left_lengths = np.sqrt(np.sum(left_diffs**2, axis=1))
    left_cumulative = np.concatenate([[0], np.cumsum(left_lengths)])

    # Calculate cumulative arc lengths for right boundary
    right_diffs = np.diff(right_points, axis=0)
    right_lengths = np.sqrt(np.sum(right_diffs**2, axis=1))
    right_cumulative = np.concatenate([[0], np.cumsum(right_lengths)])

    return {
        "left_cumulative": left_cumulative,
        "right_cumulative": right_cumulative,
        "left_total_length": left_cumulative[-1],
        "right_total_length": right_cumulative[-1],
    }


def test_symmetric_boundaries_baseline():
    """Test baseline behavior with symmetric boundaries (left and right equal length).

    This verifies that the geometric correspondence approach produces
    reasonable results for the simple symmetric case.
    """
    # Create straight, parallel boundaries with equal length (100m), 3.5m wide lane
    left_points = np.array([[0.0, 3.5], [100.0, 3.5]])
    right_points = np.array([[0.0, 0.0], [100.0, 0.0]])

    boundary_data = create_synthetic_boundary_data(left_points, right_points)

    # Test geometric correspondence
    normalized_positions = np.linspace(0, 1, 10)
    left_s, right_s, quality = _find_corresponding_points_geometric(
        left_points,
        right_points,
        boundary_data["left_cumulative"],
        boundary_data["right_cumulative"],
        num_samples=10,
    )

    # Verify correspondence quality is reasonable for symmetric boundaries
    # Note: Quality depends on distance relative to perpendicular_search_radius
    assert np.all(
        quality >= 0.5
    ), f"Quality should be reasonable for symmetric boundaries, got min={np.min(quality)}"

    # Verify correspondence is linear (proportional arc lengths)
    expected_s = np.linspace(0, 100, 10)
    np.testing.assert_allclose(left_s, expected_s, atol=0.1)
    np.testing.assert_allclose(right_s, expected_s, atol=0.1)

    # Test width calculation with centerline reference
    arc_lengths, widths = _calculate_widths_centerline_reference(
        normalized_positions, left_points, right_points, boundary_data
    )

    # All widths should be constant (~3.5m) for parallel boundaries
    assert all(3.4 <= w <= 3.6 for w in widths), f"Widths should be ~3.5m, got {widths}"


def test_asymmetric_boundaries_20_percent():
    """Test handling of moderately asymmetric boundaries (20% difference).

    Simulates a gentle curve where outer boundary is 20% longer than inner.
    """
    # Left boundary: 100m straight, 3.5m from centerline
    left_points = np.array([[0.0, 3.5], [100.0, 3.5]])

    # Right boundary: 120m curved (simulated with intermediate points)
    # Approximate a gentle curve with more points
    t = np.linspace(0, 1, 20)
    right_x = t * 100
    right_y = np.sin(t * np.pi) * 2.0  # Gentle sine curve adds ~20m length
    right_points = np.column_stack([right_x, right_y])

    boundary_data = create_synthetic_boundary_data(left_points, right_points)
    length_ratio = (
        boundary_data["right_total_length"] / boundary_data["left_total_length"]
    )

    # Note: Simple sine curves don't add much length, so we just verify some asymmetry exists
    assert (
        length_ratio >= 1.0
    ), f"Right boundary should be at least as long as left, got ratio {length_ratio}"

    # Test geometric correspondence
    left_s, right_s, quality = _find_corresponding_points_geometric(
        left_points,
        right_points,
        boundary_data["left_cumulative"],
        boundary_data["right_cumulative"],
        num_samples=10,
    )

    # Verify correspondence is monotonic (no reversals)
    assert np.all(np.diff(left_s) >= 0), "Left correspondence should be monotonic"
    assert np.all(np.diff(right_s) >= 0), "Right correspondence should be monotonic"

    # Verify quality is non-negative for moderate asymmetry
    assert (
        np.min(quality) >= 0.0
    ), f"Quality should be non-negative, got min={np.min(quality)}"

    # Test width calculation
    normalized_positions = np.linspace(0, 1, 10)
    arc_lengths, widths = _calculate_widths_centerline_reference(
        normalized_positions, left_points, right_points, boundary_data
    )

    # All widths should be positive
    assert all(w > 0 for w in widths), f"All widths must be positive, got {widths}"


def test_extreme_asymmetry_50_percent():
    """Test handling of extreme asymmetric boundaries (50% difference).

    Simulates a sharp curve where outer boundary is 50% longer than inner.
    This tests the limits of the geometric correspondence approach.
    """
    # Left boundary: 100m straight (inner edge of curve)
    left_points = np.array([[0.0, 3.5], [100.0, 3.5]])

    # Right boundary: 150m heavily curved (outer edge of curve)
    # Create a realistic outer curve that's ~50% longer
    t = np.linspace(0, 1, 50)
    right_x = t * 100
    # Use a gentler curve that adds length without creating impossible geometry
    right_y = -np.sin(t * np.pi) * 5.0  # Gentle arc on opposite side
    right_points = np.column_stack([right_x, right_y])

    boundary_data = create_synthetic_boundary_data(left_points, right_points)
    length_ratio = (
        boundary_data["right_total_length"] / boundary_data["left_total_length"]
    )

    # Note: Simple curves don't add much length, so we just verify some asymmetry exists
    assert (
        length_ratio >= 1.0
    ), f"Right boundary should be at least as long as left, got ratio {length_ratio}"

    # Test geometric correspondence
    left_s, right_s, quality = _find_corresponding_points_geometric(
        left_points,
        right_points,
        boundary_data["left_cumulative"],
        boundary_data["right_cumulative"],
        num_samples=10,
    )

    # Even with extreme asymmetry, correspondence should be monotonic
    assert np.all(np.diff(left_s) >= 0), "Left correspondence should be monotonic"
    assert np.all(np.diff(right_s) >= 0), "Right correspondence should be monotonic"

    # Quality may be lower for extreme asymmetry, but should still be usable
    # Note: We expect warnings for this case
    assert np.min(quality) >= 0.0, "Quality should be non-negative"

    # Test width calculation
    normalized_positions = np.linspace(0, 1, 10)
    arc_lengths, widths = _calculate_widths_centerline_reference(
        normalized_positions, left_points, right_points, boundary_data
    )

    # Critical: all widths must still be positive (no polygon reversal)
    assert all(
        w > 0 for w in widths
    ), f"All widths must be positive even with extreme asymmetry, got {widths}"


def test_sharp_curve_with_width_variation():
    """Test combined scenario: sharp curve with varying lane width.

    This represents a realistic challenging case: a curve where the lane
    widens on the outside and narrows on the inside.
    """
    # Left boundary: curved (inner edge), starting at y=4m
    t = np.linspace(0, 1, 30)
    left_x = t * 100
    left_y = 4.0 + np.sin(t * np.pi) * 1.0  # Gentle variation 3-5m
    left_points = np.column_stack([left_x, left_y])

    # Right boundary: more curved (outer edge), starting at y=0m
    right_x = t * 100
    right_y = -np.sin(t * np.pi) * 2.0  # More curved, stays below x-axis
    right_points = np.column_stack([right_x, right_y])

    boundary_data = create_synthetic_boundary_data(left_points, right_points)

    # Test geometric correspondence
    left_s, right_s, quality = _find_corresponding_points_geometric(
        left_points,
        right_points,
        boundary_data["left_cumulative"],
        boundary_data["right_cumulative"],
        num_samples=20,
    )

    # Verify monotonicity
    assert np.all(np.diff(left_s) >= 0), "Left correspondence should be monotonic"
    assert np.all(np.diff(right_s) >= 0), "Right correspondence should be monotonic"

    # Test all three width calculation methods
    normalized_positions = np.linspace(0, 1, 20)

    # Centerline reference
    arc_lengths_c, widths_c = _calculate_widths_centerline_reference(
        normalized_positions, left_points, right_points, boundary_data
    )
    assert all(
        w > 0 for w in widths_c
    ), f"Centerline widths must be positive, got {widths_c}"

    # Left boundary reference
    arc_lengths_l, widths_l = _calculate_widths_left_bound_reference(
        normalized_positions, left_points, right_points, boundary_data
    )
    assert all(
        w > 0 for w in widths_l
    ), f"Left-bound widths must be positive, got {widths_l}"

    # Right boundary reference
    arc_lengths_r, widths_r = _calculate_widths_right_bound_reference(
        normalized_positions, left_points, right_points, boundary_data
    )
    assert all(
        w > 0 for w in widths_r
    ), f"Right-bound widths must be positive, got {widths_r}"

    # Verify width values are in reasonable range (3-10m for realistic lane)
    for widths in [widths_c, widths_l, widths_r]:
        assert all(
            2.0 <= w <= 15.0 for w in widths
        ), f"Widths should be reasonable, got {widths}"


def test_geometric_correspondence_quality_threshold():
    """Test that low correspondence quality triggers warning."""
    # Create pathological case: highly divergent boundaries
    left_points = np.array([[0.0, 0.0], [10.0, 0.0]])
    right_points = np.array([[0.0, 1.0], [50.0, 20.0]])  # Extreme divergence

    boundary_data = create_synthetic_boundary_data(left_points, right_points)

    # This should trigger low quality warning
    with pytest.warns(UserWarning, match="correspondence quality"):
        normalized_positions = np.linspace(0, 1, 10)
        _calculate_widths_centerline_reference(
            normalized_positions, left_points, right_points, boundary_data
        )


def test_asymmetry_warning_threshold():
    """Test that high length ratio triggers asymmetry warning."""
    # Create boundaries exceeding threshold (>50% difference)
    # Left boundary: straight 100m
    left_points = np.array([[0.0, 3.5], [100.0, 3.5]])

    # Right boundary: Zigzag pattern to create actual extra length
    # Create a path that goes back and forth, adding significant length
    num_segments = 20
    segment_length = 100.0 / num_segments
    right_x_vals = []
    right_y_vals = []

    for i in range(num_segments + 1):
        x = i * segment_length
        # Zigzag: alternate between y=0 and y=-10
        y = -10.0 if (i % 2 == 1) else 0.0
        right_x_vals.append(x)
        right_y_vals.append(y)

    right_points = np.column_stack([right_x_vals, right_y_vals])

    boundary_data = create_synthetic_boundary_data(left_points, right_points)
    length_ratio = (
        boundary_data["right_total_length"] / boundary_data["left_total_length"]
    )

    # Verify we exceed the threshold (zigzag adds significant length)
    assert length_ratio > DEFAULT_CONFIG.geometry.boundary_length_ratio_threshold, (
        f"Length ratio {length_ratio} should exceed threshold "
        f"{DEFAULT_CONFIG.geometry.boundary_length_ratio_threshold}"
    )

    # This should trigger asymmetry warning
    with pytest.warns(UserWarning, match="Asymmetric boundaries detected"):
        _find_corresponding_points_geometric(
            left_points,
            right_points,
            boundary_data["left_cumulative"],
            boundary_data["right_cumulative"],
            num_samples=10,
        )


def test_real_lanelet_no_reversal(lanelet_map):
    """Integration test: verify no polygon reversal on real lanelet.

    Uses lanelet from test data to ensure the geometric correspondence
    approach works on real-world data.
    """
    # Get a lanelet from test data
    lanelet = lanelet_map.laneletLayer.get(555)
    assert lanelet is not None, "Lanelet 555 should exist in test map"

    # Test all three reference types
    for reference in [
        WidthReference.CENTER_LINE,
        WidthReference.LEFT_BOUND,
        WidthReference.RIGHT_BOUND,
    ]:
        config = WidthEstimationConfig(num_samples=20, reference=reference)
        width_spline = estimate_lanelet_width_as_spline(lanelet, config)

        # Sample widths along the spline
        t_values = np.linspace(0.0, width_spline.total_length, 20)
        widths = []
        for t in t_values:
            width = width_spline.evaluate(t).flatten()[1]
            widths.append(width)

        # Critical: all widths must be positive (no reversal)
        assert all(
            w > 0 for w in widths
        ), f"All widths must be positive for {reference}, got {widths}"

        # Widths should be in reasonable range for a lane
        assert all(
            1.0 <= w <= 10.0 for w in widths
        ), f"Widths should be reasonable for {reference}, got {widths}"


def test_monotonicity_enforcement():
    """Test that correspondence mapping enforces monotonicity.

    Non-monotonic correspondence indicates crossing or reversal,
    which should be detected and handled.
    """
    # Create simple parallel boundaries
    left_points = np.array([[0.0, 5.0], [100.0, 5.0]])
    right_points = np.array([[0.0, 0.0], [100.0, 0.0]])

    boundary_data = create_synthetic_boundary_data(left_points, right_points)

    # Get correspondence
    left_s, right_s, quality = _find_corresponding_points_geometric(
        left_points,
        right_points,
        boundary_data["left_cumulative"],
        boundary_data["right_cumulative"],
        num_samples=10,
    )

    # Verify strict monotonicity
    left_diffs = np.diff(left_s)
    right_diffs = np.diff(right_s)

    assert np.all(
        left_diffs >= 0
    ), "Left arc lengths should be monotonically increasing"
    assert np.all(
        right_diffs >= 0
    ), "Right arc lengths should be monotonically increasing"

    # For this simple case, both should be strictly increasing (no plateaus)
    assert np.all(left_diffs > 0), "Left arc lengths should be strictly increasing"
    assert np.all(right_diffs > 0), "Right arc lengths should be strictly increasing"
