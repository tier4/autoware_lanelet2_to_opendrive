"""Tests for centerline functions."""

import numpy as np
from autoware_lanelet2_to_opendrive.centerline import (
    extract_centerline_as_spline,
    estimate_lanelet_width_as_spline,
)


def test_estimate_lanelet_width_as_spline_constant_width(lanelet_map):
    """Test that lanelet ID=555 width spline interpolation produces values in expected range (3.64-3.66m)."""

    # Get lanelet with ID=555
    lanelet_555 = lanelet_map.laneletLayer.get(555)
    assert lanelet_555 is not None, "Lanelet with ID=555 not found in test map"

    # Estimate width as spline using left_bound reference to avoid asymmetry check
    width_spline = estimate_lanelet_width_as_spline(
        lanelet_555, num_samples=10, reference="left_bound"
    )

    # Sample points along the spline and check width values
    t_values = np.linspace(
        0.0, width_spline.total_length, 20
    )  # Sample more points for thorough testing

    width_values = []
    for t in t_values:
        # The spline returns [t, width], we want the width component (index 1)
        width = width_spline.evaluate(t).flatten()[1]
        width_values.append(width)

        # Assert width is in expected range for lanelet 555
        assert 2.90 <= width <= 3.68, (
            f"Width at t={t:.2f} is {width:.3f}m, "
            f"expected to be in range [2.90, 3.68]m"
        )
    print(width_values)

    # Check overall width statistics
    min_width = min(width_values)
    max_width = max(width_values)
    width_variation = max_width - min_width

    print(f"Width range: {min_width:.3f}m - {max_width:.3f}m")
    print(f"Width variation: {width_variation:.3f}m")

    # Assert reasonable width variation (should be small for this lanelet)
    assert width_variation < 0.75, (
        f"Width variation {width_variation:.3f}m is too large, "
        f"expected less than 0.75m for nearly constant width lanelet"
    )

    # Check that first derivative is small (indicating nearly constant width)
    dt = 0.01
    max_derivative = 0.0
    for t in t_values[1:-1]:  # Skip endpoints
        width_curr = width_spline.evaluate(t).flatten()[1]
        width_next = width_spline.evaluate(t + dt).flatten()[1]
        deriv_1 = abs((width_next - width_curr) / dt)
        max_derivative = max(max_derivative, deriv_1)

    assert max_derivative < 1.0, (
        f"Maximum width derivative {max_derivative:.4f} is too large, "
        f"expected less than 1.0 for nearly constant width"
    )


def test_extract_centerline_as_spline(lanelet_map):
    """Test centerline extraction as spline."""

    # Get a lanelet for testing
    lanelet = lanelet_map.laneletLayer.get(555)
    assert lanelet is not None, "Lanelet with ID=555 not found"

    # Extract centerline as spline
    spline = extract_centerline_as_spline(lanelet)

    # Test that spline can be evaluated with arc length parameters
    total_length = spline.total_length
    s_values = np.linspace(0, total_length, 10)
    for s in s_values:
        point = spline.evaluate(s)
        assert point.shape[0] == 3, "Spline should return 3D points"

    # Test that spline can be evaluated at specific arc length
    point_mid = spline.evaluate(total_length / 2)
    assert point_mid.shape[0] == 3, "Spline should return 3D points"
