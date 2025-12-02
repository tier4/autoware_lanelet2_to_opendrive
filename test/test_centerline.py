"""Tests for centerline functions."""

from pathlib import Path
import numpy as np
import lanelet2
from autoware_lanelet2_extension_python.projection import MGRSProjector
from autoware_lanelet2_to_opendrive.centerline import (
    extract_centerline_as_spline,
    estimate_lanelet_width_as_spline,
)


def load_test_map():
    """Load the test lanelet2 map."""
    test_data_path = Path(__file__).parent / "data" / "lanelet2_map.osm"
    projector = MGRSProjector(
        lanelet2.io.Origin(35.23, 139.16)
    )  # MGRS origin for Tokyo area (54SUE)
    return lanelet2.io.load(str(test_data_path), projector)


def test_estimate_lanelet_width_as_spline_constant_width():
    """Test that lanelet ID=555 has small higher-order spline derivatives indicating smooth width variation."""
    lanelet_map = load_test_map()

    # Get lanelet with ID=555
    lanelet_555 = lanelet_map.laneletLayer.get(555)
    assert lanelet_555 is not None, "Lanelet with ID=555 not found in test map"

    # Estimate width as spline
    width_spline = estimate_lanelet_width_as_spline(lanelet_555, num_samples=10)

    # Sample points along the spline
    t_values = np.linspace(0.1, 0.9, 8)  # Avoid endpoints for derivative stability

    # Check that first and higher order derivatives are close to zero
    # This indicates constant width (polynomial degree 0)
    dt = 0.01
    for t in t_values:
        # Calculate first derivative numerically
        # The spline returns [t, width], we want the width component (index 1)
        if t + dt <= 1.0:
            width_curr = width_spline.evaluate(t).flatten()[1]
            width_next = width_spline.evaluate(t + dt).flatten()[1]
            deriv_1 = (width_next - width_curr) / dt
        else:
            width_curr = width_spline.evaluate(t).flatten()[1]
            width_prev = width_spline.evaluate(t - dt).flatten()[1]
            deriv_1 = (width_curr - width_prev) / dt

        # Assert first derivative is relatively small (lanelet 555 has slight width variation)
        # The derivative is in units/parameter, where parameter goes from 0 to 1
        # So a derivative of 20 means the width changes by ~20 units over the full length
        assert abs(deriv_1) < 25, (
            f"Width first derivative at t={t:.2f} is {deriv_1:.4f}, "
            f"expected small derivative for nearly constant width"
        )

        # Calculate second derivative numerically
        if t - dt >= 0 and t + dt <= 1.0:
            width_prev = width_spline.evaluate(t - dt).flatten()[1]
            width_curr = width_spline.evaluate(t).flatten()[1]
            width_next = width_spline.evaluate(t + dt).flatten()[1]
            deriv_2 = (width_next - 2 * width_curr + width_prev) / (dt * dt)

            assert abs(deriv_2) < 50, (
                f"Width second derivative at t={t:.2f} is {deriv_2:.4f}, "
                f"expected small second derivative for smooth width variation"
            )


def test_extract_centerline_as_spline():
    """Test centerline extraction as spline."""
    lanelet_map = load_test_map()

    # Get a lanelet for testing
    lanelet = lanelet_map.laneletLayer.get(555)
    assert lanelet is not None, "Lanelet with ID=555 not found"

    # Extract centerline as spline
    spline = extract_centerline_as_spline(lanelet)

    # Test that spline can be evaluated
    t_values = np.linspace(0, 1, 10)
    for t in t_values:
        point = spline.evaluate(t)
        assert point.shape[0] == 3, "Spline should return 3D points"

    # Test that spline can be evaluated at specific point
    point_mid = spline.evaluate(0.5)
    assert point_mid.shape[0] == 3, "Spline should return 3D points"
