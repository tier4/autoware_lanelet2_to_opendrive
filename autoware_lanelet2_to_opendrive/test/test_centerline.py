"""Tests for centerline functions."""

import numpy as np
import pytest
from autoware_lanelet2_to_opendrive.centerline import (
    _calculate_optimal_num_samples,
    extract_centerline_as_spline,
    estimate_lanelet_width_as_spline,
)
from autoware_lanelet2_to_opendrive.conversion_config import (
    WidthEstimationConfig,
    WidthReference,
)


def test_estimate_lanelet_width_as_spline_constant_width(lanelet_map):
    """Test that lanelet ID=555 width spline interpolation produces values in expected range (3.64-3.66m)."""

    # Get lanelet with ID=555
    lanelet_555 = lanelet_map.laneletLayer.get(555)
    assert lanelet_555 is not None, "Lanelet with ID=555 not found in test map"

    # Estimate width as spline using left_bound reference to avoid asymmetry check
    config = WidthEstimationConfig(num_samples=10, reference=WidthReference.LEFT_BOUND)
    width_spline = estimate_lanelet_width_as_spline(lanelet_555, config)

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


@pytest.mark.xfail(
    strict=True,
    reason="_calculate_optimal_num_samples returns ceil(length / interval), "
    "one sample short of the inclusive endpoint",
)
@pytest.mark.parametrize("total_length", [20.0, 40.0, 61.5, 98.0])
def test_adaptive_sampling_does_not_exceed_configured_interval(total_length):
    """Adaptive sampling must not space width samples further apart than configured.

    ``np.linspace(0.0, 1.0, num_samples)`` is inclusive at both ends, so N
    samples span N-1 intervals. Returning ``ceil(total_length / interval)``
    samples therefore realises a spacing of
    ``total_length / (ceil(total_length / interval) - 1)``, which is always
    coarser than the configured interval.

    The lengths are chosen to stay inside the unclamped band for this config:
    at a 2 m interval, ``min_samples`` governs below 8 m and ``max_samples``
    above 98 m.
    """
    config = WidthEstimationConfig(
        adaptive_sampling=True,
        min_samples=5,
        max_samples=50,
        default_sample_interval=2.0,
    )

    num_samples = _calculate_optimal_num_samples(total_length, config)

    realised_interval = total_length / (num_samples - 1)
    assert realised_interval <= config.default_sample_interval + 1e-9, (
        f"{num_samples} samples over {total_length} m realise a spacing of "
        f"{realised_interval:.3f} m, coarser than the configured "
        f"{config.default_sample_interval} m"
    )


def test_adaptive_sampling_respects_the_sample_clamps():
    """The +1 for the inclusive endpoint must not push the count past max_samples."""
    config = WidthEstimationConfig(
        adaptive_sampling=True,
        min_samples=5,
        max_samples=50,
        default_sample_interval=2.0,
    )

    # Far below the interval: min_samples governs.
    assert _calculate_optimal_num_samples(1.0, config) == config.min_samples
    # Far above: max_samples governs, +1 included.
    assert _calculate_optimal_num_samples(1000.0, config) == config.max_samples
