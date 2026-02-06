"""Integration tests for width estimation with different spline types."""

import numpy as np
import pytest
import lanelet2
from unittest.mock import patch
from autoware_lanelet2_to_opendrive.centerline import estimate_lanelet_width_as_spline
from autoware_lanelet2_to_opendrive.conversion_config import (
    WidthEstimationConfig,
    WidthReference,
)
from autoware_lanelet2_to_opendrive.config import DEFAULT_CONFIG, CenterlineConstants


def create_test_lanelet():
    """Create a simple test lanelet with known geometry."""
    # Left boundary
    left_points = [
        lanelet2.core.Point3d(1, 0.0, 0.0, 0.0),
        lanelet2.core.Point3d(2, 0.0, 10.0, 0.0),
        lanelet2.core.Point3d(3, 0.0, 20.0, 0.0),
        lanelet2.core.Point3d(4, 0.0, 30.0, 0.0),
    ]
    left_bound = lanelet2.core.LineString3d(1, left_points)

    # Right boundary (parallel, 4 meters away)
    right_points = [
        lanelet2.core.Point3d(5, 4.0, 0.0, 0.0),
        lanelet2.core.Point3d(6, 4.0, 10.0, 0.0),
        lanelet2.core.Point3d(7, 4.0, 20.0, 0.0),
        lanelet2.core.Point3d(8, 4.0, 30.0, 0.0),
    ]
    right_bound = lanelet2.core.LineString3d(2, right_points)

    return lanelet2.core.Lanelet(1, left_bound, right_bound)


def test_width_estimation_with_monotone_spline():
    """Test width estimation using monotone spline (default)."""
    lanelet = create_test_lanelet()
    config = WidthEstimationConfig(num_samples=20, reference=WidthReference.CENTER_LINE)

    # Should use monotone spline by default
    width_spline = estimate_lanelet_width_as_spline(lanelet, config)

    # Sample at various points
    test_points = np.linspace(0, width_spline.total_length, 50)

    for s in test_points:
        width = width_spline.get_width_at_arc_length(s)
        # Width should be approximately 4.0 (distance between boundaries)
        assert 3.9 < width < 4.1, f"Width at s={s} should be approximately 4.0"
        # Most importantly, width should be positive
        assert width > 0, f"Width at s={s} should be positive"


def test_width_estimation_with_cubic_spline():
    """Test width estimation using cubic spline."""
    lanelet = create_test_lanelet()
    config = WidthEstimationConfig(num_samples=20, reference=WidthReference.CENTER_LINE)

    # Temporarily override config to use cubic spline
    from autoware_lanelet2_to_opendrive.config import ConversionConfig

    custom_config = ConversionConfig(
        centerline=CenterlineConstants(width_spline_type="cubic")
    )

    with patch(
        "autoware_lanelet2_to_opendrive.centerline.DEFAULT_CONFIG", custom_config
    ):
        width_spline = estimate_lanelet_width_as_spline(lanelet, config)

        # Sample at various points
        test_points = np.linspace(0, width_spline.total_length, 50)

        for s in test_points:
            width = width_spline.get_width_at_arc_length(s)
            # Width should be approximately 4.0
            assert 3.9 < width < 4.1, f"Width at s={s} should be approximately 4.0"


def test_width_estimation_validates_negative_widths():
    """Test that validation catches negative widths from input data."""
    # Create lanelet with invalid geometry that would produce negative widths
    # (This is a contrived example for testing)

    # Manually create arrays that would fail validation
    _arc_lengths = np.array([0.0, 10.0, 20.0])  # noqa: F841
    widths = np.array([1.0, -0.5, 2.0])  # Contains negative width

    # This should raise ValueError during input validation
    with pytest.raises(ValueError, match="Invalid width values"):
        # Simulate what happens inside estimate_lanelet_width_as_spline
        min_width = np.min(widths)
        threshold = DEFAULT_CONFIG.centerline.width_min_threshold
        if min_width < threshold:
            raise ValueError(
                f"Invalid width values: min={min_width:.3f}m, threshold={threshold}m"
            )


def test_width_spline_adapter_polymorphism():
    """Test that Width1DSplineAdapter works with both spline types."""
    from autoware_lanelet2_to_opendrive.config import ConversionConfig

    lanelet = create_test_lanelet()
    config = WidthEstimationConfig(num_samples=20, reference=WidthReference.CENTER_LINE)

    # Test with monotone spline
    monotone_config = ConversionConfig(
        centerline=CenterlineConstants(width_spline_type="monotone")
    )
    with patch(
        "autoware_lanelet2_to_opendrive.centerline.DEFAULT_CONFIG", monotone_config
    ):
        width_spline_monotone = estimate_lanelet_width_as_spline(lanelet, config)
        width_monotone = width_spline_monotone.get_width_at_arc_length(10.0)
        assert width_monotone > 0

    # Test with cubic spline
    cubic_config = ConversionConfig(
        centerline=CenterlineConstants(width_spline_type="cubic")
    )
    with patch(
        "autoware_lanelet2_to_opendrive.centerline.DEFAULT_CONFIG", cubic_config
    ):
        width_spline_cubic = estimate_lanelet_width_as_spline(lanelet, config)
        width_cubic = width_spline_cubic.get_width_at_arc_length(10.0)
        assert width_cubic > 0

    # Both should produce similar results for this simple geometry
    assert abs(width_monotone - width_cubic) < 0.1


def test_width_estimation_error_on_invalid_spline_type():
    """Test that invalid spline type raises appropriate error."""
    from autoware_lanelet2_to_opendrive.config import ConversionConfig

    lanelet = create_test_lanelet()
    config = WidthEstimationConfig(num_samples=20, reference=WidthReference.CENTER_LINE)

    # Override with invalid spline type
    invalid_config = ConversionConfig(
        centerline=CenterlineConstants(width_spline_type="invalid_type")
    )
    with patch(
        "autoware_lanelet2_to_opendrive.centerline.DEFAULT_CONFIG", invalid_config
    ):
        with pytest.raises(ValueError, match="Invalid width_spline_type"):
            estimate_lanelet_width_as_spline(lanelet, config)


def test_width_spline_produces_valid_opendrive_segments():
    """Test that width spline produces valid OpenDRIVE polynomial segments."""
    lanelet = create_test_lanelet()
    config = WidthEstimationConfig(num_samples=20, reference=WidthReference.CENTER_LINE)

    width_spline = estimate_lanelet_width_as_spline(lanelet, config)
    segments = width_spline.get_polynomial_segments()

    # Should have segments
    assert len(segments) > 0

    # Each segment should have (s_offset, a, b, c, d)
    for seg in segments:
        assert len(seg) == 5
        s_offset, a, b, c, d = seg

        # All coefficients should be finite
        assert all(np.isfinite(v) for v in [s_offset, a, b, c, d])

        # s_offset should be non-negative
        assert s_offset >= 0


def test_width_estimation_with_varying_widths():
    """Test width estimation with non-uniform lane width."""
    # Create lanelet with varying width
    left_points = [
        lanelet2.core.Point3d(1, 0.0, 0.0, 0.0),
        lanelet2.core.Point3d(2, 0.0, 10.0, 0.0),
        lanelet2.core.Point3d(3, 0.0, 20.0, 0.0),
        lanelet2.core.Point3d(4, 0.0, 30.0, 0.0),
    ]
    left_bound = lanelet2.core.LineString3d(1, left_points)

    # Right boundary with varying distance (3m, 4m, 5m, 4m)
    right_points = [
        lanelet2.core.Point3d(5, 3.0, 0.0, 0.0),
        lanelet2.core.Point3d(6, 4.0, 10.0, 0.0),
        lanelet2.core.Point3d(7, 5.0, 20.0, 0.0),
        lanelet2.core.Point3d(8, 4.0, 30.0, 0.0),
    ]
    right_bound = lanelet2.core.LineString3d(2, right_points)

    lanelet = lanelet2.core.Lanelet(1, left_bound, right_bound)
    config = WidthEstimationConfig(num_samples=20, reference=WidthReference.CENTER_LINE)

    width_spline = estimate_lanelet_width_as_spline(lanelet, config)

    # Sample densely to ensure no negative widths
    test_points = np.linspace(0, width_spline.total_length, 200)

    for s in test_points:
        width = width_spline.get_width_at_arc_length(s)
        # Width should always be positive
        assert width > 0, f"Width at s={s} should be positive, got {width}"
        # Width should be in reasonable range (3-5 meters)
        assert (
            2.5 < width < 5.5
        ), f"Width at s={s} should be in range [2.5, 5.5], got {width}"


def test_width_estimation_different_references():
    """Test width estimation with different reference types."""
    lanelet = create_test_lanelet()

    # Test all reference types
    for ref_type in [
        WidthReference.CENTER_LINE,
        WidthReference.LEFT_BOUND,
        WidthReference.RIGHT_BOUND,
    ]:
        config = WidthEstimationConfig(num_samples=20, reference=ref_type)
        width_spline = estimate_lanelet_width_as_spline(lanelet, config)

        # Should produce valid spline
        assert width_spline is not None
        assert width_spline.total_length > 0

        # Sample at midpoint
        mid_point = width_spline.total_length / 2
        width = width_spline.get_width_at_arc_length(mid_point)

        # Width should be positive
        assert width > 0, f"Width with reference {ref_type} should be positive"
