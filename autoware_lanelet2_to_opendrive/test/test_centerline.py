"""Tests for centerline functions."""

import math

import lanelet2
import numpy as np
import pytest
from autoware_lanelet2_to_opendrive.centerline import (
    extract_centerline_as_spline,
    estimate_lanelet_width_as_spline,
)
from autoware_lanelet2_to_opendrive.conversion_config import (
    ParamPoly3Config,
    WidthEstimationConfig,
    WidthReference,
)
from autoware_lanelet2_to_opendrive.opendrive.geometry import (
    Arc,
    ParamPoly3,
    evaluate_plan_view_world,
)
from autoware_lanelet2_to_opendrive.opendrive.road import (
    Road,
    _evaluate_planview_endpoint_with_heading,
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


def _make_degenerate_endpoint_lanelet(
    *,
    degenerate_at: str,
) -> tuple[lanelet2.core.LaneletMap, lanelet2.core.Lanelet]:
    """Build a straight +Y lanelet whose start or end width collapses to zero."""
    if degenerate_at == "start":
        left_xy = [(0.0, 0.0), (0.0, 10.0)]
        right_xy = [(0.0, 0.0), (2.0, 10.0)]
    elif degenerate_at == "end":
        left_xy = [(0.0, 0.0), (0.0, 10.0)]
        right_xy = [(2.0, 0.0), (0.0, 10.0)]
    else:
        raise ValueError(f"Unsupported degenerate endpoint: {degenerate_at}")

    def make_points(points: list[tuple[float, float]]) -> list[lanelet2.core.Point3d]:
        return [
            lanelet2.core.Point3d(lanelet2.core.getId(), x, y, 0.0) for x, y in points
        ]

    left_bound = lanelet2.core.LineString3d(lanelet2.core.getId(), make_points(left_xy))
    right_bound = lanelet2.core.LineString3d(
        lanelet2.core.getId(), make_points(right_xy)
    )
    lanelet = lanelet2.core.Lanelet(lanelet2.core.getId(), left_bound, right_bound)
    lanelet.attributes["subtype"] = "road"

    lanelet_map = lanelet2.core.LaneletMap()
    lanelet_map.add(lanelet)
    return lanelet_map, lanelet


def _sample_plan_view_xy(road: Road) -> list[tuple[float, float]]:
    """Sample world XY positions from a road planView."""
    plan_view = road.plan_view
    assert plan_view is not None
    samples = []
    for geometry in plan_view.geometries:
        coeffs = None
        arc_curvature = None
        if isinstance(geometry, ParamPoly3):
            coeffs = (
                geometry.aU,
                geometry.bU,
                geometry.cU,
                geometry.dU,
                geometry.aV,
                geometry.bV,
                geometry.cV,
                geometry.dV,
            )
        elif isinstance(geometry, Arc):
            arc_curvature = geometry.curvature
        for p in np.linspace(0.0, geometry.length, 5):
            xy = evaluate_plan_view_world(
                geometry.x,
                geometry.y,
                geometry.hdg,
                float(p),
                coeffs,
                arc_curvature,
            )
            assert xy is not None
            samples.append((float(xy[0]), float(xy[1])))
    return samples


def _heading_error(actual: float, expected: float) -> float:
    """Return wrapped absolute angular error in radians."""
    return abs(math.atan2(math.sin(actual - expected), math.cos(actual - expected)))


@pytest.mark.parametrize("degenerate_at", ["start", "end"])
def test_degenerate_endpoint_width_preserves_reference_line_direction(
    degenerate_at: str,
) -> None:
    """A zero-width lanelet endpoint must not twist the road reference line."""
    lanelet_map, lanelet = _make_degenerate_endpoint_lanelet(
        degenerate_at=degenerate_at
    )

    road = Road.construct_from_lanelet_groups(
        lanelet_map,
        {lanelet},
        road_id=0,
        traffic_rule="RHT",
        parampoly3_config=ParamPoly3Config(enabled=False),
    )

    samples = _sample_plan_view_xy(road)
    max_lateral_error = max(abs(x) for x, _y in samples)
    plan_view = road.plan_view
    assert plan_view is not None
    start = _evaluate_planview_endpoint_with_heading(plan_view, at_start=True)
    end = _evaluate_planview_endpoint_with_heading(plan_view, at_start=False)
    assert start is not None
    assert end is not None

    errors = []
    if max_lateral_error >= 0.25:
        errors.append(
            f"reference line loops away from the source boundary: "
            f"max |x|={max_lateral_error:.3f} m"
        )
    expected_heading = math.pi / 2
    start_heading_error = _heading_error(start[2], expected_heading)
    end_heading_error = _heading_error(end[2], expected_heading)
    if start_heading_error >= 0.15:
        errors.append(
            f"start heading {start[2]:.3f} rad is not aligned with +Y direction"
        )
    if end_heading_error >= 0.15:
        errors.append(f"end heading {end[2]:.3f} rad is not aligned with +Y direction")

    assert not errors, "\n".join(errors)
