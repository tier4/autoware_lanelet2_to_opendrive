"""Tests for fit_lane_border_polynomials."""

from __future__ import annotations

import numpy as np
import pytest

from autoware_lanelet2_to_opendrive.border_fit import fit_lane_border_polynomials
from autoware_lanelet2_to_opendrive.spline import Splines


def _straight_reference(length: float = 10.0) -> Splines:
    """Reference line along the x-axis from (0, 0) to (length, 0)."""
    points = np.array([[i * length / 10, 0.0] for i in range(11)])
    return Splines(points, num_control_points=11)


def test_constant_offset_unconstrained() -> None:
    """Boundary parallel to the reference at constant t produces near-constant border."""
    ref = _straight_reference(10.0)
    boundary_3d = np.array(
        [[i, -3.0, 0.0] for i in range(11)]
    )  # offset -3 on left-rotated normal
    # Reference normal is (-tangent.y, +tangent.x) = (0, 1). t = (boundary - ref) · normal = -3.
    borders = fit_lane_border_polynomials(boundary_3d, ref)

    assert len(borders) >= 1

    # Reconstruct t(s=0) and t(s=length) from the polynomial segments.
    first = borders[0]
    last = borders[-1]
    t_at_zero = first.a
    ds_last = ref.total_length - last.s_offset
    t_at_end = last.a + last.b * ds_last + last.c * ds_last**2 + last.d * ds_last**3

    assert t_at_zero == pytest.approx(-3.0, abs=1e-3)
    assert t_at_end == pytest.approx(-3.0, abs=1e-3)


def test_start_constraint_pins_s0() -> None:
    """t_start_constraint forces the first segment's a coefficient."""
    ref = _straight_reference(10.0)
    # Boundary tilted inward so that without a constraint t at s=0 would be ~ -2.5
    boundary_3d = np.array([[i, -2.5 + 0.05 * i, 0.0] for i in range(11)])
    borders = fit_lane_border_polynomials(boundary_3d, ref, t_start_constraint=-3.0)
    assert borders[0].a == pytest.approx(-3.0, abs=1e-9)


def test_end_constraint_pins_send() -> None:
    """t_end_constraint forces the polynomial value at s=total_length."""
    ref = _straight_reference(10.0)
    boundary_3d = np.array([[i, -3.0 + 0.05 * i, 0.0] for i in range(11)])
    borders = fit_lane_border_polynomials(boundary_3d, ref, t_end_constraint=-1.5)
    last = borders[-1]
    ds = ref.total_length - last.s_offset
    t_at_end = last.a + last.b * ds + last.c * ds**2 + last.d * ds**3
    assert t_at_end == pytest.approx(-1.5, abs=1e-9)


def test_both_constraints() -> None:
    """Both endpoint constraints are honoured."""
    ref = _straight_reference(10.0)
    boundary_3d = np.array([[i, -3.0, 0.0] for i in range(11)])
    borders = fit_lane_border_polynomials(
        boundary_3d, ref, t_start_constraint=-2.0, t_end_constraint=-2.5
    )
    assert borders[0].a == pytest.approx(-2.0, abs=1e-9)
    last = borders[-1]
    ds = ref.total_length - last.s_offset
    t_at_end = last.a + last.b * ds + last.c * ds**2 + last.d * ds**3
    assert t_at_end == pytest.approx(-2.5, abs=1e-9)


def test_minimum_two_points() -> None:
    """A boundary with only two points still produces at least one segment."""
    ref = _straight_reference(10.0)
    boundary_3d = np.array([[0.0, -3.0, 0.0], [10.0, -3.0, 0.0]])
    borders = fit_lane_border_polynomials(boundary_3d, ref)
    assert len(borders) >= 1


import lanelet2  # noqa: E402
from lanelet2.core import (  # noqa: E402
    Lanelet,
    LineString3d,
    Point3d,
    getId,
)

from autoware_lanelet2_to_opendrive.opendrive.enums import LaneMode  # noqa: E402
from autoware_lanelet2_to_opendrive.opendrive.lane import Lane  # noqa: E402


def _make_synthetic_lanelet(
    left_offset: float = 0.0,
    right_offset: float = -3.0,
    length: float = 10.0,
    n: int = 5,
) -> tuple[Lanelet, lanelet2.core.LaneletMap]:
    """Build a tiny lanelet straight along the +x direction with constant width."""
    lanelet_map = lanelet2.core.LaneletMap()
    left_pts = [
        Point3d(getId(), i * length / (n - 1), left_offset, 0.0) for i in range(n)
    ]
    right_pts = [
        Point3d(getId(), i * length / (n - 1), right_offset, 0.0) for i in range(n)
    ]
    left_ls = LineString3d(getId(), left_pts)
    right_ls = LineString3d(getId(), right_pts)
    lanelet = Lanelet(getId(), left_ls, right_ls)
    lanelet.attributes["subtype"] = "road"
    lanelet_map.add(lanelet)
    return lanelet, lanelet_map


def test_lane_construct_in_border_mode_populates_borders() -> None:
    """BORDER mode populates lane.borders (and leaves lane.widths empty)."""
    lanelet, lanelet_map = _make_synthetic_lanelet()
    ref = _straight_reference(10.0)

    lane = Lane.construct_from_lanelet(
        lanelet_map=lanelet_map,
        lanelet=lanelet,
        rule="RHT",
        reference_line_spline=ref,
        mode=LaneMode.BORDER,
    )

    assert len(lane.borders) >= 1
    assert lane.widths == []


def test_lane_construct_in_width_mode_keeps_widths() -> None:
    """WIDTH mode (default) preserves the existing width-based behavior."""
    lanelet, lanelet_map = _make_synthetic_lanelet()
    ref = _straight_reference(10.0)

    lane = Lane.construct_from_lanelet(
        lanelet_map=lanelet_map,
        lanelet=lanelet,
        rule="RHT",
        reference_line_spline=ref,
        mode=LaneMode.WIDTH,
    )

    assert len(lane.widths) >= 1
    assert lane.borders == []


from autoware_lanelet2_to_opendrive.opendrive.lane_section import LaneSection  # noqa: E402


def test_lane_section_passes_border_params_to_lane() -> None:
    """LaneSection in BORDER mode produces lanes whose borders are populated."""
    lanelet, lanelet_map = _make_synthetic_lanelet()
    section = LaneSection.construct_from_lanelet_groups(
        lanelet_map=lanelet_map,
        lanelet_group=[lanelet],
        s_offset=0.0,
        traffic_rule="RHT",
        mode=LaneMode.BORDER,
        t_start_per_lanelet={lanelet.id: -3.0},
        t_end_per_lanelet={lanelet.id: -3.0},
    )

    # Lane id -1 is the single lane (RHT convention).
    lane = section.right_lanes[-1]
    assert len(lane.borders) >= 1
    assert lane.widths == []
    # First segment a-coefficient should equal the start constraint.
    assert lane.borders[0].a == pytest.approx(-3.0, abs=1e-9)
