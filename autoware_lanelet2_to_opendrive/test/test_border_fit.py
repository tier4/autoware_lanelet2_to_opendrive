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
