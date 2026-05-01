"""Tests for Splines.find_closest_s — orthogonal projection of a 2-D point onto a spline."""

from __future__ import annotations

import numpy as np
import pytest

from autoware_lanelet2_to_opendrive.spline import Splines


def _straight_spline(length: float = 10.0, n: int = 11) -> Splines:
    """Build a horizontal spline along the x-axis with n equally-spaced control points."""
    points = np.array([[i * length / (n - 1), 0.0] for i in range(n)])
    return Splines(points, num_control_points=n)


def test_find_closest_s_on_segment() -> None:
    """A point directly above s=5 should project to s≈5."""
    spline = _straight_spline()
    s = spline.find_closest_s(np.array([5.0, 1.5]))
    assert s == pytest.approx(5.0, abs=5e-3)


def test_find_closest_s_at_start_clamp() -> None:
    """A point left of the spline's start should clamp to s=0."""
    spline = _straight_spline()
    s = spline.find_closest_s(np.array([-2.0, 0.0]))
    assert s == pytest.approx(0.0, abs=5e-3)


def test_find_closest_s_at_end_clamp() -> None:
    """A point right of the spline's end should clamp to s=total_length."""
    spline = _straight_spline()
    s = spline.find_closest_s(np.array([12.0, 0.0]))
    assert s == pytest.approx(spline.total_length, abs=5e-3)


def test_find_closest_s_curved_spline() -> None:
    """A point near the apex of a curved spline projects to the apex parameter."""
    angles = np.linspace(0.0, np.pi / 2.0, 21)
    points = np.array([[10.0 * np.sin(a), 10.0 * (1 - np.cos(a))] for a in angles])
    spline = Splines(points, num_control_points=21)

    mid_a = np.pi / 4.0
    p = np.array([10.0 * np.sin(mid_a) + 0.1, 10.0 * (1 - np.cos(mid_a)) - 0.1])

    s = spline.find_closest_s(p)
    assert 0.4 * spline.total_length <= s <= 0.6 * spline.total_length


def test_find_closest_s_multimodal_returns_valid_minimum() -> None:
    """A point equidistant from two arms of a U-shaped spline projects to a valid local minimum.

    The U has its apex at the origin facing +y; a target on the y-axis above the apex is
    equidistant from the two symmetric arms, so the grid+Newton solver may settle on either
    arm. The contract is only that the returned s yields a distance no worse than the apex.
    """
    xs = np.linspace(-5.0, 5.0, 21)
    points = np.array([[x, x * x / 5.0] for x in xs])
    spline = Splines(points, num_control_points=21)

    target = np.array([0.0, 4.0])
    s = spline.find_closest_s(target)

    pt = spline.evaluate(s)[:2]
    d_returned = float(np.linalg.norm(pt - target))
    apex_pt = spline.evaluate(spline.total_length / 2.0)[:2]
    d_apex = float(np.linalg.norm(apex_pt - target))
    assert d_returned <= d_apex + 1e-6


def test_find_closest_s_degenerate_short_spline() -> None:
    """A spline that is essentially a single point still yields a valid s in [0, L]."""
    base_x, base_y = 1.0, 2.0
    points = np.array([[base_x + i * 1e-9, base_y] for i in range(4)])
    spline = Splines(points, num_control_points=4)

    s = spline.find_closest_s(np.array([5.0, 5.0]))
    assert 0.0 <= s <= spline.total_length


def test_find_closest_s_validates_inputs() -> None:
    """Invalid n_seeds, max_iter, tol, or point_xy shape raise ValueError."""
    spline = _straight_spline()
    with pytest.raises(ValueError, match="n_seeds"):
        spline.find_closest_s(np.array([0.0, 0.0]), n_seeds=1)
    with pytest.raises(ValueError, match="max_iter"):
        spline.find_closest_s(np.array([0.0, 0.0]), max_iter=0)
    with pytest.raises(ValueError, match="tol"):
        spline.find_closest_s(np.array([0.0, 0.0]), tol=0.0)
    with pytest.raises(ValueError, match="point_xy"):
        spline.find_closest_s(np.array([0.0, 0.0, 0.0]))
