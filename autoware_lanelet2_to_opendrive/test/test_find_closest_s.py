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
