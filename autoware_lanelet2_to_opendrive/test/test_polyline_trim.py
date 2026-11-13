"""Trimming a boundary polyline back from an end, for the junction carve.

The carve ends an incoming road short of the junction and gives the freed
stretch to a connecting road, which is how published OpenDRIVE maps model a
fork (no connector in the esmini corpus is under 9 m; ours are 0.01 m).

`ReferenceLine.construct_from_lanelet_groups` samples the boundary polyline,
applies the junction endpoint overrides, resamples, and only then fits the
spline. Shortening the polyline at that point costs no re-parameterisation of
the fitted geometry: the fitter simply runs on a shorter line. These tests pin
the primitive that does the shortening.

Lengths are measured in 2D (XY) arc length, as `resample_points_3d` measures
them, so that the trim and the resampling agree about what a metre is.
"""

from __future__ import annotations

import math

import numpy as np

from autoware_lanelet2_to_opendrive.opendrive.reference_line import trim_polyline


def _xy_length(points: np.ndarray) -> float:
    return float(np.sum(np.linalg.norm(np.diff(points[:, :2], axis=0), axis=1)))


def _straight_line(n: int = 11, length: float = 10.0) -> np.ndarray:
    """A straight 10 m line in XY rising 1 m in z over its length."""
    s = np.linspace(0.0, length, n)
    return np.column_stack([s, np.zeros(n), s / length])


def test_trimming_the_end_shortens_by_exactly_the_requested_length() -> None:
    points = _straight_line()
    assert math.isclose(_xy_length(points), 10.0)

    trimmed = trim_polyline(points, trim_end=2.5)
    assert trimmed is not None

    assert math.isclose(
        _xy_length(trimmed), 7.5, abs_tol=1e-9
    ), "the trimmed polyline must be shorter by exactly the trim length"


def test_trimming_the_end_keeps_the_start_untouched() -> None:
    points = _straight_line()
    trimmed = trim_polyline(points, trim_end=2.5)
    assert trimmed is not None
    assert np.allclose(trimmed[0], points[0]), "the start point must not move"
    assert _xy_length(trimmed) < _xy_length(
        points
    ), "guard: the start staying put must not be because nothing was trimmed"


def test_the_new_end_lies_on_the_original_line() -> None:
    """The cut point is interpolated, not snapped to a vertex."""
    points = _straight_line()
    trimmed = trim_polyline(points, trim_end=2.55)
    assert trimmed is not None

    # 10 m line, 2.55 m removed -> the new end is at x = 7.45, y = 0,
    # and z rises linearly, so z = 0.745.
    assert np.allclose(
        trimmed[-1], [7.45, 0.0, 0.745], atol=1e-9
    ), f"the cut point is wrong: {trimmed[-1]}"


def test_trimming_the_start_shortens_from_the_other_end() -> None:
    points = _straight_line()
    trimmed = trim_polyline(points, trim_start=2.5)
    assert trimmed is not None

    assert math.isclose(_xy_length(trimmed), 7.5, abs_tol=1e-9)
    assert np.allclose(trimmed[-1], points[-1]), "the end point must not move"
    assert np.allclose(trimmed[0], [2.5, 0.0, 0.25], atol=1e-9)


def test_trimming_both_ends_removes_both_lengths() -> None:
    points = _straight_line()
    trimmed = trim_polyline(points, trim_start=1.0, trim_end=2.0)
    assert trimmed is not None

    assert math.isclose(_xy_length(trimmed), 7.0, abs_tol=1e-9)
    assert np.allclose(trimmed[0], [1.0, 0.0, 0.1], atol=1e-9)
    assert np.allclose(trimmed[-1], [8.0, 0.0, 0.8], atol=1e-9)


def test_no_trim_returns_the_polyline_unchanged() -> None:
    """The default must be a no-op, so the conversion is untouched."""
    points = _straight_line()
    assert trim_polyline(points) is points


def test_a_trim_that_would_consume_the_polyline_is_refused() -> None:
    """A road that cannot spare the length must be left alone, not collapsed.

    Three roads on Nishishinjuku are shorter than 1 m, so the caller has to be
    able to tell that the trim is impossible rather than receive a degenerate
    line.
    """
    points = _straight_line()
    assert trim_polyline(points, trim_end=10.0) is None
    assert trim_polyline(points, trim_start=6.0, trim_end=6.0) is None


def test_a_bend_is_trimmed_along_its_arc_not_its_chord() -> None:
    """Arc length, not straight-line distance between the endpoints."""
    # A right-angle corner: 5 m east, then 5 m north. Arc length 10 m,
    # chord 7.071 m.
    points = np.array(
        [
            [0.0, 0.0, 0.0],
            [5.0, 0.0, 0.0],
            [5.0, 5.0, 0.0],
        ]
    )
    assert math.isclose(_xy_length(points), 10.0)

    trimmed = trim_polyline(points, trim_end=2.0)
    assert trimmed is not None

    assert math.isclose(_xy_length(trimmed), 8.0, abs_tol=1e-9)
    # 2 m removed from the northward leg leaves it 3 m long.
    assert np.allclose(trimmed[-1], [5.0, 3.0, 0.0], atol=1e-9)


def test_vertices_inside_the_kept_stretch_survive_verbatim() -> None:
    """Only the trimmed tail is replaced; the rest is not resampled."""
    points = _straight_line(n=11)
    trimmed = trim_polyline(points, trim_end=2.0)
    assert trimmed is not None

    # 10 m over 11 points = 1 m spacing; removing 2 m leaves vertices
    # 0..8 untouched and the cut lands exactly on vertex 8.
    assert len(trimmed) == 9
    assert np.allclose(trimmed, points[:9])
