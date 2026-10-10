"""Regression tests for reference-line resampling before spline fitting.

The reference-line B-spline is fitted to the raw Lanelet2 boundary vertices
and its control-point count is derived from the vertex *count*, not from arc
length.  On a coarsely digitised boundary the fit is therefore
under-parameterised and unconstrained between vertices, and the resulting
curve bows metres away from the polyline it is meant to reproduce (lanelet
556 of the nishishinjuku map: 9 vertices over 159.72 m with a 50.7 m gap,
1.669 m off).  Resampling the polyline to a uniform spacing before fitting
bounds that error.
"""

from typing import List, Tuple

import lanelet2
import numpy as np
import pytest

from autoware_lanelet2_to_opendrive.config import DEFAULT_CONFIG
from autoware_lanelet2_to_opendrive.opendrive.reference_line import (
    ReferenceLine,
    resample_points_3d,
)

# Maximum distance (m) the fitted reference line may sit from the coarse
# source polyline.  The unfixed fit is off by ~1.6 m on the same geometry.
MAX_POLYLINE_DEVIATION_M = 0.10

# Tolerance (m) for endpoint overrides surviving the fit, matching the
# junction endpoint fidelity test.
ENDPOINT_TOLERANCE_M = 0.05


def _point_to_polyline_distance(point: np.ndarray, polyline: np.ndarray) -> float:
    """Return the shortest distance from ``point`` to a 2D ``polyline``."""
    best = np.inf
    for start, end in zip(polyline[:-1], polyline[1:]):
        segment = end - start
        squared = float(np.dot(segment, segment))
        t = (
            0.0
            if squared == 0.0
            else np.clip(np.dot(point - start, segment) / squared, 0.0, 1.0)
        )
        best = min(best, float(np.linalg.norm(point - (start + t * segment))))
    return best


def _symmetric_max_deviation(samples: np.ndarray, polyline: np.ndarray) -> float:
    """Symmetric point-to-segment distance between two 2D polylines."""
    forward = max(_point_to_polyline_distance(p, polyline) for p in samples)
    backward = max(_point_to_polyline_distance(v, samples) for v in polyline)
    return max(forward, backward)


def _coarse_arc_xy() -> np.ndarray:
    """A 9-vertex, 159.9 m arc whose longest gap is 50.7 m (cf. lanelet 556)."""
    segment_lengths = np.array([8.0, 10.0, 12.0, 50.7, 30.0, 20.0, 15.0, 14.0])
    fractions = (
        np.concatenate(([0.0], np.cumsum(segment_lengths))) / segment_lengths.sum()
    )
    radius, sweep = 250.0, 0.64
    return np.array(
        [
            [radius * np.sin(f * sweep), radius * (1.0 - np.cos(f * sweep))]
            for f in fractions
        ]
    )


def _build_single_lanelet_map(
    left_xy: np.ndarray, width: float = 3.5
) -> Tuple[lanelet2.core.LaneletMap, lanelet2.core.Lanelet]:
    """Build a one-lanelet map whose leftBound follows ``left_xy``."""
    left_points = [
        lanelet2.core.Point3d(i + 1, float(x), float(y), 0.0)
        for i, (x, y) in enumerate(left_xy)
    ]

    right_xy: List[List[float]] = []
    for i, (x, y) in enumerate(left_xy):
        ahead = left_xy[min(i + 1, len(left_xy) - 1)]
        behind = left_xy[max(i - 1, 0)]
        direction = np.asarray(ahead) - np.asarray(behind)
        direction = direction / (np.linalg.norm(direction) + 1e-12)
        right_xy.append([x + width * direction[1], y - width * direction[0]])
    right_points = [
        lanelet2.core.Point3d(1000 + i, float(x), float(y), 0.0)
        for i, (x, y) in enumerate(right_xy)
    ]

    lanelet = lanelet2.core.Lanelet(
        1,
        lanelet2.core.LineString3d(1, left_points),
        lanelet2.core.LineString3d(2, right_points),
    )
    lanelet_map = lanelet2.core.LaneletMap()
    lanelet_map.add(lanelet)
    return lanelet_map, lanelet


def _sample_reference_line(reference_line: ReferenceLine, step: float = 0.1):
    """Sample the reference line's XY geometry at roughly ``step`` intervals."""
    spline = reference_line.centerline_2d
    count = max(400, int(spline.total_length / step))
    return np.array(
        [spline.evaluate(s)[:2] for s in np.linspace(0.0, spline.total_length, count)]
    )


def test_coarse_boundary_reference_line_follows_polyline():
    """A coarsely digitised boundary must not bow away from its polyline."""
    left_xy = _coarse_arc_xy()
    lanelet_map, lanelet = _build_single_lanelet_map(left_xy)

    reference_line = ReferenceLine.construct_from_lanelet_groups(
        lanelet_map, [lanelet], traffic_rule="RHT"
    )

    deviation = _symmetric_max_deviation(
        _sample_reference_line(reference_line), left_xy
    )
    assert deviation < MAX_POLYLINE_DEVIATION_M, (
        f"Reference line deviates {deviation:.3f} m from a 9-vertex, "
        f"{np.sum(np.linalg.norm(np.diff(left_xy, axis=0), axis=1)):.1f} m boundary"
    )


def test_resampling_keeps_endpoint_overrides_exact():
    """Junction endpoint overrides must survive the resampling verbatim."""
    left_xy = _coarse_arc_xy()
    lanelet_map, lanelet = _build_single_lanelet_map(left_xy)

    start_override = (float(left_xy[0][0]) - 0.3, float(left_xy[0][1]) + 0.2, 1.5)
    end_override = (float(left_xy[-1][0]) + 0.4, float(left_xy[-1][1]) - 0.1, 2.5)

    reference_line = ReferenceLine.construct_from_lanelet_groups(
        lanelet_map,
        [lanelet],
        traffic_rule="RHT",
        start_xyz_override=start_override,
        end_xyz_override=end_override,
    )

    spline = reference_line.centerline_2d
    start_xy = spline.evaluate(0.0)[:2]
    end_xy = spline.evaluate(spline.total_length)[:2]

    assert (
        np.linalg.norm(start_xy - np.array(start_override[:2])) < ENDPOINT_TOLERANCE_M
    )
    assert np.linalg.norm(end_xy - np.array(end_override[:2])) < ENDPOINT_TOLERANCE_M
    assert reference_line.elevation_offset == pytest.approx(start_override[2])


def test_resample_points_3d_preserves_first_and_last_point():
    """Resampling never moves the endpoints of the polyline."""
    points = np.array(
        [[0.0, 0.0, 1.0], [30.0, 5.0, 2.0], [70.0, 40.0, 3.0]], dtype=float
    )

    resampled = resample_points_3d(points, DEFAULT_CONFIG.spline.resample_spacing)

    assert np.array_equal(resampled[0], points[0])
    assert np.array_equal(resampled[-1], points[-1])


def test_resample_points_3d_only_adds_points():
    """A densely digitised boundary is returned untouched."""
    dense = np.array(
        [[float(i) * 0.25, 0.0, float(i) * 0.01] for i in range(40)], dtype=float
    )

    resampled = resample_points_3d(dense, DEFAULT_CONFIG.spline.resample_spacing)

    assert resampled.shape == dense.shape
    assert np.array_equal(resampled, dense)


def test_resample_points_3d_respects_target_spacing():
    """Every gap longer than the target spacing is subdivided."""
    coarse = np.array([[0.0, 0.0, 0.0], [51.0, 0.0, 5.0]], dtype=float)
    spacing = DEFAULT_CONFIG.spline.resample_spacing

    resampled = resample_points_3d(coarse, spacing)

    gaps = np.linalg.norm(np.diff(resampled[:, :2], axis=0), axis=1)
    assert len(resampled) > len(coarse)
    assert gaps.max() <= spacing + 1e-9


def test_resample_points_3d_interpolates_z_against_2d_arc_length():
    """Inserted z values match a linear interpolation over the XY arc length."""
    points = np.array(
        [[0.0, 0.0, 10.0], [40.0, 0.0, 12.0], [40.0, 30.0, 11.0]], dtype=float
    )

    resampled = resample_points_3d(points, DEFAULT_CONFIG.spline.resample_spacing)

    source_s = np.concatenate(
        ([0.0], np.cumsum(np.linalg.norm(np.diff(points[:, :2], axis=0), axis=1)))
    )
    resampled_s = np.concatenate(
        ([0.0], np.cumsum(np.linalg.norm(np.diff(resampled[:, :2], axis=0), axis=1)))
    )
    expected_z = np.interp(resampled_s, source_s, points[:, 2])

    assert resampled[:, 2] == pytest.approx(expected_z, abs=1e-9)


def test_resample_points_3d_ignores_degenerate_input():
    """Zero-length 2D segments and single points do not break resampling."""
    single = np.array([[1.0, 2.0, 3.0]])
    assert np.array_equal(resample_points_3d(single, 0.5), single)

    duplicated = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [10.0, 0.0, 1.0]])
    resampled = resample_points_3d(duplicated, 0.5)
    assert np.array_equal(resampled[0], duplicated[0])
    assert np.array_equal(resampled[-1], duplicated[-1])
    assert len(resampled) > len(duplicated)
