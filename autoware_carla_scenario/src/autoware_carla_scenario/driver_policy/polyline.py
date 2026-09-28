"""Arc-length operations on polylines, shared by the driver and the runtime.

Routes and plans are both polylines, and both need the same questions answered:
how long is it, where is the point at distance ``d``, and which way does it head
there.

Ported from ``carla_driver_interface.polyline``; pure numpy.

Everything is measured in the **xy plane**: these are ground routes, and a route
that climbs a hill is not longer for it. ``z`` is carried through interpolation
but never contributes to a distance.
"""

from __future__ import annotations

import math

import numpy as np

__all__ = ["arc_lengths", "sample", "segment_heading"]


def arc_lengths(points: np.ndarray) -> np.ndarray:
    """Cumulative xy arc length along ``points``, starting at 0.

    Returns an array the same length as ``points`` (empty in, empty out).
    """
    points = np.asarray(points, dtype=np.float64)
    if len(points) == 0:
        return np.zeros(0, dtype=np.float64)
    deltas = np.linalg.norm(np.diff(points[:, :2], axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(deltas)])


def sample(
    points: np.ndarray,
    arc: np.ndarray,
    distances: np.ndarray | float,
    *,
    extrapolate: bool = False,
) -> np.ndarray:
    """Points at the given arc-length ``distances`` along the polyline.

    Vectorised: pass the whole set of distances at once. This is the hot path --
    the runtime resamples the route window every policy step -- so a scalar
    version called in a loop is the thing to avoid.

    Args:
        points: ``(N, 3)`` polyline vertices.
        arc: ``arc_lengths(points)``, passed in so callers can reuse it.
        distances: scalar or ``(M,)`` arc lengths to sample at.
        extrapolate: past the end, continue along the final segment's heading
            instead of clamping to the last vertex. Before the start always
            clamps -- there is no defined heading to run backwards along.

    Returns:
        ``(M, 3)``, or ``(3,)`` when ``distances`` is scalar.
    """
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    if len(points) < 2:
        raise ValueError("sampling needs at least two points")

    scalar = np.isscalar(distances) or np.ndim(distances) == 0
    queries = np.atleast_1d(np.asarray(distances, dtype=np.float64))
    total = float(arc[-1])

    # np.interp clamps at both ends, which is the behaviour we want everywhere
    # except past the end under `extrapolate`.
    sampled = np.stack(
        [np.interp(queries, arc, points[:, axis]) for axis in range(3)], axis=1
    )

    if extrapolate:
        beyond = queries > total
        if beyond.any():
            heading = segment_heading(points[-2], points[-1])
            direction = np.array([math.cos(heading), math.sin(heading), 0.0])
            overshoot = (queries[beyond] - total)[:, None]
            sampled[beyond] = points[-1] + overshoot * direction

    return sampled[0] if scalar else sampled


def segment_heading(p0: np.ndarray, p1: np.ndarray) -> float:
    """Heading in radians from ``p0`` to ``p1``, in the xy plane."""
    return math.atan2(float(p1[1] - p0[1]), float(p1[0] - p0[0]))
