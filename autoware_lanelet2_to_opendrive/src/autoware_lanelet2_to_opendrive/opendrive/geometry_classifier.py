"""Curvature-based classifier that splits a fitted spline into <line>,
<arc>, and <paramPoly3> primitive runs.

Issue #466. The classifier is opt-in via ``ArcSpiralConfig.enabled``;
when disabled the existing paramPoly3-only path remains.
"""

from dataclasses import dataclass
from typing import Tuple

import numpy as np

from ..config import ArcSpiralConstants
from ..conversion_config import ArcSpiralConfig
from ..spline import Splines


class ClassifiedSegment:
    """Base class for arc-length runs returned by the classifier."""


@dataclass(frozen=True)
class LineRun(ClassifiedSegment):
    """A run of arc-length [s_start, s_end] classified as a straight line."""

    s_start: float
    s_end: float


@dataclass(frozen=True)
class ArcRun(ClassifiedSegment):
    """A run classified as a constant-curvature arc."""

    s_start: float
    s_end: float
    curvature: float


@dataclass(frozen=True)
class ParamPoly3Run(ClassifiedSegment):
    """A run that did not match any analytic primitive — paramPoly3 fallback."""

    s_start: float
    s_end: float


def _kappa_at(spline: Splines, s: float) -> float:
    """Return scalar curvature magnitude estimate at arc-length ``s``.

    Uses the second derivative of the unit-speed parametrisation, whose
    norm equals |κ| in 2D. Z component is ignored — the classifier
    operates on the XY projection.
    """
    accel = spline.evaluate(s, derivative=2)
    # In an arc-length parameterisation, ||T'(s)|| = |κ|.
    return float(np.hypot(accel[0], accel[1]))


def _signed_kappa_at(spline: Splines, s: float) -> float:
    """Return signed curvature κ at arc-length ``s`` (XY projection).

    κ = T_x · N_y - T_y · N_x where T is unit tangent and N is its
    derivative. Sign is positive for left turns relative to T.
    """
    tan = spline.evaluate(s, derivative=1)
    acc = spline.evaluate(s, derivative=2)
    return float(tan[0] * acc[1] - tan[1] * acc[0])


def _grow_line(
    spline: Splines,
    s_start: float,
    s_max: float,
    config: ArcSpiralConfig,
    constants: ArcSpiralConstants,
) -> float:
    """Return the largest s in [s_start, s_max] for which |κ| stays below
    ``config.line_curvature_tol``. Returns ``s_start`` if the start
    itself fails the predicate.
    """
    step = constants.classification_step
    s = s_start
    while s + step <= s_max:
        s_next = s + step
        if _kappa_at(spline, s_next) >= config.line_curvature_tol:
            return s
        s = s_next
    # Final partial step
    if s < s_max and _kappa_at(spline, s_max) < config.line_curvature_tol:
        return s_max
    return s


def _arc_position_error(
    spline: Splines,
    s_start: float,
    s_end: float,
    kappa: float,
    constants: ArcSpiralConstants,
) -> float:
    """Maximum XY deviation between the analytic arc model with constant
    curvature ``kappa`` (anchored at the spline's start) and the spline
    itself, sampled every ``constants.classification_step``.
    """
    from .geometry import evaluate_plan_view_world

    start = spline.evaluate(s_start, derivative=0)
    tan = spline.evaluate(s_start, derivative=1)
    hdg = float(np.arctan2(tan[1], tan[0]))
    x0, y0 = float(start[0]), float(start[1])

    max_err = 0.0
    s = s_start + constants.classification_step
    while s <= s_end:
        ref = spline.evaluate(s, derivative=0)
        wx, wy = evaluate_plan_view_world(x0, y0, hdg, s - s_start, arc_curvature=kappa)
        err = float(np.hypot(ref[0] - wx, ref[1] - wy))
        if err > max_err:
            max_err = err
        s += constants.classification_step
    return max_err


def _grow_arc(
    spline: Splines,
    s_start: float,
    s_max: float,
    config: ArcSpiralConfig,
    constants: ArcSpiralConstants,
) -> Tuple[float, float]:
    """Return (s_end, kappa) for the longest constant-curvature window
    starting at ``s_start``. Returns ``(s_start, 0.0)`` if no window of
    length >= ``config.min_arc_length`` survives both the κ-deviation
    test and the position-tolerance check.

    Algorithm:
    1. Greedy walk: advance while consecutive κ steps differ by less than
       ``3 * arc_curvature_tol``.  A looser rolling check is used (rather
       than a fixed reference) so that gradual spline-fitting drift near
       segment boundaries does not prematurely terminate the walk.
    2. Robust κ estimate: take the median of all collected samples as
       ``kappa_const``.  The median is insensitive to boundary spikes.
    3. Trim: shrink the candidate window from the right until every sample
       within it satisfies ``|κ(s) - kappa_const| < arc_curvature_tol``.
    4. Length and curvature guard: reject if the window is shorter than
       ``min_arc_length`` or the median κ is below ``line_curvature_tol``.
    5. Position check with bisection: verify the analytic arc model
       deviates from the spline by at most ``arc_position_tol``.  Bisect
       up to ``arc_fit_max_bisect`` times if the initial window fails.
    """
    if s_max - s_start < config.min_arc_length:
        return (s_start, 0.0)

    step = constants.classification_step

    # Phase 1: greedy forward walk using rolling consecutive-step tolerance.
    kappa_samples_s: list = [s_start]
    kappa_samples_v: list = [_signed_kappa_at(spline, s_start)]
    s = s_start
    while s + step <= s_max:
        s_next = s + step
        kappa_next = _signed_kappa_at(spline, s_next)
        kappa_prev = kappa_samples_v[-1]
        if abs(kappa_next - kappa_prev) >= 3.0 * config.arc_curvature_tol:
            break
        kappa_samples_s.append(s_next)
        kappa_samples_v.append(kappa_next)
        s = s_next

    # Phase 2: robust κ estimate — median of all greedy samples.
    kappa_const = float(np.median(kappa_samples_v))

    if abs(kappa_const) < config.line_curvature_tol:
        return (s_start, 0.0)

    # Phase 3: trim from the right to the last sample within arc_curvature_tol.
    s_end = s_start
    for ss, kk in zip(kappa_samples_s, kappa_samples_v):
        if abs(kk - kappa_const) < config.arc_curvature_tol:
            s_end = ss

    if s_end - s_start < config.min_arc_length:
        return (s_start, 0.0)

    # Phase 4: position-tolerance check with bisection.
    if (
        _arc_position_error(spline, s_start, s_end, kappa_const, constants)
        > config.arc_position_tol
    ):
        for _ in range(constants.arc_fit_max_bisect):
            s_end = s_start + (s_end - s_start) / 2.0
            if s_end - s_start < config.min_arc_length:
                return (s_start, 0.0)
            if (
                _arc_position_error(spline, s_start, s_end, kappa_const, constants)
                <= config.arc_position_tol
            ):
                break
        else:
            return (s_start, 0.0)

    return (s_end, kappa_const)
