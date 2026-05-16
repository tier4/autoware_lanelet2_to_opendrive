"""Curvature-based classifier that splits a fitted spline into <line>,
<arc>, and <paramPoly3> primitive runs.

Issue #466. The classifier is opt-in via ``ArcSpiralConfig.enabled``;
when disabled the existing paramPoly3-only path remains.
"""

from dataclasses import dataclass
from typing import List, Tuple

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
    """Return the largest s in [s_start, s_max] over which the spline is a
    straight line, or ``s_start`` if no line of at least
    ``config.min_line_length`` qualifies.

    The straightness decision is noise-robust (#496), mirroring _grow_arc:

    1. Greedy walk while the *median* signed curvature of the window stays
       below ``config.line_curvature_tol`` in magnitude. The median
       ignores the isolated curvature spikes a B-spline fit introduces on
       an otherwise straight road; a point-wise threshold (the previous
       design) terminated on the first spike and so almost never reached
       ``min_line_length``. The test is applied only once the window is at
       least ``min_line_length`` long, so the median is taken over enough
       samples to be robust to the spikes at the spline endpoints.
    2. Validate with a straight-line (κ = 0) position-tolerance check,
       binary-searching the largest sub-window within
       ``config.arc_position_tol``. The position error grows monotonically
       with window length, so the search converges on the exact boundary.
    """
    if s_max - s_start < config.min_line_length:
        return s_start

    step = constants.classification_step

    # Phase 1: greedy walk while the window stays robustly straight.
    kappa_samples: List[float] = [_signed_kappa_at(spline, s_start)]
    s = s_start
    s_end = s_start
    while s + step <= s_max:
        s_next = s + step
        kappa_samples.append(_signed_kappa_at(spline, s_next))
        if (
            s_next - s_start >= config.min_line_length
            and abs(float(np.median(kappa_samples))) >= config.line_curvature_tol
        ):
            break
        s = s_next
        s_end = s_next
    else:
        # Reached s_max without hitting curvature — snap the final
        # sub-step gap so the run does not leave a sliver too short for
        # the paramPoly3 fallback to emit (which would shorten the
        # planView and move the road endpoint). Phase 2 still validates.
        s_end = s_max

    if s_end - s_start < config.min_line_length:
        return s_start

    # Phase 2: straight-line position-tolerance check with binary search.
    if (
        _arc_position_error(spline, s_start, s_end, 0.0, constants)
        > config.arc_position_tol
    ):
        lo, hi = s_start, s_end
        for _ in range(constants.arc_fit_max_bisect):
            mid = 0.5 * (lo + hi)
            if (
                _arc_position_error(spline, s_start, mid, 0.0, constants)
                <= config.arc_position_tol
            ):
                lo = mid
            else:
                hi = mid
        s_end = lo

    if s_end - s_start < config.min_line_length:
        return s_start
    return s_end


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


def _grow_paramPoly3(
    s_start: float,
    s_max: float,
    constants: ArcSpiralConstants,
) -> float:
    """Advance s_start by ``lookahead_steps * classification_step`` (or to
    ``s_max``). The classifier will retry line / arc detection at the
    returned end-s.
    """
    advance = constants.lookahead_steps * constants.classification_step
    return min(s_max, s_start + advance)


def classify_spline(
    spline: Splines,
    config: ArcSpiralConfig,
    constants: ArcSpiralConstants,
) -> List[ClassifiedSegment]:
    """Walk ``spline`` from s=0 and produce a contiguous list of
    classified segments covering [0, total_length]. Greedy growth in
    priority order: line → arc → paramPoly3 fallback.
    """
    L = float(spline.total_length)
    s = 0.0
    out: List[ClassifiedSegment] = []
    while s < L - 1e-9:
        s_line_end = _grow_line(spline, s, L, config, constants)
        if s_line_end - s >= config.min_line_length:
            out.append(LineRun(s_start=s, s_end=s_line_end))
            s = s_line_end
            continue

        if config.arc_enabled:
            s_arc_end, kappa = _grow_arc(spline, s, L, config, constants)
            if s_arc_end - s >= config.min_arc_length:
                out.append(ArcRun(s_start=s, s_end=s_arc_end, curvature=kappa))
                s = s_arc_end
                continue

        s_pp3_end = _grow_paramPoly3(s, L, constants)
        # Coalesce consecutive paramPoly3 runs to avoid micro-fragmentation.
        if out and isinstance(out[-1], ParamPoly3Run):
            prev = out[-1]
            out[-1] = ParamPoly3Run(s_start=prev.s_start, s_end=s_pp3_end)
        else:
            out.append(ParamPoly3Run(s_start=s, s_end=s_pp3_end))
        s = s_pp3_end
    return out
