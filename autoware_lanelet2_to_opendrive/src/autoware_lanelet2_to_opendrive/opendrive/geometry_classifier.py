"""Curvature-based classifier that splits a fitted spline into <line>,
<arc>, and <paramPoly3> primitive runs.

Issue #466. The classifier is opt-in via ``ArcSpiralConfig.enabled``;
when disabled the existing paramPoly3-only path remains.
"""

from dataclasses import dataclass

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
