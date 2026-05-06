"""Curvature-based classifier that splits a fitted spline into <line>,
<arc>, and <paramPoly3> primitive runs.

Issue #466. The classifier is opt-in via ``ArcSpiralConfig.enabled``;
when disabled the existing paramPoly3-only path remains.
"""

from dataclasses import dataclass


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
