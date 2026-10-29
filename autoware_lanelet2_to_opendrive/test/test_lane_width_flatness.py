"""A road of constant width must emit a constant ``<width>``.

Reported 2026-10-23: a straight road of constant width converts to a lane
whose edge wobbles, because ``<width>`` carries non-zero ``b``, ``c`` and
``d`` where the road is flat.

Measured here, the converter does **not** invent that curvature. Given width
samples that are exactly equal it returns exact zeros -- see
``test_exactly_parallel_boundaries_need_no_flattening``. What it does is pass
sub-micrometre noise straight through: boundaries parallel to within 0.5 um
produce non-zero coefficients in **every** segment, and the conversion of a
real map emitted zero exactly-constant ``<width>`` elements out of 4,659.

A node coordinate is finite and goes through a projection, so a road built to
one constant width never reaches the converter as one constant number. The
coefficients that describe less width than the input's own precision are
therefore not information, and emitting them makes a flat road unreadable as
flat.
"""

from typing import List, Tuple

import pytest

import lanelet2

from autoware_lanelet2_to_opendrive.centerline import estimate_lanelet_width_as_spline
from autoware_lanelet2_to_opendrive.conversion_config import (
    WidthEstimationConfig,
    WidthReference,
)
from autoware_lanelet2_to_opendrive.opendrive.enums import LaneType
from autoware_lanelet2_to_opendrive.opendrive.lane import Lane

HALF_WIDTH = 1.75
LENGTH = 100.0
NUM_POINTS = 11


class _StubWidthSpline:
    """The minimum surface ``_add_width_from_spline`` consumes."""

    def __init__(
        self,
        segments: List[Tuple[float, float, float, float, float]],
        total_length: float,
    ) -> None:
        self._segments = segments
        self.total_length = total_length

    def get_polynomial_segments(
        self,
    ) -> List[Tuple[float, float, float, float, float]]:
        return self._segments


def _emit(segments, total_length):
    """Run the segments through the emitter and return its width elements."""
    lane = Lane(lane_id=-1, lane_type=LaneType.DRIVING)
    lane._add_width_from_spline(
        _StubWidthSpline(segments, total_length), road_length=total_length
    )
    return lane.widths


def _lanelet(lanelet_id: int, right_offsets: List[float]) -> "lanelet2.core.Lanelet":
    """A straight lanelet whose right bound sits at the given lateral offsets."""
    left, right = [], []
    for index, offset in enumerate(right_offsets):
        x = LENGTH * index / (len(right_offsets) - 1)
        left.append(
            lanelet2.core.Point3d(lanelet_id * 100 + index, x, +HALF_WIDTH, 0.0)
        )
        right.append(
            lanelet2.core.Point3d(lanelet_id * 100 + 50 + index, x, offset, 0.0)
        )
    return lanelet2.core.Lanelet(
        lanelet_id,
        lanelet2.core.LineString3d(lanelet_id * 100 + 90, left),
        lanelet2.core.LineString3d(lanelet_id * 100 + 91, right),
    )


def _fit_and_emit(lanelet) -> List:
    spline = estimate_lanelet_width_as_spline(
        lanelet,
        WidthEstimationConfig(num_samples=10, reference=WidthReference.LEFT_BOUND),
    )
    return _emit(spline.get_polynomial_segments(), spline.total_length)


def _curved(widths) -> List[Tuple[float, float, float, float]]:
    return [
        (w.s_offset, w.b, w.c, w.d)
        for w in widths
        if (w.b, w.c, w.d) != (0.0, 0.0, 0.0)
    ]


@pytest.mark.xfail(
    strict=True,
    reason="width coefficients below the input's own precision are passed "
    "through; reported 2026-10-23",
)
def test_noise_level_coefficients_are_emitted_as_exact_zeros() -> None:
    """The coefficients measured on a real conversion must come out zero.

    Over the 5.005 m span they were measured on, these three terms together
    add 51 nm of width.
    """
    (width,) = _emit(
        [
            (
                0.0,
                3.4876740949583320,
                -1.021261674743996e-08,
                -3.969409710449679e-12,
                1.2602748117774818e-12,
            )
        ],
        5.005,
    )

    assert (width.b, width.c, width.d) == (0.0, 0.0, 0.0)
    assert width.a == pytest.approx(3.4876740949583320)


@pytest.mark.xfail(
    strict=True,
    reason="width coefficients below the input's own precision are passed "
    "through; reported 2026-10-23",
)
def test_boundaries_parallel_to_within_a_micrometre_emit_constant_width() -> None:
    """Sub-micrometre boundary noise must not become a width curve.

    Jittering the right bound by +/-0.5 um -- far below any surveyed
    precision -- makes the fit return non-zero coefficients in all nine
    segments, describing 380 nm of width variation.
    """
    rng = __import__("random").Random(7)
    offsets = [-HALF_WIDTH + rng.uniform(-5e-7, 5e-7) for _ in range(NUM_POINTS)]
    widths = _fit_and_emit(_lanelet(7003, offsets))

    assert widths, "no <width> produced"
    assert not _curved(widths), (
        f"{len(_curved(widths))}/{len(widths)} segments curved on a road that is "
        f"flat to within a micrometre: {_curved(widths)[:2]}"
    )


def test_a_real_taper_keeps_its_coefficients() -> None:
    """A width that genuinely changes must survive untouched.

    1 cm of width per metre over 10 m is 10 cm of width -- four orders of
    magnitude above the tolerance. Flattening it would be a worse defect than
    the one being fixed.
    """
    (width,) = _emit([(0.0, 3.5, 0.01, 0.0, 0.0)], 10.0)

    assert width.b == pytest.approx(0.01)


def test_exactly_parallel_boundaries_need_no_flattening() -> None:
    """Pins where the noise does *not* come from.

    Given samples that are exactly equal, the fit already returns exact
    zeros, so the converter is not the source of the curvature -- the input
    is. This guard holds both before and after the flattening and exists to
    keep that attribution from being re-litigated.
    """
    widths = _fit_and_emit(_lanelet(7001, [-HALF_WIDTH] * NUM_POINTS))

    assert widths, "no <width> produced"
    assert not _curved(widths)
