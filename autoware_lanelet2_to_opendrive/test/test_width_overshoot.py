"""The emitted width must stay inside the envelope of its own samples.

``CubicSpline`` with ``not-a-knot`` is C2 and achieves it by overshooting its
data between knots. On a width profile that narrows and widens again, the
emitted maximum exceeds every sample: given 3.5, 3.5, 3.2, 3.5, 3.5 m it
emits 3.623 m, so 123.5 mm of the exported lane width is width the road does
not have.

Measured on nishishinjuku, across the 892 lanes with at least three width
segments and strictly increasing knots, the width emitted beyond the range of
the fit's own knots is p50 16.9 mm, p95 428.5 mm, p99 1,105.1 mm, max
1,886.5 mm. Refitting the same knots with PCHIP gives exactly 0.0 mm at every
percentile, and is worse than ``not-a-knot`` on none of them.

What this does **not** fix: the knot values themselves, which span p50
114.6 mm and p95 1,569.1 mm on the same map. That is the boundary pairing in
``_estimate_width_samples``, not the interpolant.
"""

import numpy as np
import pytest

from autoware_lanelet2_to_opendrive.config import DEFAULT_CONFIG
from autoware_lanelet2_to_opendrive.cubic_spline_1d import CubicSpline1D

# A road that narrows by 30 cm and widens again -- the shape that oscillates.
KNOTS = np.array([0.0, 10.0, 20.0, 30.0, 40.0])
WIDTHS = np.array([3.5, 3.5, 3.2, 3.5, 3.5])


def _sampled(spline: CubicSpline1D, count: int = 801) -> np.ndarray:
    grid = np.linspace(KNOTS[0], KNOTS[-1], count)
    return np.array([spline.evaluate(float(s)) for s in grid])


@pytest.mark.xfail(
    strict=True,
    reason="the width interpolant overshoots its own samples; reported " "2026-10-23",
)
def test_the_default_width_interpolant_stays_within_its_samples() -> None:
    """The width shipped to a consumer must not exceed every sample."""
    values = _sampled(
        CubicSpline1D(
            KNOTS,
            WIDTHS,
            method=DEFAULT_CONFIG.centerline.width_interpolation_method,
        )
    )

    overshoot_mm = (values.max() - WIDTHS.max()) * 1000.0
    undershoot_mm = (WIDTHS.min() - values.min()) * 1000.0
    assert overshoot_mm == pytest.approx(0.0, abs=1e-6), (
        f"emitted width exceeds every sample by {overshoot_mm:.3f} mm "
        f"(max sample {WIDTHS.max()} m, emitted {values.max():.6f} m)"
    )
    assert undershoot_mm == pytest.approx(0.0, abs=1e-6)


@pytest.mark.xfail(
    strict=True,
    reason="the width interpolant overshoots its own samples; reported " "2026-10-23",
)
def test_pchip_is_the_default_for_width() -> None:
    """Pins the default, so a later edit cannot silently restore overshoot."""
    assert DEFAULT_CONFIG.centerline.width_interpolation_method == "pchip"


def test_the_cubic_interpolant_is_still_reachable_and_still_overshoots() -> None:
    """Keeps the comparison honest and the old behaviour available.

    Documents *why* the default moved, and guards that ``method="cubic"``
    still means what it meant.
    """
    values = _sampled(CubicSpline1D(KNOTS, WIDTHS, method="cubic"))

    assert (values.max() - WIDTHS.max()) * 1000.0 > 100.0


def test_both_interpolants_pass_through_every_sample() -> None:
    """Neither is a smoother: an interpolant must hit its knots."""
    for method in ("pchip", "cubic"):
        spline = CubicSpline1D(KNOTS, WIDTHS, method=method)
        for knot, width in zip(KNOTS, WIDTHS):
            assert spline.evaluate(float(knot)) == pytest.approx(width), method


def test_an_unknown_interpolation_method_is_rejected() -> None:
    """A typo in the config must fail loudly, not fall back silently."""
    with pytest.raises(ValueError, match="Unknown interpolation method"):
        CubicSpline1D(KNOTS, WIDTHS, method="not-a-knot")


def test_the_segment_coefficients_describe_the_emitted_curve() -> None:
    """PCHIP's coefficient matrix must read the same way CubicSpline's does.

    ``get_polynomial_coefficients`` indexes ``spline.c`` directly, so the
    emitted ``<width>`` is only correct if PCHIP lays its columns out the
    same way -- about the segment start, highest power first.
    """
    spline = CubicSpline1D(KNOTS, WIDTHS, method="pchip")
    for index in range(spline.num_segments):
        a, b, c, d = spline.get_polynomial_coefficients(index)
        s0 = KNOTS[index]
        for t in (0.0, 2.5, 7.5):
            if s0 + t > KNOTS[-1]:
                continue
            expected = spline.evaluate(float(s0 + t))
            assert a + b * t + c * t * t + d * t**3 == pytest.approx(expected)
