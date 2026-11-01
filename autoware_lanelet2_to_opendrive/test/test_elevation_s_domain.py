"""The ``<elevationProfile>`` covers exactly the road's declared s-domain.

A ``<road>`` declares one s-domain, ``road@length``, and everything written
against it -- ``<geometry>``, ``<laneSection>``, ``<elevation>`` -- has to mean
the same interval.  ``road@length`` is the sum of the emitted geometries'
``@length``.  The elevation profile used to end at
``ReferenceLine.height_spline.total_length`` instead, which is the arc length
of the *fitted* reference-line spline and a different number: over
Nishi-Shinjuku's 650 roads the two differ by up to 265 um (p50 0.81 um).

Evaluating the profile at ``s = road@length`` therefore ran past the last
cubic's intended end by that overshoot, giving an elevation wrong by
overshoot x local gradient -- up to 12.1 um on 6 of 650 roads.  Small, but it
is a bookkeeping error that presents as a geometry error, which is the
expensive kind to chase.

These tests are about the *domain*, not about whether the elevation values
themselves are right.  A separate and much larger defect exists in how the
height spline is sampled (the table is indexed by the source polyline's arc
length but queried on the fitted curve's); that one is millimetre-to-
centimetre scale and is not what is asserted here.
"""

from __future__ import annotations

import numpy as np
import pytest

from autoware_lanelet2_to_opendrive.cubic_spline_1d import CubicSpline1D
from autoware_lanelet2_to_opendrive.opendrive.elevation import ElevationProfile
from autoware_lanelet2_to_opendrive.opendrive.reference_line import ReferenceLine


def _evaluate(profile: ElevationProfile, s: float) -> float:
    """Evaluate a profile the way a consumer reads the emitted XML.

    Deliberately a fresh implementation of ``z = a + b*ds + c*ds^2 + d*ds^3``
    rather than a call back into the emitter, so that a change to the emitter's
    own arithmetic cannot make these tests agree with it by construction.
    """
    z = 0.0
    for elevation in profile.elevations:
        if elevation.s > s:
            break
        ds = s - elevation.s
        z = elevation.a + elevation.b * ds + elevation.c * ds**2 + elevation.d * ds**3
    return z


def _reference_line(
    length: float = 10.0, elevation_offset: float = 42.0
) -> ReferenceLine:
    """A reference line whose height spline has a known, non-flat profile.

    ``centerline_2d`` is never touched by ``get_elevation_profile``, so it is
    left as ``None``: passing a real ``Splines`` would only add a fit that the
    assertions do not depend on.
    """
    s = np.linspace(0.0, length, 21)
    # Non-zero first, second and third derivatives, so that an error in the
    # segment domain shows up in the value rather than cancelling.
    z = 0.03 * s + 0.004 * s**2 - 0.0002 * s**3
    return ReferenceLine(
        centerline_2d=None,  # type: ignore[arg-type]
        height_spline=CubicSpline1D(s, z, bc_type="not-a-knot"),
        elevation_offset=elevation_offset,
    )


def test_profile_ends_at_road_length_not_spline_length() -> None:
    """The last segment reaches the road's declared end exactly.

    ``road_length`` is set 2.6e-4 m beyond the height spline's own domain --
    the largest discrepancy measured on Nishi-Shinjuku -- so the test fails if
    the final boundary is taken from the spline instead.
    """
    ref = _reference_line(length=10.0)
    geometry_s_values = [0.0, 3.0, 6.0, 9.0]
    road_length = 10.000265

    profile = ref.get_elevation_profile(geometry_s_values, road_length=road_length)

    last = profile.elevations[-1]
    # Reconstruct the end of the last segment from its own coefficients: the
    # value there must be the height spline's end value, offset to absolute.
    expected = ref.height_spline.evaluate(road_length) + ref.elevation_offset
    assert _evaluate(profile, road_length) == pytest.approx(expected, abs=1e-12)

    # And the segment really does span to the road end, rather than the test
    # passing because the spline happens to be flat there.
    assert last.s == pytest.approx(geometry_s_values[-1])
    assert ref.height_spline.evaluate(road_length, derivative=1) != 0.0


def test_omitting_road_length_reproduces_the_old_domain() -> None:
    """The default is unchanged, so no caller silently shifts behaviour."""
    ref = _reference_line(length=10.0)
    geometry_s_values = [0.0, 3.0, 6.0, 9.0]

    without = ref.get_elevation_profile(geometry_s_values)
    explicit = ref.get_elevation_profile(
        geometry_s_values, road_length=ref.height_spline.total_length
    )

    assert len(without.elevations) == len(explicit.elevations)
    for lhs, rhs in zip(without.elevations, explicit.elevations):
        assert (lhs.s, lhs.a, lhs.b, lhs.c, lhs.d) == (
            rhs.s,
            rhs.a,
            rhs.b,
            rhs.c,
            rhs.d,
        )


def test_domain_error_is_what_the_fix_removes() -> None:
    """Fault injection: without the fix the end elevation is wrong, with it exact.

    The two branches are compared against the same ground truth, so this
    measures the fix rather than restating it. The control is the second
    assertion: a road whose declared length matches the spline's is unaffected
    either way, which is why 644 of Nishi-Shinjuku's 650 roads show nothing.
    """
    ref = _reference_line(length=10.0)
    geometry_s_values = [0.0, 3.0, 6.0, 9.0]
    road_length = 10.000265
    truth = ref.height_spline.evaluate(road_length) + ref.elevation_offset

    old = ref.get_elevation_profile(geometry_s_values)  # ends at spline length
    new = ref.get_elevation_profile(geometry_s_values, road_length=road_length)

    error_old = abs(_evaluate(old, road_length) - truth)
    error_new = abs(_evaluate(new, road_length) - truth)

    # The injected 265 um of s times the gradient there is a real error.
    assert error_old > 1e-7, f"injection did not bite: {error_old:.3e}"
    assert error_new < 1e-12, f"fix did not remove it: {error_new:.3e}"

    # No-op control: matched domains, nothing to fix, both exact.
    matched = ref.height_spline.total_length
    truth_matched = ref.height_spline.evaluate(matched) + ref.elevation_offset
    assert (
        abs(
            _evaluate(ref.get_elevation_profile(geometry_s_values), matched)
            - truth_matched
        )
        < 1e-12
    )
    assert (
        abs(
            _evaluate(
                ref.get_elevation_profile(geometry_s_values, road_length=matched),
                matched,
            )
            - truth_matched
        )
        < 1e-12
    )


def test_profile_is_continuous_across_its_own_segment_boundaries() -> None:
    """Consecutive segments agree where they meet.

    Each segment is a Hermite fit of the same underlying spline, so this holds
    by construction -- which is exactly why it is worth asserting: if the
    boundary list and the evaluation ever disagree about where a segment
    starts, this is what breaks first.
    """
    ref = _reference_line(length=10.0)
    geometry_s_values = [0.0, 2.5, 5.0, 7.5]
    profile = ref.get_elevation_profile(geometry_s_values, road_length=10.0)

    for index in range(len(profile.elevations) - 1):
        current = profile.elevations[index]
        following = profile.elevations[index + 1]
        ds = following.s - current.s
        from_left = current.a + current.b * ds + current.c * ds**2 + current.d * ds**3
        assert from_left == pytest.approx(following.a, abs=1e-12)
