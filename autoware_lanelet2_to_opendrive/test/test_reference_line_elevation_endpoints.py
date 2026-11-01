"""The reference line must reproduce its source elevation at *both* endpoints.

``asam.net:xodr:1.7.0:road.geometry.contact_point`` compares a road's
reference-line endpoint against the linked road's ``contactPoint`` endpoint
with a 1e-6 m gate on each of x, y and z independently.  At a plain
road-to-road joint the two roads share a Lanelet2 boundary point, so the
elevation contract is simply: the road's z at ``s = length`` must be the z of
its last source point, exactly as its z at ``s = 0`` is the z of its first.

It was not.  The height spline interpolated the source z values -- given at
*polyline* chord lengths -- at abscissae spanning the *fitted* reference
line's arc length.  The fit smooths corners away and is usually shorter, so
the final sample landed short of the last source point and the terminal z
became an interpolated value, while s=0 stayed exact because both domains
start at 0.  On the Nishi-Shinjuku fixture that put 652 of 1548 reference
lines over the 1e-6 m gate (max 3.3 mm) and produced all 26 of the map's
``road.geometry.contact_point`` findings.
"""

from typing import List, Tuple

import lanelet2
import numpy as np

from autoware_lanelet2_to_opendrive.opendrive.reference_line import ReferenceLine

#: The endpoint contract is exact arithmetic, not a fit, so it holds far
#: inside the checker's own 1e-6 m gate.
ELEVATION_TOLERANCE_M = 1e-9


def _curved_boundary_with_grade() -> np.ndarray:
    """A curved, climbing boundary whose fit is shorter than its polyline.

    The corners are what make the fitted spline shorter; the grade is what
    turns that length difference into an elevation error.
    """
    sweep = np.linspace(0.0, 1.1, 11)
    radius = 40.0
    return np.array(
        [
            [
                radius * np.sin(t),
                radius * (1.0 - np.cos(t)),
                3.0 + 1.7 * t + 0.4 * np.sin(3.0 * t),
            ]
            for t in sweep
        ]
    )


def _build_single_lanelet_map(
    left_xyz: np.ndarray, width: float = 3.5
) -> Tuple[lanelet2.core.LaneletMap, lanelet2.core.Lanelet]:
    """Build a one-lanelet map whose leftBound follows ``left_xyz``."""
    left_points = [
        lanelet2.core.Point3d(i + 1, float(x), float(y), float(z))
        for i, (x, y, z) in enumerate(left_xyz)
    ]

    right_xyz: List[List[float]] = []
    for i, (x, y, z) in enumerate(left_xyz):
        ahead = left_xyz[min(i + 1, len(left_xyz) - 1)][:2]
        behind = left_xyz[max(i - 1, 0)][:2]
        direction = np.asarray(ahead) - np.asarray(behind)
        direction = direction / (np.linalg.norm(direction) + 1e-12)
        right_xyz.append([x + width * direction[1], y - width * direction[0], z])
    right_points = [
        lanelet2.core.Point3d(1000 + i, float(x), float(y), float(z))
        for i, (x, y, z) in enumerate(right_xyz)
    ]

    lanelet = lanelet2.core.Lanelet(
        1,
        lanelet2.core.LineString3d(1, left_points),
        lanelet2.core.LineString3d(2, right_points),
    )
    lanelet_map = lanelet2.core.LaneletMap()
    lanelet_map.add(lanelet)
    return lanelet_map, lanelet


def _absolute_z(reference_line: ReferenceLine, s: float) -> float:
    return reference_line.height_spline.evaluate(s) + reference_line.elevation_offset


def test_reference_line_reproduces_source_elevation_at_both_endpoints():
    left_xyz = _curved_boundary_with_grade()
    lanelet_map, lanelet = _build_single_lanelet_map(left_xyz)

    reference_line = ReferenceLine.construct_from_lanelet_groups(
        lanelet_map, [lanelet], traffic_rule="RHT"
    )

    polyline_length = float(
        np.sum(np.linalg.norm(np.diff(left_xyz[:, :2], axis=0), axis=1))
    )
    fitted_length = reference_line.centerline_2d.total_length

    # Guard the premise: if the fit happened to have exactly the polyline's
    # length, the two parameterisations would coincide and the test would
    # pass without exercising anything.
    assert abs(fitted_length - polyline_length) > 1e-6, (
        "fixture is vacuous: fitted length "
        f"{fitted_length!r} equals polyline length {polyline_length!r}"
    )

    assert _absolute_z(reference_line, 0.0) == left_xyz[0][2]
    assert (
        abs(_absolute_z(reference_line, fitted_length) - left_xyz[-1][2])
        < ELEVATION_TOLERANCE_M
    )


def test_emitted_elevation_profile_ends_on_the_source_elevation():
    """The same contract through the emitted ``<elevation>`` records.

    This is what the checker actually reads: it evaluates the last record's
    cubic at ``s = road@length``.
    """
    left_xyz = _curved_boundary_with_grade()
    lanelet_map, lanelet = _build_single_lanelet_map(left_xyz)

    reference_line = ReferenceLine.construct_from_lanelet_groups(
        lanelet_map, [lanelet], traffic_rule="RHT"
    )
    length = reference_line.centerline_2d.total_length

    profile = reference_line.get_elevation_profile(
        geometry_s_values=[0.0, length / 3.0, 2.0 * length / 3.0]
    )
    last = profile.elevations[-1]
    ds = length - last.s
    z_end = last.a + last.b * ds + last.c * ds**2 + last.d * ds**3

    assert abs(z_end - left_xyz[-1][2]) < ELEVATION_TOLERANCE_M
    assert profile.elevations[0].a == left_xyz[0][2]
