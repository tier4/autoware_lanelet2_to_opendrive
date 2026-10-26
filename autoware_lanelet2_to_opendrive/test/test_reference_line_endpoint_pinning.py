"""Regression tests for exact reference-line endpoints.

A road's reference line endpoints are a *contract* with the neighbouring
roads, not merely two more points to fit.  The ASAM rule
``lane_smoothness.contact_point_no_horizontal_gaps`` reconstructs every
lane border from the reference line plus the cumulative lane widths and
allows the linked lanes' borders to differ by at most 1 cm; the companion
rule ``road.geometry.contact_point`` compares the reference-line endpoints
themselves.

The reference line is fitted as a clamped B-spline by weighted least
squares, in which the "hard" boundary rows carry weight 80 against the
data term's 20.  The solve therefore *trades away* part of the endpoint
constraint to fit the interior better, and on short, tightly curved
lanelets -- exactly the junction-mouth geometry the map is full of -- the
residual reaches 1-3 cm.  Measured on nishishinjuku before the fix, 21 of
207 directed plain road-to-road links had reference-line endpoints more
than 1 cm apart, and those joints accounted for 13 of the 343
``contact_point_no_horizontal_gaps`` errors.

Because the knot vector is clamped, the first and last control points
*are* the curve's endpoints, so the constraint can be imposed exactly by
elimination rather than by weight.  These tests pin that behaviour.
"""

from typing import List, Tuple

import lanelet2
import numpy as np

from autoware_lanelet2_to_opendrive.opendrive.reference_line import ReferenceLine
from autoware_lanelet2_to_opendrive.spline import Splines

# The ASAM tolerance the emitted map is judged against.
ASAM_CONTACT_TOLERANCE_M = 0.01

# What "exact" means for a float64 least-squares solve over local
# coordinates: the endpoint is a control point, so only round-off remains.
EXACT_TOLERANCE_M = 1e-9

# How much interior accuracy pinning the endpoints is allowed to cost,
# relative to the same fit without the constraint.  Measured on the whole
# nishishinjuku map the effect is +0.002 m on p95 and +0.029 m on the
# single worst road, with the median unchanged.
MAX_INTERIOR_PENALTY_M = 0.02


def _tight_arc_xy(radius: float = 5.0, sweep: float = 2.2, count: int = 7):
    """A short, tightly curved boundary, like a junction-mouth lanelet.

    With the default weight mix this shape leaves ~2 cm of endpoint error
    in the fitted reference line -- twice the ASAM tolerance.
    """
    theta = np.linspace(0.0, sweep, count)
    return np.column_stack([radius * np.sin(theta), radius * (1.0 - np.cos(theta))])


def _right_bound_for(left_xy: np.ndarray, width: float = 3.5) -> np.ndarray:
    """Offset ``left_xy`` to the right by ``width`` to form the other bound."""
    right: List[List[float]] = []
    for i, (x, y) in enumerate(left_xy):
        ahead = left_xy[min(i + 1, len(left_xy) - 1)]
        behind = left_xy[max(i - 1, 0)]
        direction = np.asarray(ahead) - np.asarray(behind)
        direction = direction / (np.linalg.norm(direction) + 1e-12)
        right.append([x + width * direction[1], y - width * direction[0]])
    return np.asarray(right)


def _build_lanelet(
    left_xy: np.ndarray, base_id: int, width: float = 3.5
) -> Tuple[lanelet2.core.Lanelet, np.ndarray]:
    """Build one lanelet whose leftBound follows ``left_xy``."""
    right_xy = _right_bound_for(left_xy, width)
    left_points = [
        lanelet2.core.Point3d(base_id + i, float(x), float(y), 0.0)
        for i, (x, y) in enumerate(left_xy)
    ]
    right_points = [
        lanelet2.core.Point3d(base_id + 500 + i, float(x), float(y), 0.0)
        for i, (x, y) in enumerate(right_xy)
    ]
    lanelet = lanelet2.core.Lanelet(
        base_id,
        lanelet2.core.LineString3d(base_id, left_points),
        lanelet2.core.LineString3d(base_id + 1, right_points),
    )
    return lanelet, right_xy


def _single_lanelet_map(left_xy: np.ndarray, base_id: int = 1):
    lanelet, right_xy = _build_lanelet(left_xy, base_id)
    lanelet_map = lanelet2.core.LaneletMap()
    lanelet_map.add(lanelet)
    return lanelet_map, lanelet, right_xy


def _reference_line_endpoints(reference_line: ReferenceLine):
    spline = reference_line.centerline_2d
    return (
        np.asarray(spline.evaluate(0.0)[:2]),
        np.asarray(spline.evaluate(spline.total_length)[:2]),
    )


def _point_to_polyline_distance(point: np.ndarray, polyline: np.ndarray) -> float:
    best = np.inf
    for start, end in zip(polyline[:-1], polyline[1:]):
        segment = end - start
        squared = float(np.dot(segment, segment))
        t = (
            0.0
            if squared == 0.0
            else float(np.clip(np.dot(point - start, segment) / squared, 0.0, 1.0))
        )
        best = min(best, float(np.linalg.norm(point - (start + t * segment))))
    return best


def test_exact_endpoints_reproduces_first_and_last_point():
    """``exact_endpoints=True`` lands on the input endpoints, not near them."""
    points = _tight_arc_xy()

    loose = Splines(points)
    pinned = Splines(points, exact_endpoints=True)

    def endpoint_error(spline: Splines) -> float:
        start = np.asarray(spline.evaluate(0.0)[:2])
        end = np.asarray(spline.evaluate(spline.total_length)[:2])
        return max(
            float(np.linalg.norm(start - points[0])),
            float(np.linalg.norm(end - points[-1])),
        )

    pinned_error = endpoint_error(pinned)
    assert (
        pinned_error < EXACT_TOLERANCE_M
    ), f"pinned fit still misses its endpoints by {pinned_error:.2e} m"
    # The weighted solve is what motivates the flag: it trades the
    # endpoint away against the interior data term.
    assert pinned_error < endpoint_error(loose)


def test_exact_endpoints_barely_costs_the_interior_fit():
    """Pinning the ends must not meaningfully distort the rest of the curve.

    Imposing the constraint by elimination -- rather than by overwriting
    the solved coefficients or by an enormous weight -- keeps the interior
    control points optimal *given* the constraint, so the extra deviation
    is a fraction of the endpoint error being removed.  (An absolute bound
    would be meaningless on this fixture: the 7-vertex chord polyline is
    itself ~0.17 m inside the arc it samples, and both fits follow the
    arc.)
    """
    points = _tight_arc_xy()

    def deviation(spline: Splines) -> float:
        samples = [
            np.asarray(spline.evaluate(s)[:2])
            for s in np.linspace(0.0, spline.total_length, 400)
        ]
        return max(_point_to_polyline_distance(p, points) for p in samples)

    loose_deviation = deviation(Splines(points))
    pinned_deviation = deviation(Splines(points, exact_endpoints=True))

    assert pinned_deviation < loose_deviation + MAX_INTERIOR_PENALTY_M, (
        f"pinning the endpoints cost {pinned_deviation - loose_deviation:.4f} m "
        f"of interior accuracy"
    )


def test_reference_line_reproduces_the_source_boundary_endpoints():
    """The emitted reference line starts and ends on the anchor boundary.

    Without endpoint pinning this fit lands ~2 cm away -- twice the ASAM
    contact-point tolerance -- so consecutive roads cannot meet.
    """
    left_xy = _tight_arc_xy()
    lanelet_map, lanelet, _ = _single_lanelet_map(left_xy)

    reference_line = ReferenceLine.construct_from_lanelet_groups(
        lanelet_map, [lanelet], traffic_rule="RHT"
    )
    start, end = _reference_line_endpoints(reference_line)

    assert float(np.linalg.norm(start - left_xy[0])) < EXACT_TOLERANCE_M
    assert float(np.linalg.norm(end - left_xy[-1])) < EXACT_TOLERANCE_M


def test_consecutive_reference_lines_meet_within_asam_tolerance():
    """Two lanelets sharing a boundary vertex yield reference lines that meet.

    This is the property ``lane_smoothness.contact_point_no_horizontal_gaps``
    and ``road.geometry.contact_point`` actually test: road A's reference
    line must end where road B's begins.
    """
    first_xy = _tight_arc_xy()
    # Second lanelet continues from the first one's last vertex, curving
    # the other way so its own fit is equally hard.
    theta = np.linspace(0.0, 2.0, 7)
    tangent = first_xy[-1] - first_xy[-2]
    angle = float(np.arctan2(tangent[1], tangent[0]))
    radius = 4.0
    local = np.column_stack([radius * np.sin(theta), -radius * (1.0 - np.cos(theta))])
    rotation = np.array(
        [
            [np.cos(angle), -np.sin(angle)],
            [np.sin(angle), np.cos(angle)],
        ]
    )
    second_xy = local @ rotation.T + first_xy[-1]
    assert np.allclose(second_xy[0], first_xy[-1])

    lanelet_map = lanelet2.core.LaneletMap()
    first, _ = _build_lanelet(first_xy, base_id=1)
    second, _ = _build_lanelet(second_xy, base_id=101)
    lanelet_map.add(first)
    lanelet_map.add(second)

    first_line = ReferenceLine.construct_from_lanelet_groups(
        lanelet_map, [first], traffic_rule="RHT"
    )
    second_line = ReferenceLine.construct_from_lanelet_groups(
        lanelet_map, [second], traffic_rule="RHT"
    )

    _, first_end = _reference_line_endpoints(first_line)
    second_start, _ = _reference_line_endpoints(second_line)

    gap = float(np.linalg.norm(first_end - second_start))
    assert gap < ASAM_CONTACT_TOLERANCE_M, (
        f"consecutive reference lines are {gap * 1000:.1f} mm apart, over the "
        f"{ASAM_CONTACT_TOLERANCE_M * 1000:.0f} mm ASAM contact-point tolerance"
    )
