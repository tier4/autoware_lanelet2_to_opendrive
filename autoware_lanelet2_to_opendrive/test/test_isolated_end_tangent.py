"""The end tangent of a road nobody touches must follow its own polyline.

``_calculate_centerline_velocity_vector`` sets a lanelet's endpoint tangent to
the perpendicular of the cut joining its left and right boundary ends, and the
reference-line fit honours that target exactly.  Where the cut is skewed the
emitted tangent is off the boundary's own direction by the skew -- up to
85.113 deg on the nishishinjuku walkways.

That rule is not a defect in general: the cut is *shared* between
longitudinally connected lanelets, so its perpendicular is a quantity both
sides compute identically, and replacing it with each road's own boundary
tangent makes 877 of 889 neighbour contacts worse (measured; see
``project_end_tangent_is_the_end_cut_perpendicular``).  Continuity is what the
skew buys.

But an end that *no other lanelet shares* has no continuity to buy.  There the
skew is paid for nothing: the fitter is asked to leave a straight polyline at
up to 86 deg off its own direction, which can only bow the curve away from the
geometry it is meant to reproduce.  Measured on nishishinjuku
(``.claude/state/scratch/non_road_reference_roads_contacts.py``), 16 of the
600 reference-lanelet ends share no point with any other lanelet, all 16 on
the 8 walkway-only roads, with cut skews up to 85.113 deg and 10 of them past
the 20.05 deg heading guard.  The 6 road_shoulder ends, which do have
contacts, are all within 0.936 deg and must keep the shared rule.
"""

import math
from typing import List, Tuple

import lanelet2
import numpy as np
import pytest

from autoware_lanelet2_to_opendrive.opendrive.reference_line import ReferenceLine

# The fit honours its tangent target to p99 0.022 deg, so anything above a
# degree means the target itself was the skewed one.
TANGENT_TOLERANCE_DEG = 1.0

# The cut skew built into the fixtures below.  Chosen past the 20.05 deg
# heading guard so the figure cannot be confused with fitting noise.
SKEW_DEG = 40.0

LANE_WIDTH_M = 3.5


def _skewed_lanelet(
    left_xy: np.ndarray,
    *,
    skew_at_start_deg: float,
    skew_at_end_deg: float,
    lanelet_id: int,
    point_id_base: int,
) -> Tuple[lanelet2.core.Lanelet, List[lanelet2.core.Point3d]]:
    """A lanelet whose end cuts are off square by the given angles.

    The left boundary follows ``left_xy`` verbatim; each right-boundary point
    is placed so that the cut from left to right is rotated by the requested
    skew away from the boundary normal.  Interior points use no skew, so the
    only thing the fixture perturbs is the cut at each end.
    """
    left_points = [
        lanelet2.core.Point3d(point_id_base + i, float(x), float(y), 0.0)
        for i, (x, y) in enumerate(left_xy)
    ]

    right_xy: List[List[float]] = []
    last = len(left_xy) - 1
    for i, (x, y) in enumerate(left_xy):
        ahead = left_xy[min(i + 1, last)]
        behind = left_xy[max(i - 1, 0)]
        direction = np.asarray(ahead, dtype=float) - np.asarray(behind, dtype=float)
        direction = direction / (np.linalg.norm(direction) + 1e-12)
        # The unskewed cut points to the right of travel.
        normal = np.array([direction[1], -direction[0]])
        if i == 0:
            skew = math.radians(skew_at_start_deg)
        elif i == last:
            skew = math.radians(skew_at_end_deg)
        else:
            skew = 0.0
        rotation = np.array(
            [[math.cos(skew), -math.sin(skew)], [math.sin(skew), math.cos(skew)]]
        )
        cut = rotation @ normal
        right_xy.append([x + LANE_WIDTH_M * cut[0], y + LANE_WIDTH_M * cut[1]])

    right_points = [
        lanelet2.core.Point3d(point_id_base + 500 + i, float(x), float(y), 0.0)
        for i, (x, y) in enumerate(right_xy)
    ]

    lanelet = lanelet2.core.Lanelet(
        lanelet_id,
        lanelet2.core.LineString3d(point_id_base + 900, left_points),
        lanelet2.core.LineString3d(point_id_base + 901, right_points),
    )
    return lanelet, left_points


def _straight_xy(x_start: float, x_end: float, count: int = 3) -> np.ndarray:
    return np.array([[x, 0.0] for x in np.linspace(x_start, x_end, count)])


def _tangent_deg(reference_line: ReferenceLine, at_start: bool) -> float:
    spline = reference_line.centerline_2d
    s = 0.0 if at_start else spline.total_length
    derivative = spline.evaluate(s, derivative=1)[:2]
    return math.degrees(math.atan2(derivative[1], derivative[0]))


@pytest.mark.xfail(
    strict=True,
    reason="the cut perpendicular is applied even where no neighbour shares the cut",
)
def test_isolated_end_tangent_follows_the_boundary() -> None:
    """A skewed cut on an end nobody shares must not steer the fit."""
    left_xy = _straight_xy(0.0, 20.0)
    lanelet, _ = _skewed_lanelet(
        left_xy,
        skew_at_start_deg=SKEW_DEG,
        skew_at_end_deg=0.0,
        lanelet_id=1,
        point_id_base=1,
    )
    lanelet_map = lanelet2.core.LaneletMap()
    lanelet_map.add(lanelet)

    reference_line = ReferenceLine.construct_from_lanelet_groups(
        lanelet_map, [lanelet], traffic_rule="RHT"
    )

    # The boundary is straight along +x, so its own tangent is 0 deg at both
    # ends.  The square end pins the measurement: if it drifted, the fixture
    # or the fit would be at fault rather than the tangent rule.
    assert abs(_tangent_deg(reference_line, at_start=False)) < TANGENT_TOLERANCE_DEG

    start_deg = _tangent_deg(reference_line, at_start=True)
    assert abs(start_deg) < TANGENT_TOLERANCE_DEG, (
        f"isolated end left along {start_deg:.3f} deg instead of the "
        f"boundary's own 0 deg; the {SKEW_DEG:.1f} deg cut skew steered a fit "
        "that has no neighbour to stay continuous with"
    )


def test_shared_end_keeps_the_cut_perpendicular() -> None:
    """An end a successor shares must keep the shared rule, skew and all.

    This is the scope pin for the fix above: neighbour continuity is bought by
    both sides deriving the tangent from the one cut they share, so an end
    with a contact must go on using the cut perpendicular even though that is
    the skewed value.
    """
    upstream_xy = _straight_xy(0.0, 20.0)
    upstream, upstream_left = _skewed_lanelet(
        upstream_xy,
        skew_at_start_deg=0.0,
        skew_at_end_deg=SKEW_DEG,
        lanelet_id=1,
        point_id_base=1,
    )
    # The successor starts from the upstream lanelet's own end points, which
    # is how Lanelet2 states connectivity.
    downstream_left = [
        upstream.leftBound[-1],
        lanelet2.core.Point3d(3001, 30.0, 0.0, 0.0),
        lanelet2.core.Point3d(3002, 40.0, 0.0, 0.0),
    ]
    downstream_right = [
        upstream.rightBound[-1],
        lanelet2.core.Point3d(3101, 30.0, -LANE_WIDTH_M, 0.0),
        lanelet2.core.Point3d(3102, 40.0, -LANE_WIDTH_M, 0.0),
    ]
    downstream = lanelet2.core.Lanelet(
        2,
        lanelet2.core.LineString3d(3900, downstream_left),
        lanelet2.core.LineString3d(3901, downstream_right),
    )

    lanelet_map = lanelet2.core.LaneletMap()
    lanelet_map.add(upstream)
    lanelet_map.add(downstream)

    reference_line = ReferenceLine.construct_from_lanelet_groups(
        lanelet_map, [upstream], traffic_rule="RHT"
    )

    end_deg = _tangent_deg(reference_line, at_start=False)
    assert abs(end_deg - SKEW_DEG) < TANGENT_TOLERANCE_DEG, (
        f"shared end left along {end_deg:.3f} deg; it must keep the cut "
        f"perpendicular at {SKEW_DEG:.1f} deg so the successor, which derives "
        "its own start tangent from the same cut, stays continuous"
    )
    # The unshared start of the same lanelet is square here, so it must be
    # unaffected either way.
    assert abs(_tangent_deg(reference_line, at_start=True)) < TANGENT_TOLERANCE_DEG
    assert len(upstream_left) == 3
