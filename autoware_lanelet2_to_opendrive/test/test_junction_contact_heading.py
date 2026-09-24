"""Regression tests for the *direction* half of the junction contact contract.

``test_reference_line_endpoint_pinning`` pins the position half: a road's
reference line starts and ends exactly where its neighbour's does.  That is
necessary but not sufficient, because OpenDRIVE does not place a lane border
at a point -- it places it at ``reference(s) + t * normal(s)``.  Two roads
that share a contact point but leave it along different tangents therefore
have lane borders that fan out from that point.  With the inner borders
coincident and the widths equal, the far borders of a lane of width ``w``
miss each other by exactly::

    |outer_a - outer_b| = 2 * w * sin(dpsi / 2)

where ``dpsi`` is the angle between the two reference lines' headings.

Measured on nishishinjuku at 31e81c3f, before this fix, that identity
accounted for all 59 ``lane_smoothness.contact_point_no_horizontal_gaps``
errors that base reported: the inner borders already agreed to 1e-13 m in
48 of them and the lane widths agreed to 6 mm in all 59, while ``dpsi``
had a median of 1.92 deg.  Clearing the 1 cm ASAM tolerance at a 3 m lane
needs ``dpsi < 0.19 deg``.  The error *counts* move as other changes land
-- they are named here only to show which defect the identity explains --
but the identity itself is a property of the representation, not of a
particular base.

These tests pin the behaviour that closes it: the junction phase hands the
connecting road the linked road's endpoint *heading* alongside its endpoint
*position*, and the spline fit honours it.
"""

import math
from typing import List, Tuple

import lanelet2
import numpy as np
import pytest

from autoware_lanelet2_to_opendrive.opendrive.reference_line import ReferenceLine

# The ASAM tolerance for lane borders of two connected drivable lanes.
ASAM_CONTACT_TOLERANCE_M = 0.01

# The heading agreement that tolerance implies for a nominal 3 m lane:
# 2 * 3.0 * sin(dpsi / 2) < 0.01  =>  dpsi < 0.191 deg.
ASAM_IMPLIED_HEADING_TOLERANCE_RAD = math.radians(0.19)

# What the fit should actually achieve once the constraint is stated.  The
# boundary rows are weighted, not eliminated, so this is not machine
# precision -- but it is an order of magnitude inside the tolerance above.
ACHIEVED_HEADING_TOLERANCE_RAD = math.radians(0.02)

# The connecting road's own boundary leaves the shared point at this angle.
# Roughly the 95th percentile of the measured nishishinjuku mismatch, and
# far enough outside the tolerance that an unpinned fit cannot pass by luck.
CONNECTOR_SKEW_RAD = math.radians(5.0)

LANE_WIDTH_M = 3.5

# The shared contact point, and the incoming road's tangent there.
CONTACT_XY = (40.0, 0.0)
INCOMING_HEADING_RAD = 0.0


def _right_bound_for(left_xy: np.ndarray, width: float = LANE_WIDTH_M) -> np.ndarray:
    """Offset ``left_xy`` to the right by ``width`` to form the other bound."""
    right: List[List[float]] = []
    for i, (x, y) in enumerate(left_xy):
        ahead = left_xy[min(i + 1, len(left_xy) - 1)]
        behind = left_xy[max(i - 1, 0)]
        direction = np.asarray(ahead) - np.asarray(behind)
        direction = direction / (np.linalg.norm(direction) + 1e-12)
        right.append([x + width * direction[1], y - width * direction[0]])
    return np.asarray(right)


def _build_lanelet(left_xy: np.ndarray, base_id: int) -> lanelet2.core.Lanelet:
    """Build one lanelet whose leftBound -- the RHT reference line -- is ``left_xy``."""
    right_xy = _right_bound_for(left_xy)
    left_points = [
        lanelet2.core.Point3d(base_id + i, float(x), float(y), 0.0)
        for i, (x, y) in enumerate(left_xy)
    ]
    right_points = [
        lanelet2.core.Point3d(base_id + 500 + i, float(x), float(y), 0.0)
        for i, (x, y) in enumerate(right_xy)
    ]
    return lanelet2.core.Lanelet(
        base_id,
        lanelet2.core.LineString3d(base_id, left_points),
        lanelet2.core.LineString3d(base_id + 1, right_points),
    )


def _straight(start: Tuple[float, float], angle: float, length: float, count: int = 9):
    """A straight boundary polyline leaving ``start`` at ``angle``."""
    steps = np.linspace(0.0, length, count)
    return np.column_stack(
        [start[0] + steps * math.cos(angle), start[1] + steps * math.sin(angle)]
    )


def _connector_boundary():
    """A connector boundary that starts at the incoming road's endpoint.

    It leaves that point ``CONNECTOR_SKEW_RAD`` off the incoming road's own
    tangent, then curves away -- the junction-mouth case where the two
    source LineStrings genuinely disagree about the direction of travel at
    the point they share.
    """
    theta = np.linspace(0.0, 0.9, 9)
    radius = 12.0
    local = np.column_stack([radius * np.sin(theta), radius * (1.0 - np.cos(theta))])
    rotation = np.array(
        [
            [math.cos(CONNECTOR_SKEW_RAD), -math.sin(CONNECTOR_SKEW_RAD)],
            [math.sin(CONNECTOR_SKEW_RAD), math.cos(CONNECTOR_SKEW_RAD)],
        ]
    )
    return local @ rotation.T + np.asarray(CONTACT_XY)


def _fit(lanelet_map, lanelet, **overrides) -> ReferenceLine:
    return ReferenceLine.construct_from_lanelet_groups(
        lanelet_map, [lanelet], traffic_rule="RHT", **overrides
    )


def _heading_at(reference_line: ReferenceLine, s: float) -> float:
    tangent = reference_line.centerline_2d.evaluate(s, derivative=1)
    return float(np.arctan2(tangent[1], tangent[0]))


def _start_heading(reference_line: ReferenceLine) -> float:
    return _heading_at(reference_line, 0.0)


def _end_heading(reference_line: ReferenceLine) -> float:
    return _heading_at(reference_line, reference_line.centerline_2d.total_length)


def _angle_between(a: float, b: float) -> float:
    return abs((b - a + math.pi) % (2 * math.pi) - math.pi)


def _outer_border_point(
    contact_xy: Tuple[float, float], heading: float, width: float
) -> np.ndarray:
    """Where OpenDRIVE puts a lane border ``width`` from the reference line.

    ``reference(s) + t * normal(s)`` with ``normal = (-sin h, cos h)``.  This
    is the construction both the ASAM checker and every consumer apply, so
    the distance between two roads' versions of it *is* the reported gap.
    """
    return np.asarray(
        [
            contact_xy[0] - width * math.sin(heading),
            contact_xy[1] + width * math.cos(heading),
        ]
    )


@pytest.fixture(scope="module")
def connector_map():
    """An incoming lanelet and a connector that share one boundary vertex."""
    incoming = _build_lanelet(
        _straight((0.0, 0.0), INCOMING_HEADING_RAD, CONTACT_XY[0]), base_id=1
    )
    connector = _build_lanelet(_connector_boundary(), base_id=1001)
    lanelet_map = lanelet2.core.LaneletMap()
    lanelet_map.add(incoming)
    lanelet_map.add(connector)
    return lanelet_map, incoming, connector


def test_the_fixture_actually_reproduces_the_defect(connector_map):
    """Without a heading override the connector leaves at its own angle.

    Guards the two tests below: if the fixture ever stopped disagreeing
    about the tangent they would pass vacuously.

    **This test passes with or without the heading override, by design.**
    It asserts a property of the fixture, not of the fix, so it is not
    evidence that the fix works and it is not dead weight either -- it is
    what stops the tests that *are* evidence from passing for the wrong
    reason.  Do not delete it on the grounds that it never fails.
    """
    lanelet_map, _, connector = connector_map

    unpinned = _fit(
        lanelet_map,
        connector,
        start_xyz_override=(CONTACT_XY[0], CONTACT_XY[1], 0.0),
    )
    skew = _angle_between(INCOMING_HEADING_RAD, _start_heading(unpinned))

    assert skew > ASAM_IMPLIED_HEADING_TOLERANCE_RAD, (
        "fixture no longer reproduces the defect: the connector's own start "
        f"tangent is only {math.degrees(skew):.4f} deg off the incoming road's"
    )


def test_start_heading_override_pins_the_reference_line_tangent(connector_map):
    """``start_hdg_override`` makes the fit leave along the linked road's tangent."""
    lanelet_map, _, connector = connector_map

    pinned = _fit(
        lanelet_map,
        connector,
        start_xyz_override=(CONTACT_XY[0], CONTACT_XY[1], 0.0),
        start_hdg_override=INCOMING_HEADING_RAD,
    )

    error = _angle_between(INCOMING_HEADING_RAD, _start_heading(pinned))
    assert error < ACHIEVED_HEADING_TOLERANCE_RAD, (
        f"pinned reference line still leaves {math.degrees(error):.4f} deg off "
        "the heading it was given"
    )


def test_end_heading_override_pins_the_far_end_too(connector_map):
    """The s=length override is wired up the same way as the s=0 one."""
    lanelet_map, _, connector = connector_map

    boundary = _connector_boundary()
    target = math.radians(30.0)

    pinned = _fit(
        lanelet_map,
        connector,
        end_xyz_override=(float(boundary[-1][0]), float(boundary[-1][1]), 0.0),
        end_hdg_override=target,
    )

    error = _angle_between(target, _end_heading(pinned))
    assert error < ACHIEVED_HEADING_TOLERANCE_RAD, (
        f"pinned reference line still ends {math.degrees(error):.4f} deg off "
        "the heading it was given"
    )


def test_lane_borders_meet_only_once_the_heading_is_pinned(connector_map):
    """The ASAM quantity: the far border of the shared lane must coincide.

    Both fits below start at exactly the same point, so the *inner* border
    (the reference line itself, ``t = 0``) always matches.  What the ASAM
    rule additionally requires is that the border at ``t = -w`` matches,
    and that one is placed along each road's own normal.
    """
    lanelet_map, _, connector = connector_map

    incoming_border = _outer_border_point(
        CONTACT_XY, INCOMING_HEADING_RAD, LANE_WIDTH_M
    )

    unpinned = _fit(
        lanelet_map,
        connector,
        start_xyz_override=(CONTACT_XY[0], CONTACT_XY[1], 0.0),
    )
    pinned = _fit(
        lanelet_map,
        connector,
        start_xyz_override=(CONTACT_XY[0], CONTACT_XY[1], 0.0),
        start_hdg_override=INCOMING_HEADING_RAD,
    )

    gap_unpinned = float(
        np.linalg.norm(
            incoming_border
            - _outer_border_point(CONTACT_XY, _start_heading(unpinned), LANE_WIDTH_M)
        )
    )
    gap_pinned = float(
        np.linalg.norm(
            incoming_border
            - _outer_border_point(CONTACT_XY, _start_heading(pinned), LANE_WIDTH_M)
        )
    )

    # Position pinning alone leaves a gap the checker reports...
    assert gap_unpinned > ASAM_CONTACT_TOLERANCE_M, (
        f"position-only pinning already closes the gap ({gap_unpinned:.4f} m); "
        "the fixture no longer exercises the rule"
    )
    # ...and the heading override closes it.
    assert gap_pinned < ASAM_CONTACT_TOLERANCE_M, (
        f"lane borders are {gap_pinned * 1000:.1f} mm apart, over the "
        f"{ASAM_CONTACT_TOLERANCE_M * 1000:.0f} mm ASAM tolerance"
    )


def test_gap_matches_the_closed_form(connector_map):
    """``2 * w * sin(dpsi / 2)`` is the whole of the gap, not an approximation.

    This is the identity the diagnosis rests on; if it ever stops holding,
    the heading override is no longer the right lever and the tests above
    would keep passing while the real map regressed.

    **Passes with or without the heading override, by design** -- like
    ``test_the_fixture_actually_reproduces_the_defect`` above, it asserts a
    property of the representation rather than of the fix.  Those two are
    the reason this module reports 4 failed / 2 passed against a tree
    without the change, rather than 6 failed.
    """
    lanelet_map, _, connector = connector_map

    unpinned = _fit(
        lanelet_map,
        connector,
        start_xyz_override=(CONTACT_XY[0], CONTACT_XY[1], 0.0),
    )
    dpsi = _angle_between(INCOMING_HEADING_RAD, _start_heading(unpinned))

    measured = float(
        np.linalg.norm(
            _outer_border_point(CONTACT_XY, INCOMING_HEADING_RAD, LANE_WIDTH_M)
            - _outer_border_point(CONTACT_XY, _start_heading(unpinned), LANE_WIDTH_M)
        )
    )
    closed_form = 2.0 * LANE_WIDTH_M * math.sin(dpsi / 2.0)

    assert measured == pytest.approx(closed_form, abs=1e-12)


# --------------------------------------------------------------------------
# End-to-end: the unit tests above prove the constraint works, but not that
# the junction phase actually hands it over.  This one fails if the plumbing
# from ``Road.evaluate_lane_anchor_pose`` through
# ``construct_connecting_roads_from_junctions`` is dropped.
# --------------------------------------------------------------------------


def _is_synthetic_stub(road_elem) -> bool:
    """True for a connector emitted by ``_make_zero_length_connecting_road``.

    Those connectors bridge two anchors that are meant to coincide and take
    their heading from the vector between them, which at that scale is
    numerical noise.  They are a separate defect on a separate code path,
    and the reference-line pinning this module tests never reaches them.

    They are recognised by what they *are* in the emitted file -- a single
    ``paramPoly3`` segment with ``v(p) = 0``, i.e. a straight line -- rather
    than by length or by junction id.

    This is deliberate and a length threshold was tried first.  Scoping by
    "shorter than 0.1 m" looks equivalent and is not: the synthetic
    connectors that bridge a lane-count change run to 0.45 m and 0.90 m, so
    two of them walked through the filter and failed the test for a reason
    the change under test cannot address.  Junction id would work today
    (the synthetic ones start at ``junction_id_offset + 10_000``) but
    encodes a configurable offset into a test.

    The signature separates cleanly on nishishinjuku: 160 connecting roads
    match it, every known synthetic stub among them, while genuinely fitted
    connectors carry 2 to 32 geometry records.  A fitted connector that
    happened to be perfectly straight would also be excluded, which only
    narrows the scope and cannot mask a failure.
    """
    plan_view = road_elem.find("planView")
    if plan_view is None:
        return False
    geometries = plan_view.findall("geometry")
    if len(geometries) != 1:
        return False
    pp3 = geometries[0].find("paramPoly3")
    if pp3 is None:
        return False
    return all(abs(float(pp3.get(k, "0.0"))) < 1e-12 for k in ("aV", "bV", "cV", "dV"))


def _road_heading_at_end(road_elem, at_start: bool) -> float:
    """World-frame reference-line tangent angle at a road's s=0 / s=length."""
    from autoware_lanelet2_to_opendrive.opendrive.geometry import element_end_param

    geometries = road_elem.find("planView").findall("geometry")
    geom = geometries[0] if at_start else geometries[-1]
    base = float(geom.get("hdg"))
    pp3 = geom.find("paramPoly3")
    if pp3 is None:
        return base
    p = 0.0 if at_start else element_end_param(geom)
    du = (
        float(pp3.get("bU", "0.0"))
        + 2.0 * float(pp3.get("cU", "0.0")) * p
        + 3.0 * float(pp3.get("dU", "0.0")) * p**2
    )
    dv = (
        float(pp3.get("bV", "0.0"))
        + 2.0 * float(pp3.get("cV", "0.0")) * p
        + 3.0 * float(pp3.get("dV", "0.0")) * p**2
    )
    return base + math.atan2(dv, du)


def test_pinned_junction_contacts_agree_on_heading():
    """Every pinned connecting-road contact leaves along the linked tangent.

    Scope: junction ``<connection>`` rows whose connecting road is a
    fitted reference line rather than a synthetic straight stub, and whose
    endpoint the position override actually reached -- recognised by the endpoint already lying
    on the linked road's lane edge, which is what the position half of the
    contract guarantees.  Where the position was not pinned there is
    nothing for the heading to be pinned to.

    Without the heading override this fails on nishishinjuku; the offender
    count depends on the base, so it is deliberately not quoted here.  The
    assertion does not depend on it either, because the scope is set by a
    measured precondition rather than by a fixed list of roads.
    """
    import lxml.etree as ET

    from test_junction_endpoint_fidelity import (  # noqa: E402
        _build_nishishinjuku_xodr,
        _distance3,
        _evaluate_lane_inner_edge,
    )
    from autoware_lanelet2_to_opendrive.opendrive.geometry import (
        evaluate_road_endpoints,
    )

    root = ET.parse(str(_build_nishishinjuku_xodr())).getroot()
    endpoints = evaluate_road_endpoints(root)

    road_by_id = {int(r.get("id")): r for r in root.findall("road")}
    junction_of = {rid: int(r.get("junction", "-1")) for rid, r in road_by_id.items()}

    # A position pin is only claimed when the endpoint sits this close to
    # the linked lane edge; the fit achieves sub-millimetre there.
    PINNED_POSITION_M = 0.001

    offenders = []
    checked = 0
    for junction_elem in root.findall("junction"):
        for conn in junction_elem.findall("connection"):
            cr_id = int(conn.get("connectingRoad"))
            inc_id = int(conn.get("incomingRoad"))
            contact = conn.get("contactPoint", "start")
            if cr_id not in road_by_id or inc_id not in road_by_id:
                continue
            if junction_of.get(inc_id, -1) != -1:
                continue
            cr = road_by_id[cr_id]
            if _is_synthetic_stub(cr):
                continue
            from_lane = next(
                (
                    int(link.get("from"))
                    for link in conn.findall("laneLink")
                    if abs(int(link.get("to"))) == 1
                ),
                None,
            )
            if from_lane is None or cr_id not in endpoints:
                continue

            at_start = contact == "start"
            inc_at_start = not at_start
            expected = _evaluate_lane_inner_edge(
                road_by_id[inc_id], from_lane, at_start=inc_at_start
            )
            if expected is None:
                continue
            conn_start, conn_end = endpoints[cr_id]
            actual = conn_start if at_start else conn_end
            if _distance3(expected, actual) > PINNED_POSITION_M:
                continue  # not pinned here; nothing to align the heading to

            checked += 1
            dpsi = _angle_between(
                _road_heading_at_end(road_by_id[inc_id], at_start=inc_at_start),
                _road_heading_at_end(cr, at_start=at_start),
            )
            if dpsi > ASAM_IMPLIED_HEADING_TOLERANCE_RAD:
                offenders.append((cr_id, inc_id, contact, math.degrees(dpsi)))

    assert checked > 0, "no pinned junction contacts found; the scope is wrong"

    if offenders:
        offenders.sort(key=lambda o: -o[3])
        sample = "\n".join(
            f"  conn_road={c} incoming={i} contact={p} dpsi={d:.4f} deg"
            for c, i, p, d in offenders[:10]
        )
        pytest.fail(
            f"{len(offenders)} of {checked} pinned junction contacts disagree on "
            f"heading by more than {math.degrees(ASAM_IMPLIED_HEADING_TOLERANCE_RAD):.3f} "
            f"deg.\nWorst 10:\n{sample}"
        )
