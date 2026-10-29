"""The synthesised connector stubs must agree with the roads they join.

A divergence, a merge, and a lanelet that connects directly to another
lanelet all produce a *stub*: a single-``paramPoly3`` connecting road that
exists only so every lane of a junction-linked road has a row in the
``<connection>`` table. OpenDRIVE 1.4 has no way to omit it — a
``<connection>`` must name a ``connectingRoad`` — and the 1.7 alternative
(``<junction type="direct">`` with ``linkedRoad``) is silently dropped by
CARLA's parser, so these stubs are load-bearing rather than cosmetic.

Being load-bearing, they have to describe the transition they stand for:

* they must point the way the traffic goes, so a vehicle placed on one does
  not receive a heading across its own carriageway, and
* they must be as wide as the lanes they connect, so the lane boundary does
  not step at the junction.

Both are measured against the roads named by the stub's own links, so the
tests need no knowledge of the source map.
"""

import math

import lxml.etree as ET
import pytest


# A stub's heading is set from one of its two linked roads' reference-line
# tangents, computed by the same function the roads themselves use, so
# agreement is exact up to floating point. 1 degree is three orders of
# magnitude above that and three orders below the defect it replaces
# (a median of 91 degrees).
HEADING_TOLERANCE_RAD = math.radians(1.0)

# The stub's constant width is taken from the lanes it joins, evaluated at
# the same ``s`` the emitted file records, so this too should be near-exact.
WIDTH_TOLERANCE_M = 0.05


def _lane_width_at(road_elem, lane_id: int, at_start: bool):
    """Width of lane ``lane_id`` of ``road_elem`` at its ``s=0`` / ``s=length``.

    Returns ``None`` when the road has no lane section, no such lane, or the
    lane carries no ``<width>`` element — never a substituted zero, which
    would let an assertion pass for the wrong reason.
    """
    lanes_elem = road_elem.find("lanes")
    if lanes_elem is None:
        return None
    sections = lanes_elem.findall("laneSection")
    if not sections:
        return None
    section = sections[0]
    side = section.find("left" if lane_id > 0 else "right")
    if side is None:
        return None
    lane = next(
        (le for le in side.findall("lane") if int(le.get("id")) == lane_id), None
    )
    if lane is None:
        return None
    width_elems = lane.findall("width")
    if not width_elems:
        return None

    if at_start:
        s_in_section = 0.0
    else:
        s_in_section = float(road_elem.get("length", "0.0")) - float(
            section.get("s", "0.0")
        )

    seg = width_elems[0]
    for we in width_elems:
        if float(we.get("sOffset", "0")) <= s_in_section:
            seg = we
        else:
            break
    ds = s_in_section - float(seg.get("sOffset", "0"))
    return (
        float(seg.get("a", "0"))
        + float(seg.get("b", "0")) * ds
        + float(seg.get("c", "0")) * ds**2
        + float(seg.get("d", "0")) * ds**3
    )


def _linked_contacts(stub_elem):
    """``[(linked road id, at_start)]`` for the stub's road-level links.

    ``at_start`` is where on the *linked* road the contact sits, which is
    what its ``contactPoint`` attribute names.
    """
    link = stub_elem.find("link")
    if link is None:
        return []
    contacts = []
    for kind in ("predecessor", "successor"):
        elem = link.find(kind)
        if elem is None or elem.get("elementType") != "road":
            continue
        contacts.append(
            (int(elem.get("elementId")), elem.get("contactPoint") == "start")
        )
    return contacts


def _stub_lane(stub_elem):
    """``(lane element, lane id)`` of a stub's single lane, or ``(None, None)``."""
    lanes_elem = stub_elem.find("lanes")
    if lanes_elem is None:
        return None, None
    sections = lanes_elem.findall("laneSection")
    if not sections:
        return None, None
    for side_name in ("left", "right"):
        side = sections[0].find(side_name)
        if side is None:
            continue
        for lane in side.findall("lane"):
            lane_id = int(lane.get("id"))
            if lane_id != 0:
                return lane, lane_id
    return None, None


def _synthetic_stubs(root):
    """``{road id: element}`` for every single-geometry connector stub."""
    from test_junction_contact_heading import _is_synthetic_stub  # noqa: E402

    stubs = {}
    for road in root.findall("road"):
        if int(road.get("junction", "-1")) == -1:
            continue
        if _is_synthetic_stub(road):
            stubs[int(road.get("id"))] = road
    return stubs


def test_synthetic_connectors_follow_a_linked_road_tangent(nishishinjuku_xodr):
    """Every stub leaves along the tangent of one of the roads it links.

    A stub stands for a transition between two roads that, in the source
    map, share their boundary points: Lanelet2 *defines* lane connectivity
    by shared ``Point`` objects, so a successor starts on the very points
    its predecessor ends on. The direction of such a transition is the road
    tangent; it cannot be recovered from the separation between the two
    renderings of that shared point, which is reconstruction error.

    The stub is required to agree with *one* of its two linked roads rather
    than both, because a merge joins two roads that genuinely differ in
    heading: the synthesiser aligns the stub with the road the junction
    hangs off, which is the predecessor for a divergence and the successor
    for a merge.
    """
    from test_junction_contact_heading import (  # noqa: E402
        _angle_between,
        _road_heading_at_end,
    )

    root = ET.parse(str(nishishinjuku_xodr)).getroot()
    road_by_id = {int(r.get("id")): r for r in root.findall("road")}
    stubs = _synthetic_stubs(root)

    assert stubs, "no synthetic connector stubs in the output; the scope is wrong"

    offenders = []
    checked = 0
    for stub_id, stub in sorted(stubs.items()):
        stub_heading = _road_heading_at_end(stub, at_start=True)

        best = None
        for linked_id, at_start in _linked_contacts(stub):
            linked = road_by_id.get(linked_id)
            if linked is None or linked.find("planView") is None:
                continue
            dpsi = _angle_between(
                stub_heading, _road_heading_at_end(linked, at_start=at_start)
            )
            if best is None or dpsi < best[0]:
                best = (dpsi, linked_id)
        if best is None:
            continue

        checked += 1
        if best[0] > HEADING_TOLERANCE_RAD:
            offenders.append((stub_id, best[1], math.degrees(best[0])))

    assert checked > 0, "no stub had a road-typed link to compare against"

    if offenders:
        offenders.sort(key=lambda o: -o[2])
        sample = "\n".join(
            f"  stub={s} closest_linked_road={r} dpsi={d:.3f} deg"
            for s, r, d in offenders[:10]
        )
        pytest.fail(
            f"{len(offenders)} of {checked} synthetic connectors are more than "
            f"{math.degrees(HEADING_TOLERANCE_RAD):.1f} deg off both roads they "
            f"link.\nWorst: {offenders[0][2]:.3f} deg.\nWorst 10:\n{sample}"
        )


def test_synthetic_connectors_are_as_wide_as_the_lanes_they_join(nishishinjuku_xodr):
    """Every stub's lane is as wide as the lanes its own links name.

    The lane widths are read from the emitted file through the stub's
    lane-level ``<predecessor>`` / ``<successor>`` links, so the expected
    value is whatever the converter itself wrote for the neighbouring
    lanes. A constant default cannot satisfy this on a real map: the lanes
    these stubs join on Nishishinjuku run from 2.40 m to 4.75 m.
    """
    root = ET.parse(str(nishishinjuku_xodr)).getroot()
    road_by_id = {int(r.get("id")): r for r in root.findall("road")}
    stubs = _synthetic_stubs(root)

    assert stubs, "no synthetic connector stubs in the output; the scope is wrong"

    offenders = []
    checked = 0
    for stub_id, stub in sorted(stubs.items()):
        stub_lane, _stub_lane_id = _stub_lane(stub)
        if stub_lane is None:
            continue
        stub_widths = stub_lane.findall("width")
        if not stub_widths:
            continue
        stub_width = float(stub_widths[0].get("a"))

        lane_link = stub_lane.find("link")
        if lane_link is None:
            continue

        # Pair each road-level link with the lane id the stub's own
        # lane-level link names on that side.
        neighbour_widths = []
        link = stub.find("link")
        for kind in ("predecessor", "successor"):
            road_side = link.find(kind) if link is not None else None
            lane_side = lane_link.find(kind)
            if road_side is None or lane_side is None:
                continue
            if road_side.get("elementType") != "road":
                continue
            linked = road_by_id.get(int(road_side.get("elementId")))
            if linked is None:
                continue
            width = _lane_width_at(
                linked,
                int(lane_side.get("id")),
                at_start=road_side.get("contactPoint") == "start",
            )
            if width is not None:
                neighbour_widths.append(width)

        if not neighbour_widths:
            continue

        expected = sum(neighbour_widths) / len(neighbour_widths)
        checked += 1
        error = abs(stub_width - expected)
        if error > WIDTH_TOLERANCE_M:
            offenders.append((stub_id, stub_width, expected, error))

    assert checked > 0, "no stub's neighbour lane widths could be read"

    if offenders:
        offenders.sort(key=lambda o: -o[3])
        sample = "\n".join(
            f"  stub={s} width={w:.3f} m expected={e:.3f} m error={d:.3f} m"
            for s, w, e, d in offenders[:10]
        )
        pytest.fail(
            f"{len(offenders)} of {checked} synthetic connectors differ from the "
            f"lanes they join by more than {WIDTH_TOLERANCE_M} m.\n"
            f"Worst: {offenders[0][3]:.3f} m.\nWorst 10:\n{sample}"
        )
