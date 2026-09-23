"""Structural invariants an emitted OpenDRIVE file must satisfy.

These are *invariants*, not a golden file: each one is a statement about the
internal consistency of the document, so legitimate changes to the geometry,
the lane layout or the junction decomposition leave them true, while the
defects they describe are the kind that make a consumer follow a link into
nothing.  The ones about lane links are exactly the shape of the defect
behind CARLA's segfault on a forking connector: a lane named a successor id
that did not exist in the road the road-level link pointed at.

Every check here was measured at zero on the Nishi-Shinjuku conversion, on
``junction_cluster`` and on the ``*_mini`` fixtures before being added; a
check that already fires somewhere is a finding to report, not an invariant
to assert, and is deliberately absent.  Three such candidates were measured
and rejected -- see ``test_xodr_structural_invariants`` for what they were
and what they measure today.

One candidate is **not here because it would fail on arrival**: a lane
``<link>`` on a regular road at a junction-typed end. The format says to omit
it -- which lane you continue into is the junction's ``<connection>`` to
decide, not the road's -- so this is a rule, not a preference, and it would
belong in this set. At ``ca7c9ca7`` the converter emits 501 of them on
Nishi-Shinjuku and 56 on ``junction_cluster``: a later pass re-points a road
link at a junction and nothing clears the lane links the earlier pass wrote.
Asserting it today would make the whole set fail, and a set that fails on
arrival gets an exception carved into it or gets ignored. Add it once that is
fixed; it needs no tolerance and no pinned count.

One of the three rejected candidates survives in weakened form.  "Two linked
roads' reference lines meet" is false in general, because a reference line is
anchored to its road's outermost lane boundary and two roads with different
lane counts therefore start metres apart.  Restricted to links whose sides carry the
*same* driving lanes it is true and worth having: 575 of Nishi-Shinjuku's
1097 road-to-road links qualify and none exceeds 5 mm, and 80 of 146 on
``junction_cluster`` with none above zero.  When the anchoring work lands the
restriction can simply be dropped.

Usage::

    violations = check_invariants(Path("map.xodr"))
    assert not violations, format_violations(violations)
"""

from __future__ import annotations

import math
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Set, Tuple

import lxml.etree as ET

#: Tolerance (m) on the join between consecutive ``<geometry>`` elements of
#: one ``<planView>``.  The emitter chains them by construction, so the
#: residual is pure floating-point: the largest observed over 9948 joins on
#: Nishi-Shinjuku and 2393 on a real-map excerpt was 1.1e-8 m.  One micron
#: leaves two orders of magnitude of headroom and is still four orders below
#: the 1 cm at which a consumer sees a gap.
GEOMETRY_JOIN_TOLERANCE_M = 1e-6

#: Tolerance (m) on ``road@length`` against the sum of its geometries'
#: ``@length``.  Same reasoning as above: the road length is accumulated from
#: the pieces, so anything above rounding means they disagree.
ROAD_LENGTH_TOLERANCE_M = 1e-6

#: Tolerance (m) on the gap between two linked roads' reference-line
#: endpoints, applied only where the two sides carry the same driving lanes
#: (see :func:`_check_road_links`).  One centimetre is ASAM's own contact
#: tolerance in
#: ``lane_smoothness.contact_point_no_horizontal_gaps``.  Measured headroom is
#: thinner than the other tolerances here: the largest gap over 575 such links
#: on Nishi-Shinjuku is 5 mm, and every one of those is a synthetic
#: divergence stub sitting at
#: :attr:`~.config.GeometryConstants.divergence_min_segment_length`, which
#: that constant's own documentation requires to stay strictly below 1 cm for
#: exactly this reason.
LINKED_ENDPOINT_TOLERANCE_M = 0.01


@dataclass(frozen=True)
class Violation:
    """One broken invariant at one place in the document.

    Attributes:
        code: Stable identifier of the invariant.
        where: Human-readable location (road id, junction id, lane id).
        detail: What was expected and what was found.
    """

    code: str
    where: str
    detail: str


def _sections(road: ET._Element) -> List[ET._Element]:
    """Return a road's ``<laneSection>`` elements in file order."""
    lanes = road.find("lanes")
    return [] if lanes is None else lanes.findall("laneSection")


def _section_at(road: ET._Element, contact: str) -> Optional[ET._Element]:
    """Return the lane section a link with *contact* (``start``/``end``) meets."""
    sections = _sections(road)
    if not sections:
        return None
    return sections[0] if contact == "start" else sections[-1]


def _lanes_of(section: Optional[ET._Element]) -> Dict[str, ET._Element]:
    """Return ``{lane id: <lane>}`` for both sides of *section* (never center)."""
    out: Dict[str, ET._Element] = {}
    if section is None:
        return out
    for side in ("left", "right"):
        group = section.find(side)
        if group is not None:
            for lane in group.findall("lane"):
                out[lane.get("id")] = lane
    return out


def _driving_ids(section: Optional[ET._Element]) -> Set[str]:
    """Return the ids of the driving lanes in *section*."""
    return {
        lane_id
        for lane_id, lane in _lanes_of(section).items()
        if lane.get("type") == "driving"
    }


def _reference_endpoint(road: ET._Element, contact: str) -> Tuple[float, float]:
    """Return the world-space point where a link with *contact* meets *road*."""
    geometries = road.find("planView").findall("geometry")
    if contact == "start":
        first = geometries[0]
        return (float(first.get("x")), float(first.get("y")))
    return _geometry_end(geometries[-1])


def _geometry_end(geometry: ET._Element) -> Tuple[float, float]:
    """Return the world-space end point of one ``<geometry>``.

    Supports the three primitives this converter emits: ``line`` (an empty
    ``<geometry>``), ``arc`` and ``paramPoly3`` in either ``pRange``.
    """
    length = float(geometry.get("length"))
    hdg = float(geometry.get("hdg"))
    x = float(geometry.get("x"))
    y = float(geometry.get("y"))
    shape = geometry[0] if len(geometry) else None

    if shape is not None and shape.tag == "paramPoly3":
        au, bu, cu, du = (float(shape.get(k)) for k in ("aU", "bU", "cU", "dU"))
        av, bv, cv, dv = (float(shape.get(k)) for k in ("aV", "bV", "cV", "dV"))
        p = 1.0 if shape.get("pRange") == "normalized" else length
        u = au + bu * p + cu * p * p + du * p**3
        v = av + bv * p + cv * p * p + dv * p**3
        return (
            x + u * math.cos(hdg) - v * math.sin(hdg),
            y + u * math.sin(hdg) + v * math.cos(hdg),
        )

    if shape is not None and shape.tag == "arc":
        curvature = float(shape.get("curvature"))
        if abs(curvature) > 1e-12:
            return (
                x + (math.sin(hdg + curvature * length) - math.sin(hdg)) / curvature,
                y - (math.cos(hdg + curvature * length) - math.cos(hdg)) / curvature,
            )

    return (x + length * math.cos(hdg), y + length * math.sin(hdg))


def check_invariants(source: Path | ET._Element) -> List[Violation]:
    """Check every invariant on an OpenDRIVE document.

    Args:
        source: Path to an ``.xodr`` file, or an already-parsed
            ``<OpenDRIVE>`` root element.

    Returns:
        Every :class:`Violation` found, in check order.  An empty list means
        the document satisfies all of them.
    """
    root = ET.parse(str(source)).getroot() if isinstance(source, Path) else source

    roads = {r.get("id"): r for r in root.iter("road")}
    junctions = {j.get("id"): j for j in root.iter("junction")}
    connecting = {
        rid for rid, r in roads.items() if r.get("junction") not in (None, "-1")
    }
    violations: List[Violation] = []

    _check_road_links(roads, junctions, violations)
    _check_junctions(roads, junctions, connecting, violations)
    _check_geometry(roads, violations)
    return violations


def _check_road_links(
    roads: Dict[str, ET._Element],
    junctions: Dict[str, ET._Element],
    out: List[Violation],
) -> None:
    """Road-level link targets resolve, and lane-level links land on real lanes."""
    for rid, road in roads.items():
        link = road.find("link")
        if link is None:
            continue
        for tag, own_end, default_contact in (
            ("successor", "end", "start"),
            ("predecessor", "start", "end"),
        ):
            element = link.find(tag)
            if element is None:
                continue
            kind = element.get("elementType")
            target = element.get("elementId")

            if kind == "junction":
                if target not in junctions:
                    out.append(
                        Violation(
                            "link_target_missing",
                            f"road {rid}/{tag}",
                            f"names junction {target}, which does not exist",
                        )
                    )
                continue

            if kind != "road":
                continue
            if target not in roads:
                out.append(
                    Violation(
                        "link_target_missing",
                        f"road {rid}/{tag}",
                        f"names road {target}, which does not exist",
                    )
                )
                continue

            contact = element.get("contactPoint") or default_contact
            own_section = _section_at(road, own_end)
            target_section = _section_at(roads[target], contact)

            # Where the two sides present the same driving lanes, their
            # reference lines describe the same edge of the same road and have
            # to meet. Where they do not -- a lane is gained or lost, or one
            # side is a connecting road -- the reference line is anchored to
            # each road's outermost lane boundary, so the two legitimately
            # start metres apart and no tolerance applies. That is why this is
            # conditional rather than a blanket statement about every link;
            # the blanket version is false today, on every map measured.
            if _driving_ids(own_section) == _driving_ids(target_section):
                gap = math.dist(
                    _reference_endpoint(road, own_end),
                    _reference_endpoint(roads[target], contact),
                )
                if gap > LINKED_ENDPOINT_TOLERANCE_M:
                    out.append(
                        Violation(
                            "linked_endpoints_apart",
                            f"road {rid}/{tag}",
                            f"meets road {target} at its {contact} {gap:.3f} m "
                            f"away, though both carry driving lanes "
                            f"{sorted(_driving_ids(own_section))} "
                            f"(tolerance {LINKED_ENDPOINT_TOLERANCE_M} m)",
                        )
                    )

            target_lanes = _lanes_of(target_section)
            for lane_id, lane in _lanes_of(own_section).items():
                lane_link = lane.find("link")
                if lane_link is None:
                    continue
                lane_target = lane_link.find(tag)
                if lane_target is None:
                    continue
                if lane_target.get("id") not in target_lanes:
                    out.append(
                        Violation(
                            "lane_link_target_missing",
                            f"road {rid} lane {lane_id}/{tag}",
                            f"names lane {lane_target.get('id')} of road {target} "
                            f"at its {contact}, which has lanes "
                            f"{sorted(target_lanes)}",
                        )
                    )


def _check_junctions(
    roads: Dict[str, ET._Element],
    junctions: Dict[str, ET._Element],
    connecting: Set[str],
    out: List[Violation],
) -> None:
    """``<connection>`` elements name real roads, in the roles the format gives them."""
    owners: Dict[str, Set[str]] = defaultdict(set)

    for jid, junction in junctions.items():
        for connection in junction.findall("connection"):
            cid = connection.get("id")
            incoming = connection.get("incomingRoad")
            connecting_road = connection.get("connectingRoad")
            where = f"junction {jid} connection {cid}"

            if incoming not in roads:
                out.append(
                    Violation(
                        "connection_road_missing",
                        where,
                        f"incomingRoad {incoming} does not exist",
                    )
                )
            elif incoming in connecting:
                # An incoming road is a road that arrives at the junction from
                # outside it. Naming a connecting road here makes a consumer
                # look for the junction's entry inside the junction itself.
                out.append(
                    Violation(
                        "incoming_road_is_connecting",
                        where,
                        f"incomingRoad {incoming} is itself a connecting road "
                        f"(junction={roads[incoming].get('junction')})",
                    )
                )

            if connecting_road not in roads:
                out.append(
                    Violation(
                        "connection_road_missing",
                        where,
                        f"connectingRoad {connecting_road} does not exist",
                    )
                )
                continue

            owners[connecting_road].add(jid)
            declared = roads[connecting_road].get("junction")
            if declared != jid:
                out.append(
                    Violation(
                        "connecting_road_junction_mismatch",
                        where,
                        f"connectingRoad {connecting_road} declares "
                        f"junction={declared}",
                    )
                )

            contact = connection.get("contactPoint") or "start"
            to_lanes = _lanes_of(_section_at(roads[connecting_road], contact))
            from_lanes: Set[str] = set()
            if incoming in roads:
                for end in ("start", "end"):
                    from_lanes |= set(_lanes_of(_section_at(roads[incoming], end)))
            for lane_link in connection.findall("laneLink"):
                if from_lanes and lane_link.get("from") not in from_lanes:
                    out.append(
                        Violation(
                            "lane_link_target_missing",
                            f"{where} laneLink",
                            f"@from={lane_link.get('from')} is not a lane of "
                            f"incomingRoad {incoming}",
                        )
                    )
                if lane_link.get("to") not in to_lanes:
                    out.append(
                        Violation(
                            "lane_link_target_missing",
                            f"{where} laneLink",
                            f"@to={lane_link.get('to')} is not a lane of "
                            f"connectingRoad {connecting_road} at its {contact}",
                        )
                    )

    for rid, jids in owners.items():
        if rid not in connecting:
            out.append(
                Violation(
                    "plain_road_used_as_connecting",
                    f"road {rid}",
                    f"declares junction=-1 but junction(s) {sorted(jids)} name it "
                    "as a connectingRoad",
                )
            )
        elif len(jids) > 1:
            out.append(
                Violation(
                    "connecting_road_shared",
                    f"road {rid}",
                    f"named as connectingRoad by junctions {sorted(jids)}",
                )
            )


def _check_geometry(roads: Dict[str, ET._Element], out: List[Violation]) -> None:
    """A road's reference line is continuous and as long as it says it is."""
    for rid, road in roads.items():
        plan_view = road.find("planView")
        if plan_view is None:
            continue
        geometries = plan_view.findall("geometry")
        for index in range(len(geometries) - 1):
            gap = math.dist(
                _geometry_end(geometries[index]),
                (
                    float(geometries[index + 1].get("x")),
                    float(geometries[index + 1].get("y")),
                ),
            )
            if gap > GEOMETRY_JOIN_TOLERANCE_M:
                out.append(
                    Violation(
                        "geometry_discontinuity",
                        f"road {rid} geometry {index}->{index + 1}",
                        f"{gap:.3e} m apart (tolerance "
                        f"{GEOMETRY_JOIN_TOLERANCE_M:.0e} m)",
                    )
                )

        declared = road.get("length")
        if declared is None or not geometries:
            continue
        total = sum(float(g.get("length")) for g in geometries)
        if abs(total - float(declared)) > ROAD_LENGTH_TOLERANCE_M:
            out.append(
                Violation(
                    "road_length_mismatch",
                    f"road {rid}",
                    f"@length={declared} but its geometries sum to {total!r}",
                )
            )


def format_violations(violations: Sequence[Violation], max_shown: int = 10) -> str:
    """Render violations as an assertion message, grouped by invariant.

    Args:
        violations: What :func:`check_invariants` returned.
        max_shown: Maximum number of instances listed per invariant.

    Returns:
        A multi-line string, or a short OK line when there is nothing to show.
    """
    if not violations:
        return "no invariant violations"
    grouped: Dict[str, List[Violation]] = defaultdict(list)
    for violation in violations:
        grouped[violation.code].append(violation)
    lines = [f"{len(violations)} invariant violation(s):"]
    for code, group in sorted(grouped.items()):
        lines.append(f"  [{code}] {len(group)}")
        for violation in group[:max_shown]:
            lines.append(f"    {violation.where}: {violation.detail}")
        if len(group) > max_shown:
            lines.append(f"    ... and {len(group) - max_shown} more")
    return "\n".join(lines)
