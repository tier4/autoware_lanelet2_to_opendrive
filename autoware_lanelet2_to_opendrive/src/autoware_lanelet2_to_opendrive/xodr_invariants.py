"""Structural invariants an emitted OpenDRIVE file must satisfy.

These are *invariants*, not a golden file: each one is a statement about the
internal consistency of the document, so legitimate changes to the geometry,
the lane layout or the junction decomposition leave them true, while the
defects they describe are the kind that make a consumer follow a link into
nothing.  The ones about lane links are exactly the shape of the defect
behind CARLA's segfault on a forking connector: a lane named a successor id
that did not exist in the road the road-level link pointed at.

Every check here but one was measured at zero on the Nishi-Shinjuku
conversion and on the ``*_mini`` fixtures before being added; a check that
already fires somewhere is a finding to report, not an invariant to assert,
and is deliberately absent.  Three such candidates were measured and
rejected -- see ``test_xodr_structural_invariants`` for what they were and
what they measure today.

The exception is ``linked_endpoints_apart``, which is **not** at zero: on
Nishi-Shinjuku it reports 22 violations over 15 distinct joins, the worst at
0.071 m against a 1 cm tolerance.  It is asserted on its own under
``xfail(strict=True)`` with a ceiling on both the count and the magnitude, so
the defect is pinned rather than hidden and cannot quietly get worse.  Every
other code below is at zero on that map.

Two of them -- ``junction_claim_contested`` and ``connection_unreachable`` --
reach zero only *with* the junction-claim merge that landed alongside them,
which is why they are in that commit and not an earlier one.  At its parent
they fire once each on Nishi-Shinjuku (both ``target`` values: junctions 1002
and 11002 both hold a ``<connection>`` for road 229's end, and road 229's
``<successor>`` names 11002, so junction 1002's row is never read) and twelve
times each on a 6,672-lanelet real map, where five road ends are contested
across nine junctions.  ``junction_cluster`` and the ``*_mini`` fixtures were
already at zero before the fix, so they do not witness it: the smallest input
that does is Nishi-Shinjuku.  The two are independent -- a row can go unread
without its end being contested, when the slot names a junction that claims
nothing there -- so both are asserted rather than one standing in for the
other.

One candidate is **not here because it would fail on arrival**: a lane
``<link>`` on a regular road at a junction-typed end. The format says to omit
it -- which lane you continue into is the junction's ``<connection>`` to
decide, not the road's -- so this is a rule, not a preference, and it would
belong in this set.  The converter emits hundreds of them on Nishi-Shinjuku:
a later pass re-points a road link at a junction and nothing clears the lane
links the earlier pass wrote.  Asserting it today would make the whole set
fail, and a set that fails on arrival gets an exception carved into it or
gets ignored.  Add it once that is fixed; it needs no tolerance and no pinned
count.

Note what that means for the checks below: the lane-link checks examine
road-typed ends only, so the largest known population of dangling lane links
-- the ones at junction-typed ends -- is out of scope by construction, not
because it is clean.

One of the three rejected candidates survives in weakened form.  "Two linked
roads' reference lines meet" is false in general, because a reference line is
anchored to its road's outermost lane boundary and two roads with different
lane counts therefore start metres apart.  Restricted to links whose sides carry the
*same* driving lanes it is worth having, though it is not yet true either:
on Nishi-Shinjuku the restricted form reports 22 violations over 15 joins,
the worst at 0.071 m (see ``linked_endpoints_apart`` above).  When the
anchoring work lands the restriction can simply be dropped.

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
#: ``lane_smoothness.contact_point_no_horizontal_gaps``, so it is the standard's
#: figure rather than one fitted to this converter's output.
#:
#: Nishi-Shinjuku does not meet it: 22 violations over 15 distinct joins sit
#: past 1 cm, the worst three at 7.1 cm (a join is reported from both sides
#: when both ends name each other).  Those are pinned by
#: ``test_nishishinjuku_linked_endpoints_still_gap`` rather than absorbed by a
#: looser tolerance, because a tolerance wide enough to pass them would also
#: pass a genuinely disconnected road.
#:
#: Synthetic divergence stubs are *not* excluded by construction, and the
#: margin here is zero rather than comfortable:
#: :attr:`~.config.GeometryConstants.divergence_min_segment_length` is
#: ``0.01`` -- exactly this tolerance -- and nothing in its documentation
#: bounds it below 1 cm.  A stub whose raw length is shorter than the floor
#: ends up to ``0.01 - raw_length`` away from the point it was meant to reach,
#: i.e. up to exactly 1 cm, and only the strict ``>`` keeps it out.  A stub is
#: single-lane with id +/-1, so against a single-driving-lane road its lane
#: set matches and it is in scope.  Measured, none of the 22 is a stub (the
#: smallest is 0.013 m), but that is a measurement, not a guarantee: raising
#: that floor above 1 cm would start reporting stubs.
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
            # to meet.  Where they do not -- a lane is gained or lost -- the
            # reference line is anchored to each road's outermost lane
            # boundary, so the two legitimately start metres apart and no
            # tolerance applies.  That is why this is conditional rather than
            # a blanket statement about every link; the blanket version is
            # false today, on every map measured.
            #
            # The restriction is on the lane sets, NOT on whether either side
            # is a connecting road: a single-lane connector and a single-lane
            # regular road both carry driving lanes {1}, so that pair is in
            # scope.  Measured on Nishi-Shinjuku, 19 of the 22 violations are
            # regular-to-regular and 3 are connecting-to-regular, so plain
            # road pairs are the dominant population rather than the
            # exception.
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
    # (incoming road, which of its ends) -> the junctions claiming that end.
    claims: Dict[Tuple[str, str], Set[str]] = defaultdict(set)

    for jid, junction in junctions.items():
        for connection in junction.findall("connection"):
            cid = connection.get("id")
            incoming = connection.get("incomingRoad")
            connecting_road = connection.get("connectingRoad")
            where = f"junction {jid} connection {cid}"

            # contactPoint="start" means the connecting road starts at the
            # junction, so the incoming road arrives there and it is its END
            # that this row claims -- and the other way round for "end".
            claimed_end = (
                "end"
                if (connection.get("contactPoint") or "start") == "start"
                else "start"
            )
            claims[(incoming, claimed_end)].add(jid)

            # That end's one <link> slot is what a consumer arriving along the
            # incoming road reads. When it names a different junction, this row
            # is never consulted at all and the connecting road it points at
            # has nothing resolving into it.
            slot = "successor" if claimed_end == "end" else "predecessor"
            element = None
            if incoming in roads:
                element = roads[incoming].find(f"link/{slot}")
            if (
                element is not None
                and element.get("elementType") == "junction"
                and element.get("elementId") != jid
            ):
                out.append(
                    Violation(
                        "connection_unreachable",
                        where,
                        f"incomingRoad {incoming} names junction "
                        f"{element.get('elementId')} at its {claimed_end}, so this "
                        "row is never consulted",
                    )
                )

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

    # A road end carries one <link> slot, so it can name one junction. When
    # two junctions hold <connection> rows for the same end, the slot names
    # one of them and every row of the other is dead: a consumer arriving
    # along that road never opens the second table, and the connecting roads
    # those rows point at are emitted with nothing resolving into them.
    for (incoming, end), jids in sorted(claims.items()):
        if len(jids) > 1:
            out.append(
                Violation(
                    "junction_claim_contested",
                    f"road {incoming}/{end}",
                    f"claimed by junctions {sorted(jids)}; the one <link> slot "
                    "there can name only one of them",
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
