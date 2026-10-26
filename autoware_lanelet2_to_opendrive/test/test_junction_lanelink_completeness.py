"""Regression test for junction laneLink completeness.

Every driving lane on a road whose ``<successor elementType="junction" .../>``
points at junction J must appear as ``from`` in at least one
``<laneLink>`` of a ``<connection>`` whose ``incomingRoad`` is that road,
inside J. Without this guarantee, a vehicle entering the junction in that
lane has no path forward and stalls (odrviewer ``--stop_at_end_of_road``
or CARLA route planner).

Prior to the fix, ``Junction.build_connections_from_roads`` enumerated
only ``turn_direction``-tagged junction-internal lanelets. Source maps
that connect an incoming lane *directly* to an outgoing lane (no
junction-internal connector lanelet) therefore had no
``<connection>`` table entry, and 10 driving lanes (~4% of the 251
junction-bound lanes in Nishishinjuku) were silently dropped.
"""

from collections import defaultdict
from typing import Dict, List, Set, Tuple

import lxml.etree as ET


def _scan_missing_lane_links(
    tree: ET._ElementTree,
) -> Tuple[List[Tuple[int, int, int]], int]:
    """Return ``(missing, total)`` for junction-bound driving lanes.

    ``missing`` is the list of ``(road_id, lane_id, junction_id)`` tuples
    where the road's successor is junction J, the lane is a driving lane
    in the last lane section, but no ``<connection>`` of J carries a
    ``<laneLink from="lane_id"/>`` whose ``incomingRoad`` matches.
    ``total`` is the count of all junction-bound driving lanes (the
    denominator the spec's ~4% figure was computed against).
    """
    root = tree.getroot()

    # (incomingRoad, from_lane) -> hit count across every <laneLink> in
    # every <connection> of every <junction>. The same incoming road
    # never appears as incomingRoad for two different junctions, so we
    # don't need to key on junction id — any hit means the lane is
    # covered.
    covered: Dict[Tuple[int, int], int] = defaultdict(int)
    for j in root.iterfind("junction"):
        for conn in j.iterfind("connection"):
            try:
                in_road = int(conn.get("incomingRoad"))
            except (TypeError, ValueError):
                continue
            for ll in conn.iterfind("laneLink"):
                try:
                    fl = int(ll.get("from"))
                except (TypeError, ValueError):
                    continue
                covered[(in_road, fl)] += 1

    missing: List[Tuple[int, int, int]] = []
    total = 0
    for road in root.iterfind("road"):
        link = road.find("link")
        if link is None:
            continue
        succ = link.find("successor")
        if succ is None or succ.get("elementType") != "junction":
            continue
        sections = road.findall("lanes/laneSection")
        if not sections:
            continue
        last_sec = sections[-1]
        rid = int(road.get("id"))
        jid = int(succ.get("elementId"))
        for side in ("left", "right"):
            side_elem = last_sec.find(side)
            if side_elem is None:
                continue
            for lane in side_elem.iterfind("lane"):
                if lane.get("type") != "driving":
                    continue
                lid = int(lane.get("id"))
                total += 1
                if covered[(rid, lid)] == 0:
                    missing.append((rid, lid, jid))
    return missing, total


def test_no_junction_bound_lane_lacks_a_lanelink(nishishinjuku_xodr):
    """Every junction-bound driving lane must appear in the junction's table.

    Pre-fix value on master @ b01c8512: 10 missing of 251 junction-bound
    lanes. Post-fix must be zero — every direct external→external
    transition implied by the routing graph is materialised as a
    synthetic zero-length connecting road.
    """
    tree = ET.parse(str(nishishinjuku_xodr))
    missing, total = _scan_missing_lane_links(tree)
    assert total >= 200, (
        f"Sanity floor: junction-bound lanes total ({total}) collapsed; "
        "the test fixture is no longer exercising the junction code path."
    )
    assert not missing, (
        f"{len(missing)}/{total} junction-bound driving lanes lack a "
        f"<laneLink>. Sample: {missing[:5]}"
    )


def _chained_connectors(root: ET._Element) -> Dict[int, Tuple[int, int]]:
    """Map every chained connecting road to ``(junction_id, predecessor)``.

    A *chained* connector is a connecting road whose road-level
    ``<link><predecessor>`` is another connecting road of the same
    junction: it is entered by traversing that earlier stage, so a
    vehicle cannot be at its start without having driven the stage
    before it.
    """
    junction_of: Dict[int, int] = {}
    predecessor_of: Dict[int, int] = {}
    for road in root.iterfind("road"):
        rid = int(road.get("id"))
        junction_of[rid] = int(road.get("junction", "-1"))
        link = road.find("link")
        pred = link.find("predecessor") if link is not None else None
        if pred is not None and pred.get("elementType") == "road":
            predecessor_of[rid] = int(pred.get("elementId"))

    chained: Dict[int, Tuple[int, int]] = {}
    for rid, jid in junction_of.items():
        pred_id = predecessor_of.get(rid)
        if jid == -1 or pred_id is None:
            continue
        if junction_of.get(pred_id, -1) == jid:
            chained[rid] = (jid, pred_id)
    return chained


def test_no_connection_advertises_a_chained_connecting_road(nishishinjuku_xodr):
    """A later stage of an in-junction chain is never directly reachable.

    ``_enumerate_lane_pairs`` used to walk back through in-junction
    predecessors so that a chained connector's ``<connection>`` could
    name a non-junction ``incomingRoad``. The side effect was that the
    external road was advertised as connected to *every* stage of the
    chain, letting a consumer enter the second connector and skip the
    first — a physical jump of one connector length (24 such transitions
    on this fixture, e.g. junction 1015 listing incomingRoad 12 with both
    connectingRoad 541 and its own predecessor 542).
    """
    root = ET.parse(str(nishishinjuku_xodr)).getroot()
    chained = _chained_connectors(root)
    assert len(chained) >= 10, (
        f"only {len(chained)} chained connector(s) in the fixture; the "
        "assertion below no longer exercises the chain path."
    )

    offenders: List[Tuple[int, int, int]] = []
    for junction in root.iterfind("junction"):
        jid = int(junction.get("id"))
        for conn in junction.iterfind("connection"):
            connecting = int(conn.get("connectingRoad"))
            if connecting in chained and chained[connecting][0] == jid:
                offenders.append((jid, int(conn.get("incomingRoad")), connecting))

    assert not offenders, (
        f"{len(offenders)} <connection>(s) advertise a chained connecting "
        f"road as directly reachable. Sample: {offenders[:5]}"
    )


def test_chained_connectors_stay_reachable_through_their_chain(nishishinjuku_xodr):
    """Dropping the shortcut must not strand the chain.

    Following the road-level ``<link><predecessor>`` chain backwards from
    every chained connector has to arrive at a connecting road that *is*
    in its junction's ``<connection>`` table, so a consumer can still
    enter the junction from a non-junction road and traverse the chain in
    order.
    """
    root = ET.parse(str(nishishinjuku_xodr)).getroot()
    chained = _chained_connectors(root)

    advertised: Dict[int, Set[int]] = defaultdict(set)
    for junction in root.iterfind("junction"):
        jid = int(junction.get("id"))
        for conn in junction.iterfind("connection"):
            advertised[jid].add(int(conn.get("connectingRoad")))

    stranded: List[Tuple[int, int]] = []
    for connecting, (jid, _pred) in chained.items():
        head = connecting
        seen: Set[int] = set()
        while head in chained and head not in seen:
            seen.add(head)
            head = chained[head][1]
        if head not in advertised[jid]:
            stranded.append((jid, connecting))

    assert not stranded, (
        f"{len(stranded)} chained connector(s) have no advertised chain "
        f"head and are unreachable. Sample: {stranded[:5]}"
    )


def test_known_problem_lanes_are_covered(nishishinjuku_xodr):
    """The 10 specific (road, lane, junction) triples from the audit must be covered.

    Listed explicitly so a future regression that re-drops *exactly* the
    same set is caught with a precise diagnostic, instead of just a count
    delta.
    """
    expected = {
        (146, 2, 1020),
        (174, 2, 1024),
        (211, 1, 1000),
        (214, 1, 1002),
        (217, 2, 1005),
        (217, 3, 1005),
        (227, 2, 1008),
        (230, 2, 1006),
        (238, 2, 1006),
        (261, 1, 1013),
    }
    tree = ET.parse(str(nishishinjuku_xodr))
    missing, _ = _scan_missing_lane_links(tree)
    missing_set: Set[Tuple[int, int, int]] = set(missing)
    still_missing = expected & missing_set
    assert not still_missing, (
        f"the previously-known direct-transition lanes are still missing: "
        f"{sorted(still_missing)}"
    )
