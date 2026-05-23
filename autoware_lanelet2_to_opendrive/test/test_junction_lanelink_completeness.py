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

import subprocess
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Set, Tuple

import lxml.etree as ET
import pytest


@pytest.fixture(scope="session")
def nishishinjuku_xodr(tmp_path_factory) -> Path:
    """Convert the Nishishinjuku fixture once per session and return the XODR.

    Mirrors ``test_connecting_road_links.py``: a fresh session-scoped
    temp directory rather than a fixed cached path, so a regression test
    always exercises the *current* converter.
    """
    fixture = Path(
        "autoware_lanelet2_to_opendrive/test/data/nishishinjuku.osm"
    ).resolve()
    if not fixture.is_file():
        pytest.skip(f"{fixture} not available; cannot build XODR")

    xodr_path = tmp_path_factory.mktemp("lanelink_completeness") / "nishishinjuku.xodr"
    cmd = [
        "uv",
        "run",
        "convert",
        "map=nishishinjuku",
        "target=carla",
        f"input_map_path={fixture}",
        f"output_map_path={xodr_path}",
    ]
    try:
        subprocess.run(cmd, check=True)
    except FileNotFoundError as exc:
        pytest.skip(f"converter unavailable: {exc}")

    if not xodr_path.is_file():
        pytest.fail(f"converter exited successfully but {xodr_path} was not produced")
    return xodr_path


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
