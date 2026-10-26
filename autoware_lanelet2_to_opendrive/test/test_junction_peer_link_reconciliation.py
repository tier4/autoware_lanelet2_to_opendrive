"""Regression tests for one-sided junction road links.

``set_incoming_road_junction_links`` and ``_apply_outgoing_junction_links``
both replace a road-to-road link with the junction link that takes priority,
but only on the road they are looking at. The peer keeps naming that road,
so one contact point ends up described two different ways: road A says
``<successor elementType="road" elementId="B">`` while road B says
``<predecessor elementType="junction">``.

Lane links are read against the road link of their own road, so the two
sides disagree at lane level too — which is exactly what the ASAM checker
``check_asam_xodr_road_lane_link_lanes_across_lane_sections`` ("Lanes that
continue across the lane sections shall be connected in both directions")
reports: 20 findings on Nishishinjuku before the fix.
"""

from __future__ import annotations

import subprocess
from pathlib import Path

import lxml.etree as ET
import pytest

from autoware_lanelet2_to_opendrive.direct_junction_completion import (
    _dangling_road_link_peer,
)
from autoware_lanelet2_to_opendrive.opendrive.enums import ContactPoint, ElementType
from autoware_lanelet2_to_opendrive.opendrive.road import Road

# --- pure helper: _dangling_road_link_peer ---------------------------------


def _pair(
    *,
    peer_facing_tag: str,
    peer_facing_type: ElementType,
    peer_facing_id: int,
) -> tuple[Road, dict[int, Road]]:
    """Road 1 --successor--> road 2@start, with road 2's predecessor set up."""
    road = Road(id=1, junction=-1)
    road.add_successor(
        element_id=2,
        element_type=ElementType.ROAD,
        contact_point=ContactPoint.START,
    )
    peer = Road(id=2, junction=-1)
    if peer_facing_tag == "predecessor":
        peer.add_predecessor(element_id=peer_facing_id, element_type=peer_facing_type)
    else:
        peer.add_successor(element_id=peer_facing_id, element_type=peer_facing_type)
    return road, {1: road, 2: peer}


def test_detects_a_successor_the_peer_answers_with_a_junction():
    road, road_by_id = _pair(
        peer_facing_tag="predecessor",
        peer_facing_type=ElementType.JUNCTION,
        peer_facing_id=1002,
    )

    found = _dangling_road_link_peer(road, "successor", road_by_id)

    assert found is not None
    peer, junction_id = found
    assert peer.id == 2
    assert junction_id == 1002


def test_detects_a_predecessor_the_peer_answers_with_a_junction():
    """The mirror case, left behind by ``set_incoming_road_junction_links``."""
    road = Road(id=1, junction=-1)
    road.add_predecessor(
        element_id=2,
        element_type=ElementType.ROAD,
        contact_point=ContactPoint.END,
    )
    peer = Road(id=2, junction=-1)
    peer.add_successor(element_id=1006, element_type=ElementType.JUNCTION)

    found = _dangling_road_link_peer(road, "predecessor", {1: road, 2: peer})

    assert found == (peer, 1006)


def test_ignores_a_reciprocated_road_link():
    road, road_by_id = _pair(
        peer_facing_tag="predecessor",
        peer_facing_type=ElementType.ROAD,
        peer_facing_id=1,
    )

    assert _dangling_road_link_peer(road, "successor", road_by_id) is None


def test_ignores_a_peer_that_names_a_third_road():
    """Not a junction link — a different defect, and not this pass's to fix."""
    road, road_by_id = _pair(
        peer_facing_tag="predecessor",
        peer_facing_type=ElementType.ROAD,
        peer_facing_id=99,
    )

    assert _dangling_road_link_peer(road, "successor", road_by_id) is None


def test_ignores_a_peer_with_no_link_at_the_contacting_end():
    road = Road(id=1, junction=-1)
    road.add_successor(
        element_id=2,
        element_type=ElementType.ROAD,
        contact_point=ContactPoint.START,
    )
    peer = Road(id=2, junction=-1)

    assert _dangling_road_link_peer(road, "successor", {1: road, 2: peer}) is None


def test_ignores_the_wrong_end_of_the_peer():
    """contactPoint=start must be answered by the peer's *predecessor*."""
    road, road_by_id = _pair(
        peer_facing_tag="successor",
        peer_facing_type=ElementType.JUNCTION,
        peer_facing_id=1002,
    )

    assert _dangling_road_link_peer(road, "successor", road_by_id) is None


def test_ignores_connecting_roads():
    """A connecting road's links are owned by its junction, not by this pass."""
    road = Road(id=1, junction=7)
    road.add_successor(
        element_id=2,
        element_type=ElementType.ROAD,
        contact_point=ContactPoint.START,
    )
    peer = Road(id=2, junction=-1)
    peer.add_predecessor(element_id=1002, element_type=ElementType.JUNCTION)

    assert _dangling_road_link_peer(road, "successor", {1: road, 2: peer}) is None


def test_ignores_a_connecting_road_as_the_peer():
    road = Road(id=1, junction=-1)
    road.add_successor(
        element_id=2,
        element_type=ElementType.ROAD,
        contact_point=ContactPoint.START,
    )
    peer = Road(id=2, junction=7)
    peer.add_predecessor(element_id=1002, element_type=ElementType.JUNCTION)

    assert _dangling_road_link_peer(road, "successor", {1: road, 2: peer}) is None


# --- end-to-end regression --------------------------------------------------


@pytest.fixture(scope="session")
def nishishinjuku_xodr(tmp_path_factory) -> Path:
    """Convert the Nishishinjuku fixture once per session and return the XODR.

    Only an end-to-end conversion exercises the pass this fix lives in. As in
    ``test_outgoing_junction_links.py``, the output goes to a fresh temp
    directory so the assertions can never read an XODR left behind by an
    earlier checkout.
    """
    fixture = Path(
        "autoware_lanelet2_to_opendrive/test/data/nishishinjuku.osm"
    ).resolve()
    if not fixture.is_file():
        pytest.skip(f"{fixture} not available; cannot build XODR")

    xodr_path = tmp_path_factory.mktemp("junction_peer_links") / "nishishinjuku.xodr"
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


def _lane_sections(road):
    return road.findall("./lanes/laneSection")


def _lanes(lane_section):
    lanes = []
    for side in ("left", "right"):
        group = lane_section.find(side)
        if group is not None:
            lanes.extend(group.findall("lane"))
    return lanes


def _linked_lane_ids(lane, tag: str) -> list[int]:
    link = lane.find("link")
    if link is None:
        return []
    return [int(e.get("id")) for e in link.findall(tag) if e.get("id") is not None]


def _road_linkage(road, tag: str):
    """``(road id, contactPoint)`` for a road-typed link, else ``None``."""
    link = road.find("link")
    if link is None:
        return None
    element = link.find(tag)
    if element is None or element.get("elementType") != "road":
        return None
    road_id, contact = element.get("elementId"), element.get("contactPoint")
    if road_id is None or contact is None:
        return None
    return int(road_id), contact


def _one_direction(first, first_tag, second, second_tag, road_a, road_b):
    """Findings for ``first``'s lanes not answered by ``second``'s lanes."""
    findings = []
    for lane in _lanes(first):
        lane_id = lane.get("id")
        if lane_id is None:
            continue
        for target_id in _linked_lane_ids(lane, first_tag):
            target = next(
                (
                    other
                    for other in _lanes(second)
                    if other.get("id") == str(target_id)
                ),
                None,
            )
            if target is None:
                continue
            if int(lane_id) not in _linked_lane_ids(target, second_tag):
                findings.append(
                    f"road {road_a} lane {lane_id} --{first_tag}--> "
                    f"road {road_b} lane {target_id}, which does not name it back"
                )
    return findings


def _both_directions(first, first_tag, second, second_tag, road_a, road_b):
    return _one_direction(
        first, first_tag, second, second_tag, road_a, road_b
    ) + _one_direction(second, second_tag, first, first_tag, road_b, road_a)


def _lanes_across_lane_section_findings(tree) -> list[str]:
    """Reimplementation of ``road.lane.link.lanes_across_lane_sections``.

    Mirrors ``qc_opendrive``'s checker: roads inside a junction are exempt
    for their first/last lane section (the standard omits the lane ``<link>``
    there), only road-typed road linkages are followed, and the contact point
    picks which lane section of the linked road answers.
    """
    roads = {int(r.get("id")): r for r in tree.iterfind(".//road")}
    findings: list[str] = []

    for road_id, road in roads.items():
        sections = _lane_sections(road)
        for i in range(1, len(sections)):
            findings += _both_directions(
                sections[i - 1],
                "successor",
                sections[i],
                "predecessor",
                road_id,
                road_id,
            )

        if road.get("junction") != "-1" or not sections:
            continue

        for tag, own_section in (
            ("predecessor", sections[0]),
            ("successor", sections[-1]),
        ):
            linkage = _road_linkage(road, tag)
            if linkage is None:
                continue
            other_id, contact = linkage
            other = roads.get(other_id)
            if other is None:
                continue
            other_sections = _lane_sections(other)
            if not other_sections:
                continue
            if contact == "start":
                other_section, other_tag = other_sections[0], "predecessor"
            elif contact == "end":
                other_section, other_tag = other_sections[-1], "successor"
            else:
                continue
            findings += _both_directions(
                own_section, tag, other_section, other_tag, road_id, other_id
            )

    return findings


def test_lanes_continuing_across_lane_sections_are_linked_both_ways(
    nishishinjuku_xodr,
):
    """ASAM ``road.lane.link.lanes_across_lane_sections`` must report nothing."""
    tree = ET.parse(str(nishishinjuku_xodr))

    findings = _lanes_across_lane_section_findings(tree)

    assert findings == [], (
        f"{len(findings)} lane link(s) recorded in only one direction:\n  "
        + "\n  ".join(findings[:20])
    )


def test_no_road_link_is_answered_with_a_junction_link(nishishinjuku_xodr):
    """The structural cause: a road-typed link the peer answers with a junction.

    Asserted separately from the checker above because a road pair can be
    one-sided without any lane link crossing it — that still misstates the
    topology, and it is one re-point away from producing findings again.
    """
    tree = ET.parse(str(nishishinjuku_xodr))
    roads = {int(r.get("id")): r for r in tree.iterfind(".//road")}

    one_sided = []
    for road_id, road in roads.items():
        if road.get("junction") != "-1":
            continue
        for tag in ("predecessor", "successor"):
            linkage = _road_linkage(road, tag)
            if linkage is None:
                continue
            other_id, contact = linkage
            other = roads.get(other_id)
            if other is None or other.get("junction") != "-1":
                continue
            facing = other.find(
                "link/predecessor" if contact == "start" else "link/successor"
            )
            if facing is not None and facing.get("elementType") == "junction":
                one_sided.append(
                    f"road {road_id} {tag} -> road {other_id}@{contact}, "
                    f"which names junction {facing.get('elementId')} back"
                )

    assert one_sided == [], (
        f"{len(one_sided)} one-sided road link(s):\n  " + "\n  ".join(one_sided[:20])
    )
