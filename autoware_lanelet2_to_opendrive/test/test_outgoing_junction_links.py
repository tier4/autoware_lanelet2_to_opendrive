"""Regression tests for issue #494 Part A — outgoing-road junction links.

The OpenDRIVE ``<connection>`` table records only the *incoming* road of a
junction, so before the fix a regular road *leaving* a junction received no
``<link><predecessor>``.  When such a road also had no onward successor it
was emitted as a topologically orphaned island (roads 43/47/93/113 in the
Nishishinjuku conversion).  ``Road.set_outgoing_road_junction_links`` walks
the routing graph to restore the missing junction predecessor link.
"""

import lxml.etree as ET
import pytest

from autoware_lanelet2_to_opendrive.opendrive.enums import ElementType
from autoware_lanelet2_to_opendrive.opendrive.road import (
    Road,
    _apply_outgoing_junction_links,
    _resolve_outgoing_junction_links,
)

# --- pure helper: _resolve_outgoing_junction_links --------------------------


def test_resolve_maps_regular_road_to_the_junction_it_exits():
    """A regular road reached from a connecting lanelet exits that junction."""
    # road 1 = regular (lanelets 10, 11); road 9 = connecting road of junction 100.
    road_to_lanelet_ids = {1: [10, 11], 9: [90]}
    road_junction = {1: -1, 9: 100}
    # both lanelets of road 1 are reached from connecting lanelet 90
    routing_previous = {10: [90], 11: [90]}

    assert _resolve_outgoing_junction_links(
        road_to_lanelet_ids, road_junction, routing_previous
    ) == {1: 100}


def test_resolve_ignores_road_reached_only_from_regular_lanelets():
    """A road whose predecessors are all regular roads exits no junction."""
    road_to_lanelet_ids = {1: [10], 2: [20]}
    road_junction = {1: -1, 2: -1}
    routing_previous = {10: [20], 20: []}

    assert (
        _resolve_outgoing_junction_links(
            road_to_lanelet_ids, road_junction, routing_previous
        )
        == {}
    )


def test_resolve_skips_road_that_resolves_to_multiple_junctions():
    """An ambiguous road (two junctions) is omitted — one link cannot point at both."""
    road_to_lanelet_ids = {1: [10, 11], 8: [80], 9: [90]}
    road_junction = {1: -1, 8: 100, 9: 200}
    routing_previous = {10: [80], 11: [90]}

    assert (
        _resolve_outgoing_junction_links(
            road_to_lanelet_ids, road_junction, routing_previous
        )
        == {}
    )


def test_resolve_does_not_treat_connecting_roads_as_outgoing():
    """Connecting roads (junction >= 0) are never assigned an outgoing link."""
    road_to_lanelet_ids = {9: [90], 8: [80]}
    road_junction = {9: 100, 8: 100}
    routing_previous = {90: [80]}

    assert (
        _resolve_outgoing_junction_links(
            road_to_lanelet_ids, road_junction, routing_previous
        )
        == {}
    )


# --- pure helper: _apply_outgoing_junction_links ----------------------------


def test_apply_sets_junction_predecessor_on_orphan_road():
    road = Road(id=43, junction=-1)

    _apply_outgoing_junction_links([road], {43: 1016})

    assert road.link is not None
    assert road.link.predecessor is not None
    assert road.link.predecessor.element_type == ElementType.JUNCTION
    assert road.link.predecessor.element_id == 1016


def test_apply_overwrites_a_stale_road_predecessor_with_the_junction():
    road = Road(id=43, junction=-1)
    road.add_predecessor(element_id=7, element_type=ElementType.ROAD)

    _apply_outgoing_junction_links([road], {43: 1016})

    assert road.link.predecessor.element_type == ElementType.JUNCTION
    assert road.link.predecessor.element_id == 1016


def test_apply_preserves_an_existing_junction_predecessor():
    road = Road(id=43, junction=-1)
    road.add_predecessor(element_id=999, element_type=ElementType.JUNCTION)

    _apply_outgoing_junction_links([road], {43: 1016})

    # Already a junction link — left untouched.
    assert road.link.predecessor.element_id == 999


def test_apply_leaves_an_existing_successor_untouched():
    road = Road(id=43, junction=-1)
    road.add_successor(element_id=50, element_type=ElementType.ROAD)

    _apply_outgoing_junction_links([road], {43: 1016})

    assert road.link.successor is not None
    assert road.link.successor.element_id == 50
    assert road.link.predecessor.element_type == ElementType.JUNCTION


# --- end-to-end regression --------------------------------------------------


def _is_driving_road(road: ET._Element) -> bool:
    return any(
        lane.get("type") == "driving" for lane in road.iterfind(".//lane[@type]")
    )


def _real_junction_ids(tree: ET._ElementTree) -> set:
    """Ids of real junctions — those below the synthetic-divergence id floor.

    Synthetic divergence/merge junctions use ``junction_id_offset + 10_000``;
    the offset keeps real ids well under 10k.
    """
    return {
        j.get("id") for j in tree.iterfind(".//junction") if int(j.get("id")) < 10_000
    }


def _roads_with_real_junction_predecessor(tree: ET._ElementTree):
    """Yield ``(road, junction_id)`` for regular roads with a real-junction predecessor."""
    real = _real_junction_ids(tree)
    for road in tree.iterfind(".//road"):
        if road.get("junction") != "-1":
            continue
        pred = road.find("link/predecessor")
        if (
            pred is not None
            and pred.get("elementType") == "junction"
            and pred.get("elementId") in real
        ):
            yield road, pred.get("elementId")


def test_no_driving_road_is_emitted_topologically_orphaned(nishishinjuku_xodr):
    """Issue #494 Part A: no drivable non-junction road may lack both links."""
    tree = ET.parse(str(nishishinjuku_xodr))

    orphans = []
    for road in tree.iterfind(".//road"):
        if road.get("junction") != "-1" or not _is_driving_road(road):
            continue
        link = road.find("link")
        pred = link.find("predecessor") if link is not None else None
        succ = link.find("successor") if link is not None else None
        if pred is None and succ is None:
            orphans.append(road.get("id"))

    assert orphans == [], (
        f"drivable non-junction roads emitted with no predecessor/successor "
        f"link: {orphans}"
    )


@pytest.mark.xfail(
    strict=True,
    reason="9 drivable roads on Nishishinjuku claim a junction that names "
    "them in no <connection> and carry no <link>, so nothing in the file "
    "reaches them; the fix is a design decision, see the docstring",
)
def test_no_driving_road_is_unreachable(nishishinjuku_xodr):
    """The invariant above, without the exclusion that hides its own population.

    ``test_no_driving_road_is_emitted_topologically_orphaned`` skips every
    road whose ``junction`` is not ``-1``. That exclusion is reasonable on its
    face -- a connecting road is reached through its junction, not through a
    road link -- but it is only true when a ``<connection>`` names the road.
    A road that claims a junction and appears in no connection is reached
    through neither, and the filter removes exactly that population from the
    check. The invariant therefore reads as satisfied while the roads it
    exists to forbid are present.

    A road is reachable here if it carries a road ``<link>`` with at least one
    side, or some junction's ``<connection>`` names it as a ``connectingRoad``.

    Measured: **9 drivable roads on Nishishinjuku** (380, 381, 409, 420, 436,
    445, 696, 697, 698) and **70 on the 36,249-lanelet customer map**
    (11.1-132.6 m each, p50 42.7 m, 67 with a driving lane, no lane ``<link>``
    at all) satisfy neither. On the customer map 462 of 8,200 connecting roads
    (5.6%, 18.3 km) appear in no connection; those 70 are the ones that also
    have no road link.

    The cause is largely the source map and is not ours: by Lanelet2's own
    definition of connectivity -- a predecessor's boundary end points being
    this lanelet's start points -- 95 of the 107 lanelets behind those 70
    roads share their start points with nothing, so there is no incoming
    movement to emit a connection from.

    What to *do* is a design decision, which is why this is an xfail and not
    a fix:

    * Writing ``junction="-1"`` is **not** an option. It satisfies this test's
      letter and breaks the test above: these roads become drivable
      non-junction roads with no links, which is the shape #494 Part A
      forbids. Measured -- it turns the 9 above into 9 failures there.
    * Not emitting them keeps both invariants but silently drops real
      pavement from the map, and removes its lanelets from the mapping.
    * Emitting a connection needs an incoming road, which for 95 of 107 does
      not exist.
    """
    tree = ET.parse(str(nishishinjuku_xodr))

    backed = {
        connection.get("connectingRoad")
        for connection in tree.iterfind(".//junction/connection")
    }

    unreachable = []
    for road in tree.iterfind(".//road"):
        if not _is_driving_road(road):
            continue
        link = road.find("link")
        has_link = link is not None and (
            link.find("predecessor") is not None or link.find("successor") is not None
        )
        if has_link or road.get("id") in backed:
            continue
        unreachable.append(road.get("id"))

    assert unreachable == [], (
        "drivable roads that no <link> and no <connection> reaches: " f"{unreachable}"
    )


def test_every_real_junction_has_an_outgoing_road_linked_back(nishishinjuku_xodr):
    """Every real junction must be referenced as a <predecessor> by an outgoing road.

    Asserting only that *some* road is linked would still pass if most
    junction-outgoing roads regressed; requiring every real junction to be
    covered catches a partial regression.
    """
    tree = ET.parse(str(nishishinjuku_xodr))
    real = _real_junction_ids(tree)
    assert real, "fixture is expected to contain real junctions"

    referenced = {jid for _road, jid in _roads_with_real_junction_predecessor(tree)}
    missing = sorted(real - referenced, key=int)
    assert missing == [], (
        f"real junctions with no outgoing road linking back as predecessor: "
        f"{missing}"
    )


def test_lane_predecessor_links_restored_for_junction_outgoing_roads(
    nishishinjuku_xodr,
):
    """Running the pass before set_all_lane_links must restore lane links.

    ``_set_single_lane_links`` skips lane-level ``<predecessor>`` creation
    when the road has no predecessor link, so a fix that set only the
    road-level link would leave the lanes unlinked. Every junction-outgoing
    road's driving lanes must therefore carry a lane ``<predecessor>``.
    """
    tree = ET.parse(str(nishishinjuku_xodr))

    without_lane_link = []
    for road, _jid in _roads_with_real_junction_predecessor(tree):
        driving_lanes = [
            lane
            for lane in road.iterfind(".//lane[@type]")
            if lane.get("type") == "driving"
        ]
        if driving_lanes and not any(
            lane.find("link/predecessor") is not None for lane in driving_lanes
        ):
            without_lane_link.append(road.get("id"))

    assert without_lane_link == [], (
        f"junction-outgoing roads whose driving lanes lack a lane-level "
        f"<predecessor>: {without_lane_link}"
    )
