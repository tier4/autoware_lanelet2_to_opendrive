"""Non-vehicle lanes get their ``<link>`` from shared boundary geometry.

Lane links are resolved through a ``Participants.Vehicle`` routing graph.
``road_shoulder`` / ``pedestrian_lane`` / ``bicycle_lane`` / ``walkway``
lanelets are not vertices of that graph, so ``routing_graph.previous`` and
``routing_graph.following`` return empty for them and their OpenDRIVE lanes
were emitted with no ``<link>`` at all -- measured as 11 of 11 non-vehicle
lanes on nishishinjuku and 4,405 of 4,405 on odaiba.

``build_boundary_longitudinal_adjacency`` supplies the succession those
lanelets do have, taken from the boundary geometry the Lanelet2 map
specification guarantees: a lanelet's end cross-section *is* its
successor's start cross-section.

These tests pin both halves: the adjacency function's rules (including that
it never touches a vehicle lanelet, which is what keeps vehicle routing
unchanged) and the emitted ``<link>`` end to end.
"""

from __future__ import annotations

import contextlib
import io
from typing import List, Optional, Tuple

import lanelet2
import pytest

from autoware_lanelet2_to_opendrive.conversion_config import (
    ConversionConfig,
    OriginSpec,
    ParamPoly3Config,
)
from autoware_lanelet2_to_opendrive.main import convert_lanelet2_to_opendrive
from autoware_lanelet2_to_opendrive.util import (
    build_boundary_longitudinal_adjacency,
)


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------


def _point(x: float, y: float) -> lanelet2.core.Point3d:
    return lanelet2.core.Point3d(lanelet2.core.getId(), x, y, 0.0)


def _chain(
    subtypes: List[str],
    width: float = 2.0,
    segment_length: float = 10.0,
    share_cross_section: bool = True,
    x0: float = 0.0,
) -> Tuple[lanelet2.core.LaneletMap, List[lanelet2.core.Lanelet]]:
    """Build a head-to-tail chain of lanelets along +x.

    Consecutive lanelets share their cross-section *points* -- the same
    ``Point3d`` instances -- which is the succession signal the Lanelet2
    map specification guarantees.  With ``share_cross_section=False`` each
    lanelet gets its own coincident-but-distinct points, which is what a
    map with duplicated geometry looks like and must NOT be reported.
    """
    lanelet_map = lanelet2.core.LaneletMap()
    lanelets: List[lanelet2.core.Lanelet] = []

    left_start, right_start = _point(x0, 0.0), _point(x0, -width)
    for index, subtype in enumerate(subtypes):
        x_end = x0 + (index + 1) * segment_length
        left_end, right_end = _point(x_end, 0.0), _point(x_end, -width)
        left = lanelet2.core.LineString3d(lanelet2.core.getId(), [left_start, left_end])
        right = lanelet2.core.LineString3d(
            lanelet2.core.getId(), [right_start, right_end]
        )
        lanelet = lanelet2.core.Lanelet(lanelet2.core.getId(), left, right)
        lanelet.attributes["subtype"] = subtype
        lanelet_map.add(lanelet)
        lanelets.append(lanelet)

        if share_cross_section:
            left_start, right_start = left_end, right_end
        else:
            left_start, right_start = _point(x_end, 0.0), _point(x_end, -width)

    return lanelet_map, lanelets


def _successions(lanelets: List[lanelet2.core.Lanelet]) -> List[Tuple[int, int]]:
    _, following_of = build_boundary_longitudinal_adjacency(lanelets)
    return sorted((a, b) for a, others in following_of.items() for b in sorted(others))


def _convert(lanelet_map: lanelet2.core.LaneletMap):
    config = ConversionConfig(
        origin=OriginSpec(mgrs_code="54SUE"),
        parampoly3=ParamPoly3Config(enabled=False),
    )
    with (
        contextlib.redirect_stdout(io.StringIO()),
        contextlib.redirect_stderr(io.StringIO()),
    ):
        opendrive, *_ = convert_lanelet2_to_opendrive(lanelet_map, config)
    return opendrive.to_xml()


def _lane_link_ids(root, lane_type: str) -> List[Tuple[Optional[str], Optional[str]]]:
    """``(predecessor id, successor id)`` for every lane of ``lane_type``."""
    out = []
    for lane in root.iter("lane"):
        if lane.get("type") != lane_type:
            continue
        link = lane.find("link")
        pred = link.find("predecessor") if link is not None else None
        succ = link.find("successor") if link is not None else None
        out.append(
            (
                pred.get("id") if pred is not None else None,
                succ.get("id") if succ is not None else None,
            )
        )
    return out


# --------------------------------------------------------------------------
# the adjacency function
# --------------------------------------------------------------------------


@pytest.mark.parametrize(
    "subtype",
    ["road_shoulder", "pedestrian_lane", "bicycle_lane", "walkway"],
)
def test_consecutive_non_vehicle_lanelets_are_a_succession(subtype: str) -> None:
    """The headline case: a shared cross-section is a succession."""
    _, lanelets = _chain([subtype, subtype])

    assert _successions(lanelets) == [(lanelets[0].id, lanelets[1].id)]


def test_succession_is_reported_on_both_sides() -> None:
    """``previous_of`` mirrors ``following_of`` exactly."""
    _, lanelets = _chain(["road_shoulder", "road_shoulder"])

    previous_of, following_of = build_boundary_longitudinal_adjacency(lanelets)

    assert following_of == {lanelets[0].id: {lanelets[1].id}}
    assert previous_of == {lanelets[1].id: {lanelets[0].id}}


def test_a_longer_chain_links_each_consecutive_pair_only() -> None:
    """Succession is immediate, not transitive."""
    _, lanelets = _chain(["pedestrian_lane"] * 4)

    assert _successions(lanelets) == [
        (lanelets[0].id, lanelets[1].id),
        (lanelets[1].id, lanelets[2].id),
        (lanelets[2].id, lanelets[3].id),
    ]


def test_vehicle_lanelets_are_never_an_endpoint() -> None:
    """Two consecutive road lanelets produce no edge.

    This is the guard that keeps the pass from changing vehicle routing:
    succession for ``road`` / ``highway`` stays with the routing graph.
    """
    _, lanelets = _chain(["road", "road"])

    assert _successions(lanelets) == []


@pytest.mark.parametrize(
    "subtypes",
    [["road", "road_shoulder"], ["road_shoulder", "road"]],
)
def test_a_mixed_pair_produces_no_edge(subtypes: List[str]) -> None:
    """Both sides must be non-vehicle; a road lanelet disqualifies the pair."""
    _, lanelets = _chain(subtypes)

    assert _successions(lanelets) == []


def test_merely_coincident_geometry_is_not_a_succession() -> None:
    """Distinct points at the same coordinates are not a shared cross-section.

    Walkways in the reference maps look like this against the roads beside
    them -- close, but separate geometry.  Guessing a link from proximity
    is what this function deliberately does not do.
    """
    _, lanelets = _chain(["road_shoulder", "road_shoulder"], share_cross_section=False)

    assert _successions(lanelets) == []


def test_lanelets_separated_by_a_gap_produce_no_edge() -> None:
    """A 50 m gap between the two lanelets means no succession.

    The first ends at x = 10, the second begins at x = 60.
    """
    _, first = _chain(["road_shoulder"])
    _, second = _chain(["road_shoulder"], x0=60.0)

    assert _successions(first + second) == []


# --------------------------------------------------------------------------
# end to end: the emitted <link>
# --------------------------------------------------------------------------


def _road_with_shoulder_chain() -> lanelet2.core.LaneletMap:
    """Two consecutive roads, each with a shoulder sharing its right bound.

    Laid out so that all three relationships the emitter needs are present:

    * each shoulder **shares the linestring object** of the road beside it,
      so ``attach_boundary_adjacent_lanelets`` folds it into that road and
      the shoulder becomes a lane rather than a standalone road;
    * the two roads share their cross-section points, so the vehicle
      routing graph links road 1 to road 2 and the emitted roads carry the
      road-level ``<link>`` that a lane link is read against (issue #202);
    * the two shoulders share their cross-section points, which is the
      succession ``build_boundary_longitudinal_adjacency`` reads.

    Without the road-level link the lane link is suppressed by design, so a
    test built from bare shoulders would fail even with the fix in place.
    """
    lanelet_map = lanelet2.core.LaneletMap()

    # Three longitudinal rails: road's left edge, the shared road/shoulder
    # edge, and the shoulder's outer edge.  Points at x = 0, 10, 20 are
    # created once and reused so consecutive segments share them.
    rails = {y: [_point(x, y) for x in (0.0, 10.0, 20.0)] for y in (0.0, -3.5, -5.5)}

    for segment in (0, 1):
        road_left = lanelet2.core.LineString3d(
            lanelet2.core.getId(), rails[0.0][segment : segment + 2]
        )
        shared = lanelet2.core.LineString3d(
            lanelet2.core.getId(), rails[-3.5][segment : segment + 2]
        )
        shoulder_right = lanelet2.core.LineString3d(
            lanelet2.core.getId(), rails[-5.5][segment : segment + 2]
        )

        road = lanelet2.core.Lanelet(lanelet2.core.getId(), road_left, shared)
        road.attributes["subtype"] = "road"
        lanelet_map.add(road)

        # The SAME linestring object is the shoulder's left bound: that
        # identity, not mere coincidence, is the attachment signal.
        shoulder = lanelet2.core.Lanelet(lanelet2.core.getId(), shared, shoulder_right)
        shoulder.attributes["subtype"] = "road_shoulder"
        lanelet_map.add(shoulder)

    return lanelet_map


def test_consecutive_shoulders_emit_a_lane_link() -> None:
    """A shoulder lane names its successor lane in the next road.

    Fails without the change: the vehicle routing graph has no vertex for a
    shoulder lanelet, so ``routing_graph.following`` returns empty and the
    lane is emitted with no ``<link>`` at all, even though the road it
    belongs to is linked.
    """
    root = _convert(_road_with_shoulder_chain())

    shoulder_links = _lane_link_ids(root, "shoulder")
    assert shoulder_links, "no shoulder lane was emitted at all"
    assert any(
        pred is not None or succ is not None for pred, succ in shoulder_links
    ), f"every shoulder lane was emitted without a <link>: {shoulder_links}"


def test_a_shoulder_never_links_to_a_driving_lane() -> None:
    """A non-vehicle lane's link target must be a lane of the same type.

    ``_find_closest_lane`` silently remaps a lane link whose target id does
    not exist in the target road to the nearest existing id.  Lane ids of
    non-vehicle lanes differ between roads far more often than driving ids
    do, so that remap can land a shoulder link on a *driving* lane -- a
    wrong link that looks right, which is worse than a missing one.
    """
    root = _convert(_road_with_shoulder_chain())

    roads = {road.get("id"): road for road in root.findall("road")}

    def _lane_type(road_id: str, lane_id: str) -> Optional[str]:
        road = roads.get(road_id)
        if road is None:
            return None
        for lane in road.iter("lane"):
            if lane.get("id") == lane_id:
                return lane.get("type")
        return None

    offenders = []
    for road in root.findall("road"):
        link = road.find("link")
        for lane in road.iter("lane"):
            if lane.get("type") != "shoulder":
                continue
            lane_link = lane.find("link")
            if lane_link is None or link is None:
                continue
            for tag in ("predecessor", "successor"):
                target = lane_link.find(tag)
                road_elem = link.find(tag)
                if target is None or road_elem is None:
                    continue
                if road_elem.get("elementType") != "road":
                    continue
                target_type = _lane_type(road_elem.get("elementId"), target.get("id"))
                if target_type is not None and target_type != "shoulder":
                    offenders.append((road.get("id"), lane.get("id"), tag, target_type))

    assert (
        not offenders
    ), f"a shoulder lane links to a lane of another type: {offenders}"


def test_a_suppressed_link_leaves_no_peer_link_across_the_boundary() -> None:
    """Where a non-vehicle lane has no link, its peer must have none either.

    ``_check_two_lane_sections`` examines a road boundary from **both**
    sides, so declining to write A's link does not remove a finding if B's
    lane across that boundary still carries one -- the checker flags B.
    Suppressing one side is only safe when the other side is also
    link-free, so this asserts the symmetry rather than assuming it.
    """
    root = _convert(_road_with_shoulder_chain())

    roads = {road.get("id"): road for road in root.findall("road")}

    def _lane(road_id, lane_id):
        road = roads.get(road_id)
        if road is None:
            return None
        for lane in road.iter("lane"):
            if lane.get("id") == lane_id:
                return lane
        return None

    def _link_id(lane, tag):
        if lane is None:
            return None
        link = lane.find("link")
        if link is None:
            return None
        e = link.find(tag)
        return e.get("id") if e is not None else None

    asymmetric = []
    for road in root.findall("road"):
        road_link = road.find("link")
        if road_link is None:
            continue
        for lane in road.iter("lane"):
            if lane.get("type") not in ("shoulder", "sidewalk", "biking"):
                continue
            for tag, back in (
                ("successor", "predecessor"),
                ("predecessor", "successor"),
            ):
                if _link_id(lane, tag) is not None:
                    continue  # this side is linked; not a suppression case
                elem = road_link.find(tag)
                if elem is None or elem.get("elementType") != "road":
                    continue
                # Same lane id is the peer this pass would have linked.
                peer = _lane(elem.get("elementId"), lane.get("id"))
                if peer is None or peer.get("type") != lane.get("type"):
                    continue
                if _link_id(peer, back) is not None:
                    asymmetric.append(
                        (road.get("id"), lane.get("id"), tag, elem.get("elementId"))
                    )

    assert not asymmetric, (
        "a non-vehicle lane has no link while its peer across the boundary "
        f"does; suppression must be symmetric: {asymmetric}"
    )


def test_the_driving_lanes_beside_them_are_still_linked() -> None:
    """Control: the vehicle side of the same map is unaffected.

    If this fails alongside the test above, the change has reached vehicle
    routing, which it must not.
    """
    root = _convert(_road_with_shoulder_chain())

    driving_links = _lane_link_ids(root, "driving")
    assert driving_links, "no driving lane was emitted at all"
    assert any(
        pred is not None or succ is not None for pred, succ in driving_links
    ), f"the driving lanes lost their links: {driving_links}"
