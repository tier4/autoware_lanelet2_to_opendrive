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
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path
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


def _sid(element, name: str) -> str:
    """An XML attribute as a plain ``str`` (absent becomes ``""``)."""
    value = element.get(name) if element is not None else None
    return "" if value is None else str(value)


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


NISHISHINJUKU_OSM = (Path(__file__).parent / "data" / "nishishinjuku.osm").resolve()


def _convert_nishishinjuku(tmp_path: Path) -> Path:
    """Run the CLI on the Shinjuku fixture and return the .xodr path."""
    out = tmp_path / "nishishinjuku.xodr"
    subprocess.run(
        [
            "uv",
            "run",
            "convert",
            "map=nishishinjuku",
            "target=default",
            f"input_map_path={NISHISHINJUKU_OSM}",
            f"output_map_path={out}",
        ],
        check=True,
    )
    return out


def test_no_non_vehicle_lane_link_survives_at_a_junction_end(tmp_path: Path) -> None:
    """LINT: a non-vehicle lane carries no ``<link>`` at a junction end.

    Deliberately **not** a crash guard. CARLA never reads a lane's own
    ``<link>`` id at a junction-typed road end -- ``GetJunctionLanes``
    keys on ``link.from == lane_id`` and resolves ``link.to`` through the
    ``<connection>`` table instead -- so a regression here is a
    conformance defect, not a null dereference. The crash-carrying
    populations are pinned by
    ``test_every_lane_link_id_resolves_on_the_target_road``.

    OpenDRIVE says the ``<link>`` element shall be omitted where a lane
    starts or ends in a junction, and
    ``road.lane.link.lanes_across_lane_sections`` skips those boundaries --
    so such a link is silently wrong rather than a finding, and the ASAM
    totals cannot catch it.

    It arises because ``set_all_lane_links`` writes a lane link while the
    road link at that end is still road-typed, and a later pass re-points
    that end at a junction without clearing the lane link.  Measured on
    nishishinjuku: 1 such shoulder link ships without
    ``_drop_asymmetric_non_vehicle_lane_links``, 0 with it.

    Needs the real map: the property depends on a junction, which the
    synthetic fixtures above deliberately do not have.
    """
    root = ET.parse(_convert_nishishinjuku(tmp_path)).getroot()

    offenders = []
    for road in root.iter("road"):
        road_link = road.find("link")
        if road_link is None:
            continue
        junction_end = {}
        for tag in ("predecessor", "successor"):
            end = road_link.find(tag)
            junction_end[tag] = (
                end is not None and _sid(end, "elementType") == "junction"
            )
        for lane in road.iter("lane"):
            if lane.get("type") not in ("sidewalk", "shoulder", "biking"):
                continue
            lane_link = lane.find("link")
            if lane_link is None:
                continue
            for tag in ("predecessor", "successor"):
                if lane_link.find(tag) is not None and junction_end[tag]:
                    offenders.append((road.get("id"), lane.get("id"), tag))

    assert not offenders, (
        "non-vehicle lane <link> survives at a junction-typed road end, "
        f"where the standard says to omit it: {offenders}"
    )


def test_every_lane_link_id_resolves_on_the_target_road(tmp_path: Path) -> None:
    """CRASH GUARD: a lane ``<link>`` must name a lane that exists.

    Covers the two populations CARLA actually reads: lane links at
    **road-typed** road ends, and the junction ``<connection><laneLink>``
    table.  Links at *junction-typed* road ends are deliberately skipped --
    CARLA's ``GetJunctionLanes`` keys on ``link.from == lane_id`` and
    resolves ``link.to``, so a lane's own ``<link>`` id is never consulted
    there.  That population is a lint, pinned separately by
    ``test_no_non_vehicle_lane_link_survives_at_a_junction_end``.

    Where CARLA does read the id it dereferences the target lane behind
    only a ``DEBUG_ASSERT``, so an id that does not exist is a **crash** in
    a release build, not a warning -- and the ASAM bundle does not check it.

    This is the same shape as the junction-end property above: a real
    defect that no checker can see, so it has to be asserted directly.

    Grouping non-vehicle lanelets onto a neighbouring road renumbers lanes,
    and ``_find_closest_lane`` remaps a link whose target id is missing
    rather than dropping it, so this is exactly the area where a dangling
    id would arise. Checked over every lane, not just the non-vehicle ones,
    because a renumbering defect would not respect that boundary.
    """
    root = ET.parse(_convert_nishishinjuku(tmp_path)).getroot()

    lanes_of: dict[str, set[str]] = {}
    ends_of: dict[str, dict[str, tuple[str, str] | None]] = {}
    for road in root.iter("road"):
        rid = _sid(road, "id")
        lanes_of[rid] = {_sid(lane, "id") for lane in road.iter("lane")}
        road_link = road.find("link")
        ends: dict[str, Optional[Tuple[str, str]]] = {}
        for tag in ("predecessor", "successor"):
            e = road_link.find(tag) if road_link is not None else None
            ends[tag] = (
                None if e is None else (_sid(e, "elementType"), _sid(e, "elementId"))
            )
        ends_of[rid] = ends

    dangling = []
    for road in root.iter("road"):
        rid = _sid(road, "id")
        for lane in road.iter("lane"):
            lane_link = lane.find("link")
            if lane_link is None:
                continue
            for tag in ("predecessor", "successor"):
                e = lane_link.find(tag)
                if e is None:
                    continue
                end = ends_of[rid][tag]
                # A junction-typed end is resolved through the junction's
                # <connection> table, checked separately below.
                if end is None or end[0] == "junction":
                    continue
                if _sid(e, "id") not in lanes_of.get(end[1], set()):
                    dangling.append((rid, _sid(lane, "id"), tag, _sid(e, "id"), end[1]))

    assert not dangling, (
        "lane <link> names a lane that does not exist on the target road; "
        f"CARLA dereferences this without a release guard: {dangling}"
    )

    # The junction <connection> table has the same exposure.
    bad_connections = []
    for junction in root.iter("junction"):
        for connection in junction.iter("connection"):
            incoming = _sid(connection, "incomingRoad")
            connecting = _sid(connection, "connectingRoad")
            for lane_link in connection.iter("laneLink"):
                if _sid(lane_link, "from") not in lanes_of.get(incoming, set()):
                    bad_connections.append(
                        (
                            _sid(junction, "id"),
                            incoming,
                            "from",
                            _sid(lane_link, "from"),
                        )
                    )
                if _sid(lane_link, "to") not in lanes_of.get(connecting, set()):
                    bad_connections.append(
                        (_sid(junction, "id"), connecting, "to", _sid(lane_link, "to"))
                    )

    assert (
        not bad_connections
    ), f"junction <laneLink> names a non-existent lane: {bad_connections}"


def _unresolved_lane_links(root) -> Tuple[List[Tuple], List[Tuple]]:
    """``(dangling lane links, dangling junction laneLinks)`` in a tree.

    The predicate behind
    ``test_every_lane_link_id_resolves_on_the_target_road``, factored out so
    the fault-injection test below can prove it actually fires.
    """
    lanes_of: dict[str, set[str]] = {}
    ends_of: dict[str, dict[str, Optional[Tuple[str, str]]]] = {}
    for road in root.iter("road"):
        rid = _sid(road, "id")
        lanes_of[rid] = {_sid(lane, "id") for lane in road.iter("lane")}
        road_link = road.find("link")
        ends: dict[str, Optional[Tuple[str, str]]] = {}
        for tag in ("predecessor", "successor"):
            e = road_link.find(tag) if road_link is not None else None
            ends[tag] = (
                None if e is None else (_sid(e, "elementType"), _sid(e, "elementId"))
            )
        ends_of[rid] = ends

    dangling = []
    for road in root.iter("road"):
        rid = _sid(road, "id")
        for lane in road.iter("lane"):
            lane_link = lane.find("link")
            if lane_link is None:
                continue
            for tag in ("predecessor", "successor"):
                e = lane_link.find(tag)
                if e is None:
                    continue
                end = ends_of[rid][tag]
                if end is None or end[0] == "junction":
                    continue
                if _sid(e, "id") not in lanes_of.get(end[1], set()):
                    dangling.append((rid, _sid(lane, "id"), tag, _sid(e, "id"), end[1]))

    bad_connections = []
    for junction in root.iter("junction"):
        for connection in junction.iter("connection"):
            incoming = _sid(connection, "incomingRoad")
            connecting = _sid(connection, "connectingRoad")
            for lane_link in connection.iter("laneLink"):
                if _sid(lane_link, "from") not in lanes_of.get(incoming, set()):
                    bad_connections.append(
                        (
                            _sid(junction, "id"),
                            incoming,
                            "from",
                            _sid(lane_link, "from"),
                        )
                    )
                if _sid(lane_link, "to") not in lanes_of.get(connecting, set()):
                    bad_connections.append(
                        (_sid(junction, "id"), connecting, "to", _sid(lane_link, "to"))
                    )
    return dangling, bad_connections


def test_the_resolvability_check_actually_fires(tmp_path: Path) -> None:
    """Fault injection: prove the crash guard's predicate is evaluated.

    A zero from an unvalidated checker is indistinguishable from a checker
    that never evaluates its predicate, and this repository has catalogued
    several of the latter.  So inject known-bad ids into a real converted
    file and require the count to rise by **exactly** the number injected --
    not merely to be non-zero, which a checker that flags everything would
    also satisfy.

    Covers both branches: a lane ``<link>`` at a road-typed end, and the
    junction ``<connection><laneLink>`` table.
    """
    root = ET.parse(_convert_nishishinjuku(tmp_path)).getroot()
    before_links, before_connections = _unresolved_lane_links(root)
    assert not before_links and not before_connections, (
        "fixture is not clean to begin with; the injection count below "
        f"would be meaningless: {before_links} {before_connections}"
    )

    bogus = "97"
    injected_links = 0
    for road in root.iter("road"):
        road_link = road.find("link")
        if road_link is None:
            continue
        predecessor = road_link.find("predecessor")
        if predecessor is None or _sid(predecessor, "elementType") != "road":
            continue
        for lane in road.iter("lane"):
            lane_link = lane.find("link")
            target = lane_link.find("predecessor") if lane_link is not None else None
            if target is None:
                continue
            target.set("id", bogus)
            injected_links += 1
            break
        if injected_links >= 2:
            break

    injected_connections = 0
    for junction in root.iter("junction"):
        for connection in junction.iter("connection"):
            for lane_link in connection.iter("laneLink"):
                lane_link.set("to", bogus)
                injected_connections += 1
                break
            break
        break

    assert injected_links == 2 and injected_connections == 1, (
        "the fixture no longer has the shape this injection needs "
        f"({injected_links} lane links, {injected_connections} connections)"
    )

    after_links, after_connections = _unresolved_lane_links(root)
    assert len(after_links) == injected_links, (
        f"expected exactly {injected_links} dangling lane links after "
        f"injection, got {len(after_links)}: {after_links}"
    )
    assert len(after_connections) == injected_connections, (
        f"expected exactly {injected_connections} dangling junction "
        f"laneLinks after injection, got {len(after_connections)}: "
        f"{after_connections}"
    )


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
