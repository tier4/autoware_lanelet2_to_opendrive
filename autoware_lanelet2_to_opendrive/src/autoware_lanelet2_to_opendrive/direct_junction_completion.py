"""Synthesise zero-length connecting roads for direct junction transitions.

``Junction.build_connections_from_roads`` enumerates only the
junction-internal lanelets tagged ``turn_direction``. When the source
OSM connects an *external* lanelet directly to another external lanelet
without a connector lanelet in between, no junction-internal lanelet
exists to attach a ``<laneLink>`` to, and that lane is silently dropped
from the ``<connection>`` table. esmini/odrviewer routes the vehicle
into the junction via the road's *other* lanes' connections, but a car
on the gap lane finds no path forward and stalls
(``--stop_at_end_of_road``).

This pass walks each junction after its turn-direction connections are
built, finds every ``(incoming road lane, outgoing road lane)`` pair the
vehicle routing graph implies but the junction does not already cover,
and materialises a zero-length connecting road plus ``<connection>``
record for it — reusing the primitives the divergence/merge synthesis
pass (#291) uses for its own zero-length connectors.

Coverage is tracked per *directed lane edge*, not per incoming lane. A
lanelet whose followers mix a ``turn_direction`` connector with a plain
regular-road lanelet reaches both through the same junction: the junction
owns the incoming road's single road-level ``<successor>`` slot, so the
plain-road branch can only survive as another ``<connection>`` of that
same junction. Gating on the incoming lane alone declared such a lane
"already covered" as soon as one branch had a connector and silently
dropped the rest of the fork.
"""

from __future__ import annotations

import logging
from typing import Dict, List, Set, Tuple

import lanelet2

from autoware_lanelet2_to_opendrive.config import DEFAULT_CONFIG
from autoware_lanelet2_to_opendrive.divergence import (
    _lane_anchor_xyz,
    _lane_width_at,
    _lane_width_at_end,
    _make_zero_length_connecting_road,
    _taper_stub_width,
)
from autoware_lanelet2_to_opendrive.opendrive.enums import (
    ContactPoint,
    ElementType,
    TrafficRule,
)
from autoware_lanelet2_to_opendrive.opendrive.junction import (
    Connection,
    Junction,
)
from autoware_lanelet2_to_opendrive.opendrive.junction import (
    LaneLink as JunctionLaneLink,
)
from autoware_lanelet2_to_opendrive.opendrive.road import (
    Road,
    _evaluate_planview_endpoint_with_heading,
)

log = logging.getLogger(__name__)


def _connection_target_lane(
    connecting_road: Road | None,
    connecting_lane_id: int,
) -> Tuple[int, int] | None:
    """Return the ``(road_id, lane_id)`` a junction ``<connection>`` leads to.

    ``connecting_lane_id`` is the ``to`` attribute of a ``<laneLink>``, i.e.
    a lane of ``connecting_road``. The pair is read from the connecting
    road's own road-level ``<successor>`` and that lane's ``<successor>``.

    Returns ``None`` when either link is still unset. Real
    ``turn_direction`` connectors are in that state here — their lane links
    are filled in by :meth:`Road.set_all_lane_links`, which runs after this
    pass — but they are already excluded from the candidate followers by
    the junction-membership filter, so an unresolved entry simply
    contributes no coverage.
    """
    if connecting_road is None or connecting_road.link is None:
        return None
    successor = connecting_road.link.successor
    if successor is None or successor.element_type != ElementType.ROAD:
        return None
    lanes = getattr(connecting_road, "lanes", None)
    if lanes is None:
        return None
    for lane_section in lanes.lane_sections:
        lane = lane_section.left_lanes.get(
            connecting_lane_id
        ) or lane_section.right_lanes.get(connecting_lane_id)
        if lane is not None and lane.successor is not None:
            return (successor.element_id, lane.successor.id)
    return None


def complete_direct_junction_lanelinks(
    *,
    lanelet_map: lanelet2.core.LaneletMap,
    routing_graph,
    all_roads: List[Road],
    junctions: List[Junction],
    lanelet_to_road_id: Dict[int, int],
    starting_road_id: int,
    traffic_rule: TrafficRule,
    min_segment_length: float,
) -> Tuple[List[Road], int]:
    """Materialise synthetic connecting roads for missed direct transitions.

    For each junction J, examines every directed lane edge the routing
    graph reports out of an incoming road of J that no ``<laneLink>`` of J
    already covers. When the incoming lane's source lanelet has a follower
    on a road that does not live inside J, synthesise a zero-length
    connecting road and append a ``<connection>`` row to ``J.connections``.
    The incoming road carries
    ``<successor elementType="junction" elementId="J">``, so a branch of
    the fork that J does not list has no forward path at all — OpenDRIVE
    allows only one road-level successor per road, and a lane link may not
    name a road other than it.

    Coverage is per directed lane edge, so a lane that already reaches one
    follower through a ``turn_direction`` connector still gets a connector
    for each of its remaining followers.

    Args:
        lanelet_map: source map for lanelet lookup.
        routing_graph: prebuilt vehicle ``RoutingGraph``.
        all_roads: regular + connecting + previously-synthesised roads.
        junctions: junctions whose connections have already been built.
        lanelet_to_road_id: lanelet id -> road id for every mapped lanelet.
        starting_road_id: first id to assign to a synthetic connector.
        traffic_rule: RHT / LHT — sets the synthetic lane's id.
        min_segment_length: minimum planView length to floor zero-length
            geometry at; downstream consumers reject zero-length roads.

    Returns:
        ``(new_roads, next_id)``. ``new_roads`` is the list of
        synthesised connecting roads; ``next_id`` is the id of the first
        slot left un-allocated.
    """
    road_by_id: Dict[int, Road] = {r.id: r for r in all_roads}
    road_lanelet_to_lane: Dict[int, Dict[int, int]] = {
        r.id: r.get_lanelet_to_lane_mapping() for r in all_roads
    }
    road_lane_to_lanelet: Dict[int, Dict[int, int]] = {
        rid: {lane: ll for ll, lane in mapping.items()}
        for rid, mapping in road_lanelet_to_lane.items()
    }

    new_roads: List[Road] = []
    next_id = starting_road_id
    connector_lane_id = 1 if traffic_rule == TrafficRule.LHT else -1

    # ``(incoming road, incoming lane, outgoing road, outgoing lane)`` edges
    # the junction table already carries. Resolved through each connection's
    # connecting road rather than through the incoming lane alone, so one
    # covered branch of a fork no longer masks the others.
    #
    # The set spans every junction rather than being rebuilt per junction: an
    # incoming road may appear in two junction tables, but only the junction
    # its road-level ``<successor>`` names is reachable, so a second copy of
    # an already-realised edge would be dead weight.
    covered: Set[Tuple[int, int, int, int]] = set()
    for junction in junctions:
        for conn in junction.connections:
            conn_road = road_by_id.get(conn.connecting_road)
            for lane_link in conn.lane_links:
                target = _connection_target_lane(conn_road, lane_link.to_lane)
                if target is not None:
                    covered.add((conn.incoming_road, lane_link.from_lane, *target))

    for junction in junctions:
        if not junction.connections:
            continue

        incoming_road_ids: Set[int] = {c.incoming_road for c in junction.connections}
        # Roads that physically live inside this junction. A follower on one
        # of them is reached through the normal connector flow, which owns
        # both the ``<connection>`` row and the connecting road itself.
        member_road_ids: Set[int] = {
            r.id for r in all_roads if r.junction == junction.id
        }

        for in_rid in sorted(incoming_road_ids):
            in_road = road_by_id.get(in_rid)
            if in_road is None:
                continue
            lane_to_ll = road_lane_to_lanelet.get(in_rid, {})
            for lane_id, ll_id in sorted(lane_to_ll.items()):
                if not lanelet_map.laneletLayer.exists(ll_id):
                    continue
                ll = lanelet_map.laneletLayer.get(ll_id)

                # ``routing_graph.following`` ordering is not specified
                # by lanelet2; collect the eligible followers and walk them
                # in sorted order so the emitted XML is stable across runs
                # and platforms. Every one of them gets its own connector:
                # a lane carries exactly one ``<link><successor>``, so a
                # fork survives only when each branch owns a connector.
                eligible_followers: List[Tuple[int, int, int]] = []
                for follower in routing_graph.following(ll):
                    out_rid = lanelet_to_road_id.get(follower.id)
                    if out_rid is None:
                        continue
                    if out_rid == in_rid:
                        # Routing self-loop on the same road is not a
                        # junction transition; skip.
                        continue
                    if out_rid in member_road_ids:
                        # The follower lives inside this junction — that
                        # path is the normal connector flow, handled
                        # elsewhere.
                        continue
                    out_lane = road_lanelet_to_lane.get(out_rid, {}).get(follower.id)
                    if out_lane is None:
                        continue
                    if (in_rid, lane_id, out_rid, out_lane) in covered:
                        continue
                    eligible_followers.append((out_rid, out_lane, follower.id))
                # Sort by (road_id, lane_id, lanelet_id) — the lanelet id
                # is the final tiebreaker for the rare case of two
                # lanelets sharing a (road, lane) pair.
                eligible_followers.sort()
                for out_rid, out_lane, _follower_lanelet_id in eligible_followers:
                    out_road = road_by_id.get(out_rid)
                    if out_road is None:
                        continue

                    start_xyz = _lane_anchor_xyz(
                        in_road, lane_id, at_start=False, traffic_rule=traffic_rule
                    )
                    if start_xyz is None:
                        start_xyz = in_road.reference_end_xyz
                    end_xyz = _lane_anchor_xyz(
                        out_road, out_lane, at_start=True, traffic_rule=traffic_rule
                    )
                    if end_xyz is None:
                        end_xyz = out_road.reference_start_xyz
                    if start_xyz is None or end_xyz is None:
                        log.warning(
                            "direct-junction completion: junction %d road %d "
                            "lane %d lacks endpoint geometry; skipped",
                            junction.id,
                            in_rid,
                            lane_id,
                        )
                        continue

                    fallback_heading = 0.0
                    if in_road.plan_view is not None:
                        endpoint_with_h = _evaluate_planview_endpoint_with_heading(
                            in_road.plan_view, at_start=False
                        )
                        if endpoint_with_h is not None:
                            fallback_heading = endpoint_with_h[2]

                    in_width = _lane_width_at_end(in_road, lane_id)
                    out_width = _lane_width_at(out_road, out_lane, at_start=True)

                    new_road = _make_zero_length_connecting_road(
                        road_id=next_id,
                        junction_id=junction.id,
                        incoming_road_id=in_rid,
                        outgoing_road_id=out_rid,
                        incoming_contact=ContactPoint.END,
                        outgoing_contact=ContactPoint.START,
                        start_xyz=start_xyz,
                        end_xyz=end_xyz,
                        min_segment_length=min_segment_length,
                        traffic_rule=traffic_rule,
                        from_lane=lane_id,
                        to_lane=out_lane,
                        fallback_heading=fallback_heading,
                        lane_width=(
                            in_width
                            if in_width is not None
                            else DEFAULT_CONFIG.geometry.divergence_default_lane_width
                        ),
                    )
                    # The connector is anchored on the incoming lane's inner
                    # border and oriented by that road's end tangent, so the
                    # taper is what puts its *outer* border on the neighbours'
                    # outer borders at both contacts -- the second of the two
                    # matches ``contact_point_no_horizontal_gaps`` requires.
                    _taper_stub_width(new_road, in_width, out_width)
                    new_roads.append(new_road)

                    junction.connections.append(
                        Connection(
                            id=len(junction.connections),
                            incoming_road=in_rid,
                            connecting_road=next_id,
                            contact_point=ContactPoint.START,
                            lane_links=[
                                JunctionLaneLink(
                                    from_lane=lane_id,
                                    to_lane=connector_lane_id,
                                )
                            ],
                        )
                    )
                    covered.add((in_rid, lane_id, out_rid, out_lane))
                    next_id += 1
                    log.debug(
                        "direct-junction completion: junction %d road %d lane %d "
                        "-> road %d lane %d via synthetic connector %d",
                        junction.id,
                        in_rid,
                        lane_id,
                        out_rid,
                        out_lane,
                        new_road.id,
                    )

    if new_roads:
        log.info(
            "direct-junction completion: synthesised %d connecting road(s) "
            "for direct external-to-external transitions",
            len(new_roads),
        )
    return new_roads, next_id
