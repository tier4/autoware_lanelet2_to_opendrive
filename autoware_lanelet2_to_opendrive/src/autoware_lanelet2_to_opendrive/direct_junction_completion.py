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
"""

from __future__ import annotations

import logging
from typing import Dict, List, Set, Tuple

import lanelet2

from autoware_lanelet2_to_opendrive.divergence import (
    _lane_anchor_xyz,
    _make_zero_length_connecting_road,
)
from autoware_lanelet2_to_opendrive.opendrive.enums import (
    ContactPoint,
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


def complete_direct_junction_lanelinks(
    *,
    lanelet_map: lanelet2.core.LaneletMap,
    routing_graph,
    all_roads: List[Road],
    junctions: List[Junction],
    lanelet_to_road_id: Dict[int, int],
    junction_lanelet_ids: Set[int],
    starting_road_id: int,
    traffic_rule: TrafficRule,
    min_segment_length: float,
) -> Tuple[List[Road], int]:
    """Materialise synthetic connecting roads for missed direct transitions.

    For each junction J, examines every (incoming road, lane) pair that
    is junction-bound but absent from any ``<laneLink>`` of J. When the
    incoming lane's source lanelet has a direct external follower
    (a lanelet outside any junction footprint, on a different road from
    the incoming and not on any of J's own connecting roads), synthesise
    a zero-length connecting road and append a ``<connection>`` row to
    ``J.connections``. The incoming road carries
    ``<successor elementType="junction" elementId="J">``, so any of its
    driving lanes that has no ``<laneLink>`` would otherwise have no
    forward path through the junction.

    Args:
        lanelet_map: source map for lanelet lookup.
        routing_graph: prebuilt vehicle ``RoutingGraph``.
        all_roads: regular + connecting + previously-synthesised roads.
        junctions: junctions whose connections have already been built.
        lanelet_to_road_id: lanelet id -> road id for every mapped lanelet.
        junction_lanelet_ids: ids of lanelets that live inside any
            junction (any ``turn_direction`` lanelet). Followers in this
            set are skipped — the normal pipeline already covers them.
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

    for junction in junctions:
        if not junction.connections:
            continue

        incoming_road_ids: Set[int] = {c.incoming_road for c in junction.connections}
        connecting_road_ids: Set[int] = {
            c.connecting_road for c in junction.connections
        }
        covered: Set[Tuple[int, int]] = {
            (c.incoming_road, ll.from_lane)
            for c in junction.connections
            for ll in c.lane_links
        }

        for in_rid in sorted(incoming_road_ids):
            in_road = road_by_id.get(in_rid)
            if in_road is None:
                continue
            lane_to_ll = road_lane_to_lanelet.get(in_rid, {})
            for lane_id, ll_id in sorted(lane_to_ll.items()):
                if (in_rid, lane_id) in covered:
                    continue
                if not lanelet_map.laneletLayer.exists(ll_id):
                    continue
                ll = lanelet_map.laneletLayer.get(ll_id)

                follower_match = None
                for follower in routing_graph.following(ll):
                    if follower.id in junction_lanelet_ids:
                        # A real connector exists for this follower path —
                        # the normal pipeline handles it.
                        continue
                    out_rid = lanelet_to_road_id.get(follower.id)
                    if out_rid is None:
                        continue
                    if out_rid == in_rid:
                        # Routing self-loop on the same road is not a
                        # junction transition; skip.
                        continue
                    if out_rid in connecting_road_ids:
                        # The follower lives on one of this junction's
                        # own connecting roads — that path is the normal
                        # connector flow, handled elsewhere.
                        continue
                    out_lane = road_lanelet_to_lane.get(out_rid, {}).get(follower.id)
                    if out_lane is None:
                        continue
                    follower_match = (out_rid, out_lane)
                    break
                if follower_match is None:
                    continue

                out_rid, out_lane = follower_match
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
                )
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
                covered.add((in_rid, lane_id))
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
