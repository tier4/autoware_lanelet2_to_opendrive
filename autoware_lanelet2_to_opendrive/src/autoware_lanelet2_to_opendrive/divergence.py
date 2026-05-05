"""Divergence/merge synthesis pass for issue #291.

When the lanelet routing graph reports more than one distinct regular-road
predecessor or successor for a single regular road, OpenDRIVE requires the
road-level link on that side to point at a junction. This module owns the
detection, sanity gating, and synthesis of those junctions plus their
zero-length connecting roads. The output objects share types with real
junctions so the existing :func:`Road.set_incoming_road_junction_links`
flow uniformly closes both sides.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from enum import Enum
from typing import Dict, Iterable, List, Set, Tuple

from autoware_lanelet2_to_opendrive.config import DEFAULT_CONFIG

from autoware_lanelet2_to_opendrive.opendrive.enums import (
    ContactPoint,
    ElementType,
    LaneType,
    TrafficRule,
)
from autoware_lanelet2_to_opendrive.opendrive.geometry import ParamPoly3, PlanView
from autoware_lanelet2_to_opendrive.opendrive.junction import (
    Connection,
    Junction,
)
from autoware_lanelet2_to_opendrive.opendrive.junction import (
    LaneLink as JunctionLaneLink,
)
from autoware_lanelet2_to_opendrive.opendrive.lane import Lane
from autoware_lanelet2_to_opendrive.opendrive.lane_section import LaneSection
from autoware_lanelet2_to_opendrive.opendrive.lane_sections import Lanes
from autoware_lanelet2_to_opendrive.opendrive.lane_elements import LaneWidth
from autoware_lanelet2_to_opendrive.opendrive.opendrive_dataclass import (
    LaneLink as LaneLevelLaneLink,
)
from autoware_lanelet2_to_opendrive.opendrive.road import (
    Road,
    _evaluate_planview_endpoint_with_heading,
)
from autoware_lanelet2_to_opendrive.opendrive.road_links import (
    Predecessor,
    RoadLink,
    Successor,
)
from autoware_lanelet2_to_opendrive.util import (
    ConnectionDirection,
    find_connecting_lanelet_groups,
)

log = logging.getLogger(__name__)


class DivergenceSide(Enum):
    """Which side of a regular road has multiple candidate roads."""

    PREDECESSOR = "predecessor"
    SUCCESSOR = "successor"


@dataclass(frozen=True)
class DivergenceSite:
    """One regular road's divergence (successor) or merge (predecessor) point.

    Attributes:
        road_id: ID of the regular road whose link is deferred.
        side: which side the multiplicity is on.
        candidate_road_ids: distinct regular-road IDs that the routing graph
            placed in that direction. Always has length >= 2; otherwise the
            site would not be recorded.
    """

    road_id: int
    side: DivergenceSide
    candidate_road_ids: List[int]

    @property
    def is_divergence(self) -> bool:
        """True for 1->N successor splits, False for N->1 predecessor merges."""
        return self.side is DivergenceSide.SUCCESSOR


def collect_divergence_sites(
    deferred_predecessor_candidates: Dict[int, List[int]],
    deferred_successor_candidates: Dict[int, List[int]],
) -> List[DivergenceSite]:
    """Materialise :class:`DivergenceSite` instances from deferred candidate maps.

    The deferred maps come from :meth:`Road.construct_from_lanelet_map`. Only
    entries with two or more candidates produce a site; singletons / empties
    are dropped because they were already linked or have nothing to link.
    Output order is deterministic: predecessor sites first (sorted by road id),
    then successor sites (sorted by road id).
    """

    def _emit(
        side: DivergenceSide, mapping: Dict[int, List[int]]
    ) -> Iterable[DivergenceSite]:
        for road_id in sorted(mapping):
            cands = mapping[road_id]
            if len(cands) >= 2:
                yield DivergenceSite(
                    road_id=road_id,
                    side=side,
                    candidate_road_ids=list(cands),
                )

    return [
        *_emit(DivergenceSide.PREDECESSOR, deferred_predecessor_candidates),
        *_emit(DivergenceSide.SUCCESSOR, deferred_successor_candidates),
    ]


@dataclass(frozen=True)
class SanityGateInputs:
    """Pre-computed inputs to the divergence/merge sanity gate.

    Attributes:
        endpoint_road: ``(x, y, z)`` of the regular road's reference-line
            endpoint on the side under consideration.
        endpoints_candidates: ``candidate_road_id -> (x, y, z)`` of each
            candidate road's mirrored endpoint.
        lane_pairs: list of ``(source_lane_id, candidate_road_id,
            candidate_lane_id)`` triples recovered from the lanelet routing
            graph for the side under consideration.
        all_successor_lanelet_road_ids: set of road IDs that **every**
            successor (or predecessor) lanelet of the source road resolves
            to. Must be a subset of ``set(candidate_road_ids)`` for the
            "group exhaustiveness" check to pass.
    """

    endpoint_road: Tuple[float, float, float]
    endpoints_candidates: Dict[int, Tuple[float, float, float]]
    lane_pairs: List[Tuple[int, int, int]]
    all_successor_lanelet_road_ids: Set[int]


def _distance(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
    return math.sqrt(sum((ai - bi) ** 2 for ai, bi in zip(a, b)))


def sanity_gate_passes(
    site: DivergenceSite,
    inputs: SanityGateInputs,
    endpoint_tolerance: float,
) -> Tuple[bool, str]:
    """Return ``(passed, reason)``.

    The gate fails fast on the first violation; the second-level reason
    string is logged by the caller. Order matches the spec: endpoint
    coincidence, lane uniqueness, group exhaustiveness.
    """
    candidate_set = set(site.candidate_road_ids)

    # 1. Endpoint coincidence.
    for cand_id in site.candidate_road_ids:
        cand_endpoint = inputs.endpoints_candidates.get(cand_id)
        if cand_endpoint is None:
            return False, f"missing endpoint for candidate road {cand_id}"
        if _distance(inputs.endpoint_road, cand_endpoint) > endpoint_tolerance:
            return (
                False,
                f"endpoint distance for candidate {cand_id} exceeds "
                f"{endpoint_tolerance:.3f} m",
            )

    # 2. Lane uniqueness: each (candidate_road, candidate_lane) appears at most once.
    seen_targets: Set[Tuple[int, int]] = set()
    for _src_lane, cand_id, cand_lane in inputs.lane_pairs:
        target = (cand_id, cand_lane)
        if target in seen_targets:
            return False, f"lane collision: two source lanes map to {target}"
        seen_targets.add(target)

    # 3. Group exhaustiveness: every successor lanelet road id must be a candidate.
    orphans = inputs.all_successor_lanelet_road_ids - candidate_set
    if orphans:
        return (
            False,
            f"orphan successor road ids not in candidates: {sorted(orphans)}",
        )

    # 4. Per-candidate lane-pair coverage: every candidate must have at least one
    #    recovered lane pair, otherwise emitting the synthetic junction would
    #    silently drop that branch (#291 review).
    cands_with_pairs = {pair[1] for pair in inputs.lane_pairs}
    missing_cands = candidate_set - cands_with_pairs
    if missing_cands:
        return (
            False,
            f"candidates with no recovered lane pairs: {sorted(missing_cands)}",
        )

    return True, ""


@dataclass(frozen=True)
class SynthesisOutput:
    """Result of synthesising one divergence/merge junction.

    Attributes:
        junction: the new :class:`Junction` (real-junction-compatible).
        connecting_roads: zero-length connecting roads. Their IDs start at
            ``starting_connecting_road_id`` and increment.
        deferred_link_patch: ``(side, source_road_id, junction_id)`` triple
            the caller applies to the source road's road-level link. ``side``
            is ``"predecessor"`` for a merge and ``"successor"`` for a
            divergence.
    """

    junction: Junction
    connecting_roads: List[Road]
    deferred_link_patch: Tuple[str, int, int]


def _make_zero_length_connecting_road(
    road_id: int,
    junction_id: int,
    incoming_road_id: int,
    outgoing_road_id: int,
    incoming_contact: ContactPoint,
    outgoing_contact: ContactPoint,
    start_xyz: Tuple[float, float, float],
    end_xyz: Tuple[float, float, float],
    min_segment_length: float,
    traffic_rule: TrafficRule,
    from_lane: int,
    to_lane: int,
    fallback_heading: float = 0.0,
    lane_width: float = 3.5,
) -> Road:
    """Build a single-segment single-lane connecting road floored at ``min_segment_length``.

    The geometry is a :class:`ParamPoly3` segment whose length is the
    planar distance between ``start_xyz`` and ``end_xyz``, lifted to at
    least ``min_segment_length`` so OpenDRIVE consumers that reject
    zero-length geometry (e.g. CARLA) still accept it.  ParamPoly3 is
    used (rather than :class:`Line`) because downstream samplers in this
    converter — ``road_lanelet_geo_mapping``, ``analyze_xodr``,
    ``objects`` — read ``aU``/``bU``/``aV``/``bV``/etc. directly without
    a geometry-type guard. For a straight segment the coefficients
    reduce to ``u(p) = p``, ``v(p) = 0``.  When ``start_xyz`` and
    ``end_xyz`` coincide the heading falls back to ``fallback_heading``
    (typically the source road's reference-line tangent) so the
    connector aligns with the linked roads instead of pointing along
    the world X axis.  The lane carries lane-level predecessor/successor
    links from ``from_lane`` to ``to_lane`` and a constant ``lane_width``
    so the eventual XML always emits a ``<width>`` element.
    """
    dx = end_xyz[0] - start_xyz[0]
    dy = end_xyz[1] - start_xyz[1]
    raw_length = math.sqrt(dx * dx + dy * dy)
    length = max(raw_length, min_segment_length)
    heading = math.atan2(dy, dx) if raw_length > 1e-9 else fallback_heading

    plan_view = PlanView(
        geometries=[
            ParamPoly3(
                s=0.0,
                x=start_xyz[0],
                y=start_xyz[1],
                hdg=heading,
                length=length,
                aU=0.0,
                bU=1.0,
                cU=0.0,
                dU=0.0,
                aV=0.0,
                bV=0.0,
                cV=0.0,
                dV=0.0,
                pRange="arcLength",
            )
        ]
    )

    is_lht = traffic_rule == TrafficRule.LHT
    lane_id = 1 if is_lht else -1

    lane = Lane(
        lane_id=lane_id,
        lane_type=LaneType.DRIVING,
        predecessor=LaneLevelLaneLink(id=from_lane),
        successor=LaneLevelLaneLink(id=to_lane),
        rule=traffic_rule.value if hasattr(traffic_rule, "value") else None,
    )
    # Emit a constant width so consumers do not see an undefined-width lane
    # (#291 review). Without this the connector lane has no <width> element.
    lane.widths.append(
        LaneWidth(
            s_offset=0.0,
            a=lane_width,
            b=0.0,
            c=0.0,
            d=0.0,
        )
    )

    lane_section = LaneSection(s_offset=0.0)
    if is_lht:
        lane_section.left_lanes[lane_id] = lane
    else:
        lane_section.right_lanes[lane_id] = lane

    lanes = Lanes(lane_sections=[lane_section])

    link = RoadLink(
        predecessor=Predecessor(
            element_type=ElementType.ROAD,
            element_id=incoming_road_id,
            contact_point=incoming_contact,
        ),
        successor=Successor(
            element_type=ElementType.ROAD,
            element_id=outgoing_road_id,
            contact_point=outgoing_contact,
        ),
    )

    return Road(
        id=road_id,
        length=length,
        junction=junction_id,
        rule=traffic_rule,
        plan_view=plan_view,
        lanes=lanes,
        link=link,
        reference_start_xyz=start_xyz,
        reference_end_xyz=end_xyz,
    )


def synthesise_junction_for_site(
    site: DivergenceSite,
    inputs: SanityGateInputs,
    starting_connecting_road_id: int,
    junction_id: int,
    traffic_rule: TrafficRule,
    min_segment_length: float,
    fallback_heading: float = 0.0,
    lane_width: float = 3.5,
) -> SynthesisOutput:
    """Build one synthetic :class:`Junction` plus N zero-length connecting roads.

    For a divergence (``site.is_divergence`` is ``True``) every connecting
    road runs from the source road's end to one candidate road's start.
    For a merge each connecting road runs from a candidate road's end to
    the source road's start.  Connection IDs and connecting-road IDs are
    assigned in deterministic source-lane order so the emitted XML is
    stable across runs.
    """
    junction = Junction(
        id=junction_id, name=f"divergence_{site.road_id}", connections=[]
    )
    connecting_roads: List[Road] = []
    next_road_id = starting_connecting_road_id

    is_divergence = site.is_divergence
    source_road_id = site.road_id

    # Sort lane pairs by source lane id (descending: -1, -2, -3 for RHT;
    # ascending +1, +2, +3 for LHT) so the emitted connecting-road IDs are
    # deterministic and follow the natural left-to-right lane order on the
    # source road.
    is_lht = traffic_rule == TrafficRule.LHT
    sorted_pairs = sorted(inputs.lane_pairs, key=lambda t: (t[0] if is_lht else -t[0]))

    connecting_lane_id = 1 if is_lht else -1

    for src_lane, cand_road_id, cand_lane in sorted_pairs:
        if is_divergence:
            incoming_road_id = source_road_id
            outgoing_road_id = cand_road_id
            incoming_contact = ContactPoint.END
            outgoing_contact = ContactPoint.START
            start_xyz = inputs.endpoint_road
            end_xyz = inputs.endpoints_candidates[cand_road_id]
            connection_incoming = source_road_id
            from_lane, to_lane = src_lane, cand_lane
        else:
            incoming_road_id = cand_road_id
            outgoing_road_id = source_road_id
            incoming_contact = ContactPoint.END
            outgoing_contact = ContactPoint.START
            start_xyz = inputs.endpoints_candidates[cand_road_id]
            end_xyz = inputs.endpoint_road
            connection_incoming = cand_road_id
            from_lane, to_lane = cand_lane, src_lane

        road = _make_zero_length_connecting_road(
            road_id=next_road_id,
            junction_id=junction.id,
            incoming_road_id=incoming_road_id,
            outgoing_road_id=outgoing_road_id,
            incoming_contact=incoming_contact,
            outgoing_contact=outgoing_contact,
            start_xyz=start_xyz,
            end_xyz=end_xyz,
            min_segment_length=min_segment_length,
            traffic_rule=traffic_rule,
            from_lane=from_lane,
            to_lane=to_lane,
            fallback_heading=fallback_heading,
            lane_width=lane_width,
        )

        connecting_roads.append(road)

        connection = Connection(
            id=len(junction.connections),
            incoming_road=connection_incoming,
            connecting_road=next_road_id,
            contact_point=ContactPoint.START,
            lane_links=[
                JunctionLaneLink(from_lane=from_lane, to_lane=connecting_lane_id)
            ],
        )
        junction.connections.append(connection)

        next_road_id += 1

    side_keyword = "successor" if is_divergence else "predecessor"
    return SynthesisOutput(
        junction=junction,
        connecting_roads=connecting_roads,
        deferred_link_patch=(side_keyword, source_road_id, junction.id),
    )


@dataclass(frozen=True)
class DivergenceSynthesisResult:
    """Aggregated synthetic objects produced by :func:`apply_divergence_synthesis`.

    Attributes:
        junctions: synthetic ``Junction`` objects, one per site that passed
            the sanity gate.
        connecting_roads: synthetic zero-length connecting roads. Their IDs
            are contiguous and start at ``starting_connecting_road_id`` of
            the call.
    """

    junctions: List[Junction]
    connecting_roads: List[Road]


def _lane_pairs_for_site(
    site: DivergenceSite,
    roads_by_id: Dict[int, Road],
    lanelet_map,
    routing_graph,
    lanelet_to_road: Dict[int, int],
) -> Tuple[List[Tuple[int, int, int]], Set[int]]:
    """Recover lane-to-lane pairs and the set of all neighbour-road ids.

    Walks the routing graph from each lanelet of ``site.road_id``'s lane
    map in the configured direction (FOLLOWING for divergence, PREVIOUS
    for merge). For every neighbour lanelet that lives in a regular road
    different from ``site.road_id``, emits one ``(source_lane,
    candidate_road, candidate_lane)`` triple. Lanes are recovered via
    ``Road.get_lanelet_to_lane_mapping()`` (existing helper).

    Returns ``(lane_pairs, neighbour_road_ids)``. ``neighbour_road_ids``
    is the set used by :func:`sanity_gate_passes` for the orphan check.
    """
    direction = (
        ConnectionDirection.FOLLOWING
        if site.is_divergence
        else ConnectionDirection.PREVIOUS
    )
    source_road = roads_by_id[site.road_id]
    source_lane_map = source_road.get_lanelet_to_lane_mapping()

    pairs: List[Tuple[int, int, int]] = []
    neighbour_road_ids: Set[int] = set()
    seen_pairs: Set[Tuple[int, int, int]] = set()

    for src_lanelet_id, src_lane_id in source_lane_map.items():
        # `find_connecting_lanelet_groups` accepts an iterable; pass a
        # single-element list. The lanelet object must come from the map.
        lanelet_obj = lanelet_map.laneletLayer[src_lanelet_id]
        groups = find_connecting_lanelet_groups(
            lanelet_map, [lanelet_obj], direction, routing_graph
        )
        for group in groups:
            for ll in group:
                neighbour_road_id = lanelet_to_road.get(ll.id)
                if neighbour_road_id is None or neighbour_road_id == site.road_id:
                    continue
                neighbour_road_ids.add(neighbour_road_id)
                neighbour_road = roads_by_id.get(neighbour_road_id)
                if neighbour_road is None:
                    continue
                neighbour_lane_id = neighbour_road.get_lanelet_to_lane_mapping().get(
                    ll.id
                )
                if neighbour_lane_id is None:
                    continue
                triple = (src_lane_id, neighbour_road_id, neighbour_lane_id)
                if triple in seen_pairs:
                    continue
                seen_pairs.add(triple)
                pairs.append(triple)

    return pairs, neighbour_road_ids


def apply_divergence_synthesis(
    sites: List[DivergenceSite],
    roads_by_id: Dict[int, Road],
    lanelet_map,
    routing_graph,
    lanelet_to_road: Dict[int, int],
    traffic_rule: TrafficRule,
    starting_connecting_road_id: int,
    starting_junction_id: int,
    endpoint_tolerance: float,
    min_segment_length: float,
) -> DivergenceSynthesisResult:
    """Run detection -> sanity gate -> synthesis for every site.

    Sites that fail the gate fall back to the existing "first candidate
    wins" road-level link (logged as a WARNING). Sites that pass are
    synthesised; the returned ``DivergenceSynthesisResult`` is fed into
    the caller's ``junctions`` and ``connecting_roads`` lists.
    """
    next_road_id = starting_connecting_road_id
    next_junction_id = starting_junction_id

    out_junctions: List[Junction] = []
    out_connecting_roads: List[Road] = []

    for site in sites:
        source_road = roads_by_id.get(site.road_id)
        if source_road is None:
            log.warning("divergence: source road %d missing; skipping", site.road_id)
            continue

        endpoint_road = (
            source_road.reference_end_xyz
            if site.is_divergence
            else source_road.reference_start_xyz
        )
        if endpoint_road is None:
            log.warning(
                "divergence: source road %d has no reference endpoint; skipping",
                site.road_id,
            )
            continue

        endpoints_candidates: Dict[int, Tuple[float, float, float]] = {}
        for cand_id in site.candidate_road_ids:
            cand_road = roads_by_id.get(cand_id)
            if cand_road is None:
                # Missing candidate forces gate failure — represent with
                # placeholder None so the gate can flag it.
                continue
            cand_endpoint = (
                cand_road.reference_start_xyz
                if site.is_divergence
                else cand_road.reference_end_xyz
            )
            if cand_endpoint is not None:
                endpoints_candidates[cand_id] = cand_endpoint

        lane_pairs, neighbour_road_ids = _lane_pairs_for_site(
            site, roads_by_id, lanelet_map, routing_graph, lanelet_to_road
        )

        gate_inputs = SanityGateInputs(
            endpoint_road=endpoint_road,
            endpoints_candidates=endpoints_candidates,
            lane_pairs=lane_pairs,
            all_successor_lanelet_road_ids=neighbour_road_ids,
        )
        ok, reason = sanity_gate_passes(site, gate_inputs, endpoint_tolerance)
        if not ok:
            log.warning(
                "divergence: sanity gate failed for road %d (%s side): %s; "
                "falling back to first-candidate link",
                site.road_id,
                site.side.value,
                reason,
            )
            first_candidate = site.candidate_road_ids[0]
            if site.is_divergence:
                source_road.add_successor(
                    element_id=first_candidate,
                    element_type=ElementType.ROAD,
                    contact_point=ContactPoint.START,
                )
            else:
                source_road.add_predecessor(
                    element_id=first_candidate,
                    element_type=ElementType.ROAD,
                    contact_point=ContactPoint.END,
                )
            continue

        # Source road tangent at the divergence/merge endpoint becomes the
        # heading fallback when start_xyz and end_xyz coincide (#291 review).
        source_plan_view = getattr(source_road, "plan_view", None)
        fallback_heading = 0.0
        if source_plan_view is not None:
            endpoint_with_heading = _evaluate_planview_endpoint_with_heading(
                source_plan_view,
                at_start=not site.is_divergence,
            )
            if endpoint_with_heading is not None:
                fallback_heading = endpoint_with_heading[2]

        synthesis = synthesise_junction_for_site(
            site=site,
            inputs=gate_inputs,
            starting_connecting_road_id=next_road_id,
            junction_id=next_junction_id,
            traffic_rule=traffic_rule,
            min_segment_length=min_segment_length,
            fallback_heading=fallback_heading,
            lane_width=DEFAULT_CONFIG.geometry.divergence_default_lane_width,
        )

        out_junctions.append(synthesis.junction)
        out_connecting_roads.extend(synthesis.connecting_roads)
        next_road_id += len(synthesis.connecting_roads)
        next_junction_id += 1

        side, _src, jid = synthesis.deferred_link_patch
        if side == "successor":
            source_road.add_successor(
                element_id=jid,
                element_type=ElementType.JUNCTION,
                contact_point=None,
            )
        else:
            source_road.add_predecessor(
                element_id=jid,
                element_type=ElementType.JUNCTION,
                contact_point=None,
            )

        # Mirror-side patch on each candidate so both sides agree on the
        # junction link (#291 review). Without this the candidates retain
        # their direct road->road links from construct_from_lanelet_map and
        # the topology becomes inconsistent.
        for cand_id in site.candidate_road_ids:
            cand_road = roads_by_id.get(cand_id)
            if cand_road is None:
                continue
            if site.is_divergence:
                cand_road.add_predecessor(
                    element_id=jid,
                    element_type=ElementType.JUNCTION,
                    contact_point=None,
                )
            else:
                cand_road.add_successor(
                    element_id=jid,
                    element_type=ElementType.JUNCTION,
                    contact_point=None,
                )

    return DivergenceSynthesisResult(
        junctions=out_junctions, connecting_roads=out_connecting_roads
    )
