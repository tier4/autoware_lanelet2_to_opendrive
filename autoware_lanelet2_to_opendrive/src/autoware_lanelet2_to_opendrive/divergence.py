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
from autoware_lanelet2_to_opendrive.opendrive.elevation import (
    Elevation,
    ElevationProfile,
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
from autoware_lanelet2_to_opendrive.opendrive.reference_line import ReferenceLine
from autoware_lanelet2_to_opendrive.opendrive.opendrive_dataclass import (
    LaneLink as LaneLevelLaneLink,
)
from autoware_lanelet2_to_opendrive.opendrive.road import (
    Road,
    _evaluate_lane_width,
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
        lane_pair_endpoint_distances: optional ``lane_pair -> distance`` map
            holding the gap between the rendered source and candidate lane
            anchors of each entry in ``lane_pairs``. When supplied for every
            lane pair, endpoint coincidence is validated at lane level
            instead of against the road reference endpoints.
    """

    endpoint_road: Tuple[float, float, float]
    endpoints_candidates: Dict[int, Tuple[float, float, float]]
    lane_pairs: List[Tuple[int, int, int]]
    all_successor_lanelet_road_ids: Set[int]
    lane_pair_endpoint_distances: Dict[Tuple[int, int, int], float] | None = None


def _distance(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
    return math.sqrt(sum((ai - bi) ** 2 for ai, bi in zip(a, b)))


def _complete_lane_pair_distances(
    inputs: SanityGateInputs,
) -> Dict[Tuple[int, int, int], float] | None:
    """Return the lane-pair distance map only when it covers every lane pair.

    A partially populated map cannot be used for gating because the entries
    it lacks would silently escape the endpoint check; the caller then falls
    back to the road-reference comparison.
    """
    distances = inputs.lane_pair_endpoint_distances
    if distances is None or not inputs.lane_pairs:
        return None
    if any(lane_pair not in distances for lane_pair in inputs.lane_pairs):
        return None
    return distances


def _nearest_candidate_within_tolerance(
    site: DivergenceSite,
    inputs: SanityGateInputs,
    endpoint_tolerance: float,
) -> Tuple[int | None, float]:
    """Pick the geometrically closest candidate for the single-link fallback.

    Returns ``(candidate_road_id, distance)``; ``candidate_road_id`` is
    ``None`` when no candidate is within ``endpoint_tolerance``. Distances
    come from the rendered lane anchors when available (the closest lane pair
    of that candidate) and from the road reference endpoints otherwise.
    Linking to an arbitrary candidate would emit a connector whose heading can
    be almost reversed, so an out-of-tolerance site emits nothing instead.
    """
    distances = inputs.lane_pair_endpoint_distances or {}
    best_id: int | None = None
    best_distance = math.inf
    for cand_id in site.candidate_road_ids:
        candidate_distance = min(
            (
                distances[lane_pair]
                for lane_pair in inputs.lane_pairs
                if lane_pair[1] == cand_id and lane_pair in distances
            ),
            default=math.inf,
        )
        if candidate_distance == math.inf:
            cand_endpoint = inputs.endpoints_candidates.get(cand_id)
            if cand_endpoint is None:
                continue
            candidate_distance = _distance(inputs.endpoint_road, cand_endpoint)
        if candidate_distance < best_distance:
            best_distance = candidate_distance
            best_id = cand_id
    if best_id is None or best_distance > endpoint_tolerance:
        return None, best_distance
    return best_id, best_distance


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
    #
    # Prefer the rendered per-lane anchors when the caller supplied one for
    # every recovered lane pair. In a multi-lane split or merge a candidate
    # attached to a non-reference lane is legitimately offset from the source
    # road's reference endpoint by one or more lane widths, so comparing
    # reference endpoints would reject a perfectly valid site and drop the
    # branch. Reference-endpoint coincidence stays as the fallback for
    # callers without lane-anchor metadata.
    for cand_id in site.candidate_road_ids:
        if inputs.endpoints_candidates.get(cand_id) is None:
            return False, f"missing endpoint for candidate road {cand_id}"

    lane_pair_distances = _complete_lane_pair_distances(inputs)
    if lane_pair_distances is not None:
        for lane_pair in inputs.lane_pairs:
            if lane_pair_distances[lane_pair] > endpoint_tolerance:
                return (
                    False,
                    f"lane-aware endpoint distance for lane pair {lane_pair} "
                    f"exceeds {endpoint_tolerance:.3f} m",
                )
    else:
        for cand_id in site.candidate_road_ids:
            cand_endpoint = inputs.endpoints_candidates[cand_id]
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


@dataclass(frozen=True)
class _DirectedLaneEdge:
    """One canonical directed lane connection across a split/merge boundary.

    Both a predecessor-side merge site and a successor-side split site of the
    same N->M boundary describe the very same set of lane connections; the
    directed edge is the representation they agree on, so the two sites can
    be recognised as one junction instead of two conflicting ones.
    """

    incoming_road_id: int
    incoming_lane_id: int
    outgoing_road_id: int
    outgoing_lane_id: int
    start_xyz: Tuple[float, float, float]
    end_xyz: Tuple[float, float, float]
    # Tangent of the *incoming* road where the connector leaves it. The
    # connector stands for a zero-length transition, so this — not the
    # residual between the two anchors — is the direction its rendered
    # segment must take.
    fallback_heading: float
    # Width of the incoming / outgoing lane at the respective contact, so
    # the connector can reproduce the cross-section it bridges instead of
    # a nominal constant. ``None`` when the neighbour carries no width.
    start_lane_width: float | None = None
    end_lane_width: float | None = None

    @property
    def key(self) -> Tuple[int, int, int, int]:
        """Identity of the edge, ignoring the rendered geometry."""
        return (
            self.incoming_road_id,
            self.incoming_lane_id,
            self.outgoing_road_id,
            self.outgoing_lane_id,
        )


@dataclass(frozen=True)
class _ReadyDivergenceSite:
    """A site that cleared the sanity gate, normalised to directed lane edges."""

    site: DivergenceSite
    edges: List[_DirectedLaneEdge]


@dataclass(frozen=True)
class _ReadyDivergenceGroup:
    """One canonical split/merge boundary ready for junction synthesis."""

    name: str
    edges: List[_DirectedLaneEdge]


@dataclass(frozen=True)
class _BoundarySynthesisOutput:
    """Synthetic junction plus connecting roads for one canonical boundary."""

    junction: Junction
    connecting_roads: List[Road]


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
    reduce to ``u(p) = p``, ``v(p) = 0``.

    The heading only comes from the ``start_xyz -> end_xyz`` vector when
    that vector is at least ``min_segment_length`` long.  Below the floor
    the connector is rendered as a segment of ``min_segment_length``
    anyway, so the vector no longer describes the segment that is drawn —
    and at the scale these anchors actually differ (sub-centimetre, the
    residual disagreement between two roads that are meant to meet)
    ``atan2`` returns numerical noise, which rotates the whole
    cross-section out of the lane it is supposed to continue.  In that
    regime the heading falls back to ``fallback_heading`` — the incoming
    road's own end tangent — so the stub's lane borders line up with the
    borders it joins.  ``shared_connecting_roads._make_stub`` collapses
    its own anchors for exactly this reason.

    The lane carries lane-level predecessor/successor links from
    ``from_lane`` to ``to_lane`` and a constant ``lane_width`` so the
    eventual XML always emits a ``<width>`` element.  Callers that know
    the widths of the lanes being bridged should follow up with
    :func:`_taper_stub_width` so the stub reproduces the neighbours'
    cross-section rather than a nominal one.
    """
    dx = end_xyz[0] - start_xyz[0]
    dy = end_xyz[1] - start_xyz[1]
    raw_length = math.sqrt(dx * dx + dy * dy)
    length = max(raw_length, min_segment_length)
    direction_floor = max(min_segment_length, 1e-9)
    heading = math.atan2(dy, dx) if raw_length >= direction_floor else fallback_heading

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

    # Linear elevation profile so the connector matches the linked roads'
    # z at both endpoints. Without this the synthesised road serializes
    # without an <elevationProfile> and downstream consumers (CARLA,
    # qc-validate, test_junction_endpoint_fidelity) read z=0 — producing
    # gross 3D mismatches at ramps, hills, or any non-flat divergence.
    z_start = start_xyz[2]
    z_end = end_xyz[2]
    elev_b = (z_end - z_start) / length if length > 0 else 0.0
    elevation_profile = ElevationProfile(
        elevations=[
            Elevation(s=0.0, a=z_start, b=elev_b, c=0.0, d=0.0),
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

    # OpenDRIVE 1.4 XSD requires every <laneSection> to contain a <center>
    # element. Use the same dummy ReferenceLine pattern that
    # ``parse_roads_from_xodr`` uses when reconstructing roads from XML
    # — bypass ``ReferenceLine.__init__`` (which requires a Splines
    # centerline) and inject a stub Lane(id=0, type=NONE).
    center_ref = ReferenceLine.__new__(ReferenceLine)
    center_ref._lane = Lane(lane_id=0, lane_type=LaneType.NONE, level=False)
    lane_section.center_lane = center_ref

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
        elevation_profile=elevation_profile,
        lanes=lanes,
        link=link,
        reference_start_xyz=start_xyz,
        reference_end_xyz=end_xyz,
    )


def _lane_width_at(road: Road | None, lane_id: int, *, at_start: bool) -> float | None:
    """Width of ``lane_id`` at the start or end of ``road``.

    ``None`` whenever the road carries no usable lane geometry, mirroring
    ``_evaluate_lane_width``'s own ``None`` semantics so callers fall back
    to a nominal width instead of silently collapsing a lane to zero.
    Attributes are read through ``getattr`` for the same reason
    :func:`_lane_anchor_xyz` does: mock road stubs in the unit tests omit
    them from their ``spec``.
    """
    if road is None:
        return None
    lanes = getattr(road, "lanes", None)
    if lanes is None or not lanes.lane_sections:
        return None
    lane_section = lanes.lane_sections[0 if at_start else -1]
    lane = lane_section.left_lanes.get(lane_id) or lane_section.right_lanes.get(lane_id)
    if lane is None:
        return None
    length = getattr(road, "length", None)
    if not at_start and not isinstance(length, (int, float)):
        return None
    return _evaluate_lane_width(lane, 0.0 if at_start else float(length))


def _lane_width_at_end(road: Road | None, lane_id: int) -> float | None:
    """Width of ``lane_id`` where ``road`` ends."""
    return _lane_width_at(road, lane_id, at_start=False)


def _taper_stub_width(
    stub: Road, start_width: float | None, end_width: float | None
) -> None:
    """Interpolate the stub's lane width between the lanes it joins.

    A nominal constant width (``divergence_default_lane_width``, 3.5 m)
    leaves the stub's *outer* lane border up to a metre away from the
    border of the road on either side, which
    ``lane_smoothness.contact_point_no_horizontal_gaps`` reads as a gap
    even where the reference lines meet exactly: that checker compares
    both the inner and the outer border of the two lanes at the contact
    and wants at least two of the four pairings inside 1 cm.  A linear
    taper reproduces both neighbours' widths at the respective contact
    points, so together with the anchored start point and the incoming
    road's end tangent the whole cross-section is reproduced, not just
    the reference line.

    No-op when either width is unknown, so the caller's nominal constant
    survives rather than being replaced by a worse guess.
    """
    if start_width is None or end_width is None or stub.lanes is None:
        return
    if stub.length <= 0.0:
        return
    lane_section = stub.lanes.lane_sections[0]
    lanes = list(lane_section.left_lanes.values()) + list(
        lane_section.right_lanes.values()
    )
    for lane in lanes:
        lane.widths = [
            LaneWidth(
                s_offset=0.0,
                a=start_width,
                b=(end_width - start_width) / stub.length,
                c=0.0,
                d=0.0,
            )
        ]


def _directed_edges_for_site(
    site: DivergenceSite,
    inputs: SanityGateInputs,
    traffic_rule: TrafficRule,
    fallback_heading: float,
    lane_pair_endpoints: (
        Dict[
            Tuple[int, int, int],
            Tuple[
                Tuple[float, float, float] | None,
                Tuple[float, float, float] | None,
            ],
        ]
        | None
    ) = None,
    roads_by_id: Dict[int, Road] | None = None,
) -> List[_DirectedLaneEdge]:
    """Normalise a predecessor/successor site into directed lane edges.

    Lane pairs are visited in source-lane order (descending for RHT,
    ascending for LHT) so the connecting-road IDs derived from them stay
    deterministic. ``lane_pair_endpoints`` overrides the road reference
    endpoints with the rendered per-lane anchors when the caller has them.

    ``roads_by_id`` lets each edge carry the cross-section of the pair it
    bridges: the *incoming* road's end tangent (rather than the site's
    source road, which on a merge site is the outgoing one) and the two
    lane widths at the contacts. Omitted by callers that hold mock roads,
    which then keep the site-level ``fallback_heading`` and a nominal
    width.
    """
    is_lht = traffic_rule == TrafficRule.LHT
    sorted_pairs = sorted(inputs.lane_pairs, key=lambda t: (t[0] if is_lht else -t[0]))

    edges: List[_DirectedLaneEdge] = []
    for src_lane, cand_road_id, cand_lane in sorted_pairs:
        if site.is_divergence:
            incoming_road_id, incoming_lane_id = site.road_id, src_lane
            outgoing_road_id, outgoing_lane_id = cand_road_id, cand_lane
            start_xyz = inputs.endpoint_road
            end_xyz = inputs.endpoints_candidates[cand_road_id]
        else:
            incoming_road_id, incoming_lane_id = cand_road_id, cand_lane
            outgoing_road_id, outgoing_lane_id = site.road_id, src_lane
            start_xyz = inputs.endpoints_candidates[cand_road_id]
            end_xyz = inputs.endpoint_road

        if lane_pair_endpoints is not None:
            anchored = lane_pair_endpoints.get((src_lane, cand_road_id, cand_lane))
            if anchored is not None:
                anchored_start, anchored_end = anchored
                if anchored_start is not None:
                    start_xyz = anchored_start
                if anchored_end is not None:
                    end_xyz = anchored_end

        incoming_road = (roads_by_id or {}).get(incoming_road_id)
        outgoing_road = (roads_by_id or {}).get(outgoing_road_id)

        edge_heading = fallback_heading
        if incoming_road is not None and incoming_road.plan_view is not None:
            endpoint = _evaluate_planview_endpoint_with_heading(
                incoming_road.plan_view, at_start=False
            )
            if endpoint is not None:
                edge_heading = endpoint[2]

        edges.append(
            _DirectedLaneEdge(
                incoming_road_id=incoming_road_id,
                incoming_lane_id=incoming_lane_id,
                outgoing_road_id=outgoing_road_id,
                outgoing_lane_id=outgoing_lane_id,
                start_xyz=start_xyz,
                end_xyz=end_xyz,
                fallback_heading=edge_heading,
                start_lane_width=_lane_width_at_end(incoming_road, incoming_lane_id),
                end_lane_width=_lane_width_at(
                    outgoing_road, outgoing_lane_id, at_start=True
                ),
            )
        )
    return edges


def _group_overlapping_ready_sites(
    ready_sites: List[_ReadyDivergenceSite],
) -> List[_ReadyDivergenceGroup]:
    """Merge sites that describe the same physical boundary into one group.

    An N->M boundary is discovered twice: once as a predecessor-side merge on
    the downstream road and once as a successor-side split on an upstream
    road. Emitting two junctions for it would make the two road-level links
    disagree, so sites sharing at least one directed lane edge are unioned
    and synthesised as a single junction.
    """
    if not ready_sites:
        return []

    parent = list(range(len(ready_sites)))

    def find(index: int) -> int:
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index

    def union(left: int, right: int) -> None:
        left_root, right_root = find(left), find(right)
        if left_root != right_root:
            parent[right_root] = left_root

    edge_owner: Dict[Tuple[int, int, int, int], int] = {}
    for index, ready in enumerate(ready_sites):
        for edge in ready.edges:
            union(index, edge_owner.setdefault(edge.key, index))

    grouped_indices: Dict[int, List[int]] = {}
    for index in range(len(ready_sites)):
        grouped_indices.setdefault(find(index), []).append(index)

    groups: List[_ReadyDivergenceGroup] = []
    for indices in grouped_indices.values():
        # Deduplicate shared edges, keeping the shortest rendered connector.
        edges_by_key: Dict[Tuple[int, int, int, int], _DirectedLaneEdge] = {}
        for index in indices:
            for edge in ready_sites[index].edges:
                previous = edges_by_key.get(edge.key)
                if previous is None or _distance(
                    edge.start_xyz, edge.end_xyz
                ) < _distance(previous.start_xyz, previous.end_xyz):
                    edges_by_key[edge.key] = edge
        groups.append(
            _ReadyDivergenceGroup(
                name=f"divergence_{ready_sites[indices[0]].site.road_id}",
                edges=list(edges_by_key.values()),
            )
        )
    return groups


def _synthesise_junction_for_edges(
    junction_name: str,
    edges: List[_DirectedLaneEdge],
    starting_connecting_road_id: int,
    junction_id: int,
    traffic_rule: TrafficRule,
    min_segment_length: float,
    lane_width: float,
) -> _BoundarySynthesisOutput:
    """Emit one zero-length connecting road per directed lane edge.

    One ``<lane><link><successor>`` per lane is an OpenDRIVE invariant, so a
    fork can only keep all of its branches when every branch owns its own
    connector inside the junction.
    """
    junction = Junction(id=junction_id, name=junction_name, connections=[])
    connecting_roads: List[Road] = []
    connecting_lane_id = 1 if traffic_rule == TrafficRule.LHT else -1
    next_road_id = starting_connecting_road_id

    for edge in edges:
        connector = _make_zero_length_connecting_road(
            road_id=next_road_id,
            junction_id=junction.id,
            incoming_road_id=edge.incoming_road_id,
            outgoing_road_id=edge.outgoing_road_id,
            incoming_contact=ContactPoint.END,
            outgoing_contact=ContactPoint.START,
            start_xyz=edge.start_xyz,
            end_xyz=edge.end_xyz,
            min_segment_length=min_segment_length,
            traffic_rule=traffic_rule,
            from_lane=edge.incoming_lane_id,
            to_lane=edge.outgoing_lane_id,
            fallback_heading=edge.fallback_heading,
            lane_width=(
                edge.start_lane_width
                if edge.start_lane_width is not None
                else lane_width
            ),
        )
        # ``start_xyz`` anchors the connector on the incoming lane's inner
        # border and ``fallback_heading`` orients its normal at that lane's
        # outer border; the taper puts the outer border at the right
        # distance, at this end and at the other one. Together they
        # reproduce the cross-section the connector bridges, which is what
        # ``contact_point_no_horizontal_gaps`` compares.
        _taper_stub_width(connector, edge.start_lane_width, edge.end_lane_width)
        connecting_roads.append(connector)
        junction.connections.append(
            Connection(
                id=len(junction.connections),
                incoming_road=edge.incoming_road_id,
                connecting_road=next_road_id,
                contact_point=ContactPoint.START,
                lane_links=[
                    JunctionLaneLink(
                        from_lane=edge.incoming_lane_id,
                        to_lane=connecting_lane_id,
                    )
                ],
            )
        )
        next_road_id += 1

    return _BoundarySynthesisOutput(
        junction=junction, connecting_roads=connecting_roads
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
    lane_pair_endpoints: (
        Dict[
            Tuple[int, int, int],
            Tuple[Tuple[float, float, float], Tuple[float, float, float]],
        ]
        | None
    ) = None,
) -> SynthesisOutput:
    """Build one synthetic :class:`Junction` plus N zero-length connecting roads.

    For a divergence (``site.is_divergence`` is ``True``) every connecting
    road runs from the source road's end to one candidate road's start.
    For a merge each connecting road runs from a candidate road's end to
    the source road's start.  Connection IDs and connecting-road IDs are
    assigned in deterministic source-lane order so the emitted XML is
    stable across runs.

    ``lane_pair_endpoints`` optionally supplies per-(src_lane,
    cand_road, cand_lane) ``(start_xyz, end_xyz)`` pairs computed from
    each road's lane-aware anchor (see ``Road.evaluate_lane_anchor_xyz``)
    so the synthetic connector terminates on the *lane* edge rather than
    the road's reference line.  This matches the endpoint-pinning
    contract enforced by ``test_junction_endpoint_fidelity`` for
    real ``turn_direction`` connecting roads (#437).  When omitted, the
    synthesiser falls back to ``inputs.endpoint_road`` /
    ``inputs.endpoints_candidates`` so non-Road callers (unit tests
    using mock road stubs) still work.
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

        # Override with lane-aware anchor endpoints when the driver
        # supplied them. Per-pair anchors place each connector on the
        # corresponding lane edge instead of all sharing the source
        # road's reference line endpoint (#291 endpoint-fidelity fix).
        if lane_pair_endpoints is not None:
            anchored = lane_pair_endpoints.get((src_lane, cand_road_id, cand_lane))
            if anchored is not None:
                anchored_start, anchored_end = anchored
                if anchored_start is not None:
                    start_xyz = anchored_start
                if anchored_end is not None:
                    end_xyz = anchored_end

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


def _lane_anchor_xyz(
    road: Road | None,
    lane_id: int,
    *,
    at_start: bool,
    traffic_rule: TrafficRule,
) -> Tuple[float, float, float] | None:
    """Resolve ``Road.evaluate_lane_anchor_xyz`` for a given OpenDRIVE lane id.

    Maps the lane id (negative for RHT, positive for LHT) to the road's
    ``sorted_lanelet_ids`` index (``0..n-1`` left-to-right) and forwards
    the call. Returns ``None`` when the road lacks the metadata required
    by the helper, mirroring its own ``None`` semantics so callers can
    fall back to the road's reference endpoint.  Uses ``getattr`` for
    defence against mock road stubs that omit the attribute from their
    ``spec``.
    """
    if road is None:
        return None
    sorted_lanelet_ids = getattr(road, "sorted_lanelet_ids", None)
    if sorted_lanelet_ids is None:
        return None
    evaluator = getattr(road, "evaluate_lane_anchor_xyz", None)
    if evaluator is None:
        return None
    n = len(sorted_lanelet_ids)
    if traffic_rule == TrafficRule.LHT:
        sorted_index = n - lane_id
    else:
        sorted_index = -lane_id - 1
    if not (0 <= sorted_index < n):
        return None
    return evaluator(sorted_index=sorted_index, at_start=at_start)


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

    Sites that pass the gate are normalised to directed lane edges; sites
    describing the same N->M boundary are unioned so they share one junction,
    which then carries one zero-length connector per directed lane edge and
    therefore keeps every branch of a fork. Sites that fail the gate fall
    back to a road-level link towards the geometrically nearest candidate,
    and emit nothing at all when no candidate is within ``endpoint_tolerance``
    (both logged as a WARNING). The returned ``DivergenceSynthesisResult`` is
    fed into the caller's ``junctions`` and ``connecting_roads`` lists.
    """
    next_road_id = starting_connecting_road_id
    next_junction_id = starting_junction_id

    out_junctions: List[Junction] = []
    out_connecting_roads: List[Road] = []
    ready_sites: List[_ReadyDivergenceSite] = []

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
        # Per-pair lane-aware anchor endpoints. For divergence the source
        # side uses ``Road.evaluate_lane_anchor_xyz(lane_index, at_start=
        # False)`` so each connector terminates on the lane edge of the
        # source's specific src_lane; the candidate side uses
        # ``at_start=True`` on the candidate's cand_lane.  For merge it is
        # mirrored.  When the source/candidate road lacks the metadata
        # required by ``evaluate_lane_anchor_xyz`` (sorted_lanelet_ids,
        # lanes, plan_view), we fall back to the road's reference
        # endpoint by leaving the entry as ``None`` — the synthesiser
        # then uses ``inputs.endpoint_road`` / ``endpoints_candidates``.
        # The distance between the two rendered anchors is what the sanity
        # gate validates, so a candidate attached to a non-reference lane is
        # judged on its own lane rather than on the reference line.
        lane_pair_endpoints: Dict[
            Tuple[int, int, int],
            Tuple[
                Tuple[float, float, float] | None,
                Tuple[float, float, float] | None,
            ],
        ] = {}
        lane_pair_endpoint_distances: Dict[Tuple[int, int, int], float] = {}
        for src_lane, cand_road_id, cand_lane in lane_pairs:
            cand_road = roads_by_id.get(cand_road_id)
            if site.is_divergence:
                src_anchor = _lane_anchor_xyz(
                    source_road, src_lane, at_start=False, traffic_rule=traffic_rule
                )
                cand_anchor = _lane_anchor_xyz(
                    cand_road, cand_lane, at_start=True, traffic_rule=traffic_rule
                )
                start_anchor, end_anchor = src_anchor, cand_anchor
            else:
                src_anchor = _lane_anchor_xyz(
                    source_road, src_lane, at_start=True, traffic_rule=traffic_rule
                )
                cand_anchor = _lane_anchor_xyz(
                    cand_road, cand_lane, at_start=False, traffic_rule=traffic_rule
                )
                start_anchor, end_anchor = cand_anchor, src_anchor
            lane_pair_endpoints[(src_lane, cand_road_id, cand_lane)] = (
                start_anchor,
                end_anchor,
            )
            if start_anchor is not None and end_anchor is not None:
                lane_pair_endpoint_distances[(src_lane, cand_road_id, cand_lane)] = (
                    _distance(start_anchor, end_anchor)
                )

        gate_inputs = SanityGateInputs(
            endpoint_road=endpoint_road,
            endpoints_candidates=endpoints_candidates,
            lane_pairs=lane_pairs,
            all_successor_lanelet_road_ids=neighbour_road_ids,
            lane_pair_endpoint_distances=lane_pair_endpoint_distances or None,
        )
        ok, reason = sanity_gate_passes(site, gate_inputs, endpoint_tolerance)
        if not ok:
            nearest_id, nearest_distance = _nearest_candidate_within_tolerance(
                site, gate_inputs, endpoint_tolerance
            )
            if nearest_id is None:
                log.warning(
                    "divergence: sanity gate failed for road %d (%s side): %s; "
                    "no candidate within %.3f m (nearest %.3f m), emitting no link",
                    site.road_id,
                    site.side.value,
                    reason,
                    endpoint_tolerance,
                    nearest_distance,
                )
                continue
            log.warning(
                "divergence: sanity gate failed for road %d (%s side): %s; "
                "falling back to nearest-candidate link %d (%.3f m)",
                site.road_id,
                site.side.value,
                reason,
                nearest_id,
                nearest_distance,
            )
            if site.is_divergence:
                source_road.add_successor(
                    element_id=nearest_id,
                    element_type=ElementType.ROAD,
                    contact_point=ContactPoint.START,
                )
            else:
                source_road.add_predecessor(
                    element_id=nearest_id,
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

        ready_sites.append(
            _ReadyDivergenceSite(
                site=site,
                edges=_directed_edges_for_site(
                    site=site,
                    inputs=gate_inputs,
                    traffic_rule=traffic_rule,
                    fallback_heading=fallback_heading,
                    lane_pair_endpoints=lane_pair_endpoints,
                    roads_by_id=roads_by_id,
                ),
            )
        )

    for group in _group_overlapping_ready_sites(ready_sites):
        synthesis = _synthesise_junction_for_edges(
            junction_name=group.name,
            edges=group.edges,
            starting_connecting_road_id=next_road_id,
            junction_id=next_junction_id,
            traffic_rule=traffic_rule,
            min_segment_length=min_segment_length,
            lane_width=DEFAULT_CONFIG.geometry.divergence_default_lane_width,
        )

        out_junctions.append(synthesis.junction)
        out_connecting_roads.extend(synthesis.connecting_roads)
        next_road_id += len(synthesis.connecting_roads)
        next_junction_id += 1

        # Both sides of the boundary must name the junction, otherwise the
        # roads keep the direct road->road links that
        # ``construct_from_lanelet_map`` left behind and the topology becomes
        # inconsistent (#291 review).
        jid = synthesis.junction.id
        for road_id in sorted({edge.incoming_road_id for edge in group.edges}):
            incoming_road = roads_by_id.get(road_id)
            if incoming_road is not None:
                incoming_road.add_successor(
                    element_id=jid,
                    element_type=ElementType.JUNCTION,
                    contact_point=None,
                )
        for road_id in sorted({edge.outgoing_road_id for edge in group.edges}):
            outgoing_road = roads_by_id.get(road_id)
            if outgoing_road is not None:
                outgoing_road.add_predecessor(
                    element_id=jid,
                    element_type=ElementType.JUNCTION,
                    contact_point=None,
                )

    return DivergenceSynthesisResult(
        junctions=out_junctions, connecting_roads=out_connecting_roads
    )
