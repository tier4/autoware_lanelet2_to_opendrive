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

import math
from dataclasses import dataclass
from enum import Enum
from typing import Dict, Iterable, List, Set, Tuple


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

    return True, ""
