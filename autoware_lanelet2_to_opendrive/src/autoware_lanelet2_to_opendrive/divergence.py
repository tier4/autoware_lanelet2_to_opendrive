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

from dataclasses import dataclass
from enum import Enum
from typing import List


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
