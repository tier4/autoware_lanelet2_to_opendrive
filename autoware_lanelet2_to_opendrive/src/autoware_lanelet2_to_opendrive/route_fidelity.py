"""Does the emitted OpenDRIVE state the successions the Lanelet2 map states?

Stub. The reading of the file and the comparison with the map are not
implemented yet; see ``test/test_route_fidelity.py`` for what they must do.
"""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple

import lxml.etree as ET

Slot = Tuple[int, int]
"""An emitted lane: ``(road id, lane id)``."""

Transition = Tuple[Slot, Slot]
"""One lane-to-lane succession across a road boundary."""


class EmittedNetwork:
    """The transitions an ``.xodr`` states."""

    def __init__(self, path: Path) -> None:
        root = ET.parse(str(path)).getroot()
        self.roads: Dict[int, ET._Element] = {}
        for road in root.iter("road"):
            road_id = road.get("id")
            if road_id is not None:
                self.roads[int(road_id)] = road
        self.junctions: Dict[int, List[Tuple[int, int, Dict[int, int]]]] = defaultdict(
            list
        )

    def forward_end(self, road_id: int, lane_id: int) -> str:
        return "successor"

    def next_slots(self, slot: Slot) -> List[Slot]:
        return []

    def transitions(self, walk_through: Set[int], max_hops: int = 6) -> Set[Transition]:
        return set()


@dataclass
class RouteFidelity:
    """What the map asked for and what the file delivered."""

    stated: int = 0
    internal: int = 0
    unmapped: int = 0
    walked_through: int = 0
    emitted: int = 0
    missing: List[object] = field(default_factory=list)
    surplus: List[Transition] = field(default_factory=list)

    @property
    def missing_share(self) -> float:
        return (len(self.missing) / self.stated) if self.stated else 0.0


def measure(
    map_path: Path,
    xodr_path: Path,
    mapping_path: Path,
    projector: Optional[object] = None,
) -> RouteFidelity:
    """Compare the successions the map states with the transitions emitted."""
    return RouteFidelity()
