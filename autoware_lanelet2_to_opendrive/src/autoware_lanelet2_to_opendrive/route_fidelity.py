"""Does the emitted OpenDRIVE preserve the successions the Lanelet2 map states?

Geometry has the converter's own ``symmetric_point_to_segment_max``, and
conformance has the ASAM bundle, but what the map *means for driving* -- which
lane you may reach from which -- has had no instrument in the repository. A
conversion can score a fraction of a millimetre on every reference line and
still have dropped a turn.

This measures **transitions**: one lane-to-lane succession across a road
boundary. It does not measure reachability, because the two are not
independent -- every route lost on the reference maps traced back to a missing
transition, and feeding those transitions back recovered every lost route. A
transition is the smallest thing that can be wrong, so it is what gets
counted.

**What the map states.** ``RoutingGraph.following`` over the convertible
lanelets, with each lanelet projected onto the ``(road, lane)`` slot it was
emitted as, read from the conversion's mapping file. Two cases are not
transitions and are reported separately rather than counted as losses:

* a succession whose two lanelets land on the *same* slot is internal to one
  road and carries no link;
* a lanelet can be emitted as *several* slots -- a duplicated fork stage is
  one lanelet on several connecting roads -- and the succession is then
  satisfied by **any** of the slot pairs, not by all of them. Treating the
  cross product as separate requirements demands transitions between the
  duplicates themselves, which is not something the map asks for.

**What the file states.** Reconstructed the way a reader reads it. Travel
direction follows the road's ``rule``: under ``RHT`` a right-hand
(negative-id) lane travels along +s and reads its successor at the road's
successor end, and under ``LHT`` it is the positive-id lanes that do. A
junction-typed end resolves through the junction's ``connection`` /
``laneLink`` rows, not through the lane links. A road with no backing lanelet
-- a synthetic connector the converter inserts where the map states a
succession directly -- is **walked through** rather than counted as a node;
counting it would report every connector as a transition the map does not
state and the succession it stands for as missing.
"""

from __future__ import annotations

import argparse
import json
import logging
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple

import lanelet2
import lxml.etree as ET

from .util import (
    CONVERTIBLE_LANELET_SUBTYPES,
    create_routing_graph,
    filter_lanelets_by_subtype,
)

logger = logging.getLogger(__name__)

Slot = Tuple[int, int]
"""An emitted lane: ``(road id, lane id)``."""

Transition = Tuple[Slot, Slot]
"""One lane-to-lane succession across a road boundary."""

DEFAULT_MAX_CONNECTOR_HOPS = 6
"""How many backing-lanelet-free roads may be chained between two real lanes.

Connectors chain: a fork stage can be followed by a merge stage. Six is well
past the longest chain measured on the reference maps and bounds the walk on a
pathological file.
"""


def load_mapping(path: Path) -> Tuple[Dict[int, Slot], Dict[int, List[Slot]]]:
    """Read the conversion's lanelet-to-slot mapping.

    Returns the primary slot of each lanelet and, separately, the extra slots
    of a lanelet emitted more than once.
    """
    with open(path, encoding="utf-8") as handle:
        data = json.load(handle)
    primary = {
        int(key): (int(value[0]), int(value[1]))
        for key, value in data["lanelet_to_road_and_lane"].items()
    }
    extra: Dict[int, List[Slot]] = {}
    for key, pairs in (data.get("lanelet_additional_road_lanes") or {}).items():
        extra[int(key)] = [(int(pair[0]), int(pair[1])) for pair in pairs]
    return primary, extra


class EmittedNetwork:
    """The transitions an ``.xodr`` states, resolved the way a reader does."""

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
        for junction in root.iter("junction"):
            junction_id = junction.get("id")
            if junction_id is None:
                continue
            for connection in junction.findall("connection"):
                incoming = connection.get("incomingRoad")
                connecting = connection.get("connectingRoad")
                if incoming is None or connecting is None:
                    continue
                links: Dict[int, int] = {}
                for link in connection.findall("laneLink"):
                    source, target = link.get("from"), link.get("to")
                    if source is not None and target is not None:
                        links[int(source)] = int(target)
                self.junctions[int(junction_id)].append(
                    (int(incoming), int(connecting), links)
                )

    def _lane_elements(self, road_id: int) -> Iterable[ET._Element]:
        road = self.roads.get(road_id)
        if road is None:
            return ()
        lanes = road.find("lanes")
        if lanes is None:
            return ()
        return lanes.iter("lane")

    def driving_lane_ids(self, road_id: int) -> Set[int]:
        out: Set[int] = set()
        for lane in self._lane_elements(road_id):
            lane_id = lane.get("id")
            if lane_id is not None and lane.get("type") == "driving":
                out.add(int(lane_id))
        return out

    def lane(self, road_id: int, lane_id: int) -> Optional[ET._Element]:
        for candidate in self._lane_elements(road_id):
            if candidate.get("id") == str(lane_id):
                return candidate
        return None

    def forward_end(self, road_id: int, lane_id: int) -> str:
        """Which road end travel in this lane leaves by.

        A lane's own direction is which side of the reference line it sits on
        together with the road's ``rule``: under right-hand traffic the
        right-hand (negative-id) lanes travel along +s, and under left-hand
        traffic the left-hand (positive-id) ones do.
        """
        road = self.roads.get(road_id)
        rule = (road.get("rule") if road is not None else None) or "RHT"
        travels_with_s = (lane_id > 0) if rule.upper() == "LHT" else (lane_id < 0)
        return "successor" if travels_with_s else "predecessor"

    def next_slots(self, slot: Slot) -> List[Slot]:
        """Where travel leads from this lane in one step."""
        road_id, lane_id = slot
        road = self.roads.get(road_id)
        lane = self.lane(road_id, lane_id)
        if road is None or lane is None:
            return []
        end = self.forward_end(road_id, lane_id)
        link = road.find("link")
        neighbour = link.find(end) if link is not None else None
        if neighbour is None:
            return []
        target = neighbour.get("elementId")
        if target is None:
            return []
        kind = neighbour.get("elementType")
        if kind == "road":
            # A lane's links are nested: <lane><link><successor id=.../>.
            lane_link_parent = lane.find("link")
            lane_link = (
                lane_link_parent.find(end) if lane_link_parent is not None else None
            )
            target_lane = lane_link.get("id") if lane_link is not None else None
            if target_lane is None:
                return []
            return [(int(target), int(target_lane))]
        if kind == "junction":
            return [
                (connecting, links[lane_id])
                for incoming, connecting, links in self.junctions.get(int(target), [])
                if incoming == road_id and lane_id in links
            ]
        return []

    def transitions(
        self,
        walk_through: Set[int],
        max_hops: int = DEFAULT_MAX_CONNECTOR_HOPS,
    ) -> Set[Transition]:
        """Transitions between lanes a lanelet backs, contracting the rest."""
        found: Set[Transition] = set()
        for road_id in self.roads:
            if road_id in walk_through:
                continue
            for lane_id in self.driving_lane_ids(road_id):
                frontier = [((road_id, lane_id), 0)]
                seen: Set[Slot] = set()
                while frontier:
                    slot, hops = frontier.pop()
                    if hops > max_hops:
                        continue
                    for following in self.next_slots(slot):
                        if following in seen:
                            continue
                        seen.add(following)
                        if following[0] in walk_through:
                            frontier.append((following, hops + 1))
                        else:
                            found.add(((road_id, lane_id), following))
        return found


@dataclass(frozen=True)
class Succession:
    """One succession the map states, and the slot pairs that would satisfy it."""

    source_lanelet: int
    target_lanelet: int
    options: frozenset  # frozenset[Transition]


@dataclass
class RouteFidelity:
    """What the map asked for and what the file delivered."""

    stated: int = 0
    internal: int = 0
    unmapped: int = 0
    walked_through: int = 0
    emitted: int = 0
    missing: List[Succession] = field(default_factory=list)
    surplus: List[Transition] = field(default_factory=list)

    @property
    def missing_share(self) -> float:
        return (len(self.missing) / self.stated) if self.stated else 0.0


def transitions_required(
    sources: Sequence[Slot], targets: Sequence[Slot]
) -> Optional[frozenset]:
    """The slot pairs any one of which would realise a succession.

    ``sources`` and ``targets`` are the slots the two lanelets of a succession
    were emitted as. Returns ``None`` when the succession is internal to one
    road and so needs no transition at all.
    """
    options = {
        (source, target) for source in sources for target in targets if source != target
    }
    return frozenset(options) if options else None


def measure(
    map_path: Path,
    xodr_path: Path,
    mapping_path: Path,
    projector: Optional[object] = None,
) -> RouteFidelity:
    """Compare the successions the map states with the transitions emitted."""
    primary, extra = load_mapping(mapping_path)
    if projector is None:
        projector = _default_projector()
    lanelet_map = lanelet2.io.load(str(map_path), projector)
    graph = create_routing_graph(lanelet_map)
    convertible = filter_lanelets_by_subtype(
        list(lanelet_map.laneletLayer), CONVERTIBLE_LANELET_SUBTYPES
    )

    def slots_of(lanelet_id: int) -> List[Slot]:
        out: List[Slot] = []
        if lanelet_id in primary:
            out.append(primary[lanelet_id])
        out.extend(extra.get(lanelet_id, []))
        return out

    result = RouteFidelity()
    successions: List[Succession] = []
    for lanelet in convertible:
        for following in graph.following(lanelet):
            sources, targets = slots_of(lanelet.id), slots_of(following.id)
            if not sources or not targets:
                result.unmapped += 1
                continue
            options = transitions_required(sources, targets)
            if options is None:
                result.internal += 1
                continue
            successions.append(Succession(lanelet.id, following.id, options))

    network = EmittedNetwork(xodr_path)
    backed = {slot[0] for slot in primary.values()}
    for pairs in extra.values():
        backed.update(slot[0] for slot in pairs)
    walk_through = set(network.roads) - backed
    emitted = network.transitions(walk_through)
    wanted = {option for succession in successions for option in succession.options}

    result.stated = len(successions)
    result.walked_through = len(walk_through)
    result.emitted = len(emitted)
    result.missing = [s for s in successions if not (s.options & emitted)]
    result.surplus = sorted(emitted - wanted)
    return result


def _default_projector():
    """A projector that loads the map; which one it is does not matter here.

    ``lanelet2.io.load`` will not load a map without one, but this
    measurement never reads a coordinate: the successions come from the
    routing graph, which Lanelet2 derives from shared linestrings, and the
    slots come from the conversion's mapping sidecar. MGRS is used because
    its origin is inert -- different origins give identical coordinates --
    so nothing has to be resolved to load any map consistently. Pass
    ``projector=`` to :func:`measure` to read the map in the conversion's
    own frame instead.
    """
    from autoware_lanelet2_extension_python.projection import MGRSProjector

    return MGRSProjector(lanelet2.io.Origin(0.0, 0.0))


def format_report(result: RouteFidelity, examples: int = 8) -> str:
    """The measurement as text, with enough detail to chase a loss."""
    lines = [
        f"  successions the map states                     : {result.stated:6d}",
        f"  internal to one (road, lane)                   : {result.internal:6d}",
        f"  successions whose lanelet is not in the mapping: {result.unmapped:6d}",
        f"  roads with no backing lanelet (walked through) : {result.walked_through:6d}",
        f"  transitions the file states                    : {result.emitted:6d}",
        "",
        f"  MISSING (stated by the map, absent from the file): {len(result.missing):6d}"
        f"  ({100.0 * result.missing_share:.2f}%)",
        f"  EXTRA   (in the file, not stated by the map)    : {len(result.surplus):6d}",
    ]
    if result.missing:
        lines.append("")
        lines.append("  first missing successions:")
        for succession in result.missing[:examples]:
            source, target = sorted(succession.options)[0]
            lines.append(
                f"    lanelet {succession.source_lanelet} -> "
                f"{succession.target_lanelet}:"
                f" road {source[0]} lane {source[1]} ->"
                f" road {target[0]} lane {target[1]}"
            )
    if result.surplus:
        lines.append("")
        lines.append("  first surplus transitions:")
        for source, target in result.surplus[:examples]:
            lines.append(
                f"    road {source[0]} lane {source[1]} ->"
                f" road {target[0]} lane {target[1]}"
            )
    return "\n".join(lines)


def main(argv: Optional[Sequence[str]] = None) -> int:
    """CLI entry point."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("map_path", type=Path, help="the source Lanelet2 .osm")
    parser.add_argument("xodr_path", type=Path, help="the emitted .xodr")
    parser.add_argument(
        "mapping_path",
        type=Path,
        nargs="?",
        help="the conversion's mapping json; defaults to XODR with"
        " '.mapping.json' in place of its suffix",
    )
    parser.add_argument(
        "--json", action="store_true", help="print the counts as json instead"
    )
    args = parser.parse_args(list(argv) if argv is not None else None)

    mapping_path = args.mapping_path or args.xodr_path.with_suffix(".mapping.json")
    result = measure(args.map_path, args.xodr_path, mapping_path)

    if args.json:
        print(
            json.dumps(
                {
                    "stated": result.stated,
                    "internal": result.internal,
                    "unmapped": result.unmapped,
                    "walked_through": result.walked_through,
                    "emitted": result.emitted,
                    "missing": len(result.missing),
                    "surplus": len(result.surplus),
                },
                indent=2,
            )
        )
    else:
        print(f"map  : {args.map_path}")
        print(f"file : {args.xodr_path}")
        print()
        print(format_report(result))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
