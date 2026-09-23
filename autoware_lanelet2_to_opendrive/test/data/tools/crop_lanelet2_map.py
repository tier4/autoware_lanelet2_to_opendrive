"""Cut a self-contained geographic excerpt out of a large Lanelet2 map.

Hand-built fixtures cannot reproduce what a real map has: multi-lane
junctions, connectors that fork or chain, boundaries digitised with tens of
metres between vertices, and geometry that is only approximately what it
depicts.  A whole real map is too slow to convert in a test suite, so this
cuts a piece out of one.

Two stages:

1. **Choose the lanelets.**  Using lanelet2 itself: every lanelet whose
   centroid falls inside the bounding box, then ``--hops`` rounds of routing
   and adjacency neighbours, so a junction inside the box keeps the approach
   and exit roads that give it its shape.  Without that expansion the
   junctions at the edge of the box lose their context and the excerpt
   exercises less than the box suggests.

2. **Rewrite the XML.**  Kept lanelet relations are copied verbatim, so ids,
   coordinates and tags are exactly the source map's.  A regulatory element
   survives only when *every* lanelet it refers to survived; otherwise it is
   dropped and the reference to it is removed from the lanelets that own it,
   because a member pointing outside the excerpt makes lanelet2 refuse to
   load the result with "Failed to get id ... from map".  Ways and nodes
   follow as the transitive closure of what is left.

The output is a plain Lanelet2 OSM file: load it, convert it and preprocess
it exactly like any other map.

Example::

    python crop_lanelet2_map.py source.osm excerpt.osm \\
        --x 86783 --y 45734 --w 400 --hops 1

``--x``/``--y`` are in the projected frame of the projector below (metres),
which is what the ``--survey`` listing prints, so the usual workflow is to
run ``--survey`` first and read a box off it.
"""

from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path
from typing import Dict, Iterable, Set, Tuple

import lxml.etree as ET

# Importing both submodules registers Autoware's custom regulatory-element
# types with the lanelet2 C++ factory; without it a map that uses them fails
# to load. Same requirement as the test suite's conftest.
from autoware_lanelet2_extension_python.projection import MGRSProjector
import autoware_lanelet2_extension_python.regulatory_elements as _ll2_ext_reg  # noqa: F401
import lanelet2
from lanelet2.routing import RoutingCostDistance, RoutingGraph

#: Any consistent projection works: the cut is made on relative positions and
#: the output carries the source file's own lat/lon unchanged. This one
#: matches the test suite's conftest so a box read off one is valid for both.
_SURVEY_ORIGIN = (35.23, 139.16)


def _load(source: Path) -> Tuple[lanelet2.core.LaneletMap, RoutingGraph]:
    """Load *source* and build the vehicle routing graph over it."""
    projector = MGRSProjector(lanelet2.io.Origin(*_SURVEY_ORIGIN))
    lanelet_map = lanelet2.io.load(str(source), projector)
    rules = lanelet2.traffic_rules.create(
        lanelet2.traffic_rules.Locations.Germany,
        lanelet2.traffic_rules.Participants.Vehicle,
    )
    return lanelet_map, RoutingGraph(lanelet_map, rules, [RoutingCostDistance(0.0)])


def _centroid(lanelet: lanelet2.core.Lanelet) -> Tuple[float, float]:
    """Return the mean of a lanelet's centerline points."""
    points = list(lanelet.centerline)
    return (
        sum(p.x for p in points) / len(points),
        sum(p.y for p in points) / len(points),
    )


def select_lanelets(
    lanelet_map: lanelet2.core.LaneletMap,
    graph: RoutingGraph,
    x: float,
    y: float,
    width: float,
    hops: int,
) -> Set[int]:
    """Return the lanelet ids to keep: the box, grown by *hops* neighbour rounds."""
    keep = {
        lanelet.id
        for lanelet in lanelet_map.laneletLayer
        if _inside(_centroid(lanelet), x, y, width)
    }
    frontier = set(keep)
    for _ in range(hops):
        found: Set[int] = set()
        for lanelet_id in frontier:
            found |= _neighbours(graph, lanelet_map.laneletLayer[lanelet_id])
        frontier = found - keep
        keep |= frontier
    return keep


def _inside(point: Tuple[float, float], x: float, y: float, width: float) -> bool:
    """True when *point* lies in the half-open square anchored at (x, y)."""
    return x <= point[0] < x + width and y <= point[1] < y + width


def _neighbours(graph: RoutingGraph, lanelet: lanelet2.core.Lanelet) -> Set[int]:
    """Return every lanelet reachable from *lanelet* in one routing or lateral step.

    Non-vehicle lanelets (crosswalks, walkways) are absent from the vehicle
    routing graph, so they only ever enter the excerpt through the bounding
    box, never through this expansion.
    """
    found: Set[int] = set()
    for many in (graph.following, graph.previous):
        found.update(other.id for other in many(lanelet))
    for one in (graph.left, graph.right, graph.adjacentLeft, graph.adjacentRight):
        other = one(lanelet)
        if other is not None:
            found.add(other.id)
    return found


def _tags(element: ET._Element) -> Dict[str, str]:
    """Return an OSM element's ``<tag k= v=>`` children as a dict."""
    return {t.get("k"): t.get("v") for t in element.findall("tag")}


def _copy(element: ET._Element) -> ET._Element:
    """Return an independent copy of *element*, subtree included."""
    return ET.fromstring(ET.tostring(element))


def write_excerpt(source: Path, destination: Path, keep: Set[int]) -> Dict[str, int]:
    """Write the OSM subset induced by *keep* and return a census of it."""
    root = ET.parse(str(source)).getroot()
    nodes: Dict[str, ET._Element] = {}
    ways: Dict[str, ET._Element] = {}
    relations: Dict[str, ET._Element] = {}
    for element in root:
        if element.tag == "node":
            nodes[element.get("id")] = element
        elif element.tag == "way":
            ways[element.get("id")] = element
        elif element.tag == "relation":
            relations[element.get("id")] = element

    keep_ids = {str(i) for i in keep}
    lanelets = {rid: relations[rid] for rid in keep_ids if rid in relations}

    referenced = {
        member.get("ref")
        for lanelet in lanelets.values()
        for member in lanelet.findall("member")
        if member.get("role") == "regulatory_element"
    }
    kept_regs: Set[str] = set()
    dropped_regs: Set[str] = set()
    for rid in referenced:
        element = relations.get(rid)
        if element is not None and _fully_inside(element, keep_ids, ways):
            kept_regs.add(rid)
        else:
            dropped_regs.add(rid)

    keep_ways = {
        member.get("ref")
        for element in list(lanelets.values()) + [relations[r] for r in kept_regs]
        for member in element.findall("member")
        if member.get("type") == "way"
    } & set(ways)
    keep_nodes = {
        nd.get("ref") for wid in keep_ways for nd in ways[wid].findall("nd")
    } & set(nodes)

    out = ET.Element("osm", generator="crop_lanelet2_map")
    meta = root.find("MetaInfo")
    if meta is not None:
        out.append(_copy(meta))
    for nid in sorted(keep_nodes, key=int):
        out.append(_copy(nodes[nid]))
    for wid in sorted(keep_ways, key=int):
        out.append(_copy(ways[wid]))
    for rid in sorted(kept_regs, key=int):
        out.append(_copy(relations[rid]))
    for rid in sorted(lanelets, key=int):
        lanelet = _copy(lanelets[rid])
        for member in list(lanelet.findall("member")):
            if (
                member.get("role") == "regulatory_element"
                and member.get("ref") in dropped_regs
            ):
                lanelet.remove(member)
        out.append(lanelet)

    ET.ElementTree(out).write(
        str(destination), encoding="UTF-8", xml_declaration=True, pretty_print=True
    )

    census = Counter(_tags(e).get("subtype") for e in lanelets.values())
    return {
        "lanelets": len(lanelets),
        "regulatory_elements_kept": len(kept_regs),
        "regulatory_elements_dropped": len(dropped_regs),
        "ways": len(keep_ways),
        "nodes": len(keep_nodes),
        **{
            f"subtype:{k}": v
            for k, v in sorted(census.items(), key=lambda kv: str(kv[0]))
        },
    }


def _fully_inside(
    element: ET._Element, keep_ids: Set[str], ways: Dict[str, ET._Element]
) -> bool:
    """True when every primitive a regulatory element refers to survived the cut."""
    for member in element.findall("member"):
        if member.get("type") == "relation" and member.get("ref") not in keep_ids:
            return False
        if member.get("type") == "way" and member.get("ref") not in ways:
            return False
    return True


def survey(lanelet_map, graph, step: float = 50.0) -> None:  # pragma: no cover
    """Print, per grid cell, the structures a crop there would contain.

    Intended to be eyeballed: pick a cell with connectors that fork and chain
    and with non-vehicle lanelets, then pass its corner as ``--x``/``--y``.
    """
    cells: Dict[Tuple[int, int], Counter] = {}
    turning = {
        lanelet.id
        for lanelet in lanelet_map.laneletLayer
        if "turn_direction" in lanelet.attributes
    }
    for lanelet in lanelet_map.laneletLayer:
        cx, cy = _centroid(lanelet)
        cell = cells.setdefault((int(cx // step), int(cy // step)), Counter())
        cell["lanelets"] += 1
        subtype = (
            lanelet.attributes["subtype"] if "subtype" in lanelet.attributes else ""
        )
        if subtype not in ("road", ""):
            cell["non_vehicle"] += 1
        if lanelet.id in turning:
            cell["connectors"] += 1
            following = [other.id for other in graph.following(lanelet)]
            if len(following) > 1:
                cell["forking"] += 1
            if any(other in turning for other in following):
                cell["chained"] += 1
        if len(graph.previous(lanelet)) > 1:
            cell["merging"] += 1
    for (ix, iy), counts in sorted(
        cells.items(), key=lambda kv: -kv[1]["forking"] * 4 - kv[1]["chained"]
    )[:20]:
        print(f"x={ix * step:.0f} y={iy * step:.0f}  {dict(counts)}")


def main(argv: Iterable[str] | None = None) -> int:
    """CLI entry point."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("source", type=Path, help="large Lanelet2 .osm to cut from")
    parser.add_argument("destination", type=Path, nargs="?", help="excerpt .osm")
    parser.add_argument("--x", type=float, help="box south-west corner, easting (m)")
    parser.add_argument("--y", type=float, help="box south-west corner, northing (m)")
    parser.add_argument("--w", type=float, help="box side length (m)")
    parser.add_argument(
        "--hops",
        type=int,
        default=1,
        help="rounds of routing/adjacency neighbours added around the box",
    )
    parser.add_argument(
        "--survey",
        action="store_true",
        help="print candidate boxes with what each contains, then exit",
    )
    args = parser.parse_args(list(argv) if argv is not None else None)

    lanelet_map, graph = _load(args.source)
    print(f"loaded {len(lanelet_map.laneletLayer)} lanelets from {args.source}")

    if args.survey:
        survey(lanelet_map, graph)
        return 0

    missing = [n for n in ("x", "y", "w") if getattr(args, n) is None]
    if missing or args.destination is None:
        parser.error(
            "destination and --x/--y/--w are required unless --survey is given"
        )

    keep = select_lanelets(lanelet_map, graph, args.x, args.y, args.w, args.hops)
    census = write_excerpt(args.source, args.destination, keep)
    print(f"wrote {args.destination}")
    print(json.dumps(census, indent=2))
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
