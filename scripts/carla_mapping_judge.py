#!/usr/bin/env python3
"""Let CARLA arbitrate between the two lanelet -> (road, lane) mappings.

``convert`` records which road and lane each lanelet was emitted as
(``<out>.mapping.json``). ``analyze`` re-derives the same mapping from the
emitted geometry and reports every lanelet where the two disagree
("Mapping Cross-Validation"). A disagreement says one of the two is wrong,
not which.

This asks a third party. For a lanelet it takes the midpoint of the
centreline, asks ``carla.Map.get_waypoint()`` which road and lane lies
there, and records which mapping CARLA's answer matches.

The controls come first: the same question over lanelets where the two
mappings **agree**, in three populations (roads with lanes of one sign,
roads with lanes of both signs, roads inside a junction). If CARLA does not
reproduce the agreed answer there, it cannot arbitrate, and the rest is not
evidence. Roads overlap inside a junction, so that control is expected to be
lower than the other two.

It prints counts, shares and the distance from the midpoint to the waypoint.
No road id, lanelet id or coordinate is printed, so the output of a run on a
map that cannot be shared can be.

Needs the ``carla`` client, i.e. the ``dev`` / ``carla`` image
(``docs/docker.md``); ``carla.Map()`` runs without a server. The second
argument is the map the sidecar's ids refer to (the ``.preprocessed.osm``
when preprocessing ran), as for ``analyze``::

    python scripts/carla_mapping_judge.py OUT.xodr MAP.osm --label <commit>
"""

from __future__ import annotations

import argparse
import inspect
import random
import sys
from collections import Counter
from pathlib import Path
from typing import Any, Callable, Iterable, Optional

Slot = tuple[int, int]

#: Disagreement kinds, in the order they are reported.
KINDS = ("same road, other lane", "other road", "the cross-check has no answer")


def verdict(
    answer: Optional[Slot], conversion: Slot, cross_check: Optional[Slot]
) -> str:
    """Say which mapping the consumer's answer matches.

    ``answer`` is the (road, lane) the consumer reports, ``conversion`` the
    conversion-time mapping's and ``cross_check`` the geometric
    re-derivation's (``None`` when it has no answer for the lanelet).
    """
    if answer is None:
        return "carla has no waypoint there"
    if answer == conversion and answer == cross_check:
        return "carla = both"
    if answer == conversion:
        return "carla = conversion mapping"
    if answer == cross_check:
        return "carla = geometric cross-check"
    if answer[0] == conversion[0]:
        return "carla = conversion's road, a third lane"
    return "carla = another road"


def disagreement_kind(conversion: Slot, cross_check: Optional[Slot]) -> Optional[str]:
    """Classify a pair of answers; ``None`` when the two mappings agree."""
    if cross_check is None:
        return "the cross-check has no answer"
    if cross_check == conversion:
        return None
    if cross_check[0] == conversion[0]:
        return "same road, other lane"
    return "other road"


def two_sided_roads(conversion: dict[int, Slot]) -> set[int]:
    """Roads that carry lanes of both signs in the conversion-time mapping."""
    signs: dict[int, set[bool]] = {}
    for road, lane in conversion.values():
        signs.setdefault(road, set()).add(lane > 0)
    return {road for road, seen in signs.items() if len(seen) == 2}


def percentile(values: list[float], fraction: float) -> float:
    """Nearest-rank percentile of ``values``; ``nan`` when empty."""
    if not values:
        return float("nan")
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(len(ordered) * fraction))]


def judge(
    lanelets: Iterable[int],
    conversion: dict[int, Slot],
    cross_check: dict[int, Slot],
    ask: Callable[[int], tuple[Optional[Slot], Optional[float]]],
) -> tuple[Counter[str], list[float]]:
    """Tally :func:`verdict` over ``lanelets``.

    ``ask(lanelet)`` returns the consumer's ``(road, lane)`` and the distance
    from the query point to the waypoint it snapped to, or ``(None, None)``.
    """
    tally: Counter[str] = Counter()
    distances: list[float] = []
    for lanelet in lanelets:
        answer, distance = ask(lanelet)
        if answer is not None and distance is not None:
            distances.append(distance)
        tally[verdict(answer, conversion[lanelet], cross_check.get(lanelet))] += 1
    return tally, distances


def format_block(title: str, tally: Counter[str], distances: list[float]) -> str:
    """One population: its size, the snapping distance, and the shares."""
    total = sum(tally.values())
    lines = [
        f"{title}  (n={total}; waypoint to midpoint distance: "
        f"median {percentile(distances, 0.5):.2f} m, "
        f"p95 {percentile(distances, 0.95):.2f} m)"
    ]
    for key, count in sorted(tally.items(), key=lambda item: (-item[1], item[0])):
        lines.append(f"  {count:6d}  {100.0 * count / max(total, 1):5.1f}%  {key}")
    return "\n".join(lines)


def _as_slots(mapping: dict[Any, Any]) -> dict[int, Slot]:
    return {int(k): (int(v[0]), int(v[1])) for k, v in mapping.items()}


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "xodr", type=Path, help="the OpenDRIVE file (with its .mapping.json)"
    )
    parser.add_argument(
        "osm", type=Path, help="the Lanelet2 map the sidecar's ids refer to"
    )
    parser.add_argument(
        "--label", default="probe", help="printed first; put the commit here"
    )
    parser.add_argument(
        "--sample",
        type=int,
        default=3000,
        help="lanelets per control population (default 3000)",
    )
    parser.add_argument(
        "--seed", type=int, default=0, help="seed for the control samples"
    )
    args = parser.parse_args(argv)

    import carla  # noqa: PLC0415 -- only the dev / carla image has it

    from autoware_lanelet2_to_opendrive import analyze_xodr  # noqa: PLC0415
    from autoware_lanelet2_to_opendrive.road_lanelet_geo_mapping import (  # noqa: PLC0415
        build_mapping,
        parse_roads_from_xodr,
    )

    print(f"===== {args.label} =====", flush=True)
    context = analyze_xodr._load_validation_context(args.xodr, args.osm)
    if context is None:
        print("the validation context could not be loaded (no .mapping.json?)")
        return 2
    recorded = context.conv_mapping
    conversion = _as_slots(recorded.lanelet_to_road_and_lane)

    # Hand build_mapping the same optional inputs analyze gives it, on
    # whichever revision this runs: older ones take neither keyword.
    accepted = inspect.signature(build_mapping).parameters
    optional = {
        "chain_roads": getattr(recorded, "chain_merged_roads", None),
        "duplicate_roads": getattr(recorded, "exit_fork_duplicate_roads", None),
    }
    roads = parse_roads_from_xodr(args.xodr, xodr_root=context.xodr_root)
    cross_check = _as_slots(
        build_mapping(
            context.lanelet_map,
            roads,
            (context.offset_x, context.offset_y),
            recorded.xodr_sha256,
            recorded.osm_sha256,
            **{k: v for k, v in optional.items() if k in accepted},
        ).lanelet_to_road_and_lane
    )

    carla_map = carla.Map(args.label, args.xodr.read_text(encoding="utf-8"))

    def ask(lanelet_id: int) -> tuple[Optional[Slot], Optional[float]]:
        line = context.lanelet_map.laneletLayer.get(lanelet_id).centerline
        point = line[len(line) // 2]
        # OpenDRIVE is right-handed, CARLA left-handed: y changes sign.
        location = carla.Location(
            x=point.x - context.offset_x, y=-(point.y - context.offset_y), z=point.z
        )
        waypoint = carla_map.get_waypoint(
            location, project_to_road=True, lane_type=carla.LaneType.Any
        )
        if waypoint is None:
            return None, None
        return (
            waypoint.road_id,
            waypoint.lane_id,
        ), waypoint.transform.location.distance(location)

    two_sided = two_sided_roads(conversion)
    in_junction = {
        int(road.get("id"))
        for road in context.xodr_root.iter("road")
        if road.get("junction") not in (None, "-1")
    }
    kinds = {
        lanelet: disagreement_kind(slot, cross_check.get(lanelet))
        for lanelet, slot in conversion.items()
    }
    agree = sorted(lanelet for lanelet, kind in kinds.items() if kind is None)
    disagree = sorted(lanelet for lanelet, kind in kinds.items() if kind is not None)
    print(
        f"mapping entries: {len(conversion)}; the two mappings agree on {len(agree)}, "
        f"disagree on {len(disagree)}",
        flush=True,
    )

    def road_of(lanelet: int) -> int:
        return conversion[lanelet][0]

    rng = random.Random(args.seed)
    controls = (
        (
            "CONTROL 1: the two mappings agree, one-sided road outside a junction",
            [
                x
                for x in agree
                if road_of(x) not in two_sided and road_of(x) not in in_junction
            ],
        ),
        (
            "CONTROL 2: the two mappings agree, road with lanes on both sides, outside a junction",
            [
                x
                for x in agree
                if road_of(x) in two_sided and road_of(x) not in in_junction
            ],
        ),
        (
            "CONTROL 3: the two mappings agree, road inside a junction (roads overlap there)",
            [x for x in agree if road_of(x) in in_junction],
        ),
    )
    for title, population in controls:
        sample = rng.sample(population, min(args.sample, len(population)))
        print(
            "\n" + format_block(title, *judge(sample, conversion, cross_check, ask)),
            flush=True,
        )

    for kind in KINDS:
        for side, on_side in (
            ("road with lanes on both sides", True),
            ("one-sided road", False),
        ):
            for where, inside in (
                ("outside a junction", False),
                ("inside a junction", True),
            ):
                selected = [
                    x
                    for x in disagree
                    if kinds[x] == kind
                    and (road_of(x) in two_sided) == on_side
                    and (road_of(x) in in_junction) == inside
                ]
                if selected:
                    title = f"DISAGREE: {kind}; {side}, {where}"
                    print(
                        "\n"
                        + format_block(
                            title, *judge(selected, conversion, cross_check, ask)
                        ),
                        flush=True,
                    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
