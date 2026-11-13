#!/usr/bin/env python3
"""Which way does CARLA face on a short connecting road?

A synthetic connector is a few millimetres long. A probe that steps back from
a lane end to read a heading skips it entirely, so a connector whose
reference line runs out and comes back -- tangent reversed over most of its
length -- is invisible to it, and to every checker that reads the file
without evaluating the geometry.

This asks the consumer directly. For each road shorter than ``--limit`` that
carries a driving lane, it takes ``carla.Map.get_waypoint_xodr(road, lane,
length / 2)`` and compares the yaw with the yaw of the road that feeds it
(the ``<predecessor>`` road, read 5 cm inside its own end). The difference is
sorted into four classes: agrees within 5 degrees, off by 5 to 80, crosswise
(80 to 150), reversed (over 150).

It prints counts and percentiles of the angle. No road id, lanelet id or
coordinate is printed, so the output of a run on a map that cannot be shared
can be.

Needs the ``carla`` client, i.e. the ``dev`` / ``carla`` image
(``docs/docker.md``); ``carla.Map()`` runs without a server::

    python scripts/carla_connector_yaw.py OUT.xodr --label <commit>

Run it on the output before and after a change: the first line carries the
label, and "no road predecessor to compare against" is the part of the
population the comparison does not cover.
"""

from __future__ import annotations

import argparse
import sys
from collections import Counter
from typing import Any, Optional

from lxml import etree

#: Class boundaries for the yaw difference, in degrees.
AGREE_DEG = 5.0
CROSSWISE_DEG = 80.0
REVERSED_DEG = 150.0

#: How far inside the feeding road's end its heading is read, in metres, so
#: the reading is that road's geometry rather than the boundary.
FEEDER_BACK_OFF_M = 0.05

ShortRoad = tuple[int, float, list[int], Optional[tuple[int, str]]]


def read_short_roads(
    path: str, limit: float
) -> tuple[list[ShortRoad], dict[int, float]]:
    """Read the roads shorter than ``limit`` that carry a driving lane.

    Returns ``(short, lengths)``. ``short`` holds ``(road id, length, driving
    lane ids, predecessor)`` where ``predecessor`` is ``(road id, contact
    point)`` when the road's ``<predecessor>`` is a road and ``None``
    otherwise. ``lengths`` maps every road id in the file to its length.
    """
    short: list[ShortRoad] = []
    lengths: dict[int, float] = {}
    context = etree.iterparse(path, events=("end",), tag="road", huge_tree=True)
    for _event, element in context:
        road_id = int(element.get("id"))
        length = float(element.get("length") or 0.0)
        lengths[road_id] = length
        if length < limit:
            predecessor: Optional[tuple[int, str]] = None
            child = element.find("link/predecessor")
            if child is not None and child.get("elementType") == "road":
                predecessor = (
                    int(child.get("elementId")),
                    child.get("contactPoint") or "end",
                )
            lanes = sorted(
                {
                    int(lane.get("id"))
                    for lane in element.iterfind("lanes/laneSection/*/lane")
                    if lane.get("type") == "driving"
                }
            )
            if lanes:
                short.append((road_id, length, lanes, predecessor))
        element.clear()
        while element.getprevious() is not None:
            del element.getparent()[0]
    del context
    return short, lengths


def angle_between(a: float, b: float) -> float:
    """Absolute difference of two headings in degrees, in ``[0, 180]``."""
    return abs((b - a + 180.0) % 360.0 - 180.0)


def classify(delta: float) -> str:
    """Name the class a yaw difference (degrees) falls into."""
    if delta > REVERSED_DEG:
        return "reversed (over 150 deg)"
    if delta > CROSSWISE_DEG:
        return "crosswise (80 to 150 deg)"
    if delta > AGREE_DEG:
        return "off by 5 to 80 deg"
    return "agrees within 5 deg"


def percentile(values: list[float], fraction: float) -> float:
    """Nearest-rank percentile of ``values``; ``nan`` when empty."""
    if not values:
        return float("nan")
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(len(ordered) * fraction))]


def measure(
    carla_map: Any, short: list[ShortRoad], lengths: dict[int, float]
) -> tuple[Counter[str], list[float]]:
    """Compare each short road's mid-point yaw with its feeding road's.

    ``carla_map`` needs ``get_waypoint_xodr(road, lane, s)`` returning an
    object with ``transform.rotation.yaw`` or ``None``. Returns the count per
    class (including the reasons a lane could not be compared) and the yaw
    differences that were measured.
    """
    verdict: Counter[str] = Counter()
    deltas: list[float] = []
    for road_id, length, lanes, predecessor in short:
        for lane_id in lanes:
            middle = carla_map.get_waypoint_xodr(road_id, lane_id, length * 0.5)
            if middle is None:
                verdict["no waypoint on the short road"] += 1
                continue
            if predecessor is None:
                verdict["no road predecessor to compare against"] += 1
                continue
            feeder_id, contact = predecessor
            feeder_length = lengths.get(feeder_id)
            if feeder_length is None:
                verdict["predecessor not in the file"] += 1
                continue
            s = (
                max(0.0, feeder_length - FEEDER_BACK_OFF_M)
                if contact == "end"
                else min(feeder_length, FEEDER_BACK_OFF_M)
            )
            feeder = None
            for candidate_lane in (lane_id, -lane_id, 1, -1):
                feeder = carla_map.get_waypoint_xodr(feeder_id, candidate_lane, s)
                if feeder is not None:
                    break
            if feeder is None:
                verdict["no waypoint on the feeding road"] += 1
                continue
            delta = angle_between(
                feeder.transform.rotation.yaw, middle.transform.rotation.yaw
            )
            deltas.append(delta)
            verdict[classify(delta)] += 1
    return verdict, deltas


def report(
    label: str, limit: float, n_roads: int, verdict: Counter[str], deltas: list[float]
) -> str:
    """Format the result: counts with their denominator, then percentiles."""
    total = sum(verdict.values())
    lines = [
        f"===== {label} =====",
        f"roads under {limit} m carrying a driving lane: {n_roads}",
        f"driving lanes on them: {total}; compared with a feeding road: {len(deltas)}",
        "yaw on the short road against the road feeding it:",
    ]
    for key, count in sorted(verdict.items(), key=lambda item: (-item[1], item[0])):
        lines.append(f"  {count:6d}  {key}")
    if deltas:
        lines.append(
            "yaw difference (deg): "
            f"median {percentile(deltas, 0.5):.2f}, "
            f"p95 {percentile(deltas, 0.95):.2f}, max {max(deltas):.2f}"
        )
    return "\n".join(lines)


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("xodr", help="the OpenDRIVE file to load")
    parser.add_argument(
        "--label", default="probe", help="printed first; put the commit here"
    )
    parser.add_argument(
        "--limit",
        type=float,
        default=0.5,
        help="roads shorter than this many metres are measured (default 0.5)",
    )
    args = parser.parse_args(argv)

    import carla  # noqa: PLC0415 -- only the dev / carla image has it

    short, lengths = read_short_roads(args.xodr, args.limit)
    with open(args.xodr, encoding="utf-8") as handle:
        carla_map = carla.Map(args.label, handle.read())
    verdict, deltas = measure(carla_map, short, lengths)
    print(report(args.label, args.limit, len(short), verdict, deltas))
    return 0


if __name__ == "__main__":
    sys.exit(main())
