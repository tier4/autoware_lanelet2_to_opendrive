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
    raise NotImplementedError


def angle_between(a: float, b: float) -> float:
    """Absolute difference of two headings in degrees, in ``[0, 180]``."""
    raise NotImplementedError


def classify(delta: float) -> str:
    """Name the class a yaw difference (degrees) falls into."""
    raise NotImplementedError


def percentile(values: list[float], fraction: float) -> float:
    """Nearest-rank percentile of ``values``; ``nan`` when empty."""
    raise NotImplementedError


def measure(
    carla_map: Any, short: list[ShortRoad], lengths: dict[int, float]
) -> tuple[Counter[str], list[float]]:
    """Compare each short road's mid-point yaw with its feeding road's.

    ``carla_map`` needs ``get_waypoint_xodr(road, lane, s)`` returning an
    object with ``transform.rotation.yaw`` or ``None``. Returns the count per
    class (including the reasons a lane could not be compared) and the yaw
    differences that were measured.
    """
    raise NotImplementedError


def report(
    label: str, limit: float, n_roads: int, verdict: Counter[str], deltas: list[float]
) -> str:
    """Format the result: counts with their denominator, then percentiles."""
    raise NotImplementedError


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
