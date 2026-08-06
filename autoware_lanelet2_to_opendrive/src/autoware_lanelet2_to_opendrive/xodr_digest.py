"""Structure snapshot (digest) extractor for OpenDRIVE (.xodr) files.

The repository keeps no byte-exact golden ``.xodr`` files: conversion output
is gitignored and regenerated on every run, and floating-point formatting,
element ordering, and the header's ``date`` attribute all vary between runs
even when the semantic content is unchanged. That makes plain byte diffing
useless for showing "this change only touched the intended part of the
output".

This tool produces a normalized JSON digest instead: coordinates are rounded
to millimeter precision, ``date`` and the generator-version comment are
dropped, and roads/junctions are explicitly sorted by ``id``. Diffing two
digests then shows exactly which roads/junctions/counts changed, and nothing
else changed.

The file is streamed with ``xml.etree.ElementTree.iterparse`` and each
``<road>``/``<junction>`` subtree is ``clear()``-ed once processed, so the
whole document is never held in memory (the real artifact is 6.1MB and is
expected to grow).

Usage:
    uv run xodr-digest input.xodr
    uv run xodr-digest input.xodr --output digest.json
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
import xml.etree.ElementTree as ET
from collections import Counter
from pathlib import Path
from typing import Any

# Coordinate fields (currently only the header's north/south/east/west
# bounds) are rounded to millimeter precision. The real artifact emits full
# double-precision digits (e.g. ``15.390938860426232``) that shift in their
# last digits purely due to floating-point operation order, which would
# otherwise show up as noise in every digest diff.
COORDINATE_DECIMALS = 3

# length / s fields are rounded to centimeter precision.
LENGTH_DECIMALS = 2

# Geometry primitive tag names that can appear as the single child of a
# <planView><geometry> element.
_GEOMETRY_TYPES = frozenset({"line", "spiral", "arc", "poly3", "paramPoly3"})

_LANE_SIDES = ("left", "center", "right")


def _round_coord(value: str | None) -> float:
    return round(float(value or 0.0), COORDINATE_DECIMALS)


def _round_length(value: str | None) -> float:
    return round(float(value or 0.0), LENGTH_DECIMALS)


def _is_road_level_link(link_child: ET.Element) -> bool:
    """Tell a road-level from a lane-level ``<predecessor>``/``<successor>``.

    Both link kinds reuse the identical tag name:

    - road-level: ``<road><link><predecessor elementType="road" elementId=".."/>``
    - lane-level: ``<lane><link><predecessor id=".."/>``

    A road-level link always carries an ``elementType`` attribute; a
    lane-level link never does. That attribute is therefore the only
    reliable discriminator (path-based heuristics would also work here, but
    would silently break if either element were ever nested differently).
    """
    return link_child.get("elementType") is not None


def _new_digest_state() -> dict[str, Any]:
    """Create the mutable accumulator folded into while roads are streamed."""
    return {
        "connection_total": 0,
        "road_level_link": {"predecessor": 0, "successor": 0},
        "lane_level_link": {"predecessor": 0, "successor": 0},
        "signal_count": 0,
        "object_count": 0,
        "object_by_type": {},
        "lane_total": 0,
        "lane_by_side": {"left": 0, "center": 0, "right": 0},
        "lane_by_type": {},
        "lane_section_total": 0,
        "lane_count_per_road": {},
        "geometry_total": 0,
        "geometry_by_type": {},
        "segments_per_road": [],
        # Raw (unrounded) per-road lengths. Kept unrounded so that (a) the
        # aggregate total/median/mean can be rounded once at the end instead
        # of accumulating rounding error across hundreds of roads, and (b)
        # the near-zero-length buckets below classify roads by their actual
        # value rather than by a value that may have shifted across a
        # bucket boundary due to 2dp rounding.
        "road_lengths_raw": [],
        "length_zero_count": 0,
        "length_exactly_0_01_count": 0,
        "length_lt_0_1_count": 0,
        "length_lt_1_0_count": 0,
    }


def _process_road(road_elem: ET.Element, state: dict[str, Any]) -> dict[str, Any]:
    """Extract one road's digest entry and fold its counters into `state`.

    Returns the ``{"id", "length", "lane_count", "geometry_types"}`` entry
    for the ``roads`` array; every other statistic is accumulated in-place
    into `state` so it can be summarized once all roads have been seen.
    """
    road_id = int(road_elem.get("id", "-1"))
    # `raw_length` is the unrounded value straight from the XML attribute.
    # It drives every length classification below; `length` (rounded to
    # 2dp) is only for the per-road display entry returned at the bottom.
    raw_length = float(road_elem.get("length") or 0.0)
    length = round(raw_length, LENGTH_DECIMALS)

    for tag in ("predecessor", "successor"):
        for link_child in road_elem.iter(tag):
            bucket = (
                state["road_level_link"]
                if _is_road_level_link(link_child)
                else state["lane_level_link"]
            )
            bucket[tag] += 1

    geometry_types: list[str] = []
    for geometry in road_elem.findall("planView/geometry"):
        for child in geometry:
            if child.tag in _GEOMETRY_TYPES:
                geometry_types.append(child.tag)
                state["geometry_by_type"][child.tag] = (
                    state["geometry_by_type"].get(child.tag, 0) + 1
                )
                break
    state["geometry_total"] += len(geometry_types)
    state["segments_per_road"].append(len(geometry_types))

    lane_count = 0
    for lane_section in road_elem.iter("laneSection"):
        state["lane_section_total"] += 1
        for side in _LANE_SIDES:
            side_elem = lane_section.find(side)
            if side_elem is None:
                continue
            for lane in side_elem.findall("lane"):
                lane_count += 1
                state["lane_by_side"][side] += 1
                lane_type = lane.get("type", "")
                state["lane_by_type"][lane_type] = (
                    state["lane_by_type"].get(lane_type, 0) + 1
                )
    state["lane_total"] += lane_count
    state["lane_count_per_road"][lane_count] = (
        state["lane_count_per_road"].get(lane_count, 0) + 1
    )

    for _signal in road_elem.iter("signal"):
        state["signal_count"] += 1

    for obj in road_elem.iter("object"):
        state["object_count"] += 1
        obj_type = obj.get("type", "")
        state["object_by_type"][obj_type] = state["object_by_type"].get(obj_type, 0) + 1

    state["road_lengths_raw"].append(raw_length)
    if raw_length == 0.0:
        state["length_zero_count"] += 1
    # Strict/exact match against the raw value: this count exists to detect
    # roads where the converter wrote the literal lower-clamp value 0.01,
    # not roads whose length merely rounds to 0.01 at 2dp (e.g. 0.0051 would
    # round to 0.01 but was never actually clamped to it).
    if raw_length == 0.01:
        state["length_exactly_0_01_count"] += 1
    # Cumulative counts (not disjoint buckets): `lt_1_0` includes everything
    # `lt_0_1` includes, plus more. A range like "[0.1, 1.0)" can be
    # recovered by the caller as `lt_1_0_count - lt_0_1_count`. Comparing
    # against `raw_length` (not the 2dp-rounded `length`) avoids values that
    # sit just below a threshold (e.g. 0.0999999999) from being pushed
    # across it by rounding.
    if raw_length < 0.1:
        state["length_lt_0_1_count"] += 1
    if raw_length < 1.0:
        state["length_lt_1_0_count"] += 1

    return {
        "id": road_id,
        "length": length,
        "lane_count": lane_count,
        "geometry_types": geometry_types,
    }


def _length_summary(raw_lengths: list[float]) -> dict[str, float]:
    """Summarize road lengths. Safe on an empty list (0-road ``.xodr``).

    `raw_lengths` must hold the *unrounded* per-road lengths. Each statistic
    is computed from the raw values and rounded exactly once at the end.
    Rounding each road's length first and summing the rounded values instead
    would accumulate rounding error across hundreds of roads (measured
    0.03m drift over 693 roads on the real artifact) and is not equivalent.
    """
    if not raw_lengths:
        return {
            "total_m": 0.0,
            "median_m": 0.0,
            "mean_m": 0.0,
            "min_m": 0.0,
            "max_m": 0.0,
        }
    return {
        "total_m": round(sum(raw_lengths), LENGTH_DECIMALS),
        "median_m": round(statistics.median(raw_lengths), LENGTH_DECIMALS),
        "mean_m": round(statistics.mean(raw_lengths), LENGTH_DECIMALS),
        "min_m": round(min(raw_lengths), LENGTH_DECIMALS),
        "max_m": round(max(raw_lengths), LENGTH_DECIMALS),
    }


def _segments_summary(counts: list[int]) -> dict[str, int]:
    """Summarize per-road planView segment counts. Safe on an empty list.

    The mode is resolved deterministically (smallest segment count among
    ties) rather than relying on ``set()`` iteration order.
    """
    if not counts:
        return {"min": 0, "max": 0, "mode": 0, "mode_count": 0}
    frequency = Counter(counts)
    max_freq = max(frequency.values())
    mode_value = min(value for value, freq in frequency.items() if freq == max_freq)
    return {
        "min": min(counts),
        "max": max(counts),
        "mode": mode_value,
        "mode_count": frequency[mode_value],
    }


def build_digest(xodr_path: Path) -> dict[str, Any]:
    """Stream-parse `xodr_path` and return a normalized structure digest.

    Uses ``iterparse`` and calls ``elem.clear()`` after each ``<road>`` /
    ``<junction>`` / ``<header>`` is fully processed, so the file is never
    held in memory as a whole.
    """
    state = _new_digest_state()
    roads: list[dict[str, Any]] = []
    junctions: list[dict[str, Any]] = []
    header: dict[str, Any] = {}
    geo_reference: str | None = None

    for _event, elem in ET.iterparse(str(xodr_path), events=("end",)):
        tag = elem.tag
        if tag == "header":
            # `date` (generation timestamp) is deliberately not read: it is
            # non-deterministic across runs and must not leak into the
            # digest. Comments (the generator-version string at the top of
            # the file) are never yielded by iterparse's default event set,
            # so they are excluded automatically.
            header = {
                "revMajor": int(elem.get("revMajor", "0")),
                "revMinor": int(elem.get("revMinor", "0")),
                "name": elem.get("name", ""),
                "version": elem.get("version", ""),
                "north": _round_coord(elem.get("north")),
                "south": _round_coord(elem.get("south")),
                "east": _round_coord(elem.get("east")),
                "west": _round_coord(elem.get("west")),
            }
            geo_ref_elem = elem.find("geoReference")
            if geo_ref_elem is not None and geo_ref_elem.text:
                geo_reference = geo_ref_elem.text.strip()
            elem.clear()
        elif tag == "road":
            roads.append(_process_road(elem, state))
            elem.clear()
        elif tag == "junction":
            junction_id = int(elem.get("id", "-1"))
            connection_count = len(elem.findall("connection"))
            state["connection_total"] += connection_count
            junctions.append({"id": junction_id, "connection_count": connection_count})
            elem.clear()

    # Explicit sort even though real files appear to already be in id order:
    # nothing in the converter guarantees that, and an accidental future
    # reorder must not show up as a digest diff.
    roads.sort(key=lambda r: r["id"])
    junctions.sort(key=lambda j: j["id"])

    return {
        "header": header,
        "geo_reference": geo_reference,
        "counts": {
            "road": len(roads),
            "junction": len(junctions),
            "connection": state["connection_total"],
            "road_level_link": state["road_level_link"],
            "lane_level_link": state["lane_level_link"],
            "signal": state["signal_count"],
            "object": {
                "total": state["object_count"],
                "by_type": dict(sorted(state["object_by_type"].items())),
            },
        },
        "length": {
            **_length_summary(state["road_lengths_raw"]),
            "zero_count": state["length_zero_count"],
            # Strict/exact match against the raw length value (see
            # `_process_road`), not a match against the rounded value.
            "exactly_0_01_count": state["length_exactly_0_01_count"],
            # Cumulative counts: `lt_1_0_count` includes `lt_0_1_count`. A
            # disjoint range like [0.1, 1.0) is `lt_1_0_count -
            # lt_0_1_count`, not stored separately.
            "lt_0_1_count": state["length_lt_0_1_count"],
            "lt_1_0_count": state["length_lt_1_0_count"],
        },
        "geometry": {
            "total_segments": state["geometry_total"],
            "by_type": dict(sorted(state["geometry_by_type"].items())),
            "segments_per_road": _segments_summary(state["segments_per_road"]),
        },
        "lane": {
            "total": state["lane_total"],
            "by_side": state["lane_by_side"],
            "by_type": dict(sorted(state["lane_by_type"].items())),
            "count_per_road_distribution": {
                str(k): v for k, v in sorted(state["lane_count_per_road"].items())
            },
            "lane_section_count": state["lane_section_total"],
        },
        "junctions": junctions,
        "roads": roads,
    }


def format_digest(digest: dict[str, Any]) -> str:
    """Render `digest` as JSON with each `roads`/`junctions` entry on one line.

    ``json.dumps(..., indent=2)`` would spread every array element across
    several lines, which makes a `diff` between two digests nearly
    unreadable when only a handful of roads changed. This keeps the rest of
    the document pretty-printed but renders each `roads`/`junctions` entry
    as one compact JSON object per line.
    """
    array_keys = ("junctions", "roads")
    scalar_part = {k: v for k, v in digest.items() if k not in array_keys}

    # `body` is "{\n  ...\n}"; strip the outer braces so the scalar keys can
    # be spliced into the hand-built document below, ahead of the array
    # keys rendered one-entry-per-line.
    body = json.dumps(scalar_part, indent=2)
    inner = body[1:-1].strip("\n")

    lines = ["{"]
    if inner:
        lines.append(inner + ",")
    for key in array_keys:
        items = digest.get(key, [])
        if not items:
            lines.append(f'  "{key}": [],')
            continue
        lines.append(f'  "{key}": [')
        lines.append(",\n".join("    " + json.dumps(item) for item in items))
        lines.append("  ],")
    lines[-1] = lines[-1].rstrip(",")
    lines.append("}")
    return "\n".join(lines) + "\n"


def main() -> None:
    """Entry point for the `xodr-digest` command."""
    parser = argparse.ArgumentParser(
        prog="xodr-digest",
        description=(
            "Extract a normalized structure digest (counts, per-road "
            "summary, header) from a .xodr file, for diff-based review of "
            "conversion output in the absence of golden files."
        ),
    )
    parser.add_argument("xodr", type=Path, help=".xodr file to summarize")
    parser.add_argument(
        "--output",
        "-o",
        type=Path,
        default=None,
        help="Write the digest JSON to this file instead of stdout.",
    )
    args = parser.parse_args()

    if not args.xodr.exists():
        print(f"Error: file not found: {args.xodr}", file=sys.stderr)
        sys.exit(2)

    digest = build_digest(args.xodr)
    text = format_digest(digest)

    if args.output is not None:
        args.output.write_text(text, encoding="utf-8")
    else:
        sys.stdout.write(text)


if __name__ == "__main__":
    main()
