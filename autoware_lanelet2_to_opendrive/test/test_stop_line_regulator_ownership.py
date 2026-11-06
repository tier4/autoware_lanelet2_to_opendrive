"""A stop line belongs to the lane that stops at it, not to the nearest road.

``_extract_and_assign_stop_lines`` picks a stop line's road with
``find_nearest_road_for_linestring``. Nearest is not owner: Lanelet2 states
the owner explicitly, because a lanelet refers to a ``regulatory_element``
and that element names the line as its ``ref_line`` member. When the two
disagree the object hangs off a road whose lanes do not stop at it, and since
every driving lane in this output has a positive id, the ``t`` the emitter
then computes is **negative** -- the stop line is placed outside the
carriageway of the road that carries it.

Measured on 2026-11-06 against the source map, so the position itself is not
in doubt (the emitted point sits a median 0.8 mm from the source linestring
on Nishishinjuku and 2.6 cm on the customer map; a sign error would put it
``2|t|`` away):

| | Nishishinjuku | customer map |
| --- | --- | --- |
| stop-line objects | 196 | 1,464 |
| of those, outside the carriageway | 43 | 554 |
| attached to a road no regulator maps to | 96 | 777 |
| following the regulator puts it inside a lane | 28 | 423 |
| following the regulator breaks one that was inside | **0** | 1 |

The 68 of 96 that happen to land inside anyway are still the wrong road: a
consumer asking which stop line governs a lane gets nothing for the lane that
really stops there.

Stop lines the map does not regulate from any lanelet (30 on Nishishinjuku,
243 on the customer map) cannot be placed by this rule and are out of scope
here; the nearest road stays their answer.
"""

from __future__ import annotations

import json
from collections import defaultdict
from pathlib import Path

import lxml.etree as ET
import pytest

from autoware_lanelet2_to_opendrive.analyze_xodr import _STOP_LINE_OBJECT_NAME_RE


def _stop_line_roads(xodr_path: Path) -> dict[int, int]:
    """stop-line linestring id -> the road its ``<object>`` hangs off.

    Both dialects are accepted through the converter's own name pattern:
    ``target=default`` emits ``type="stopLine" name="stop_line_<id>"`` and
    ``target=carla`` emits ``type="roadMark" name="Stencil_STOP"``, which
    carries the linestring id only in ``@id``. Reading the name for the id
    found nothing at all under the CARLA target, which is the one the
    simulator consumes.
    """
    out: dict[int, int] = {}
    tree = ET.parse(str(xodr_path))
    for road in tree.getroot().iterfind("road"):
        road_id = int(road.get("id"))
        for item in road.iterfind("objects/object"):
            if not _STOP_LINE_OBJECT_NAME_RE.match(item.get("name") or ""):
                continue
            identifier = item.get("id")
            if identifier is not None and identifier.isdigit():
                out[int(identifier)] = road_id
    return out


def _regulating_lanelets(osm_path: Path) -> dict[int, set[int]]:
    """stop-line way id -> the lanelets whose regulatory element names it."""
    ref_lines: dict[int, list[int]] = defaultdict(list)
    lanelet_regulators: dict[int, list[int]] = defaultdict(list)
    for _event, element in ET.iterparse(str(osm_path), events=("end",), tag="relation"):
        kind = ""
        for tag in element.iterfind("tag"):
            if tag.get("k") == "type":
                kind = tag.get("v") or ""
                break
        if kind == "regulatory_element":
            for member in element.iterfind("member"):
                if member.get("role") == "ref_line" and member.get("type") == "way":
                    ref_lines[int(element.get("id"))].append(int(member.get("ref")))
        elif kind == "lanelet":
            for member in element.iterfind("member"):
                if member.get("type") == "relation":
                    lanelet_regulators[int(element.get("id"))].append(
                        int(member.get("ref"))
                    )
        element.clear()
    owners: dict[int, set[int]] = defaultdict(set)
    for lanelet, regulators in lanelet_regulators.items():
        for regulator in regulators:
            for way in ref_lines.get(regulator, ()):
                owners[way].add(lanelet)
    return owners


def _roads_of_lanelet(mapping_path: Path) -> dict[int, set[int]]:
    """lanelet id -> every road that carries it.

    ``lanelet_to_road_and_lane`` holds one road per lanelet, but a
    chain-merged connecting road spans several consecutive junction lanelets
    and a fork shares its prefix, so the mapping alone under-reports
    membership. ``chain_merged_roads`` states the rest.
    """
    data = json.loads(mapping_path.read_text(encoding="utf-8"))
    out: dict[int, set[int]] = defaultdict(set)
    for lanelet, value in data["lanelet_to_road_and_lane"].items():
        out[int(lanelet)].add(int(value[0]))
    for road, lanelets in (data.get("chain_merged_roads") or {}).items():
        for lanelet in lanelets:
            out[int(lanelet)].add(int(road))
    return out


@pytest.mark.xfail(
    strict=True,
    reason="find_nearest_road_for_linestring ignores the regulatory relation",
)
def test_every_regulated_stop_line_hangs_off_its_regulators_road(
    nishishinjuku_xodr: Path,
) -> None:
    xodr_path = Path(nishishinjuku_xodr)
    osm_path = xodr_path.with_name(xodr_path.stem + ".preprocessed.osm")
    mapping_path = xodr_path.with_name(xodr_path.stem + ".mapping.json")
    assert osm_path.is_file(), f"{osm_path} not produced by the conversion"
    assert mapping_path.is_file(), f"{mapping_path} not produced by the conversion"

    assigned = _stop_line_roads(xodr_path)
    owners = _regulating_lanelets(osm_path)
    roads_of = _roads_of_lanelet(mapping_path)
    assert assigned, "no stop-line objects in the conversion"

    regulated = 0
    violations = []
    for way, road_id in sorted(assigned.items()):
        regulating = owners.get(way)
        if not regulating:
            continue
        expected = {r for lanelet in regulating for r in roads_of.get(lanelet, ())}
        if not expected:
            continue
        regulated += 1
        if road_id not in expected:
            violations.append((way, road_id, sorted(expected)))

    assert regulated > 0, "no stop line in this map is regulated by a lanelet"
    assert violations == [], (
        f"{len(violations)} of {regulated} regulated stop lines hang off a road "
        f"no regulating lanelet became; first five: {violations[:5]}"
    )
