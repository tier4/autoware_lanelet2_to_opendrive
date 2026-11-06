"""A stop line must sit on the side of the road its driving lanes are on.

A road object's ``@t`` is signed off the reference line, and every driving
lane in this output carries a positive id (the roads are ``rule="LHT"``), so
a stop line emitted with ``t < 0`` is outside the carriageway *by
construction* -- on the far side of the reference line from every lane that
could stop at it. A simulator asking which stop line governs a lane finds
nothing there.

``e6daee23`` fixed the stop lines the map regulates from a lanelet: the road
is now chosen among the regulating lanelets' roads by the side its signed
half-width puts the line on. It cannot reach a stop line **no lanelet
regulates** -- 30 of 196 on Nishi-Shinjuku, 243 of 1,464 on the customer map
-- because there is no relation to follow. Those keep
``find_nearest_road_for_linestring``, and nearest does not mean "on the
right side of".

That fallback is fixable by the same construction, and the alternative road
exists. Measured 2026-11-06 over every road within 30 m of each such line:

| | Nishi-Shinjuku | customer map |
| --- | --- | --- |
| stop lines no lanelet regulates | 30 | 243 |
| of those, emitted with ``t < 0`` | 14 | 123 |
| that another road would hold **inside** its driving lanes | **14** | **123** |
| median distance to that road's nearest sample | 1.53 m | 1.49 m |

So the population is reachable in full, and this asserts the property
directly rather than the mechanism: no stop-line object sits on the wrong
side of its own road.

The side is read from the document (the sign of the road's driving lane ids)
rather than assumed, so the test still means what it says if the traffic
rule or the lane numbering changes.
"""

from __future__ import annotations

from pathlib import Path

import lxml.etree as ET

from autoware_lanelet2_to_opendrive.analyze_xodr import _STOP_LINE_OBJECT_NAME_RE


def _driving_side(road: ET._Element) -> int:
    """+1 if the road's driving lanes have positive ids, -1 if negative, 0 if none."""
    sides = set()
    for section in road.iterfind("lanes/laneSection"):
        for which in ("left", "right"):
            container = section.find(which)
            if container is None:
                continue
            for lane in container.iterfind("lane"):
                if lane.get("type") != "driving":
                    continue
                identifier = int(lane.get("id"))
                if identifier:
                    sides.add(1 if identifier > 0 else -1)
    if len(sides) == 1:
        return sides.pop()
    return 0


def test_no_stop_line_sits_on_the_wrong_side_of_its_road(
    nishishinjuku_xodr: Path,
) -> None:
    tree = ET.parse(str(nishishinjuku_xodr))

    total = 0
    wrong_side = []
    for road in tree.getroot().iterfind("road"):
        side = _driving_side(road)
        for item in road.iterfind("objects/object"):
            if not _STOP_LINE_OBJECT_NAME_RE.match(item.get("name") or ""):
                continue
            total += 1
            if side == 0:
                continue
            t = float(item.get("t") or 0.0)
            if t * side < 0.0:
                wrong_side.append((item.get("id"), int(road.get("id")), round(t, 3)))

    assert total > 0, "no stop-line objects in the conversion"
    assert wrong_side == [], (
        f"{len(wrong_side)} of {total} stop-line objects sit on the far side of "
        f"their road's reference line from its driving lanes; first five: "
        f"{sorted(wrong_side)[:5]}"
    )
