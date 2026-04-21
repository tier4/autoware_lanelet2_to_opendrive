"""End-to-end tests for P0-4: per-lane speed change along s.

Two consecutive lanelets sharing a longitudinal boundary — the first with
``speed_limit=60``, the second with ``speed_limit=40`` — must produce lane
``<speed>`` elements that cover both values. This is lane-level speed
(``<lane><speed>`` entries), not ``<road><type><speed>``.

Architectural note (2026-04-21): the converter's ``find_adjacent_groups``
only groups lanelets laterally (left/right neighbours). Longitudinally
chained lanelets become separate ``<road>`` elements connected via
``<road><link><successor>``; each road carries its own single-lanelet
``LaneSection`` and therefore its own ``<lane><speed>``. As a result,
a 60→40 km/h transition along s is expressed today as two chained
roads, not as two ``<speed>`` entries with different ``sOffset`` on a
single lane. This test documents that both speed values reach the
output regardless of which of those two shapes the converter takes;
if a future refactor merges longitudinally chained lanelets into one
road/section, the per-transition emission must still satisfy this
assertion (likely via multiple ``<speed>`` entries within one lane).
"""

import subprocess
from pathlib import Path

import lxml.etree as ET

FIXTURE = (Path(__file__).parent / "data" / "speed_change_mini.osm").resolve()


def _run_convert(tmp_path: Path) -> Path:
    """Run the ``convert`` CLI on the fixture and return output path."""
    out = tmp_path / "speed_change_mini.xodr"
    # Use the minimal example_mgrs_offset map config to avoid preprocessing
    # ops that reference IDs not in this synthetic fixture (same approach as
    # walkway_mini in test_walkway_sidewalk.py).
    subprocess.run(
        [
            "uv",
            "run",
            "convert",
            "map=example_mgrs_offset",
            "target=carla",
            f"input_map_path={FIXTURE}",
            f"output_map_path={out}",
        ],
        check=True,
    )
    return out


def test_lane_speed_changes_along_s(tmp_path: Path) -> None:
    """Both 40 and 60 km/h must appear in lane-level <speed> elements.

    Whether the two lanelets land in a single road (multiple ``<speed>``
    entries on one lane with different ``sOffset``s) or in two chained
    roads (one ``<speed>`` per lane each), both speed values must reach
    the output — otherwise a speed transition between lanelets has been
    silently dropped.
    """
    out = _run_convert(tmp_path)
    root = ET.parse(str(out)).getroot()
    speeds = root.findall(".//lane/speed")
    maxes = sorted({int(float(s.get("max"))) for s in speeds})
    assert (
        40 in maxes and 60 in maxes
    ), f"expected both 40 and 60 in lane speeds along s; got {maxes}"
