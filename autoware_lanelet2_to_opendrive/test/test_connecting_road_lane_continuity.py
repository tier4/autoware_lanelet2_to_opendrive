"""End-to-end smoke checks for connecting-road <border> emission.

Runs the full converter on nishishinjuku.osm and verifies that:
  1. At least one connecting road actually emits <lane><border>
     polynomials (i.e. BORDER mode is reached on a real map), and
  2. For each connecting-road outermost right lane that has a <border>,
     the polynomial value at s=0 falls in a plausible negative range
     for an outer right edge.

These are coarser than the conversion-vs-geometric mapping cross-check
the converter itself runs at the end of the pipeline, but they
exercise the wiring of the BORDER pinning path end-to-end without
re-implementing geometric reverse-mapping in the test.
"""

from __future__ import annotations

import os
import subprocess
import tempfile
from pathlib import Path

import lxml.etree as ET
import pytest


TOLERANCE_T = 0.05  # 5 cm


def _nishishinjuku_xodr_path() -> Path:
    worker_id = os.environ.get("PYTEST_XDIST_WORKER", "main")
    return (
        Path(tempfile.gettempdir())
        / f"nishishinjuku_carla_lane_continuity_{worker_id}.xodr"
    )


def _build_xodr() -> Path:
    xodr_path = _nishishinjuku_xodr_path()
    if xodr_path.exists():
        return xodr_path

    fixture = Path(
        "autoware_lanelet2_to_opendrive/test/data/nishishinjuku.osm"
    ).resolve()
    if not fixture.is_file():
        pytest.skip(f"{fixture} not available; cannot build XODR")

    cmd = [
        "uv",
        "run",
        "convert",
        "map=nishishinjuku",
        "target=carla",
        f"input_map_path={fixture}",
        f"output_map_path={xodr_path}",
    ]
    try:
        subprocess.run(cmd, check=True)
    except FileNotFoundError as exc:
        pytest.skip(f"converter unavailable: {exc}")
    if not xodr_path.exists():
        pytest.fail(
            f"converter exited 0 but did not produce expected output {xodr_path}"
        )
    return xodr_path


def _eval_poly(s_off: float, a: float, b: float, c: float, d: float, s: float) -> float:
    ds = s - s_off
    return a + b * ds + c * ds * ds + d * ds * ds * ds


def _lane_t_at_s(lane_elem: ET.Element, s: float) -> float | None:
    """Evaluate the lane's <border> polynomial at parameter s.

    Returns the absolute t-coordinate of the <border> active at s, or
    None if the lane has no <border> at all (e.g. WIDTH-mode lanes).
    Width-mode evaluation is intentionally not implemented here: it
    requires summing inner lanes' widths, and the tests in this file
    only need to inspect BORDER-mode connecting-road lanes.
    """
    borders = lane_elem.findall("border")
    if not borders:
        return None
    chosen = max(
        (b for b in borders if float(b.get("sOffset", "0")) <= s),
        key=lambda b: float(b.get("sOffset", "0")),
        default=None,
    )
    if chosen is None:
        return None
    return _eval_poly(
        float(chosen.get("sOffset")),
        float(chosen.get("a")),
        float(chosen.get("b")),
        float(chosen.get("c")),
        float(chosen.get("d")),
        s,
    )


def test_connecting_road_borders_emit_when_pinned() -> None:
    """At least one connecting road in nishishinjuku has <border> lanes."""
    xodr = _build_xodr()
    tree = ET.parse(str(xodr))
    root = tree.getroot()

    has_border = False
    for road in root.findall("road"):
        if int(road.get("junction", "-1")) == -1:
            continue  # skip regular roads
        for lane in road.iter("lane"):
            if lane.findall("border"):
                has_border = True
                break
        if has_border:
            break

    assert (
        has_border
    ), "No connecting road emits <lane><border>. BORDER mode is not active."


def test_connecting_road_inner_lane_starts_at_zero() -> None:
    """For each connecting-road lane -1 with <border>, t at s=0 must be ~ -width_upstream.

    This is a softer check than full mapping cross-validation but verifies
    that the BORDER polynomial endpoint constraint is being applied.
    """
    xodr = _build_xodr()
    tree = ET.parse(str(xodr))
    root = tree.getroot()

    issues: list[str] = []
    for road in root.findall("road"):
        if int(road.get("junction", "-1")) == -1:
            continue
        right = road.find(".//laneSection/right")
        if right is None:
            continue
        lane_minus_1 = next(
            (lane for lane in right.findall("lane") if lane.get("id") == "-1"), None
        )
        if lane_minus_1 is None:
            continue
        t0 = _lane_t_at_s(lane_minus_1, 0.0)
        if t0 is None:
            continue  # WIDTH lane (no override on this side); skip
        # Must be negative (outer right edge) and within reasonable range
        if t0 > 0.0 or t0 < -10.0:
            issues.append(f"road={road.get('id')} t0={t0:.3f}")

    assert not issues, f"Connecting-road -1 borders out of range at s=0: {issues[:5]}"
