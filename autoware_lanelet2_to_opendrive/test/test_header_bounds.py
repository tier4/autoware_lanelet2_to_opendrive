"""Regression test for issue #465: header @north/@south/@east/@west must
reflect the actual map extent, not the hard-coded ``0.0`` values.

The test runs the ``convert`` CLI on the synthetic ``walkway_mini.osm``
fixture (already used by ``test_walkway_sidewalk.py``) and asserts that
the four header attributes are present, finite, non-zero, and
internally consistent (``west < east`` and ``south < north``).
"""

import math
import subprocess
from pathlib import Path

import lxml.etree as ET
import pytest

FIXTURE = (Path(__file__).parent / "data" / "walkway_mini.osm").resolve()


def _run_convert(tmp_path: Path) -> Path:
    out = tmp_path / "walkway_mini.xodr"
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


def test_header_bounding_box_reflects_map_extent(tmp_path: Path) -> None:
    """Header @north/@south/@east/@west must be derived from pointLayer."""
    out = _run_convert(tmp_path)
    header = ET.parse(out).getroot().find("header")
    assert header is not None, "header element missing from output"

    north = float(header.attrib["north"])
    south = float(header.attrib["south"])
    east = float(header.attrib["east"])
    west = float(header.attrib["west"])

    # Issue #465: every axis must be finite (not NaN/inf — `nan != 0.0` is
    # true, so the non-zero check below would otherwise let a broken header
    # slip through) and non-zero (catches partial-fix regressions where only
    # some of north/south/east/west get populated).
    for name, value in (
        ("north", north),
        ("south", south),
        ("east", east),
        ("west", west),
    ):
        assert math.isfinite(value), f"header @{name} is not finite ({value})"
        assert value != 0.0, f"header @{name} is 0.0 — issue #465 regression"

    # Self-consistency: north > south, east > west.
    assert south < north, f"south ({south}) must be < north ({north})"
    assert west < east, f"west ({west}) must be < east ({east})"


@pytest.mark.xfail(strict=True, reason="header bbox frame, #465 follow-up")
def test_header_bounding_box_shares_the_geometry_frame(tmp_path: Path) -> None:
    """The bbox and ``<geometry>`` must be expressed in the same frame.

    ``main.py`` fills ``west/east/south/north`` from
    ``compute_point_layer_bounds(self.lanelet_map)``, which returns projected
    coordinates taken *before* the export offset is subtracted, while
    ``<geometry>`` is written after it. With a non-zero offset the two
    disagreed by exactly that offset: nishishinjuku declared
    ``west=81167.85 east=82279.96`` for geometry running -485.98..618.63.
    esmini, RoadRunner and the qc bundle all read these attributes.
    """
    out = _run_convert(tmp_path)
    root = ET.parse(out).getroot()
    header = root.find("header")
    assert header is not None, "header element missing from output"

    xs = [float(g.attrib["x"]) for g in root.iter("geometry")]
    ys = [float(g.attrib["y"]) for g in root.iter("geometry")]
    assert xs, "output has no <geometry> elements to compare against"

    # ``<geometry>`` records segment start points and a fitted spline may
    # bulge slightly outside the source points, so the two extents are not
    # identical -- only in the same frame. A frame mismatch is the size of an
    # offset (1e4..1e5 m), so the margin only has to exceed a fitting error.
    margin = 100.0
    west, east = float(header.attrib["west"]), float(header.attrib["east"])
    south, north = float(header.attrib["south"]), float(header.attrib["north"])

    assert west - margin <= min(xs), f"west={west} vs min geometry x={min(xs)}"
    assert max(xs) <= east + margin, f"east={east} vs max geometry x={max(xs)}"
    assert south - margin <= min(ys), f"south={south} vs min geometry y={min(ys)}"
    assert max(ys) <= north + margin, f"north={north} vs max geometry y={max(ys)}"
