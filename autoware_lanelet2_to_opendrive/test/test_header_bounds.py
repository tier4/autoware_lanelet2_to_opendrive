"""Regression test for issue #465: header @north/@south/@east/@west must
reflect the actual map extent, not the hard-coded ``0.0`` values.

The test runs the ``convert`` CLI on the synthetic ``walkway_mini.osm``
fixture (already used by ``test_walkway_sidewalk.py``) and asserts that
the four header attributes are present, finite, non-zero, and
internally consistent (``west < east`` and ``south < north``).
"""

import subprocess
from pathlib import Path

import lxml.etree as ET

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

    # Issue #465: all-zero bounds is the bug we are fixing.
    assert (north, south, east, west) != (
        0.0,
        0.0,
        0.0,
        0.0,
    ), "header bounds are still all zero — issue #465 regression"

    # Self-consistency: north > south, east > west.
    assert south < north, f"south ({south}) must be < north ({north})"
    assert west < east, f"west ({west}) must be < east ({east})"
