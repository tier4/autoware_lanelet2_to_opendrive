"""The header's bounding box must describe the file it sits in.

Issue #465 replaced hard-coded ``0.0`` values with the map's real extent,
and the test below pins that. Presence, finiteness, non-zero and ordering
are necessary but nowhere near sufficient: the extent is computed from the
raw ``pointLayer`` while every ``<geometry>`` is written with the
configured offset subtracted, so the two can describe frames kilometres
apart and still satisfy all four of those checks. ``west < east`` in
particular holds by construction for any axis-aligned fixture.

``test_header_box_contains_the_emitted_geometry`` closes that by comparing
the header against the only thing that can contradict it -- the
coordinates in the same file.
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


@pytest.mark.xfail(
    strict=True,
    reason="header bounds are computed before the offset is subtracted (#465)",
)
def test_header_box_contains_the_emitted_geometry(tmp_path: Path) -> None:
    """The header box must contain the coordinates in its own file.

    ``compute_point_layer_bounds`` scans the raw ``pointLayer``, which is in
    the projected frame *before* the configured offset is subtracted, while
    ``extract_points`` subtracts it for every reference line. Under
    ``map=example_mgrs_offset`` the offset is tens of kilometres, so the
    header describes a box the geometry is nowhere near.

    Reference-line start points are a subset of the map's points -- the
    boundary points the header is computed from lie further out, and lane
    width widens the real extent further -- so containment is the right
    relation and equality is not. The second assertion guards the other
    direction: a header made to contain everything by being enormous is not
    a fix either.
    """
    out = _run_convert(tmp_path)
    root = ET.parse(out).getroot()
    header = root.find("header")
    assert header is not None, "header element missing from output"

    north = float(header.attrib["north"])
    south = float(header.attrib["south"])
    east = float(header.attrib["east"])
    west = float(header.attrib["west"])

    points = [
        (float(geometry.attrib["x"]), float(geometry.attrib["y"]))
        for geometry in root.iter("geometry")
    ]
    assert points, "no <geometry> records in the output; the fixture did not convert"

    outside = [
        (x, y) for x, y in points if not (west <= x <= east and south <= y <= north)
    ]
    assert not outside, (
        f"{len(outside)} of {len(points)} geometry points lie outside the header"
        f" box west={west} east={east} south={south} north={north};"
        f" first is {outside[0]}"
    )

    span_x = max(x for x, _ in points) - min(x for x, _ in points)
    span_y = max(y for _, y in points) - min(y for _, y in points)
    assert (
        (east - west) <= span_x + 100.0
    ), f"header is {east - west} m wide for geometry spanning {span_x} m"
    assert (
        (north - south) <= span_y + 100.0
    ), f"header is {north - south} m tall for geometry spanning {span_y} m"
