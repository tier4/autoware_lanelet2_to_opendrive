"""Tests for the two tools that build a fixture out of a real map.

Both live under ``test/data/tools/`` as standalone scripts rather than package
modules, because they are operator tools run by hand and must not become an
import dependency of the converter.  They are loaded here the same way, by
path.

The property these tests exist for is **de-identification**: the whole point
of ``rebuild_as_synthetic.py`` is that a fixture derived from a map which
cannot enter the repository becomes committable.  A tool that keeps the source
position is worse than no tool, because the output *looks* anonymised.
"""

from __future__ import annotations

import importlib.util
import re
import sys
from collections import Counter
from pathlib import Path
from typing import Dict, Set

import pytest

_TOOLS = Path(__file__).parent / "data" / "tools"
_DATA = Path(__file__).parent / "data"
_REFERENCE_MAP = _DATA / "nishishinjuku.osm"


def _load(name: str):
    """Import a tool by path, with its own directory importable beside it."""
    path = _TOOLS / f"{name}.py"
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.path.insert(0, str(_TOOLS))  # rebuild_as_synthetic imports beside it
    try:
        spec.loader.exec_module(module)
    finally:
        sys.path.remove(str(_TOOLS))
    return module


rebuild_tool = _load("rebuild_as_synthetic")
crop_tool = _load("crop_lanelet2_map")


# --------------------------------------------------------------------------
# What every tag key in the reference map is, and what the rebuild does to it.
#
# This is the guard that makes the pass-through default safe.  The tool keeps
# unrecognised tags on purpose -- an allowlist would silently drop a tag the
# converter learns to read later -- but "keep what I do not recognise" is only
# safe while nothing unrecognised carries the source's position.  So every key
# that actually occurs is classified here, and
# ``test_every_tag_key_in_the_reference_map_is_classified`` fails on a key
# that is not, rather than letting it through unexamined.
# --------------------------------------------------------------------------

#: Tag values that are a position in the source map.  Keeping one keeps the
#: source location whatever happens to ``lat``/``lon``.
_POSITIONAL_TAGS: Set[str] = {"mgrs_code", "local_x", "local_y", "ele"}

#: Tag values that are an id in the source map.
_ID_TAGS: Set[str] = {"traffic_light_id", "intersection_area"}

#: Tags the converter reads for meaning, not for position.  These must survive
#: the rebuild unchanged, or the fixture stops behaving like its source.
_SEMANTIC_TAGS: Set[str] = {
    "area",
    "arrow",
    "color",
    "fms_lane_passable",
    "height",
    "lane_change",
    "lane_change:left",
    "lane_change:right",
    "location",
    "one_way",
    "participant:pedestrian",
    "participant:vehicle",
    "speed_limit",
    "subtype",
    "turn_direction",
    "turn_signal_distance",
    "type",
    "width",
}


def _tag_keys(osm: Path) -> Counter:
    """Every ``<tag k=>`` key in *osm*, with its count."""
    raw = osm.read_text(encoding="utf-8")
    return Counter(re.findall(r'<tag k="([^"]+)"', raw))


def test_every_tag_key_in_the_reference_map_is_classified() -> None:
    """A tag key nobody has looked at must not reach the output silently.

    The rebuild passes unrecognised tags through, so this is the check that
    decides whether that default is safe.  When this fails, classify the new
    key above: positional keys have to be dropped or transformed, id keys
    dropped, semantic keys passed through.
    """
    known = _POSITIONAL_TAGS | _ID_TAGS | _SEMANTIC_TAGS
    unclassified = sorted(set(_tag_keys(_REFERENCE_MAP)) - known)
    assert not unclassified, (
        "these tag keys occur in the reference map and are not classified "
        f"in test_fixture_build_tools: {unclassified}"
    )


# --------------------------------------------------------------------------
# The rebuild itself, on a small synthesised map rather than the 11 MB one.
# --------------------------------------------------------------------------

_SOURCE_MGRS = "54SUE815501"
_SOURCE_ELE = 34.137


def _write_source(path: Path) -> None:
    """A two-way map carrying exactly the tags the de-identification must handle."""
    nodes = []
    for index in range(4):
        nodes.append(
            f'  <node id="{100 + index}" lat="35.6{index}" lon="139.7{index}">\n'
            f'    <tag k="mgrs_code" v="{_SOURCE_MGRS}"/>\n'
            f'    <tag k="local_x" v="{81596.0 + index}"/>\n'
            f'    <tag k="local_y" v="{50194.0 + index}"/>\n'
            f'    <tag k="ele" v="{_SOURCE_ELE + index}"/>\n'
            "  </node>\n"
        )
    path.write_text(
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        '<osm version="0.6">\n' + "".join(nodes) + '  <way id="200">\n'
        '    <nd ref="100"/>\n'
        '    <nd ref="101"/>\n'
        '    <tag k="type" v="line_thin"/>\n'
        '    <tag k="subtype" v="solid"/>\n'
        "  </way>\n"
        '  <way id="201">\n'
        '    <nd ref="102"/>\n'
        '    <nd ref="103"/>\n'
        '    <tag k="type" v="line_thin"/>\n'
        "  </way>\n"
        '  <relation id="300">\n'
        '    <member type="way" role="left" ref="200"/>\n'
        '    <member type="way" role="right" ref="201"/>\n'
        '    <tag k="type" v="lanelet"/>\n'
        '    <tag k="subtype" v="road"/>\n'
        '    <tag k="location" v="urban"/>\n'
        '    <tag k="one_way" v="yes"/>\n'
        '    <tag k="traffic_light_id" v="999"/>\n'
        "  </relation>\n"
        "</osm>\n",
        encoding="utf-8",
    )


@pytest.fixture
def rebuilt(tmp_path: Path) -> Dict[str, str]:
    """Rebuild a small source and return the output's text plus its path."""
    source = tmp_path / "source.osm"
    _write_source(source)
    destination = tmp_path / "synthetic.osm"
    rebuild_tool.rebuild(
        source=source,
        destination=destination,
        rotate_degrees=37.0,
        to_lat=35.0,
        to_lon=139.0,
        warp=0.0,
        jitter=0.0,
        wavelength=50.0,
        seed=1,
    )
    return {"text": destination.read_text(encoding="utf-8"), "path": str(destination)}


def test_the_source_mgrs_reference_does_not_survive(rebuilt: Dict[str, str]) -> None:
    """``mgrs_code`` locates the source node to 100 m and must not be kept.

    Nothing in the converter reads it -- it appears in ``conversion_config``
    only as a *config* key naming the map's grid, never as a node tag -- so
    there is nothing to preserve and no reason to carry it.
    """
    assert _SOURCE_MGRS not in rebuilt["text"], (
        "the source's MGRS reference is still in the rebuilt map, so the "
        "output locates its origin to 100 m"
    )
    assert 'k="mgrs_code"' not in rebuilt["text"]


def test_absolute_elevation_does_not_survive(rebuilt: Dict[str, str]) -> None:
    """``ele`` is absolute height above the geoid, which is a position.

    It cannot simply be dropped: lanelet2 reads it into ``point.z`` and the
    elevation profile is built from those, so dropping it would flatten the
    fixture and change what the converter emits.  It is offset instead, so
    relative elevation -- the part the converter uses -- survives exactly
    while the absolute value does not.
    """
    values = [float(v) for v in re.findall(r'k="ele" v="([^"]+)"', rebuilt["text"])]
    assert values, "the rebuild dropped `ele` entirely, which flattens the map"

    source = [_SOURCE_ELE + index for index in range(4)]
    for value in values:
        assert not any(
            abs(value - original) < 1e-6 for original in source
        ), f"ele={value} is a source elevation, so absolute height survived"

    # the shape has to be preserved exactly: same spacing, same order
    assert len(values) == len(source)
    offsets = {round(value - original, 9) for value, original in zip(values, source)}
    assert len(offsets) == 1, (
        f"ele was not offset by one constant: {sorted(offsets)} -- relative "
        "elevation changed, so the emitted profile would differ"
    )


def test_semantic_tags_survive_unchanged(rebuilt: Dict[str, str]) -> None:
    """The fixture has to behave like its source, so meaning passes through."""
    for key, value in (
        ("type", "lanelet"),
        ("subtype", "road"),
        ("location", "urban"),
        ("one_way", "yes"),
    ):
        assert f'k="{key}" v="{value}"' in rebuilt["text"], f"{key} was dropped"


def test_source_ids_do_not_survive(rebuilt: Dict[str, str]) -> None:
    """A tag whose value is a source id is a piece of the source map.

    Asserted on tag *values* specifically: searching the whole document for
    the digits would also match them inside a rewritten ``lat``/``lon``, which
    is how the first version of this test failed for the wrong reason.
    """
    assert 'k="traffic_light_id"' not in rebuilt["text"]
    values = set(re.findall(r'<tag k="[^"]+" v="([^"]+)"', rebuilt["text"]))
    assert "999" not in values


# --------------------------------------------------------------------------
# write_excerpt: what a cut does to the lanelets it keeps.
# --------------------------------------------------------------------------


def _write_regelem_source(path: Path) -> None:
    """Two lanelets sharing one right_of_way element, plus a standalone one.

    Lanelet 300 has priority and owns the element; lanelet 301 yields.  The
    question the tests below ask is what happens to 300 when 301 is cut away.
    """
    nodes = "".join(
        f'  <node id="{100 + i}" lat="35.6{i}" lon="139.7{i}">\n'
        f'    <tag k="ele" v="{30.0 + i}"/>\n'
        "  </node>\n"
        for i in range(8)
    )
    ways = "".join(
        f'  <way id="{200 + i}">\n'
        f'    <nd ref="{100 + 2 * i}"/>\n'
        f'    <nd ref="{101 + 2 * i}"/>\n'
        '    <tag k="type" v="line_thin"/>\n'
        "  </way>\n"
        for i in range(4)
    )
    path.write_text(
        '<?xml version="1.0" encoding="UTF-8"?>\n<osm version="0.6">\n'
        + nodes
        + ways
        + '  <relation id="300">\n'
        '    <member type="way" role="left" ref="200"/>\n'
        '    <member type="way" role="right" ref="201"/>\n'
        '    <member type="relation" role="regulatory_element" ref="400"/>\n'
        '    <tag k="type" v="lanelet"/>\n'
        '    <tag k="subtype" v="road"/>\n'
        "  </relation>\n"
        '  <relation id="301">\n'
        '    <member type="way" role="left" ref="202"/>\n'
        '    <member type="way" role="right" ref="203"/>\n'
        '    <tag k="type" v="lanelet"/>\n'
        '    <tag k="subtype" v="road"/>\n'
        "  </relation>\n"
        '  <relation id="400">\n'
        '    <member type="relation" role="right_of_way" ref="300"/>\n'
        '    <member type="relation" role="yield" ref="301"/>\n'
        '    <tag k="type" v="regulatory_element"/>\n'
        '    <tag k="subtype" v="right_of_way"/>\n'
        "  </relation>\n"
        "</osm>\n",
        encoding="utf-8",
    )


def _regelem_refs(text: str, lanelet_id: str) -> list:
    body = re.search('<relation id="' + lanelet_id + '".*?</relation>', text, re.S)
    assert body is not None, f"lanelet {lanelet_id} is not in the excerpt"
    return re.findall(
        r'<member type="relation" role="regulatory_element" ref="(\d+)"', body.group(0)
    )


def test_an_excerpt_keeping_both_sides_keeps_the_element(tmp_path: Path) -> None:
    """The control: with nothing cut away, nothing is stripped."""
    source = tmp_path / "source.osm"
    _write_regelem_source(source)
    destination = tmp_path / "both.osm"

    census = crop_tool.write_excerpt(source, destination, {300, 301})

    assert census["regulatory_elements_kept"] == 1
    assert census["regulatory_elements_dropped"] == 0
    text = destination.read_text(encoding="utf-8")
    assert _regelem_refs(text, "300") == ["400"]


def test_cutting_one_side_strips_the_element_from_the_lanelet_that_is_kept(
    tmp_path: Path,
) -> None:
    """An excerpt is not simply a subset: a kept lanelet can lose attributes.

    Cutting lanelet 301 away leaves the ``right_of_way`` element with no
    yielding lanelet, which lanelet2's own constructor refuses -- so the
    element has to go, and with it the reference from lanelet 300, which was
    kept.  This is necessary behaviour and it is pinned here because it is
    easy to misread the tool as geometry-only: it is exact about geometry and
    lossy about regulatory attributes.

    Measured on the reference map for the same shape: keeping only lanelet
    306 of ``right_of_way`` element 123826 (whose yield side is lanelets
    309/310/311) reports ``regulatory_elements_dropped: 1`` and strips
    element 123826 from the retained lanelet 306.
    """
    source = tmp_path / "source.osm"
    _write_regelem_source(source)
    destination = tmp_path / "one_side.osm"

    census = crop_tool.write_excerpt(source, destination, {300})

    assert census["regulatory_elements_dropped"] == 1, (
        "the census is how a caller learns the cut changed the semantics; "
        "it must count this"
    )
    assert census["regulatory_elements_kept"] == 0

    text = destination.read_text(encoding="utf-8")
    assert _regelem_refs(text, "300") == [], (
        "lanelet 300 was kept but still references an element the excerpt "
        "does not contain, so the excerpt would not load"
    )
    assert '<relation id="400"' not in text


def test_the_excerpt_has_no_dangling_member_references(tmp_path: Path) -> None:
    """Whatever is cut, what remains must resolve inside the file.

    This is the property the tool exists for -- an excerpt that only loads as
    part of its source is useless as a fixture -- and it is the one a
    regulatory-element drop could most easily break.
    """
    source = tmp_path / "source.osm"
    _write_regelem_source(source)
    destination = tmp_path / "excerpt.osm"

    crop_tool.write_excerpt(source, destination, {300})
    text = destination.read_text(encoding="utf-8")

    present = {
        kind: set(re.findall(r"<" + kind + r' id="(\d+)"', text))
        for kind in ("node", "way", "relation")
    }
    for kind, ref in re.findall(r'<member type="(\w+)" role="[^"]*" ref="(\d+)"', text):
        assert ref in present[kind], f"member points at missing {kind} {ref}"
    for ref in re.findall(r'<nd ref="(\d+)"', text):
        assert ref in present["node"], f"<nd> points at missing node {ref}"
