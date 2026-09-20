"""Tests for the pre-flight inspection of an input Lanelet2 map.

The motivating case is a real customer map that ``lanelet2.io.load`` refused
with a 38-line wall of per-primitive parser errors.  Nineteen of those lines
were one problem -- a ``right_of_way`` regulatory element with no member in
the ``yield`` role -- and the other nineteen were its cascade, each lanelet
that referred to an element the parser had already thrown away.  Nothing in
the message said "nineteen of one thing", and nothing said what to change.

These tests pin the two properties that matter: the checks name the right
elements, and they stay silent on maps that load.  The customer map itself
cannot be committed, so the positive cases are built here from the minimal
XML that reproduces each constraint.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from autoware_lanelet2_to_opendrive.map_preflight import (
    MapPreflightError,
    PreflightReport,
    inspect_lanelet2_osm,
    main,
)

DATA = Path(__file__).parent / "data"

#: The shape of the 19 elements that made the customer map unloadable: every
#: member carries the ``right_of_way`` role, so the element names who has
#: priority and never names who gives way.
_BAD_RIGHT_OF_WAY = """\
  <relation id="900">
    <member type="relation" role="right_of_way" ref="100"/>
    <member type="relation" role="right_of_way" ref="101"/>
    <tag k="type" v="regulatory_element"/>
    <tag k="subtype" v="right_of_way"/>
  </relation>
"""

#: The same element repaired the way the customer map was repaired: the
#: first member becomes the yielding lanelet.
_GOOD_RIGHT_OF_WAY = """\
  <relation id="900">
    <member type="relation" role="yield" ref="100"/>
    <member type="relation" role="right_of_way" ref="101"/>
    <tag k="type" v="regulatory_element"/>
    <tag k="subtype" v="right_of_way"/>
  </relation>
"""

_LANELETS = """\
  <relation id="100">
    <member type="way" role="left" ref="10"/>
    <member type="way" role="right" ref="11"/>
    <tag k="type" v="lanelet"/>
    <tag k="subtype" v="road"/>
  </relation>
  <relation id="101">
    <member type="way" role="left" ref="11"/>
    <member type="way" role="right" ref="12"/>
    <tag k="type" v="lanelet"/>
    <tag k="subtype" v="road"/>
  </relation>
"""

_NODES_AND_WAYS = """\
  <node id="1" lat="35.0" lon="139.0"/>
  <node id="2" lat="35.0001" lon="139.0"/>
  <node id="3" lat="35.0" lon="139.0001"/>
  <node id="4" lat="35.0001" lon="139.0001"/>
  <node id="5" lat="35.0" lon="139.0002"/>
  <node id="6" lat="35.0001" lon="139.0002"/>
  <way id="10"><nd ref="1"/><nd ref="2"/></way>
  <way id="11"><nd ref="3"/><nd ref="4"/></way>
  <way id="12"><nd ref="5"/><nd ref="6"/></way>
"""


def _write_map(tmp_path: Path, body: str, name: str = "map.osm") -> Path:
    """Write a minimal Lanelet2 OSM file containing *body* and return its path."""
    path = tmp_path / name
    path.write_text(
        f'<?xml version="1.0" encoding="UTF-8"?>\n<osm generator="test">\n'
        f"{body}</osm>\n"
    )
    return path


def _codes(report: PreflightReport) -> list[str]:
    """Return the problem codes in the report, in report order."""
    return [finding.code for finding in report.findings]


def test_right_of_way_without_yield_is_reported(tmp_path: Path) -> None:
    """The element that broke the customer map is named, counted and located."""
    path = _write_map(tmp_path, _NODES_AND_WAYS + _LANELETS + _BAD_RIGHT_OF_WAY)
    report = inspect_lanelet2_osm(path)

    assert _codes(report) == ["right_of_way_without_yield"]
    (finding,) = report.findings
    assert finding.count == 1
    assert finding.element_ids == ("900",)
    # The remedy has to name the role, otherwise the report says what is
    # broken without saying what to type.
    assert "yield" in finding.remedy


def test_repaired_right_of_way_is_silent(tmp_path: Path) -> None:
    """Giving one member the yield role is exactly what makes the map load."""
    report = inspect_lanelet2_osm(
        _write_map(tmp_path, _NODES_AND_WAYS + _LANELETS + _GOOD_RIGHT_OF_WAY)
    )
    assert report.ok
    assert report.counts == {"nodes": 6, "ways": 3, "relations": 3}


def test_every_bad_element_is_listed(tmp_path: Path) -> None:
    """A class with many members reports all of them, not just the first."""
    bad = "".join(
        _BAD_RIGHT_OF_WAY.replace('id="900"', f'id="{900 + i}"') for i in range(19)
    )
    report = inspect_lanelet2_osm(
        _write_map(tmp_path, _NODES_AND_WAYS + _LANELETS + bad)
    )

    (finding,) = report.findings
    assert finding.count == 19
    assert finding.element_ids == tuple(str(900 + i) for i in range(19))


def test_report_caps_the_id_list_but_not_the_count(tmp_path: Path) -> None:
    """A long class stays readable: ids are capped, the total never is."""
    bad = "".join(
        _BAD_RIGHT_OF_WAY.replace('id="900"', f'id="{900 + i}"') for i in range(19)
    )
    report = inspect_lanelet2_osm(
        _write_map(tmp_path, _NODES_AND_WAYS + _LANELETS + bad)
    )
    text = report.format(max_ids=5)

    assert "19 element(s)" in text
    assert "... and 14 more" in text
    assert "904" in text
    assert "905" not in text


def test_dangling_relation_member_is_reported(tmp_path: Path) -> None:
    """A member pointing at an id that is not in the file is named."""
    body = (
        _NODES_AND_WAYS
        + _LANELETS
        + _GOOD_RIGHT_OF_WAY.replace('ref="101"', 'ref="404"')
    )
    report = inspect_lanelet2_osm(_write_map(tmp_path, body))

    assert _codes(report) == ["dangling_member_ref"]
    assert report.findings[0].element_ids == ("900",)


def test_forward_reference_is_not_dangling(tmp_path: Path) -> None:
    """A relation may name an id defined later in the file."""
    body = _NODES_AND_WAYS + _GOOD_RIGHT_OF_WAY + _LANELETS
    assert inspect_lanelet2_osm(_write_map(tmp_path, body)).ok


def test_lanelet_missing_a_bound_is_reported(tmp_path: Path) -> None:
    """A lanelet needs both boundaries; one of them is not enough."""
    body = (
        _NODES_AND_WAYS
        + """\
  <relation id="100">
    <member type="way" role="left" ref="10"/>
    <tag k="type" v="lanelet"/>
    <tag k="subtype" v="road"/>
  </relation>
"""
    )
    report = inspect_lanelet2_osm(_write_map(tmp_path, body))
    assert _codes(report) == ["lanelet_missing_bound"]
    assert report.findings[0].element_ids == ("100",)


def test_way_referencing_a_missing_node_is_reported(tmp_path: Path) -> None:
    """A boundary that lists a node the file does not define is named."""
    body = _NODES_AND_WAYS.replace('<nd ref="5"/>', '<nd ref="777"/>')
    report = inspect_lanelet2_osm(_write_map(tmp_path, body))
    assert _codes(report) == ["way_references_missing_node"]
    assert report.findings[0].element_ids == ("12",)


def test_duplicate_id_is_reported(tmp_path: Path) -> None:
    """The same id twice in one layer means the later one silently wins."""
    body = _NODES_AND_WAYS + '  <node id="1" lat="36.0" lon="140.0"/>\n'
    report = inspect_lanelet2_osm(_write_map(tmp_path, body))
    assert _codes(report) == ["duplicate_primitive_id"]
    assert report.findings[0].element_ids == ("1",)


def test_several_classes_are_reported_separately(tmp_path: Path) -> None:
    """Two kinds of problem stay two entries, each with its own count."""
    body = (
        _NODES_AND_WAYS
        + _LANELETS
        + _BAD_RIGHT_OF_WAY
        + """\
  <relation id="901">
    <member type="way" role="left" ref="10"/>
    <tag k="type" v="lanelet"/>
    <tag k="subtype" v="road"/>
  </relation>
"""
    )
    report = inspect_lanelet2_osm(_write_map(tmp_path, body))
    assert _codes(report) == ["right_of_way_without_yield", "lanelet_missing_bound"]
    assert report.total_elements == 2


def test_not_well_formed_xml_does_not_block_conversion() -> None:
    """Lanelet2's reader is laxer than strict XML, so lxml's refusal is not a finding.

    ``walkway_mini.osm`` carries a ``--`` inside an XML comment.  lxml rejects
    the file; lanelet2 reads it and the suite converts it.  The pre-flight has
    to report that it could not look, not that the map is broken.
    """
    report = inspect_lanelet2_osm(DATA / "walkway_mini.osm")
    assert report.ok
    assert not report.inspected
    assert report.xml_error is not None
    assert "not inspected" in report.format()


@pytest.mark.parametrize(
    "fixture",
    [
        "nishishinjuku.osm",
        "junction_fanout_mini.osm",
        "junction_fanin_mini.osm",
        "parking_lot_mini.osm",
        "short_lanelet_mini.osm",
        "speed_change_mini.osm",
        # The two TransverseMercator fixtures live in subdirectories and
        # are converted by CI's carla-import-test job, so they are maps
        # the suite loads just as much as the flat ones above.
        "tmerc_mini/lanelet2_map.osm",
        "tmerc_mini_k9999/lanelet2_map.osm",
    ],
)
def test_committed_fixtures_are_clean(fixture: str) -> None:
    """Every map the suite loads has to pass.

    This is the regression that matters most: a false positive here stops
    every conversion in the project.
    """
    report = inspect_lanelet2_osm(DATA / fixture)
    assert report.inspected, report.format()
    assert not report.findings, report.format()


@pytest.mark.parametrize(
    "code, body",
    [
        (
            "right_of_way_without_yield",
            _NODES_AND_WAYS + _LANELETS + _BAD_RIGHT_OF_WAY,
        ),
        (
            "dangling_member_ref",
            _NODES_AND_WAYS
            + _LANELETS
            + _GOOD_RIGHT_OF_WAY.replace('ref="101"', 'ref="404"'),
        ),
        (
            "way_references_missing_node",
            _NODES_AND_WAYS.replace('<nd ref="5"/>', '<nd ref="777"/>') + _LANELETS,
        ),
        (
            "lanelet_missing_bound",
            _NODES_AND_WAYS
            + """\
  <relation id="100">
    <member type="way" role="left" ref="10"/>
    <tag k="type" v="lanelet"/>
    <tag k="subtype" v="road"/>
  </relation>
""",
        ),
    ],
)
def test_blocking_findings_really_do_stop_lanelet2(
    tmp_path: Path, code: str, body: str
) -> None:
    """Each blocking class is a map ``lanelet2.io.load`` actually refuses.

    Without this the pre-flight could drift into rejecting maps that convert
    perfectly well, which is a worse failure than the wall of parser errors
    it replaces.
    """
    import lanelet2
    from autoware_lanelet2_extension_python.projection import MGRSProjector

    path = _write_map(tmp_path, body, f"{code}.osm")

    report = inspect_lanelet2_osm(path)
    assert [f.code for f in report.findings] == [code]
    assert report.findings[0].blocking
    assert not report.ok

    with pytest.raises(Exception):
        lanelet2.io.load(str(path), MGRSProjector(lanelet2.io.Origin(35.0, 139.0)))


def test_non_blocking_finding_is_reported_but_lets_the_map_through(
    tmp_path: Path,
) -> None:
    """A duplicate id is worth saying out loud, but it is not a load failure."""
    import lanelet2
    from autoware_lanelet2_extension_python.projection import MGRSProjector

    path = _write_map(
        tmp_path,
        _NODES_AND_WAYS + '  <node id="1" lat="35.0" lon="139.0"/>\n' + _LANELETS,
        "duplicate.osm",
    )

    report = inspect_lanelet2_osm(path)
    assert [f.code for f in report.findings] == ["duplicate_primitive_id"]
    assert not report.findings[0].blocking
    assert report.ok
    assert "does not stop the load" in report.format()

    # The map loads: rejecting it would have been a false positive.
    lanelet2.io.load(str(path), MGRSProjector(lanelet2.io.Origin(35.0, 139.0)))


def test_cli_exit_codes(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    """The CLI exits non-zero on a broken map, zero on a clean one -- and says why."""
    broken = _write_map(
        tmp_path, _NODES_AND_WAYS + _LANELETS + _BAD_RIGHT_OF_WAY, "broken.osm"
    )
    clean = _write_map(
        tmp_path, _NODES_AND_WAYS + _LANELETS + _GOOD_RIGHT_OF_WAY, "clean.osm"
    )

    assert main([str(broken)]) == 1
    assert "right_of_way_without_yield" in capsys.readouterr().out

    assert main([str(clean)]) == 0
    assert "no structural problems" in capsys.readouterr().out


def test_missing_file_raises(tmp_path: Path) -> None:
    """A path that does not exist is a caller error, not a map problem."""
    with pytest.raises(FileNotFoundError):
        inspect_lanelet2_osm(tmp_path / "nope.osm")


def test_error_message_carries_the_report(tmp_path: Path) -> None:
    """Printing the exception alone still shows the classes and counts."""
    report = inspect_lanelet2_osm(
        _write_map(tmp_path, _NODES_AND_WAYS + _LANELETS + _BAD_RIGHT_OF_WAY)
    )
    error = MapPreflightError(report)
    assert "right_of_way_without_yield" in str(error)
    assert error.report is report
