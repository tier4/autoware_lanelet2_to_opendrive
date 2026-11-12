"""Tests for ``scripts/map_issue.py``.

The tool's job is to let a defect be reported without the map, so the tests
that matter are the negative controls: each way a piece of the source map
could reach ``public/`` is injected on purpose, and the leak check has to
name it. A leak check that has only ever been seen to pass has not been
tested.
"""

import importlib.util
import json
import shutil
import sys
from pathlib import Path

import pytest

# ``scripts/`` is deliberately not a package (see test_tdd.py); load by path.
_spec = importlib.util.spec_from_file_location(
    "map_issue", Path(__file__).with_name("map_issue.py")
)
assert _spec is not None and _spec.loader is not None
map_issue = importlib.util.module_from_spec(_spec)
sys.modules["map_issue"] = map_issue
_spec.loader.exec_module(map_issue)

sys.path.insert(0, str(map_issue.TOOLS))
from rebuild_as_synthetic import rebuild  # noqa: E402


# --- a small "source map" with everything that identifies one ---------------

_NODES = [
    # id, lat, lon, ele, local_x, local_y
    (10001, 35.6500001, 139.7400001, 31.25, 81000.10, 92000.10),
    (10002, 35.6500002, 139.7402001, 31.40, 81018.20, 92000.20),
    (10003, 35.6500301, 139.7400002, 31.30, 81000.30, 92003.40),
    (10004, 35.6500302, 139.7402002, 31.45, 81018.40, 92003.50),
]


def _write_source(path: Path) -> Path:
    lines = ['<?xml version="1.0" encoding="UTF-8"?>', '<osm generator="test">']
    for node_id, lat, lon, ele, x, y in _NODES:
        lines.append(
            f'  <node id="{node_id}" lat="{lat:.7f}" lon="{lon:.7f}">'
            f'<tag k="ele" v="{ele}"/><tag k="local_x" v="{x}"/>'
            f'<tag k="local_y" v="{y}"/><tag k="mgrs_code" v="54SUE810920"/></node>'
        )
    lines += [
        '  <way id="20001"><nd ref="10001"/><nd ref="10002"/>'
        '<tag k="type" v="line_thin"/></way>',
        '  <way id="20002"><nd ref="10003"/><nd ref="10004"/>'
        '<tag k="type" v="line_thin"/></way>',
        '  <relation id="50001"><member type="way" ref="20002" role="left"/>'
        '<member type="way" ref="20001" role="right"/>'
        '<tag k="type" v="lanelet"/><tag k="subtype" v="road"/></relation>',
        "</osm>",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


@pytest.fixture
def bundle(tmp_path: Path) -> dict:
    """A source map, its (identical) reduced excerpt and a clean public/."""
    source = _write_source(tmp_path / "customer_map.osm")
    private, public = tmp_path / "out" / "private", tmp_path / "out" / "public"
    private.mkdir(parents=True)
    public.mkdir(parents=True)
    reduced = private / "reduced.osm"
    shutil.copy(source, reduced)
    rebuild(reduced, public / "repro.osm", 37.0, 35.0, 139.0, 0.25, 0.02, 150.0, 7)
    (public / "fingerprint.json").write_text(
        json.dumps({"findings": [{"finding": "invariant:x", "count": 3}]}),
        encoding="utf-8",
    )
    return {"source": source, "reduced": reduced, "public": public}


def _failed(checks: list) -> list:
    return [c.name for c in checks if not c.ok]


def _check(bundle: dict) -> list:
    return map_issue.leak_check(bundle["public"], bundle["source"], bundle["reduced"])


# --- leak check: the clean case, then one injected leak at a time -----------


def test_a_rebuilt_reproducer_passes(bundle: dict) -> None:
    checks = _check(bundle)
    assert _failed(checks) == []
    # Every family of check actually ran: a pass over zero checks is not one.
    assert len(checks) >= 8


def test_the_reduced_excerpt_itself_is_caught(bundle: dict) -> None:
    shutil.copy(bundle["reduced"], bundle["public"] / "repro.osm")
    failed = _failed(_check(bundle))
    assert "ids are a fresh counter" in failed
    assert "no tag carries a source id or position" in failed
    assert "elevation is re-origined" in failed
    assert "local_x/local_y are re-origined" in failed
    assert "no node keeps its latitude or longitude" in failed
    assert "placed outside the source map's bounding box" in failed


def test_renumbering_alone_is_not_enough(bundle: dict) -> None:
    """Fresh ids over the surveyed coordinates is still the surveyed map."""
    text = bundle["reduced"].read_text(encoding="utf-8")
    for old, new in (
        ("10001", "1"),
        ("10002", "2"),
        ("10003", "3"),
        ("10004", "4"),
        ("20001", "5"),
        ("20002", "6"),
        ("50001", "7"),
    ):
        text = text.replace(f'"{old}"', f'"{new}"')
    (bundle["public"] / "repro.osm").write_text(text, encoding="utf-8")
    failed = _failed(_check(bundle))
    assert "ids are a fresh counter" not in failed
    assert "no node keeps its latitude or longitude" in failed


def test_a_reproducer_placed_inside_the_source_map_is_caught(bundle: dict) -> None:
    rebuild(
        bundle["reduced"],
        bundle["public"] / "repro.osm",
        37.0,
        35.65002,
        139.7401,
        0.25,
        0.02,
        150.0,
        7,
    )
    assert "placed outside the source map's bounding box" in _failed(_check(bundle))


@pytest.mark.parametrize(
    "leak",
    [
        "see lanelet 50001",  # a source element id
        "near 35.6500301 N",  # a coordinate inside the source bounding box
        "from customer_map.osm",  # the source file name
    ],
)
def test_a_source_id_coordinate_or_name_in_the_issue_is_caught(
    bundle: dict, leak: str
) -> None:
    (bundle["public"] / "issue.md").write_text(f"It fails {leak}.\n", encoding="utf-8")
    assert "issue.md: no source path, hash, id or coordinate" in _failed(_check(bundle))


def test_a_count_is_not_mistaken_for_an_id(bundle: dict) -> None:
    (bundle["public"] / "issue.md").write_text(
        "3 findings over 979 lanelets, 26 checkers.\n", encoding="utf-8"
    )
    assert _failed(_check(bundle)) == []


def test_an_unexpected_file_in_public_is_caught(bundle: dict) -> None:
    (bundle["public"] / "locations.json").write_text("{}", encoding="utf-8")
    assert "only expected files" in _failed(_check(bundle))


# --- scrubbing and findings -------------------------------------------------


def test_scrub_removes_ids_measurements_and_paths() -> None:
    message = "road 412/successor: meets road 87 at its start 0.071 m (/maps/a.osm)"
    assert map_issue.scrub(message) == (
        "road <n>/successor: meets road <n> at its start <n> m (<path>)"
    )


def test_the_public_form_of_a_finding_carries_no_location() -> None:
    finding = map_issue.Finding("invariant", "linked_endpoints_apart")
    finding.add("road 412 meets road 87 0.071 m apart", roads=[412, 87])
    finding.add("road 9 meets road 10 0.013 m apart", roads=[9, 10])
    public = finding.public()
    assert public["count"] == 2
    # Two instances of one kind scrub to one example.
    assert public["examples"] == ["road <n> meets road <n> <n> m apart"]
    assert "412" not in json.dumps(public)
    assert finding.roads == [[87, 412], [9, 10]]


def test_still_fails_asks_for_the_same_finding_not_any_finding() -> None:
    findings = map_issue.Findings()
    findings.get("invariant", "lane_link_target_missing").add("road 1 lane 2")
    assert map_issue.still_fails(findings, "invariant:lane_link_target_missing")
    assert not map_issue.still_fails(findings, "invariant:linked_endpoints_apart")
    assert not map_issue.still_fails(findings, "qc:lane_link_target_missing")
    assert map_issue.still_fails(
        findings, "invariant:lane_link_target_missing", match=r"lane <n>"
    )
    assert not map_issue.still_fails(
        findings, "invariant:lane_link_target_missing", match=r"junction"
    )


def test_crash_of_reads_the_last_exception_in_the_log() -> None:
    log = "\n".join(
        [
            "Traceback (most recent call last):",
            '  File "a.py", line 1, in <module>',
            "KeyError: 'first'",
            "During handling of the above exception, another exception occurred:",
            "Traceback (most recent call last):",
            "RuntimeError: Failed to move point 10030",
            "",
        ]
    )
    assert map_issue.crash_of(log) == ("RuntimeError", "Failed to move point 10030")
    assert map_issue.crash_of("nothing went wrong\n") is None


def test_the_same_crash_at_another_place_is_the_same_finding(tmp_path: Path) -> None:
    def crash(message: str) -> list:
        conversion = map_issue.Conversion(tmp_path / "m.osm", tmp_path, 1, 0.0, message)
        return sorted(map_issue.measure_findings(conversion, ["crash"]).by_key)

    here = crash("RuntimeError: Failed to move point 10030")
    there = crash("RuntimeError: Failed to move point 77")
    other = crash("RuntimeError: Failed to merge lanelets 1 and 2")
    assert here == there
    assert here != other


def test_every_mapping_field_attaches_lanelets_to_roads(tmp_path: Path) -> None:
    mapping = tmp_path / "out.mapping.json"
    mapping.write_text(
        json.dumps(
            {
                "lanelet_to_road_and_lane": {"100": [1, -1], "101": [1, -2]},
                "lanelet_additional_road_lanes": {"100": [[5, 1]]},
                "chain_merged_roads": {"7": 1},
                "exit_fork_duplicate_roads": {"8": [5]},
            }
        ),
        encoding="utf-8",
    )
    roads = map_issue.lanelets_of_roads(mapping)
    assert roads[1] == {100, 101}
    assert roads[5] == {100}
    assert roads[7] == {100, 101}
    assert roads[8] == {100}


def test_published_overrides_withhold_path_values() -> None:
    assert map_issue._public_overrides(
        ["map=junction_cluster", "map.projector_info=/maps/customer/info.yaml"]
    ) == ["map=junction_cluster", "map.projector_info=<path>"]


def test_the_issue_draft_is_built_from_public_documents_only() -> None:
    fingerprint = {
        "converter": {"version": "2.62.0", "commit": "abc1234"},
        "overrides": ["map=junction_cluster"],
        "conversion": {"completed": True, "seconds": 12.0, "preprocessing_ran": False},
        "map_census": {"lanelets": 9},
        "notes": {"qc_checkers": {"total": 26, "completed": 25}},
        "instruments_incomplete": {"route": "KeyError: <n>"},
        "findings": [
            {"finding": "invariant:x", "count": 3, "examples": ["road <n> | lane <n>"]}
        ],
    }
    body = map_issue.draft_issue(fingerprint, None)
    assert "| `invariant:x` | 3 | road <n> \\| lane <n> |" in body
    assert "lower bound" in body
    assert "Instrument `route` did not complete" in body
    assert "L0 only" in body


# --- a report that starts from a place, and a probe -------------------------


def test_findings_near_counts_instances_touching_the_area() -> None:
    located = {
        "invariant:a": [[1, 2], [7, 8], [9]],
        "qc:b": [[100]],
        "route:c": [],
    }
    assert map_issue.findings_near(located, [2, 9, 55]) == {
        "invariant:a": (2, 3),
        # Kept, at zero: "nothing here" is the answer that calls for a probe.
        "qc:b": (0, 1),
        "route:c": (0, 0),
    }


def _completed_conversion(directory: Path) -> "map_issue.Conversion":
    (directory / "out.xodr").write_text("<OpenDRIVE></OpenDRIVE>\n", encoding="utf-8")
    (directory / "out.mapping.json").write_text("{}", encoding="utf-8")
    return map_issue.Conversion(directory / "m.osm", directory, 0, 0.0, "")


def test_a_probe_becomes_an_instrument(tmp_path: Path) -> None:
    probe = tmp_path / "probe.py"
    probe.write_text(
        "import json, sys\n"
        "assert len(sys.argv) == 4\n"
        "print('a human-readable line the tool ignores')\n"
        "print(json.dumps({'code': 'odd', 'message': 'road 12 is odd',"
        " 'roads': [12], 'lanelets': [3401]}))\n"
        "print(json.dumps({'code': 'odd', 'message': 'road 13 is odd',"
        " 'roads': [13]}))\n",
        encoding="utf-8",
    )
    findings = map_issue.measure_findings(
        _completed_conversion(tmp_path), ["probe"], probe=probe
    )
    assert findings.incomplete == {}
    found = findings.by_key["probe:odd"]
    assert found.count == 2
    assert found.public()["examples"] == ["road <n> is odd"]
    assert found.roads == [[12], [13]]
    assert found.lanelets == [[3401], []]


def test_a_probe_that_fails_is_incomplete_not_clean(tmp_path: Path) -> None:
    probe = tmp_path / "probe.py"
    probe.write_text("raise SystemExit('boom')\n", encoding="utf-8")
    findings = map_issue.measure_findings(
        _completed_conversion(tmp_path), ["probe"], probe=probe
    )
    assert findings.by_key == {}
    assert "probe" in findings.incomplete
    assert not map_issue.still_fails(findings, "probe:odd")


def test_a_probe_finding_without_a_probe_is_incomplete(tmp_path: Path) -> None:
    findings = map_issue.measure_findings(_completed_conversion(tmp_path), ["probe"])
    assert "probe" in findings.incomplete


# --- "around here": every way a reporter may name a place -------------------

_XODR = """<OpenDRIVE>
  <road id="1" length="10" junction="-1">
    <link><successor elementType="road" elementId="2"/></link>
    <planView><geometry s="0" x="0" y="0" hdg="0" length="10"><line/></geometry></planView>
  </road>
  <road id="2" length="10" junction="-1">
    <link><predecessor elementType="road" elementId="1"/>
          <successor elementType="road" elementId="3"/></link>
    <planView>
      <geometry s="0" x="10" y="0" hdg="0" length="10">
        <paramPoly3 aU="0" bU="10" cU="0" dU="0" aV="0" bV="0" cV="5" dV="0"
                    pRange="normalized"/>
      </geometry>
    </planView>
  </road>
  <road id="3" length="15.707963" junction="-1">
    <link><predecessor elementType="road" elementId="2"/></link>
    <planView>
      <geometry s="0" x="100" y="0" hdg="0" length="15.707963">
        <arc curvature="0.1"/>
      </geometry>
    </planView>
  </road>
</OpenDRIVE>
"""


@pytest.fixture
def converted(tmp_path: Path) -> dict:
    xodr = tmp_path / "out.xodr"
    xodr.write_text(_XODR, encoding="utf-8")
    mapping = tmp_path / "out.mapping.json"
    # Road 2 has no lanelet of its own: a synthetic connector.
    mapping.write_text(
        json.dumps({"lanelet_to_road_and_lane": {"501": [1, 1], "503": [3, 1]}}),
        encoding="utf-8",
    )
    return {
        "xodr": xodr,
        "mapping": mapping,
        "source": _write_source(tmp_path / "m.osm"),
    }


def test_reference_lines_are_sampled_along_each_primitive(converted: dict) -> None:
    samples = map_issue.reference_line_samples(converted["xodr"])
    assert samples[1][0] == (0.0, 0.0)
    assert samples[1][-1] == pytest.approx((10.0, 0.0))
    # paramPoly3, normalized: u = 10p, v = 5p^2.
    assert samples[2][-1] == pytest.approx((20.0, 5.0))
    # A quarter circle of radius 10 turning left ends at (+10, +10).
    assert samples[3][-1] == pytest.approx((110.0, 10.0), abs=1e-4)


def _place(converted: dict, **how: object) -> list:
    return map_issue.resolve_place(
        converted["source"], converted["xodr"], converted["mapping"], **how
    )


def test_a_place_can_be_a_road_or_a_position_in_the_output(converted: dict) -> None:
    assert _place(converted, lanelets=[7, 3, 7]) == [3, 7]
    assert _place(converted, roads=[1]) == [501]
    # A connector is located by the roads on either side of it.
    assert _place(converted, roads=[2]) == [501, 503]
    assert _place(converted, xy=(5.0, 0.5), radius=1.0) == [501]
    assert _place(converted, xy=(110.0, 10.0), radius=1.0) == [503]
    assert _place(converted, xy=(5.0, 500.0), radius=1.0) == []
    with pytest.raises(ValueError):
        _place(converted)


def test_a_place_can_be_a_latitude_and_longitude(converted: dict) -> None:
    assert _place(converted, latlon=(35.6500001, 139.7400001), radius=5.0) == [50001]
    assert _place(converted, latlon=(35.7, 139.8), radius=5.0) == []


def test_the_symptom_goes_into_the_issue_and_is_leak_checked(bundle: dict) -> None:
    fingerprint = {
        "converter": {"version": "1", "commit": "abc"},
        "overrides": [],
        "conversion": {"completed": True, "seconds": 1.0, "preprocessing_ran": False},
        "map_census": {"lanelets": 1},
        "findings": [],
    }
    clean = map_issue.draft_issue(
        fingerprint, None, "The lane ends in the middle of a merge.", "CARLA <version>"
    )
    assert "The lane ends in the middle of a merge." in clean
    assert "Seen in: CARLA <version>" in clean
    (bundle["public"] / "issue.md").write_text(clean, encoding="utf-8")
    assert _failed(_check(bundle)) == []
    # A reporter's own words are where a place slips in.
    leaky = map_issue.draft_issue(fingerprint, None, "It breaks at lanelet 50001.")
    (bundle["public"] / "issue.md").write_text(leaky, encoding="utf-8")
    assert "issue.md: no source path, hash, id or coordinate" in _failed(_check(bundle))


# --- the whole procedure, on a committed map --------------------------------

# Reports every road that carries two or more lanes on one side. It stands in
# for a real instrument: it is not a defect, but it is located, it survives
# only while two adjacent lanelets do, and it does not depend on which
# built-in instruments this checkout has or on what they currently find.
_PROBE = """
import json, sys
import xml.etree.ElementTree as ET

for road in ET.parse(sys.argv[1]).getroot().iter("road"):
    for side in ("left", "right"):
        lanes = [l for s in road.iter(side) for l in s.findall("lane")]
        if len(lanes) >= 2:
            print(json.dumps({
                "code": "several_lanes_on_one_side",
                "message": f"road {road.get('id')} has {len(lanes)} {side} lanes",
                "roads": [int(road.get("id"))],
            }))
"""


@pytest.mark.slow
def test_the_procedure_end_to_end_on_a_committed_map(tmp_path: Path) -> None:
    """fingerprint -> near -> reduce -> sanitize -> draft, and nothing leaks.

    Runs the real converter, so it needs the project environment; on a host
    that cannot build it (see CLAUDE.md) the pure tests above still run.
    """
    pytest.importorskip("lanelet2")
    pytest.importorskip("autoware_lanelet2_to_opendrive")
    source = (
        map_issue.REPO
        / "autoware_lanelet2_to_opendrive"
        / "test"
        / "data"
        / "junction_fanout_mini.osm"
    )
    out = tmp_path / "report"
    probe = tmp_path / "probe.py"
    probe.write_text(_PROBE, encoding="utf-8")
    finding = "probe:several_lanes_on_one_side"
    with_probe = ["--probe", str(probe)]
    neutral = ["--", "map=junction_cluster"]

    assert (
        map_issue.main(
            ["fingerprint", str(source), "--out", str(out), *with_probe, *neutral]
        )
        == 0
    )
    fingerprint = json.loads((out / "public" / "fingerprint.json").read_text())
    located = json.loads((out / "private" / "locations.json").read_text())
    assert finding in [row["finding"] for row in fingerprint["findings"]]
    instances = located["lanelets_by_finding"][finding]
    assert instances, "the probe's roads were not traced back to lanelets"

    # The report is "on this map, around here, I see this". "Here" is a
    # position in the converted file: the middle of a road the probe flagged.
    conversion = out / "private" / "conversion"
    by_road = map_issue.lanelets_of_roads(conversion / "out.mapping.json")
    flagged = next(r for r, ids in sorted(by_road.items()) if sorted(ids) in instances)
    points = map_issue.reference_line_samples(conversion / "out.xodr")[flagged]
    x, y = points[len(points) // 2]
    assert (
        map_issue.main(
            ["near", str(source), "--from", str(out), f"--xy={x},{y}", "--radius", "1"]
        )
        == 0
    )
    place = json.loads((out / "private" / "place.json").read_text())["lanelets"]
    assert set(by_road[flagged]) <= set(place)

    before = fingerprint["map_census"]["lanelets"]
    assert (
        map_issue.main(
            ["reduce", str(source), "--from", str(out), "--finding", finding]
            + ["--place", *with_probe, *neutral]
        )
        == 0
    )
    sanitize = ["sanitize", "--from", str(out), "--finding", finding]
    # Without the owner's agreement no reproducer is written at all.
    assert map_issue.main([*sanitize, *with_probe, *neutral]) == 2
    assert not (out / "public" / "repro.osm").exists()
    assert map_issue.main([*sanitize, "--owner-agreed", *with_probe, *neutral]) == 0
    symptom = "Two lanes are emitted side by side on one road at a fork."
    assert (
        map_issue.main(
            ["draft", "--from", str(out), "--symptom", symptom]
            + ["--seen-in", "a structural probe"]
        )
        == 0
    )
    assert symptom in (out / "public" / "issue.md").read_text(encoding="utf-8")

    repro = json.loads((out / "public" / "repro.json").read_text())
    assert repro["reproduces"] == finding
    assert finding in [row["finding"] for row in repro["findings"]]
    # Two adjacent lanelets are the least that can make a two-lane road.
    assert 2 <= repro["map_census"]["lanelets"] < before
    assert sorted(p.name for p in (out / "public").iterdir()) == sorted(
        map_issue.PUBLIC_FILES
    )
    checks = map_issue.leak_check(
        out / "public", source, out / "private" / "reduced.osm"
    )
    assert [c.name for c in checks if not c.ok] == []
