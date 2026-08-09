"""Tests for the xodr_digest structure-snapshot tool.

Fixtures are hand-written minimal OpenDRIVE XML (no real map data): the
production nishishinjuku.osm/.xodr artifacts are intentionally not used
here, since this tool only needs to exercise XML shapes, not real geometry.
"""

from __future__ import annotations

import json
import math
import sys

import pytest
from autoware_lanelet2_to_opendrive.xodr_digest import (
    build_digest,
    format_digest,
    main,
)

from autoware_lanelet2_to_opendrive import xodr_digest

_HEADER = """\
<header revMajor="1" revMinor="7" name="test-map" version="1.00"
        north="12345.678999" south="0.0" east="100.0" west="0.0"
        date="2026-06-16T23:29:43">
  <geoReference>+proj=utm +zone=54 +datum=WGS84</geoReference>
</header>
"""


def _xodr(body: str) -> str:
    """Wrap `body` (roads/junctions XML) in a minimal valid OpenDRIVE envelope."""
    return f"""<?xml version="1.0" standalone="yes"?>
<OpenDRIVE>
{_HEADER}
{body}
</OpenDRIVE>
"""


def _write(tmp_path, body: str, name: str = "test.xodr"):
    path = tmp_path / name
    path.write_text(_xodr(body), encoding="utf-8")
    return path


# A single self-contained road: one road-level link, one lane-level link,
# one paramPoly3 geometry segment, three lanes (left/center/right), one
# signal, one object.
_ROAD_1 = """
<road name="r1" length="32.774999" id="1" junction="-1">
  <link>
    <predecessor elementType="road" elementId="10" contactPoint="end"/>
    <successor elementType="junction" elementId="20"/>
  </link>
  <planView>
    <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="32.77">
      <paramPoly3 aU="0" bU="1" cU="0" dU="0" aV="0" bV="0" cV="0" dV="0" pRange="arcLength"/>
    </geometry>
  </planView>
  <lanes>
    <laneSection s="0.0">
      <left>
        <lane id="1" type="driving" level="false">
          <link>
            <predecessor id="1"/>
            <successor id="1"/>
          </link>
          <width sOffset="0.0" a="3.5" b="0" c="0" d="0"/>
        </lane>
      </left>
      <center>
        <lane id="0" type="none" level="false"/>
      </center>
      <right/>
    </laneSection>
  </lanes>
  <signals>
    <signal s="1.0" t="0.0" id="100" name="sig1" dynamic="no" orientation="+"
            zOffset="0" type="1000001" subtype="-1" country="OpenDRIVE"/>
  </signals>
  <objects>
    <object type="crosswalk" name="cw1" id="200" s="1.0" t="0.0" zOffset="0"
            validLength="0" orientation="+" length="1" width="1" height="1"/>
  </objects>
</road>
"""

_JUNCTION_1 = """
<junction name="j1" id="20">
  <connection id="0" incomingRoad="1" connectingRoad="2" contactPoint="start">
    <laneLink from="1" to="1"/>
  </connection>
  <connection id="1" incomingRoad="2" connectingRoad="1" contactPoint="end">
    <laneLink from="-1" to="-1"/>
  </connection>
</junction>
"""


def _road_with_hdgs(road_id: int, hdgs_rad: list[float]) -> str:
    """Build a minimal road with one `<planView><geometry>` per hdg value.

    Only `planView` (and a bare `laneSection` to keep the fixture a valid
    road) is populated - links/signals/objects are irrelevant to the
    heading-delta metrics under test.
    """
    geometries = "".join(
        f'<geometry s="{i * 10.0}" x="0.0" y="0.0" hdg="{hdg!r}" length="10.0">'
        '<paramPoly3 aU="0" bU="1" cU="0" dU="0" aV="0" bV="0" cV="0" dV="0" '
        'pRange="arcLength"/>'
        "</geometry>"
        for i, hdg in enumerate(hdgs_rad)
    )
    return f"""
<road name="r{road_id}" length="{10.0 * len(hdgs_rad)!r}" id="{road_id}" junction="-1">
  <planView>
    {geometries}
  </planView>
  <lanes>
    <laneSection s="0.0">
      <center>
        <lane id="0" type="none" level="false"/>
      </center>
    </laneSection>
  </lanes>
</road>
"""


def _hdgs_from_deg_deltas(deltas_deg: list[float]) -> list[float]:
    """Cumulative headings (radians) that produce the given degree deltas.

    Returns `len(deltas_deg) + 1` headings, starting at 0.0, so that the
    consecutive-geometry deltas computed by `_hdg_deltas_deg` reproduce
    `deltas_deg` (up to floating-point rounding).
    """
    hdg = 0.0
    hdgs = [hdg]
    for delta_deg in deltas_deg:
        hdg += math.radians(delta_deg)
        hdgs.append(hdg)
    return hdgs


def test_normal_case_counts(tmp_path):
    """A road + a junction produce exactly the expected counts."""
    path = _write(tmp_path, _ROAD_1 + _JUNCTION_1)
    digest = build_digest(path)

    assert digest["counts"]["road"] == 1
    assert digest["counts"]["junction"] == 1
    assert digest["counts"]["connection"] == 2
    assert digest["counts"]["signal"] == 1
    assert digest["counts"]["object"]["total"] == 1
    assert digest["counts"]["object"]["by_type"] == {"crosswalk": 1}
    assert digest["lane"]["total"] == 2  # left driving lane + center lane
    assert digest["lane"]["by_side"] == {"left": 1, "center": 1, "right": 0}
    assert digest["lane"]["lane_section_count"] == 1
    assert digest["geometry"]["total_segments"] == 1
    assert digest["geometry"]["by_type"] == {"paramPoly3": 1}
    assert digest["roads"][0]["geometry_types"] == ["paramPoly3"]
    assert digest["junctions"] == [{"id": 20, "connection_count": 2}]


def test_normalization_rounds_floats(tmp_path):
    """Coordinates round to 3dp, lengths round to 2dp."""
    path = _write(tmp_path, _ROAD_1)
    digest = build_digest(path)

    # north="12345.678999" -> rounded to 3 decimal places.
    assert digest["header"]["north"] == 12345.679
    # length="32.774999" -> rounded to 2 decimal places.
    assert digest["roads"][0]["length"] == 32.77


def test_normalization_excludes_date(tmp_path):
    """The header `date` attribute must not leak into the digest anywhere."""
    path = _write(tmp_path, _ROAD_1)
    digest = build_digest(path)

    assert "date" not in digest["header"]
    rendered = format_digest(digest)
    assert "2026-06-16" not in rendered


def test_normalization_sorts_roads_by_id(tmp_path):
    """Roads must be sorted ascending by id regardless of file order."""
    road_5 = _ROAD_1.replace('id="1"', 'id="5"', 1)
    road_2 = _ROAD_1.replace('id="1"', 'id="2"', 1)
    # Written out of order: 5 then 2.
    path = _write(tmp_path, road_5 + road_2)
    digest = build_digest(path)

    assert [r["id"] for r in digest["roads"]] == [2, 5]


def test_road_level_vs_lane_level_link_disambiguation(tmp_path):
    """Road-level and lane-level <predecessor>/<successor> must not be conflated.

    Both reuse the identical tag name; only the `elementType` attribute
    (present on road-level links, absent on lane-level links) tells them
    apart. _ROAD_1 has exactly one road-level predecessor + successor
    (with elementType) and one lane-level predecessor + successor
    (without elementType).
    """
    path = _write(tmp_path, _ROAD_1)
    digest = build_digest(path)

    assert digest["counts"]["road_level_link"] == {"predecessor": 1, "successor": 1}
    assert digest["counts"]["lane_level_link"] == {"predecessor": 1, "successor": 1}


def test_zero_and_near_zero_length_roads(tmp_path):
    """`length="0"`, the exact `0.01` clamp value, and small-length buckets."""

    def road_with_length(road_id: int, length: str) -> str:
        return _ROAD_1.replace('id="1"', f'id="{road_id}"', 1).replace(
            'length="32.774999"', f'length="{length}"', 1
        )

    body = "".join(
        [
            road_with_length(1, "0"),
            road_with_length(2, "0.01"),
            road_with_length(3, "0.05"),
            road_with_length(4, "0.5"),
            road_with_length(5, "50"),
        ]
    )
    path = _write(tmp_path, body)
    digest = build_digest(path)

    assert digest["length"]["zero_count"] == 1
    assert digest["length"]["exactly_0_01_count"] == 1
    # Cumulative, not a disjoint bucket: length < 0.1 covers roads 1 (0),
    # 2 (0.01), 3 (0.05).
    assert digest["length"]["lt_0_1_count"] == 3
    # Cumulative: length < 1.0 covers roads 1, 2, 3, 4 (0.5).
    assert digest["length"]["lt_1_0_count"] == 4
    # The disjoint range [0.1, 1.0) -- road 4 only -- is derived by the
    # caller as lt_1_0_count - lt_0_1_count, not stored directly.
    assert digest["length"]["lt_1_0_count"] - digest["length"]["lt_0_1_count"] == 1


def test_total_length_sums_raw_then_rounds_once(tmp_path):
    """`total_m` must round the *sum* once, not sum pre-rounded road lengths.

    Three roads of raw length 0.004 each individually round to 0.00 at 2dp,
    so summing the rounded values gives 0.00. Summing the raw values first
    (0.012) and rounding once gives the correct 0.01. This mirrors the real
    artifact's 693-road drift (27,664.34 from round-then-sum vs the correct
    27,664.37 from sum-then-round).
    """

    def road_with_length(road_id: int, length: str) -> str:
        return _ROAD_1.replace('id="1"', f'id="{road_id}"', 1).replace(
            'length="32.774999"', f'length="{length}"', 1
        )

    body = "".join(road_with_length(i, "0.004") for i in range(1, 4))
    path = _write(tmp_path, body)
    digest = build_digest(path)

    assert digest["length"]["total_m"] == 0.01
    # Each road's own displayed length is still rounded individually.
    assert [r["length"] for r in digest["roads"]] == [0.0, 0.0, 0.0]


def test_exactly_0_01_count_uses_raw_strict_equality(tmp_path):
    """`exactly_0_01_count` must match the raw value, not the rounded one.

    0.0104 and 0.0096 both round to 0.01 at 2dp but are not exactly 0.01, so
    a rounded-value comparison would (incorrectly) count them.
    "0.010000000000000002" is a double that is bit-distinct from 0.01
    (confirmed: float("0.01") != float("0.010000000000000002")) yet is
    indistinguishable from it once rounded to 2dp, so it exercises the same
    raw-vs-rounded pitfall from the opposite direction. Only the road whose
    raw length is exactly 0.01 must be counted.
    """

    def road_with_length(road_id: int, length: str) -> str:
        return _ROAD_1.replace('id="1"', f'id="{road_id}"', 1).replace(
            'length="32.774999"', f'length="{length}"', 1
        )

    body = "".join(
        [
            road_with_length(1, "0.01"),
            road_with_length(2, "0.0104"),
            road_with_length(3, "0.0096"),
            road_with_length(4, "0.010000000000000002"),
        ]
    )
    path = _write(tmp_path, body)
    digest = build_digest(path)

    assert digest["length"]["exactly_0_01_count"] == 1


def test_cumulative_length_buckets_are_exclusive_at_their_own_threshold(tmp_path):
    """`lt_0_1_count`/`lt_1_0_count` use a strict `<`: the threshold value
    itself belongs to the *next* bucket, not the one it names.
    """

    def road_with_length(road_id: int, length: str) -> str:
        return _ROAD_1.replace('id="1"', f'id="{road_id}"', 1).replace(
            'length="32.774999"', f'length="{length}"', 1
        )

    body = "".join(
        [
            road_with_length(1, "0.05"),  # < 0.1 and < 1.0
            road_with_length(2, "0.1"),  # not < 0.1, but < 1.0
            road_with_length(3, "1.0"),  # not < 0.1, not < 1.0
        ]
    )
    path = _write(tmp_path, body)
    digest = build_digest(path)

    assert digest["length"]["lt_0_1_count"] == 1
    assert digest["length"]["lt_1_0_count"] == 2


def test_width_metrics_single_segment_constant_width(tmp_path):
    """A lane with one constant `<width>` segment reports that constant value.

    _ROAD_1's left lane (id=1) has a single `<width sOffset="0.0" a="3.5"
    b="0" c="0" d="0"/>`; its center lane (id=0) has none. `lanes` must
    contain exactly the left lane's entry - the center lane is skipped
    entirely rather than emitted with a zero-segment placeholder (see the
    exclusion comment in `_process_road`: center lanes never carry a
    `<width>` in the generator's output).
    """
    path = _write(tmp_path, _ROAD_1)
    digest = build_digest(path)

    assert digest["roads"][0]["lanes"] == [
        {
            "side": "left",
            "id": 1,
            "lane_section_s": 0.0,
            "width_segment_count": 1,
            "width_min_m": 3.5,
            "width_max_m": 3.5,
            "width_mean_m": 3.5,
        }
    ]


def test_width_metrics_multi_segment_lane_takes_min_max_across_segments(tmp_path):
    """Two piecewise-constant `<width>` segments: min/max span both of them.

    Neither segment alone contains the other's value, so a metric that
    skipped a segment entirely under a fixed, small global sample count
    (rather than sampling every segment) would under-report the true
    min/max - this is the road-65/67 multi-segment-lane scenario in
    miniature (those lanes have 4-7 `<width>` segments each).
    """
    road = """
<road name="r1" length="10.0" id="1" junction="-1">
  <lanes>
    <laneSection s="0.0">
      <left>
        <lane id="1" type="driving" level="false">
          <width sOffset="0.0" a="2.0" b="0" c="0" d="0"/>
          <width sOffset="5.0" a="4.0" b="0" c="0" d="0"/>
        </lane>
      </left>
      <center>
        <lane id="0" type="none" level="false"/>
      </center>
    </laneSection>
  </lanes>
</road>
"""
    path = _write(tmp_path, road)
    digest = build_digest(path)

    assert digest["roads"][0]["lanes"] == [
        {
            "side": "left",
            "id": 1,
            "lane_section_s": 0.0,
            "width_segment_count": 2,
            "width_min_m": 2.0,
            "width_max_m": 4.0,
            "width_mean_m": 3.0,
        }
    ]


def test_width_metrics_cubic_polynomial_evaluated_correctly(tmp_path):
    """`width_min/max/mean_m` must use the full cubic, not just the `a` term.

    a=1.0, b=0.1, c=0.01, d=0.001 over a 10m span, sampled at this segment's
    own start/mid/end (ds=0/5/10 per `_lane_width_samples`):
      ds=0  -> 1.0
      ds=5  -> 1.0 + 0.1*5 + 0.01*25 + 0.001*125  = 1.875
      ds=10 -> 1.0 + 0.1*10 + 0.01*100 + 0.001*1000 = 4.0
    """
    road = """
<road name="r1" length="10.0" id="1" junction="-1">
  <lanes>
    <laneSection s="0.0">
      <left>
        <lane id="1" type="driving" level="false">
          <width sOffset="0.0" a="1.0" b="0.1" c="0.01" d="0.001"/>
        </lane>
      </left>
      <center>
        <lane id="0" type="none" level="false"/>
      </center>
    </laneSection>
  </lanes>
</road>
"""
    path = _write(tmp_path, road)
    digest = build_digest(path)

    lane = digest["roads"][0]["lanes"][0]
    assert lane["width_min_m"] == pytest.approx(1.0, abs=1e-6)
    assert lane["width_max_m"] == pytest.approx(4.0, abs=1e-6)
    assert lane["width_mean_m"] == pytest.approx(2.292, abs=1e-3)


def test_width_metrics_sorted_by_side_then_id(tmp_path):
    """`lanes` entries must be sorted `(lane_section_s, side, id)`, not file order."""
    road = """
<road name="r1" length="10.0" id="1" junction="-1">
  <lanes>
    <laneSection s="0.0">
      <left>
        <lane id="2" type="driving" level="false">
          <width sOffset="0.0" a="3.0" b="0" c="0" d="0"/>
        </lane>
        <lane id="1" type="driving" level="false">
          <width sOffset="0.0" a="3.0" b="0" c="0" d="0"/>
        </lane>
      </left>
      <right>
        <lane id="1" type="driving" level="false">
          <width sOffset="0.0" a="3.0" b="0" c="0" d="0"/>
        </lane>
      </right>
    </laneSection>
  </lanes>
</road>
"""
    path = _write(tmp_path, road)
    digest = build_digest(path)

    entries = digest["roads"][0]["lanes"]
    assert [(e["side"], e["id"]) for e in entries] == [
        ("left", 1),
        ("left", 2),
        ("right", 1),
    ]


def test_hdg_delta_straight_road_is_all_zero(tmp_path):
    """A road whose geometries all share the same `hdg` is a straight line."""
    road = _road_with_hdgs(1, _hdgs_from_deg_deltas([0.0, 0.0, 0.0]))
    path = _write(tmp_path, road)
    digest = build_digest(path)

    entry = digest["roads"][0]
    assert entry["max_abs_hdg_delta_deg"] == 0.0
    assert entry["total_hdg_change_deg"] == 0.0
    assert entry["hdg_sign_changes"] == 0


def test_hdg_delta_smooth_curve_has_large_total_but_small_max(tmp_path):
    """A gentle, steady curve accumulates a large total but a small per-step max.

    Four consecutive +3 deg steps: `total` (12 deg) is far larger than any
    single `max_abs` (3 deg), and since every step turns the same way there
    are no sign reversals.
    """
    road = _road_with_hdgs(1, _hdgs_from_deg_deltas([3.0, 3.0, 3.0, 3.0]))
    path = _write(tmp_path, road)
    digest = build_digest(path)

    entry = digest["roads"][0]
    assert entry["max_abs_hdg_delta_deg"] == pytest.approx(3.0, abs=1e-6)
    assert entry["total_hdg_change_deg"] == pytest.approx(12.0, abs=1e-6)
    assert entry["hdg_sign_changes"] == 0


def test_hdg_delta_meandering_road_counts_sign_changes(tmp_path):
    """Deltas alternating +/-/+/- must count 3 direction reversals."""
    road = _road_with_hdgs(1, _hdgs_from_deg_deltas([20.0, -10.0, 15.0, -5.0]))
    path = _write(tmp_path, road)
    digest = build_digest(path)

    entry = digest["roads"][0]
    assert entry["hdg_sign_changes"] == 3
    assert entry["max_abs_hdg_delta_deg"] == pytest.approx(20.0, abs=1e-6)
    assert entry["total_hdg_change_deg"] == pytest.approx(20.0, abs=1e-6)


def test_hdg_delta_wraps_across_pi_boundary(tmp_path):
    """`hdg` wrapping past +-pi must not be reported as a near-360 deg jump.

    170 deg followed by -170 deg is a 20 deg turn in reality (both headings
    point almost the same way, just represented on opposite sides of the
    +-180 wraparound), not the ~340 deg a naive subtraction would produce.
    This is the case the resent report called out as most important to get
    right.
    """
    hdgs_rad = [math.radians(170.0), math.radians(-170.0)]
    road = _road_with_hdgs(1, hdgs_rad)
    path = _write(tmp_path, road)
    digest = build_digest(path)

    entry = digest["roads"][0]
    assert entry["max_abs_hdg_delta_deg"] == pytest.approx(20.0, abs=1e-6)
    assert entry["total_hdg_change_deg"] == pytest.approx(20.0, abs=1e-6)
    # The un-normalized jump would have been ~340 deg; make sure that
    # spurious value never leaks out.
    assert abs(entry["total_hdg_change_deg"]) < 180.0


def test_hdg_delta_zero_or_one_geometry_does_not_crash(tmp_path):
    """0 or 1 geometries leave a delta undefined; must report 0, not crash."""
    no_geometry = _road_with_hdgs(1, [])
    one_geometry = _road_with_hdgs(2, [1.234])
    path = _write(tmp_path, no_geometry + one_geometry)
    digest = build_digest(path)

    for entry in digest["roads"]:
        assert entry["max_abs_hdg_delta_deg"] == 0.0
        assert entry["total_hdg_change_deg"] == 0.0
        assert entry["hdg_sign_changes"] == 0


def test_hdg_delta_map_level_percentiles_use_linear_interpolation(tmp_path):
    """`hdg_delta_p50/p90/p99_deg` must use linear interpolation between
    closest ranks (numpy's default `'linear'`, Excel's `PERCENTILE.INC`),
    pinned here so a future refactor cannot silently shift these numbers
    (see `_percentile`'s docstring for why that would be a problem).

    Ten roads with `max_abs_hdg_delta_deg` = 1, 2, ..., 10 (each road has a
    single delta of exactly that many degrees). For values 1..10 (n=10),
    linear interpolation gives:
      p50 -> rank = 0.50 * 9 = 4.5  -> halfway between values[4]=5, values[5]=6 -> 5.5
      p90 -> rank = 0.90 * 9 = 8.1  -> values[8]=9 + 0.1 * (values[9] - values[8]) -> 9.1
      p99 -> rank = 0.99 * 9 = 8.91 -> values[8]=9 + 0.91 * (values[9] - values[8]) -> 9.91
    """
    roads = "".join(
        _road_with_hdgs(i, _hdgs_from_deg_deltas([float(i)])) for i in range(1, 11)
    )
    path = _write(tmp_path, roads)
    digest = build_digest(path)

    assert digest["max_abs_hdg_delta_deg"] == pytest.approx(10.0, abs=1e-6)
    assert digest["hdg_delta_p50_deg"] == pytest.approx(5.5, abs=1e-6)
    assert digest["hdg_delta_p90_deg"] == pytest.approx(9.1, abs=1e-6)
    assert digest["hdg_delta_p99_deg"] == pytest.approx(9.91, abs=1e-6)


def test_hdg_delta_map_level_percentiles_single_road_returns_its_own_value(tmp_path):
    """A single road's percentiles must equal its own value, not divide by
    zero (the multi-element interpolation formula divides by `n - 1`)."""
    road = _road_with_hdgs(1, _hdgs_from_deg_deltas([7.0]))
    path = _write(tmp_path, road)
    digest = build_digest(path)

    assert digest["max_abs_hdg_delta_deg"] == pytest.approx(7.0, abs=1e-6)
    assert digest["hdg_delta_p50_deg"] == pytest.approx(7.0, abs=1e-6)
    assert digest["hdg_delta_p90_deg"] == pytest.approx(7.0, abs=1e-6)
    assert digest["hdg_delta_p99_deg"] == pytest.approx(7.0, abs=1e-6)


def test_empty_xodr_zero_roads_does_not_crash(tmp_path):
    """A `.xodr` with zero roads/junctions must not raise and reports zeros."""
    path = _write(tmp_path, "")
    digest = build_digest(path)

    assert digest["counts"]["road"] == 0
    assert digest["counts"]["junction"] == 0
    assert digest["roads"] == []
    assert digest["junctions"] == []
    assert digest["length"] == {
        "total_m": 0.0,
        "median_m": 0.0,
        "mean_m": 0.0,
        "min_m": 0.0,
        "max_m": 0.0,
        "zero_count": 0,
        "exactly_0_01_count": 0,
        "lt_0_1_count": 0,
        "lt_1_0_count": 0,
    }
    assert digest["max_abs_hdg_delta_deg"] == 0.0
    assert digest["hdg_delta_p50_deg"] == 0.0
    assert digest["hdg_delta_p90_deg"] == 0.0
    assert digest["hdg_delta_p99_deg"] == 0.0

    # format_digest must also handle the empty-arrays case without error.
    rendered = format_digest(digest)
    parsed = json.loads(rendered)
    assert parsed["roads"] == []
    assert parsed["junctions"] == []


def test_format_digest_puts_one_road_per_line(tmp_path):
    """Each road entry must render as a single, independently-parseable JSON line."""
    road_a = _ROAD_1.replace('id="1"', 'id="1"', 1)
    road_b = _ROAD_1.replace('id="1"', 'id="2"', 1)
    path = _write(tmp_path, road_a + road_b)
    digest = build_digest(path)

    rendered = format_digest(digest)
    road_lines = [
        line.strip().rstrip(",")
        for line in rendered.splitlines()
        if '"id": 1,' in line or '"id": 2,' in line
    ]
    assert len(road_lines) == 2
    for line in road_lines:
        parsed = json.loads(line)
        assert parsed["id"] in (1, 2)


def test_main_writes_digest_to_stdout(tmp_path, monkeypatch, capsys):
    path = _write(tmp_path, _ROAD_1)
    monkeypatch.setattr(sys, "argv", ["xodr-digest", str(path)])

    main()

    captured = capsys.readouterr()
    parsed = json.loads(captured.out)
    assert parsed["counts"]["road"] == 1


def test_main_writes_digest_to_output_file(tmp_path, monkeypatch):
    path = _write(tmp_path, _ROAD_1)
    out_path = tmp_path / "digest.json"
    monkeypatch.setattr(
        sys, "argv", ["xodr-digest", str(path), "--output", str(out_path)]
    )

    main()

    parsed = json.loads(out_path.read_text(encoding="utf-8"))
    assert parsed["counts"]["road"] == 1


def test_main_exits_2_when_file_not_found(tmp_path, monkeypatch, capsys):
    missing = tmp_path / "does_not_exist.xodr"
    monkeypatch.setattr(sys, "argv", ["xodr-digest", str(missing)])

    with pytest.raises(SystemExit) as excinfo:
        main()

    assert excinfo.value.code == 2
    captured = capsys.readouterr()
    assert "file not found" in captured.err


def test_build_digest_is_reexported_symbol_used_by_module():
    # Guard against accidental renames breaking the public API surface
    # exercised above (build_digest / format_digest / main).
    assert xodr_digest.build_digest is build_digest
    assert xodr_digest.format_digest is format_digest
