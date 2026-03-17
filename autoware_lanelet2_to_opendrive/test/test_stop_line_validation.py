"""Tests for stop line validation in analyze_xodr and road_lanelet_geo_mapping."""

from __future__ import annotations

import json
import textwrap
from pathlib import Path

import pytest

from autoware_lanelet2_to_opendrive.road_lanelet_geo_mapping import (
    GeoRoadLaneletMapping,
    SkippedStopLineEntry,
    StopLineMappingEntry,
    save_mapping_json,
)


# ---------------------------------------------------------------------------
# StopLineMappingEntry / SkippedStopLineEntry
# ---------------------------------------------------------------------------


class TestStopLineMappingEntry:
    def test_to_dict(self) -> None:
        entry = StopLineMappingEntry(road_id=5, signal_types=[294])
        assert entry.to_dict() == {"road_id": 5, "signal_types": [294]}

    def test_from_dict(self) -> None:
        entry = StopLineMappingEntry.from_dict(
            {"road_id": 12, "signal_types": [205, 294]}
        )
        assert entry.road_id == 12
        assert entry.signal_types == [205, 294]

    def test_round_trip(self) -> None:
        original = StopLineMappingEntry(road_id=8, signal_types=[206])
        restored = StopLineMappingEntry.from_dict(original.to_dict())
        assert restored.road_id == original.road_id
        assert restored.signal_types == original.signal_types


class TestSkippedStopLineEntry:
    def test_to_dict(self) -> None:
        entry = SkippedStopLineEntry(reason="no_nearest_road")
        assert entry.to_dict() == {"reason": "no_nearest_road"}

    def test_from_dict(self) -> None:
        entry = SkippedStopLineEntry.from_dict({"reason": "construction_failed"})
        assert entry.reason == "construction_failed"


# ---------------------------------------------------------------------------
# GeoRoadLaneletMapping version 3/4 compatibility
# ---------------------------------------------------------------------------


class TestGeoRoadLaneletMappingVersion:
    def test_version_3_from_dict_has_none_stop_line_fields(self) -> None:
        """Version 3 JSON (no stop line info) should yield None fields."""
        data = {
            "version": 3,
            "xodr_sha256": "aaa",
            "osm_sha256": "bbb",
            "lanelet_to_road_and_lane": {"10": [1, -1]},
        }
        mapping = GeoRoadLaneletMapping.from_dict(data)
        assert mapping.stop_line_mapping is None
        assert mapping.skipped_stop_lines is None
        assert mapping.lanelet_to_road_and_lane == {10: (1, -1)}

    def test_version_4_round_trip(self) -> None:
        """Version 4 JSON with stop line data survives round-trip."""
        original = GeoRoadLaneletMapping(
            xodr_sha256="aaa",
            osm_sha256="bbb",
            lanelet_to_road_and_lane={10: (1, -1)},
            stop_line_mapping={
                1812: StopLineMappingEntry(road_id=5, signal_types=[294]),
                1230: StopLineMappingEntry(road_id=8, signal_types=[205, 294]),
            },
            skipped_stop_lines={
                2001: SkippedStopLineEntry(reason="no_nearest_road"),
                2002: SkippedStopLineEntry(reason="construction_failed"),
            },
        )
        data = original.to_dict()
        assert data["version"] == 4
        assert "stop_line_mapping" in data
        assert "skipped_stop_lines" in data

        restored = GeoRoadLaneletMapping.from_dict(data)
        assert restored.stop_line_mapping is not None
        assert restored.skipped_stop_lines is not None
        assert restored.stop_line_mapping[1812].road_id == 5
        assert restored.stop_line_mapping[1812].signal_types == [294]
        assert restored.stop_line_mapping[1230].signal_types == [205, 294]
        assert restored.skipped_stop_lines[2001].reason == "no_nearest_road"
        assert restored.skipped_stop_lines[2002].reason == "construction_failed"

    def test_version_4_json_file(self, tmp_path: Path) -> None:
        """Version 4 mapping can be saved and loaded from JSON file."""
        xodr_path = tmp_path / "test.xodr"
        xodr_path.write_text("<OpenDRIVE/>")

        mapping = GeoRoadLaneletMapping(
            xodr_sha256="abc",
            osm_sha256="def",
            lanelet_to_road_and_lane={10: (1, -1)},
            stop_line_mapping={
                100: StopLineMappingEntry(road_id=1, signal_types=[294]),
            },
            skipped_stop_lines={
                200: SkippedStopLineEntry(reason="no_nearest_road"),
            },
        )
        result_path = save_mapping_json(mapping, xodr_path)
        data = json.loads(result_path.read_text(encoding="utf-8"))
        assert data["version"] == 4
        assert data["stop_line_mapping"]["100"]["road_id"] == 1
        assert data["skipped_stop_lines"]["200"]["reason"] == "no_nearest_road"

    def test_version_4_without_stop_line_data(self) -> None:
        """Version 4 mapping without stop line data serializes cleanly."""
        mapping = GeoRoadLaneletMapping(
            xodr_sha256="aaa",
            osm_sha256="bbb",
            lanelet_to_road_and_lane={},
        )
        data = mapping.to_dict()
        assert data["version"] == 4
        assert "stop_line_mapping" not in data
        assert "skipped_stop_lines" not in data


# ---------------------------------------------------------------------------
# _parse_stop_lines_from_xodr
# ---------------------------------------------------------------------------


class TestParseStopLinesFromXodr:
    def _write_xodr(self, tmp_path: Path, content: str) -> Path:
        xodr_path = tmp_path / "test.xodr"
        xodr_path.write_text(content, encoding="utf-8")
        return xodr_path

    def test_standard_stop_line_object(self, tmp_path: Path) -> None:
        from autoware_lanelet2_to_opendrive.analyze_xodr import (
            _parse_stop_lines_from_xodr,
        )

        xodr = textwrap.dedent("""\
            <?xml version="1.0" encoding="UTF-8"?>
            <OpenDRIVE>
              <road id="5">
                <objects>
                  <object id="1812" type="stopLine" name="stop_line_1812"
                          s="10.0" t="0.0" zOffset="0.0" hdg="0.0"
                          length="3.0" width="0.1"/>
                </objects>
                <signals/>
              </road>
            </OpenDRIVE>
        """)
        obj_to_road, sig_types, all_sigs = _parse_stop_lines_from_xodr(
            self._write_xodr(tmp_path, xodr)
        )
        assert obj_to_road == {1812: 5}
        assert sig_types == {}
        assert all_sigs == set()

    def test_carla_stop_line_object(self, tmp_path: Path) -> None:
        from autoware_lanelet2_to_opendrive.analyze_xodr import (
            _parse_stop_lines_from_xodr,
        )

        xodr = textwrap.dedent("""\
            <?xml version="1.0" encoding="UTF-8"?>
            <OpenDRIVE>
              <road id="12">
                <objects>
                  <object id="999" type="-1" name="Stencil_STOP"
                          s="5.0" t="0.0" zOffset="0.0" hdg="0.0"
                          length="2.0" width="0.1"/>
                </objects>
                <signals/>
              </road>
            </OpenDRIVE>
        """)
        obj_to_road, _, _ = _parse_stop_lines_from_xodr(
            self._write_xodr(tmp_path, xodr)
        )
        assert obj_to_road == {999: 12}

    def test_signal_name_parsing(self, tmp_path: Path) -> None:
        from autoware_lanelet2_to_opendrive.analyze_xodr import (
            _parse_stop_lines_from_xodr,
        )

        xodr = textwrap.dedent("""\
            <?xml version="1.0" encoding="UTF-8"?>
            <OpenDRIVE>
              <road id="5">
                <objects/>
                <signals>
                  <signal id="100" name="StopLine_1812" type="294"
                          s="10.0" t="0.0" zOffset="0.0" orientation="-"
                          dynamic="no" country="OpenDRIVE" subtype="-1"/>
                  <signal id="101" name="StopSign_1864" type="206"
                          s="20.0" t="0.0" zOffset="0.0" orientation="-"
                          dynamic="no" country="OpenDRIVE" subtype="-1"/>
                  <signal id="102" name="YieldSign_1230" type="205"
                          s="30.0" t="0.0" zOffset="0.0" orientation="-"
                          dynamic="no" country="OpenDRIVE" subtype="-1"/>
                  <signal id="103" name="StopLine_1230" type="294"
                          s="30.0" t="0.0" zOffset="0.0" orientation="-"
                          dynamic="no" country="OpenDRIVE" subtype="-1"/>
                  <signal id="50" name="TrafficLight_999" type="1000001"
                          s="10.0" t="-5.0" zOffset="3.0" orientation="-"
                          dynamic="yes" country="OpenDRIVE" subtype="-1"/>
                </signals>
              </road>
            </OpenDRIVE>
        """)
        _, sig_types, all_sigs = _parse_stop_lines_from_xodr(
            self._write_xodr(tmp_path, xodr)
        )
        assert sig_types[1812] == [294]
        assert sig_types[1864] == [206]
        assert sorted(sig_types[1230]) == [205, 294]
        assert all_sigs == {50, 100, 101, 102, 103}

    def test_non_stop_line_objects_ignored(self, tmp_path: Path) -> None:
        from autoware_lanelet2_to_opendrive.analyze_xodr import (
            _parse_stop_lines_from_xodr,
        )

        xodr = textwrap.dedent("""\
            <?xml version="1.0" encoding="UTF-8"?>
            <OpenDRIVE>
              <road id="1">
                <objects>
                  <object id="500" type="barrier" name="some_barrier"
                          s="0.0" t="0.0" zOffset="0.0" hdg="0.0"
                          length="1.0" width="1.0"/>
                </objects>
                <signals/>
              </road>
            </OpenDRIVE>
        """)
        obj_to_road, _, _ = _parse_stop_lines_from_xodr(
            self._write_xodr(tmp_path, xodr)
        )
        assert obj_to_road == {}


# ---------------------------------------------------------------------------
# _parse_signal_dependencies_from_xodr
# ---------------------------------------------------------------------------


class TestParseSignalDependencies:
    def test_dependency_parsing(self, tmp_path: Path) -> None:
        from autoware_lanelet2_to_opendrive.analyze_xodr import (
            _parse_signal_dependencies_from_xodr,
        )

        xodr = textwrap.dedent("""\
            <?xml version="1.0" encoding="UTF-8"?>
            <OpenDRIVE>
              <road id="5">
                <objects/>
                <signals>
                  <signal id="100" name="StopLine_1812" type="294"
                          s="10.0" t="0.0" zOffset="0.0" orientation="-"
                          dynamic="no" country="OpenDRIVE" subtype="-1">
                    <dependency id="50" type="trafficLight"/>
                    <dependency id="51" type="trafficLight"/>
                  </signal>
                  <signal id="50" name="TrafficLight_999" type="1000001"
                          s="10.0" t="-5.0" zOffset="3.0" orientation="-"
                          dynamic="yes" country="OpenDRIVE" subtype="-1"/>
                </signals>
              </road>
            </OpenDRIVE>
        """)
        xodr_path = tmp_path / "test.xodr"
        xodr_path.write_text(xodr, encoding="utf-8")
        deps = _parse_signal_dependencies_from_xodr(xodr_path)
        assert deps == {100: [50, 51]}

    def test_no_dependencies(self, tmp_path: Path) -> None:
        from autoware_lanelet2_to_opendrive.analyze_xodr import (
            _parse_signal_dependencies_from_xodr,
        )

        xodr = textwrap.dedent("""\
            <?xml version="1.0" encoding="UTF-8"?>
            <OpenDRIVE>
              <road id="1">
                <objects/>
                <signals>
                  <signal id="100" name="StopLine_1812" type="294"
                          s="10.0" t="0.0" zOffset="0.0" orientation="-"
                          dynamic="no" country="OpenDRIVE" subtype="-1"/>
                </signals>
              </road>
            </OpenDRIVE>
        """)
        xodr_path = tmp_path / "test.xodr"
        xodr_path.write_text(xodr, encoding="utf-8")
        deps = _parse_signal_dependencies_from_xodr(xodr_path)
        assert deps == {}


# ---------------------------------------------------------------------------
# Integration test with nishishinjuku_carla.xodr
# ---------------------------------------------------------------------------

_TEST_DATA = Path(__file__).parent / "data"
_XODR_PATH = _TEST_DATA / "nishishinjuku_carla.xodr"


@pytest.mark.skipif(
    not _XODR_PATH.exists(),
    reason="nishishinjuku_carla.xodr test data not available",
)
class TestNishishinjukuCarlaStopLines:
    def test_parse_carla_stop_line_objects(self) -> None:
        """nishishinjuku_carla.xodr should contain CARLA-format stop line objects."""
        from autoware_lanelet2_to_opendrive.analyze_xodr import (
            _parse_stop_lines_from_xodr,
        )

        obj_to_road, _sig_types, _all_sigs = _parse_stop_lines_from_xodr(_XODR_PATH)
        # The plan states 196 CARLA stop line objects
        assert len(obj_to_road) > 0
        print(f"Found {len(obj_to_road)} CARLA stop line objects")
