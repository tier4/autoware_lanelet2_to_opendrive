"""Tests for road_lanelet_geo_mapping: MappingMismatchError, validate, save."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from autoware_lanelet2_to_opendrive.road_lanelet_geo_mapping import (
    GeoRoadLaneletMapping,
    MappingMismatchError,
    save_mapping_json,
    validate_mapping_consistency,
)


# ---------------------------------------------------------------------------
# validate_mapping_consistency
# ---------------------------------------------------------------------------


class TestValidateMappingConsistency:
    def test_matching_mappings_pass(self) -> None:
        conversion = {10: (1, -1), 20: (2, -1)}
        geo = GeoRoadLaneletMapping(
            xodr_sha256="a",
            osm_sha256="b",
            lanelet_to_road_and_lane={10: (1, -1), 20: (2, -1)},
        )
        # Should not raise
        validate_mapping_consistency(conversion, geo)

    def test_empty_mappings_pass(self) -> None:
        geo = GeoRoadLaneletMapping(xodr_sha256="a", osm_sha256="b")
        validate_mapping_consistency({}, geo)

    def test_value_mismatch_raises(self) -> None:
        conversion = {10: (1, -1), 20: (2, -1)}
        geo = GeoRoadLaneletMapping(
            xodr_sha256="a",
            osm_sha256="b",
            lanelet_to_road_and_lane={10: (1, -1), 20: (2, -2)},
        )
        with pytest.raises(MappingMismatchError, match="1 entries differ"):
            validate_mapping_consistency(conversion, geo)

    def test_missing_in_geo_raises(self) -> None:
        conversion = {10: (1, -1), 20: (2, -1)}
        geo = GeoRoadLaneletMapping(
            xodr_sha256="a",
            osm_sha256="b",
            lanelet_to_road_and_lane={10: (1, -1)},
        )
        with pytest.raises(MappingMismatchError, match="1 entries differ"):
            validate_mapping_consistency(conversion, geo)

    def test_missing_in_conversion_raises(self) -> None:
        conversion = {10: (1, -1)}
        geo = GeoRoadLaneletMapping(
            xodr_sha256="a",
            osm_sha256="b",
            lanelet_to_road_and_lane={10: (1, -1), 20: (2, -1)},
        )
        with pytest.raises(MappingMismatchError, match="1 entries differ"):
            validate_mapping_consistency(conversion, geo)

    def test_multiple_mismatches(self) -> None:
        conversion = {10: (1, -1), 20: (2, -1), 30: (3, -1)}
        geo = GeoRoadLaneletMapping(
            xodr_sha256="a",
            osm_sha256="b",
            lanelet_to_road_and_lane={10: (1, -2), 20: (2, -2), 30: (3, -2)},
        )
        with pytest.raises(MappingMismatchError, match="3 entries differ"):
            validate_mapping_consistency(conversion, geo)


# ---------------------------------------------------------------------------
# save_mapping_json
# ---------------------------------------------------------------------------


class TestSaveMappingJson:
    def test_creates_json_file(self, tmp_path: Path) -> None:
        xodr_path = tmp_path / "test.xodr"
        xodr_path.write_text("<OpenDRIVE/>")

        mapping = GeoRoadLaneletMapping(
            xodr_sha256="abc",
            osm_sha256="def",
            lanelet_to_road_and_lane={10: (1, -1), 20: (2, -1)},
        )
        result_path = save_mapping_json(mapping, xodr_path)

        expected_path = tmp_path / "test.mapping.json"
        assert result_path == expected_path
        assert expected_path.exists()

    def test_json_content_valid(self, tmp_path: Path) -> None:
        xodr_path = tmp_path / "test.xodr"
        xodr_path.write_text("<OpenDRIVE/>")

        mapping = GeoRoadLaneletMapping(
            xodr_sha256="abc",
            osm_sha256="def",
            lanelet_to_road_and_lane={10: (1, -1), 20: (2, -1)},
        )
        result_path = save_mapping_json(mapping, xodr_path)

        data = json.loads(result_path.read_text(encoding="utf-8"))
        assert data["version"] == 3
        assert data["xodr_sha256"] == "abc"
        assert data["osm_sha256"] == "def"
        assert "10" in data["lanelet_to_road_and_lane"]
        assert data["lanelet_to_road_and_lane"]["10"] == [1, -1]

    def test_round_trip_from_saved_json(self, tmp_path: Path) -> None:
        xodr_path = tmp_path / "test.xodr"
        xodr_path.write_text("<OpenDRIVE/>")

        original = GeoRoadLaneletMapping(
            xodr_sha256="abc",
            osm_sha256="def",
            lanelet_to_road_and_lane={10: (1, -1), 20: (2, -1)},
        )
        result_path = save_mapping_json(original, xodr_path)

        data = json.loads(result_path.read_text(encoding="utf-8"))
        restored = GeoRoadLaneletMapping.from_dict(data)
        assert restored.lanelet_to_road_and_lane == original.lanelet_to_road_and_lane
        assert restored.xodr_sha256 == original.xodr_sha256
        assert restored.osm_sha256 == original.osm_sha256


# ---------------------------------------------------------------------------
# MappingMismatchError
# ---------------------------------------------------------------------------


class TestMappingMismatchError:
    def test_is_exception(self) -> None:
        assert issubclass(MappingMismatchError, Exception)

    def test_can_be_raised_and_caught(self) -> None:
        with pytest.raises(MappingMismatchError):
            raise MappingMismatchError("test message")
