"""Tests for road_lanelet_geo_mapping: MappingMismatchError, validate, save."""

from __future__ import annotations

import json
from pathlib import Path
from unittest.mock import MagicMock

import pytest

from autoware_lanelet2_to_opendrive.road_lanelet_geo_mapping import (
    GeoRoadLaneletMapping,
    MappingMismatchError,
    _RoadCandidates,
    _resolve_conflicts,
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
        assert data["version"] == 4
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


# ---------------------------------------------------------------------------
# _resolve_conflicts
# ---------------------------------------------------------------------------


def _make_rc(
    road_id: int,
    candidates: list[tuple[float, int]],
    raw_dists: dict[int, float] | None = None,
) -> _RoadCandidates:
    """Helper to build a minimal _RoadCandidates for conflict-resolution tests."""
    return _RoadCandidates(
        road=MagicMock(),
        road_id=road_id,
        lane_ids=[-1],
        is_rht=True,
        ref_line=MagicMock(),
        candidates=candidates,
        raw_dists=raw_dists or {lid: dist for dist, lid in candidates},
        walk_lane_ids=[-1],
    )


class TestResolveConflicts:
    """Unit tests for the _resolve_conflicts function."""

    def test_no_conflict(self) -> None:
        """Two roads claim different lanelets — both keep index 0."""
        rc_a = _make_rc(100, [(1.0, 10)])
        rc_b = _make_rc(200, [(2.0, 20)])
        result = _resolve_conflicts([rc_a, rc_b])
        assert result == {0: 0, 1: 0}

    def test_2way_cost_advance_loser(self) -> None:
        """Greedy loser has a cheap alternative — advance loser (same as greedy)."""
        # Both claim lanelet 10. Road 100 is closer (1.0) than 200 (2.0).
        # Loser (200) next candidate: (3.0, 20). Winner (100) next: (10.0, 30).
        # cost_advance_loser = 1.0 + 3.0 = 4.0
        # cost_advance_winner = 2.0 + 10.0 = 12.0
        # Advance loser is cheaper -> advance loser.
        rc_a = _make_rc(100, [(1.0, 10), (10.0, 30)])
        rc_b = _make_rc(200, [(2.0, 10), (3.0, 20)])
        result = _resolve_conflicts([rc_a, rc_b])
        # Road 100 keeps lanelet 10 (idx 0), road 200 advances to lanelet 20 (idx 1)
        assert result == {0: 0, 1: 1}

    def test_2way_cost_advance_winner(self) -> None:
        """Winner has a cheap alternative — cost-aware advances winner instead."""
        # Both claim lanelet 10. Road 100 is closer (1.0) than 200 (2.0).
        # Loser (200) next candidate: (20.0, 20). Winner (100) next: (1.5, 30).
        # cost_advance_loser = 1.0 + 20.0 = 21.0
        # cost_advance_winner = 2.0 + 1.5 = 3.5
        # Advance winner is cheaper -> advance winner (differs from greedy).
        rc_a = _make_rc(100, [(1.0, 10), (1.5, 30)])
        rc_b = _make_rc(200, [(2.0, 10), (20.0, 20)])
        result = _resolve_conflicts([rc_a, rc_b])
        # Road 100 advances to lanelet 30 (idx 1), road 200 keeps lanelet 10 (idx 0)
        assert result == {0: 1, 1: 0}

    def test_save_the_drowning(self) -> None:
        """Loser exhausted, winner has alt — advance winner to save loser."""
        rc_a = _make_rc(100, [(1.0, 10), (5.0, 30)])
        rc_b = _make_rc(200, [(2.0, 10)])  # only one candidate
        result = _resolve_conflicts([rc_a, rc_b])
        # Winner (100) advances to 30, loser (200) keeps 10
        assert result == {0: 1, 1: 0}

    def test_both_exhausted_loser_dropped(self) -> None:
        """Both have only one candidate for the same lanelet — loser is dropped."""
        rc_a = _make_rc(100, [(1.0, 10)])
        rc_b = _make_rc(200, [(2.0, 10)])
        result = _resolve_conflicts([rc_a, rc_b])
        # Winner (100) keeps 10, loser (200) is dropped
        assert result == {0: 0}
        assert 1 not in result

    def test_multi_way_conflict(self) -> None:
        """Three roads claim the same lanelet — all losers advance."""
        rc_a = _make_rc(100, [(1.0, 10), (5.0, 30)])
        rc_b = _make_rc(200, [(2.0, 10), (6.0, 40)])
        rc_c = _make_rc(300, [(3.0, 10), (7.0, 50)])
        result = _resolve_conflicts([rc_a, rc_b, rc_c])
        # Winner (100) keeps lanelet 10, losers advance
        assert result[0] == 0  # road 100 -> lanelet 10
        assert result[1] == 1  # road 200 -> lanelet 40
        assert result[2] == 1  # road 300 -> lanelet 50

    def test_chain_conflict(self) -> None:
        """Advancing a loser creates a new conflict resolved in next iteration."""
        # Road 100 and 200 both claim lanelet 10.
        # Road 200 loses, advances to lanelet 20.
        # Road 300 also claims lanelet 20 — new conflict.
        rc_a = _make_rc(100, [(1.0, 10)])
        rc_b = _make_rc(200, [(2.0, 10), (3.0, 20)])
        rc_c = _make_rc(300, [(2.5, 20), (4.0, 30)])
        result = _resolve_conflicts([rc_a, rc_b, rc_c])
        # Road 100 -> lanelet 10 (idx 0)
        assert result[0] == 0
        # Road 200 advanced to lanelet 20 (idx 1), then conflicts with 300.
        # 300 is closer to 20 (2.5 < 3.0), so 200 loses again -> advances to next
        # But road 200 has no 3rd candidate, so check cost or exhaustion.
        # 200's next after idx 1 = idx 2 = out of range -> exhausted.
        # 300's next after idx 0 = idx 1 -> has alt.
        # Save the drowning: advance 300 to idx 1 (lanelet 30).
        assert result[1] == 1  # road 200 -> lanelet 20
        assert result[2] == 1  # road 300 -> lanelet 30

    def test_winner_exhausted_loser_has_alt(self) -> None:
        """Winner exhausted but loser has alternatives — advance loser."""
        rc_a = _make_rc(100, [(1.0, 10)])  # winner, no alternatives
        rc_b = _make_rc(200, [(2.0, 10), (3.0, 20)])  # loser, has alt
        result = _resolve_conflicts([rc_a, rc_b])
        # Winner (100) has no alt, loser (200) has alt -> advance loser
        assert result == {0: 0, 1: 1}
