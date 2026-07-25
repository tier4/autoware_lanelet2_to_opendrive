"""Tests for road_lanelet_geo_mapping: MappingMismatchError, validate, save."""

from __future__ import annotations

import json
import math
from pathlib import Path
from unittest.mock import MagicMock

import numpy as np
import pytest

from autoware_lanelet2_to_opendrive.opendrive.geometry import (
    Arc,
    Line,
    ParamPoly3,
    PlanView,
)
from autoware_lanelet2_to_opendrive.opendrive.road import Road
from autoware_lanelet2_to_opendrive.road_lanelet_geo_mapping import (
    GeoRoadLaneletMapping,
    MappingMismatchError,
    ProjectionMetadata,
    _RoadCandidates,
    _compute_all_candidates,
    build_mapping,
    _resolve_conflicts,
    _sample_reference_line_from_road,
    _walk_lane_group_from_seed,
    parse_roads_from_xodr,
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
        assert "version" not in data
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

    def test_round_trip_preserves_skipped_synthetic_roads(self, tmp_path: Path) -> None:
        """#493: synthetic divergence connectors are recorded in the mapping
        JSON so consumers can tell them apart from real mapping failures."""
        xodr_path = tmp_path / "test.xodr"
        xodr_path.write_text("<OpenDRIVE/>")

        original = GeoRoadLaneletMapping(
            xodr_sha256="abc",
            osm_sha256="def",
            lanelet_to_road_and_lane={10: (1, -1)},
            skipped_synthetic_roads=[274, 290, 303],
        )
        result_path = save_mapping_json(original, xodr_path)

        data = json.loads(result_path.read_text(encoding="utf-8"))
        assert data["skipped_synthetic_roads"] == [274, 290, 303]
        restored = GeoRoadLaneletMapping.from_dict(data)
        assert restored.skipped_synthetic_roads == [274, 290, 303]

    def test_round_trip_preserves_projection_metadata(self, tmp_path: Path) -> None:
        """Projection metadata preserves the converter-time local frame exactly."""
        xodr_path = tmp_path / "test.xodr"
        xodr_path.write_text("<OpenDRIVE/>")

        original = GeoRoadLaneletMapping(
            xodr_sha256="abc",
            osm_sha256="def",
            lanelet_to_road_and_lane={10: (1, 1)},
            projection_metadata=ProjectionMetadata(
                projector_type="MGRSProjector",
                mgrs_code="54SUE",
                origin_lat=35.64594559004192,
                origin_lon=139.80712515470023,
                offset_x=92008.5,
                offset_y=45335.1,
                offset_z=7.25,
            ),
        )
        result_path = save_mapping_json(original, xodr_path)

        data = json.loads(result_path.read_text(encoding="utf-8"))
        assert data["projection_metadata"]["offset_x"] == 92008.5
        assert data["projection_metadata"]["offset_y"] == 45335.1
        restored = GeoRoadLaneletMapping.from_dict(data)
        assert restored.projection_metadata is not None
        assert restored.projection_metadata.offset_x == 92008.5
        assert restored.projection_metadata.offset_y == 45335.1
        assert restored.projection_metadata.offset_z == 7.25

    def test_from_dict_accepts_mapping_without_projection_metadata(self) -> None:
        """Old sidecars without projection metadata remain readable."""
        mapping = GeoRoadLaneletMapping.from_dict(
            {
                "xodr_sha256": "abc",
                "osm_sha256": "def",
                "lanelet_to_road_and_lane": {"10": [1, 1]},
            }
        )

        assert mapping.projection_metadata is None


# ---------------------------------------------------------------------------
# _compute_all_candidates — synthetic divergence connector handling (#493)
# ---------------------------------------------------------------------------


class TestComputeAllCandidatesSyntheticSkip:
    """#493: synthetic divergence/merge connecting roads (junction != -1,
    sub-0.5m total geometry) must be excluded from geometric matching *and*
    reported separately, not counted as 0-candidate matching failures."""

    def test_synthetic_connector_is_skipped_and_recorded(self) -> None:
        import lxml.etree as ET

        from autoware_lanelet2_to_opendrive.road_lanelet_geo_mapping import (
            _compute_all_candidates,
        )

        # A degenerate divergence connecting road: junction != -1, total
        # planView length far below the synthetic-connector threshold.
        xodr = (
            "<OpenDRIVE><road id='100' junction='11000'><planView>"
            "<geometry s='0.0' x='0.0' y='0.0' hdg='0.0' length='0.05'>"
            "<line/></geometry></planView></road></OpenDRIVE>"
        )
        roads = parse_roads_from_xodr(
            Path("unused.xodr"), xodr_root=ET.fromstring(xodr)
        )
        all_rc, no_candidate_diag, skipped = _compute_all_candidates(
            roads, {}, {}, {}, {}
        )

        assert skipped == {100}
        assert 100 not in {rc.road_id for rc in all_rc}
        assert 100 not in no_candidate_diag

    def test_helper_identifies_synthetic_connectors_only(self) -> None:
        """_synthetic_connector_road_ids selects junction roads with sub-0.5m
        geometry, leaving real connecting roads and regular roads alone."""
        import lxml.etree as ET

        from autoware_lanelet2_to_opendrive.road_lanelet_geo_mapping import (
            _synthetic_connector_road_ids,
        )

        xodr = (
            "<OpenDRIVE>"
            # synthetic connector: junction != -1, sub-0.5m total geometry
            "<road id='100' junction='11000'><planView>"
            "<geometry s='0.0' x='0.0' y='0.0' hdg='0.0' length='0.05'>"
            "<line/></geometry></planView></road>"
            # real connecting road: junction != -1 but several metres long
            "<road id='200' junction='1000'><planView>"
            "<geometry s='0.0' x='0.0' y='0.0' hdg='0.0' length='12.0'>"
            "<line/></geometry></planView></road>"
            # regular road: junction == -1 (short, but not a connector)
            "<road id='300' junction='-1'><planView>"
            "<geometry s='0.0' x='0.0' y='0.0' hdg='0.0' length='0.05'>"
            "<line/></geometry></planView></road>"
            "</OpenDRIVE>"
        )
        roads = parse_roads_from_xodr(
            Path("unused.xodr"), xodr_root=ET.fromstring(xodr)
        )
        assert _synthetic_connector_road_ids(roads) == {100}


class TestComputeAllCandidatesDirectedFallback:
    """Directed fallback should not prefer an opposite-direction overlap.

    Regression coverage for the Odaiba phase1_tight reciprocal road swap:
    when symmetric matching rejects short partial overlaps, the final directed
    fallback used to ignore direction and could choose a nearby/opposite road
    group before the same-direction source boundary.
    """

    def test_directed_fallback_prefers_same_direction_candidate(self) -> None:
        import lxml.etree as ET

        from autoware_lanelet2_to_opendrive.road_lanelet_geo_mapping import (
            _compute_all_candidates,
        )

        xodr = (
            "<OpenDRIVE><road id='100' junction='-1'><planView>"
            "<geometry s='0.0' x='0.0' y='0.0' hdg='0.0' length='20.0'>"
            "<line/></geometry></planView><lanes><laneSection s='0.0'>"
            "<left><lane id='1' type='driving' level='false'/></left>"
            "<center><lane id='0' type='none' level='false'/></center>"
            "</laneSection></lanes></road></OpenDRIVE>"
        )
        roads = parse_roads_from_xodr(
            Path("unused.xodr"), xodr_root=ET.fromstring(xodr)
        )
        # LHT road with positive lane IDs searches right boundaries. Both
        # candidates partially overlap the long reference line, so symmetric
        # distance rejects them; only directed fallback can recover a match.
        same_direction_lid = 200
        opposite_direction_lid = 100
        lanelet_right = {
            opposite_direction_lid: np.array([[11.0, 0.0], [9.0, 0.0]]),
            same_direction_lid: np.array([[9.0, 0.0], [11.0, 0.0]]),
        }
        lanelet_right_bbox = {
            lid: (float(pts[:, 0].min()), 0.0, float(pts[:, 0].max()), 0.0)
            for lid, pts in lanelet_right.items()
        }

        all_rc, no_candidate_diag, skipped = _compute_all_candidates(
            roads,
            lanelet_left={},
            lanelet_right=lanelet_right,
            lanelet_left_bbox={},
            lanelet_right_bbox=lanelet_right_bbox,
        )

        assert skipped == set()
        assert no_candidate_diag == {}
        assert len(all_rc) == 1
        assert all_rc[0].candidates[0][1] == same_direction_lid

    def test_directed_fallback_still_allows_opposite_direction_if_needed(
        self,
    ) -> None:
        import lxml.etree as ET

        from autoware_lanelet2_to_opendrive.road_lanelet_geo_mapping import (
            _compute_all_candidates,
        )

        xodr = (
            "<OpenDRIVE><road id='100' junction='-1'><planView>"
            "<geometry s='0.0' x='0.0' y='0.0' hdg='0.0' length='20.0'>"
            "<line/></geometry></planView><lanes><laneSection s='0.0'>"
            "<left><lane id='1' type='driving' level='false'/></left>"
            "<center><lane id='0' type='none' level='false'/></center>"
            "</laneSection></lanes></road></OpenDRIVE>"
        )
        roads = parse_roads_from_xodr(
            Path("unused.xodr"), xodr_root=ET.fromstring(xodr)
        )
        lanelet_right = {100: np.array([[11.0, 0.0], [9.0, 0.0]])}
        lanelet_right_bbox = {100: (9.0, 0.0, 11.0, 0.0)}

        all_rc, no_candidate_diag, skipped = _compute_all_candidates(
            roads,
            lanelet_left={},
            lanelet_right=lanelet_right,
            lanelet_left_bbox={},
            lanelet_right_bbox=lanelet_right_bbox,
        )

        assert skipped == set()
        assert no_candidate_diag == {}
        assert len(all_rc) == 1
        assert all_rc[0].candidates[0][1] == 100


class TestComputeAllCandidatesFallbackPolicy:
    """Direction-consistent candidates must not be hidden by fallback order."""

    @staticmethod
    def _lht_line_road(length: float = 20.0) -> list[Road]:
        import lxml.etree as ET

        xodr = (
            "<OpenDRIVE><road id='100' junction='-1'><planView>"
            f"<geometry s='0.0' x='0.0' y='0.0' hdg='0.0' length='{length}'>"
            "<line/></geometry></planView><lanes><laneSection s='0.0'>"
            "<left><lane id='1' type='driving' level='false'/></left>"
            "<center><lane id='0' type='none' level='false'/></center>"
            "</laneSection></lanes></road></OpenDRIVE>"
        )
        return parse_roads_from_xodr(Path("unused.xodr"), xodr_root=ET.fromstring(xodr))

    @staticmethod
    def _bbox(points: np.ndarray) -> tuple[float, float, float, float]:
        return (
            float(points[:, 0].min()),
            float(points[:, 1].min()),
            float(points[:, 0].max()),
            float(points[:, 1].max()),
        )

    def test_preserves_direction_consistent_directed_candidate(self) -> None:
        true_lid = 100
        wrong_lid = 200
        dense_reversed = np.column_stack(
            (np.linspace(20.0, 0.0, 41), np.full(41, 0.25))
        )
        lanelet_right = {
            # Sparse but endpoint-aligned: directed-close, symmetric-far.
            true_lid: np.array([[0.0, 0.0], [20.0, 0.0]]),
            # Dense and nearby, but opposite direction.
            wrong_lid: dense_reversed,
        }

        all_rc, no_candidate_diag, skipped = _compute_all_candidates(
            self._lht_line_road(),
            lanelet_left={},
            lanelet_right=lanelet_right,
            lanelet_left_bbox={},
            lanelet_right_bbox={
                lid: self._bbox(points) for lid, points in lanelet_right.items()
            },
        )

        assert skipped == set()
        assert no_candidate_diag == {}
        assert len(all_rc) == 1
        assert all_rc[0].candidates[0][1] == true_lid

    def test_same_direction_partial_branch_does_not_beat_full_shape(self) -> None:
        true_lid = 100
        wrong_lid = 200
        lanelet_right = {
            true_lid: np.column_stack((np.linspace(0.0, 20.0, 41), np.full(41, 1.0))),
            # Same direction and directed-close, but only touches the road start.
            wrong_lid: np.column_stack((np.linspace(0.0, 2.0, 9), np.zeros(9))),
        }

        all_rc, no_candidate_diag, skipped = _compute_all_candidates(
            self._lht_line_road(),
            lanelet_left={},
            lanelet_right=lanelet_right,
            lanelet_left_bbox={},
            lanelet_right_bbox={
                lid: self._bbox(points) for lid, points in lanelet_right.items()
            },
        )

        assert skipped == set()
        assert no_candidate_diag == {}
        assert len(all_rc) == 1
        assert all_rc[0].candidates[0][1] == true_lid

    def test_directionless_symmetric_fallback_is_preserved(self) -> None:
        reversed_lid = 100
        lanelet_right = {
            reversed_lid: np.column_stack(
                (np.linspace(20.0, 0.0, 41), np.full(41, 0.25))
            )
        }

        all_rc, no_candidate_diag, skipped = _compute_all_candidates(
            self._lht_line_road(),
            lanelet_left={},
            lanelet_right=lanelet_right,
            lanelet_left_bbox={},
            lanelet_right_bbox={
                lid: self._bbox(points) for lid, points in lanelet_right.items()
            },
        )

        assert skipped == set()
        assert no_candidate_diag == {}
        assert len(all_rc) == 1
        assert all_rc[0].candidates[0][1] == reversed_lid


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


def _make_lht_walk_rc(road_id: int, lane_ids: list[int]) -> _RoadCandidates:
    """Helper to build a minimal LHT _RoadCandidates for lateral walk tests."""
    return _RoadCandidates(
        road=MagicMock(),
        road_id=road_id,
        lane_ids=lane_ids,
        is_rht=False,
        ref_line=MagicMock(),
        candidates=[],
        raw_dists={},
        walk_lane_ids=list(lane_ids),
    )


def _make_rht_walk_rc(road_id: int, lane_ids: list[int]) -> _RoadCandidates:
    """Helper to build a minimal RHT _RoadCandidates for lateral walk tests."""
    return _RoadCandidates(
        road=MagicMock(),
        road_id=road_id,
        lane_ids=lane_ids,
        is_rht=True,
        ref_line=MagicMock(),
        candidates=[],
        raw_dists={},
        walk_lane_ids=list(lane_ids),
    )


class _MockPoint:
    def __init__(self, x: float, y: float) -> None:
        self.x = x
        self.y = y


class _MockLineString(list):
    def __init__(self, line_id: int, y: float) -> None:
        super().__init__([_MockPoint(x * 0.5, y) for x in range(21)])
        self.id = line_id


class _MockLanelet:
    def __init__(
        self,
        lanelet_id: int,
        left_bound: _MockLineString,
        right_bound: _MockLineString,
    ) -> None:
        self.id = lanelet_id
        self.leftBound = left_bound
        self.rightBound = right_bound


class _MockLaneletLayer:
    def __init__(self, lanelets: list[_MockLanelet]) -> None:
        self._lanelets = {lanelet.id: lanelet for lanelet in lanelets}

    def __iter__(self):
        return iter(self._lanelets.values())

    def __getitem__(self, lanelet_id: int) -> _MockLanelet:
        return self._lanelets[lanelet_id]


class _MockLaneletMap:
    def __init__(self, lanelets: list[_MockLanelet]) -> None:
        self.laneletLayer = _MockLaneletLayer(lanelets)


def _parse_lht_line_roads(roads: list[tuple[int, float, int]]) -> list[Road]:
    """Build LHT roads whose positive lane IDs walk left from the reference."""
    road_xml = []
    for road_id, y, lane_count in roads:
        lane_xml = "".join(
            f"<lane id='{lane_id}' type='driving' level='false'/>"
            for lane_id in range(1, lane_count + 1)
        )
        road_xml.append(
            f"<road id='{road_id}' junction='-1'>"
            f"<planView><geometry s='0.0' x='0.0' y='{y}' hdg='0.0' "
            "length='10.0'><line/></geometry></planView>"
            f"<lanes><laneSection s='0.0'><left>{lane_xml}</left>"
            "<center><lane id='0' type='none' level='false'/></center>"
            "</laneSection></lanes></road>"
        )
    import lxml.etree as ET

    return parse_roads_from_xodr(
        Path("unused.xodr"),
        xodr_root=ET.fromstring("<OpenDRIVE>" + "".join(road_xml) + "</OpenDRIVE>"),
    )


def _mapping_by_road(mapping: GeoRoadLaneletMapping) -> dict[int, dict[int, int]]:
    result: dict[int, dict[int, int]] = {}
    for lanelet_id, (road_id, lane_id) in mapping.lanelet_to_road_and_lane.items():
        result.setdefault(road_id, {})[lane_id] = lanelet_id
    return result


class TestPhase2OwnershipCompletion:
    """Phase 2 seed ownership must not split a complete lateral lane group."""

    def test_partial_false_seed_does_not_block_full_lateral_group(self) -> None:
        right = _MockLineString(1, 0.0)
        shared = _MockLineString(2, 3.0)
        left = _MockLineString(3, 6.0)
        lanelet_map = _MockLaneletMap(
            [
                _MockLanelet(101, shared, right),
                _MockLanelet(102, left, shared),
            ]
        )
        roads = _parse_lht_line_roads(
            [
                (10, -2.5, 2),  # False road: locally sees lanelet 101 only.
                (20, 0.0, 2),  # True road: 101 -> 102 forms a full group.
            ]
        )

        mapping = build_mapping(lanelet_map, roads, (0.0, 0.0), "x", "o")

        assert _mapping_by_road(mapping)[20] == {1: 101, 2: 102}
        assert 101 not in _mapping_by_road(mapping).get(10, {}).values()

    def test_partial_false_seed_resolution_is_candidate_order_independent(
        self,
    ) -> None:
        right = _MockLineString(1, 0.0)
        shared = _MockLineString(2, 3.0)
        left = _MockLineString(3, 6.0)
        lanelet_map = _MockLaneletMap(
            [
                _MockLanelet(101, shared, right),
                _MockLanelet(102, left, shared),
            ]
        )

        forward = build_mapping(
            lanelet_map,
            _parse_lht_line_roads([(10, -2.5, 2), (20, 0.0, 2)]),
            (0.0, 0.0),
            "x",
            "o",
        )
        reverse = build_mapping(
            lanelet_map,
            _parse_lht_line_roads([(20, 0.0, 2), (10, -2.5, 2)]),
            (0.0, 0.0),
            "x",
            "o",
        )

        assert forward.lanelet_to_road_and_lane == reverse.lanelet_to_road_and_lane

    def test_adjacent_road_on_reference_edge_stays_separate(self) -> None:
        edge = _MockLineString(1, 0.0)
        road_boundary = _MockLineString(2, 3.0)
        shared = _MockLineString(3, 6.0)
        left = _MockLineString(4, 9.0)
        lanelet_map = _MockLaneletMap(
            [
                _MockLanelet(101, road_boundary, edge),
                _MockLanelet(201, shared, road_boundary),
                _MockLanelet(202, left, shared),
            ]
        )
        roads = _parse_lht_line_roads(
            [
                (10, 0.0, 1),
                (20, 3.0, 2),
            ]
        )

        mapping = build_mapping(lanelet_map, roads, (0.0, 0.0), "x", "o")

        by_road = _mapping_by_road(mapping)
        assert by_road[10] == {1: 101}
        assert by_road[20] == {1: 201, 2: 202}


class TestWalkLaneGroupFromSeed:
    """Lateral lane reconstruction must not assume the seed is the edge lane."""

    def test_reconstructs_lht_three_lane_group_from_any_seed(self) -> None:
        rc = _make_lht_walk_rc(100, [1, 2, 3])
        left_neighbors = {10: 20, 20: 30}
        right_neighbors = {30: 20, 20: 10}

        def left_neighbor(lanelet_id: int) -> int | None:
            return left_neighbors.get(lanelet_id)

        def right_neighbor(lanelet_id: int) -> int | None:
            return right_neighbors.get(lanelet_id)

        expected = [(10, 100, 1), (20, 100, 2), (30, 100, 3)]
        for seed in (10, 20, 30):
            assert (
                _walk_lane_group_from_seed(
                    rc,
                    seed,
                    right_neighbor,
                    left_neighbor,
                    blocked_lanelets=set(),
                )
                == expected
            )

    def test_reconstructs_rht_three_lane_group_from_any_seed(self) -> None:
        rc = _make_rht_walk_rc(100, [-1, -2, -3])
        left_neighbors = {30: 20, 20: 10}
        right_neighbors = {10: 20, 20: 30}

        def left_neighbor(lanelet_id: int) -> int | None:
            return left_neighbors.get(lanelet_id)

        def right_neighbor(lanelet_id: int) -> int | None:
            return right_neighbors.get(lanelet_id)

        expected = [(10, 100, -1), (20, 100, -2), (30, 100, -3)]
        for seed in (10, 20, 30):
            assert (
                _walk_lane_group_from_seed(
                    rc,
                    seed,
                    right_neighbor,
                    left_neighbor,
                    blocked_lanelets=set(),
                )
                == expected
            )

    def test_rescue_seed_on_inner_lane_preserves_lane_order(self) -> None:
        rc = _make_lht_walk_rc(31, [1, 2])

        assert _walk_lane_group_from_seed(
            rc,
            20,
            right_neighbor=lambda lanelet_id: 10 if lanelet_id == 20 else None,
            left_neighbor=lambda lanelet_id: None,
            blocked_lanelets=set(),
        ) == [(10, 31, 1), (20, 31, 2)]

    def test_blocked_adjacent_lane_is_not_crossed(self) -> None:
        rc = _make_lht_walk_rc(31, [1, 2])

        assert _walk_lane_group_from_seed(
            rc,
            20,
            right_neighbor=lambda lanelet_id: 10 if lanelet_id == 20 else None,
            left_neighbor=lambda lanelet_id: 99 if lanelet_id == 20 else None,
            blocked_lanelets={99},
        ) == [(10, 31, 1), (20, 31, 2)]


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


# ---------------------------------------------------------------------------
# _sample_reference_line_from_road  (issue #495)
# ---------------------------------------------------------------------------


class TestSampleReferenceLineFromRoad:
    """A road's planView must be sampled with each geometry's analytic model.

    Regression test for #495: with arc-primitive detection enabled, an
    ``<arc>`` segment must be sampled along its curve, not as the straight
    chord between its endpoints — chord sampling inflated road↔lanelet
    distances and aborted conversion with a ``MappingMismatchError``.
    """

    def test_arc_geometry_is_sampled_along_its_curve(self) -> None:
        # Quarter circle of radius 10 m: starts at the origin heading +x,
        # curves left (positive curvature), ends at (10, 10) heading +y.
        radius = 10.0
        curvature = 1.0 / radius
        length = radius * math.pi / 2.0
        road = Road(
            id=1,
            plan_view=PlanView(
                geometries=[
                    Arc(
                        s=0.0,
                        x=0.0,
                        y=0.0,
                        hdg=0.0,
                        length=length,
                        curvature=curvature,
                    )
                ]
            ),
        )

        # spacing 2.0 m over the 15.708 m quarter circle -> 8 intervals.
        pts = _sample_reference_line_from_road(road, sample_spacing=2.0)

        # Every sample must lie on the analytic arc, not the straight chord
        # (chord sampling keeps y == 0, which is the bug being fixed).
        for i in range(8):
            p = length * i / 8
            expected = (
                math.sin(curvature * p) / curvature,
                (1.0 - math.cos(curvature * p)) / curvature,
            )
            assert pts[i] == pytest.approx(expected, abs=1e-6)
        # Closing endpoint: the quarter circle ends exactly at (radius, radius).
        assert pts[-1] == pytest.approx((radius, radius), abs=1e-6)

    def test_line_geometry_is_sampled_as_a_straight_line(self) -> None:
        # A 20 m line heading 30 degrees, starting at (1, 2).
        hdg = math.radians(30.0)
        length = 20.0
        road = Road(
            id=2,
            plan_view=PlanView(
                geometries=[Line(s=0.0, x=1.0, y=2.0, hdg=hdg, length=length)]
            ),
        )

        # spacing 5.0 m over the 20 m line -> 4 intervals.
        pts = _sample_reference_line_from_road(road, sample_spacing=5.0)

        for i in range(4):
            p = length * i / 4
            expected = (1.0 + p * math.cos(hdg), 2.0 + p * math.sin(hdg))
            assert pts[i] == pytest.approx(expected, abs=1e-6)
        assert pts[-1] == pytest.approx(
            (1.0 + length * math.cos(hdg), 2.0 + length * math.sin(hdg)),
            abs=1e-6,
        )

    def test_param_poly3_geometry_is_sampled_with_cubic_model(self) -> None:
        # A paramPoly3 heading +x from the origin with a quadratic lateral
        # offset v = 0.1 * u**2.
        length = 10.0
        road = Road(
            id=3,
            plan_view=PlanView(
                geometries=[
                    ParamPoly3(
                        s=0.0,
                        x=0.0,
                        y=0.0,
                        hdg=0.0,
                        length=length,
                        aU=0.0,
                        bU=1.0,
                        cU=0.0,
                        dU=0.0,
                        aV=0.0,
                        bV=0.0,
                        cV=0.1,
                        dV=0.0,
                    )
                ]
            ),
        )

        # spacing 2.0 m over the 10 m paramPoly3 -> 5 intervals.
        pts = _sample_reference_line_from_road(road, sample_spacing=2.0)

        for i in range(5):
            p = length * i / 5
            assert pts[i] == pytest.approx((p, 0.1 * p * p), abs=1e-6)
        assert pts[-1] == pytest.approx((length, 0.1 * length * length), abs=1e-6)

    def test_sampling_is_uniform_across_uneven_segments(self) -> None:
        """Samples are spaced uniformly by arc-length regardless of how the
        planView is split into segments (#499).

        Arc-primitive detection re-segments a road into a few long arcs plus
        short paramPoly3 runs. A fixed sample count per segment would bunch
        points onto the short segments and skew the mean-distance metric used
        for road-lanelet matching, flipping which lanelet a road maps to.
        Two collinear straight segments of very different lengths must still
        yield evenly-spaced samples.
        """
        # A 4 m line followed by a collinear 16 m line (total 20 m, heading +x).
        road = Road(
            id=1,
            plan_view=PlanView(
                geometries=[
                    Line(s=0.0, x=0.0, y=0.0, hdg=0.0, length=4.0),
                    Line(s=4.0, x=4.0, y=0.0, hdg=0.0, length=16.0),
                ]
            ),
        )

        pts = _sample_reference_line_from_road(road, sample_spacing=1.0)

        # 1.0 m spacing over 20 m -> 20 intervals, 21 points, uniform.
        assert len(pts) == 21
        spacings = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        assert spacings.max() - spacings.min() < 1e-9
        assert spacings[0] == pytest.approx(1.0)
        assert pts[0] == pytest.approx((0.0, 0.0), abs=1e-9)
        assert pts[-1] == pytest.approx((20.0, 0.0), abs=1e-9)


# ---------------------------------------------------------------------------
# parse_roads_from_xodr  (issue #502)
# ---------------------------------------------------------------------------


class TestParseRoadsFromXodr:
    """parse_roads_from_xodr must reconstruct every planView primitive.

    Regression test for #502: the XODR re-parser used by the analyze/QC
    path previously handled only ``<paramPoly3>`` and silently dropped
    ``<arc>`` and ``<line>`` geometry, breaking validation of maps
    converted with arc-primitive detection enabled.
    """

    def test_parses_line_arc_and_param_poly3(self) -> None:
        import lxml.etree as ET

        xodr = (
            "<OpenDRIVE><road id='7' junction='-1'><planView>"
            "<geometry s='0.0' x='1.0' y='2.0' hdg='0.5' length='10.0'>"
            "<line/></geometry>"
            "<geometry s='10.0' x='3.0' y='4.0' hdg='0.6' length='20.0'>"
            "<arc curvature='0.04'/></geometry>"
            "<geometry s='30.0' x='5.0' y='6.0' hdg='0.7' length='8.0'>"
            "<paramPoly3 aU='0.0' bU='1.0' cU='0.0' dU='0.0'"
            " aV='0.0' bV='0.0' cV='0.1' dV='0.0'/></geometry>"
            "</planView></road></OpenDRIVE>"
        )
        roads = parse_roads_from_xodr(
            Path("unused.xodr"), xodr_root=ET.fromstring(xodr)
        )

        assert len(roads) == 1
        plan_view = roads[0].plan_view
        assert plan_view is not None
        geometries = plan_view.geometries
        assert [type(g).__name__ for g in geometries] == [
            "Line",
            "Arc",
            "ParamPoly3",
        ]

        line, arc, pp3 = geometries
        assert isinstance(line, Line)
        assert (line.s, line.x, line.y, line.hdg, line.length) == (
            0.0,
            1.0,
            2.0,
            0.5,
            10.0,
        )
        assert isinstance(arc, Arc)
        assert arc.length == pytest.approx(20.0)
        assert arc.curvature == pytest.approx(0.04)
        assert isinstance(pp3, ParamPoly3)
        assert pp3.cV == pytest.approx(0.1)
