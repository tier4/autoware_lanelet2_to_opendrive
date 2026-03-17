"""Tests for boundary-based RoadLaneletMapping.

Uses real nishishinjuku map data.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import pytest

from autoware_carla_scenario.coordinate import (
    Lanelet2Pose,
    MapManager,
    OpenDrivePose,
    to_opendrive,
)
from autoware_carla_scenario.coordinate.transform import (
    _carla_to_opendrive,
    _lanelet2_to_carla,
    _lanelet2_to_opendrive_direct,
)
from autoware_lanelet2_to_opendrive.road_lanelet_geo_mapping import (
    GeoRoadLaneletMapping,
    _sha256_of_file,
    build_mapping,
    load_or_build_mapping,
    parse_roads_from_xodr,
)

DATA_DIR = Path(__file__).parent / "data"
XODR_PATH = DATA_DIR / "nishishinjuku.xodr"
OSM_PATH = DATA_DIR / "nishishinjuku.osm"


@pytest.fixture(scope="module")
def map_manager():
    """Load nishishinjuku map once for the entire test module."""
    MapManager.reset()
    mm = MapManager.get_instance()
    mm.initialize(XODR_PATH, OSM_PATH)
    yield mm
    mm._lanelet_map = None
    mm._road_network = None
    mm._geo_origin = None
    mm._mgrs_offset = None
    mm._road_lanelet_mapping = None
    MapManager.reset()


@pytest.fixture(scope="module")
def parsed_roads():
    """Parse XODR to converter-compatible Road objects."""
    return parse_roads_from_xodr(XODR_PATH)


# ---------------------------------------------------------------------------
# Serialization round-trip
# ---------------------------------------------------------------------------


class TestSerialization:
    def test_round_trip(self):
        mapping = GeoRoadLaneletMapping(
            xodr_sha256="abc",
            osm_sha256="def",
            lanelet_to_road_and_lane={10: (1, -1), 20: (1, -2)},
        )
        restored = GeoRoadLaneletMapping.from_dict(mapping.to_dict())
        assert restored.lanelet_to_road_and_lane == mapping.lanelet_to_road_and_lane

    def test_version_field(self):
        m = GeoRoadLaneletMapping(xodr_sha256="a", osm_sha256="b")
        assert m.to_dict()["version"] == 3

    def test_empty_mapping(self):
        m = GeoRoadLaneletMapping(xodr_sha256="a", osm_sha256="b")
        assert (
            GeoRoadLaneletMapping.from_dict(m.to_dict()).lanelet_to_road_and_lane == {}
        )


# ---------------------------------------------------------------------------
# SHA256 cache
# ---------------------------------------------------------------------------


class TestSHA256Cache:
    def test_sha256_of_file(self):
        digest = _sha256_of_file(XODR_PATH)
        assert len(digest) == 64

    def test_cache_creates_file(self, map_manager, parsed_roads, tmp_path):
        import shutil

        xodr_copy = tmp_path / "test.xodr"
        osm_copy = tmp_path / "test.osm"
        shutil.copy(XODR_PATH, xodr_copy)
        shutil.copy(OSM_PATH, osm_copy)

        mapping = load_or_build_mapping(
            xodr_copy,
            osm_copy,
            map_manager.lanelet_map,
            parsed_roads,
            map_manager.mgrs_offset,
        )
        cache_file = tmp_path / "test.mapping.json"
        assert cache_file.exists()
        assert len(mapping.lanelet_to_road_and_lane) > 0

    def test_cache_hit(self, map_manager, parsed_roads, tmp_path):
        import shutil

        xodr_copy = tmp_path / "test.xodr"
        osm_copy = tmp_path / "test.osm"
        shutil.copy(XODR_PATH, xodr_copy)
        shutil.copy(OSM_PATH, osm_copy)

        m1 = load_or_build_mapping(
            xodr_copy,
            osm_copy,
            map_manager.lanelet_map,
            parsed_roads,
            map_manager.mgrs_offset,
        )
        m2 = load_or_build_mapping(
            xodr_copy,
            osm_copy,
            map_manager.lanelet_map,
            parsed_roads,
            map_manager.mgrs_offset,
        )
        assert m1.lanelet_to_road_and_lane == m2.lanelet_to_road_and_lane

    def test_cache_miss_on_changed_sha(self, map_manager, parsed_roads, tmp_path):
        import shutil

        xodr_copy = tmp_path / "test.xodr"
        osm_copy = tmp_path / "test.osm"
        shutil.copy(XODR_PATH, xodr_copy)
        shutil.copy(OSM_PATH, osm_copy)

        m1 = load_or_build_mapping(
            xodr_copy,
            osm_copy,
            map_manager.lanelet_map,
            parsed_roads,
            map_manager.mgrs_offset,
        )
        # Corrupt the cache
        cache_file = tmp_path / "test.mapping.json"
        data = json.loads(cache_file.read_text())
        data["xodr_sha256"] = "wrong"
        cache_file.write_text(json.dumps(data))

        m2 = load_or_build_mapping(
            xodr_copy,
            osm_copy,
            map_manager.lanelet_map,
            parsed_roads,
            map_manager.mgrs_offset,
        )
        assert m2.xodr_sha256 == m1.xodr_sha256


# ---------------------------------------------------------------------------
# Build mapping
# ---------------------------------------------------------------------------


class TestBuildMapping:
    def test_returns_valid_mapping(self, map_manager, parsed_roads):
        mapping = build_mapping(
            map_manager.lanelet_map,
            parsed_roads,
            map_manager.mgrs_offset,
            xodr_sha256="t",
            osm_sha256="t",
        )
        assert len(mapping.lanelet_to_road_and_lane) > 0

    def test_road_ids_exist_in_parsed_roads(self, map_manager, parsed_roads):
        mapping = build_mapping(
            map_manager.lanelet_map,
            parsed_roads,
            map_manager.mgrs_offset,
            xodr_sha256="t",
            osm_sha256="t",
        )
        road_ids = {road.id for road in parsed_roads}
        for _lid, (rid, _lane) in mapping.lanelet_to_road_and_lane.items():
            assert rid in road_ids

    def test_lane_ids_are_nonzero(self, map_manager, parsed_roads):
        mapping = build_mapping(
            map_manager.lanelet_map,
            parsed_roads,
            map_manager.mgrs_offset,
            xodr_sha256="t",
            osm_sha256="t",
        )
        nonzero = sum(
            1 for _, (_, lid) in mapping.lanelet_to_road_and_lane.items() if lid != 0
        )
        assert nonzero > 0


# ---------------------------------------------------------------------------
# Direct conversion
# ---------------------------------------------------------------------------


class TestDirectConversion:
    def test_returns_opendrive_pose(self, map_manager):
        lid = next(iter(map_manager.lanelet_map.laneletLayer)).id
        result = to_opendrive(Lanelet2Pose(lanelet_id=lid, s=0.0))
        assert isinstance(result, OpenDrivePose)

    def test_road_id_from_mapping(self, map_manager):
        mapping = map_manager.road_lanelet_mapping
        assert mapping is not None
        lid = next(iter(mapping.lanelet_to_road_and_lane))
        expected_rid, _ = mapping.lanelet_to_road_and_lane[lid]
        result = to_opendrive(Lanelet2Pose(lanelet_id=lid, s=0.0))
        assert result.road_id == str(expected_rid)

    def test_direct_vs_indirect_close(self, map_manager):
        lid = next(iter(map_manager.lanelet_map.laneletLayer)).id
        pose = Lanelet2Pose(lanelet_id=lid, s=0.0)
        direct = _lanelet2_to_opendrive_direct(pose)
        assert direct is not None
        indirect = _carla_to_opendrive(_lanelet2_to_carla(pose))
        assert abs(direct.s - indirect.s) < 10.0
        assert abs(direct.t - indirect.t) < 10.0

    def test_finite_values(self, map_manager):
        lid = next(iter(map_manager.lanelet_map.laneletLayer)).id
        r = _lanelet2_to_opendrive_direct(Lanelet2Pose(lanelet_id=lid, s=5.0))
        assert r is not None
        assert math.isfinite(r.s) and math.isfinite(r.t) and math.isfinite(r.heading)


# ---------------------------------------------------------------------------
# Fallback
# ---------------------------------------------------------------------------


class TestFallback:
    def test_fallback_when_mapping_none(self, map_manager):
        orig = map_manager._road_lanelet_mapping
        try:
            map_manager._road_lanelet_mapping = None
            lid = next(iter(map_manager.lanelet_map.laneletLayer)).id
            result = to_opendrive(Lanelet2Pose(lanelet_id=lid, s=0.0))
            assert isinstance(result, OpenDrivePose)
        finally:
            map_manager._road_lanelet_mapping = orig

    def test_direct_returns_none_when_mapping_none(self, map_manager):
        orig = map_manager._road_lanelet_mapping
        try:
            map_manager._road_lanelet_mapping = None
            lid = next(iter(map_manager.lanelet_map.laneletLayer)).id
            assert (
                _lanelet2_to_opendrive_direct(Lanelet2Pose(lanelet_id=lid, s=0.0))
                is None
            )
        finally:
            map_manager._road_lanelet_mapping = orig

    def test_direct_returns_none_for_unknown_lanelet(self, map_manager):
        assert (
            _lanelet2_to_opendrive_direct(Lanelet2Pose(lanelet_id=999999, s=0.0))
            is None
        )
