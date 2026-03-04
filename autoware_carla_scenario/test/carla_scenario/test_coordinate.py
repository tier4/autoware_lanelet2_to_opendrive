"""Integration tests for coordinate transformation using real nishishinjuku map data.

Tests use real .xodr and .osm files from the test data directory.
No CARLA connection is required.
"""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from autoware_carla_scenario.coordinate import (
    CarlaWorldPose,
    Lanelet2Pose,
    MapManager,
    OpenDrivePose,
    to_carla_world,
    to_lanelet2,
    to_opendrive,
)
from autoware_carla_scenario.coordinate.map_manager import _parse_geo_reference
from autoware_carla_scenario.coordinate.transform import (
    _carla_to_lanelet2,
    _carla_to_opendrive,
    _lanelet2_to_carla,
    _opendrive_to_carla,
)

DATA_DIR = Path(__file__).parent / "data"
XODR_PATH = DATA_DIR / "nishishinjuku.xodr"
OSM_PATH = DATA_DIR / "nishishinjuku.osm"


# ---------------------------------------------------------------------------
# Module-level fixtures
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def map_manager():
    """Load nishishinjuku map once for the entire test module."""
    MapManager.reset()
    mm = MapManager.get_instance()
    mm.initialize(XODR_PATH, OSM_PATH)
    yield mm
    # Release lanelet2/pyxodr objects explicitly during teardown so they are
    # destroyed while the C++ runtime is still in a valid state, not during
    # Python interpreter shutdown (which can trigger std::terminate).
    mm._lanelet_map = None
    mm._road_network = None
    mm._geo_origin = None
    mm._mgrs_offset = None
    MapManager.reset()


# ---------------------------------------------------------------------------
# TestParseGeoReference – pure string parsing, no map files needed
# ---------------------------------------------------------------------------


class TestParseGeoReference:
    _PROJ = "+proj=tmerc +lat_0=35.123 +lon_0=139.456 +k=1 +x_0=0 +y_0=0"
    _XODR = (
        "<OpenDRIVE><header>"
        f"<geoReference><![CDATA[{_PROJ}]]></geoReference>"
        "</header></OpenDRIVE>"
    )

    def test_lat_lon_parsed_correctly(self):
        lat, lon, alt = _parse_geo_reference(self._XODR)
        assert lat == pytest.approx(35.123)
        assert lon == pytest.approx(139.456)
        assert alt == pytest.approx(0.0)

    def test_missing_lat_raises(self):
        bad = "<geoReference><![CDATA[+proj=tmerc +lon_0=139.0]]></geoReference>"
        with pytest.raises(ValueError, match="lat_0"):
            _parse_geo_reference(bad)

    def test_missing_geo_reference_raises(self):
        with pytest.raises(ValueError, match="geoReference"):
            _parse_geo_reference("<OpenDRIVE></OpenDRIVE>")

    def test_without_cdata_wrapper(self):
        xodr = "<geoReference>+proj=tmerc +lat_0=10.0 +lon_0=20.0</geoReference>"
        lat, lon, alt = _parse_geo_reference(xodr)
        assert lat == pytest.approx(10.0)
        assert lon == pytest.approx(20.0)


# ---------------------------------------------------------------------------
# TestMapManagerInit – verify map loaded successfully
# ---------------------------------------------------------------------------


class TestMapManagerInit:
    def test_map_loads_successfully(self, map_manager):
        assert map_manager.lanelet_map is not None
        assert map_manager.road_network is not None

    def test_geo_origin_is_nishishinjuku(self, map_manager):
        """Origin should be in the Tokyo / Nishishinjuku area."""
        lat, lon, _alt = map_manager.geo_origin
        assert lat == pytest.approx(35.688, abs=0.01)
        assert lon == pytest.approx(139.692, abs=0.01)

    def test_mgrs_offset_is_nonzero(self, map_manager):
        ox, oy = map_manager.mgrs_offset
        assert ox > 0
        assert oy > 0

    def test_lanelet_map_has_lanelets(self, map_manager):
        lanelets = list(map_manager.lanelet_map.laneletLayer)
        assert len(lanelets) > 0

    def test_road_network_has_roads(self, map_manager):
        roads = map_manager.road_network.road_ids_to_object
        assert len(roads) > 0

    def test_singleton_same_instance(self, map_manager):
        assert MapManager.get_instance() is map_manager

    def test_double_init_raises(self, map_manager):
        with pytest.raises(RuntimeError, match="already initialized"):
            map_manager.initialize(XODR_PATH, OSM_PATH)

    def test_file_not_found_raises(self, map_manager):
        # Use object.__new__ to bypass the singleton and get a raw uninitialized
        # instance without touching the shared singleton's state.
        raw_mm = object.__new__(MapManager)
        raw_mm._lanelet_map = None
        raw_mm._road_network = None
        raw_mm._geo_origin = None
        raw_mm._mgrs_offset = None
        with pytest.raises(FileNotFoundError):
            raw_mm.initialize(Path("nonexistent.xodr"), Path("nonexistent.osm"))


# ---------------------------------------------------------------------------
# TestLanelet2ToCarla
# ---------------------------------------------------------------------------


class TestLanelet2ToCarla:
    def test_result_is_carla_world_pose(self, map_manager):
        lanelet_id = next(iter(map_manager.lanelet_map.laneletLayer)).id
        result = _lanelet2_to_carla(Lanelet2Pose(lanelet_id=lanelet_id, s=0.0))
        assert isinstance(result, CarlaWorldPose)

    def test_carla_coords_are_finite(self, map_manager):
        lanelet_id = next(iter(map_manager.lanelet_map.laneletLayer)).id
        result = _lanelet2_to_carla(Lanelet2Pose(lanelet_id=lanelet_id, s=0.0))
        assert math.isfinite(result.x)
        assert math.isfinite(result.y)
        assert math.isfinite(result.z)
        assert math.isfinite(result.yaw)

    def test_carla_x_is_positive(self, map_manager):
        """MGRS easting for Tokyo is positive (zone 54S)."""
        lanelet_id = next(iter(map_manager.lanelet_map.laneletLayer)).id
        result = _lanelet2_to_carla(Lanelet2Pose(lanelet_id=lanelet_id, s=0.0))
        assert result.x > 0

    def test_lateral_offset_displaces_by_correct_distance(self, map_manager):
        """t=1 m should shift position by exactly 1 m."""
        lanelet_id = next(iter(map_manager.lanelet_map.laneletLayer)).id
        r0 = _lanelet2_to_carla(Lanelet2Pose(lanelet_id=lanelet_id, s=0.0, t=0.0))
        r1 = _lanelet2_to_carla(Lanelet2Pose(lanelet_id=lanelet_id, s=0.0, t=1.0))
        dist = math.sqrt((r0.x - r1.x) ** 2 + (r0.y - r1.y) ** 2)
        assert dist == pytest.approx(1.0, abs=0.01)


# ---------------------------------------------------------------------------
# TestOpenDriveToCarla
# ---------------------------------------------------------------------------


class TestOpenDriveToCarla:
    def test_result_is_carla_world_pose(self, map_manager):
        road_id = next(iter(map_manager.road_network.road_ids_to_object))
        result = _opendrive_to_carla(OpenDrivePose(road_id=road_id, lane_id=-1, s=0.0))
        assert isinstance(result, CarlaWorldPose)

    def test_carla_coords_are_finite(self, map_manager):
        road_id = next(iter(map_manager.road_network.road_ids_to_object))
        result = _opendrive_to_carla(OpenDrivePose(road_id=road_id, lane_id=-1, s=0.0))
        assert math.isfinite(result.x)
        assert math.isfinite(result.y)
        assert math.isfinite(result.z)

    def test_carla_x_is_positive(self, map_manager):
        """After adding MGRS offset, x should be in the Tokyo MGRS range (> 0)."""
        road_id = next(iter(map_manager.road_network.road_ids_to_object))
        result = _opendrive_to_carla(OpenDrivePose(road_id=road_id, lane_id=-1, s=0.0))
        assert result.x > 0

    def test_lateral_offset_displaces_by_correct_distance(self, map_manager):
        """t=1 m should shift position by exactly 1 m."""
        road_id = next(iter(map_manager.road_network.road_ids_to_object))
        r0 = _opendrive_to_carla(
            OpenDrivePose(road_id=road_id, lane_id=0, s=0.0, t=0.0)
        )
        r1 = _opendrive_to_carla(
            OpenDrivePose(road_id=road_id, lane_id=1, s=0.0, t=1.0)
        )
        dist = math.sqrt((r0.x - r1.x) ** 2 + (r0.y - r1.y) ** 2)
        assert dist == pytest.approx(1.0, abs=0.01)

    def test_lanelet2_and_opendrive_origins_are_close(self, map_manager):
        """Lanelet2 and OpenDRIVE share the same map area; positions should overlap."""
        lanelet_id = next(iter(map_manager.lanelet_map.laneletLayer)).id
        road_id = next(iter(map_manager.road_network.road_ids_to_object))

        ll2_carla = _lanelet2_to_carla(Lanelet2Pose(lanelet_id=lanelet_id, s=0.0))
        od_carla = _opendrive_to_carla(
            OpenDrivePose(road_id=road_id, lane_id=-1, s=0.0)
        )

        # Both should be in the Nishishinjuku area (x roughly 81000–83000 in MGRS)
        assert abs(ll2_carla.x - od_carla.x) < 3000
        assert abs(ll2_carla.y - od_carla.y) < 3000


# ---------------------------------------------------------------------------
# TestRoundTripLanelet2 – Lanelet2Pose → CarlaWorldPose → Lanelet2Pose
# ---------------------------------------------------------------------------


class TestRoundTripLanelet2:
    def _ll2_carla_ll2_position_error(
        self, map_manager, lanelet_id: int, s: float, t: float
    ) -> float:
        """Return the Euclidean error (in CARLA space) for a round-trip."""
        original = Lanelet2Pose(lanelet_id=lanelet_id, s=s, t=t)
        carla = _lanelet2_to_carla(original)
        recovered = _carla_to_lanelet2(carla)
        recovered_carla = _lanelet2_to_carla(recovered)
        return math.sqrt(
            (carla.x - recovered_carla.x) ** 2 + (carla.y - recovered_carla.y) ** 2
        )

    def test_round_trip_on_centerline(self, map_manager):
        lanelet_id = next(iter(map_manager.lanelet_map.laneletLayer)).id
        error = self._ll2_carla_ll2_position_error(
            map_manager, lanelet_id, s=0.0, t=0.0
        )
        assert error < 1.0  # within 1 m

    def test_round_trip_with_lateral_offset(self, map_manager):
        lanelet_id = next(iter(map_manager.lanelet_map.laneletLayer)).id
        error = self._ll2_carla_ll2_position_error(
            map_manager, lanelet_id, s=0.0, t=0.5
        )
        assert error < 1.0


# ---------------------------------------------------------------------------
# TestRoundTripOpenDrive – OpenDrivePose → CarlaWorldPose → OpenDrivePose
# ---------------------------------------------------------------------------


class TestRoundTripOpenDrive:
    def _od_carla_od_position_error(
        self, map_manager, road_id: str, s: float, t: float
    ) -> float:
        """Return the Euclidean error (in CARLA space) for a round-trip."""
        original = OpenDrivePose(road_id=road_id, lane_id=0, s=s, t=t)
        carla = _opendrive_to_carla(original)
        recovered = _carla_to_opendrive(carla)
        recovered_carla = _opendrive_to_carla(recovered)
        return math.sqrt(
            (carla.x - recovered_carla.x) ** 2 + (carla.y - recovered_carla.y) ** 2
        )

    def test_round_trip_on_reference_line(self, map_manager):
        road_id = next(iter(map_manager.road_network.road_ids_to_object))
        error = self._od_carla_od_position_error(map_manager, road_id, s=0.0, t=0.0)
        assert error < 1.0

    def test_round_trip_with_lateral_offset(self, map_manager):
        road_id = next(iter(map_manager.road_network.road_ids_to_object))
        error = self._od_carla_od_position_error(map_manager, road_id, s=0.0, t=-1.0)
        assert error < 1.0


# ---------------------------------------------------------------------------
# TestCrossSystem – Lanelet2 ↔ OpenDRIVE via CARLA intermediate
# ---------------------------------------------------------------------------


class TestCrossSystem:
    def test_lanelet2_to_opendrive_and_back(self, map_manager):
        """Lanelet2 → CARLA → OpenDRIVE → CARLA: position should be close."""
        lanelet_id = next(iter(map_manager.lanelet_map.laneletLayer)).id
        ll2_pose = Lanelet2Pose(lanelet_id=lanelet_id, s=0.0, t=0.0)

        carla_orig = _lanelet2_to_carla(ll2_pose)
        od_pose = _carla_to_opendrive(carla_orig)
        carla_recovered = _opendrive_to_carla(od_pose)

        dist = math.sqrt(
            (carla_orig.x - carla_recovered.x) ** 2
            + (carla_orig.y - carla_recovered.y) ** 2
        )
        # Allow up to 5 m – lanelet centerlines may not coincide with road ref lines
        assert dist < 5.0

    def test_opendrive_to_lanelet2_and_back(self, map_manager):
        """OpenDRIVE → CARLA → Lanelet2 → CARLA: position should be close."""
        road_id = next(iter(map_manager.road_network.road_ids_to_object))
        od_pose = OpenDrivePose(road_id=road_id, lane_id=-1, s=0.0, t=0.0)

        carla_orig = _opendrive_to_carla(od_pose)
        ll2_pose = _carla_to_lanelet2(carla_orig)
        carla_recovered = _lanelet2_to_carla(ll2_pose)

        dist = math.sqrt(
            (carla_orig.x - carla_recovered.x) ** 2
            + (carla_orig.y - carla_recovered.y) ** 2
        )
        assert dist < 5.0


# ---------------------------------------------------------------------------
# TestPublicAPI – overloaded dispatch
# ---------------------------------------------------------------------------


class TestPublicAPI:
    def test_to_carla_world_from_lanelet2(self, map_manager):
        lanelet_id = next(iter(map_manager.lanelet_map.laneletLayer)).id
        result = to_carla_world(Lanelet2Pose(lanelet_id=lanelet_id, s=0.0))
        assert isinstance(result, CarlaWorldPose)

    def test_to_carla_world_from_opendrive(self, map_manager):
        road_id = next(iter(map_manager.road_network.road_ids_to_object))
        result = to_carla_world(OpenDrivePose(road_id=road_id, lane_id=-1, s=0.0))
        assert isinstance(result, CarlaWorldPose)

    def test_to_opendrive_from_carla(self, map_manager):
        lanelet_id = next(iter(map_manager.lanelet_map.laneletLayer)).id
        carla = to_carla_world(Lanelet2Pose(lanelet_id=lanelet_id, s=0.0))
        result = to_opendrive(carla)
        assert isinstance(result, OpenDrivePose)

    def test_to_lanelet2_from_carla(self, map_manager):
        road_id = next(iter(map_manager.road_network.road_ids_to_object))
        carla = to_carla_world(OpenDrivePose(road_id=road_id, lane_id=-1, s=0.0))
        result = to_lanelet2(carla)
        assert isinstance(result, Lanelet2Pose)

    def test_to_opendrive_from_lanelet2(self, map_manager):
        lanelet_id = next(iter(map_manager.lanelet_map.laneletLayer)).id
        result = to_opendrive(Lanelet2Pose(lanelet_id=lanelet_id, s=0.0))
        assert isinstance(result, OpenDrivePose)

    def test_to_lanelet2_from_opendrive(self, map_manager):
        road_id = next(iter(map_manager.road_network.road_ids_to_object))
        result = to_lanelet2(OpenDrivePose(road_id=road_id, lane_id=-1, s=0.0))
        assert isinstance(result, Lanelet2Pose)

    def test_invalid_type_raises(self):
        with pytest.raises(TypeError):
            to_carla_world("invalid")  # type: ignore[arg-type]
