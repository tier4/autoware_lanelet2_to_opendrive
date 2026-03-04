"""Unit tests for the coordinate transformation module.

All tests use mocks — no CARLA connection or real map files required.
"""

from __future__ import annotations

from pathlib import Path
from unittest.mock import MagicMock, patch

import numpy as np
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


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture(autouse=True)
def reset_map_manager():
    """Reset MapManager singleton before and after each test."""
    MapManager.reset()
    yield
    MapManager.reset()


def _make_mock_lanelet(
    lanelet_id: int, centerline_pts: list[tuple[float, float, float]]
):
    """Build a mock Lanelet object with a centerline."""
    lanelet = MagicMock()
    lanelet.id = lanelet_id
    pts = []
    for x, y, z in centerline_pts:
        pt = MagicMock()
        pt.x, pt.y, pt.z = x, y, z
        pts.append(pt)
    lanelet.centerline = pts
    return lanelet


def _make_mock_road(road_id: str, ref_line: np.ndarray, z_coords: np.ndarray):
    """Build a mock pyxodr Road object."""
    road = MagicMock()
    road.reference_line = ref_line
    road.z_coordinates = z_coords
    road.lane_sections = []
    return road, road_id


def _inject_mock_map_manager(lanelet_map=None, road_network=None):
    """Inject pre-built mocks into the MapManager singleton."""
    mm = MapManager.get_instance()
    mm._lanelet_map = lanelet_map
    mm._road_network = road_network
    mm._geo_origin = (35.0, 139.0, 0.0)
    return mm


# ---------------------------------------------------------------------------
# TestMapManagerSingleton
# ---------------------------------------------------------------------------


class TestMapManagerSingleton:
    def test_same_instance(self):
        a = MapManager.get_instance()
        b = MapManager.get_instance()
        assert a is b

    def test_double_init_raises(self, tmp_path):
        xodr = tmp_path / "map.xodr"
        osm = tmp_path / "map.osm"

        proj = "+proj=tmerc +lat_0=35.0 +lon_0=139.0 +k=1 +x_0=0 +y_0=0"
        xodr.write_text(
            f"<OpenDRIVE><header><geoReference><![CDATA[{proj}]]></geoReference></header></OpenDRIVE>"
        )
        osm.write_text("<osm></osm>")

        mm = MapManager.get_instance()

        # Patch lanelet2 and pyxodr so no real files are needed
        with (
            patch("lanelet2.io.load", return_value=MagicMock()),
            patch("lanelet2.io.Origin", return_value=MagicMock()),
            patch("lanelet2.projection.UtmProjector", return_value=MagicMock()),
            patch(
                "autoware_carla_scenario.coordinate.map_manager.RoadNetwork",
                return_value=MagicMock(),
            ),
        ):
            mm.initialize(xodr, osm)
            with pytest.raises(RuntimeError, match="already initialized"):
                mm.initialize(xodr, osm)

    def test_reset_clears_instance(self):
        a = MapManager.get_instance()
        MapManager.reset()
        b = MapManager.get_instance()
        assert a is not b

    def test_access_before_init_raises(self):
        mm = MapManager.get_instance()
        with pytest.raises(RuntimeError, match="not initialized"):
            _ = mm.lanelet_map
        with pytest.raises(RuntimeError, match="not initialized"):
            _ = mm.road_network

    def test_file_not_found_raises(self, tmp_path):
        mm = MapManager.get_instance()
        with pytest.raises(FileNotFoundError):
            mm.initialize(Path("nonexistent.xodr"), Path("nonexistent.osm"))


# ---------------------------------------------------------------------------
# TestParseGeoReference
# ---------------------------------------------------------------------------


class TestParseGeoReference:
    _PROJ = (
        "+proj=tmerc +lat_0=35.123 +lon_0=139.456 +k=1 +x_0=0 +y_0=0 "
        "+ellps=GRS80 +units=m +no_defs"
    )
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
# TestLanelet2ToCarla
# ---------------------------------------------------------------------------


class TestLanelet2ToCarla:
    """Straight centerline along +x axis: (0,0,0)→(5,0,0)→(10,0,0)."""

    CL = [(0.0, 0.0, 0.0), (5.0, 0.0, 0.0), (10.0, 0.0, 0.0)]
    LANELET_ID = 42

    def _setup(self):
        lanelet = _make_mock_lanelet(self.LANELET_ID, self.CL)
        layer = {self.LANELET_ID: lanelet}
        lmap = MagicMock()
        lmap.laneletLayer = layer
        _inject_mock_map_manager(lanelet_map=lmap)

    def test_start_of_lanelet(self):
        self._setup()
        pose = Lanelet2Pose(lanelet_id=self.LANELET_ID, s=0.0)
        result = _lanelet2_to_carla(pose)
        assert result.x == pytest.approx(0.0, abs=1e-6)
        assert result.y == pytest.approx(0.0, abs=1e-6)  # −ll2_y = −0 = 0

    def test_midpoint(self):
        self._setup()
        pose = Lanelet2Pose(lanelet_id=self.LANELET_ID, s=5.0)
        result = _lanelet2_to_carla(pose)
        assert result.x == pytest.approx(5.0, abs=1e-6)
        assert result.y == pytest.approx(0.0, abs=1e-6)

    def test_end_of_lanelet(self):
        self._setup()
        pose = Lanelet2Pose(lanelet_id=self.LANELET_ID, s=10.0)
        result = _lanelet2_to_carla(pose)
        assert result.x == pytest.approx(10.0, abs=1e-6)

    def test_lateral_offset_left(self):
        """t>0 is left of +x heading, which is +y in Lanelet2 → −y in CARLA."""
        self._setup()
        pose = Lanelet2Pose(lanelet_id=self.LANELET_ID, s=5.0, t=2.0)
        result = _lanelet2_to_carla(pose)
        assert result.x == pytest.approx(5.0, abs=1e-6)
        # Heading=0 (East): left offset → ll2_y += 2 → carla_y = −2
        assert result.y == pytest.approx(-2.0, abs=1e-6)

    def test_yaw_east_is_zero(self):
        """Heading=0 (East) → CARLA yaw = 0."""
        self._setup()
        pose = Lanelet2Pose(lanelet_id=self.LANELET_ID, s=5.0)
        result = _lanelet2_to_carla(pose)
        assert result.yaw == pytest.approx(0.0, abs=1e-6)

    def test_yaw_north_is_negative_90(self):
        """Lanelet heading=π/2 (North) → CARLA yaw = −90 deg."""
        cl_north = [(0.0, 0.0, 0.0), (0.0, 5.0, 0.0), (0.0, 10.0, 0.0)]
        lanelet = _make_mock_lanelet(99, cl_north)
        lmap = MagicMock()
        lmap.laneletLayer = {99: lanelet}
        _inject_mock_map_manager(lanelet_map=lmap)

        pose = Lanelet2Pose(lanelet_id=99, s=5.0)
        result = _lanelet2_to_carla(pose)
        assert result.yaw == pytest.approx(-90.0, abs=1e-4)


# ---------------------------------------------------------------------------
# TestOpenDriveToCarla
# ---------------------------------------------------------------------------


class TestOpenDriveToCarla:
    """Straight reference line along +x axis from (0,0) to (10,0)."""

    REF = np.array([[0.0, 0.0], [5.0, 0.0], [10.0, 0.0]])
    Z = np.array([0.0, 0.0, 0.0])
    ROAD_ID = "road_0"

    def _setup(self):
        road = MagicMock()
        road.reference_line = self.REF
        road.z_coordinates = self.Z
        road.lane_sections = []

        rnet = MagicMock()
        rnet.road_ids_to_object = {self.ROAD_ID: road}
        _inject_mock_map_manager(road_network=rnet)

    def test_start_position(self):
        self._setup()
        pose = OpenDrivePose(road_id=self.ROAD_ID, lane_id=-1, s=0.0)
        result = _opendrive_to_carla(pose)
        assert result.x == pytest.approx(0.0, abs=1e-6)
        assert result.y == pytest.approx(0.0, abs=1e-6)

    def test_midpoint(self):
        self._setup()
        pose = OpenDrivePose(road_id=self.ROAD_ID, lane_id=-1, s=5.0)
        result = _opendrive_to_carla(pose)
        assert result.x == pytest.approx(5.0, abs=1e-6)

    def test_lateral_offset_positive_t_is_left(self):
        """Positive t = left of +x heading = +y in UTM → −y in CARLA."""
        self._setup()
        pose = OpenDrivePose(road_id=self.ROAD_ID, lane_id=1, s=5.0, t=3.0)
        result = _opendrive_to_carla(pose)
        assert result.x == pytest.approx(5.0, abs=1e-6)
        assert result.y == pytest.approx(-3.0, abs=1e-6)

    def test_lateral_offset_negative_t_is_right(self):
        """Negative t = right of +x heading = −y in UTM → +y in CARLA."""
        self._setup()
        pose = OpenDrivePose(road_id=self.ROAD_ID, lane_id=-1, s=5.0, t=-3.0)
        result = _opendrive_to_carla(pose)
        assert result.y == pytest.approx(3.0, abs=1e-6)


# ---------------------------------------------------------------------------
# TestCarlaToOpendrive (round-trip)
# ---------------------------------------------------------------------------


class TestCarlaToOpendrive:
    """Round-trip: OpenDrivePose → CarlaWorldPose → OpenDrivePose ≈ original."""

    REF = np.array([[0.0, 0.0], [10.0, 0.0], [20.0, 0.0]])
    Z = np.zeros(3)
    ROAD_ID = "0"

    def _setup(self):
        road = MagicMock()
        road.reference_line = self.REF
        road.z_coordinates = self.Z
        road.lane_sections = []

        rnet = MagicMock()
        rnet.road_ids_to_object = {self.ROAD_ID: road}
        _inject_mock_map_manager(road_network=rnet)

    def test_round_trip_on_reference_line(self):
        self._setup()
        original = OpenDrivePose(road_id=self.ROAD_ID, lane_id=0, s=10.0, t=0.0)
        carla = _opendrive_to_carla(original)
        recovered = _carla_to_opendrive(carla)

        assert recovered.road_id == self.ROAD_ID
        assert recovered.s == pytest.approx(original.s, abs=0.5)
        assert recovered.t == pytest.approx(original.t, abs=0.5)

    def test_round_trip_with_lateral_offset(self):
        self._setup()
        original = OpenDrivePose(road_id=self.ROAD_ID, lane_id=-1, s=10.0, t=-2.0)
        carla = _opendrive_to_carla(original)
        recovered = _carla_to_opendrive(carla)

        assert recovered.road_id == self.ROAD_ID
        assert recovered.s == pytest.approx(original.s, abs=0.5)
        assert recovered.t == pytest.approx(original.t, abs=0.5)


# ---------------------------------------------------------------------------
# TestCarlaToLanelet2 (round-trip)
# ---------------------------------------------------------------------------


class TestCarlaToLanelet2:
    """Round-trip: Lanelet2Pose → CarlaWorldPose → Lanelet2Pose ≈ original."""

    CL = [(0.0, 0.0, 0.0), (5.0, 0.0, 0.0), (10.0, 0.0, 0.0)]
    LANELET_ID = 100

    def _setup(self):
        lanelet = _make_mock_lanelet(self.LANELET_ID, self.CL)
        lanelet.id = self.LANELET_ID

        # findNearest returns [(dist, lanelet)]
        nearest_result = [(0.0, lanelet)]

        lmap = MagicMock()
        lmap.laneletLayer = {self.LANELET_ID: lanelet}
        _inject_mock_map_manager(lanelet_map=lmap)

        # Patch lanelet2.geometry.findNearest
        self._lanelet = lanelet
        self._nearest_result = nearest_result

    def test_round_trip_on_centerline(self):
        self._setup()
        original = Lanelet2Pose(lanelet_id=self.LANELET_ID, s=5.0, t=0.0)
        carla = _lanelet2_to_carla(original)

        with patch(
            "lanelet2.geometry.findNearest",
            return_value=self._nearest_result,
        ):
            recovered = _carla_to_lanelet2(carla)

        assert recovered.lanelet_id == self.LANELET_ID
        assert recovered.s == pytest.approx(original.s, abs=0.5)
        assert recovered.t == pytest.approx(original.t, abs=0.5)

    def test_round_trip_with_lateral_offset(self):
        self._setup()
        original = Lanelet2Pose(lanelet_id=self.LANELET_ID, s=5.0, t=1.0)
        carla = _lanelet2_to_carla(original)

        with patch(
            "lanelet2.geometry.findNearest",
            return_value=self._nearest_result,
        ):
            recovered = _carla_to_lanelet2(carla)

        assert recovered.s == pytest.approx(original.s, abs=0.5)
        assert recovered.t == pytest.approx(original.t, abs=0.5)


# ---------------------------------------------------------------------------
# TestPublicAPI (overloaded dispatch)
# ---------------------------------------------------------------------------


class TestPublicAPI:
    """Verify that the public overloaded functions dispatch correctly."""

    CL = [(0.0, 0.0, 0.0), (10.0, 0.0, 0.0)]
    REF = np.array([[0.0, 0.0], [10.0, 0.0]])
    Z = np.zeros(2)

    def _setup(self):
        lanelet = _make_mock_lanelet(1, self.CL)
        lanelet.id = 1
        lmap = MagicMock()
        lmap.laneletLayer = {1: lanelet}

        road = MagicMock()
        road.reference_line = self.REF
        road.z_coordinates = self.Z
        road.lane_sections = []
        rnet = MagicMock()
        rnet.road_ids_to_object = {"0": road}

        _inject_mock_map_manager(lanelet_map=lmap, road_network=rnet)
        self._lanelet = lanelet

    def test_to_carla_world_from_lanelet2(self):
        self._setup()
        result = to_carla_world(Lanelet2Pose(lanelet_id=1, s=5.0))
        assert isinstance(result, CarlaWorldPose)

    def test_to_carla_world_from_opendrive(self):
        self._setup()
        result = to_carla_world(OpenDrivePose(road_id="0", lane_id=0, s=5.0))
        assert isinstance(result, CarlaWorldPose)

    def test_to_opendrive_from_carla(self):
        self._setup()
        result = to_opendrive(CarlaWorldPose(x=5.0, y=0.0, z=0.0))
        assert isinstance(result, OpenDrivePose)

    def test_to_lanelet2_from_carla(self):
        self._setup()
        nearest_result = [(0.0, self._lanelet)]
        with patch(
            "lanelet2.geometry.findNearest",
            return_value=nearest_result,
        ):
            result = to_lanelet2(CarlaWorldPose(x=5.0, y=0.0, z=0.0))
        assert isinstance(result, Lanelet2Pose)

    def test_to_opendrive_from_lanelet2(self):
        """Lanelet2 → OpenDRIVE goes through CARLA as intermediate."""
        self._setup()
        result = to_opendrive(Lanelet2Pose(lanelet_id=1, s=5.0))
        assert isinstance(result, OpenDrivePose)

    def test_to_lanelet2_from_opendrive(self):
        """OpenDRIVE → Lanelet2 goes through CARLA as intermediate."""
        self._setup()
        nearest_result = [(0.0, self._lanelet)]
        with patch(
            "lanelet2.geometry.findNearest",
            return_value=nearest_result,
        ):
            result = to_lanelet2(OpenDrivePose(road_id="0", lane_id=0, s=5.0))
        assert isinstance(result, Lanelet2Pose)

    def test_invalid_type_raises(self):
        with pytest.raises(TypeError):
            to_carla_world("invalid")  # type: ignore[arg-type]
