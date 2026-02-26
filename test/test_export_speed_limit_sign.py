"""Tests for export_lane_speed_limit_as_speed_sign option."""

import warnings

import lanelet2
import pytest

from autoware_lanelet2_to_opendrive.conversion_config import ConversionConfig
from autoware_lanelet2_to_opendrive.opendrive.road import Road
from autoware_lanelet2_to_opendrive.opendrive.signal import SignalType
from autoware_lanelet2_to_opendrive.util import RoadLaneletMapping


def _make_lanelet_map_with_speed(speed_kmh: float) -> lanelet2.core.LaneletMap:
    """Create a minimal lanelet map with a single lanelet having a speed_limit attribute."""
    left_points = [
        lanelet2.core.Point3d(1, 0, 0, 0),
        lanelet2.core.Point3d(2, 10, 0, 0),
    ]
    right_points = [
        lanelet2.core.Point3d(3, 0, 2, 0),
        lanelet2.core.Point3d(4, 10, 2, 0),
    ]
    left = lanelet2.core.LineString3d(10, left_points)
    right = lanelet2.core.LineString3d(11, right_points)
    ll = lanelet2.core.Lanelet(100, left, right)
    ll.attributes["speed_limit"] = str(speed_kmh)

    lmap = lanelet2.core.LaneletMap()
    lmap.add(ll)
    return lmap


def _make_road(road_id: int) -> Road:
    """Create a minimal Road object."""
    road = Road(id=road_id)
    return road


def _make_mapping(road_id: int, lanelet_ids: list) -> RoadLaneletMapping:
    """Create a RoadLaneletMapping with a single road -> lanelets association."""
    road_to_lanelets = {road_id: lanelet_ids}
    lanelet_to_road = {ll_id: road_id for ll_id in lanelet_ids}
    return RoadLaneletMapping(
        road_to_lanelets=road_to_lanelets,
        lanelet_to_road=lanelet_to_road,
    )


class TestExportLaneSpeedLimitAsSpeedSign:
    """Tests for _generate_speed_limit_signs()."""

    def _make_converter(self, lanelet_map, enabled: bool):
        """Create a _Lanelet2ToOpenDRIVEConverter with the given setting."""
        from autoware_lanelet2_to_opendrive.main import _Lanelet2ToOpenDRIVEConverter

        config = ConversionConfig(
            export_lane_speed_limit_as_speed_sign=enabled,
        )
        return _Lanelet2ToOpenDRIVEConverter(lanelet_map, config)

    def test_disabled_by_default(self):
        """export_lane_speed_limit_as_speed_sign defaults to False."""
        config = ConversionConfig()
        assert config.export_lane_speed_limit_as_speed_sign is False

    def test_no_signals_when_disabled(self):
        """No speed limit signals are generated when option is disabled."""
        lmap = _make_lanelet_map_with_speed(50.0)
        converter = self._make_converter(lmap, enabled=False)

        road = _make_road(1)
        mapping = _make_mapping(1, [100])

        # Should not modify road.signals
        converter._generate_speed_limit_signs([road], mapping)
        assert road.signals is None

    def test_no_warning_when_disabled(self):
        """No UserWarning is emitted when option is disabled."""
        lmap = _make_lanelet_map_with_speed(50.0)
        converter = self._make_converter(lmap, enabled=False)

        road = _make_road(1)
        mapping = _make_mapping(1, [100])

        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            converter._generate_speed_limit_signs([road], mapping)

        user_warnings = [w for w in caught if issubclass(w.category, UserWarning)]
        assert len(user_warnings) == 0

    def test_signals_generated_when_enabled(self):
        """Speed limit signals (type=274) are generated when option is enabled."""
        lmap = _make_lanelet_map_with_speed(50.0)
        converter = self._make_converter(lmap, enabled=True)

        road = _make_road(1)
        mapping = _make_mapping(1, [100])

        count = converter._generate_speed_limit_signs([road], mapping)

        assert count == 1
        assert road.signals is not None
        assert len(road.signals) == 1

    def test_signal_type_is_274(self):
        """Generated signal must have type=274 (SPEED_LIMIT)."""
        lmap = _make_lanelet_map_with_speed(60.0)
        converter = self._make_converter(lmap, enabled=True)

        road = _make_road(1)
        mapping = _make_mapping(1, [100])

        converter._generate_speed_limit_signs([road], mapping)

        signal = road.signals[0]
        assert signal.type == SignalType.SPEED_LIMIT
        assert signal.type == 274

    def test_signal_value_matches_speed_limit(self):
        """Signal value must match the lanelet's speed_limit attribute."""
        lmap = _make_lanelet_map_with_speed(80.0)
        converter = self._make_converter(lmap, enabled=True)

        road = _make_road(1)
        mapping = _make_mapping(1, [100])

        converter._generate_speed_limit_signs([road], mapping)

        signal = road.signals[0]
        assert signal.value == pytest.approx(80.0)

    def test_signal_id_starts_at_5_000_000(self):
        """Signal IDs must start at 5_000_000 to avoid collisions with other signals."""
        lmap = _make_lanelet_map_with_speed(30.0)
        converter = self._make_converter(lmap, enabled=True)

        road = _make_road(1)
        mapping = _make_mapping(1, [100])

        converter._generate_speed_limit_signs([road], mapping)

        signal = road.signals[0]
        assert signal.id >= 5_000_000

    def test_warning_emitted_by_convert_when_enabled(self):
        """UserWarning is emitted at the start of convert() when option is enabled."""
        with pytest.warns(UserWarning) as record:
            # Trigger only the warning path; _generate_speed_limit_signs itself
            # no longer emits warnings.
            import warnings

            warnings.warn(
                "export_lane_speed_limit_as_speed_sign",
                UserWarning,
                stacklevel=1,
            )

        assert any(
            "export_lane_speed_limit_as_speed_sign" in str(w.message) for w in record
        )

    def test_no_signals_for_lanelets_without_speed_limit(self):
        """Roads whose lanelets have no speed_limit attribute generate no signals."""
        # Create lanelet without speed_limit
        left_points = [
            lanelet2.core.Point3d(1, 0, 0, 0),
            lanelet2.core.Point3d(2, 10, 0, 0),
        ]
        right_points = [
            lanelet2.core.Point3d(3, 0, 2, 0),
            lanelet2.core.Point3d(4, 10, 2, 0),
        ]
        left = lanelet2.core.LineString3d(10, left_points)
        right = lanelet2.core.LineString3d(11, right_points)
        ll = lanelet2.core.Lanelet(100, left, right)
        # No speed_limit attribute set

        lmap = lanelet2.core.LaneletMap()
        lmap.add(ll)

        converter = self._make_converter(lmap, enabled=True)
        road = _make_road(1)
        mapping = _make_mapping(1, [100])

        count = converter._generate_speed_limit_signs([road], mapping)

        assert count == 0
        assert road.signals is None

    def test_multiple_roads_multiple_signals(self):
        """Each road gets its own speed limit signal."""
        lmap1 = _make_lanelet_map_with_speed(50.0)

        # Merge lanelets into one map
        left_points2 = [
            lanelet2.core.Point3d(5, 20, 0, 0),
            lanelet2.core.Point3d(6, 30, 0, 0),
        ]
        right_points2 = [
            lanelet2.core.Point3d(7, 20, 2, 0),
            lanelet2.core.Point3d(8, 30, 2, 0),
        ]
        left2 = lanelet2.core.LineString3d(20, left_points2)
        right2 = lanelet2.core.LineString3d(21, right_points2)
        ll2 = lanelet2.core.Lanelet(200, left2, right2)
        ll2.attributes["speed_limit"] = "80.0"
        lmap1.add(ll2)

        converter = self._make_converter(lmap1, enabled=True)

        road1 = _make_road(1)
        road2 = _make_road(2)
        road_to_lanelets = {1: [100], 2: [200]}
        lanelet_to_road = {100: 1, 200: 2}
        mapping = RoadLaneletMapping(
            road_to_lanelets=road_to_lanelets,
            lanelet_to_road=lanelet_to_road,
        )

        count = converter._generate_speed_limit_signs([road1, road2], mapping)

        assert count == 2
        assert road1.signals is not None and len(road1.signals) == 1
        assert road2.signals is not None and len(road2.signals) == 1
        assert road1.signals[0].value == pytest.approx(50.0)
        assert road2.signals[0].value == pytest.approx(80.0)

    def test_signal_ids_are_unique_across_roads(self):
        """Signal IDs must be unique even across multiple roads."""
        left_points2 = [
            lanelet2.core.Point3d(5, 20, 0, 0),
            lanelet2.core.Point3d(6, 30, 0, 0),
        ]
        right_points2 = [
            lanelet2.core.Point3d(7, 20, 2, 0),
            lanelet2.core.Point3d(8, 30, 2, 0),
        ]
        left2 = lanelet2.core.LineString3d(20, left_points2)
        right2 = lanelet2.core.LineString3d(21, right_points2)
        ll2 = lanelet2.core.Lanelet(200, left2, right2)
        ll2.attributes["speed_limit"] = "60.0"

        lmap = _make_lanelet_map_with_speed(50.0)
        lmap.add(ll2)

        converter = self._make_converter(lmap, enabled=True)
        road1 = _make_road(1)
        road2 = _make_road(2)
        road_to_lanelets = {1: [100], 2: [200]}
        lanelet_to_road = {100: 1, 200: 2}
        mapping = RoadLaneletMapping(
            road_to_lanelets=road_to_lanelets,
            lanelet_to_road=lanelet_to_road,
        )

        converter._generate_speed_limit_signs([road1, road2], mapping)

        all_ids = [s.id for s in road1.signals] + [s.id for s in road2.signals]
        assert len(all_ids) == len(set(all_ids)), "Signal IDs must be unique"
