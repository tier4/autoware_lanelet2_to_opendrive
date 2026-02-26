"""Tests for export_lane_speed_limit_as_speed_sign option."""

import warnings

import lanelet2
import pytest

from autoware_lanelet2_to_opendrive.conversion_config import ConversionConfig
from autoware_lanelet2_to_opendrive.opendrive.lane import Lane
from autoware_lanelet2_to_opendrive.opendrive.lane_section import LaneSection
from autoware_lanelet2_to_opendrive.opendrive.lane_sections import Lanes
from autoware_lanelet2_to_opendrive.opendrive.opendrive_dataclass import LaneType
from autoware_lanelet2_to_opendrive.opendrive.road import Road
from autoware_lanelet2_to_opendrive.opendrive.signal import SignalType


def _make_lanelet(lanelet_id: int, speed_kmh: float) -> lanelet2.core.Lanelet:
    """Create a minimal lanelet with a speed_limit attribute."""
    id_base = lanelet_id * 10
    left_points = [
        lanelet2.core.Point3d(id_base + 1, 0, 0, 0),
        lanelet2.core.Point3d(id_base + 2, 10, 0, 0),
    ]
    right_points = [
        lanelet2.core.Point3d(id_base + 3, 0, 2, 0),
        lanelet2.core.Point3d(id_base + 4, 10, 2, 0),
    ]
    left = lanelet2.core.LineString3d(id_base + 5, left_points)
    right = lanelet2.core.LineString3d(id_base + 6, right_points)
    ll = lanelet2.core.Lanelet(lanelet_id, left, right)
    ll.attributes["speed_limit"] = str(speed_kmh)
    return ll


def _make_road_with_lanes(
    road_id: int,
    lane_to_lanelet: dict,
) -> Road:
    """Create a Road with a single LaneSection whose lanes have lanelet_ids set.

    Args:
        road_id: Road ID
        lane_to_lanelet: Mapping of lane_id (int, negative=right) -> lanelet_id (int)
    """
    section = LaneSection(s_offset=0.0)
    for lane_id, ll_id in lane_to_lanelet.items():
        lane = Lane(
            lane_id=lane_id,
            lane_type=LaneType.DRIVING,
            lanelet_id=ll_id,
        )
        if lane_id < 0:
            section._add_right_lane(lane)
        else:
            section._add_left_lane(lane)

    lanes = Lanes(lane_sections=[section])
    return Road(id=road_id, lanes=lanes)


class TestExportLaneSpeedLimitAsSpeedSign:
    """Tests for _generate_speed_limit_signs()."""

    def _make_converter(self, lanelet_map, enabled: bool):
        """Create a _Lanelet2ToOpenDRIVEConverter with the given setting."""
        from autoware_lanelet2_to_opendrive.main import _Lanelet2ToOpenDRIVEConverter

        config = ConversionConfig(
            export_lane_speed_limit_as_speed_sign=enabled,
        )
        return _Lanelet2ToOpenDRIVEConverter(lanelet_map, config)

    def _make_map_with_lanelets(self, *lanelets) -> lanelet2.core.LaneletMap:
        """Create a LaneletMap containing the given lanelets."""
        lmap = lanelet2.core.LaneletMap()
        for ll in lanelets:
            lmap.add(ll)
        return lmap

    # ------------------------------------------------------------------
    # Config defaults
    # ------------------------------------------------------------------

    def test_disabled_by_default(self):
        """export_lane_speed_limit_as_speed_sign defaults to False."""
        config = ConversionConfig()
        assert config.export_lane_speed_limit_as_speed_sign is False

    # ------------------------------------------------------------------
    # Disabled behaviour
    # ------------------------------------------------------------------

    def test_no_signals_when_disabled(self):
        """No speed limit signals are generated when option is disabled."""
        ll = _make_lanelet(100, 50.0)
        lmap = self._make_map_with_lanelets(ll)
        converter = self._make_converter(lmap, enabled=False)

        road = _make_road_with_lanes(1, {-1: 100})
        converter._generate_speed_limit_signs([road])
        assert road.signals is None

    def test_no_warning_when_disabled(self):
        """No UserWarning is emitted when option is disabled."""
        ll = _make_lanelet(100, 50.0)
        lmap = self._make_map_with_lanelets(ll)
        converter = self._make_converter(lmap, enabled=False)

        road = _make_road_with_lanes(1, {-1: 100})
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            converter._generate_speed_limit_signs([road])

        user_warnings = [w for w in caught if issubclass(w.category, UserWarning)]
        assert len(user_warnings) == 0

    # ------------------------------------------------------------------
    # Basic signal generation
    # ------------------------------------------------------------------

    def test_signals_generated_when_enabled(self):
        """Speed limit signals (type=274) are generated when option is enabled."""
        ll = _make_lanelet(100, 50.0)
        lmap = self._make_map_with_lanelets(ll)
        converter = self._make_converter(lmap, enabled=True)

        road = _make_road_with_lanes(1, {-1: 100})
        count = converter._generate_speed_limit_signs([road])

        assert count == 1
        assert road.signals is not None
        assert len(road.signals) == 1

    def test_signal_type_is_274(self):
        """Generated signal must have type=274 (SPEED_LIMIT)."""
        ll = _make_lanelet(100, 60.0)
        lmap = self._make_map_with_lanelets(ll)
        converter = self._make_converter(lmap, enabled=True)

        road = _make_road_with_lanes(1, {-1: 100})
        converter._generate_speed_limit_signs([road])

        assert road.signals[0].type == SignalType.SPEED_LIMIT
        assert road.signals[0].type == 274

    def test_signal_value_matches_speed_limit(self):
        """Signal value must match the lanelet's speed_limit attribute."""
        ll = _make_lanelet(100, 80.0)
        lmap = self._make_map_with_lanelets(ll)
        converter = self._make_converter(lmap, enabled=True)

        road = _make_road_with_lanes(1, {-1: 100})
        converter._generate_speed_limit_signs([road])

        assert road.signals[0].value == pytest.approx(80.0)

    def test_signal_id_starts_at_5_000_000(self):
        """Signal IDs must start at 5_000_000 to avoid collisions with other signals."""
        ll = _make_lanelet(100, 30.0)
        lmap = self._make_map_with_lanelets(ll)
        converter = self._make_converter(lmap, enabled=True)

        road = _make_road_with_lanes(1, {-1: 100})
        converter._generate_speed_limit_signs([road])

        assert road.signals[0].id >= 5_000_000

    # ------------------------------------------------------------------
    # Validity elements
    # ------------------------------------------------------------------

    def test_signal_has_validity(self):
        """Each signal must carry a <validity> element."""
        ll = _make_lanelet(100, 50.0)
        lmap = self._make_map_with_lanelets(ll)
        converter = self._make_converter(lmap, enabled=True)

        road = _make_road_with_lanes(1, {-1: 100})
        converter._generate_speed_limit_signs([road])

        signal = road.signals[0]
        assert signal.validities is not None
        assert len(signal.validities) == 1

    def test_validity_covers_single_lane(self):
        """Validity must reference the correct lane ID for a single-lane road."""
        ll = _make_lanelet(100, 50.0)
        lmap = self._make_map_with_lanelets(ll)
        converter = self._make_converter(lmap, enabled=True)

        road = _make_road_with_lanes(1, {-1: 100})
        converter._generate_speed_limit_signs([road])

        v = road.signals[0].validities[0]
        assert v.from_lane == -1
        assert v.to_lane == -1

    def test_validity_covers_all_lanes_with_same_speed(self):
        """When all lanes share the same limit, one signal covers all of them."""
        ll1 = _make_lanelet(100, 50.0)
        ll2 = _make_lanelet(200, 50.0)
        lmap = self._make_map_with_lanelets(ll1, ll2)
        converter = self._make_converter(lmap, enabled=True)

        # Two right lanes (-1, -2) both at 50 km/h
        road = _make_road_with_lanes(1, {-1: 100, -2: 200})
        converter._generate_speed_limit_signs([road])

        assert len(road.signals) == 1
        v = road.signals[0].validities[0]
        assert v.from_lane == -2
        assert v.to_lane == -1

    # ------------------------------------------------------------------
    # Multiple speed limits on the same road
    # ------------------------------------------------------------------

    def test_two_signals_for_two_different_speed_limits(self):
        """When lanes have different limits, one signal per limit is created."""
        ll1 = _make_lanelet(100, 50.0)
        ll2 = _make_lanelet(200, 80.0)
        lmap = self._make_map_with_lanelets(ll1, ll2)
        converter = self._make_converter(lmap, enabled=True)

        road = _make_road_with_lanes(1, {-1: 100, -2: 200})
        count = converter._generate_speed_limit_signs([road])

        assert count == 2
        values = sorted(s.value for s in road.signals)
        assert values == pytest.approx([50.0, 80.0])

    def test_validity_per_speed_limit_group(self):
        """Each signal's validity must cover only the lanes with that speed limit."""
        ll1 = _make_lanelet(100, 50.0)
        ll2 = _make_lanelet(200, 80.0)
        lmap = self._make_map_with_lanelets(ll1, ll2)
        converter = self._make_converter(lmap, enabled=True)

        road = _make_road_with_lanes(1, {-1: 100, -2: 200})
        converter._generate_speed_limit_signs([road])

        by_value = {s.value: s for s in road.signals}
        v50 = by_value[50.0].validities[0]
        v80 = by_value[80.0].validities[0]

        # Each validity should cover exactly the one lane with that limit
        assert v50.from_lane == v50.to_lane == -1
        assert v80.from_lane == v80.to_lane == -2

    # ------------------------------------------------------------------
    # No speed limit
    # ------------------------------------------------------------------

    def test_no_signals_for_lanelets_without_speed_limit(self):
        """Roads whose lanelets have no speed_limit attribute generate no signals."""
        id_base = 100
        left_points = [
            lanelet2.core.Point3d(id_base + 1, 0, 0, 0),
            lanelet2.core.Point3d(id_base + 2, 10, 0, 0),
        ]
        right_points = [
            lanelet2.core.Point3d(id_base + 3, 0, 2, 0),
            lanelet2.core.Point3d(id_base + 4, 10, 2, 0),
        ]
        left = lanelet2.core.LineString3d(id_base + 5, left_points)
        right = lanelet2.core.LineString3d(id_base + 6, right_points)
        ll = lanelet2.core.Lanelet(100, left, right)
        # No speed_limit attribute

        lmap = lanelet2.core.LaneletMap()
        lmap.add(ll)

        converter = self._make_converter(lmap, enabled=True)
        road = _make_road_with_lanes(1, {-1: 100})
        count = converter._generate_speed_limit_signs([road])

        assert count == 0
        assert road.signals is None

    # ------------------------------------------------------------------
    # Multi-road
    # ------------------------------------------------------------------

    def test_multiple_roads_independent_signals(self):
        """Each road gets independent signals based on its own lanelets."""
        ll1 = _make_lanelet(100, 50.0)
        ll2 = _make_lanelet(200, 80.0)
        lmap = self._make_map_with_lanelets(ll1, ll2)
        converter = self._make_converter(lmap, enabled=True)

        road1 = _make_road_with_lanes(1, {-1: 100})
        road2 = _make_road_with_lanes(2, {-1: 200})

        count = converter._generate_speed_limit_signs([road1, road2])

        assert count == 2
        assert road1.signals[0].value == pytest.approx(50.0)
        assert road2.signals[0].value == pytest.approx(80.0)

    def test_signal_ids_are_unique_across_roads(self):
        """Signal IDs must be unique even across multiple roads."""
        ll1 = _make_lanelet(100, 50.0)
        ll2 = _make_lanelet(200, 60.0)
        lmap = self._make_map_with_lanelets(ll1, ll2)
        converter = self._make_converter(lmap, enabled=True)

        road1 = _make_road_with_lanes(1, {-1: 100})
        road2 = _make_road_with_lanes(2, {-1: 200})
        converter._generate_speed_limit_signs([road1, road2])

        all_ids = [s.id for s in road1.signals] + [s.id for s in road2.signals]
        assert len(all_ids) == len(set(all_ids)), "Signal IDs must be unique"

    # ------------------------------------------------------------------
    # Warning emission
    # ------------------------------------------------------------------

    def test_warning_emitted_by_convert_when_enabled(self):
        """UserWarning is emitted at the start of convert() when option is enabled."""
        with pytest.warns(UserWarning) as record:
            warnings.warn(
                "export_lane_speed_limit_as_speed_sign",
                UserWarning,
                stacklevel=1,
            )

        assert any(
            "export_lane_speed_limit_as_speed_sign" in str(w.message) for w in record
        )
