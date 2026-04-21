"""Regression tests for the Lanelet2 subtype -> OpenDRIVE LaneType mapping.

These tests exercise ``Lane.construct_from_lanelet`` directly with minimal
in-memory lanelets (no ``.osm`` round-trip, no MGRS projection) to lock in
the subtype -> ``LaneType`` mapping. They close the coverage gap introduced
when commit b6edad5 silently extended the subtype filter in
``opendrive/road.py`` to include ``highway`` (and now ``road_shoulder``) in
addition to ``road`` and ``walkway``.

If a future refactor drops or changes a branch in the subtype mapping, these
tests will fail and force an explicit decision.
"""

import lanelet2
import pytest

from autoware_lanelet2_to_opendrive.opendrive.lane import Lane
from autoware_lanelet2_to_opendrive.opendrive.opendrive_dataclass import LaneType


def _make_straight_lanelet(
    subtype: str | None,
) -> tuple[lanelet2.core.LaneletMap, lanelet2.core.Lanelet]:
    """Build a minimal 10 m straight lanelet of width 2 m with the given subtype.

    Each invocation uses fresh IDs via ``lanelet2.core.getId()`` so tests do
    not clash when run in parallel (``pytest -n auto``).
    """
    left_points = [
        lanelet2.core.Point3d(lanelet2.core.getId(), 0.0, 0.0, 0.0),
        lanelet2.core.Point3d(lanelet2.core.getId(), 10.0, 0.0, 0.0),
    ]
    right_points = [
        lanelet2.core.Point3d(lanelet2.core.getId(), 0.0, 2.0, 0.0),
        lanelet2.core.Point3d(lanelet2.core.getId(), 10.0, 2.0, 0.0),
    ]
    left_bound = lanelet2.core.LineString3d(lanelet2.core.getId(), left_points)
    right_bound = lanelet2.core.LineString3d(lanelet2.core.getId(), right_points)
    lanelet = lanelet2.core.Lanelet(lanelet2.core.getId(), left_bound, right_bound)
    if subtype is not None:
        lanelet.attributes["subtype"] = subtype

    lanelet_map = lanelet2.core.LaneletMap()
    lanelet_map.add(lanelet)
    return lanelet_map, lanelet


@pytest.mark.parametrize(
    "subtype,expected_type",
    [
        ("road", LaneType.DRIVING),
        ("highway", LaneType.DRIVING),
        ("walkway", LaneType.SIDEWALK),
        ("road_shoulder", LaneType.SHOULDER),
        ("bicycle_lane", LaneType.BIKING),
    ],
)
def test_subtype_maps_to_expected_lane_type(
    subtype: str, expected_type: LaneType
) -> None:
    """Each documented Lanelet2 subtype maps to its OpenDRIVE counterpart.

    Parametrized so that adding a new subtype branch in
    ``Lane.construct_from_lanelet`` has an obvious place to extend coverage.
    """
    lanelet_map, lanelet = _make_straight_lanelet(subtype)

    lane = Lane.construct_from_lanelet(lanelet_map, lanelet)

    assert lane.lane_type is expected_type


def test_highway_subtype_maps_to_driving() -> None:
    """Dedicated regression guard for the ``highway`` mapping added in b6edad5.

    The commit extended the subtype filter in ``opendrive/road.py`` to
    include ``highway`` but added no test for it; this test fills that gap.
    """
    lanelet_map, lanelet = _make_straight_lanelet("highway")

    lane = Lane.construct_from_lanelet(lanelet_map, lanelet)

    assert lane.lane_type is LaneType.DRIVING


def test_road_shoulder_subtype_maps_to_shoulder() -> None:
    """``road_shoulder`` must produce ``LaneType.SHOULDER``.

    Prior to this commit ``road_shoulder`` was handled in
    ``Lane.construct_from_lanelet`` but was not in the subtype filter in
    ``opendrive/road.py``, making the branch unreachable end-to-end. The
    filter has since been extended; this test locks the mapping in.
    """
    lanelet_map, lanelet = _make_straight_lanelet("road_shoulder")

    lane = Lane.construct_from_lanelet(lanelet_map, lanelet)

    assert lane.lane_type is LaneType.SHOULDER


def test_walkway_subtype_maps_to_sidewalk() -> None:
    """Confirmation test: walkway still maps to sidewalk (P0-1 behaviour)."""
    lanelet_map, lanelet = _make_straight_lanelet("walkway")

    lane = Lane.construct_from_lanelet(lanelet_map, lanelet)

    assert lane.lane_type is LaneType.SIDEWALK


def test_unknown_subtype_falls_back_to_driving() -> None:
    """An unrecognised subtype (e.g. ``ferry``) falls back to DRIVING."""
    lanelet_map, lanelet = _make_straight_lanelet("ferry")

    lane = Lane.construct_from_lanelet(lanelet_map, lanelet)

    assert lane.lane_type is LaneType.DRIVING


def test_missing_subtype_falls_back_to_driving() -> None:
    """A lanelet without any ``subtype`` attribute falls back to DRIVING."""
    lanelet_map, lanelet = _make_straight_lanelet(subtype=None)

    lane = Lane.construct_from_lanelet(lanelet_map, lanelet)

    assert lane.lane_type is LaneType.DRIVING
