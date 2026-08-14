"""Regression tests for the Lanelet2 subtype -> OpenDRIVE LaneType mapping.

These tests exercise ``Lane.construct_from_lanelet`` directly with minimal
in-memory lanelets (no ``.osm`` round-trip, no MGRS projection) to lock in
the subtype -> ``LaneType`` mapping. They close the coverage gap left by
PR #427 (and PR #424 lineage), which extended the road-construction subtype
filter in ``opendrive/road.py`` beyond the original ``road``/``walkway`` set.

If a future refactor drops or changes a branch in the subtype mapping, these
tests will fail and force an explicit decision.
"""

import contextlib
import io

import lanelet2
import pytest

from autoware_lanelet2_to_opendrive.config import COORDINATE_OFFSET
from autoware_lanelet2_to_opendrive.conversion_config import (
    ConversionConfig,
    OriginSpec,
    ParamPoly3Config,
)
from autoware_lanelet2_to_opendrive.main import convert_lanelet2_to_opendrive
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
        ("pedestrian_lane", LaneType.SIDEWALK),
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

    lane = Lane.construct_from_lanelet(lanelet_map, lanelet, lane_id=-1)

    assert lane.lane_type is expected_type


def test_highway_subtype_maps_to_driving() -> None:
    """Dedicated regression guard for the ``highway`` mapping.

    Prior to PR #427 (and PR #424 lineage) the subtype filter in
    ``opendrive/road.py`` accepted only ``road`` and ``walkway``; this PR
    extends it to include ``highway``. No test previously exercised that
    branch end-to-end, so this test fills the gap.
    """
    lanelet_map, lanelet = _make_straight_lanelet("highway")

    lane = Lane.construct_from_lanelet(lanelet_map, lanelet, lane_id=-1)

    assert lane.lane_type is LaneType.DRIVING


def test_road_shoulder_subtype_maps_to_shoulder() -> None:
    """``road_shoulder`` must produce ``LaneType.SHOULDER``.

    Prior to PR #427 ``road_shoulder`` was handled in
    ``Lane.construct_from_lanelet`` but was absent from the subtype filter
    in ``opendrive/road.py``, making the branch unreachable end-to-end.
    This PR extends the filter; this test locks the mapping in.
    """
    lanelet_map, lanelet = _make_straight_lanelet("road_shoulder")

    lane = Lane.construct_from_lanelet(lanelet_map, lanelet, lane_id=-1)

    assert lane.lane_type is LaneType.SHOULDER


def test_walkway_subtype_maps_to_sidewalk() -> None:
    """Confirmation test: walkway still maps to sidewalk (P0-1 behaviour)."""
    lanelet_map, lanelet = _make_straight_lanelet("walkway")

    lane = Lane.construct_from_lanelet(lanelet_map, lanelet, lane_id=-1)

    assert lane.lane_type is LaneType.SIDEWALK


def test_unknown_subtype_falls_back_to_driving() -> None:
    """An unrecognised subtype (e.g. ``ferry``) falls back to DRIVING."""
    lanelet_map, lanelet = _make_straight_lanelet("ferry")

    lane = Lane.construct_from_lanelet(lanelet_map, lanelet, lane_id=-1)

    assert lane.lane_type is LaneType.DRIVING


def test_missing_subtype_falls_back_to_driving() -> None:
    """A lanelet without any ``subtype`` attribute falls back to DRIVING."""
    lanelet_map, lanelet = _make_straight_lanelet(subtype=None)

    lane = Lane.construct_from_lanelet(lanelet_map, lanelet, lane_id=-1)

    assert lane.lane_type is LaneType.DRIVING


def _make_adjacent_subtype_map(
    adjacent_subtype: str,
) -> lanelet2.core.LaneletMap:
    """Build a minimal road lanelet with one adjacent subtype lanelet."""
    lanelet_map = lanelet2.core.LaneletMap()

    def make_points(y: float) -> list[lanelet2.core.Point3d]:
        return [
            lanelet2.core.Point3d(lanelet2.core.getId(), 0.0, y, 0.0),
            lanelet2.core.Point3d(lanelet2.core.getId(), 10.0, y, 0.0),
        ]

    road_left = lanelet2.core.LineString3d(lanelet2.core.getId(), make_points(0.0))
    shared = lanelet2.core.LineString3d(lanelet2.core.getId(), make_points(-3.5))
    adjacent_outer = lanelet2.core.LineString3d(
        lanelet2.core.getId(), make_points(-5.5)
    )

    road = lanelet2.core.Lanelet(lanelet2.core.getId(), road_left, shared)
    road.attributes["subtype"] = "road"
    adjacent = lanelet2.core.Lanelet(lanelet2.core.getId(), shared, adjacent_outer)
    adjacent.attributes["subtype"] = adjacent_subtype

    lanelet_map.add(road)
    lanelet_map.add(adjacent)
    return lanelet_map


def _convert_adjacent_subtype_map(adjacent_subtype: str):
    """Run the normal converter path for a minimal adjacent subtype map."""
    lanelet_map = _make_adjacent_subtype_map(adjacent_subtype)
    config = ConversionConfig(
        origin=OriginSpec(mgrs_code="54SUE"),
        parampoly3=ParamPoly3Config(enabled=False),
    )

    COORDINATE_OFFSET.reset()
    try:
        with (
            contextlib.redirect_stdout(io.StringIO()),
            contextlib.redirect_stderr(io.StringIO()),
        ):
            opendrive, *_ = convert_lanelet2_to_opendrive(lanelet_map, config)
        return opendrive.to_xml()
    finally:
        COORDINATE_OFFSET.reset()


def _lane_width_values(lane) -> list[float]:
    """Return constant width coefficients emitted for a lane."""
    return [float(width.get("a")) for width in lane.findall("width")]


def test_pedestrian_lane_reaches_e2e_conversion_as_sidewalk() -> None:
    """An adjacent pedestrian_lane must be emitted as an OpenDRIVE sidewalk."""
    root = _convert_adjacent_subtype_map("pedestrian_lane")

    driving_lanes = root.findall(".//lane[@type='driving']")
    sidewalk_lanes = root.findall(".//lane[@type='sidewalk']")

    assert len(driving_lanes) == 1, "road lanelet should remain a driving lane"
    assert sidewalk_lanes, "pedestrian_lane should produce lane[type='sidewalk']"


def test_bicycle_lane_reaches_e2e_conversion_as_biking_with_width() -> None:
    """An adjacent bicycle_lane must be emitted as biking with meaningful width."""
    root = _convert_adjacent_subtype_map("bicycle_lane")

    driving_lanes = root.findall(".//lane[@type='driving']")
    biking_lanes = root.findall(".//lane[@type='biking']")

    assert len(driving_lanes) == 1, "road lanelet should remain a driving lane"
    assert biking_lanes, "bicycle_lane should produce lane[type='biking']"

    widths = [width for lane in biking_lanes for width in _lane_width_values(lane)]
    assert widths, "biking lanes should emit OpenDRIVE lane width entries"
    assert all(1.5 <= width <= 2.5 for width in widths)
