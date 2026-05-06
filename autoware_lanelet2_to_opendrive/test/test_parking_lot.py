"""Unit tests for ``opendrive/parking.py`` (P2-1 Task 2).

These tests cover the pure-Python helpers (OBB derivation, attribute
filters, polygon area, ParkingSpaceObject XML rendering) plus the
``ParkingLot`` association logic using mocks.  End-to-end pipeline
tests using a real OSM fixture are covered separately in Task 3.
"""

from __future__ import annotations

import logging
import math
from typing import List, Tuple
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from autoware_lanelet2_to_opendrive.conversion_config import ParkingLotConfig
from autoware_lanelet2_to_opendrive.opendrive.enums import LaneType
from autoware_lanelet2_to_opendrive.opendrive.parking import (
    ParkingLot,
    ParkingSpaceObject,
    _build_synthetic_road,
    _compute_obb,
    _filter_parking_lot_areas,
    _filter_parking_space_linestrings,
    _min_distance_to_polygon,
    _point_in_polygon,
    _polygon_area,
)


# ---------------------------------------------------------------------------
# Mock helpers
# ---------------------------------------------------------------------------


def _make_attribute_map(values: dict) -> MagicMock:
    """Mock that mimics the parts of ``AttributeMap`` we use.

    Supports ``"key" in attrs`` and ``attrs["key"]`` only.
    """
    attrs = MagicMock()
    attrs.__contains__.side_effect = lambda key: key in values
    attrs.__getitem__.side_effect = lambda key: values[key]
    return attrs


def _make_mock_linestring(
    ls_id: int,
    points_2d: List[Tuple[float, float]],
    attributes: dict | None = None,
) -> MagicMock:
    """Build a minimal mock ``LineString3d`` with ``id`` + ``attributes``."""
    ls = MagicMock()
    ls.id = ls_id
    if attributes is None:
        attributes = {}
    ls.attributes = _make_attribute_map(attributes)
    # ``extract_points`` calls ``[[p.x, p.y] for p in boundary]`` so the
    # iterator must yield objects with ``x``/``y`` (and ``z``) members.
    pts = []
    for x, y in points_2d:
        pt = MagicMock()
        pt.x = float(x)
        pt.y = float(y)
        pt.z = 0.0
        pts.append(pt)

    def _iter():
        return iter(pts)

    ls.__iter__.side_effect = _iter
    ls.__len__.return_value = len(pts)
    return ls


def _make_mock_area(
    area_id: int,
    polygon: List[Tuple[float, float]],
    attributes: dict | None = None,
) -> MagicMock:
    """Build a minimal mock ``Area`` with one outer LineString."""
    area = MagicMock()
    area.id = area_id
    if attributes is None:
        attributes = {}
    area.attributes = _make_attribute_map(attributes)
    outer_ls = _make_mock_linestring(area_id * 100 + 1, polygon)
    # ``outerBound`` is a property in the lanelet2 Python binding (not a method),
    # so the mock must expose it as an attribute, not a callable return value.
    area.outerBound = [outer_ls]
    return area


# ---------------------------------------------------------------------------
# _compute_obb
# ---------------------------------------------------------------------------


def test_compute_obb_axis_aligned_rectangle_along_x():
    """A wide thin rectangle along x has long axis collinear with x."""
    pts = np.array(
        [
            [0.0, 0.0],
            [10.0, 0.0],
            [10.0, 1.0],
            [0.0, 1.0],
        ]
    )
    centre, long_axis, along, across = _compute_obb(pts)
    assert centre == pytest.approx(np.array([5.0, 0.5]))
    # Eigenvectors are sign-canonicalised so that ``long_axis[0] >= 0``;
    # for a rectangle along world-x this pins the result to exactly +x.
    assert long_axis[0] == pytest.approx(1.0, abs=1e-6)
    assert long_axis[1] == pytest.approx(0.0, abs=1e-6)
    assert along == pytest.approx(10.0)
    assert across == pytest.approx(1.0)


def test_compute_obb_axis_aligned_rectangle_along_y():
    """A tall thin rectangle along y has long axis collinear with y."""
    pts = np.array(
        [
            [0.0, 0.0],
            [1.0, 0.0],
            [1.0, 8.0],
            [0.0, 8.0],
        ]
    )
    _, long_axis, along, across = _compute_obb(pts)
    # When x is numerically zero, the canonicalisation prefers +y, so
    # the long axis is pinned to exactly (0, 1) rather than (0, -1).
    assert long_axis[0] == pytest.approx(0.0, abs=1e-6)
    assert long_axis[1] == pytest.approx(1.0, abs=1e-6)
    assert along == pytest.approx(8.0)
    assert across == pytest.approx(1.0)


def test_compute_obb_canonical_sign_invariant_under_vertex_reversal():
    """Reversing vertex order must not flip the long-axis sign."""
    pts = np.array(
        [
            [0.0, 0.0],
            [10.0, 0.0],
            [10.0, 1.0],
            [0.0, 1.0],
        ]
    )
    _, long_axis_a, _, _ = _compute_obb(pts)
    _, long_axis_b, _, _ = _compute_obb(pts[::-1])
    # Whichever sign numpy picks, the canonicalisation forces both
    # results onto the same side, so the synthetic-road frame is stable
    # regardless of vertex order.
    np.testing.assert_allclose(long_axis_a, long_axis_b, atol=1e-6)
    assert long_axis_a[0] >= -1e-6


def test_compute_obb_square_falls_back_to_world_x():
    """A perfect square has no preferred long axis; fall back to (1, 0)."""
    pts = np.array(
        [
            [0.0, 0.0],
            [4.0, 0.0],
            [4.0, 4.0],
            [0.0, 4.0],
        ]
    )
    _, long_axis, along, across = _compute_obb(pts)
    np.testing.assert_allclose(long_axis, np.array([1.0, 0.0]))
    assert along == pytest.approx(4.0)
    assert across == pytest.approx(4.0)


def test_compute_obb_axes_are_orthogonal_for_sliver():
    """Sliver-thin rectangles still have orthogonal long/short axes."""
    pts = np.array(
        [
            [0.0, 0.0],
            [20.0, 0.001],
            [20.0, 0.002],
            [0.0, 0.001],
        ]
    )
    _, long_axis, along, across = _compute_obb(pts)
    # short axis is constructed by 90° rotation, so by definition
    # orthogonal — check the long axis is unit length.
    assert np.linalg.norm(long_axis) == pytest.approx(1.0, abs=1e-6)
    # along should be much greater than across for a sliver
    assert along > across * 100


# ---------------------------------------------------------------------------
# _polygon_area
# ---------------------------------------------------------------------------


# ---------------------------------------------------------------------------
# _point_in_polygon / _min_distance_to_polygon
# ---------------------------------------------------------------------------


def test_point_in_polygon_inside_outside():
    square = np.array([[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]])
    assert _point_in_polygon(np.array([5.0, 5.0]), square) is True
    assert _point_in_polygon(np.array([15.0, 5.0]), square) is False
    assert _point_in_polygon(np.array([-1.0, 5.0]), square) is False


def test_min_distance_to_polygon_returns_zero_when_inside():
    """Centroid inside the polygon must yield distance 0, not the
    distance to the nearest *vertex* (which used to break association
    on large lots whose interior centroids sat tens of metres from the
    nearest corner)."""
    big_square = np.array([[0.0, 0.0], [100.0, 0.0], [100.0, 100.0], [0.0, 100.0]])
    centroid = np.array([50.0, 50.0])
    assert _min_distance_to_polygon(centroid, big_square) == pytest.approx(0.0)


def test_min_distance_to_polygon_uses_edge_distance_outside():
    """Outside-the-polygon distance is the perpendicular distance to
    the nearest edge, not the distance to the nearest vertex."""
    square = np.array([[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]])
    # Point sitting 5 m above the middle of the top edge: the nearest
    # vertex is at (0, 10) or (10, 10) — sqrt(5^2 + 5^2) ≈ 7.07 m away —
    # but the nearest *edge* point is (5, 10), exactly 5 m away.
    point = np.array([5.0, 15.0])
    assert _min_distance_to_polygon(point, square) == pytest.approx(5.0)


def test_construct_all_from_lanelet_map_centroid_inside_large_lot():
    """A stall centroid inside a 100m × 100m lot must associate with
    that lot even though every vertex is well past the 30 m default
    threshold (regression for the vertex-only distance bug)."""
    lot = _make_mock_area(
        1,
        [(0.0, 0.0), (100.0, 0.0), (100.0, 100.0), (0.0, 100.0)],
        attributes={"type": "parking_lot"},
    )
    inner_stall = _make_mock_linestring(
        100,
        [(50.0, 50.0), (53.0, 50.0)],
        attributes={"type": "parking_space"},
    )
    lanelet_map = MagicMock()
    lanelet_map.areaLayer = [lot]
    lanelet_map.lineStringLayer = [inner_stall]
    result = ParkingLot.construct_all_from_lanelet_map(lanelet_map, ParkingLotConfig())
    assert len(result) == 1
    assert inner_stall in result[0].stalls


def test_polygon_area_unit_square():
    pts = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]])
    assert _polygon_area(pts) == pytest.approx(1.0)


def test_polygon_area_handles_degenerate_input():
    assert _polygon_area(np.array([[0.0, 0.0]])) == 0.0
    assert _polygon_area(np.empty((0, 2))) == 0.0


# ---------------------------------------------------------------------------
# Filters – type / subtype aliases
# ---------------------------------------------------------------------------


def test_filter_parking_lot_areas_accepts_type_alias():
    a = _make_mock_area(1, [(0, 0), (1, 0), (1, 1)], attributes={"type": "parking_lot"})
    b = _make_mock_area(
        2, [(0, 0), (1, 0), (1, 1)], attributes={"subtype": "parking_lot"}
    )
    other = _make_mock_area(3, [(0, 0), (1, 0), (1, 1)], attributes={"type": "other"})
    result = _filter_parking_lot_areas([a, b, other])
    assert a in result
    assert b in result
    assert other not in result


def test_filter_parking_space_linestrings_accepts_subtype_alias():
    a = _make_mock_linestring(
        1, [(0.0, 0.0), (5.0, 0.0)], attributes={"type": "parking_space"}
    )
    b = _make_mock_linestring(
        2, [(0.0, 0.0), (5.0, 0.0)], attributes={"subtype": "parking_space"}
    )
    other = _make_mock_linestring(
        3, [(0.0, 0.0), (5.0, 0.0)], attributes={"type": "stop_line"}
    )
    result = _filter_parking_space_linestrings([a, b, other])
    assert a in result
    assert b in result
    assert other not in result


# ---------------------------------------------------------------------------
# ParkingSpaceObject – dataclass + XML serialisation
# ---------------------------------------------------------------------------


def test_parking_space_object_to_xml_attributes():
    obj = ParkingSpaceObject(
        id=42,
        name="parking_space_42",
        s=10.0,
        t=-2.5,
        z_offset=0.05,
        hdg=math.pi / 2,
        length=5.0,
        width=2.5,
    )
    elem = obj.to_xml()
    assert elem.tag == "object"
    assert elem.get("type") == "parkingSpace"
    assert elem.get("id") == "42"
    assert elem.get("name") == "parking_space_42"
    assert float(elem.get("s")) == pytest.approx(10.0)
    assert float(elem.get("t")) == pytest.approx(-2.5)
    assert float(elem.get("zOffset")) == pytest.approx(0.05)
    assert float(elem.get("hdg")) == pytest.approx(math.pi / 2)
    assert float(elem.get("width")) == pytest.approx(2.5)
    assert float(elem.get("length")) == pytest.approx(5.0)
    assert elem.get("orientation") == "none"


# ---------------------------------------------------------------------------
# ParkingLot.construct_all_from_lanelet_map – stall ↔ area assignment
# ---------------------------------------------------------------------------


def test_construct_all_from_lanelet_map_orphan_stall_filtered():
    """A stall outside threshold of every parking lot is dropped."""
    # Lot at the origin with extent [-5, 5] × [-5, 5].
    lot = _make_mock_area(
        1,
        [(-5.0, -5.0), (5.0, -5.0), (5.0, 5.0), (-5.0, 5.0)],
        attributes={"type": "parking_lot"},
    )
    nearby_stall = _make_mock_linestring(
        100,
        [(0.0, 0.0), (3.0, 0.0)],
        attributes={"type": "parking_space"},
    )
    # Far away (1000m) — well outside the default 30 m threshold.
    far_stall = _make_mock_linestring(
        101,
        [(1000.0, 1000.0), (1003.0, 1000.0)],
        attributes={"type": "parking_space"},
    )

    lanelet_map = MagicMock()
    lanelet_map.areaLayer = [lot]
    lanelet_map.lineStringLayer = [nearby_stall, far_stall]

    result = ParkingLot.construct_all_from_lanelet_map(lanelet_map, ParkingLotConfig())
    assert len(result) == 1
    assert nearby_stall in result[0].stalls
    assert far_stall not in result[0].stalls


def test_construct_all_from_lanelet_map_disabled_returns_empty():
    """With ``enabled=False`` no parking lots are returned."""
    lot = _make_mock_area(
        1, [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)], attributes={"type": "parking_lot"}
    )
    lanelet_map = MagicMock()
    lanelet_map.areaLayer = [lot]
    lanelet_map.lineStringLayer = []
    result = ParkingLot.construct_all_from_lanelet_map(
        lanelet_map, ParkingLotConfig(enabled=False)
    )
    assert result == []


def test_construct_all_from_lanelet_map_no_areas():
    """Empty area layer is a silent no-op."""
    lanelet_map = MagicMock()
    lanelet_map.areaLayer = []
    lanelet_map.lineStringLayer = []
    result = ParkingLot.construct_all_from_lanelet_map(lanelet_map, ParkingLotConfig())
    assert result == []


# ---------------------------------------------------------------------------
# ParkingLot.to_road_and_objects – synthetic road shape
# ---------------------------------------------------------------------------


def test_to_road_and_objects_happy_path_lane_widths_and_length():
    """A 10×4 m parking lot yields a 10 m straight road with two PARKING lanes."""
    polygon = [
        (-5.0, -2.0),
        (5.0, -2.0),
        (5.0, 2.0),
        (-5.0, 2.0),
    ]
    lot_area = _make_mock_area(1, polygon, attributes={"type": "parking_lot"})
    lot = ParkingLot(area=lot_area, stalls=[])

    road, objects = lot.to_road_and_objects(road_id=42, config=ParkingLotConfig())
    assert road is not None
    assert objects == []
    assert road.id == 42
    assert road.junction == -1
    assert road.length == pytest.approx(10.0)
    assert road.plan_view is not None
    assert len(road.plan_view.geometries) == 1
    geom = road.plan_view.geometries[0]
    assert geom.length == pytest.approx(10.0)

    assert road.lanes is not None
    assert len(road.lanes.lane_sections) == 1
    lane_section = road.lanes.lane_sections[0]
    # Right lane id=-1 and left lane id=+1, each width = across_length / 2
    assert -1 in lane_section.right_lanes
    assert 1 in lane_section.left_lanes
    right = lane_section.right_lanes[-1]
    left = lane_section.left_lanes[1]
    assert right.widths[0].a == pytest.approx(2.0)  # 4 m / 2
    assert left.widths[0].a == pytest.approx(2.0)


def test_to_road_and_objects_skips_degenerate_area(caplog):
    """An area below ``min_area_polygon_m2`` returns ``(None, [])`` + warning."""
    polygon = [(0.0, 0.0), (0.1, 0.0), (0.1, 0.1), (0.0, 0.1)]
    lot_area = _make_mock_area(2, polygon, attributes={"type": "parking_lot"})
    lot = ParkingLot(area=lot_area, stalls=[])

    config = ParkingLotConfig(min_area_polygon_m2=1.0)
    with caplog.at_level(logging.WARNING):
        road, objects = lot.to_road_and_objects(road_id=0, config=config)
    assert road is None
    assert objects == []
    assert any("polygon area" in rec.message for rec in caplog.records)


# ---------------------------------------------------------------------------
# ParkingSpaceObject.construct_from_stall_linestring – relative heading
# ---------------------------------------------------------------------------


def test_construct_from_stall_linestring_relative_heading():
    """Stall heading is reported relative to the road tangent."""
    # Build a synthetic road via _build_synthetic_road so we have a
    # real PlanView that ``_project_point_onto_road`` can sample.
    polygon = np.array(
        [
            [-5.0, -2.0],
            [5.0, -2.0],
            [5.0, 2.0],
            [-5.0, 2.0],
        ]
    )
    lot_area = _make_mock_area(
        7, [(x, y) for x, y in polygon], attributes={"type": "parking_lot"}
    )
    lot = ParkingLot(area=lot_area, stalls=[])
    road = _build_synthetic_road(
        lot, road_id=0, config=ParkingLotConfig(), polygon_xy=polygon
    )
    assert road is not None

    # A perpendicular stall (along world-y at the centre) should
    # come out with hdg ≈ ±π/2 relative to the road's world-x tangent.
    stall = _make_mock_linestring(50, [(0.0, -1.0), (0.0, 1.0)])

    # The road's reference line runs along world-x — patch
    # extract_points to return our stall coordinates without involving
    # the real lanelet2 module.
    pts_2d = np.array([[0.0, -1.0], [0.0, 1.0]])
    with patch(
        "autoware_lanelet2_to_opendrive.opendrive.parking.extract_points",
        return_value=pts_2d,
    ):
        obj = ParkingSpaceObject.construct_from_stall_linestring(
            stall=stall,
            road=road,
            object_id=50,
            default_width=2.5,
        )
    assert obj is not None
    # Stall direction is +y in world frame; road tangent is +x → hdg ≈ +π/2
    assert abs(obj.hdg) == pytest.approx(math.pi / 2, abs=1e-3)
    assert obj.length == pytest.approx(2.0)
    assert obj.width == pytest.approx(2.5)


# ---------------------------------------------------------------------------
# LaneSection.get_all_lanes works on synthetic parking-lot roads
# ---------------------------------------------------------------------------


def test_synthetic_road_lane_section_get_all_lanes():
    """``get_all_lanes`` must unwrap the bare-Lane centre on parking roads.

    The lanelet-driven path stores a ``ReferenceLine`` (which wraps a
    ``Lane`` in ``_lane``) as the centre slot, while the parking-lot
    synthetic road stores a bare ``Lane`` directly.  ``get_all_lanes``
    must handle both forms without raising ``AttributeError``.
    """
    polygon = np.array(
        [
            [-5.0, -2.0],
            [5.0, -2.0],
            [5.0, 2.0],
            [-5.0, 2.0],
        ]
    )
    lot_area = _make_mock_area(
        9, [(x, y) for x, y in polygon], attributes={"type": "parking_lot"}
    )
    lot = ParkingLot(area=lot_area, stalls=[])
    road = _build_synthetic_road(
        lot, road_id=0, config=ParkingLotConfig(), polygon_xy=polygon
    )
    assert road is not None

    lane_section = road.lanes.lane_sections[0]
    all_lanes = lane_section.get_all_lanes()

    # Expect exactly three lanes: left +1, centre 0, right -1.
    assert len(all_lanes) == 3
    ids = {lane.lane_id for lane in all_lanes}
    assert ids == {-1, 0, 1}

    by_id = {lane.lane_id: lane for lane in all_lanes}
    assert by_id[0].lane_type == LaneType.NONE
    assert by_id[-1].lane_type == LaneType.PARKING
    assert by_id[1].lane_type == LaneType.PARKING


# ---------------------------------------------------------------------------
# Integration tests (end-to-end)
# ---------------------------------------------------------------------------
#
# The unit tests above exercise the helpers directly with mocks.  The
# tests below run the full ``convert`` CLI on a small OSM fixture and
# inspect the emitted OpenDRIVE XML.

import subprocess  # noqa: E402
from pathlib import Path  # noqa: E402

import lxml.etree as ET  # noqa: E402

_FIXTURE_PARKING_LOT = (
    Path(__file__).parent / "data" / "parking_lot_mini.osm"
).resolve()


def _run_convert_parking_lot(tmp_path: Path) -> Path:
    """Run the ``convert`` CLI on the parking-lot fixture and return path."""
    out = tmp_path / "parking_lot_mini.xodr"
    subprocess.run(
        [
            "uv",
            "run",
            "convert",
            "map=example_mgrs_offset",
            "target=carla",
            f"input_map_path={_FIXTURE_PARKING_LOT}",
            f"output_map_path={out}",
        ],
        check=True,
    )
    return out


def test_parking_lot_mini_emits_parking_road(tmp_path: Path) -> None:
    """The fixture must produce one parking road with two parking lanes
    and four parkingSpace objects.

    Asserts:
      * At least one ``<road>`` containing exactly two
        ``<lane type="parking">`` elements (id=-1 and id=+1).
      * That road has four ``<object type="parkingSpace">`` children.
      * Each stall has 0 <= s <= road length and |t| <= half across-axis.
      * Each stall ``hdg`` is approximately ±π/2 from the road
        reference-line tangent (the stalls are perpendicular to the
        long axis of the lot).
    """
    out = _run_convert_parking_lot(tmp_path)
    root = ET.parse(out).getroot()

    # Find roads that contain at least one lane[type='parking'].
    candidate_roads = [
        road
        for road in root.findall(".//road")
        if road.findall(".//lane[@type='parking']")
    ]
    assert candidate_roads, (
        "parking_lot_mini fixture should produce at least one road "
        "with lane[type='parking']"
    )

    # Pick the first parking road and verify its layout.
    parking_road = candidate_roads[0]
    parking_lanes = parking_road.findall(".//lane[@type='parking']")
    parking_lane_ids = sorted(int(lane.get("id")) for lane in parking_lanes)
    assert parking_lane_ids == [-1, 1], (
        "parking road should have two parking lanes (id=-1, id=+1); "
        f"got {parking_lane_ids}"
    )

    # Four <object type='parkingSpace'> children attached to this road.
    parking_objects = parking_road.findall(".//object[@type='parkingSpace']")
    assert len(parking_objects) == 4, (
        f"expected 4 parkingSpace objects on the parking road, "
        f"got {len(parking_objects)}"
    )

    road_length = float(parking_road.get("length"))
    # Half across-axis = max half-width of the parking lanes.
    half_widths = [
        float(w.get("a")) for lane in parking_lanes for w in lane.findall(".//width")
    ]
    half_across = max(half_widths) if half_widths else 0.0

    # Each lane's width (the 'a' coefficient of its <width> element) must
    # equal half the OBB across-axis length within 10 cm (spec §9.3).
    # The fixture's nodes carry both lat/lon and local_x/local_y; the
    # MGRS projection of the lat/lon yields an across-axis of ~11.1 m
    # (rather than the 10 m suggested by the round-number local_y tags),
    # so half-across is ~5.55 m. Tolerance still tracks the spec's 10 cm.
    expected_half_width = 5.55
    for lane in parking_lanes:
        width_elements = lane.findall(".//width")
        assert width_elements, f"lane id={lane.get('id')} has no <width> child elements"
        for width_elem in width_elements:
            width_a = float(width_elem.get("a"))
            assert width_a == pytest.approx(expected_half_width, abs=0.1), (
                f"lane id={lane.get('id')} width.a={width_a} "
                f"does not match expected half-width "
                f"{expected_half_width} m (±0.1 m)"
            )

    for obj in parking_objects:
        s = float(obj.get("s"))
        t = float(obj.get("t"))
        hdg = float(obj.get("hdg"))
        assert (
            0.0 <= s <= road_length
        ), f"parking space s={s} out of road [0, {road_length}]"
        # ``t`` is signed; allow a small tolerance for the half-width
        # (stalls are at lot centre but the centroid sampling is exact
        # for a 2-node stall).
        assert (
            abs(t) <= half_across + 1e-6
        ), f"parking space t={t} exceeds half across-axis {half_across}"
        # Stalls run perpendicular to the lot long axis, so the
        # relative heading should be ~±π/2.  Allow ±0.1 rad slack.
        deviation = abs(abs(hdg) - math.pi / 2.0)
        assert deviation < 0.1, (
            f"parking space hdg={hdg} is not approximately perpendicular "
            f"(±π/2) to the road reference line"
        )
