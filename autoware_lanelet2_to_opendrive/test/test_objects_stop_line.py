"""Tests for StopLineObject and related functions in opendrive/objects.py."""

from __future__ import annotations

import math
from typing import List
from unittest.mock import MagicMock

import numpy as np
import pytest

from autoware_lanelet2_to_opendrive.opendrive.objects import (
    StopLineObject,
    find_nearest_road_for_linestring,
)


# ---------------------------------------------------------------------------
# Helpers to build minimal mock objects
# ---------------------------------------------------------------------------


def _make_mock_road(road_id: int, wx: float, wy: float, s: float = 0.0) -> MagicMock:
    """Build a minimal mock Road that _sample_road_points can iterate over."""
    from autoware_lanelet2_to_opendrive.opendrive.geometry import ParamPoly3

    # Build a real ParamPoly3 geometry so _sample_road_points works properly
    geom = ParamPoly3(
        s=s,
        x=wx,
        y=wy,
        hdg=0.0,
        length=10.0,
        aU=0.0,
        bU=1.0,
        cU=0.0,
        dU=0.0,
        aV=0.0,
        bV=0.0,
        cV=0.0,
        dV=0.0,
    )

    plan_view = MagicMock()
    plan_view.geometries = [geom]

    road = MagicMock()
    road.id = road_id
    road.plan_view = plan_view
    road.get_elevation_at_s.return_value = 0.0
    return road


def _make_mock_linestring(
    ls_id: int,
    points_2d: List[tuple],
    points_3d: List[tuple] | None = None,
) -> MagicMock:
    """Build a minimal mock LineString3d."""
    if points_3d is None:
        points_3d = [(x, y, 0.0) for x, y in points_2d]

    # Create mock point objects
    mock_points_2d = []
    for x, y in points_2d:
        pt = MagicMock()
        pt.x = x
        pt.y = y
        mock_points_2d.append(pt)

    mock_points_3d = []
    for x, y, z in points_3d:
        pt = MagicMock()
        pt.x = x
        pt.y = y
        pt.z = z
        mock_points_3d.append(pt)

    ls = MagicMock()
    ls.id = ls_id
    ls.__iter__ = MagicMock(return_value=iter(mock_points_3d))
    ls.__len__ = MagicMock(return_value=len(mock_points_3d))
    return ls


# ---------------------------------------------------------------------------
# Unit tests – StopLineObject dataclass
# ---------------------------------------------------------------------------


def test_stop_line_object_creation():
    """Test basic dataclass creation with default optional fields."""
    obj = StopLineObject(
        id=100,
        name="stop_line_100",
        s=10.0,
        t=2.5,
        z_offset=0.0,
        hdg=math.pi / 2,
    )
    assert obj.id == 100
    assert obj.name == "stop_line_100"
    assert obj.s == 10.0
    assert obj.t == 2.5
    assert obj.z_offset == 0.0
    assert obj.hdg == pytest.approx(math.pi / 2)
    assert obj.pitch == 0.0
    assert obj.roll == 0.0
    assert obj.orientation == "none"
    assert obj.width == 0.0
    assert obj.length == 0.0


def test_stop_line_object_to_xml():
    """Test that to_xml() returns an <object type="stopLine"> element."""
    obj = StopLineObject(
        id=42,
        name="stop_line_42",
        s=5.0,
        t=1.0,
        z_offset=0.0,
        hdg=1.5707963,
        width=3.5,
        length=0.0,
    )
    elem = obj.to_xml()
    assert elem.tag == "object"
    assert elem.get("type") == "stopLine"


def test_stop_line_object_xml_attributes():
    """Test that XML element contains correct attribute values."""
    obj = StopLineObject(
        id=99,
        name="stop_line_99",
        s=20.0,
        t=-1.5,
        z_offset=0.05,
        hdg=math.pi / 2,
        pitch=0.0,
        roll=0.0,
        orientation="none",
        width=4.0,
        length=0.0,
    )
    elem = obj.to_xml()
    assert elem.get("id") == "99"
    assert elem.get("name") == "stop_line_99"
    assert float(elem.get("s")) == pytest.approx(20.0)
    assert float(elem.get("t")) == pytest.approx(-1.5)
    assert float(elem.get("zOffset")) == pytest.approx(0.05)
    assert float(elem.get("width")) == pytest.approx(4.0)
    assert float(elem.get("length")) == pytest.approx(0.0)
    assert elem.get("orientation") == "none"


# ---------------------------------------------------------------------------
# Unit tests – construct_from_linestring
# ---------------------------------------------------------------------------


def test_construct_from_linestring_basic():
    """Test that a valid 2-point linestring produces a StopLineObject."""
    road = _make_mock_road(road_id=0, wx=0.0, wy=0.0)

    # Patch extract_points inside objects module
    from unittest.mock import patch

    pts_2d = np.array([[0.0, -2.0], [0.0, 2.0]])
    pts_3d = np.array([[0.0, -2.0, 0.1], [0.0, 2.0, 0.1]])

    ls = MagicMock()
    ls.id = 1001

    with patch(
        "autoware_lanelet2_to_opendrive.opendrive.objects.extract_points"
    ) as mock_extract:
        mock_extract.side_effect = lambda linestring, dimensions: (
            pts_2d if dimensions == 2 else pts_3d
        )

        result = StopLineObject.construct_from_linestring(
            linestring=ls, road=road, object_id=ls.id
        )

    assert result is not None
    assert isinstance(result, StopLineObject)
    assert result.id == 1001
    assert result.name == "stop_line_1001"
    # Length should equal the distance between first and last 2D point (span along u-axis/heading)
    assert result.length == pytest.approx(4.0, rel=0.01)
    # Width defaults to 0.1 (painted thickness in v-direction)
    assert result.width == pytest.approx(0.1)


def test_construct_from_linestring_insufficient_points():
    """Test that a linestring with only 1 point returns None."""
    road = _make_mock_road(road_id=0, wx=0.0, wy=0.0)

    ls = MagicMock()
    ls.id = 2002

    from unittest.mock import patch

    pts_2d = np.array([[0.0, 0.0]])  # only 1 point

    with patch(
        "autoware_lanelet2_to_opendrive.opendrive.objects.extract_points"
    ) as mock_extract:
        mock_extract.return_value = pts_2d
        result = StopLineObject.construct_from_linestring(
            linestring=ls, road=road, object_id=ls.id
        )

    assert result is None


def test_construct_from_linestring_returns_none_on_projection_failure():
    """Test that projection failure causes None to be returned."""
    road = MagicMock()
    road.id = 0
    road.plan_view = None  # No plan_view means _sample_road_points returns []

    ls = MagicMock()
    ls.id = 3003

    from unittest.mock import patch

    pts_2d = np.array([[1.0, -2.0], [1.0, 2.0]])
    pts_3d = np.array([[1.0, -2.0, 0.0], [1.0, 2.0, 0.0]])

    with patch(
        "autoware_lanelet2_to_opendrive.opendrive.objects.extract_points"
    ) as mock_extract:
        mock_extract.side_effect = lambda linestring, dimensions: (
            pts_2d if dimensions == 2 else pts_3d
        )
        result = StopLineObject.construct_from_linestring(
            linestring=ls, road=road, object_id=ls.id
        )

    assert result is None


# ---------------------------------------------------------------------------
# Unit tests – find_nearest_road_for_linestring
# ---------------------------------------------------------------------------


def test_find_nearest_road_for_linestring():
    """Test that the nearest road is returned for a linestring centroid."""
    road_near = _make_mock_road(road_id=0, wx=0.0, wy=0.0)
    road_far = _make_mock_road(road_id=1, wx=100.0, wy=100.0)

    ls = MagicMock()
    ls.id = 4004

    from unittest.mock import patch

    pts_2d = np.array([[-0.5, 0.0], [0.5, 0.0]])

    with patch(
        "autoware_lanelet2_to_opendrive.opendrive.objects.extract_points"
    ) as mock_extract:
        mock_extract.return_value = pts_2d
        result = find_nearest_road_for_linestring(ls, [road_near, road_far])

    assert result is not None
    assert result.id == road_near.id


def test_find_nearest_road_for_linestring_beyond_threshold():
    """Test that None is returned when all roads are beyond the threshold."""
    road_far = _make_mock_road(road_id=0, wx=200.0, wy=200.0)

    ls = MagicMock()
    ls.id = 5005

    from unittest.mock import patch

    pts_2d = np.array([[0.0, 0.0], [1.0, 0.0]])  # centroid near origin

    with patch(
        "autoware_lanelet2_to_opendrive.opendrive.objects.extract_points"
    ) as mock_extract:
        mock_extract.return_value = pts_2d
        result = find_nearest_road_for_linestring(ls, [road_far], threshold_m=50.0)

    assert result is None


# ---------------------------------------------------------------------------
# Unit tests – CARLA Stencil_STOP format
# ---------------------------------------------------------------------------


def test_to_xml_carla_format():
    """Test that carla_format=True produces CARLA Stencil_STOP XML attributes."""
    obj = StopLineObject(
        id=77,
        name="stop_line_77",
        s=15.0,
        t=0.5,
        z_offset=0.03,
        hdg=math.pi / 2,
        width=2.0,
        length=3.5,
        carla_format=True,
    )
    elem = obj.to_xml()

    # CARLA-specific attributes
    assert elem.get("type") == "-1"
    assert elem.get("name") == "Stencil_STOP"
    assert elem.get("orientation") == "-"
    assert float(elem.get("zOffset")) == pytest.approx(0.0)

    # Geometric attributes must remain unchanged
    assert float(elem.get("s")) == pytest.approx(15.0)
    assert float(elem.get("t")) == pytest.approx(0.5)
    assert float(elem.get("width")) == pytest.approx(2.0)
    assert float(elem.get("length")) == pytest.approx(3.5)
    assert elem.get("id") == "77"


def test_to_xml_carla_vs_default():
    """Test that carla_format=False (default) leaves standard OpenDRIVE output unchanged."""
    obj = StopLineObject(
        id=88,
        name="stop_line_88",
        s=5.0,
        t=1.0,
        z_offset=0.05,
        hdg=0.0,
        width=0.1,
        length=4.0,
        carla_format=False,
    )
    elem = obj.to_xml()

    assert elem.get("type") == "stopLine"
    assert elem.get("name") == "stop_line_88"
    assert elem.get("orientation") == "none"
    assert float(elem.get("zOffset")) == pytest.approx(0.05)


def test_construct_from_linestring_carla_format():
    """Test that construct_from_linestring propagates carla_format correctly."""
    road = _make_mock_road(road_id=0, wx=0.0, wy=0.0)

    from unittest.mock import patch

    pts_2d = np.array([[0.0, -2.0], [0.0, 2.0]])
    pts_3d = np.array([[0.0, -2.0, 0.1], [0.0, 2.0, 0.1]])

    ls = MagicMock()
    ls.id = 6006

    with patch(
        "autoware_lanelet2_to_opendrive.opendrive.objects.extract_points"
    ) as mock_extract:
        mock_extract.side_effect = lambda linestring, dimensions: (
            pts_2d if dimensions == 2 else pts_3d
        )
        result = StopLineObject.construct_from_linestring(
            linestring=ls,
            road=road,
            object_id=ls.id,
            width=2.0,
            carla_format=True,
        )

    assert result is not None
    assert result.carla_format is True

    elem = result.to_xml()
    assert elem.get("type") == "-1"
    assert elem.get("name") == "Stencil_STOP"
    assert elem.get("orientation") == "-"
    assert float(elem.get("zOffset")) == pytest.approx(0.0)
    assert float(elem.get("width")) == pytest.approx(2.0)


# ---------------------------------------------------------------------------
# Integration test – real map
# ---------------------------------------------------------------------------


def test_stop_lines_extracted_from_real_map(lanelet_map):
    """Test that stop_line linestrings exist in the test map."""
    stop_lines = [
        ls
        for ls in lanelet_map.lineStringLayer
        if "type" in ls.attributes and ls.attributes["type"] == "stop_line"
    ]
    # The nishisinjyuku.osm test map should have a significant number of stop lines
    assert len(stop_lines) > 0, "Expected stop_line linestrings in the test map"


def test_stop_sign_stop_line_ids_from_real_map(lanelet_map):
    """Test that _build_stop_sign_stop_line_ids finds stop sign stop lines."""
    from autoware_lanelet2_to_opendrive.conversion_config import ConversionConfig
    from autoware_lanelet2_to_opendrive.main import _Lanelet2ToOpenDRIVEConverter

    converter = _Lanelet2ToOpenDRIVEConverter(lanelet_map, ConversionConfig())
    stop_sign_sl_ids = converter._build_stop_sign_stop_line_ids()

    # The nishishinjuku.osm test map has 4 stop sign regulatory elements
    # with ref_line stop lines: ways 1784, 1401, 301355, 3002425
    expected_ids = {1784, 1401, 301355, 3002425}
    assert (
        stop_sign_sl_ids == expected_ids
    ), f"Expected stop sign stop line IDs {expected_ids}, got {stop_sign_sl_ids}"


# ---------------------------------------------------------------------------
# Integration tests – road marking stop lines (real map)
# ---------------------------------------------------------------------------


def test_road_marking_stop_line_ids_from_real_map(lanelet_map):
    """Test that _build_road_marking_stop_line_ids finds road marking stop lines."""
    from autoware_lanelet2_to_opendrive.conversion_config import ConversionConfig
    from autoware_lanelet2_to_opendrive.main import _Lanelet2ToOpenDRIVEConverter

    converter = _Lanelet2ToOpenDRIVEConverter(lanelet_map, ConversionConfig())
    rm_sl_ids = converter._build_road_marking_stop_line_ids()

    # The nishishinjuku.osm test map has 32 road_marking regulatory elements
    # referring to 30 unique stop_line linestrings
    assert (
        len(rm_sl_ids) == 30
    ), f"Expected 30 road marking stop line IDs, got {len(rm_sl_ids)}"


def test_road_marking_no_overlap_with_stop_sign(lanelet_map):
    """Test that road marking stop line IDs do not overlap with stop sign IDs."""
    from autoware_lanelet2_to_opendrive.conversion_config import ConversionConfig
    from autoware_lanelet2_to_opendrive.main import _Lanelet2ToOpenDRIVEConverter

    converter = _Lanelet2ToOpenDRIVEConverter(lanelet_map, ConversionConfig())
    rm_sl_ids = converter._build_road_marking_stop_line_ids()
    ss_sl_ids = converter._build_stop_sign_stop_line_ids()

    overlap = rm_sl_ids & ss_sl_ids
    assert len(overlap) == 0, (
        f"Road marking and stop sign stop line IDs should not overlap, "
        f"but found overlap: {overlap}"
    )


# ---------------------------------------------------------------------------
# Arc geometry sampling – _sample_road_points / _project_point_onto_road (#504)
# ---------------------------------------------------------------------------


def _make_arc_road(curvature: float, length: float, s: float = 0.0) -> MagicMock:
    """Build a minimal mock Road whose plan view is a single <arc> segment."""
    from autoware_lanelet2_to_opendrive.opendrive.geometry import Arc

    arc = Arc(s=s, x=100.0, y=200.0, hdg=0.3, length=length, curvature=curvature)
    plan_view = MagicMock()
    plan_view.geometries = [arc]
    road = MagicMock()
    road.id = 348
    road.plan_view = plan_view
    return road


def test_sample_road_points_follows_arc_geometry():
    """_sample_road_points must sample <arc> along the curve, not the tangent.

    Regression for #504: sampling an arc as a straight tangent line skews the
    projected (s, t) of stop lines placed on curved roads.
    """
    from autoware_lanelet2_to_opendrive.opendrive.geometry import (
        evaluate_plan_view_world,
    )
    from autoware_lanelet2_to_opendrive.opendrive.objects import _sample_road_points

    curvature, length, s0 = 0.04, 20.0, 5.0  # radius 25 m, ~46 deg of turn
    road = _make_arc_road(curvature, length, s=s0)
    samples = _sample_road_points(road)
    assert samples, "expected non-empty samples for a positive-length arc"

    geom = road.plan_view.geometries[0]
    max_tangent_gap = 0.0
    for wx, wy, s, hdg in samples:
        p = s - s0
        exp_x, exp_y = evaluate_plan_view_world(
            geom.x, geom.y, geom.hdg, p, arc_curvature=curvature
        )
        assert (
            math.hypot(wx - exp_x, wy - exp_y) < 1e-6
        ), f"sample at s={s} is off the analytic arc"
        exp_hdg = geom.hdg + curvature * p
        assert abs(hdg - exp_hdg) < 1e-6, f"heading at s={s} not tangent to arc"
        # Gap between the true arc point and the straight tangent at the same p.
        tan_x = geom.x + p * math.cos(geom.hdg)
        tan_y = geom.y + p * math.sin(geom.hdg)
        max_tangent_gap = max(max_tangent_gap, math.hypot(exp_x - tan_x, exp_y - tan_y))

    # Sanity: the arc curves far enough from its tangent that a straight-line
    # approximation would be a real defect (the test is not vacuous).
    assert max_tangent_gap > 1.0


def test_project_point_onto_arc_road_recovers_s():
    """Projecting a point on an arc recovers its s with a near-zero offset."""
    from autoware_lanelet2_to_opendrive.opendrive.geometry import (
        evaluate_plan_view_world,
    )
    from autoware_lanelet2_to_opendrive.opendrive.objects import (
        _project_point_onto_road,
    )

    curvature, length, s0 = 0.04, 20.0, 5.0
    road = _make_arc_road(curvature, length, s=s0)
    geom = road.plan_view.geometries[0]

    # A point lying exactly on a sample of the arc (i = 7 of 10, p = 20*7/9).
    p_target = length * 7 / 9
    wx, wy = evaluate_plan_view_world(
        geom.x, geom.y, geom.hdg, p_target, arc_curvature=curvature
    )

    result = _project_point_onto_road(np.array([wx, wy]), road)
    assert result is not None
    s, t, _hdg = result
    assert abs(s - (s0 + p_target)) < 1e-6, f"recovered s={s}, expected {s0 + p_target}"
    assert abs(t) < 1e-6, f"point on the reference line should have t~0, got {t}"
