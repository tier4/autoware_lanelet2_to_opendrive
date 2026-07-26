"""Tests for StopLineObject and related functions in opendrive/objects.py."""

from __future__ import annotations

import math
from typing import List
from unittest.mock import MagicMock

import numpy as np
import pytest

from autoware_lanelet2_to_opendrive.opendrive.objects import (
    CornerLocal,
    StopLineObject,
    find_best_road_for_stop_line,
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
    road.length = 10.0
    road.plan_view = plan_view
    road.get_elevation_at_s.return_value = 0.0
    return road


def _make_mock_line_road(
    road_id: int,
    x: float,
    y: float,
    hdg: float,
    length: float,
) -> MagicMock:
    """Build a minimal straight Road for stop-line road selection tests."""
    from autoware_lanelet2_to_opendrive.opendrive.geometry import Line

    plan_view = MagicMock()
    plan_view.geometries = [Line(s=0.0, x=x, y=y, hdg=hdg, length=length)]

    road = MagicMock()
    road.id = road_id
    road.length = length
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


def _object_origin_world(
    obj: StopLineObject, road: MagicMock
) -> tuple[np.ndarray, float]:
    """Return object origin and absolute heading for simple test roads."""
    from autoware_lanelet2_to_opendrive.opendrive.objects import (
        _evaluate_geometry_at,
    )

    geom = road.plan_view.geometries[0]
    x, y, road_hdg = _evaluate_geometry_at(geom, obj.s - geom.s)
    normal = np.array([-math.sin(road_hdg), math.cos(road_hdg)])
    origin = np.array([x, y]) + obj.t * normal
    return origin, road_hdg + obj.hdg


def _outline_world_points(obj: StopLineObject, road: MagicMock) -> np.ndarray:
    origin, hdg = _object_origin_world(obj, road)
    tangent = np.array([math.cos(hdg), math.sin(hdg)])
    normal = np.array([-math.sin(hdg), math.cos(hdg)])
    return np.asarray(
        [origin + corner.u * tangent + corner.v * normal for corner in obj.corners]
    )


def _expected_stop_line_rectangle(
    p0: np.ndarray,
    p1: np.ndarray,
    width: float,
) -> np.ndarray:
    direction = p1 - p0
    direction = direction / np.linalg.norm(direction)
    normal = np.array([-direction[1], direction[0]])
    half_width = 0.5 * width
    return np.asarray(
        [
            p0 - half_width * normal,
            p1 - half_width * normal,
            p1 + half_width * normal,
            p0 + half_width * normal,
            p0 - half_width * normal,
        ]
    )


def _assert_outline_matches_source_rectangle(
    obj: StopLineObject,
    road: MagicMock,
    p0: np.ndarray,
    p1: np.ndarray,
    width: float,
) -> None:
    actual = _outline_world_points(obj, road)
    expected = _expected_stop_line_rectangle(p0, p1, width)
    assert actual.shape == expected.shape
    assert np.max(np.linalg.norm(actual - expected, axis=1)) == pytest.approx(
        0.0,
        abs=1e-9,
    )


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


def test_stop_line_object_to_xml_emits_standard_outline():
    """Standard stopLine objects may use outline/cornerLocal for physical geometry."""
    obj = StopLineObject(
        id=43,
        name="stop_line_43",
        s=0.0,
        t=0.0,
        z_offset=0.0,
        hdg=0.0,
        width=0.1,
        length=2.0,
        corners=[
            CornerLocal(u=-1.0, v=-0.05),
            CornerLocal(u=1.0, v=-0.05),
            CornerLocal(u=1.0, v=0.05),
            CornerLocal(u=-1.0, v=0.05),
            CornerLocal(u=-1.0, v=-0.05),
        ],
    )

    elem = obj.to_xml()

    outline = elem.find("outline")
    assert outline is not None
    corners = outline.findall("cornerLocal")
    assert len(corners) == 5
    assert float(corners[0].get("u")) == pytest.approx(-1.0)
    assert float(corners[0].get("v")) == pytest.approx(-0.05)


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


@pytest.mark.parametrize(
    ("x", "expected_s"),
    [
        (5.0, 5.0),  # safely inside the road domain
        (0.0, 0.0),  # exactly at road start
        (10.0, 10.0),  # exactly at road end
    ],
)
def test_construct_from_linestring_in_domain_keeps_standard_line_object(
    x: float,
    expected_s: float,
):
    """In-domain stop lines do not need an outline fallback."""
    road = _make_mock_line_road(road_id=1, x=0.0, y=0.0, hdg=0.0, length=10.0)
    pts_2d = np.array([[x, -2.0], [x, 2.0]])
    pts_3d = np.array([[x, -2.0, 0.0], [x, 2.0, 0.0]])
    ls = MagicMock()
    ls.id = 1010

    from unittest.mock import patch

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
            use_physical_outline=True,
        )

    assert result is not None
    assert result.s == pytest.approx(expected_s)
    assert result.t == pytest.approx(0.0)
    assert result.length == pytest.approx(4.0)
    assert result.corners == []


@pytest.mark.parametrize(
    ("x", "expected_s"),
    [
        (-0.2, 0.0),  # slightly before road start
        (10.2, 10.0),  # slightly after road end
    ],
)
def test_construct_from_linestring_endpoint_clamp_uses_physical_outline(
    x: float,
    expected_s: float,
):
    """Endpoint-clamped standard stop lines keep their physical rectangle."""
    road = _make_mock_line_road(road_id=2, x=0.0, y=0.0, hdg=0.0, length=10.0)
    pts_2d = np.array([[x, -2.0], [x, 2.0]])
    pts_3d = np.array([[x, -2.0, 0.0], [x, 2.0, 0.0]])
    ls = MagicMock()
    ls.id = 2020

    from unittest.mock import patch

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
            width=0.1,
            use_physical_outline=True,
        )

    assert result is not None
    assert result.s == pytest.approx(expected_s)
    assert len(result.corners) == 5
    _assert_outline_matches_source_rectangle(
        result,
        road,
        pts_2d[0],
        pts_2d[-1],
        width=0.1,
    )


def test_construct_from_linestring_oblique_endpoint_clamp_uses_physical_outline():
    """The outline fallback preserves oblique stop-line endpoints and heading."""
    road = _make_mock_line_road(road_id=3, x=0.0, y=0.0, hdg=0.0, length=10.0)
    pts_2d = np.array([[10.2, -1.0], [10.7, 1.0]])
    pts_3d = np.array([[10.2, -1.0, 0.0], [10.7, 1.0, 0.0]])
    ls = MagicMock()
    ls.id = 3030

    from unittest.mock import patch

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
            width=0.1,
            use_physical_outline=True,
        )

    assert result is not None
    assert result.s == pytest.approx(10.0)
    assert result.hdg == pytest.approx(math.atan2(2.0, 0.5))
    assert result.length == pytest.approx(math.hypot(0.5, 2.0))
    _assert_outline_matches_source_rectangle(
        result,
        road,
        pts_2d[0],
        pts_2d[-1],
        width=0.1,
    )


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


def test_find_best_road_for_stop_line_prefers_semantic_candidate_over_adjacent_branch():
    """Semantic candidates should beat unrelated nearby branches at junctions."""
    unrelated = _make_mock_line_road(road_id=10, x=0.2, y=0.0, hdg=0.0, length=10.0)
    semantic = _make_mock_line_road(road_id=20, x=-5.0, y=1.0, hdg=0.0, length=10.0)
    pts_2d = np.array([[-0.5, 0.0], [0.5, 0.0]])
    ls = MagicMock()
    ls.id = 7007

    from unittest.mock import patch

    with patch(
        "autoware_lanelet2_to_opendrive.opendrive.objects.extract_points"
    ) as mock_extract:
        mock_extract.return_value = pts_2d
        result = find_best_road_for_stop_line(
            ls,
            [unrelated, semantic],
            related_roads=[semantic],
        )

    assert result is not None
    assert result.id == semantic.id


def test_find_best_road_for_stop_line_prefers_incoming_predecessor_at_junction_boundary():
    """A stop line at a junction entry belongs to the incoming road endpoint."""
    incoming = _make_mock_line_road(
        road_id=30,
        x=-10.0,
        y=0.0,
        hdg=0.0,
        length=10.0,
    )
    connecting = _make_mock_line_road(
        road_id=40,
        x=0.2,
        y=0.0,
        hdg=0.0,
        length=10.0,
    )
    pts_2d = np.array([[-0.5, 0.0], [0.5, 0.0]])
    pts_3d = np.array([[-0.5, 0.0, 0.0], [0.5, 0.0, 0.0]])
    ls = MagicMock()
    ls.id = 8008

    from unittest.mock import patch

    with patch(
        "autoware_lanelet2_to_opendrive.opendrive.objects.extract_points"
    ) as mock_extract:
        mock_extract.side_effect = lambda linestring, dimensions: (
            pts_2d if dimensions == 2 else pts_3d
        )
        result = find_best_road_for_stop_line(
            ls,
            [connecting, incoming],
            related_roads=[connecting],
            predecessor_roads=[incoming],
            endpoint_tolerance=0.5,
        )

        assert result is not None
        assert result.id == incoming.id

        obj = StopLineObject.construct_from_linestring(ls, result, object_id=ls.id)
        assert obj is not None
        assert obj.s == pytest.approx(incoming.length)
        assert obj.t == pytest.approx(0.0)


def test_find_best_road_for_stop_line_preserves_nearest_fallback_without_semantics():
    """Without semantic/topological candidates, keep the previous nearest-road behavior."""
    road_near = _make_mock_line_road(road_id=50, x=-5.0, y=0.0, hdg=0.0, length=10.0)
    road_far = _make_mock_line_road(road_id=60, x=-5.0, y=5.0, hdg=0.0, length=10.0)
    pts_2d = np.array([[-0.5, 0.0], [0.5, 0.0]])
    ls = MagicMock()
    ls.id = 9009

    from unittest.mock import patch

    with patch(
        "autoware_lanelet2_to_opendrive.opendrive.objects.extract_points"
    ) as mock_extract:
        mock_extract.return_value = pts_2d
        result = find_best_road_for_stop_line(ls, [road_far, road_near])

    assert result is not None
    assert result.id == road_near.id


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
    assert result.corners == []

    elem = result.to_xml()
    assert elem.get("type") == "-1"
    assert elem.get("name") == "Stencil_STOP"
    assert elem.get("orientation") == "-"
    assert float(elem.get("zOffset")) == pytest.approx(0.0)
    assert float(elem.get("width")) == pytest.approx(2.0)
    assert elem.find("outline") is None


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


def test_construct_from_linestring_inside_curved_reference_keeps_line_object():
    """Curved roads still use ordinary s/t/hdg when the line is in-domain."""
    from autoware_lanelet2_to_opendrive.opendrive.geometry import (
        evaluate_plan_view_world,
    )

    curvature, length, s0 = 0.04, 20.0, 5.0
    road = _make_arc_road(curvature, length, s=s0)
    road.length = s0 + length
    road.get_elevation_at_s.return_value = 0.0
    geom = road.plan_view.geometries[0]
    p_target = 6.0
    center = np.asarray(
        evaluate_plan_view_world(
            geom.x,
            geom.y,
            geom.hdg,
            p_target,
            arc_curvature=curvature,
        )
    )
    road_hdg = geom.hdg + curvature * p_target
    stop_dir = np.array([-math.sin(road_hdg), math.cos(road_hdg)])
    pts_2d = np.asarray([center - 2.0 * stop_dir, center + 2.0 * stop_dir])
    pts_3d = np.column_stack([pts_2d, np.zeros(2)])
    ls = MagicMock()
    ls.id = 4040

    from unittest.mock import patch

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
            use_physical_outline=True,
        )

    assert result is not None
    assert result.s == pytest.approx(s0 + p_target, abs=1e-8)
    assert result.t == pytest.approx(0.0, abs=1e-8)
    assert result.corners == []

    origin, abs_hdg = _object_origin_world(result, road)
    tangent = np.array([math.cos(abs_hdg), math.sin(abs_hdg)])
    actual = np.asarray(
        [
            origin - 0.5 * result.length * tangent,
            origin + 0.5 * result.length * tangent,
        ]
    )
    assert np.max(np.linalg.norm(actual - pts_2d, axis=1)) == pytest.approx(
        0.0,
        abs=1e-8,
    )


def test_project_point_onto_straight_road_uses_continuous_projection():
    """Projection must not quantize object origins to sampled road points."""
    from autoware_lanelet2_to_opendrive.opendrive.geometry import Line
    from autoware_lanelet2_to_opendrive.opendrive.objects import (
        _SAMPLE_POINTS_PER_GEOMETRY,
        _project_point_onto_road,
    )

    plan_view = MagicMock()
    plan_view.geometries = [Line(s=0.0, x=10.0, y=20.0, hdg=0.0, length=10.0)]
    road = MagicMock()
    road.id = 1
    road.plan_view = plan_view

    p_target = 5.5
    point = np.array([10.0 + p_target, 22.0])
    sample_spacing = 10.0 / (_SAMPLE_POINTS_PER_GEOMETRY - 1)
    nearest_sample = round(p_target / sample_spacing) * sample_spacing
    assert abs(nearest_sample - p_target) > 0.05

    result = _project_point_onto_road(point, road)
    assert result is not None
    s, t, hdg = result
    assert s == pytest.approx(p_target, abs=1e-10)
    assert t == pytest.approx(2.0, abs=1e-10)
    assert hdg == pytest.approx(0.0, abs=1e-10)


def test_project_point_onto_parampoly3_road_recovers_offset_point():
    """Curved ParamPoly3 projection should recover a normal-offset point."""
    from autoware_lanelet2_to_opendrive.opendrive.geometry import (
        ParamPoly3,
        evaluate_plan_view_world,
    )
    from autoware_lanelet2_to_opendrive.opendrive.objects import (
        _project_point_onto_road,
    )

    geom = ParamPoly3(
        s=3.0,
        x=20.0,
        y=30.0,
        hdg=0.2,
        length=10.0,
        aU=0.0,
        bU=1.0,
        cU=0.0,
        dU=0.0,
        aV=0.0,
        bV=0.0,
        cV=0.02,
        dV=0.0,
    )
    plan_view = MagicMock()
    plan_view.geometries = [geom]
    road = MagicMock()
    road.id = 2
    road.plan_view = plan_view

    p_target = 4.2
    t_target = 1.25
    wx, wy = evaluate_plan_view_world(
        geom.x,
        geom.y,
        geom.hdg,
        p_target,
        param_poly3_coeffs=(
            geom.aU,
            geom.bU,
            geom.cU,
            geom.dU,
            geom.aV,
            geom.bV,
            geom.cV,
            geom.dV,
        ),
    )
    du = geom.bU + 2.0 * geom.cU * p_target + 3.0 * geom.dU * p_target**2
    dv = geom.bV + 2.0 * geom.cV * p_target + 3.0 * geom.dV * p_target**2
    cos_h = math.cos(geom.hdg)
    sin_h = math.sin(geom.hdg)
    tx = du * cos_h - dv * sin_h
    ty = du * sin_h + dv * cos_h
    tangent_norm = math.hypot(tx, ty)
    left_normal = np.array([-ty / tangent_norm, tx / tangent_norm])
    point = np.array([wx, wy]) + t_target * left_normal

    result = _project_point_onto_road(point, road)
    assert result is not None
    s, t, hdg = result
    assert s == pytest.approx(geom.s + p_target, abs=1e-8)
    assert t == pytest.approx(t_target, abs=1e-8)
    assert hdg == pytest.approx(geom.hdg + math.atan2(dv, du), abs=1e-8)


def test_project_point_onto_segment_boundary_uses_nearest_continuous_segment():
    """Projection near a boundary should not snap back to a sampled endpoint."""
    from autoware_lanelet2_to_opendrive.opendrive.geometry import Line
    from autoware_lanelet2_to_opendrive.opendrive.objects import (
        _project_point_onto_road,
    )

    plan_view = MagicMock()
    plan_view.geometries = [
        Line(s=0.0, x=0.0, y=0.0, hdg=0.0, length=10.0),
        Line(s=10.0, x=10.0, y=0.0, hdg=0.0, length=10.0),
    ]
    road = MagicMock()
    road.id = 3
    road.plan_view = plan_view

    result = _project_point_onto_road(np.array([10.02, -1.0]), road)
    assert result is not None
    s, t, hdg = result
    assert s == pytest.approx(10.02, abs=1e-10)
    assert t == pytest.approx(-1.0, abs=1e-10)
    assert hdg == pytest.approx(0.0, abs=1e-10)


def test_project_point_onto_road_clamps_to_endpoints():
    """Projection outside the road should stay on the finite road segment."""
    from autoware_lanelet2_to_opendrive.opendrive.geometry import Line
    from autoware_lanelet2_to_opendrive.opendrive.objects import (
        _project_point_onto_road,
    )

    plan_view = MagicMock()
    plan_view.geometries = [Line(s=0.0, x=0.0, y=0.0, hdg=0.0, length=10.0)]
    road = MagicMock()
    road.id = 4
    road.plan_view = plan_view

    start_result = _project_point_onto_road(np.array([-0.7, 2.0]), road)
    assert start_result is not None
    start_s, start_t, _ = start_result
    assert start_s == pytest.approx(0.0, abs=1e-10)
    assert start_t == pytest.approx(2.0, abs=1e-10)

    end_result = _project_point_onto_road(np.array([11.0, -3.0]), road)
    assert end_result is not None
    end_s, end_t, _ = end_result
    assert end_s == pytest.approx(10.0, abs=1e-10)
    assert end_t == pytest.approx(-3.0, abs=1e-10)
