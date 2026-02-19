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
    assert result.width == 0.0


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
