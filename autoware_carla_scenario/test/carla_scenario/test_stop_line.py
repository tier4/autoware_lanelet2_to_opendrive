"""Unit tests for stop line utilities.

Tests are split between:
- ``TestGetStopLineLinestrings``: regulatory element traversal (``utils.stop_line``)
- ``TestGetStopLinePoses``: centroid → Lanelet2Pose conversion (``coordinate.stop_line``)
"""

from __future__ import annotations

from unittest.mock import MagicMock, patch

import pytest


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_linestring(
    ls_id: int, points: list[tuple[float, float]], attrs: dict | None = None
) -> MagicMock:
    """Create a mock linestring with an id, points, and optional attributes."""
    mock_points = []
    for x, y in points:
        pt = MagicMock()
        pt.x = x
        pt.y = y
        mock_points.append(pt)

    ls = MagicMock()
    ls.id = ls_id
    ls.__iter__ = lambda self: iter(mock_points)
    ls.attributes = attrs or {}
    return ls


def _make_reg_elem(
    re_id: int,
    *,
    has_stop_line: bool = False,
    stop_line: object | None = None,
    subtype: str | None = None,
    params: dict | None = None,
) -> MagicMock:
    """Create a mock regulatory element."""
    re = MagicMock()
    re.id = re_id

    attrs = {}
    if subtype is not None:
        attrs["subtype"] = subtype
    re.attributes = attrs

    if has_stop_line:
        re.stopLine = stop_line
    else:
        del re.stopLine

    if params is not None:
        re.parameters = params
    else:
        re.parameters = {}

    return re


def _make_arc_coordinates(length: float) -> MagicMock:
    """Create a mock arc coordinates result."""
    arc = MagicMock()
    arc.length = length
    return arc


# ---------------------------------------------------------------------------
# TestGetStopLineLinestrings – utils/stop_line.py
# ---------------------------------------------------------------------------


class TestGetStopLineLinestrings:
    """Tests for get_stop_line_linestrings() (regulatory element traversal)."""

    @patch("autoware_carla_scenario.utils.stop_line.MapManager")
    def test_lanelet_not_found_raises(self, mock_mm_cls: MagicMock) -> None:
        from autoware_carla_scenario.utils.stop_line import get_stop_line_linestrings

        mm = MagicMock()
        mock_mm_cls.get_instance.return_value = mm
        mm.lanelet_map.laneletLayer.__getitem__ = MagicMock(
            side_effect=KeyError("not found")
        )

        with pytest.raises(ValueError, match="Lanelet ID 999 not found"):
            get_stop_line_linestrings(999)

    @patch("autoware_carla_scenario.utils.stop_line.MapManager")
    def test_no_stop_lines_returns_empty(self, mock_mm_cls: MagicMock) -> None:
        from autoware_carla_scenario.utils.stop_line import get_stop_line_linestrings

        lanelet = MagicMock()
        lanelet.regulatoryElements = []

        mm = MagicMock()
        mock_mm_cls.get_instance.return_value = mm
        mm.lanelet_map.laneletLayer.__getitem__ = MagicMock(return_value=lanelet)

        assert get_stop_line_linestrings(100) == []

    @patch("autoware_carla_scenario.utils.stop_line.MapManager")
    def test_traffic_light_stop_line(self, mock_mm_cls: MagicMock) -> None:
        from autoware_carla_scenario.utils.stop_line import get_stop_line_linestrings

        stop_ls = _make_linestring(10, [(1.0, 2.0)])
        reg_elem = _make_reg_elem(1, has_stop_line=True, stop_line=stop_ls)

        lanelet = MagicMock()
        lanelet.regulatoryElements = [reg_elem]

        mm = MagicMock()
        mock_mm_cls.get_instance.return_value = mm
        mm.lanelet_map.laneletLayer.__getitem__ = MagicMock(return_value=lanelet)

        result = get_stop_line_linestrings(100)
        assert len(result) == 1
        assert result[0].id == 10

    @patch("autoware_carla_scenario.utils.stop_line.MapManager")
    def test_traffic_sign_ref_line(self, mock_mm_cls: MagicMock) -> None:
        from autoware_carla_scenario.utils.stop_line import get_stop_line_linestrings

        stop_ls = _make_linestring(20, [(5.0, 6.0)], attrs={"type": "stop_line"})
        reg_elem = _make_reg_elem(
            2, subtype="traffic_sign", params={"ref_line": [stop_ls]}
        )

        lanelet = MagicMock()
        lanelet.regulatoryElements = [reg_elem]

        mm = MagicMock()
        mock_mm_cls.get_instance.return_value = mm
        mm.lanelet_map.laneletLayer.__getitem__ = MagicMock(return_value=lanelet)

        result = get_stop_line_linestrings(200)
        assert len(result) == 1
        assert result[0].id == 20

    @patch("autoware_carla_scenario.utils.stop_line.MapManager")
    def test_road_marking_refers(self, mock_mm_cls: MagicMock) -> None:
        from autoware_carla_scenario.utils.stop_line import get_stop_line_linestrings

        stop_ls = _make_linestring(30, [(7.0, 8.0)], attrs={"type": "stop_line"})
        reg_elem = _make_reg_elem(
            3, subtype="road_marking", params={"refers": [stop_ls]}
        )

        lanelet = MagicMock()
        lanelet.regulatoryElements = [reg_elem]

        mm = MagicMock()
        mock_mm_cls.get_instance.return_value = mm
        mm.lanelet_map.laneletLayer.__getitem__ = MagicMock(return_value=lanelet)

        result = get_stop_line_linestrings(300)
        assert len(result) == 1
        assert result[0].id == 30

    @patch("autoware_carla_scenario.utils.stop_line.MapManager")
    def test_duplicate_stop_lines_deduplicated(self, mock_mm_cls: MagicMock) -> None:
        from autoware_carla_scenario.utils.stop_line import get_stop_line_linestrings

        stop_ls = _make_linestring(10, [(1.0, 2.0)])
        re1 = _make_reg_elem(1, has_stop_line=True, stop_line=stop_ls)
        re2 = _make_reg_elem(2, has_stop_line=True, stop_line=stop_ls)

        lanelet = MagicMock()
        lanelet.regulatoryElements = [re1, re2]

        mm = MagicMock()
        mock_mm_cls.get_instance.return_value = mm
        mm.lanelet_map.laneletLayer.__getitem__ = MagicMock(return_value=lanelet)

        assert len(get_stop_line_linestrings(100)) == 1

    @patch("autoware_carla_scenario.utils.stop_line.MapManager")
    def test_multiple_different_stop_lines(self, mock_mm_cls: MagicMock) -> None:
        from autoware_carla_scenario.utils.stop_line import get_stop_line_linestrings

        stop_ls1 = _make_linestring(10, [(1.0, 2.0)])
        stop_ls2 = _make_linestring(20, [(3.0, 4.0)])
        re1 = _make_reg_elem(1, has_stop_line=True, stop_line=stop_ls1)
        re2 = _make_reg_elem(2, has_stop_line=True, stop_line=stop_ls2)

        lanelet = MagicMock()
        lanelet.regulatoryElements = [re1, re2]

        mm = MagicMock()
        mock_mm_cls.get_instance.return_value = mm
        mm.lanelet_map.laneletLayer.__getitem__ = MagicMock(return_value=lanelet)

        result = get_stop_line_linestrings(100)
        assert len(result) == 2


# ---------------------------------------------------------------------------
# TestGetStopLinePoses – coordinate/stop_line.py
# ---------------------------------------------------------------------------


class TestGetStopLinePoses:
    """Tests for get_stop_line_poses() (centroid → Lanelet2Pose conversion)."""

    @patch("autoware_carla_scenario.coordinate.stop_line.get_stop_line_linestrings")
    def test_empty_linestrings_returns_empty(self, mock_get_ls: MagicMock) -> None:
        from autoware_carla_scenario.coordinate.stop_line import get_stop_line_poses

        mock_get_ls.return_value = []
        assert get_stop_line_poses(100) == []

    @patch("autoware_carla_scenario.coordinate.stop_line.lanelet2")
    @patch("autoware_carla_scenario.coordinate.stop_line.MapManager")
    @patch("autoware_carla_scenario.coordinate.stop_line.get_stop_line_linestrings")
    def test_centroid_computed_and_projected(
        self,
        mock_get_ls: MagicMock,
        mock_mm_cls: MagicMock,
        mock_ll2: MagicMock,
    ) -> None:
        from autoware_carla_scenario.coordinate.stop_line import get_stop_line_poses

        stop_ls = _make_linestring(10, [(1.0, 2.0), (3.0, 4.0)])
        mock_get_ls.return_value = [stop_ls]

        lanelet = MagicMock()
        mm = MagicMock()
        mock_mm_cls.get_instance.return_value = mm
        mm.lanelet_map.laneletLayer.__getitem__ = MagicMock(return_value=lanelet)

        mock_ll2.geometry.toArcCoordinates.return_value = _make_arc_coordinates(15.0)
        mock_ll2.geometry.to2D.return_value = lanelet

        result = get_stop_line_poses(100)
        assert len(result) == 1
        assert result[0].lanelet_id == 100
        assert result[0].s == pytest.approx(15.0)
        assert result[0].t == pytest.approx(0.0)

        # Centroid: (1+3)/2=2.0, (2+4)/2=3.0
        mock_ll2.core.BasicPoint2d.assert_called_once_with(2.0, 3.0)

    @patch("autoware_carla_scenario.coordinate.stop_line.lanelet2")
    @patch("autoware_carla_scenario.coordinate.stop_line.MapManager")
    @patch("autoware_carla_scenario.coordinate.stop_line.get_stop_line_linestrings")
    def test_multiple_linestrings(
        self,
        mock_get_ls: MagicMock,
        mock_mm_cls: MagicMock,
        mock_ll2: MagicMock,
    ) -> None:
        from autoware_carla_scenario.coordinate.stop_line import get_stop_line_poses

        ls1 = _make_linestring(10, [(1.0, 2.0)])
        ls2 = _make_linestring(20, [(3.0, 4.0)])
        mock_get_ls.return_value = [ls1, ls2]

        lanelet = MagicMock()
        mm = MagicMock()
        mock_mm_cls.get_instance.return_value = mm
        mm.lanelet_map.laneletLayer.__getitem__ = MagicMock(return_value=lanelet)

        mock_ll2.geometry.toArcCoordinates.side_effect = [
            _make_arc_coordinates(10.0),
            _make_arc_coordinates(20.0),
        ]
        mock_ll2.geometry.to2D.return_value = lanelet

        result = get_stop_line_poses(100)
        assert len(result) == 2
        assert result[0].s == pytest.approx(10.0)
        assert result[1].s == pytest.approx(20.0)
