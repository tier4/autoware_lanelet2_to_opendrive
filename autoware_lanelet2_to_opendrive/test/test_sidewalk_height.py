"""Unit tests for sidewalk / shoulder lane <height> emission (issue #469).

These tests exercise ``Lane.construct_from_lanelet`` directly with synthetic
in-memory lanelets — no .osm fixture, no MGRS projection. They verify the
inner-boundary curb detection and the symmetric ``inner=outer=H`` height
emission.
"""

from __future__ import annotations

import lanelet2
import pytest

from autoware_lanelet2_to_opendrive.opendrive.lane import Lane


def _make_lanelet(
    subtype: str,
    left_type: str | None = None,
    right_type: str | None = None,
) -> tuple[lanelet2.core.LaneletMap, lanelet2.core.Lanelet]:
    """Build a 10 m x 2 m straight lanelet with optional boundary types."""
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
    if left_type is not None:
        left_bound.attributes["type"] = left_type
    if right_type is not None:
        right_bound.attributes["type"] = right_type
    lanelet = lanelet2.core.Lanelet(lanelet2.core.getId(), left_bound, right_bound)
    lanelet.attributes["subtype"] = subtype
    lanelet_map = lanelet2.core.LaneletMap()
    lanelet_map.add(lanelet)
    return lanelet_map, lanelet


def test_rht_walkway_with_inner_curbstone_emits_height() -> None:
    """RHT + walkway + leftBound type=curbstone → exactly one height (0.15/0.15)."""
    lanelet_map, lanelet = _make_lanelet("walkway", left_type="curbstone")

    lane = Lane.construct_from_lanelet(lanelet_map, lanelet, lane_id=-1, rule="RHT")

    assert len(lane.heights) == 1
    h = lane.heights[0]
    assert h.s_offset == 0.0
    assert h.inner == pytest.approx(0.15)
    assert h.outer == pytest.approx(0.15)


def test_lht_walkway_with_inner_road_border_emits_height() -> None:
    """LHT + walkway + rightBound type=road_border → one height entry."""
    lanelet_map, lanelet = _make_lanelet("walkway", right_type="road_border")

    lane = Lane.construct_from_lanelet(lanelet_map, lanelet, lane_id=-1, rule="LHT")

    assert len(lane.heights) == 1
    assert lane.heights[0].inner == pytest.approx(0.15)
    assert lane.heights[0].outer == pytest.approx(0.15)


def test_walkway_without_curb_emits_no_height() -> None:
    """Walkway with no curb-tagged inner boundary → no height entry."""
    lanelet_map, lanelet = _make_lanelet("walkway", left_type="line_thin")

    lane = Lane.construct_from_lanelet(lanelet_map, lanelet, lane_id=-1, rule="RHT")

    assert lane.heights == []


def test_walkway_with_curb_only_on_outer_emits_no_height() -> None:
    """Inner-only check: outer-side curb is not enough to trigger emission."""
    lanelet_map, lanelet = _make_lanelet("walkway", right_type="curbstone")

    lane = Lane.construct_from_lanelet(lanelet_map, lanelet, lane_id=-1, rule="RHT")

    assert lane.heights == []


def test_driving_lanelet_with_inner_curb_emits_no_height() -> None:
    """A road lanelet (lane_type=DRIVING) is unaffected even with a curb."""
    lanelet_map, lanelet = _make_lanelet("road", left_type="curbstone")

    lane = Lane.construct_from_lanelet(lanelet_map, lanelet, lane_id=-1, rule="RHT")

    assert lane.heights == []


def test_road_shoulder_with_inner_curb_emits_height() -> None:
    """Shoulder is in scope: RHT + road_shoulder + leftBound curb → height."""
    lanelet_map, lanelet = _make_lanelet("road_shoulder", left_type="curbstone")

    lane = Lane.construct_from_lanelet(lanelet_map, lanelet, lane_id=-1, rule="RHT")

    assert len(lane.heights) == 1
    assert lane.heights[0].inner == pytest.approx(0.15)


def test_default_rule_is_rht() -> None:
    """rule=None is treated as RHT (same convention as width anchoring)."""
    lanelet_map, lanelet = _make_lanelet("walkway", left_type="curbstone")

    lane = Lane.construct_from_lanelet(lanelet_map, lanelet, lane_id=-1, rule=None)

    assert len(lane.heights) == 1


def test_sidewalk_height_value_comes_from_config(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Custom DEFAULT_CONFIG.geometry.sidewalk_height is reflected in output."""
    from autoware_lanelet2_to_opendrive.config import (
        ConversionConstants,
        GeometryConstants,
    )
    import autoware_lanelet2_to_opendrive.opendrive.lane as lane_module

    # Both ConversionConstants and GeometryConstants are frozen dataclasses,
    # so we replace the module-level DEFAULT_CONFIG reference rather than
    # mutating attributes in place.
    custom = ConversionConstants(geometry=GeometryConstants(sidewalk_height=0.25))
    monkeypatch.setattr(lane_module, "DEFAULT_CONFIG", custom)

    lanelet_map, lanelet = _make_lanelet("walkway", left_type="curbstone")
    lane = Lane.construct_from_lanelet(lanelet_map, lanelet, lane_id=-1, rule="RHT")

    assert len(lane.heights) == 1
    assert lane.heights[0].inner == pytest.approx(0.25)
    assert lane.heights[0].outer == pytest.approx(0.25)
