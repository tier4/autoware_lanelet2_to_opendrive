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

    lane = Lane.construct_from_lanelet(lanelet_map, lanelet, rule="RHT")

    assert len(lane.heights) == 1
    h = lane.heights[0]
    assert h.s_offset == 0.0
    assert h.inner == pytest.approx(0.15)
    assert h.outer == pytest.approx(0.15)
