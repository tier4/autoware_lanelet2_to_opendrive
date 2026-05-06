"""Boundary invariants for ``Lane.lane_id``.

These tests guard the contract introduced in issue #471: a Lane whose
``lane_id`` is still ``None`` must never produce XML, never be added to a
``LaneSection``, and must round-trip the value supplied to
``Lane.construct_from_lanelet``.
"""

import pytest

from autoware_lanelet2_to_opendrive.opendrive.lane import Lane
from autoware_lanelet2_to_opendrive.opendrive.opendrive_dataclass import LaneType


def test_to_xml_rejects_unresolved_lane_id() -> None:
    """``Lane.to_xml`` must refuse to serialise a Lane whose ID is None."""
    lane = Lane(lane_id=None, lane_type=LaneType.DRIVING)

    with pytest.raises(AssertionError, match="lane_id"):
        lane.to_xml()


def test_construct_from_lanelet_round_trips_lane_id() -> None:
    """The lane_id passed to ``Lane.construct_from_lanelet`` survives onto the result."""
    import lanelet2

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
    lanelet.attributes["subtype"] = "road"

    lanelet_map = lanelet2.core.LaneletMap()
    lanelet_map.add(lanelet)

    lane = Lane.construct_from_lanelet(lanelet_map, lanelet, lane_id=-2)

    assert lane.lane_id == -2


def test_add_left_lane_rejects_unresolved_lane_id() -> None:
    """``LaneSection._add_left_lane`` rejects a Lane whose ID is None."""
    from autoware_lanelet2_to_opendrive.opendrive.lane_section import LaneSection

    section = LaneSection(s_offset=0.0)
    lane = Lane(lane_id=None, lane_type=LaneType.DRIVING)

    with pytest.raises(ValueError, match="resolved lane_id"):
        section._add_left_lane(lane)


def test_add_right_lane_rejects_unresolved_lane_id() -> None:
    """``LaneSection._add_right_lane`` rejects a Lane whose ID is None."""
    from autoware_lanelet2_to_opendrive.opendrive.lane_section import LaneSection

    section = LaneSection(s_offset=0.0)
    lane = Lane(lane_id=None, lane_type=LaneType.DRIVING)

    with pytest.raises(ValueError, match="resolved lane_id"):
        section._add_right_lane(lane)
