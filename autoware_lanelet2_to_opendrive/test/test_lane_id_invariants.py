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
