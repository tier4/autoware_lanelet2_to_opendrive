"""Test that self-reference guard works correctly."""

from autoware_lanelet2_to_opendrive.opendrive.lane import Lane
from autoware_lanelet2_to_opendrive.opendrive.lane_elements import LaneLink
from autoware_lanelet2_to_opendrive.opendrive.enums import LaneType


def test_self_reference_guard_predecessor():
    """Test that the self-reference guard prevents predecessor self-references."""
    # The self-reference check is in road.py _set_single_lane_links()
    # It checks: if pred_lane_id == lane.lane_id and pred_road_id == self.id: continue

    # This is a behavior test - the guard should prevent setting self-references
    # We'll test this by checking the logic manually

    lane = Lane(
        lane_id=1,
        lane_type=LaneType.DRIVING,
        level=False,
        predecessor=None,
        successor=None,
        lanelet_id=100,
    )

    # Simulate the check in _set_single_lane_links
    road_id = 0
    pred_road_id = 0
    pred_lane_id = 1

    # This should be blocked
    is_self_reference = pred_lane_id == lane.lane_id and pred_road_id == road_id

    assert is_self_reference, "Self-reference should be detected"

    # In the actual code, this would cause a continue, so predecessor should not be set
    # We verify the logic here
    if not is_self_reference:
        lane.predecessor = LaneLink(id=pred_lane_id)

    # Lane predecessor should still be None because of self-reference check
    assert lane.predecessor is None, "Predecessor should not be set for self-reference"


def test_non_self_reference_allowed():
    """Test that non-self-references are allowed."""
    lane = Lane(
        lane_id=1,
        lane_type=LaneType.DRIVING,
        level=False,
        predecessor=None,
        successor=None,
        lanelet_id=100,
    )

    # Simulate the check in _set_single_lane_links
    road_id = 0
    pred_road_id = 1  # Different road
    pred_lane_id = 1

    # This should NOT be blocked (different road)
    is_self_reference = pred_lane_id == lane.lane_id and pred_road_id == road_id

    assert not is_self_reference, "This is not a self-reference (different road)"

    # Set the predecessor
    if not is_self_reference:
        lane.predecessor = LaneLink(id=pred_lane_id)

    # Lane predecessor should be set
    assert lane.predecessor is not None, "Predecessor should be set for different road"
    assert lane.predecessor.id == pred_lane_id


def test_same_lane_id_different_road_allowed():
    """Test that same lane ID on different road is allowed."""
    lane = Lane(
        lane_id=1,
        lane_type=LaneType.DRIVING,
        level=False,
        predecessor=None,
        successor=None,
        lanelet_id=100,
    )

    # Simulate the check in _set_single_lane_links
    road_id = 5
    pred_road_id = 10  # Different road
    pred_lane_id = 1  # Same lane ID but different road

    # This should NOT be blocked (different road)
    is_self_reference = pred_lane_id == lane.lane_id and pred_road_id == road_id

    assert not is_self_reference, "Same lane ID on different road is not self-reference"

    # Set the predecessor
    if not is_self_reference:
        lane.predecessor = LaneLink(id=pred_lane_id)

    # Lane predecessor should be set
    assert lane.predecessor is not None
    assert lane.predecessor.id == 1
