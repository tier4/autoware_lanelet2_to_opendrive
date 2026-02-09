"""Tests for OpenDRIVE validation functionality."""

from autoware_lanelet2_to_opendrive.opendrive.lane import Lane
from autoware_lanelet2_to_opendrive.opendrive.lane_elements import LaneLink
from autoware_lanelet2_to_opendrive.opendrive.lane_section import LaneSection
from autoware_lanelet2_to_opendrive.opendrive.lane_sections import Lanes
from autoware_lanelet2_to_opendrive.opendrive.opendrive_dataclass import LaneType
from autoware_lanelet2_to_opendrive.opendrive.road import Road
from autoware_lanelet2_to_opendrive.opendrive.road_links import (
    Predecessor,
    RoadLink,
    Successor,
)
from autoware_lanelet2_to_opendrive.opendrive.enums import ContactPoint, ElementType
from autoware_lanelet2_to_opendrive.opendrive.validation import (
    validate_lane_road_link_consistency,
)


def create_test_lane(
    lane_id: int,
    has_predecessor: bool = False,
    has_successor: bool = False,
) -> Lane:
    """Create a test lane with optional predecessor/successor."""
    lane = Lane(
        lane_id=lane_id,
        lane_type=LaneType.DRIVING,
        level=False,
    )

    if has_predecessor:
        lane.predecessor = LaneLink(id=lane_id)

    if has_successor:
        lane.successor = LaneLink(id=lane_id)

    return lane


def create_test_road(
    road_id: int,
    has_road_predecessor: bool = False,
    has_road_successor: bool = False,
    junction: int = -1,
) -> Road:
    """Create a test road with optional road-level predecessor/successor."""
    # Create road link if needed
    road_link = None
    if has_road_predecessor or has_road_successor:
        predecessor = None
        successor = None

        if has_road_predecessor:
            predecessor = Predecessor(
                element_type=ElementType.ROAD,
                element_id=road_id - 1,
                contact_point=ContactPoint.END,
            )

        if has_road_successor:
            successor = Successor(
                element_type=ElementType.ROAD,
                element_id=road_id + 1,
                contact_point=ContactPoint.START,
            )

        road_link = RoadLink(predecessor=predecessor, successor=successor)

    # Create lane section with test lanes
    lane_section = LaneSection(s_offset=0.0)

    # Create lanes sections
    lanes = Lanes(lane_sections=[lane_section])

    # Create road
    road = Road(
        id=road_id,
        name=f"test_road_{road_id}",
        length=100.0,
        junction=junction,
        link=road_link,
        lanes=lanes,
    )

    return road


class TestLaneRoadLinkConsistency:
    """Test suite for lane-road link consistency validation."""

    def test_valid_road_with_no_connections(self):
        """Test that a road with no lane or road connections is valid."""
        road = create_test_road(
            road_id=1, has_road_predecessor=False, has_road_successor=False
        )

        # Add lane without connections
        lane = create_test_lane(lane_id=-1, has_predecessor=False, has_successor=False)
        road.lanes.lane_sections[0].right_lanes = {-1: lane}

        result = validate_lane_road_link_consistency([road])

        assert result.is_valid
        assert result.error_count == 0

    def test_valid_road_with_consistent_connections(self):
        """Test that a road with consistent lane and road connections is valid."""
        road = create_test_road(
            road_id=1, has_road_predecessor=True, has_road_successor=True
        )

        # Add lane with connections (matches road connections)
        lane = create_test_lane(lane_id=-1, has_predecessor=True, has_successor=True)
        road.lanes.lane_sections[0].right_lanes = {-1: lane}

        result = validate_lane_road_link_consistency([road])

        assert result.is_valid
        assert result.error_count == 0

    def test_invalid_lane_predecessor_without_road_predecessor(self):
        """Test that lane predecessor without road predecessor is invalid."""
        road = create_test_road(
            road_id=1, has_road_predecessor=False, has_road_successor=True
        )

        # Add lane with predecessor but road has no predecessor
        lane = create_test_lane(lane_id=-1, has_predecessor=True, has_successor=True)
        road.lanes.lane_sections[0].right_lanes = {-1: lane}

        result = validate_lane_road_link_consistency([road])

        assert not result.is_valid
        assert result.error_count == 1
        assert result.errors[0].road_id == 1
        assert result.errors[0].lane_id == -1
        assert result.errors[0].connection_type == "predecessor"
        assert "Lane has predecessor but road does not" in result.errors[0].message

    def test_invalid_lane_successor_without_road_successor(self):
        """Test that lane successor without road successor is invalid."""
        road = create_test_road(
            road_id=1, has_road_predecessor=True, has_road_successor=False
        )

        # Add lane with successor but road has no successor
        lane = create_test_lane(lane_id=-1, has_predecessor=True, has_successor=True)
        road.lanes.lane_sections[0].right_lanes = {-1: lane}

        result = validate_lane_road_link_consistency([road])

        assert not result.is_valid
        assert result.error_count == 1
        assert result.errors[0].road_id == 1
        assert result.errors[0].lane_id == -1
        assert result.errors[0].connection_type == "successor"
        assert "Lane has successor but road does not" in result.errors[0].message

    def test_invalid_multiple_lanes_with_invalid_connections(self):
        """Test that multiple lanes with invalid connections are detected."""
        road = create_test_road(
            road_id=1, has_road_predecessor=False, has_road_successor=False
        )

        # Add multiple lanes with invalid connections
        lane1 = create_test_lane(lane_id=-1, has_predecessor=True, has_successor=True)
        lane2 = create_test_lane(lane_id=-2, has_predecessor=True, has_successor=False)

        road.lanes.lane_sections[0].right_lanes = {-1: lane1, -2: lane2}

        result = validate_lane_road_link_consistency([road])

        assert not result.is_valid
        assert result.error_count == 3  # lane1 pred, lane1 succ, lane2 pred

    def test_connecting_road_allows_lane_links_without_road_links(self):
        """Test that connecting roads (junction members) can have lane links without road links."""
        # Create connecting road (junction member)
        road = create_test_road(
            road_id=1,
            has_road_predecessor=False,
            has_road_successor=False,
            junction=100,  # Member of junction 100
        )

        # Add lane with connections (should be allowed for connecting roads)
        lane = create_test_lane(lane_id=-1, has_predecessor=True, has_successor=True)
        road.lanes.lane_sections[0].right_lanes = {-1: lane}

        result = validate_lane_road_link_consistency([road])

        assert result.is_valid
        assert result.error_count == 0

    def test_validation_result_error_summary(self):
        """Test that ValidationResult provides useful error summary."""
        road = create_test_road(
            road_id=1, has_road_predecessor=False, has_road_successor=False
        )

        lane = create_test_lane(lane_id=-1, has_predecessor=True, has_successor=False)
        road.lanes.lane_sections[0].right_lanes = {-1: lane}

        result = validate_lane_road_link_consistency([road])

        summary = result.get_error_summary()

        assert "Found 1 validation errors" in summary
        assert "Road 1 Lane -1" in summary
        assert "predecessor" in summary

    def test_empty_roads_list(self):
        """Test validation with empty roads list."""
        result = validate_lane_road_link_consistency([])

        assert result.is_valid
        assert result.error_count == 0

    def test_road_without_lanes(self):
        """Test validation with road that has no lanes."""
        road = Road(
            id=1,
            name="test_road_1",
            length=100.0,
            junction=-1,
            link=None,
            lanes=None,
        )

        result = validate_lane_road_link_consistency([road])

        assert result.is_valid
        assert result.error_count == 0
