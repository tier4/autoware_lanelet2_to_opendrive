"""Tests for Lane class implementation."""

import pytest
from autoware_lanelet2_to_opendrive.opendrive.lane import Lane
from autoware_lanelet2_to_opendrive.opendrive.opendrive_dataclass import (
    LaneType,
    RoadMark,
    RoadMarkType,
    RoadMarkColor,
)

# Check if scenariogeneration is available
try:
    from scenariogeneration import xodr

    HAS_SCENARIOGENERATION = True
    del xodr  # Remove unused variable
except ImportError:
    HAS_SCENARIOGENERATION = False


class TestLane:
    """Test cases for Lane class."""

    def test_lane_creation(self):
        """Test basic lane creation."""
        lane = Lane(lane_id=1, lane_type=LaneType.DRIVING)

        assert lane.lane_id == 1
        assert lane.lane_type == LaneType.DRIVING
        assert lane.level is False
        assert len(lane.widths) == 0
        assert len(lane.road_marks) == 0

    def test_add_constant_width(self):
        """Test adding constant width to lane."""
        lane = Lane(lane_id=1, lane_type=LaneType.DRIVING)
        lane.add_constant_width(width=3.5)

        assert len(lane.widths) == 1
        assert lane.widths[0].s_offset == 0.0
        assert lane.widths[0].a == 3.5

    def test_add_road_mark(self):
        """Test adding road marks to lane."""
        lane = Lane(lane_id=1, lane_type=LaneType.DRIVING)
        road_mark = RoadMark(
            s_offset=0.0, type=RoadMarkType.SOLID, color=RoadMarkColor.WHITE
        )
        lane.add_road_mark(road_mark)

        assert len(lane.road_marks) == 1
        assert lane.road_marks[0].type == RoadMarkType.SOLID
        assert lane.road_marks[0].color == RoadMarkColor.WHITE

    def test_lane_repr(self):
        """Test lane string representation."""
        lane = Lane(lane_id=-1, lane_type=LaneType.SHOULDER)
        lane.add_constant_width(2.0)

        repr_str = repr(lane)
        assert "Lane(id=-1" in repr_str
        assert "type=shoulder" in repr_str
        assert "widths=1" in repr_str

    @pytest.mark.skipif(
        not HAS_SCENARIOGENERATION, reason="scenariogeneration not available"
    )
    def test_to_standard_lane(self):
        """Test conversion to standard lane."""
        lane = Lane(lane_id=1, lane_type=LaneType.DRIVING)
        lane.add_constant_width(3.5)
        lane.add_road_mark(
            RoadMark(s_offset=0.0, type=RoadMarkType.SOLID, color=RoadMarkColor.WHITE)
        )

        xodr_lane = lane.to_standard_lane()
        assert xodr_lane is not None

    @pytest.mark.skipif(
        not HAS_SCENARIOGENERATION, reason="scenariogeneration not available"
    )
    def test_to_xodr_roadmark(self):
        """Test XODR road mark conversion."""
        lane = Lane(lane_id=1, lane_type=LaneType.DRIVING)
        lane.add_road_mark(
            RoadMark(s_offset=0.0, type=RoadMarkType.SOLID, color=RoadMarkColor.WHITE)
        )

        road_marks = lane.to_xodr_roadmark()
        assert road_marks is not None
        assert len(road_marks) == 1

    def test_to_lane_def_data(self):
        """Test lane definition data conversion."""
        lane = Lane(lane_id=-1, lane_type=LaneType.DRIVING)
        lane.add_constant_width(3.5)

        data = lane.to_lane_def_data()
        assert data["lane_id"] == -1
        assert data["widths"] == [3.5]
        assert data["lane_type"] == "driving"


class TestLaneEnums:
    """Test cases for lane enumeration types."""

    def test_lane_type_values(self):
        """Test lane type enum values."""
        assert LaneType.DRIVING.value == "driving"
        assert LaneType.SIDEWALK.value == "sidewalk"
        assert LaneType.SHOULDER.value == "shoulder"

    def test_road_mark_type_values(self):
        """Test road mark type enum values."""
        assert RoadMarkType.SOLID.value == "solid"
        assert RoadMarkType.BROKEN.value == "broken"
        assert RoadMarkType.SOLID_BROKEN.value == "solid broken"

    def test_road_mark_color_values(self):
        """Test road mark color enum values."""
        assert RoadMarkColor.WHITE.value == "white"
        assert RoadMarkColor.YELLOW.value == "yellow"
        assert RoadMarkColor.STANDARD.value == "standard"
