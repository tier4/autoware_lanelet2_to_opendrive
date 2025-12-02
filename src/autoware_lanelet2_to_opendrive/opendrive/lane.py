"""Lane implementation for OpenDRIVE conversion."""

from dataclasses import dataclass
from enum import Enum
from typing import List, Optional, Any, Dict
import numpy as np

from scenariogeneration import xodr


class LaneType(Enum):
    """OpenDRIVE lane types."""

    NONE = "none"
    DRIVING = "driving"
    STOP = "stop"
    SHOULDER = "shoulder"
    BIKING = "biking"
    SIDEWALK = "sidewalk"
    BORDER = "border"
    RESTRICTED = "restricted"
    PARKING = "parking"
    BIDIRECTIONAL = "bidirectional"
    MEDIAN = "median"
    SPECIAL1 = "special1"
    SPECIAL2 = "special2"
    SPECIAL3 = "special3"
    ROAD_WORKS = "roadWorks"
    TRAM = "tram"
    RAIL = "rail"
    ENTRY = "entry"
    EXIT = "exit"
    OFF_RAMP = "offRamp"
    ON_RAMP = "onRamp"
    CONNECTING_RAMP = "connectingRamp"
    BUS = "bus"
    TAXI = "taxi"
    HOV = "hov"


class RoadMarkType(Enum):
    """OpenDRIVE road mark types."""

    NONE = "none"
    SOLID = "solid"
    BROKEN = "broken"
    SOLID_SOLID = "solid solid"
    SOLID_BROKEN = "solid broken"
    BROKEN_SOLID = "broken solid"
    BROKEN_BROKEN = "broken broken"
    BOTTS_DOTS = "botts dots"
    REFLECTORS = "reflectors"
    GRASS = "grass"
    COBBLE = "cobble"
    PAVING_STONES = "paving stones"
    PAVING = "paving"
    CONCRETE = "concrete"
    ASPHALT = "asphalt"


class RoadMarkColor(Enum):
    """OpenDRIVE road mark colors."""

    STANDARD = "standard"
    BLUE = "blue"
    GREEN = "green"
    RED = "red"
    WHITE = "white"
    YELLOW = "yellow"
    ORANGE = "orange"


class LaneChangeType(Enum):
    """OpenDRIVE lane change types."""

    INCREASE = "increase"
    DECREASE = "decrease"
    BOTH = "both"
    NONE = "none"


@dataclass
class LaneWidth:
    """Lane width definition."""

    s_offset: float
    a: float  # Width at s_offset
    b: float = 0.0  # Linear coefficient
    c: float = 0.0  # Quadratic coefficient
    d: float = 0.0  # Cubic coefficient


@dataclass
class RoadMark:
    """Road mark definition."""

    s_offset: float
    type: RoadMarkType
    weight: str = "standard"
    color: RoadMarkColor = RoadMarkColor.STANDARD
    width: float = 0.12
    lane_change: LaneChangeType = LaneChangeType.BOTH


@dataclass
class LaneLink:
    """Lane link definition for predecessors and successors."""

    id: int


@dataclass
class LaneBorder:
    """Lane border definition."""

    s_offset: float
    a: float  # Border position at s_offset
    b: float = 0.0  # Linear coefficient
    c: float = 0.0  # Quadratic coefficient
    d: float = 0.0  # Cubic coefficient


@dataclass
class LaneHeight:
    """Lane height definition."""

    s_offset: float
    inner: float
    outer: float


class Lane:
    """
    OpenDRIVE lane representation with scenariogeneration support.

    This class represents a single lane in OpenDRIVE format and provides
    methods to convert to XODR XML using the scenariogeneration library.
    """

    def __init__(
        self,
        lane_id: int,
        lane_type: LaneType,
        level: bool = False,
        predecessor: Optional[LaneLink] = None,
        successor: Optional[LaneLink] = None,
    ):
        """
        Initialize a Lane object.

        Args:
            lane_id: Lane ID (negative for right lanes, positive for left lanes)
            lane_type: Type of the lane
            level: Whether the lane is level
            predecessor: Link to predecessor lane
            successor: Link to successor lane
        """
        self.lane_id = lane_id
        self.lane_type = lane_type
        self.level = level
        self.predecessor = predecessor
        self.successor = successor

        # Lane geometry definitions
        self.widths: List[LaneWidth] = []
        self.road_marks: List[RoadMark] = []
        self.borders: List[LaneBorder] = []
        self.heights: List[LaneHeight] = []

    def add_width(self, width: LaneWidth) -> None:
        """Add a width definition to the lane."""
        self.widths.append(width)

    def add_road_mark(self, road_mark: RoadMark) -> None:
        """Add a road mark to the lane."""
        self.road_marks.append(road_mark)

    def add_border(self, border: LaneBorder) -> None:
        """Add a border definition to the lane."""
        self.borders.append(border)

    def add_height(self, height: LaneHeight) -> None:
        """Add a height definition to the lane."""
        self.heights.append(height)

    def add_constant_width(self, width: float, s_start: float = 0.0) -> None:
        """Add a constant width definition."""
        self.add_width(LaneWidth(s_offset=s_start, a=width))

    def add_width_from_spline(
        self, width_spline, num_samples: int = 10, road_length: Optional[float] = None
    ) -> None:
        """
        Add width definitions from an arc length parameterized spline.

        Args:
            width_spline: ArcLengthParameterizedCatmullRomSpline for width
            num_samples: Number of width samples
            road_length: Total road length (uses spline length if None)
        """
        if road_length is None:
            road_length = width_spline.total_length

        # Sample width at regular intervals
        s_values = np.linspace(0, road_length, num_samples)

        for s in s_values:
            # Evaluate width at arc length s
            # The spline returns [s, width], we want the width (index 1)
            if s <= width_spline.total_length:
                width_point = width_spline.evaluate(s).flatten()
                width = width_point[1] if len(width_point) > 1 else width_point[0]
            else:
                # Use last available width if beyond spline range
                width_point = width_spline.evaluate(width_spline.total_length).flatten()
                width = width_point[1] if len(width_point) > 1 else width_point[0]

            self.add_width(LaneWidth(s_offset=s, a=width))

    def to_xodr_roadmark(self) -> List[Any]:
        """
        Create XODR RoadMark objects from this lane's road marks.

        Returns:
            List of XODR RoadMark objects
        """
        road_marks = []
        for road_mark in self.road_marks:
            # Convert enum values to scenariogeneration enums
            mark_type = getattr(
                xodr.enumerations.RoadMarkType, road_mark.type.value, None
            )
            mark_color = getattr(
                xodr.enumerations.RoadMarkColor, road_mark.color.value, None
            )

            if mark_type is None:
                mark_type = xodr.enumerations.RoadMarkType.solid
            if mark_color is None:
                mark_color = xodr.enumerations.RoadMarkColor.standard

            xodr_mark = xodr.RoadMark(
                marking_type=mark_type, color=mark_color, soffset=road_mark.s_offset
            )
            road_marks.append(xodr_mark)

        return road_marks

    def to_standard_lane(self, lane_width: Optional[float] = None) -> Any:
        """
        Convert to standard scenariogeneration lane.

        Args:
            lane_width: Width for the lane (uses first width if None)

        Returns:
            Standard Lane object from scenariogeneration
        """
        # Determine lane width
        if lane_width is None:
            if self.widths:
                lane_width = self.widths[0].a
            else:
                lane_width = 3.5  # Default width

        # Create road mark
        if self.road_marks:
            road_mark = self.to_xodr_roadmark()[0]  # Use first road mark
        else:
            road_mark = xodr.RoadMark(xodr.enumerations.RoadMarkType.solid)

        # Create standard lane
        lane = xodr.standard_lane(offset=lane_width, rm=road_mark)

        return lane

    def to_lane_def_data(self) -> Dict[str, Any]:
        """
        Convert lane data for use with LaneDef.

        Returns:
            Dictionary with lane width data
        """
        widths = []
        for width in self.widths:
            widths.append(width.a)

        return {
            "lane_id": self.lane_id,
            "widths": widths if widths else [3.5],
            "lane_type": self.lane_type.value,
        }

    def __repr__(self) -> str:
        """String representation of the lane."""
        return (
            f"Lane(id={self.lane_id}, type={self.lane_type.value}, "
            f"widths={len(self.widths)}, marks={len(self.road_marks)})"
        )


class LaneSection:
    """
    OpenDRIVE lane section containing multiple lanes.
    """

    def __init__(self, s_start: float = 0.0):
        """
        Initialize a lane section.

        Args:
            s_start: Start position along the road
        """
        self.s_start = s_start
        self.left_lanes: List[Lane] = []
        self.center_lane: Optional[Lane] = None
        self.right_lanes: List[Lane] = []

    def add_left_lane(self, lane: Lane) -> None:
        """Add a left lane to the section."""
        self.left_lanes.append(lane)

    def add_right_lane(self, lane: Lane) -> None:
        """Add a right lane to the section."""
        self.right_lanes.append(lane)

    def set_center_lane(self, lane: Lane) -> None:
        """Set the center lane."""
        self.center_lane = lane

    def to_lane_def(self, s_end: float) -> Any:
        """
        Convert lane section to LaneDef for scenariogeneration.

        Args:
            s_end: End position of the lane section

        Returns:
            LaneDef object for use with scenariogeneration road creation
        """
        # Count lanes
        n_lanes = len(self.left_lanes) + len(self.right_lanes)

        # Collect lane widths (right lanes have negative IDs, left lanes positive)
        widths = []

        # Add right lanes (negative IDs, in reverse order for scenariogeneration)
        for lane in sorted(self.right_lanes, key=lambda x: x.lane_id):
            if lane.widths:
                widths.append(lane.widths[0].a)
            else:
                widths.append(3.5)

        # Add left lanes (positive IDs)
        for lane in sorted(self.left_lanes, key=lambda x: x.lane_id):
            if lane.widths:
                widths.append(lane.widths[0].a)
            else:
                widths.append(3.5)

        # Create LaneDef
        lane_def = xodr.LaneDef(
            s_start=self.s_start,
            s_end=s_end,
            n_lanes_start=n_lanes,
            n_lanes_end=n_lanes,
            lane_start_widths=widths,
            lane_end_widths=widths,
        )

        return lane_def

    def get_lane_data(self) -> Dict[str, Any]:
        """
        Get lane data for manual XODR construction.

        Returns:
            Dictionary with organized lane data
        """
        data: Dict[str, Any] = {
            "s_start": self.s_start,
            "left_lanes": [],
            "right_lanes": [],
            "center_lane": None,
        }

        for lane in self.left_lanes:
            data["left_lanes"].append(lane.to_lane_def_data())

        for lane in self.right_lanes:
            data["right_lanes"].append(lane.to_lane_def_data())

        if self.center_lane:
            data["center_lane"] = self.center_lane.to_lane_def_data()

        return data

    def __repr__(self) -> str:
        """String representation of the lane section."""
        return (
            f"LaneSection(s={self.s_start}, "
            f"left={len(self.left_lanes)}, "
            f"center={1 if self.center_lane else 0}, "
            f"right={len(self.right_lanes)})"
        )


def create_driving_lane_from_lanelet(
    lane_id: int,
    width_spline,
    road_length: Optional[float] = None,
    num_width_samples: int = 10,
) -> Lane:
    """
    Create a driving lane from a lanelet with width spline.

    Args:
        lane_id: Lane ID for the OpenDRIVE lane
        width_spline: ArcLengthParameterizedCatmullRomSpline for width
        road_length: Total road length
        num_width_samples: Number of width samples

    Returns:
        Configured Lane object
    """
    lane = Lane(lane_id=lane_id, lane_type=LaneType.DRIVING)

    # Add width from spline
    lane.add_width_from_spline(
        width_spline=width_spline,
        num_samples=num_width_samples,
        road_length=road_length,
    )

    # Add default road marking
    lane.add_road_mark(
        RoadMark(s_offset=0.0, type=RoadMarkType.SOLID, color=RoadMarkColor.WHITE)
    )

    return lane
