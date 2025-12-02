"""Lane implementation for OpenDRIVE conversion."""

from typing import List, Optional, Any, Dict
import numpy as np
import lanelet2

from scenariogeneration import xodr
from ..centerline import estimate_lanelet_width_as_spline
from ..geometry import ArcLengthParameterizedCatmullRomSpline

from .opendrive_dataclass import (
    LaneType,
    LaneWidth,
    RoadMark,
    LaneLink,
    LaneBorder,
    LaneHeight,
)


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

    @staticmethod
    def construct_from_lanelet(
        lanelet_map: lanelet2.core.LaneletMap, lanelet: lanelet2.core.Lanelet
    ) -> "Lane":
        """
        Construct a Lane from a Lanelet2 lanelet.

        Args:
            lanelet_map: The Lanelet2 map containing the lanelet
            lanelet: The lanelet to convert to Lane

        Returns:
            Lane instance constructed from the lanelet
        """
        # TODO: Determine lane ID based on lanelet position and direction
        lane_id = 0

        # Determine lane type from lanelet attributes
        if "subtype" in lanelet.attributes:
            subtype = lanelet.attributes["subtype"]
            if subtype in ["road", "highway"]:
                lane_type = LaneType.DRIVING
            elif subtype == "walkway":
                lane_type = LaneType.SIDEWALK
            elif subtype == "bicycle_lane":
                lane_type = LaneType.BIKING
            else:
                lane_type = LaneType.DRIVING
        else:
            lane_type = LaneType.DRIVING

        # TODO: Calculate predecessor and successor from lanelet connections
        predecessor = None
        successor = None

        # Create the Lane instance
        lane = Lane(
            lane_id=lane_id,
            lane_type=lane_type,
            level=False,
            predecessor=predecessor,
            successor=successor,
        )

        # Calculate lane width using spline curve from estimate_lanelet_width_as_spline
        try:
            width_spline = estimate_lanelet_width_as_spline(lanelet)

            # Sample the spline at multiple points to create width definitions
            total_length = width_spline.total_length
            num_samples = 10

            for i in range(num_samples):
                s_offset = (
                    (i / (num_samples - 1)) * total_length if num_samples > 1 else 0.0
                )
                width_value = width_spline.evaluate(s_offset)["position"][
                    1
                ]  # y-component is width

                # Add width definition at this s-coordinate
                lane.add_width(LaneWidth(s_offset=s_offset, a=width_value))

        except Exception:
            # Fallback to simple average width calculation if spline method fails
            left_bound = lanelet.leftBound
            right_bound = lanelet.rightBound

            if len(left_bound) > 0 and len(right_bound) > 0:
                # Calculate average width along the lanelet
                widths = []
                for i in range(min(len(left_bound), len(right_bound))):
                    left_point = left_bound[i]
                    right_point = right_bound[i]
                    width = np.linalg.norm(
                        [left_point.x - right_point.x, left_point.y - right_point.y]
                    )
                    widths.append(width)

                if widths:
                    avg_width = np.mean(widths)
                    lane.add_constant_width(avg_width)

        # TODO: Add road marks based on lanelet line types

        return lane

    def add_constant_width(self, width: float, s_start: float = 0.0) -> None:
        """Add a constant width definition."""
        self.add_width(LaneWidth(s_offset=s_start, a=width))

    def add_width_from_spline(
        self,
        width_spline: ArcLengthParameterizedCatmullRomSpline,
        num_samples: int = 10,
        road_length: Optional[float] = None,
    ) -> None:
        """
        Add width definitions from an arc length parameterized spline using cubic parameters.

        Args:
            width_spline: ArcLengthParameterizedCatmullRomSpline for width
            num_samples: Number of width samples (used for sampling within each segment)
            road_length: Total road length (uses spline length if None)
        """
        if road_length is None:
            road_length = width_spline.total_length

        # Get cubic spline parameters for each segment
        segments = width_spline.as_cubic_spline_parameters()

        for segment in segments:
            s_start = segment["s_start"]
            s_end = segment["s_end"]
            a, b, c, d = segment["a"], segment["b"], segment["c"], segment["d"]

            # Clamp segment to road_length
            if s_start >= road_length:
                break
            s_end = min(s_end, road_length)

            # Sample points within this segment
            segment_length = s_end - s_start
            if segment_length <= 0:
                continue

            # Calculate number of samples for this segment based on its relative length
            segment_samples = max(1, int(num_samples * segment_length / road_length))
            s_values = np.linspace(s_start, s_end, segment_samples + 1)

            for s in s_values:
                # Calculate local coordinate relative to segment start
                local_s = s - s_start

                # Evaluate cubic polynomial: width = a + b*s + c*s^2 + d*s^3
                width = a + b * local_s + c * local_s**2 + d * local_s**3

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
