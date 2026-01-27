"""Lane implementation for OpenDRIVE conversion."""

from typing import List, Optional, Dict, Union, Any
import lanelet2
import lxml.etree as ET

from ..centerline import estimate_lanelet_width_as_spline, Width1DSplineAdapter
from ..spline import Splines
from ..util import find_adjacent_groups
from ..conversion_config import WidthEstimationConfig, WidthReference

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
        lanelet_id: Optional[int] = None,
    ):
        """
        Initialize a Lane object.

        Args:
            lane_id: Lane ID (negative for right lanes, positive for left lanes)
            lane_type: Type of the lane
            level: Whether the lane is level
            predecessor: Link to predecessor lane
            successor: Link to successor lane
            lanelet_id: ID of the corresponding Lanelet2 lanelet (for tracing connections)
        """
        self.lane_id = lane_id
        self.lane_type = lane_type
        self.level = level
        self.predecessor = predecessor
        self.successor = successor
        self.lanelet_id = lanelet_id

        # Lane geometry definitions
        self.widths: List[LaneWidth] = []
        self.road_marks: List[RoadMark] = []
        self.borders: List[LaneBorder] = []
        self.heights: List[LaneHeight] = []

    def _add_width(self, width: LaneWidth) -> None:
        """Add a width definition to the lane."""
        self.widths.append(width)

    def _add_road_mark(self, road_mark: RoadMark) -> None:
        """Add a road mark to the lane."""
        self.road_marks.append(road_mark)

    def _add_border(self, border: LaneBorder) -> None:
        """Add a border definition to the lane."""
        self.borders.append(border)

    def _add_height(self, height: LaneHeight) -> None:
        """Add a height definition to the lane."""
        self.heights.append(height)

    def _add_width_from_spline(
        self,
        width_spline: Union[Width1DSplineAdapter, Splines],
        road_length: Optional[float] = None,
    ) -> None:
        """
        Add width definitions from a width spline using polynomial coefficients.

        For Width1DSplineAdapter objects (from estimate_lanelet_width_as_spline),
        this extracts the proper cubic polynomial coefficients (a, b, c, d).
        For legacy Splines objects, it falls back to sampling.

        Args:
            width_spline: Width spline object (Width1DSplineAdapter or Splines)
            road_length: Total road length (uses spline length if None)
        """
        if road_length is None:
            road_length = width_spline.total_length

        # Check if this is a Width1DSplineAdapter with polynomial segments
        if hasattr(width_spline, "get_polynomial_segments"):
            # Use proper polynomial coefficients from 1D cubic spline
            segments = width_spline.get_polynomial_segments()

            for s_offset, a, b, c, d in segments:
                # Only add segments within the road length
                if s_offset < road_length:
                    self._add_width(LaneWidth(s_offset=s_offset, a=a, b=b, c=c, d=d))
        else:
            # Fallback for legacy Splines objects - sample at intervals
            num_segments = 10  # Number of segments to create
            segment_length = road_length / num_segments

            for i in range(num_segments):
                s_start = i * segment_length
                s_end = min((i + 1) * segment_length, road_length)

                if s_end <= s_start:
                    continue

                # Evaluate spline at start to get width value
                # For width splines, we use the Y coordinate as the width value
                start_pos = width_spline.evaluate(s_start)
                width_value = start_pos[1]  # Y coordinate contains width

                # Create constant width definition for this segment
                self._add_width(
                    LaneWidth(s_offset=s_start, a=width_value, b=0.0, c=0.0, d=0.0)
                )

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

        # Predecessor and successor will be set later via set_lane_links
        # after all roads are constructed and lanelet-to-lane mappings are available
        predecessor = None
        successor = None

        # Create the Lane instance with the lanelet ID for connection tracing
        lane = Lane(
            lane_id=lane_id,
            lane_type=lane_type,
            level=False,
            predecessor=predecessor,
            successor=successor,
            lanelet_id=lanelet.id,
        )

        groups = find_adjacent_groups(lanelet_map, {lanelet})

        if len(groups) == 1 and len(groups[0]) == 1:
            config = WidthEstimationConfig(reference=WidthReference.LEFT_BOUND)
            width_spline = estimate_lanelet_width_as_spline(lanelet, config)
        else:
            # Calculate lane width using spline curve from estimate_lanelet_width_as_spline
            config = WidthEstimationConfig(reference=WidthReference.CENTER_LINE)
            width_spline = estimate_lanelet_width_as_spline(lanelet, config)

        # Sample the spline at multiple points to create width definitions
        lane._add_width_from_spline(width_spline)
        # TODO: Add road marks based on lanelet line types

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

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("lane")
        elem.set("id", str(self.lane_id))
        elem.set("type", self.lane_type.value)
        elem.set("level", "true" if self.level else "false")

        # Add predecessor and successor links if available
        if self.predecessor or self.successor:
            link_elem = ET.SubElement(elem, "link")
            if self.predecessor:
                pred_elem = ET.SubElement(link_elem, "predecessor")
                pred_elem.set("id", str(self.predecessor.id))
            if self.successor:
                succ_elem = ET.SubElement(link_elem, "successor")
                succ_elem.set("id", str(self.successor.id))

        # Add width definitions
        for width in self.widths:
            elem.append(width.to_xml())

        # Add road marks
        for road_mark in self.road_marks:
            elem.append(road_mark.to_xml())

        # Add borders
        for border in self.borders:
            elem.append(border.to_xml())

        # Add heights
        for height in self.heights:
            elem.append(height.to_xml())

        return elem

    def __repr__(self) -> str:
        """String representation of the lane."""
        return (
            f"Lane(id={self.lane_id}, type={self.lane_type.value}, "
            f"widths={len(self.widths)}, marks={len(self.road_marks)})"
        )
