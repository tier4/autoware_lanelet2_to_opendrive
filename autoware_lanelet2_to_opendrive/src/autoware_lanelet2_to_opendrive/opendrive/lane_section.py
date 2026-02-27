"""LaneSection implementation for OpenDRIVE conversion."""

from typing import List, Optional, Dict, Union, Set, TYPE_CHECKING
import lanelet2
import lxml.etree as ET
from lanelet2.routing import RoutingGraph

if TYPE_CHECKING:
    from .lane import Lane
from .reference_line import ReferenceLine
from ..conversion_config import WidthEstimationConfig
from .enums import RoadMarkType, RoadMarkColor, RoadMarkLaneChange
from .lane_elements import RoadMark


class LaneSection:
    """
    OpenDRIVE LaneSection representation.

    A LaneSection contains lanes for a specific longitudinal section of a road.
    It includes left lanes, right lanes, and a center (reference) lane.
    """

    def __init__(
        self,
        s_offset: float = 0.0,
    ):
        """
        Initialize a LaneSection object.

        Args:
            s_offset: Start position of the lane section along the reference line
        """
        self.s_offset = s_offset

        # Lanes are stored by ID: negative for right, positive for left, 0 for center
        self.left_lanes: Dict[int, "Lane"] = {}  # Positive IDs
        self.center_lane: Optional[ReferenceLine] = None  # ID = 0
        self.right_lanes: Dict[int, "Lane"] = {}  # Negative IDs

        # Lane offset for single lane sections
        self.lane_offset: Optional[Dict[str, float]] = None

    def _add_left_lane(self, lane: "Lane") -> None:
        """Add a left lane to the section."""
        if lane.lane_id <= 0:
            raise ValueError(f"Left lane must have positive ID, got {lane.lane_id}")
        self.left_lanes[lane.lane_id] = lane

    def _add_right_lane(self, lane: "Lane") -> None:
        """Add a right lane to the section."""
        if lane.lane_id >= 0:
            raise ValueError(f"Right lane must have negative ID, got {lane.lane_id}")
        self.right_lanes[lane.lane_id] = lane

    def _set_center_lane(self, reference_line: ReferenceLine) -> None:
        """Set the center/reference lane."""
        if reference_line.lane_id != 0:
            raise ValueError(
                f"Center lane must have ID 0, got {reference_line.lane_id}"
            )
        self.center_lane = reference_line

    @staticmethod
    def construct_from_lanelet_groups(
        lanelet_map: lanelet2.core.LaneletMap,
        lanelet_group: Union[
            Set[lanelet2.core.Lanelet],
            List[lanelet2.core.Lanelet],
            lanelet2.core.LaneletLayer,
        ],
        s_offset: float = 0.0,
        traffic_rule: Optional[str] = None,
        width_config: Optional[WidthEstimationConfig] = None,
        routing_graph: Optional[RoutingGraph] = None,
    ) -> "LaneSection":
        """
        Construct a LaneSection from a group of Lanelet2 lanelets.

        Args:
            lanelet_map: The Lanelet2 map containing the lanelets
            lanelet_group: Group of lanelets representing lanes in a road section
            s_offset: Start position of the lane section
            traffic_rule: Traffic rule for lanes (RHT or LHT)
            width_config: Configuration for width spline sampling
            routing_graph: Optional pre-built routing graph for lane-change detection.
                If None, creates a new one.

        Returns:
            LaneSection instance constructed from the lanelet group
        """
        if not lanelet_group:
            raise ValueError("Lanelet group cannot be empty")

        # Create the LaneSection instance
        lane_section = LaneSection(s_offset=s_offset)

        # Import Lane here to avoid circular import
        from .lane import Lane

        # Sort lanelets from left to right
        from ..util import sort_adjacent_groups

        # Convert to set if needed
        if isinstance(lanelet_group, set):
            lanelet_set = lanelet_group
        elif isinstance(lanelet_group, list):
            lanelet_set = set(lanelet_group)
        else:  # lanelet2.core.LaneletLayer
            lanelet_set = set(lanelet_group)

        # Use provided routing graph or create a new one
        if routing_graph is None:
            from ..util import create_routing_graph

            routing_graph = create_routing_graph(lanelet_map)

        sorted_lanelets = sort_adjacent_groups(lanelet_map, lanelet_set, routing_graph)

        # Create and set the reference line
        reference_line = ReferenceLine.construct_from_lanelet_groups(
            lanelet_map, lanelet_group, traffic_rule=traffic_rule
        )
        lane_section._set_center_lane(reference_line)

        # Normalize traffic_rule to uppercase, default to RHT
        traffic_rule_normalized = (traffic_rule or "RHT").upper()

        # Validate traffic_rule
        if traffic_rule_normalized not in ("RHT", "LHT"):
            raise ValueError(
                f"Invalid traffic_rule: '{traffic_rule}'. Must be 'RHT' or 'LHT'."
            )

        # Extract reference line spline for width calculation
        reference_line_spline = reference_line.centerline_2d

        # Create lanes and collect them for road mark assignment
        lanes_built: List["Lane"] = []

        # Create lanes based on traffic rule (RHT and LHT use the same structure)
        for i, lanelet in enumerate(sorted_lanelets):
            lane_id = -(i + 1)  # -1, -2, -3, ...
            lane = Lane.construct_from_lanelet(
                lanelet_map,
                lanelet,
                rule=traffic_rule_normalized,
                width_config=width_config,
                reference_line_spline=reference_line_spline,
            )
            lane.lane_id = lane_id
            lane_section._add_right_lane(lane)
            lanes_built.append(lane)

        # Assign road marks based on lane-change permission.
        #
        # In OpenDRIVE, the road mark on a negative-ID lane describes its INNER
        # (left/center-side) boundary:
        #   lane -1  road mark  = boundary between center (0) and lane -1
        #   lane -2  road mark  = boundary between lane -1 and lane -2
        #   lane -(i+1) road mark = boundary between lane -i and lane -(i+1)
        #
        # Therefore:
        #   i == 0  (lane -1, innermost): center line — always solid, no lane change
        #   i  > 0  (lane -(i+1)): check whether sorted_lanelets[i-1] can change right
        for i, lane in enumerate(lanes_built):
            if i == 0:
                # Innermost lane: road mark describes the center-to-lane boundary.
                # The center lane is a reference line (type="none"), so no lane change.
                mark_type = RoadMarkType.SOLID
                lane_change = RoadMarkLaneChange.NONE
            else:
                # Lane -(i+1): road mark describes the boundary between lane -i and
                # lane -(i+1).  Check whether the i-th lanelet can change rightward.
                can_change = routing_graph.right(sorted_lanelets[i - 1]) is not None
                mark_type = RoadMarkType.BROKEN if can_change else RoadMarkType.SOLID
                lane_change = (
                    RoadMarkLaneChange.BOTH if can_change else RoadMarkLaneChange.NONE
                )
            lane._add_road_mark(
                RoadMark(
                    s_offset=0.0,
                    type=mark_type,
                    color=RoadMarkColor.STANDARD,
                    lane_change=lane_change,
                )
            )

        return lane_section

    def get_all_lanes(self) -> List["Lane"]:
        """Get all lanes in the section (left + center + right)."""
        all_lanes = []

        # Add left lanes (sorted)
        for lane_id in sorted(self.left_lanes.keys()):
            all_lanes.append(self.left_lanes[lane_id])

        # Add center lane
        if self.center_lane:
            all_lanes.append(self.center_lane._lane)

        # Add right lanes (sorted)
        for lane_id in sorted(self.right_lanes.keys(), reverse=True):
            all_lanes.append(self.right_lanes[lane_id])

        return all_lanes

    def get_lanelet_to_lane_mapping(self) -> Dict[int, int]:
        """Get mapping from lanelet ID to lane ID for all lanes in this section.

        Returns:
            Dictionary mapping lanelet_id -> lane_id
        """
        mapping: Dict[int, int] = {}

        # Add left lanes
        for lane in self.left_lanes.values():
            if lane.lanelet_id is not None:
                mapping[lane.lanelet_id] = lane.lane_id

        # Add right lanes
        for lane in self.right_lanes.values():
            if lane.lanelet_id is not None:
                mapping[lane.lanelet_id] = lane.lane_id

        return mapping

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("laneSection")
        elem.set("s", str(self.s_offset))

        # Add laneOffset if present (for single lane sections)
        if self.lane_offset:
            offset_elem = ET.SubElement(elem, "laneOffset")
            offset_elem.set("s", str(self.lane_offset["s"]))
            offset_elem.set("a", str(self.lane_offset["a"]))
            offset_elem.set("b", str(self.lane_offset["b"]))
            offset_elem.set("c", str(self.lane_offset["c"]))
            offset_elem.set("d", str(self.lane_offset["d"]))

        # Add left lanes
        if self.left_lanes:
            left_elem = ET.SubElement(elem, "left")
            for lane_id in sorted(self.left_lanes.keys()):
                left_elem.append(self.left_lanes[lane_id].to_xml())

        # Add center lane
        if self.center_lane:
            center_elem = ET.SubElement(elem, "center")
            center_elem.append(self.center_lane.to_xml())

        # Add right lanes
        if self.right_lanes:
            right_elem = ET.SubElement(elem, "right")
            for lane_id in sorted(self.right_lanes.keys(), reverse=True):
                right_elem.append(self.right_lanes[lane_id].to_xml())

        return elem

    def __repr__(self) -> str:
        """String representation of the lane section."""
        return (
            f"LaneSection(s_offset={self.s_offset}, "
            f"left_lanes={len(self.left_lanes)}, "
            f"center_lane={'Yes' if self.center_lane else 'No'}, "
            f"right_lanes={len(self.right_lanes)})"
        )
