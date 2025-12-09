"""LaneSection implementation for OpenDRIVE conversion."""

from typing import List, Optional, Dict, Union, Set, TYPE_CHECKING
import lanelet2
import lxml.etree as ET
from scenariogeneration import xodr

if TYPE_CHECKING:
    from .lane import Lane
from .reference_line import ReferenceLine


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
    ) -> "LaneSection":
        """
        Construct a LaneSection from a group of Lanelet2 lanelets.

        Args:
            lanelet_map: The Lanelet2 map containing the lanelets
            lanelet_group: Group of lanelets representing lanes in a road section
            s_offset: Start position of the lane section

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

        sorted_lanelets = sort_adjacent_groups(lanelet_map, lanelet_set)
        num_lanes = len(sorted_lanelets)

        # Create and set the reference line
        reference_line = ReferenceLine.construct_from_lanelet_groups(
            lanelet_map, lanelet_group
        )
        lane_section._set_center_lane(reference_line)

        # Determine the center position for lane ID assignment
        # For single lane: treat as right lane with reference line as left boundary
        # For odd number of lanes: center lane gets ID closest to 0
        # For even number: lanes are split evenly between left and right
        if num_lanes == 1:
            # Single lane: use the lanelet as a right lane (ID = -1)
            # The left boundary is used as the reference line (via ReferenceLine.construct_from_lanelet_groups)
            single_lanelet = sorted_lanelets[0]
            lane = Lane.construct_from_lanelet(lanelet_map, single_lanelet)
            lane.lane_id = -1
            lane_section._add_right_lane(lane)

            # No lane offset needed since we use left boundary as reference line
            # The lane width will be measured from the left boundary (reference line)
            # to the right boundary
        elif num_lanes % 2 == 1:
            # Odd number of lanes
            center_index = num_lanes // 2

            # Assign lane IDs
            for i, lanelet in enumerate(sorted_lanelets):
                if i < center_index:
                    # Left lanes (positive IDs, starting from center outward)
                    lane_id = center_index - i
                    lane = Lane.construct_from_lanelet(lanelet_map, lanelet)
                    lane.lane_id = lane_id
                    lane_section._add_left_lane(lane)
                elif i > center_index:
                    # Right lanes (negative IDs, starting from center outward)
                    lane_id = center_index - i  # This will be negative
                    lane = Lane.construct_from_lanelet(lanelet_map, lanelet)
                    lane.lane_id = lane_id
                    lane_section._add_right_lane(lane)
                # Skip center lane (i == center_index) as it's represented by the reference line
        else:
            # Even number of lanes
            mid_point = num_lanes // 2

            for i, lanelet in enumerate(sorted_lanelets):
                if i < mid_point:
                    # Left lanes
                    lane_id = mid_point - i
                    lane = Lane.construct_from_lanelet(lanelet_map, lanelet)
                    lane.lane_id = lane_id
                    lane_section._add_left_lane(lane)
                else:
                    # Right lanes (ensure negative ID)
                    lane_id = mid_point - i - 1  # This will be -1, -2, etc.
                    lane = Lane.construct_from_lanelet(lanelet_map, lanelet)
                    lane.lane_id = lane_id
                    lane_section._add_right_lane(lane)

        return lane_section

    def to_standard_lane_section(self) -> xodr.LaneSection:
        """
        Convert to scenariogeneration LaneSection object.

        Returns:
            xodr.LaneSection instance
        """
        # Convert center lane to standard lane
        if self.center_lane:
            center_lane_std = self.center_lane.to_standard_lane()
        else:
            # Create a minimal center lane if not present
            center_lane_std = xodr.Lane(lane_id=0)

        # Create the standard lane section with center lane
        standard_section = xodr.LaneSection(s=self.s_offset, centerlane=center_lane_std)

        # Add left lanes (sorted by ID)
        for lane_id in sorted(self.left_lanes.keys()):
            lane_std = self.left_lanes[lane_id].to_standard_lane()
            standard_section.add_left_lane(lane_std)

        # Add right lanes (sorted by ID in reverse for proper order)
        for lane_id in sorted(self.right_lanes.keys(), reverse=True):
            lane_std = self.right_lanes[lane_id].to_standard_lane()
            standard_section.add_right_lane(lane_std)

        return standard_section

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
