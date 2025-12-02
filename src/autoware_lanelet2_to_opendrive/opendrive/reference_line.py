"""ReferenceLine implementation for OpenDRIVE conversion."""

import lanelet2
from typing import Union, List, Set
from .lane import Lane
from .opendrive_dataclass import LaneType, LaneWidth
from ..geometry import ArcLengthParameterizedCatmullRomSpline


class ReferenceLine(Lane):
    """
    OpenDRIVE reference line representation that inherits from Lane.

    The reference line is the center line of a road and serves as the basis
    for defining lane geometry in OpenDRIVE format. It inherits all lane
    functionality but is specifically designed for reference line purposes.
    """

    calculated_centerline_spline: ArcLengthParameterizedCatmullRomSpline

    def __init__(self, centerline_spline: ArcLengthParameterizedCatmullRomSpline):
        self.centerline_spline = centerline_spline
        """
        Initialize a ReferenceLine object.
        """
        # Reference line always has lane_id = 0 and is always a driving type
        super().__init__(
            lane_id=0,
            lane_type=LaneType.DRIVING,
            level=False,
        )

    @staticmethod
    def construct_from_lanelet_groups(
        lanelet_map: lanelet2.core.LaneletMap,
        lanelet_group: Union[
            Set[lanelet2.core.Lanelet],
            List[lanelet2.core.Lanelet],
            lanelet2.core.LaneletLayer,
        ],
    ) -> "ReferenceLine":
        """
        Construct a ReferenceLine from a group of Lanelet2 lanelets.

        Args:
            lanelet_map: The Lanelet2 map containing the lanelets
            lanelet_group: List of lanelets representing lanes in a road

        Returns:
            ReferenceLine instance constructed from the center of the lanelet group
        """
        if not lanelet_group:
            raise ValueError("Lanelet group cannot be empty")

        # Sort the lanelets from left to right
        from ..util import sort_adjacent_groups

        sorted_lanelets = sort_adjacent_groups(lanelet_map, lanelet_group)

        num_lanes = len(sorted_lanelets)

        # Calculate centerline spline based on the number of lanes
        if num_lanes % 2 == 1:
            # Odd number of lanes: use the center lane's centerline
            center_lane_index = num_lanes // 2
            center_lanelet = sorted_lanelets[center_lane_index]

            from ..centerline import extract_centerline_as_spline

            centerline_spline = extract_centerline_as_spline(center_lanelet)
        else:
            # Even number of lanes: use the line between X/2 and X/2+1 lanes
            left_lane_index = num_lanes // 2 - 1  # X/2 (0-indexed)
            right_lane_index = num_lanes // 2  # X/2+1 (0-indexed)

            two_lanelets = {
                sorted_lanelets[left_lane_index],
                sorted_lanelets[right_lane_index],
            }

            from ..centerline import extract_centerline_as_spline_from_two_lanelets

            centerline_spline = extract_centerline_as_spline_from_two_lanelets(
                lanelet_map, two_lanelets
            )

        # Create the ReferenceLine instance
        reference_line = ReferenceLine(centerline_spline=centerline_spline)

        # Reference line is a virtual line, so I hardcode a small constant width
        reference_line._add_width(LaneWidth(s_offset=0, a=0.1))

        # TODO: Add road marks based on lanelet line types

        return reference_line

    def __repr__(self) -> str:
        """String representation of the reference line."""
        return f"ReferenceLine(widths={len(self.widths)}, marks={len(self.road_marks)})"
