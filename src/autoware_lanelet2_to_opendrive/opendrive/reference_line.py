"""ReferenceLine implementation for OpenDRIVE conversion."""

import numpy as np
import lanelet2
from typing import Union, List, Set, TYPE_CHECKING
from ..spline import Splines
from ..cubic_spline_1d import CubicSpline1D

if TYPE_CHECKING:
    pass

# Import enums directly to avoid circular import
from .enums import LaneType
from .lane_elements import LaneWidth


class ReferenceLine:
    """
    OpenDRIVE reference line representation.

    The reference line is the center line of a road and serves as the basis
    for defining lane geometry in OpenDRIVE format. It contains a Lane
    instance for compatibility but is specifically designed for reference line purposes.
    """

    def __init__(self, centerline_spline: Splines):
        self.centerline_spline: Splines = centerline_spline

        # Sample centerline_spline at 1m intervals to create elevation spline
        total_length = centerline_spline.total_length

        # Create arc length samples at 1m intervals
        num_samples = max(2, int(np.ceil(total_length)) + 1)
        arc_lengths = np.linspace(0, total_length, num_samples)

        # Extract elevation (z coordinate) at each arc length
        elevations = np.array([centerline_spline.evaluate(s)[2] for s in arc_lengths])

        # Create CubicSpline1D for elevation as a function of arc length
        self.elevation_spline = CubicSpline1D(
            arc_lengths, elevations, bc_type="not-a-knot"
        )

        # Create a Lane instance for the reference line
        # Import here to avoid circular import
        from .lane import Lane

        # Reference line always has lane_id = 0 and type = none for center lane
        self._lane = Lane(
            lane_id=0,
            lane_type=LaneType.NONE,
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

        # Always use the left boundary of the leftmost lanelet as reference line
        leftmost_lanelet = sorted_lanelets[0]

        from ..centerline import extract_border_from_spline

        centerline_spline = extract_border_from_spline(leftmost_lanelet, border="left")

        # Create the ReferenceLine instance
        reference_line = ReferenceLine(centerline_spline=centerline_spline)

        # Reference line is a virtual line, so I hardcode a small constant width
        reference_line._lane._add_width(LaneWidth(s_offset=0, a=0.1))

        # TODO: Add road marks based on lanelet line types

        return reference_line

    @property
    def widths(self):
        """Access to lane widths."""
        return self._lane.widths

    @property
    def road_marks(self):
        """Access to road marks."""
        return self._lane.road_marks

    @property
    def lane_id(self):
        """Access to lane ID."""
        return self._lane.lane_id

    @property
    def lane_type(self):
        """Access to lane type."""
        return self._lane.lane_type

    @property
    def level(self):
        """Access to lane level."""
        return self._lane.level

    def to_xml(self):
        """Convert to XML element via the internal lane."""
        return self._lane.to_xml()

    def __repr__(self) -> str:
        """String representation of the reference line."""
        return f"ReferenceLine(widths={len(self.widths)}, marks={len(self.road_marks)})"
