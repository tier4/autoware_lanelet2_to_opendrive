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
from .elevation import ElevationProfile, Elevation


class ReferenceLine:
    """
    OpenDRIVE reference line representation.

    The reference line is the center line of a road and serves as the basis
    for defining lane geometry in OpenDRIVE format. It contains a Lane
    instance for compatibility but is specifically designed for reference line purposes.
    """

    def __init__(self, centerline_spline: Splines):
        self.centerline_spline: Splines = centerline_spline

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

    def get_elevation_profile(self) -> ElevationProfile:
        """
        Get elevation profile from the reference line by directly sampling centerline_spline.

        IMPORTANT: This method uses XY-plane arc length (2D projection) to match ParamPoly3
        geometry coordinates. The 3D spline's natural arc length is NOT used because
        ParamPoly3.from_spline() only uses XY coordinates (ignoring Z), which causes
        misalignment on slopes.

        Returns:
            ElevationProfile object containing elevation segments with polynomial coefficients
        """
        # Sample centerline_spline at high density for accurate elevation capture
        total_length_3d = self.centerline_spline.total_length

        # Create 3D arc length samples at 0.1m intervals for high precision
        # Use minimum of 10 samples to ensure good spline quality even for short roads
        num_samples = max(10, int(np.ceil(total_length_3d / 0.1)) + 1)
        arc_lengths_3d = np.linspace(0, total_length_3d, num_samples)

        # Extract 3D coordinates at each 3D arc length
        points_3d = np.array(
            [self.centerline_spline.evaluate(s) for s in arc_lengths_3d]
        )

        # Calculate XY-plane arc lengths (2D projection to match ParamPoly3)
        # ParamPoly3.from_spline() uses only XY coordinates, so we must too
        xy_distances = np.linalg.norm(np.diff(points_3d[:, :2], axis=0), axis=1)
        xy_arc_lengths = np.concatenate(([0], np.cumsum(xy_distances)))

        # Extract elevations (z coordinates)
        elevations = points_3d[:, 2]

        # Create CubicSpline1D mapping XY arc length -> elevation
        elevation_spline = CubicSpline1D(
            xy_arc_lengths, elevations, bc_type="not-a-knot"
        )

        # Extract elevation segments from the spline
        elevation_segments = elevation_spline.get_segments()

        # Create Elevation objects for each segment
        elevations = [
            Elevation(s=s_offset, a=a, b=b, c=c, d=d)
            for s_offset, a, b, c, d in elevation_segments
        ]

        return ElevationProfile(elevations=elevations)

    def to_xml(self):
        """Convert to XML element via the internal lane."""
        return self._lane.to_xml()

    def __repr__(self) -> str:
        """String representation of the reference line."""
        return f"ReferenceLine(widths={len(self.widths)}, marks={len(self.road_marks)})"
