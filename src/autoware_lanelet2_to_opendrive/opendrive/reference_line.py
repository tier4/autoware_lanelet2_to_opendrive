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

    def __init__(
        self,
        centerline_2d: Splines,
        height_spline: CubicSpline1D,
        elevation_offset: float = 0.0,
    ):
        """
        Initialize a ReferenceLine with a 2D centerline spline and height spline.

        Args:
            centerline_2d: 2D spline (XY coordinates only) representing the reference line
            height_spline: 1D spline mapping XY arc length (s) to relative elevation (z)
            elevation_offset: Absolute elevation (z coordinate) at the road start point (s=0)
        """
        self.centerline_2d: Splines = centerline_2d
        self.height_spline: CubicSpline1D = height_spline

        # Store the absolute elevation at the road start point (s=0)
        # This is needed for calculating signal z_offsets correctly
        self.elevation_offset = elevation_offset

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

        # Extract the left boundary as 3D spline first to get elevation data
        centerline_3d = extract_border_from_spline(
            leftmost_lanelet, border="left", dimensions=3
        )
        elevation_offset = centerline_3d.evaluate(0.0)[2]

        # Extract the left boundary as 2D spline (XY only) for reference line
        centerline_2d = extract_border_from_spline(
            leftmost_lanelet, border="left", dimensions=2
        )

        # Generate height_spline: mapping from XY arc length (s) to relative elevation (z)
        # Sample the 3D spline and calculate cumulative XY distances
        total_length_2d = centerline_2d.total_length
        total_length_3d = centerline_3d.total_length

        # Sample 3D spline at high density (0.05m intervals)
        num_samples_3d = max(20, int(np.ceil(total_length_3d / 0.05)) + 1)
        arc_lengths_3d = np.linspace(0, total_length_3d, num_samples_3d)

        # Extract all 3D coordinates
        all_points_3d = np.array([centerline_3d.evaluate(s) for s in arc_lengths_3d])

        # Calculate cumulative XY distances (arc length in 2D projection)
        xy_distances = np.linalg.norm(np.diff(all_points_3d[:, :2], axis=0), axis=1)
        xy_cumulative = np.concatenate(([0], np.cumsum(xy_distances)))

        # Find points where XY cumulative distance <= total_length_2d
        valid_indices = xy_cumulative <= (total_length_2d + 1e-6)

        # Extract valid points and their XY arc lengths
        points_3d = all_points_3d[valid_indices]
        xy_arc_lengths = xy_cumulative[valid_indices]

        # Ensure the last point exactly matches total_length_2d
        if len(xy_arc_lengths) > 0 and xy_arc_lengths[-1] < total_length_2d:
            # Interpolate the last point
            next_idx = np.searchsorted(xy_cumulative, total_length_2d)
            if next_idx < len(xy_cumulative):
                t = (total_length_2d - xy_cumulative[next_idx - 1]) / (
                    xy_cumulative[next_idx] - xy_cumulative[next_idx - 1]
                )
                last_point = all_points_3d[next_idx - 1] + t * (
                    all_points_3d[next_idx] - all_points_3d[next_idx - 1]
                )
                points_3d = np.vstack([points_3d, last_point])
                xy_arc_lengths = np.append(xy_arc_lengths, total_length_2d)

        # Resample to desired density (0.1m intervals) for height spline
        num_samples = max(10, int(np.ceil(total_length_2d / 0.1)) + 1)
        xy_arc_lengths_resampled = np.linspace(0, total_length_2d, num_samples)

        # Interpolate z-coordinates at resampled positions
        elevations_resampled = np.interp(
            xy_arc_lengths_resampled, xy_arc_lengths, points_3d[:, 2]
        )

        # Convert to relative elevation (offset from first point)
        relative_elevations = elevations_resampled - elevation_offset

        # Create height_spline mapping XY arc length -> relative elevation
        height_spline = CubicSpline1D(
            xy_arc_lengths_resampled, relative_elevations, bc_type="not-a-knot"
        )

        # Create the ReferenceLine instance with 2D centerline, height spline, and elevation offset
        reference_line = ReferenceLine(
            centerline_2d=centerline_2d,
            height_spline=height_spline,
            elevation_offset=elevation_offset,
        )

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
        Get elevation profile from the reference line.

        The elevation profile uses XY-plane arc length (2D projection) to match ParamPoly3
        geometry coordinates, ensuring that s-coordinates are consistent between the
        reference line geometry and elevation profile.

        Returns:
            ElevationProfile object containing elevation segments with polynomial coefficients
        """
        # Extract elevation segments directly from height_spline
        # height_spline maps XY arc length (s) to relative elevation (z - elevation_offset)
        elevation_segments = self.height_spline.get_segments()

        # Create Elevation objects for each segment
        # Add elevation_offset to the 'a' coefficient of each segment to convert
        # from relative elevation back to absolute inertial z-coordinate
        elevations = [
            Elevation(s=s_offset, a=a + self.elevation_offset, b=b, c=c, d=d)
            for s_offset, a, b, c, d in elevation_segments
        ]

        return ElevationProfile(elevations=elevations)

    def to_xml(self):
        """Convert to XML element via the internal lane."""
        return self._lane.to_xml()

    def __repr__(self) -> str:
        """String representation of the reference line."""
        return f"ReferenceLine(widths={len(self.widths)}, marks={len(self.road_marks)})"
