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
        elevation_offset: float = 0.0,
        centerline_3d: Union[Splines, None] = None,
    ):
        """
        Initialize a ReferenceLine with a 2D centerline spline.

        Args:
            centerline_2d: 2D spline (XY coordinates only) representing the reference line
            elevation_offset: Absolute elevation (z coordinate) at the road start point (s=0)
            centerline_3d: Optional 3D spline for temporary use in elevation profile generation
                          (will be removed in Phase 2 when height_spline is introduced)
        """
        self.centerline_2d: Splines = centerline_2d

        # Store the absolute elevation at the road start point (s=0)
        # This is needed for calculating signal z_offsets correctly
        self.elevation_offset = elevation_offset

        # TODO: Remove this in Phase 2 (Issue #58) when height_spline is introduced
        # Temporarily store 3D spline for elevation profile generation
        self._centerline_3d_temp = centerline_3d

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

        # Extract the left boundary as 3D spline first to get elevation_offset
        centerline_3d = extract_border_from_spline(
            leftmost_lanelet, border="left", dimensions=3
        )
        elevation_offset = centerline_3d.evaluate(0.0)[2]

        # Extract the left boundary as 2D spline (XY only) for reference line
        centerline_2d = extract_border_from_spline(
            leftmost_lanelet, border="left", dimensions=2
        )

        # Create the ReferenceLine instance with 2D centerline and elevation offset
        # TODO: Remove centerline_3d parameter in Phase 2 (Issue #58)
        reference_line = ReferenceLine(
            centerline_2d=centerline_2d,
            elevation_offset=elevation_offset,
            centerline_3d=centerline_3d,
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

        IMPORTANT: This method uses XY-plane arc length (2D projection) to match ParamPoly3
        geometry coordinates. The 3D spline's natural arc length is NOT used because
        ParamPoly3.from_spline() only uses XY coordinates (ignoring Z), which causes
        misalignment on slopes.

        TODO: In Phase 2 (Issue #58), this will be simplified to directly use height_spline.

        Returns:
            ElevationProfile object containing elevation segments with polynomial coefficients
        """
        # TODO: Phase 2 - Replace this temporary implementation with height_spline
        if self._centerline_3d_temp is None:
            raise RuntimeError(
                "Cannot generate elevation profile without 3D centerline data. "
                "This should be addressed in Phase 2 (Issue #58)."
            )

        # IMPORTANT: Use 2D spline length to match road length from ParamPoly3
        # The road length is calculated from centerline_2d, so elevation profile
        # must use the same s-coordinate range
        total_length_2d = self.centerline_2d.total_length

        # Sample 3D spline densely and accumulate XY distances until reaching 2D length
        # This ensures elevation profile s-coordinates match road length
        total_length_3d = self._centerline_3d_temp.total_length

        # Sample 3D spline at very high density (0.05m intervals)
        num_samples_3d = max(20, int(np.ceil(total_length_3d / 0.05)) + 1)
        arc_lengths_3d = np.linspace(0, total_length_3d, num_samples_3d)

        # Extract all 3D coordinates
        all_points_3d = np.array(
            [self._centerline_3d_temp.evaluate(s) for s in arc_lengths_3d]
        )

        # Calculate cumulative XY distances
        xy_distances = np.linalg.norm(np.diff(all_points_3d[:, :2], axis=0), axis=1)
        xy_cumulative = np.concatenate(([0], np.cumsum(xy_distances)))

        # Find points where XY cumulative distance <= total_length_2d
        # Add a small tolerance to include the endpoint
        valid_indices = xy_cumulative <= (total_length_2d + 1e-6)

        # Extract valid points and their XY arc lengths
        points_3d = all_points_3d[valid_indices]
        xy_arc_lengths = xy_cumulative[valid_indices]

        # Ensure the last point exactly matches total_length_2d
        if len(xy_arc_lengths) > 0 and xy_arc_lengths[-1] < total_length_2d:
            # Need to interpolate the last point
            # Find the next point beyond total_length_2d
            next_idx = np.searchsorted(xy_cumulative, total_length_2d)
            if next_idx < len(xy_cumulative):
                # Linear interpolation between last valid point and next point
                t = (total_length_2d - xy_cumulative[next_idx - 1]) / (
                    xy_cumulative[next_idx] - xy_cumulative[next_idx - 1]
                )
                last_point = all_points_3d[next_idx - 1] + t * (
                    all_points_3d[next_idx] - all_points_3d[next_idx - 1]
                )
                points_3d = np.vstack([points_3d, last_point])
                xy_arc_lengths = np.append(xy_arc_lengths, total_length_2d)

        # Resample to desired density (0.1m intervals) for elevation spline
        num_samples = max(10, int(np.ceil(total_length_2d / 0.1)) + 1)
        xy_arc_lengths_resampled = np.linspace(0, total_length_2d, num_samples)

        # Interpolate z-coordinates at resampled positions
        elevations_resampled = np.interp(
            xy_arc_lengths_resampled, xy_arc_lengths, points_3d[:, 2]
        )

        # Update for subsequent code
        xy_arc_lengths = xy_arc_lengths_resampled
        elevations = elevations_resampled

        # Convert to relative elevation (offset from first point)
        # OpenDRIVE elevation profile represents height changes relative to road start
        # not absolute altitude from sea level
        elevation_offset = elevations[0]
        relative_elevations = elevations - elevation_offset

        # Create CubicSpline1D mapping XY arc length -> relative elevation
        elevation_spline = CubicSpline1D(
            xy_arc_lengths, relative_elevations, bc_type="not-a-knot"
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
