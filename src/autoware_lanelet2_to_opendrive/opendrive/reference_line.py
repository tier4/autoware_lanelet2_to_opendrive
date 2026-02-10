"""ReferenceLine implementation for OpenDRIVE conversion."""

from typing import TYPE_CHECKING, List, Optional, Set, Union

import lanelet2
import lxml.etree as ET
import numpy as np

from ..cubic_spline_1d import CubicSpline1D
from ..spline import Splines

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
        traffic_rule: Optional[str] = None,
    ) -> "ReferenceLine":
        """
        Construct a ReferenceLine from a group of Lanelet2 lanelets.

        Args:
            lanelet_map: The Lanelet2 map containing the lanelets
            lanelet_group: List of lanelets representing lanes in a road
            traffic_rule: Traffic handedness - "RHT" (Right-Hand Traffic) or "LHT" (Left-Hand Traffic).
                         Defaults to "RHT" if not specified.
                         - RHT: Uses leftmost lanelet's left boundary as reference line
                         - LHT: Uses rightmost lanelet's right boundary as reference line

        Returns:
            ReferenceLine instance constructed from the center of the lanelet group
        """
        if not lanelet_group:
            raise ValueError("Lanelet group cannot be empty")

        # Sort the lanelets from left to right
        from ..util import sort_adjacent_groups

        sorted_lanelets = sort_adjacent_groups(lanelet_map, lanelet_group)

        # Normalize traffic_rule to uppercase, default to RHT
        traffic_rule_normalized = (traffic_rule or "RHT").upper()

        # Validate traffic_rule
        if traffic_rule_normalized not in ("RHT", "LHT"):
            raise ValueError(
                f"Invalid traffic_rule: '{traffic_rule}'. Must be 'RHT' or 'LHT'."
            )

        # Select reference lanelet and boundary based on traffic rule
        if traffic_rule_normalized == "RHT":
            # RHT: Use leftmost lanelet's left boundary
            reference_lanelet = sorted_lanelets[0]
            border = "left"
            boundary = reference_lanelet.leftBound
        else:  # LHT
            # LHT: Use rightmost lanelet's left boundary (inner edge)
            reference_lanelet = sorted_lanelets[-1]
            border = "left"
            boundary = reference_lanelet.leftBound

        from ..centerline import extract_border_from_spline

        # Extract 3D points directly from the selected boundary (no 3D spline needed)
        # This avoids the 3D-to-2D arc length mapping issues on steep slopes
        from ..util import extract_points_3d

        points_3d = extract_points_3d(boundary)

        # Calculate XY cumulative distances (2D arc length) directly from points
        xy_distances = np.linalg.norm(np.diff(points_3d[:, :2], axis=0), axis=1)
        xy_arc_lengths = np.concatenate(([0], np.cumsum(xy_distances)))

        # Get elevation offset (absolute Z at s=0)
        elevation_offset = points_3d[0, 2]

        # Extract the selected boundary as 2D spline (XY only) for reference line geometry
        centerline_2d = extract_border_from_spline(
            reference_lanelet, border=border, dimensions=2
        )

        # Generate height_spline: mapping from XY arc length (s) to relative elevation (z)
        # Use the original point Z-coordinates directly (no 3D spline sampling needed)
        z_values = points_3d[:, 2]
        relative_elevations_raw = z_values - elevation_offset

        # Resample to desired density (0.1m intervals) for height spline
        total_length_2d = centerline_2d.total_length
        num_samples = max(10, int(np.ceil(total_length_2d / 0.1)) + 1)
        xy_arc_lengths_resampled = np.linspace(0, total_length_2d, num_samples)

        # Interpolate z-coordinates at resampled positions
        relative_elevations = np.interp(
            xy_arc_lengths_resampled, xy_arc_lengths, relative_elevations_raw
        )

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
    def widths(self) -> List[LaneWidth]:
        """Access to lane widths."""
        return self._lane.widths

    @property
    def road_marks(self) -> List:
        """Access to road marks."""
        return self._lane.road_marks

    @property
    def lane_id(self) -> int:
        """Access to lane ID."""
        return self._lane.lane_id

    @property
    def lane_type(self) -> LaneType:
        """Access to lane type."""
        return self._lane.lane_type

    @property
    def level(self) -> bool:
        """Access to lane level."""
        return self._lane.level

    def get_elevation_profile(
        self, geometry_s_values: list[float] | None = None
    ) -> ElevationProfile:
        """
        Get elevation profile from the reference line.

        The elevation profile uses XY-plane arc length (2D projection) to match ParamPoly3
        geometry coordinates, ensuring that s-coordinates are consistent between the
        reference line geometry and elevation profile.

        Args:
            geometry_s_values: Optional list of s-coordinates for segment boundaries.
                If provided, elevation segments will be aligned with these boundaries
                (typically from ParamPoly3 geometries) to ensure consistency.
                If None, uses the internal height_spline segment boundaries.

        Returns:
            ElevationProfile object containing elevation segments with polynomial coefficients
        """
        if geometry_s_values is None:
            # Fallback: use height_spline segments directly
            elevation_segments = self.height_spline.get_segments()
            elevations = [
                Elevation(s=s_offset, a=a + self.elevation_offset, b=b, c=c, d=d)
                for s_offset, a, b, c, d in elevation_segments
            ]
        else:
            # Create elevation segments aligned with geometry boundaries
            # Use cubic Hermite interpolation to compute polynomial coefficients
            elevations = []
            for i in range(len(geometry_s_values)):
                s_start = geometry_s_values[i]
                s_end = (
                    geometry_s_values[i + 1]
                    if i + 1 < len(geometry_s_values)
                    else self.height_spline.total_length
                )
                segment_length = s_end - s_start

                if segment_length <= 0:
                    continue

                # Evaluate height_spline at segment boundaries
                # Get relative elevation and derivatives
                z_start = self.height_spline.evaluate(s_start, derivative=0)
                dz_start = self.height_spline.evaluate(s_start, derivative=1)
                z_end = self.height_spline.evaluate(s_end, derivative=0)
                dz_end = self.height_spline.evaluate(s_end, derivative=1)

                # Compute cubic polynomial coefficients using Hermite interpolation
                # z(ds) = a + b*ds + c*ds^2 + d*ds^3, where ds = s - s_start
                # Boundary conditions:
                #   z(0) = z_start, z(L) = z_end
                #   z'(0) = dz_start, z'(L) = dz_end
                L = segment_length

                a = z_start
                b = dz_start
                c = (3 * (z_end - z_start) / L - 2 * dz_start - dz_end) / L
                d = (2 * (z_start - z_end) / L + dz_start + dz_end) / (L * L)

                # Add elevation_offset to 'a' to convert to absolute z-coordinate
                elevations.append(
                    Elevation(
                        s=s_start,
                        a=a + self.elevation_offset,
                        b=b,
                        c=c,
                        d=d,
                    )
                )

        return ElevationProfile(elevations=elevations)

    def to_xml(self) -> ET.Element:
        """Convert to XML element via the internal lane."""
        return self._lane.to_xml()

    def __repr__(self) -> str:
        """String representation of the reference line."""
        return f"ReferenceLine(widths={len(self.widths)}, marks={len(self.road_marks)})"
