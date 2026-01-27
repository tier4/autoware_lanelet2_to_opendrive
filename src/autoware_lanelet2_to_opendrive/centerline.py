import numpy as np
import lanelet2
from typing import Set
from .config import DEFAULT_CONFIG
from .spline import Splines
from .util import sort_adjacent_groups, extract_points_3d, extract_points_2d
from .cubic_spline_1d import CubicSpline1D


class AsymmetryLaneletException(Exception):
    """Exception raised when a lanelet has asymmetric left and right widths."""

    pass


class Width1DSplineAdapter:
    """
    Adapter class that wraps CubicSpline1D to provide compatibility with existing interface.
    """

    def __init__(self, width_spline_1d: CubicSpline1D):
        """
        Initialize the adapter.

        Args:
            width_spline_1d: The 1D width spline object
        """
        self.spline_1d = width_spline_1d
        self.total_length = width_spline_1d.total_arc_length

    def evaluate(self, s: float, derivative: int = 0) -> np.ndarray:
        """
        Evaluate the width at a given arc length.

        Args:
            s: Arc length along the reference line
            derivative: Derivative order (only 0 supported)

        Returns:
            3D array [s, width, 0] for compatibility
        """
        return self.spline_1d.evaluate_with_3d_compatibility(s, derivative)

    def evaluate_arc_length(self, s: float, derivative: int = 0) -> np.ndarray:
        """Alias for evaluate method."""
        return self.evaluate(s, derivative)

    def get_width_at_arc_length(self, s: float) -> float:
        """Get just the width value at a given arc length."""
        return self.spline_1d.evaluate(s, derivative=0)

    def get_polynomial_segments(self) -> list[tuple[float, float, float, float, float]]:
        """Get polynomial segments for OpenDRIVE export."""
        return self.spline_1d.get_segments()


class WidthSplineWrapper:
    """
    Wrapper class that provides a proper interface for width splines.

    This class wraps a parametric spline that maps t ∈ [0,1] to (arc_length, width)
    and provides methods to query width at a given arc_length.
    """

    def __init__(self, parametric_spline: Splines, total_arc_length: float):
        """
        Initialize the width spline wrapper.

        Args:
            parametric_spline: A Splines object that maps t to (arc_length, width, 0)
            total_arc_length: Total arc length of the reference line
        """
        self.parametric_spline = parametric_spline
        self.total_arc_length = total_arc_length

        # Build lookup table for arc_length -> parameter t mapping
        self._build_lookup_table()

    def _build_lookup_table(self, num_samples: int = 100) -> None:
        """Build a lookup table for converting arc_length to parameter t."""
        self.t_samples = np.linspace(0, 1, num_samples)
        self.arc_length_samples = []
        self.width_samples = []

        for t in self.t_samples:
            # Evaluate the parametric spline at t
            # Note: _evaluate_normalized returns points in the translated coordinate system
            point = self.parametric_spline._evaluate_normalized(t)
            # Need to add back the origin offset to get actual values
            actual_point = point + self.parametric_spline._origin_offset
            # point[0] is arc_length, point[1] is width
            self.arc_length_samples.append(actual_point[0])
            self.width_samples.append(actual_point[1])

        self.arc_length_samples = np.array(self.arc_length_samples)
        self.width_samples = np.array(self.width_samples)

    def evaluate(self, s: float, derivative: int = 0) -> np.ndarray:
        """
        Evaluate the width at a given arc length.

        Args:
            s: Arc length along the reference line
            derivative: Derivative order (only 0 is supported for now)

        Returns:
            For derivative=0: Returns a 3D array [s, width, 0] for compatibility
        """
        if derivative != 0:
            raise NotImplementedError("Width derivatives not yet implemented")

        # Clamp arc length to valid range
        s = np.clip(s, 0, self.total_arc_length)

        # Find parameter t corresponding to arc length s
        # Use linear interpolation on the lookup table
        t = np.interp(s, self.arc_length_samples, self.t_samples)

        # Evaluate the parametric spline at t
        point = self.parametric_spline._evaluate_normalized(t)
        actual_point = point + self.parametric_spline._origin_offset

        # Return in format [arc_length, width, 0] for compatibility
        return np.array([s, actual_point[1], 0.0])

    def evaluate_arc_length(self, s: float, derivative: int = 0) -> np.ndarray:
        """Alias for evaluate method."""
        return self.evaluate(s, derivative)

    @property
    def total_length(self) -> float:
        """Get the total arc length."""
        return self.total_arc_length

    def get_width_at_arc_length(self, s: float) -> float:
        """
        Get just the width value at a given arc length.

        Args:
            s: Arc length along the reference line

        Returns:
            Width value at the given arc length
        """
        result = self.evaluate(s)
        return result[1]  # Return just the width component


def _get_boundary_start_vel(boundary) -> np.ndarray:
    """
    Calculate start velocity vector for a boundary spline.

    Args:
        boundary: List of points representing a boundary

    Returns:
        3D velocity vector along the boundary direction
    """
    if len(boundary) < 2:
        return np.array([1.0, 0.0, 0.0])  # Default direction

    # Calculate direction from first to second point
    start_point = np.array([boundary[0].x, boundary[0].y, boundary[0].z])
    next_point = np.array([boundary[1].x, boundary[1].y, boundary[1].z])

    direction = next_point - start_point
    length = np.linalg.norm(direction)

    if length < DEFAULT_CONFIG.geometry.epsilon:
        return np.array([1.0, 0.0, 0.0])  # Fallback

    return direction / length


def _get_boundary_end_vel(boundary) -> np.ndarray:
    """
    Calculate end velocity vector for a boundary spline.

    Args:
        boundary: List of points representing a boundary

    Returns:
        3D velocity vector along the boundary direction
    """
    if len(boundary) < 2:
        return np.array([1.0, 0.0, 0.0])  # Default direction

    # Calculate direction from second-to-last to last point
    prev_point = np.array([boundary[-2].x, boundary[-2].y, boundary[-2].z])
    end_point = np.array([boundary[-1].x, boundary[-1].y, boundary[-1].z])

    direction = end_point - prev_point
    length = np.linalg.norm(direction)

    if length < DEFAULT_CONFIG.geometry.epsilon:
        return np.array([1.0, 0.0, 0.0])  # Fallback

    return direction / length


def _get_start_vel(lanelet: lanelet2.core.Lanelet) -> np.ndarray:
    """
    Calculate start velocity vector for centerline spline from lanelet boundaries.

    Args:
        lanelet: A Lanelet2 lanelet object

    Returns:
        3D velocity vector perpendicular to the line connecting first points of left and right boundaries
    """
    left_bound = lanelet.leftBound
    right_bound = lanelet.rightBound

    if len(left_bound) < 1 or len(right_bound) < 1:
        raise ValueError(
            "Lanelet must have at least 1 point in both left and right boundaries"
        )

    # Get first points of boundaries
    left_first = np.array([left_bound[0].x, left_bound[0].y])
    right_first = np.array([right_bound[0].x, right_bound[0].y])

    # Calculate line segment connecting left and right boundaries
    segment = right_first - left_first  # Vector from left to right

    # Calculate perpendicular vector (2D) pointing forward along the lanelet
    # Rotate 90 degrees counter-clockwise: (x, y) -> (-y, x)
    perp_2d = np.array([-segment[1], segment[0]])

    # Normalize to unit vector
    length = np.linalg.norm(perp_2d)
    if length < DEFAULT_CONFIG.geometry.epsilon:
        # Fallback to default direction if boundaries are parallel
        perp_2d = np.array([1.0, 0.0])
    else:
        perp_2d = perp_2d / length

    # Convert to 3D by adding z=0
    return np.array([perp_2d[0], perp_2d[1], 0.0])


def _get_end_vel(lanelet: lanelet2.core.Lanelet) -> np.ndarray:
    """
    Calculate end velocity vector for centerline spline from lanelet boundaries.

    Args:
        lanelet: A Lanelet2 lanelet object

    Returns:
        3D velocity vector perpendicular to the line connecting last points of left and right boundaries
    """
    left_bound = lanelet.leftBound
    right_bound = lanelet.rightBound

    if len(left_bound) < 1 or len(right_bound) < 1:
        raise ValueError(
            "Lanelet must have at least 1 point in both left and right boundaries"
        )

    # Get last points of boundaries
    left_last = np.array([left_bound[-1].x, left_bound[-1].y])
    right_last = np.array([right_bound[-1].x, right_bound[-1].y])

    # Calculate line segment connecting left and right boundaries
    segment = right_last - left_last  # Vector from left to right

    # Calculate perpendicular vector (2D) pointing forward along the lanelet
    # Rotate 90 degrees counter-clockwise: (x, y) -> (-y, x)
    perp_2d = np.array([-segment[1], segment[0]])

    # Normalize to unit vector
    length = np.linalg.norm(perp_2d)
    if length < DEFAULT_CONFIG.geometry.epsilon:
        # Fallback to default direction if boundaries are parallel
        perp_2d = np.array([1.0, 0.0])
    else:
        perp_2d = perp_2d / length

    # Convert to 3D by adding z=0
    return np.array([perp_2d[0], perp_2d[1], 0.0])


def extract_centerline_as_spline(
    lanelet: lanelet2.core.Lanelet, num_control_points: int = 10
) -> Splines:
    """
    Extract centerline from a Lanelet using midpoints between left and right borders.

    Uses line segment representation of borders to calculate centerline points
    by interpolating along both borders using normalized coordinates.

    Args:
        lanelet: A Lanelet2 lanelet object
        num_control_points: Number of control points for B-spline interpolation

    Returns:
        Splines object that can be evaluated using arc length
    """
    # Get raw boundary points
    left_bound = lanelet.leftBound
    right_bound = lanelet.rightBound

    if len(left_bound) < 2 or len(right_bound) < 2:
        raise ValueError("Both boundaries must have at least 2 points")

    # Convert to numpy arrays with coordinate offset applied
    left_points = extract_points_3d(left_bound)
    right_points = extract_points_3d(right_bound)

    # Calculate cumulative arc lengths for both boundaries
    left_dists = np.linalg.norm(np.diff(left_points, axis=0), axis=1)
    left_cumulative = np.concatenate(([0], np.cumsum(left_dists)))
    left_total_length = left_cumulative[-1]

    right_dists = np.linalg.norm(np.diff(right_points, axis=0), axis=1)
    right_cumulative = np.concatenate(([0], np.cumsum(right_dists)))
    right_total_length = right_cumulative[-1]

    # Number of sample points for centerline
    num_samples = max(
        DEFAULT_CONFIG.centerline.min_sample_points_for_centerline,
        num_control_points * DEFAULT_CONFIG.centerline.sample_point_multiplier,
    )

    centerline_points = []
    for i in range(num_samples):
        # Normalized coordinate from 0 to 1
        t_normalized = i / (num_samples - 1) if num_samples > 1 else 0.0

        # Convert to arc length for each boundary
        left_s = t_normalized * left_total_length
        right_s = t_normalized * right_total_length

        # Interpolate on left boundary line segments
        left_point = _interpolate_on_line_segments(left_points, left_cumulative, left_s)

        # Interpolate on right boundary line segments
        right_point = _interpolate_on_line_segments(
            right_points, right_cumulative, right_s
        )

        # Calculate midpoint
        midpoint = (left_point + right_point) / 2.0
        centerline_points.append(midpoint)

    centerline_points = np.array(centerline_points)

    # Create B-spline with constrained fitting
    return Splines(
        centerline_points,
        start_vel=_get_start_vel(lanelet),
        end_vel=_get_end_vel(lanelet),
        num_control_points=num_control_points,
    )


def _interpolate_on_line_segments(
    points: np.ndarray, cumulative_lengths: np.ndarray, s: float
) -> np.ndarray:
    """
    Interpolate a point along line segments at given arc length.

    Args:
        points: Array of points defining the line segments (N x 3)
        cumulative_lengths: Cumulative arc lengths at each point (N,)
        s: Target arc length for interpolation

    Returns:
        Interpolated 3D point
    """
    # Clamp s to valid range
    s = np.clip(s, 0.0, cumulative_lengths[-1])

    # Find which segment contains the target arc length
    segment_idx = np.searchsorted(cumulative_lengths, s) - 1
    segment_idx = np.clip(segment_idx, 0, len(points) - 2)

    # Get segment endpoints
    p1 = points[segment_idx]
    p2 = points[segment_idx + 1]

    # Get arc lengths at segment endpoints
    s1 = cumulative_lengths[segment_idx]
    s2 = cumulative_lengths[segment_idx + 1]

    # Avoid division by zero for zero-length segments
    if abs(s2 - s1) < DEFAULT_CONFIG.geometry.epsilon:
        return p1

    # Linear interpolation within the segment
    t = (s - s1) / (s2 - s1)
    return p1 + t * (p2 - p1)


def extract_border_from_spline(
    lanelet: lanelet2.core.Lanelet,
    border: str,
    num_control_points: int = 10,
    dimensions: int = 2,
) -> Splines:
    """
    Extract border line from a Lanelet and return as B-spline with arc length parameterization.

    Args:
        lanelet: A Lanelet2 lanelet object
        border: Border specification - "left" or "right"
        num_control_points: Number of control points for B-spline interpolation
        dimensions: Number of dimensions (2 for XY only)

    Returns:
        Splines object that can be evaluated using arc length

    Raises:
        ValueError: If border is not "left" or "right", if insufficient points,
                    or if dimensions >= 3
    """
    if border not in ["left", "right"]:
        raise ValueError(f"Invalid border: {border}. Must be 'left' or 'right'")

    if dimensions >= 3:
        raise ValueError(
            f"Invalid dimensions: {dimensions}. OpenDRIVE reference lines must be 2D. "
            "Please refer to the OpenDRIVE specification."
        )

    # Get the appropriate boundary
    if border == "left":
        boundary = lanelet.leftBound
    else:  # border == "right"
        boundary = lanelet.rightBound

    if len(boundary) < 2:
        raise ValueError(
            f"Lanelet must have at least 2 points in its {border} boundary"
        )

    # Extract 2D points from the boundary (XY only, per OpenDRIVE spec)
    # Coordinate offset is applied automatically
    points = extract_points_2d(boundary)

    # Get velocity vectors (XY components only)
    start_vel = _get_boundary_start_vel(boundary)[:2]
    end_vel = _get_boundary_end_vel(boundary)[:2]

    # Create B-spline with constrained fitting
    return Splines(
        points,
        start_vel=start_vel,
        end_vel=end_vel,
        num_control_points=num_control_points,
    )


def estimate_lanelet_width_as_spline(
    lanelet: lanelet2.core.Lanelet,
    num_samples: int = 20,
    num_control_points: int = 10,
    reference: str = "center_line",
) -> Width1DSplineAdapter:
    """
    Estimate lanelet width as a spline by measuring distances between corresponding
    points on borders using direct linear interpolation of original Lanelet2 points.

    Args:
        lanelet: A Lanelet2 lanelet object
        num_samples: Number of points to sample along the lanelet for width estimation
        num_control_points: Number of control points for width spline interpolation (unused, kept for compatibility)
        reference: Reference line to use - "center_line", "left_bound", or "right_bound"

    Returns:
        Width1DSplineAdapter object representing width as a function of arc length along the reference

    Raises:
        ValueError: If reference is invalid or if lanelet has insufficient points
    """
    if reference not in ["center_line", "left_bound", "right_bound"]:
        raise ValueError(
            f"Invalid reference: {reference}. Must be 'center_line', 'left_bound', or 'right_bound'"
        )

    # Get raw boundary points directly from lanelet
    left_bound = lanelet.leftBound
    right_bound = lanelet.rightBound

    if len(left_bound) < 2 or len(right_bound) < 2:
        raise ValueError("Both boundaries must have at least 2 points")

    # Convert to numpy arrays with coordinate offset applied
    left_points = extract_points_3d(left_bound)
    right_points = extract_points_3d(right_bound)

    # Calculate cumulative arc lengths for both boundaries
    left_dists = np.linalg.norm(np.diff(left_points, axis=0), axis=1)
    left_cumulative = np.concatenate(([0], np.cumsum(left_dists)))
    left_total_length = left_cumulative[-1]

    right_dists = np.linalg.norm(np.diff(right_points, axis=0), axis=1)
    right_cumulative = np.concatenate(([0], np.cumsum(right_dists)))
    right_total_length = right_cumulative[-1]

    # Sample points along normalized arc length (0 to 1)
    normalized_positions = np.linspace(0.0, 1.0, num_samples)

    # Calculate widths at each normalized position
    arc_lengths: list[float] = []
    widths: list[float] = []

    for t_norm in normalized_positions:
        # Convert normalized position to actual arc length for each boundary
        s_left = t_norm * left_total_length
        s_right = t_norm * right_total_length

        # Interpolate positions on boundaries using linear interpolation
        left_pos = _interpolate_on_line_segments(left_points, left_cumulative, s_left)
        right_pos = _interpolate_on_line_segments(
            right_points, right_cumulative, s_right
        )

        # Calculate width based on reference type
        if reference == "center_line":
            # Calculate centerline position as midpoint
            center_pos = (left_pos + right_pos) / 2.0

            # Width is the total distance from center to both borders
            left_dist = np.linalg.norm(center_pos - left_pos)
            right_dist = np.linalg.norm(center_pos - right_pos)
            width = left_dist + right_dist

            # Arc length is based on centerline
            # For first point, arc length is 0
            if len(arc_lengths) == 0:
                arc_length = 0.0
            else:
                # Calculate arc length increment from previous centerline position
                prev_center_pos = (
                    _interpolate_on_line_segments(
                        left_points,
                        left_cumulative,
                        normalized_positions[len(arc_lengths) - 1] * left_total_length,
                    )
                    + _interpolate_on_line_segments(
                        right_points,
                        right_cumulative,
                        normalized_positions[len(arc_lengths) - 1] * right_total_length,
                    )
                ) / 2.0
                arc_length = arc_lengths[-1] + np.linalg.norm(
                    center_pos - prev_center_pos
                )
        elif reference == "left_bound":
            # Width is distance from left to right border
            width = np.linalg.norm(left_pos - right_pos)
            arc_length = s_left
        elif reference == "right_bound":
            # Width is distance from right to left border
            width = np.linalg.norm(right_pos - left_pos)
            arc_length = s_right

        arc_lengths.append(arc_length)
        widths.append(width)

    # Create a proper 1D cubic spline for width as a function of arc length
    arc_lengths_array = np.array(arc_lengths)
    widths_array = np.array(widths)

    # Use not-a-knot boundary conditions for smoother interpolation with less oscillation
    width_spline_1d = CubicSpline1D(
        arc_lengths_array,
        widths_array,
        bc_type=DEFAULT_CONFIG.centerline.boundary_condition_default,
    )

    # Create a wrapper that provides the same interface as the old WidthSplineWrapper
    return Width1DSplineAdapter(width_spline_1d)


def extract_centerline_as_spline_from_two_lanelets(
    lanelet_map: lanelet2.core.LaneletMap,
    two_lanelets: Set[lanelet2.core.Lanelet],
    num_control_points: int = 10,
) -> Splines:
    """
    Extract centerline as spline from two adjacent lanelets using the left lanelet's right bound.

    Args:
        lanelet_map: The lanelet2 map containing the lanelets
        two_lanelets: Set containing exactly two lanelets
        num_control_points: Number of control points for B-spline interpolation

    Returns:
        Splines object representing the right bound of the left lanelet

    Raises:
        ValueError: If two_lanelets does not contain exactly 2 lanelets
    """
    if len(two_lanelets) != 2:
        raise ValueError(f"Expected exactly 2 lanelets, got {len(two_lanelets)}")

    # Sort the two lanelets from left to right
    sorted_lanelets = sort_adjacent_groups(lanelet_map, two_lanelets)

    # Get the left lanelet (first in sorted order)
    left_lanelet = sorted_lanelets[0]

    # Extract points from the right bound of the left lanelet
    right_bound = left_lanelet.rightBound

    if len(right_bound) < 2:
        raise ValueError("Left lanelet must have at least 2 points in its right bound")

    # Extract 3D points with coordinate offset applied
    points = extract_points_3d(right_bound)

    # Create and return the B-spline
    return Splines(
        points,
        start_vel=_get_start_vel(left_lanelet),
        end_vel=_get_end_vel(left_lanelet),
        num_control_points=num_control_points,
    )
