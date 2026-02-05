import numpy as np
import lanelet2
from typing import List, Set, Tuple
from .config import DEFAULT_CONFIG
from .spline import Splines
from .util import sort_adjacent_groups, extract_points_3d, extract_points_2d
from .cubic_spline_1d import CubicSpline1D
from .conversion_config import WidthEstimationConfig


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

    def get_polynomial_segments(self) -> List[Tuple[float, float, float, float, float]]:
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


def _calculate_boundary_velocity_vector(boundary, at_start: bool) -> np.ndarray:
    """
    Calculate velocity vector at boundary point.

    Args:
        boundary: List of points representing a boundary
        at_start: True for start point, False for end point

    Returns:
        Normalized 3D velocity vector along the boundary direction
    """
    if len(boundary) < 2:
        return np.array([1.0, 0.0, 0.0])  # Default direction

    # Select endpoint and neighbor based on at_start parameter
    if at_start:
        endpoint_idx = 0
        neighbor_idx = 1
    else:
        endpoint_idx = -1
        neighbor_idx = -2

    # Calculate direction vector
    endpoint = np.array(
        [
            boundary[endpoint_idx].x,
            boundary[endpoint_idx].y,
            boundary[endpoint_idx].z,
        ]
    )
    neighbor = np.array(
        [
            boundary[neighbor_idx].x,
            boundary[neighbor_idx].y,
            boundary[neighbor_idx].z,
        ]
    )

    if at_start:
        direction = neighbor - endpoint
    else:
        direction = endpoint - neighbor

    length = np.linalg.norm(direction)

    if length < DEFAULT_CONFIG.geometry.epsilon:
        return np.array([1.0, 0.0, 0.0])  # Fallback

    return direction / length


def _get_boundary_start_vel(boundary) -> np.ndarray:
    """
    Calculate start velocity vector for a boundary spline.

    Args:
        boundary: List of points representing a boundary

    Returns:
        3D velocity vector along the boundary direction
    """
    return _calculate_boundary_velocity_vector(boundary, at_start=True)


def _get_boundary_end_vel(boundary) -> np.ndarray:
    """
    Calculate end velocity vector for a boundary spline.

    Args:
        boundary: List of points representing a boundary

    Returns:
        3D velocity vector along the boundary direction
    """
    return _calculate_boundary_velocity_vector(boundary, at_start=False)


def _calculate_centerline_velocity_vector(
    lanelet: lanelet2.core.Lanelet, at_start: bool
) -> np.ndarray:
    """
    Calculate velocity vector for centerline spline from lanelet boundaries.

    This function computes a velocity vector perpendicular to the line segment
    connecting the left and right boundaries at either the start or end of the lanelet.

    Args:
        lanelet: A Lanelet2 lanelet object
        at_start: True for start point, False for end point

    Returns:
        3D velocity vector perpendicular to the line connecting left and right boundaries
    """
    left_bound = lanelet.leftBound
    right_bound = lanelet.rightBound

    if len(left_bound) < 1 or len(right_bound) < 1:
        raise ValueError(
            "Lanelet must have at least 1 point in both left and right boundaries"
        )

    # Select endpoint based on at_start parameter
    idx = 0 if at_start else -1

    # Get points at the selected endpoint
    left_point = np.array([left_bound[idx].x, left_bound[idx].y])
    right_point = np.array([right_bound[idx].x, right_bound[idx].y])

    # Calculate line segment connecting left and right boundaries
    segment = right_point - left_point  # Vector from left to right

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


def _get_start_vel(lanelet: lanelet2.core.Lanelet) -> np.ndarray:
    """
    Calculate start velocity vector for centerline spline from lanelet boundaries.

    Args:
        lanelet: A Lanelet2 lanelet object

    Returns:
        3D velocity vector perpendicular to the line connecting first points of left and right boundaries
    """
    return _calculate_centerline_velocity_vector(lanelet, at_start=True)


def _get_end_vel(lanelet: lanelet2.core.Lanelet) -> np.ndarray:
    """
    Calculate end velocity vector for centerline spline from lanelet boundaries.

    Args:
        lanelet: A Lanelet2 lanelet object

    Returns:
        3D velocity vector perpendicular to the line connecting last points of left and right boundaries
    """
    return _calculate_centerline_velocity_vector(lanelet, at_start=False)


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
    config: WidthEstimationConfig,
) -> Width1DSplineAdapter:
    """
    Estimate lanelet width as a spline by measuring distances between corresponding
    points on borders using direct linear interpolation of original Lanelet2 points.

    Args:
        lanelet: A Lanelet2 lanelet object
        config: WidthEstimationConfig specifying width calculation parameters

    Returns:
        Width1DSplineAdapter object representing width as a function of arc length along the reference

    Raises:
        ValueError: If lanelet has insufficient points
    """
    # Extract parameters from config
    num_samples = config.num_samples
    reference = config.reference.value  # Get string value from enum

    # Get raw boundary points directly from lanelet
    left_bound = lanelet.leftBound
    right_bound = lanelet.rightBound

    if len(left_bound) < 2 or len(right_bound) < 2:
        raise ValueError("Both boundaries must have at least 2 points")

    # Convert to numpy arrays with coordinate offset applied
    left_points = extract_points_3d(left_bound)
    right_points = extract_points_3d(right_bound)

    # Calculate cumulative arc lengths for both boundaries
    boundary_data = _compute_boundary_arc_lengths(left_points, right_points)

    # Sample points along normalized arc length (0 to 1)
    normalized_positions = np.linspace(0.0, 1.0, num_samples)

    # Calculate widths at each normalized position using reference-specific strategy
    arc_lengths, widths = _calculate_widths_by_reference(
        reference,
        normalized_positions,
        left_points,
        right_points,
        boundary_data,
    )

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


def _compute_boundary_arc_lengths(
    left_points: np.ndarray, right_points: np.ndarray
) -> dict:
    """
    Compute cumulative arc lengths for both boundaries.

    Args:
        left_points: Array of left boundary points
        right_points: Array of right boundary points

    Returns:
        Dictionary with cumulative distances and total lengths for both boundaries
    """
    left_dists = np.linalg.norm(np.diff(left_points, axis=0), axis=1)
    left_cumulative = np.concatenate(([0], np.cumsum(left_dists)))
    left_total_length = left_cumulative[-1]

    right_dists = np.linalg.norm(np.diff(right_points, axis=0), axis=1)
    right_cumulative = np.concatenate(([0], np.cumsum(right_dists)))
    right_total_length = right_cumulative[-1]

    return {
        "left_cumulative": left_cumulative,
        "left_total_length": left_total_length,
        "right_cumulative": right_cumulative,
        "right_total_length": right_total_length,
    }


def _calculate_widths_by_reference(
    reference: str,
    normalized_positions: np.ndarray,
    left_points: np.ndarray,
    right_points: np.ndarray,
    boundary_data: dict,
) -> Tuple[List[float], List[float]]:
    """
    Calculate widths at each normalized position based on reference type.

    Args:
        reference: Reference type ("center_line", "left_bound", or "right_bound")
        normalized_positions: Normalized positions [0, 1] to sample
        left_points: Array of left boundary points
        right_points: Array of right boundary points
        boundary_data: Dictionary with boundary arc length data

    Returns:
        Tuple of (arc_lengths, widths) lists
    """
    if reference == "center_line":
        return _calculate_widths_centerline_reference(
            normalized_positions, left_points, right_points, boundary_data
        )
    elif reference == "left_bound":
        return _calculate_widths_left_bound_reference(
            normalized_positions, left_points, right_points, boundary_data
        )
    elif reference == "right_bound":
        return _calculate_widths_right_bound_reference(
            normalized_positions, left_points, right_points, boundary_data
        )
    else:
        raise ValueError(f"Unsupported reference type: {reference}")


def _find_corresponding_points_geometric(
    left_points: np.ndarray,
    right_points: np.ndarray,
    left_cumulative: np.ndarray,
    right_cumulative: np.ndarray,
    num_samples: int = 100,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Find geometrically corresponding points on left/right boundaries.

    Uses perpendicular projection instead of normalized arc length mapping
    to establish correspondence between points on asymmetric boundaries.

    Args:
        left_points: Left boundary points (N x 2)
        right_points: Right boundary points (M x 2)
        left_cumulative: Left boundary cumulative arc lengths
        right_cumulative: Right boundary cumulative arc lengths
        num_samples: Number of sample points

    Returns:
        Tuple of (left_arc_lengths, right_arc_lengths, correspondence_quality)
        - left_arc_lengths: Arc lengths on left boundary (num_samples,)
        - right_arc_lengths: Corresponding arc lengths on right boundary (num_samples,)
        - correspondence_quality: Quality metric for each pair [0, 1] (num_samples,)
    """
    # Sample left boundary uniformly
    left_total_length = left_cumulative[-1]
    right_total_length = right_cumulative[-1]

    # Generate uniform samples on left boundary
    left_arc_lengths = np.linspace(0, left_total_length, num_samples)
    right_arc_lengths = np.zeros(num_samples)
    correspondence_quality = np.zeros(num_samples)

    # Enforce start-to-start and end-to-end correspondence
    right_arc_lengths[0] = 0.0
    right_arc_lengths[-1] = right_total_length
    correspondence_quality[0] = 1.0
    correspondence_quality[-1] = 1.0

    # For each left sample point (except endpoints), find corresponding right point
    for i in range(1, num_samples - 1):
        s_left = left_arc_lengths[i]

        # Interpolate position on left boundary
        left_pos = _interpolate_on_line_segments(left_points, left_cumulative, s_left)

        # Find closest point on right boundary using perpendicular projection
        min_distance = float("inf")
        best_s_right = 0.0

        # Search along right boundary for closest point
        # Use fine sampling for accurate perpendicular projection
        search_samples = max(100, len(right_points) * 10)
        right_search_s = np.linspace(0, right_total_length, search_samples)

        for s_right in right_search_s:
            right_pos = _interpolate_on_line_segments(
                right_points, right_cumulative, s_right
            )
            distance = np.linalg.norm(left_pos - right_pos)

            if distance < min_distance:
                min_distance = distance
                best_s_right = s_right

        right_arc_lengths[i] = best_s_right

        # Calculate quality metric based on distance
        # Quality decreases as distance increases
        max_distance = DEFAULT_CONFIG.geometry.perpendicular_search_radius
        quality = max(0.0, 1.0 - min_distance / max_distance)
        correspondence_quality[i] = quality

    # Check for monotonicity (correspondence should increase along boundaries)
    # Non-monotonic correspondence indicates crossing or reversal
    is_monotonic = np.all(np.diff(right_arc_lengths) >= 0)
    if not is_monotonic:
        # Penalize quality for non-monotonic correspondence
        correspondence_quality *= 0.5

    # Warn if length ratio exceeds threshold
    length_ratio = max(left_total_length, right_total_length) / min(
        left_total_length, right_total_length
    )
    if length_ratio > DEFAULT_CONFIG.geometry.boundary_length_ratio_threshold:
        import warnings

        warnings.warn(
            f"Asymmetric boundaries detected: length ratio {length_ratio:.2f} "
            f"exceeds threshold {DEFAULT_CONFIG.geometry.boundary_length_ratio_threshold}. "
            f"Left: {left_total_length:.2f}m, Right: {right_total_length:.2f}m",
            UserWarning,
        )

    return left_arc_lengths, right_arc_lengths, correspondence_quality


def _calculate_widths_centerline_reference(
    normalized_positions: np.ndarray,
    left_points: np.ndarray,
    right_points: np.ndarray,
    boundary_data: dict,
) -> Tuple[List[float], List[float]]:
    """
    Calculate widths using centerline as reference.

    Args:
        normalized_positions: Normalized positions [0, 1] to sample
        left_points: Array of left boundary points
        right_points: Array of right boundary points
        boundary_data: Dictionary with boundary arc length data

    Returns:
        Tuple of (arc_lengths, widths) lists
    """
    arc_lengths: List[float] = []
    widths: List[float] = []

    # Use geometric correspondence instead of normalized arc length mapping
    left_s_samples, right_s_samples, quality = _find_corresponding_points_geometric(
        left_points,
        right_points,
        boundary_data["left_cumulative"],
        boundary_data["right_cumulative"],
        num_samples=len(normalized_positions),
    )

    # Warn if correspondence quality is low
    min_quality = np.min(quality)
    if min_quality < DEFAULT_CONFIG.geometry.correspondence_quality_threshold:
        import warnings

        warnings.warn(
            f"Low geometric correspondence quality detected: {min_quality:.2f} "
            f"(threshold: {DEFAULT_CONFIG.geometry.correspondence_quality_threshold}). "
            f"Lane width calculations may be inaccurate.",
            UserWarning,
        )

    # Calculate widths using geometrically corresponding points
    prev_center_pos = None
    for i, (s_left, s_right) in enumerate(zip(left_s_samples, right_s_samples)):
        # Interpolate positions on boundaries
        left_pos = _interpolate_on_line_segments(
            left_points, boundary_data["left_cumulative"], s_left
        )
        right_pos = _interpolate_on_line_segments(
            right_points, boundary_data["right_cumulative"], s_right
        )

        # Calculate centerline position as midpoint
        center_pos = (left_pos + right_pos) / 2.0

        # Width is the total distance from center to both borders
        left_dist = np.linalg.norm(center_pos - left_pos)
        right_dist = np.linalg.norm(center_pos - right_pos)
        width = left_dist + right_dist

        # Arc length is based on centerline
        if i == 0:
            arc_length = 0.0
        else:
            # Calculate arc length increment from previous centerline position
            arc_length = arc_lengths[-1] + np.linalg.norm(center_pos - prev_center_pos)

        arc_lengths.append(arc_length)
        widths.append(width)
        prev_center_pos = center_pos

    return arc_lengths, widths


def _calculate_widths_left_bound_reference(
    normalized_positions: np.ndarray,
    left_points: np.ndarray,
    right_points: np.ndarray,
    boundary_data: dict,
) -> Tuple[List[float], List[float]]:
    """
    Calculate widths using left boundary as reference.

    Args:
        normalized_positions: Normalized positions [0, 1] to sample
        left_points: Array of left boundary points
        right_points: Array of right boundary points
        boundary_data: Dictionary with boundary arc length data

    Returns:
        Tuple of (arc_lengths, widths) lists
    """
    arc_lengths: List[float] = []
    widths: List[float] = []

    # Use geometric correspondence instead of normalized arc length mapping
    left_s_samples, right_s_samples, quality = _find_corresponding_points_geometric(
        left_points,
        right_points,
        boundary_data["left_cumulative"],
        boundary_data["right_cumulative"],
        num_samples=len(normalized_positions),
    )

    # Warn if correspondence quality is low
    min_quality = np.min(quality)
    if min_quality < DEFAULT_CONFIG.geometry.correspondence_quality_threshold:
        import warnings

        warnings.warn(
            f"Low geometric correspondence quality detected: {min_quality:.2f} "
            f"(threshold: {DEFAULT_CONFIG.geometry.correspondence_quality_threshold}). "
            f"Lane width calculations may be inaccurate.",
            UserWarning,
        )

    # Calculate widths using geometrically corresponding points
    for s_left, s_right in zip(left_s_samples, right_s_samples):
        left_pos = _interpolate_on_line_segments(
            left_points, boundary_data["left_cumulative"], s_left
        )
        right_pos = _interpolate_on_line_segments(
            right_points, boundary_data["right_cumulative"], s_right
        )

        # Width is distance from left to right border
        width = np.linalg.norm(left_pos - right_pos)
        arc_length = s_left

        arc_lengths.append(arc_length)
        widths.append(width)

    return arc_lengths, widths


def _calculate_widths_right_bound_reference(
    normalized_positions: np.ndarray,
    left_points: np.ndarray,
    right_points: np.ndarray,
    boundary_data: dict,
) -> Tuple[List[float], List[float]]:
    """
    Calculate widths using right boundary as reference.

    Args:
        normalized_positions: Normalized positions [0, 1] to sample
        left_points: Array of left boundary points
        right_points: Array of right boundary points
        boundary_data: Dictionary with boundary arc length data

    Returns:
        Tuple of (arc_lengths, widths) lists
    """
    arc_lengths: List[float] = []
    widths: List[float] = []

    # Use geometric correspondence instead of normalized arc length mapping
    left_s_samples, right_s_samples, quality = _find_corresponding_points_geometric(
        left_points,
        right_points,
        boundary_data["left_cumulative"],
        boundary_data["right_cumulative"],
        num_samples=len(normalized_positions),
    )

    # Warn if correspondence quality is low
    min_quality = np.min(quality)
    if min_quality < DEFAULT_CONFIG.geometry.correspondence_quality_threshold:
        import warnings

        warnings.warn(
            f"Low geometric correspondence quality detected: {min_quality:.2f} "
            f"(threshold: {DEFAULT_CONFIG.geometry.correspondence_quality_threshold}). "
            f"Lane width calculations may be inaccurate.",
            UserWarning,
        )

    # Calculate widths using geometrically corresponding points
    for s_left, s_right in zip(left_s_samples, right_s_samples):
        left_pos = _interpolate_on_line_segments(
            left_points, boundary_data["left_cumulative"], s_left
        )
        right_pos = _interpolate_on_line_segments(
            right_points, boundary_data["right_cumulative"], s_right
        )

        # Width is distance from right to left border
        width = np.linalg.norm(right_pos - left_pos)
        arc_length = s_right

        arc_lengths.append(arc_length)
        widths.append(width)

    return arc_lengths, widths


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
