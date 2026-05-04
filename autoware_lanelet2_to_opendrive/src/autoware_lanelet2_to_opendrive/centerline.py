import logging
import numpy as np
import lanelet2
from typing import TYPE_CHECKING, List, Optional, Set, Tuple
from .config import DEFAULT_CONFIG
from .spline import Splines
from .util import sort_adjacent_groups, extract_points_3d, extract_points_2d
from .cubic_spline_1d import CubicSpline1D
from .conversion_config import WidthEstimationConfig, WidthReference

if TYPE_CHECKING:
    # Avoid runtime circular import (opendrive package's __init__ chains back
    # into this module via opendrive_dataclass → road → centerline). The
    # actual ``LanePolynomial`` import happens inside
    # ``compute_lane_outer_polynomial`` at call time.
    from .opendrive.lane_elements import LanePolynomial

logger = logging.getLogger(__name__)


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
    lanelet: lanelet2.core.Lanelet, num_control_points: Optional[int] = None
) -> Splines:
    """
    Extract centerline from a Lanelet using midpoints between left and right borders.

    Uses line segment representation of borders to calculate centerline points
    by interpolating along both borders using normalized coordinates.

    Args:
        lanelet: A Lanelet2 lanelet object
        num_control_points: Number of control points for B-spline interpolation.
                           If None, automatically computed based on geometry complexity.

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
    # If num_control_points is None, estimate from input points
    if num_control_points is None:
        estimated_samples = max(len(left_points), len(right_points))
        num_samples = max(
            DEFAULT_CONFIG.centerline.min_sample_points_for_centerline,
            estimated_samples,
        )
    else:
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
    num_control_points: Optional[int] = None,
    dimensions: int = 2,
) -> Splines:
    """
    Extract border line from a Lanelet and return as B-spline with arc length parameterization.

    Args:
        lanelet: A Lanelet2 lanelet object
        border: Border specification - "left" or "right"
        num_control_points: Number of control points for B-spline interpolation.
                           If None, automatically computed based on geometry complexity.
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

    # Get velocity vectors perpendicular to the line connecting left and right boundaries
    # This ensures both left and right boundaries have consistent tangent directions at endpoints,
    # preventing gaps in OpenDRIVE lane geometry
    start_vel = _calculate_centerline_velocity_vector(lanelet, at_start=True)[:2]
    end_vel = _calculate_centerline_velocity_vector(lanelet, at_start=False)[:2]

    # Create B-spline with constrained fitting
    return Splines(
        points,
        start_vel=start_vel,
        end_vel=end_vel,
        num_control_points=num_control_points,
    )


def _calculate_optimal_num_samples(
    total_length: float,
    config: WidthEstimationConfig,
) -> int:
    """
    Calculate optimal number of samples based on road length.

    Args:
        total_length: Total length of the road/lanelet
        config: WidthEstimationConfig containing sampling parameters

    Returns:
        Optimal number of samples (clamped to [min_samples, max_samples])

    Examples:
        >>> _calculate_optimal_num_samples(50.0, config)  # 50m road
        10  # 10 samples at 5m intervals

        >>> _calculate_optimal_num_samples(10.0, config)  # 10m road
        5  # min_samples (clamped)
    """
    if not config.adaptive_sampling:
        # Use fixed sampling
        return config.num_samples

    if total_length <= 0:
        return config.min_samples

    # Calculate based on target interval
    num_by_interval = int(np.ceil(total_length / config.default_sample_interval))

    # Clamp to valid range
    num_samples = max(config.min_samples, min(num_by_interval, config.max_samples))

    return num_samples


def estimate_lanelet_width_with_reference_line(
    lanelet: lanelet2.core.Lanelet,
    reference_line_spline: "Splines",
    config: WidthEstimationConfig,
    anchor_start_override: Optional[Tuple[float, float, float]] = None,
    anchor_end_override: Optional[Tuple[float, float, float]] = None,
) -> Width1DSplineAdapter:
    """
    Estimate lanelet width as a spline using road reference line s-coordinates.

    This function calculates width by aligning s-coordinates to the road reference
    line instead of individual lanelet boundaries. This ensures correct width
    representation in OpenDRIVE format, especially for high-curvature sections.

    Args:
        lanelet: The lanelet to calculate width for
        reference_line_spline: Road reference line spline for s-coordinate alignment
        config: Width estimation configuration
        anchor_start_override: Optional (x, y, z) coordinate that replaces the
            first sample of the anchor boundary before width measurement.  Set
            by the junction phase for the OUTERMOST lanelet of a connecting
            road so the lane width at s=0 matches the linked regular road's
            outer lane edge (P0-2 lane-width side of the override).  Only the
            XY components are used; Z is ignored.
        anchor_end_override: Optional (x, y, z) coordinate that replaces the
            last sample of the anchor boundary, used for the s=length side.

    Returns:
        Width1DSplineAdapter with s-coordinates aligned to road reference line

    Raises:
        ValueError: If reference type is not supported or lanelet boundaries are invalid
    """
    # Handle centerline reference - not affected by this issue
    if config.reference == WidthReference.CENTER_LINE:
        return estimate_lanelet_width_as_spline(lanelet, config)

    # Extract boundary points
    from .util import extract_points_3d

    left_points = extract_points_3d(lanelet.leftBound)
    right_points = extract_points_3d(lanelet.rightBound)

    # Determine anchor boundary based on traffic rule
    if config.reference == WidthReference.LEFT_BOUND:
        # RHT: Use left boundary as anchor
        anchor_points = left_points
        other_points = right_points
    elif config.reference == WidthReference.RIGHT_BOUND:
        # LHT: Use right boundary as anchor
        anchor_points = right_points
        other_points = left_points
    else:
        raise ValueError(f"Unsupported width reference: {config.reference}")

    # Apply anchor-boundary endpoint overrides for the OUTERMOST lanelet of a
    # connecting road (P0-2 junction endpoint fidelity, lane-width side).
    # The reference line's s=0 / s=length is pinned to the linked regular
    # road's endpoint XY; without this override the width at those s would
    # still be |original anchor - other| and lane corridors would drift
    # laterally relative to the regular road.  Replacing anchor[0] / [-1]
    # with the override keeps lane -1 / lane +1 inner edge on the reference
    # line and outer edge near the lanelet's other-side boundary, which is
    # what the regular road's matching lane edge resolves to in practice.
    if anchor_start_override is not None or anchor_end_override is not None:
        anchor_points = anchor_points.copy()
        if anchor_start_override is not None:
            anchor_points[0, :2] = np.asarray(anchor_start_override, dtype=float)[:2]
        if anchor_end_override is not None:
            anchor_points[-1, :2] = np.asarray(anchor_end_override, dtype=float)[:2]

    # Get road reference line total length
    road_length = reference_line_spline.total_length

    # Calculate arc lengths for anchor and other boundaries
    anchor_dists = np.linalg.norm(np.diff(anchor_points[:, :2], axis=0), axis=1)
    anchor_cumulative = np.concatenate(([0], np.cumsum(anchor_dists)))
    anchor_total_length = anchor_cumulative[-1]

    other_dists = np.linalg.norm(np.diff(other_points[:, :2], axis=0), axis=1)
    other_cumulative = np.concatenate(([0], np.cumsum(other_dists)))
    other_total_length = other_cumulative[-1]

    # Log boundary length information
    logger.debug(
        f"Width calculation for lanelet {lanelet.id}: "
        f"road_length={road_length:.3f}m, "
        f"anchor_length={anchor_total_length:.3f}m, "
        f"other_length={other_total_length:.3f}m, "
        f"reference={config.reference}"
    )

    # Determine number of samples using same logic as existing function
    num_samples = _calculate_optimal_num_samples(road_length, config)
    normalized_positions = np.linspace(0, 1, num_samples)

    arc_lengths: List[float] = []
    widths: List[float] = []

    for t_norm in normalized_positions:
        # Map normalized position to s-coordinate on road reference line
        s_road = t_norm * road_length

        # Map normalized position to arc lengths on lanelet boundaries
        s_anchor = t_norm * anchor_total_length
        s_other = t_norm * other_total_length

        # Interpolate positions on boundaries
        anchor_pos = _interpolate_on_line_segments(
            anchor_points, anchor_cumulative, s_anchor
        )
        other_pos = _interpolate_on_line_segments(
            other_points, other_cumulative, s_other
        )

        # Calculate width as distance between boundaries
        width = np.linalg.norm(anchor_pos - other_pos)

        # Use road reference line s-coordinate
        arc_lengths.append(s_road)
        widths.append(width)

    # Log width statistics
    widths_array = np.array(widths)
    logger.debug(
        f"Width statistics for lanelet {lanelet.id}: "
        f"min={widths_array.min():.3f}m, "
        f"max={widths_array.max():.3f}m, "
        f"mean={widths_array.mean():.3f}m, "
        f"std={widths_array.std():.3f}m"
    )

    # Validate s-coordinates are monotonically increasing
    arc_lengths_array = np.array(arc_lengths)
    if not np.all(np.diff(arc_lengths_array) >= 0):
        logger.error(
            f"s-coordinates are not monotonic for lanelet {lanelet.id} - width calculation may be incorrect"
        )

    # Create 1D cubic spline from (s_road, width) pairs
    spline = CubicSpline1D(arc_lengths_array, widths_array, bc_type="not-a-knot")

    return Width1DSplineAdapter(spline)


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

    # Calculate optimal number of samples (adaptive or fixed)
    # Use the maximum of left/right boundary lengths as total length
    total_length = max(
        boundary_data["left_total_length"], boundary_data["right_total_length"]
    )
    num_samples = _calculate_optimal_num_samples(total_length, config)

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
        return _calculate_widths_boundary_reference(
            normalized_positions,
            left_points,
            right_points,
            boundary_data,
            use_left_as_reference=True,
        )
    elif reference == "right_bound":
        return _calculate_widths_boundary_reference(
            normalized_positions,
            left_points,
            right_points,
            boundary_data,
            use_left_as_reference=False,
        )
    else:
        raise ValueError(f"Unsupported reference type: {reference}")


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

    for t_norm in normalized_positions:
        # Convert normalized position to actual arc length for each boundary
        s_left = t_norm * boundary_data["left_total_length"]
        s_right = t_norm * boundary_data["right_total_length"]

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
        if len(arc_lengths) == 0:
            arc_length = 0.0
        else:
            # Calculate arc length increment from previous centerline position
            prev_t_norm = normalized_positions[len(arc_lengths) - 1]
            prev_center_pos = (
                _interpolate_on_line_segments(
                    left_points,
                    boundary_data["left_cumulative"],
                    prev_t_norm * boundary_data["left_total_length"],
                )
                + _interpolate_on_line_segments(
                    right_points,
                    boundary_data["right_cumulative"],
                    prev_t_norm * boundary_data["right_total_length"],
                )
            ) / 2.0
            arc_length = arc_lengths[-1] + np.linalg.norm(center_pos - prev_center_pos)

        arc_lengths.append(arc_length)
        widths.append(width)

    return arc_lengths, widths


def _calculate_widths_boundary_reference(
    normalized_positions: np.ndarray,
    left_points: np.ndarray,
    right_points: np.ndarray,
    boundary_data: dict,
    use_left_as_reference: bool,
) -> Tuple[List[float], List[float]]:
    """
    Calculate widths using either the left or right boundary as arc-length reference.

    Args:
        normalized_positions: Normalized positions [0, 1] to sample
        left_points: Array of left boundary points
        right_points: Array of right boundary points
        boundary_data: Dictionary with boundary arc length data
        use_left_as_reference: If True, use left boundary arc length as the reference;
            if False, use right boundary arc length as the reference.

    Returns:
        Tuple of (arc_lengths, widths) lists
    """
    arc_lengths: List[float] = []
    widths: List[float] = []

    for t_norm in normalized_positions:
        s_left = t_norm * boundary_data["left_total_length"]
        s_right = t_norm * boundary_data["right_total_length"]

        left_pos = _interpolate_on_line_segments(
            left_points, boundary_data["left_cumulative"], s_left
        )
        right_pos = _interpolate_on_line_segments(
            right_points, boundary_data["right_cumulative"], s_right
        )

        # Width is the distance between the two boundary points
        width = np.linalg.norm(left_pos - right_pos)
        arc_length = s_left if use_left_as_reference else s_right

        arc_lengths.append(arc_length)
        widths.append(width)

    return arc_lengths, widths


def extract_centerline_as_spline_from_two_lanelets(
    lanelet_map: lanelet2.core.LaneletMap,
    two_lanelets: Set[lanelet2.core.Lanelet],
    num_control_points: Optional[int] = None,
) -> Splines:
    """
    Extract centerline as spline from two adjacent lanelets using the left lanelet's right bound.

    Args:
        lanelet_map: The lanelet2 map containing the lanelets
        two_lanelets: Set containing exactly two lanelets
        num_control_points: Number of control points for B-spline interpolation.
                           If None, automatically computed based on geometry complexity.

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


# ---------------------------------------------------------------------------
# Issue #440: <lane><border> emission for asymmetric lanelets
# ---------------------------------------------------------------------------
#
# The cubic <width> emission interpolates a few sample widths exactly; for
# many real-world lanelets (curved, mostly symmetric) that is faithful. For
# the tail (one-sided bulges, S-shapes, sharp asymmetries) the cubic places
# the outer edge somewhere the actual rightBound / leftBound is not. The
# perpendicular-projection trigger below detects that mis-fit and routes
# the lanelet through ``<lane><border>`` instead, which lets us state the
# outer edge as a signed-t cubic against the road reference line directly.


def _closest_point_on_polyline_2d(p: np.ndarray, polyline_xy: np.ndarray) -> np.ndarray:
    """Return the 2D point on ``polyline_xy`` closest to ``p``.

    Args:
        p: 2D query point with shape (2,).
        polyline_xy: Polyline vertices with shape (N, 2), N >= 2.

    Returns:
        2D point on the polyline (interior of a segment or a vertex).
    """
    eps = DEFAULT_CONFIG.geometry.epsilon
    best_pt = polyline_xy[0].astype(float, copy=True)
    best_d2 = float("inf")
    for i in range(len(polyline_xy) - 1):
        a = polyline_xy[i]
        b = polyline_xy[i + 1]
        ab = b - a
        ab_norm2 = float(np.dot(ab, ab))
        if ab_norm2 < eps:
            cand = a.astype(float, copy=True)
        else:
            t = float(np.dot(p - a, ab) / ab_norm2)
            t_clamped = max(0.0, min(1.0, t))
            cand = a + t_clamped * ab
        d2 = float(np.sum((p - cand) ** 2))
        if d2 < best_d2:
            best_d2 = d2
            best_pt = cand
    return best_pt


def _polyline_tangent_2d(
    points_xy: np.ndarray, cumulative: np.ndarray, s: float
) -> np.ndarray:
    """Return the 2D tangent direction of a polyline at arc length ``s``.

    Returned vector is the unnormalised segment direction (caller normalises).
    """
    s = float(np.clip(s, 0.0, cumulative[-1]))
    seg_idx = int(np.searchsorted(cumulative, s) - 1)
    seg_idx = int(np.clip(seg_idx, 0, len(points_xy) - 2))
    return points_xy[seg_idx + 1] - points_xy[seg_idx]


def _max_outer_bound_deviation(
    lanelet,
    reference_line_spline: "Splines",
    width_adapter: Width1DSplineAdapter,
    config: WidthEstimationConfig,
    rule: str,
) -> float:
    """Maximum perpendicular distance between the cubic-``<width>``-predicted
    outer edge and the actual outer-bound polyline of a lanelet.

    For each sample at uniform ``t_norm`` ∈ [0, 1] across the lanelet:

    1. Locate the anchor point ``p_anchor`` on the lanelet's anchor-bound
       polyline at arc length ``t_norm * anchor_total_length``.
    2. Compute the 2D unit normal to the anchor tangent, signed toward the
       outer side (right-hand for RHT, left-hand for LHT).
    3. Predict the outer edge:
       ``p_pred = p_anchor + width(s_road) * n_outer``,
       where ``s_road = t_norm * road_length`` is the road-reference-line
       arc length passed to ``width_adapter``.
    4. Find the closest 2D point ``q`` on the actual outer-bound polyline
       to ``p_pred``.
    5. ``deviation_i = ||p_pred - q||``.

    The maximum across samples is returned. A perfectly-fit symmetric
    lanelet returns ≈ 0; a lanelet whose cubic ``<width>`` mis-locates the
    outer edge (S-shape, one-sided bulge) returns a value at least as
    large as the worst mis-location in metres.

    For ``WidthReference.CENTER_LINE`` (no perpendicular concept tied to a
    specific bound) this returns 0.0 unconditionally — those lanelets
    keep the existing ``<width>`` path.

    Args:
        lanelet: lanelet-like object with ``leftBound`` / ``rightBound``
            iterables of points exposing ``.x .y .z``.
        reference_line_spline: Road reference line.
        width_adapter: Adapter returned by
            ``estimate_lanelet_width_with_reference_line``.
        config: ``WidthEstimationConfig`` (only ``num_samples`` is used).
        rule: ``"RHT"`` or ``"LHT"``. Selects the lanelet's anchor /
            outer bounds to match
            ``estimate_lanelet_width_with_reference_line``.

    Returns:
        Maximum perpendicular deviation in metres, or ``0.0`` if the
        lanelet is too short to sample.
    """
    if config.reference == WidthReference.CENTER_LINE:
        return 0.0

    rule_upper = (rule or "RHT").upper()
    if rule_upper not in ("RHT", "LHT"):
        raise ValueError(f"rule must be 'RHT' or 'LHT', got {rule!r}")

    if rule_upper == "RHT":
        anchor_points = extract_points_3d(lanelet.leftBound)
        outer_points = extract_points_3d(lanelet.rightBound)
        # Outer (rightBound) is to the RIGHT of the anchor tangent.
        # Right-hand 2D normal: rotate tangent -90° CCW → (+τ_y, -τ_x).
        normal_sign = -1.0
    else:  # "LHT"
        anchor_points = extract_points_3d(lanelet.rightBound)
        outer_points = extract_points_3d(lanelet.leftBound)
        normal_sign = +1.0

    if len(anchor_points) < 2 or len(outer_points) < 2:
        return 0.0

    eps = DEFAULT_CONFIG.geometry.epsilon
    anchor_xy = anchor_points[:, :2]
    outer_xy = outer_points[:, :2]

    anchor_dists = np.linalg.norm(np.diff(anchor_xy, axis=0), axis=1)
    anchor_cum = np.concatenate(([0.0], np.cumsum(anchor_dists)))
    anchor_total = float(anchor_cum[-1])
    if anchor_total < eps:
        return 0.0

    road_length = float(reference_line_spline.total_length)
    if road_length < eps:
        return 0.0

    num_samples = max(_calculate_optimal_num_samples(road_length, config), 4)
    normalized = np.linspace(0.0, 1.0, num_samples)

    max_dev = 0.0
    for t_norm in normalized:
        s_anchor = float(t_norm) * anchor_total
        p_anchor = _interpolate_on_line_segments(anchor_xy, anchor_cum, s_anchor)
        tau = _polyline_tangent_2d(anchor_xy, anchor_cum, s_anchor)
        tau_norm = float(np.linalg.norm(tau))
        if tau_norm < eps:
            continue
        tau_unit = tau / tau_norm
        n_left = np.array([-tau_unit[1], tau_unit[0]])
        n_outer = normal_sign * n_left

        s_road = float(t_norm) * road_length
        width = float(width_adapter.get_width_at_arc_length(s_road))
        p_pred = p_anchor + width * n_outer

        q = _closest_point_on_polyline_2d(p_pred, outer_xy)
        d = float(np.linalg.norm(p_pred - q))
        if d > max_dev:
            max_dev = d

    return max_dev


def _fit_signed_t_spline(
    bound_points: np.ndarray,
    reference_line_spline: "Splines",
    config: WidthEstimationConfig,
) -> CubicSpline1D:
    """Fit a cubic spline of signed t(s) for an outer-bound polyline against
    the road reference line.

    Sampling strategy: uniform reference-line arc length ``s ∈ [0, L_ref]``.
    At each ``s``:

    1. ``p_ref(s)`` = reference line position.
    2. ``τ(s)`` = reference line tangent.
    3. ``n_left(s)`` = unit ``+90°`` CCW rotation of ``τ`` (OpenDRIVE
       convention: t > 0 is left of the reference line).
    4. ``q(s)`` = closest 2D point on ``bound_points[:, :2]`` to
       ``p_ref(s)``.
    5. ``t_signed(s) = (q(s) - p_ref(s)) · n_left(s)``.

    Sign by construction:
      - RHT outer = rightBound (right of ref line)  → ``t_signed < 0``.
      - LHT outer = leftBound  (left of ref line)   → ``t_signed > 0``.

    Sampling at uniform ``s`` guarantees the first segment has
    ``sOffset = 0.0`` and the curve covers the full road s range, matching
    the existing ``<width>`` emission convention.

    Args:
        bound_points: Outer-bound polyline (N, 3) in world frame.
        reference_line_spline: Road reference line.
        config: ``WidthEstimationConfig`` (only ``num_samples`` is used).

    Returns:
        ``CubicSpline1D`` mapping reference-line ``s`` to signed t with
        ``bc_type="not-a-knot"``.

    Raises:
        ValueError: if fewer than two boundary points are supplied or the
            reference line has zero length.
    """
    if bound_points.shape[0] < 2:
        raise ValueError(
            f"Need at least 2 boundary points for signed-t fit, "
            f"got {bound_points.shape[0]}"
        )

    L = float(reference_line_spline.total_length)
    eps = DEFAULT_CONFIG.geometry.epsilon
    if L < eps:
        raise ValueError("Reference line has zero length; cannot fit signed t(s).")

    bound_xy = bound_points[:, :2]
    num_samples = max(_calculate_optimal_num_samples(L, config), 4)
    s_values = np.linspace(0.0, L, num_samples)
    t_values: List[float] = []

    last_t: float = 0.0
    for s in s_values:
        p_ref = reference_line_spline.evaluate(float(s), derivative=0)[:2]
        tau = reference_line_spline.evaluate(float(s), derivative=1)[:2]
        tau_norm = float(np.linalg.norm(tau))
        if tau_norm < eps:
            t_values.append(last_t)
            continue
        n_left = np.array([-tau[1], tau[0]]) / tau_norm
        q = _closest_point_on_polyline_2d(p_ref, bound_xy)
        t_signed = float(np.dot(q - p_ref, n_left))
        t_values.append(t_signed)
        last_t = t_signed

    return CubicSpline1D(
        np.asarray(s_values, dtype=float),
        np.asarray(t_values, dtype=float),
        bc_type="not-a-knot",
    )


def compute_lane_outer_polynomial(
    lanelet,
    reference_line_spline: "Splines",
    config: WidthEstimationConfig,
    *,
    rule: str,
    anchor_start_override: Optional[Tuple[float, float, float]] = None,
    anchor_end_override: Optional[Tuple[float, float, float]] = None,
    deviation_tolerance: Optional[float] = None,
) -> "LanePolynomial":
    """Decide whether a lanelet emits as ``<lane><width>`` or ``<lane><border>``.

    Default path: run ``estimate_lanelet_width_with_reference_line`` and
    return the result wrapped in ``LanePolynomial(kind="width", ...)`` —
    numerically identical to today's output.

    Trigger: the cubic-``<width>``-predicted outer edge mis-locates the
    actual outer-bound polyline by more than
    ``deviation_tolerance`` metres
    (default ``DEFAULT_CONFIG.lane_border.outer_bound_deviation_tolerance``,
    0.30 m). On trigger, fits a signed-t cubic of the outer-bound polyline
    against the road reference line and returns
    ``LanePolynomial(kind="border", ...)``.

    Args:
        lanelet: Lanelet2 lanelet (or mock with ``leftBound`` / ``rightBound``
            / ``id``).
        reference_line_spline: Road reference line.
        config: ``WidthEstimationConfig`` used by the width estimator and
            sample-count helpers.
        rule: ``"RHT"`` or ``"LHT"``. Selects the outer bound for both
            the trigger metric and the border fit.
        anchor_start_override: Optional ``(x, y, z)`` override forwarded
            to the width estimator (junction endpoint pinning).
        anchor_end_override: Optional ``(x, y, z)`` override at the
            ``s = length`` end of the anchor.
        deviation_tolerance: Optional override of the
            ``DEFAULT_CONFIG.lane_border.outer_bound_deviation_tolerance``
            value, in metres.

    Returns:
        ``LanePolynomial`` with ``kind == "width"`` or ``kind == "border"``.
    """
    from .opendrive.lane_elements import LanePolynomial

    rule_upper = (rule or "RHT").upper()
    if rule_upper not in ("RHT", "LHT"):
        raise ValueError(f"rule must be 'RHT' or 'LHT', got {rule!r}")

    # Anchor selection in ``estimate_lanelet_width_with_reference_line`` is
    # driven by ``config.reference`` (LEFT_BOUND for RHT, RIGHT_BOUND for
    # LHT). The deviation metric and border fit below derive their anchor /
    # outer choice from ``rule``. If those disagree, the width adapter and
    # the deviation logic operate on different bounds — silently producing
    # the wrong border sign or false-positive triggers.
    if rule_upper == "RHT" and config.reference == WidthReference.RIGHT_BOUND:
        raise ValueError(
            "rule='RHT' is inconsistent with config.reference=RIGHT_BOUND; "
            "RHT pairs with LEFT_BOUND (or CENTER_LINE)."
        )
    if rule_upper == "LHT" and config.reference == WidthReference.LEFT_BOUND:
        raise ValueError(
            "rule='LHT' is inconsistent with config.reference=LEFT_BOUND; "
            "LHT pairs with RIGHT_BOUND (or CENTER_LINE)."
        )

    tol = (
        deviation_tolerance
        if deviation_tolerance is not None
        else DEFAULT_CONFIG.lane_border.outer_bound_deviation_tolerance
    )

    width_adapter = estimate_lanelet_width_with_reference_line(
        lanelet,
        reference_line_spline,
        config,
        anchor_start_override=anchor_start_override,
        anchor_end_override=anchor_end_override,
    )

    deviation = _max_outer_bound_deviation(
        lanelet, reference_line_spline, width_adapter, config, rule_upper
    )

    if deviation <= tol:
        return LanePolynomial(
            kind="width",
            segments=width_adapter.get_polynomial_segments(),
            total_length=float(width_adapter.total_length),
        )

    outer_bound = lanelet.rightBound if rule_upper == "RHT" else lanelet.leftBound
    bound_points = extract_points_3d(outer_bound)
    border_spline = _fit_signed_t_spline(bound_points, reference_line_spline, config)
    logger.info(
        f"Lanelet {getattr(lanelet, 'id', '?')}: emitting <border> "
        f"(outer-bound deviation={deviation:.3f}m > tol={tol:.3f}m)"
    )
    return LanePolynomial(
        kind="border",
        segments=border_spline.get_segments(),
        total_length=float(border_spline.total_arc_length),
    )
