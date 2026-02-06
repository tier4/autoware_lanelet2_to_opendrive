"""Configuration dataclasses for conversion parameters and constants.

This module centralizes all magic numbers and configurable parameters used
throughout the Lanelet2 to OpenDRIVE conversion process. All constants are
organized into logical dataclasses for easy access and modification.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class GeometryConstants:
    """Constants for geometry calculations and numerical stability.

    Attributes:
        epsilon: Tolerance for numerical stability in geometry calculations
        point_distance_threshold: Minimum distance between distinct points
        param_poly3_num_segments: Number of ParamPoly3 segments to create from spline.
                                  Higher values create smoother curves with less heading
                                  discontinuity but increase file size. Recommended: 20-50.
        enable_adaptive_subdivision: Enable adaptive ParamPoly3 segment subdivision
                                    based on heading change detection
        max_heading_change_deg: Maximum allowed heading change per segment (degrees)
                               for adaptive subdivision. Segments exceeding this will be
                               subdivided automatically.
        max_subdivision_iterations: Maximum number of adaptive subdivision iterations
    """

    epsilon: float = 1e-10
    point_distance_threshold: float = 0.001
    param_poly3_num_segments: int = 30
    enable_adaptive_subdivision: bool = False
    max_heading_change_deg: float = 30.0
    max_subdivision_iterations: int = 3


@dataclass(frozen=True)
class SplineConstants:
    """Constants for B-spline fitting and interpolation.

    Attributes:
        speed_epsilon: Tolerance for speed/velocity magnitude checks
        hard_constraint_weight: Weight for boundary position/velocity constraints
        soft_constraint_weight: Weight for intermediate point data fitting
        knot_alpha_weight: Weight for uniform knot distribution
        knot_beta_weight: Weight for curvature-adaptive knot placement
        position_tolerance: Maximum error for endpoint position constraints
        velocity_tolerance: Maximum error for endpoint velocity constraints
        max_avg_error: Maximum acceptable average fitting error
        max_point_error: Maximum acceptable single-point fitting error
        warn_percentile: Percentile for error reporting in warnings
        arc_length_table_samples: Number of samples for arc length lookup table
    """

    speed_epsilon: float = 1e-12
    hard_constraint_weight: float = 80.0
    soft_constraint_weight: float = 20.0
    knot_alpha_weight: float = 2.0
    knot_beta_weight: float = 2.0
    position_tolerance: float = 5.0
    velocity_tolerance: float = 15.0
    max_avg_error: float = 2.0
    max_point_error: float = 8.0
    warn_percentile: float = 95.0
    arc_length_table_samples: int = 1000


@dataclass(frozen=True)
class CenterlineConstants:
    """Constants for centerline extraction and width estimation.

    Attributes:
        min_sample_points_for_centerline: Minimum number of sample points for centerline extraction
        sample_point_multiplier: Multiplier for calculating sample points based on control points
        boundary_condition_default: Default boundary condition for cubic spline interpolation
    """

    min_sample_points_for_centerline: int = 20
    sample_point_multiplier: int = 2
    boundary_condition_default: str = "not-a-knot"


@dataclass(frozen=True)
class PreprocessingConstants:
    """Constants for lanelet preprocessing operations.

    Attributes:
        merge_tolerance_default: Default tolerance for lanelet merging
        replace_tolerance_default: Default tolerance for lanelet replacement
        validate_tolerance_default: Default tolerance for continuity validation
    """

    merge_tolerance_default: float = 1e-3
    replace_tolerance_default: float = 1e-3
    validate_tolerance_default: float = 1e-3


@dataclass(frozen=True)
class OpenDriveConstants:
    """Constants for OpenDRIVE format and ID management.

    Issue #132 fix: OpenDRIVE specification allows roads and junctions to share
    the same ID space, but some tools (e.g., CARLA MapBuilder) cannot distinguish
    between them, causing ID collisions. To prevent this, we add an offset to
    junction IDs to ensure they never conflict with road IDs.

    Attributes:
        junction_id_offset: Offset added to all junction IDs to avoid conflicts
                           with road IDs. Default is 1000, which means:
                           - Junction 0 becomes ID 1000
                           - Junction 1 becomes ID 1001
                           - etc.
                           This ensures junction IDs never collide with road IDs
                           even for large maps with hundreds of roads.
    """

    junction_id_offset: int = 1000


@dataclass(frozen=True)
class ConversionConfig:
    """Main configuration container for all conversion constants.

    This dataclass provides centralized access to all constant configurations
    used throughout the conversion process. Create an instance to access
    different constant groups.

    Example:
        ```python
        from autoware_lanelet2_to_opendrive.config import ConversionConfig

        config = ConversionConfig()

        # Access geometry constants
        if distance < config.geometry.epsilon:
            # handle near-zero case
            pass

        # Access spline constants
        spline = Splines(
            points,
            num_control_points=10,
            hard_weight=config.spline.hard_constraint_weight,
            soft_weight=config.spline.soft_constraint_weight,
        )

        # Access preprocessing constants
        merged = merge_lanelets(
            map, ids, tolerance=config.preprocessing.merge_tolerance_default
        )
        ```

    Attributes:
        geometry: Geometry calculation constants
        spline: Spline fitting constants
        centerline: Centerline extraction constants
        preprocessing: Preprocessing operation constants
        opendrive: OpenDRIVE format constants
    """

    geometry: GeometryConstants = GeometryConstants()
    spline: SplineConstants = SplineConstants()
    centerline: CenterlineConstants = CenterlineConstants()
    preprocessing: PreprocessingConstants = PreprocessingConstants()
    opendrive: OpenDriveConstants = OpenDriveConstants()


@dataclass
class CoordinateOffset:
    """Runtime coordinate offset configuration.

    This class holds the coordinate offset values that are applied to all
    coordinates during OpenDRIVE export. The offset is subtracted from
    Lanelet2 coordinates to convert them to local coordinates.

    When offset is configured (e.g., from MGRS grid + offset in config),
    all coordinates in the output xodr file will be shifted by subtracting
    these offset values, making them relative to the offset point.

    Attributes:
        x: Easting offset in meters to subtract from X coordinates
        y: Northing offset in meters to subtract from Y coordinates
        z: Altitude offset in meters to subtract from Z coordinates
    """

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def set(self, x: float, y: float, z: float = 0.0) -> None:
        """Set coordinate offset values.

        Args:
            x: Easting offset in meters
            y: Northing offset in meters
            z: Altitude offset in meters (default 0.0)
        """
        self.x = x
        self.y = y
        self.z = z

    def reset(self) -> None:
        """Reset offset to zero (no offset applied)."""
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

    @property
    def is_active(self) -> bool:
        """Check if any offset is active (non-zero)."""
        return self.x != 0.0 or self.y != 0.0 or self.z != 0.0


# Global default configuration instance
# This can be imported and used directly throughout the codebase
DEFAULT_CONFIG = ConversionConfig()

# Global runtime coordinate offset
# Set this before conversion to apply coordinate offset to all outputs
COORDINATE_OFFSET = CoordinateOffset()

# Runtime overrides for geometry parameters
# These override DEFAULT_CONFIG values when set
_runtime_param_poly3_num_segments: int | None = None
_runtime_enable_adaptive_subdivision: bool | None = None
_runtime_max_heading_change_deg: float | None = None
_runtime_max_subdivision_iterations: int | None = None


def set_param_poly3_num_segments(num_segments: int) -> None:
    """Set runtime override for ParamPoly3 num_segments parameter.

    This function allows dynamic configuration of the number of ParamPoly3 segments
    used during conversion, typically from Hydra configuration or command-line arguments.

    Args:
        num_segments: Number of ParamPoly3 segments to use (must be positive)

    Raises:
        ValueError: If num_segments is not positive
    """
    global _runtime_param_poly3_num_segments
    if num_segments <= 0:
        raise ValueError(f"num_segments must be positive, got {num_segments}")
    _runtime_param_poly3_num_segments = num_segments


def get_param_poly3_num_segments() -> int:
    """Get the effective ParamPoly3 num_segments value.

    Returns runtime override if set, otherwise returns default from config.

    Returns:
        Number of ParamPoly3 segments to use
    """
    if _runtime_param_poly3_num_segments is not None:
        return _runtime_param_poly3_num_segments
    return DEFAULT_CONFIG.geometry.param_poly3_num_segments


def reset_param_poly3_num_segments() -> None:
    """Reset runtime override to use default config value."""
    global _runtime_param_poly3_num_segments
    _runtime_param_poly3_num_segments = None


def set_adaptive_subdivision_enabled(enabled: bool) -> None:
    """Set runtime override for adaptive subdivision enabled flag.

    Args:
        enabled: Whether to enable adaptive subdivision
    """
    global _runtime_enable_adaptive_subdivision
    _runtime_enable_adaptive_subdivision = enabled


def get_adaptive_subdivision_enabled() -> bool:
    """Get the effective adaptive subdivision enabled value.

    Returns runtime override if set, otherwise returns default from config.

    Returns:
        Whether adaptive subdivision is enabled
    """
    if _runtime_enable_adaptive_subdivision is not None:
        return _runtime_enable_adaptive_subdivision
    return DEFAULT_CONFIG.geometry.enable_adaptive_subdivision


def set_max_heading_change_deg(degrees: float) -> None:
    """Set runtime override for max heading change threshold.

    Args:
        degrees: Maximum heading change in degrees (must be positive)

    Raises:
        ValueError: If degrees is not positive
    """
    global _runtime_max_heading_change_deg
    if degrees <= 0:
        raise ValueError(f"max_heading_change_deg must be positive, got {degrees}")
    _runtime_max_heading_change_deg = degrees


def get_max_heading_change_deg() -> float:
    """Get the effective max heading change threshold.

    Returns runtime override if set, otherwise returns default from config.

    Returns:
        Maximum heading change in degrees
    """
    if _runtime_max_heading_change_deg is not None:
        return _runtime_max_heading_change_deg
    return DEFAULT_CONFIG.geometry.max_heading_change_deg


def set_max_subdivision_iterations(iterations: int) -> None:
    """Set runtime override for max subdivision iterations.

    Args:
        iterations: Maximum number of subdivision iterations (must be positive)

    Raises:
        ValueError: If iterations is not positive
    """
    global _runtime_max_subdivision_iterations
    if iterations <= 0:
        raise ValueError(
            f"max_subdivision_iterations must be positive, got {iterations}"
        )
    _runtime_max_subdivision_iterations = iterations


def get_max_subdivision_iterations() -> int:
    """Get the effective max subdivision iterations value.

    Returns runtime override if set, otherwise returns default from config.

    Returns:
        Maximum number of subdivision iterations
    """
    if _runtime_max_subdivision_iterations is not None:
        return _runtime_max_subdivision_iterations
    return DEFAULT_CONFIG.geometry.max_subdivision_iterations


def reset_adaptive_subdivision_config() -> None:
    """Reset all adaptive subdivision runtime overrides to use default config values."""
    global _runtime_enable_adaptive_subdivision
    global _runtime_max_heading_change_deg
    global _runtime_max_subdivision_iterations
    _runtime_enable_adaptive_subdivision = None
    _runtime_max_heading_change_deg = None
    _runtime_max_subdivision_iterations = None
