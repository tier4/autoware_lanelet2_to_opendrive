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
    """

    epsilon: float = 1e-10
    point_distance_threshold: float = 0.001


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
        min_control_points: Minimum number of control points for spline fitting
        max_control_points: Maximum number of control points for spline fitting
        control_points_ratio: Ratio of control points to input points (base calculation)
        curvature_threshold: Threshold for high curvature detection (radians)
        curvature_multiplier: Multiplier for control points in high curvature regions
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
    min_control_points: int = 5
    max_control_points: int = 50
    control_points_ratio: float = 0.4
    curvature_threshold: float = 0.2
    curvature_multiplier: float = 1.5


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
class ParamPoly3Constants:
    """Constants for ParamPoly3 geometry generation.

    Controls how B-splines are converted to ParamPoly3 segments for OpenDRIVE
    output, preventing issues like zero-length segments that cause CARLA crashes.

    Attributes:
        min_segment_length: Minimum allowed segment length in meters (0.5m default)
                           Segments shorter than this will be rejected.
                           CARLA requirement: segments must be >= 0.5m
        default_segment_length: Target segment length for dynamic calculation (1.0m)
                               Used when num_segments is not explicitly specified
        max_segments: Maximum number of segments per road (100 default)
                     Prevents excessive segmentation of very long roads
        min_segments: Minimum number of segments per road (1 default)
                     Ensures at least one segment is created
        coefficient_epsilon: Threshold for rounding small coefficients to zero (1e-8)
                            Prevents numerical instability in paramPoly3
    """

    min_segment_length: float = 0.5
    default_segment_length: float = 1.0
    max_segments: int = 100
    min_segments: int = 1
    coefficient_epsilon: float = 1e-8


@dataclass(frozen=True)
class InternalConfig:
    """Main configuration container for all internal constants.

    This dataclass provides centralized access to all constant configurations
    used throughout the conversion process. Create an instance to access
    different constant groups.

    Example:
        ```python
        from autoware_lanelet2_to_opendrive.config import InternalConfig

        config = InternalConfig()

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
    parampoly3: ParamPoly3Constants = ParamPoly3Constants()


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
DEFAULT_CONFIG = InternalConfig()

# Global runtime coordinate offset
# Set this before conversion to apply coordinate offset to all outputs
COORDINATE_OFFSET = CoordinateOffset()
