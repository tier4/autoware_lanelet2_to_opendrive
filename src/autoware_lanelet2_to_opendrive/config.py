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
        preprocessing: Preprocessing operation constants
    """

    geometry: GeometryConstants = GeometryConstants()
    spline: SplineConstants = SplineConstants()
    preprocessing: PreprocessingConstants = PreprocessingConstants()


# Global default configuration instance
# This can be imported and used directly throughout the codebase
DEFAULT_CONFIG = ConversionConfig()
