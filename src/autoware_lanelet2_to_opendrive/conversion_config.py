"""Configuration dataclasses for conversion operations.

This module provides dataclass-based configuration objects to replace functions
with many parameters, improving type safety and API clarity.
"""

from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any, Dict, Optional, Set, Tuple
from lanelet2.routing import RoutingGraph


class WidthReference(Enum):
    """Reference line for width calculation."""

    CENTER_LINE = "center_line"
    LEFT_BOUND = "left_bound"
    RIGHT_BOUND = "right_bound"


@dataclass
class OriginSpec:
    """Coordinate origin specification for map conversion.

    Supports multiple origin specification methods:
    - MGRS grid code (mgrs_code)
    - Latitude/Longitude coordinates (lat/lon)
    """

    mgrs_code: Optional[str] = None
    lat: Optional[float] = None
    lon: Optional[float] = None


@dataclass
class ParamPoly3Config:
    """Configuration for ParamPoly3 segment generation.

    Controls how B-splines are converted to ParamPoly3 segments for OpenDRIVE
    output. These settings prevent issues like zero-length segments that cause
    CARLA crashes.

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
        enabled: Enable dynamic segment calculation (True by default)
                If False, uses fixed segment count (legacy behavior)
    """

    min_segment_length: float = 0.5
    default_segment_length: float = 1.0
    max_segments: int = 100
    min_segments: int = 1
    coefficient_epsilon: float = 1e-8
    enabled: bool = True


@dataclass
class WidthEstimationConfig:
    """Configuration for lanelet width estimation.

    This dataclass specifies how to calculate lane width as a function of
    arc length along a reference line.

    Attributes:
        num_samples: Number of points to sample along the lanelet for width
            estimation (default: 20). Ignored if adaptive_sampling is enabled.
        num_control_points: Number of control points for width spline
            interpolation (currently unused, kept for compatibility)
        reference: Reference line to use for width measurement
        adaptive_sampling: Enable adaptive sampling based on road length (default: False)
        min_samples: Minimum number of samples per road (default: 5)
        max_samples: Maximum number of samples per road (default: 50)
        default_sample_interval: Target interval between samples in meters (default: 5.0m)
            Used when adaptive_sampling is True
    """

    num_samples: int = 20
    num_control_points: int = 10
    reference: WidthReference = WidthReference.CENTER_LINE
    adaptive_sampling: bool = False
    min_samples: int = 5
    max_samples: int = 50
    default_sample_interval: float = 5.0


@dataclass
class GeometrySimplificationConfig:
    """Configuration for geometry simplification after ParamPoly3 generation.

    Simplifies OpenDRIVE geometry by converting trivial ParamPoly3 segments to
    simpler geometry types (Line/Arc) and consolidating short consecutive segments.

    Attributes:
        enabled: Enable geometry simplification (default: False for backward compatibility)
        convert_to_line: Enable conversion of straight ParamPoly3 to Line geometry
        convert_to_arc: Enable conversion of circular ParamPoly3 to Arc geometry
        consolidate_segments: Enable merging of short consecutive segments
        line_cu_threshold: Threshold for |cU| coefficient to consider as line
        line_du_threshold: Threshold for |dU| coefficient to consider as line
        line_cv_threshold: Threshold for |cV| coefficient to consider as line
        line_dv_threshold: Threshold for |dV| coefficient to consider as line
        arc_curvature_error_threshold: Maximum curvature error for Arc fitting
        arc_position_error_threshold: Maximum position error for Arc fitting (meters)
        min_segment_length: Minimum segment length for consolidation (meters)
        max_heading_diff_degrees: Maximum heading difference for merging (degrees)
    """

    enabled: bool = False  # Disabled by default for backward compatibility
    convert_to_line: bool = True
    convert_to_arc: bool = True
    consolidate_segments: bool = True

    # Line conversion thresholds
    line_cu_threshold: float = 0.001
    line_du_threshold: float = 0.0001
    line_cv_threshold: float = 0.001
    line_dv_threshold: float = 0.0001

    # Arc conversion thresholds
    arc_curvature_error_threshold: float = 0.01
    arc_position_error_threshold: float = 0.05

    # Consolidation thresholds
    min_segment_length: float = 5.0
    max_heading_diff_degrees: float = 5.0


@dataclass
class ConversionConfig:
    """Configuration for Lanelet2 to OpenDRIVE conversion.

    This dataclass groups all parameters needed for the main conversion
    function, improving clarity and extensibility.

    Attributes:
        output_path: Path where the OpenDRIVE file will be saved
        origin: Origin specification for coordinate system
        exclude_non_junction_signals: If True, exclude traffic signals not
            associated with junction lanelets (required for CARLA compatibility)
        junction_id_offset: Offset added to junction IDs to avoid conflicts
            with road IDs (default: 1000)
        traffic_rule: Traffic rule for lanes (RHT: Right-Hand Traffic,
            LHT: Left-Hand Traffic). Defaults to "RHT"
        parampoly3: Configuration for ParamPoly3 segment generation
        width_estimation: Configuration for width spline sampling
        geometry_simplification: Configuration for geometry simplification
    """

    output_path: Optional[Path] = None
    origin: OriginSpec = field(default_factory=OriginSpec)
    exclude_non_junction_signals: bool = False
    junction_id_offset: int = 1000
    traffic_rule: Optional[str] = "RHT"
    parampoly3: ParamPoly3Config = field(default_factory=ParamPoly3Config)
    width_estimation: WidthEstimationConfig = field(
        default_factory=WidthEstimationConfig
    )
    geometry_simplification: GeometrySimplificationConfig = field(
        default_factory=GeometrySimplificationConfig
    )

    def __post_init__(self):
        """Validate configuration after initialization."""
        if self.traffic_rule not in ("RHT", "LHT", None):
            raise ValueError(
                f"traffic_rule must be 'RHT' or 'LHT', got '{self.traffic_rule}'"
            )


@dataclass
class LaneLinksContext:
    """Context for setting up lane predecessor/successor links.

    This dataclass groups all the context needed to establish lane connectivity,
    making the API clearer and more maintainable.

    Attributes:
        lanelet_map: The Lanelet2 map containing connectivity information
        lanelet_to_road_and_lane: Global mapping from lanelet_id to (road_id, lane_id)
        routing_graph: Optional pre-built routing graph for connectivity analysis
        road_lane_ids: Optional mapping from road_id to set of existing lane_ids
            for validation
        road_id_to_road: Optional mapping from road_id to Road objects for
            checking if target roads are connecting roads
    """

    lanelet_map: Any  # lanelet2.core.LaneletMap
    lanelet_to_road_and_lane: Dict[int, Tuple[int, int]]
    routing_graph: Optional[RoutingGraph] = None
    road_lane_ids: Optional[Dict[int, Set[int]]] = None
    road_id_to_road: Optional[Dict[int, Any]] = None  # Dict[int, Road]
