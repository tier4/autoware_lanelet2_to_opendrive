"""Configuration dataclasses for conversion operations.

This module provides dataclass-based configuration objects to replace functions
with many parameters, improving type safety and API clarity.
"""

from dataclasses import dataclass, field, replace
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

    Exactly one of the following combinations must be set:

    * ``mgrs_code`` only – the origin is the south-west corner of the MGRS
      grid square (e.g. ``"54SUE"``).  The geoReference PROJ string is
      derived from the grid square's lat/lon.
    * ``lat`` + ``lon`` – the origin is given directly as WGS84 decimal
      degrees.  Use this when an MGRS offset has been applied (the offset
      shifts the origin away from the grid square corner) or when a lat/lon
      origin is preferred.  The geoReference PROJ string uses these values
      directly.
    * ``mgrs_code`` + ``lat`` + ``lon`` – all three fields set. This occurs
      when an MGRS grid is specified together with an offset: the lat/lon
      hold the offset-adjusted position while ``mgrs_code`` retains the
      grid zone for reference. The geoReference uses ``lat``/``lon``.
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
class StopLineConfig:
    """Configuration for stop line object generation.

    Attributes:
        width: Painted width of the stop line in the v-direction (along road),
            in meters. Corresponds to the OpenDRIVE object ``width`` attribute.
            Default 0.1m represents a typical road marking thickness.
            For CARLA Stencil_STOP, this is the stencil width (recommend ~2.0m).
        carla_stop_line: If True, output stop lines in CARLA's Stencil_STOP format
            (<object type="-1" name="Stencil_STOP">) instead of standard OpenDRIVE
            format. zOffset is fixed at 0.0 and orientation is set to "-".
    """

    width: float = 0.1
    carla_stop_line: bool = False


@dataclass
class TrafficLightConfig:
    """Configuration for traffic light actor spawn offset.

    When spawning traffic light actors in CARLA, their position may need
    fine-tuning. These offsets are applied to the positionInertial
    calculation: the (offset_x, offset_y) values are rotated by the
    signal's hdg angle and then subtracted from the centroid position.

    Attributes:
        offset_x: Offset along the signal's facing direction (hdg) in meters.
            Positive values shift the signal forward (away from approaching
            traffic). Default 0.0 (no offset).
        offset_y: Offset perpendicular to the signal's facing direction in
            meters. Positive values shift the signal to the left when facing
            in the hdg direction. Default 0.0 (no offset).
        offset_z: Vertical offset in meters. Subtracted directly from the
            z coordinate. Default 0.0 (no offset).
    """

    offset_x: float = 0.0
    offset_y: float = 0.0
    offset_z: float = 0.0


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
        stopline: Configuration for stop line object generation.
            Set stopline.carla_stop_line=True to enable CARLA Stencil_STOP format
        traffic_light: Configuration for traffic light actor spawn offset.
            The offsets are rotated by hdg and subtracted from positionInertial
            coordinates to adjust signal placement in CARLA.
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
    stopline: StopLineConfig = field(default_factory=StopLineConfig)
    traffic_light: TrafficLightConfig = field(default_factory=TrafficLightConfig)

    def __post_init__(self):
        """Validate configuration after initialization."""
        if self.traffic_rule not in ("RHT", "LHT", None):
            raise ValueError(
                f"traffic_rule must be 'RHT' or 'LHT', got '{self.traffic_rule}'"
            )

    def with_mgrs_code(self, mgrs_code: str) -> "ConversionConfig":
        """Return a copy of this config with ``origin.mgrs_code`` set.

        Used to merge a legacy ``mgrs_code`` argument into the config so the
        converter always has a single, consistent source of truth.
        """
        return replace(self, origin=replace(self.origin, mgrs_code=mgrs_code))


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
