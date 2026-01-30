"""Configuration dataclasses for conversion operations.

This module provides dataclass-based configuration objects to replace functions
with many parameters, improving type safety and API clarity.
"""

from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any, Dict, List, Optional, Set, Tuple
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
class ConversionConfig:
    """Configuration for Lanelet2 to OpenDRIVE conversion.

    This dataclass groups all parameters needed for the main conversion
    function, improving clarity and extensibility.

    Attributes:
        output_path: Path where the OpenDRIVE file will be saved
        origin: Origin specification for coordinate system
        exclude_non_junction_signals: If True, exclude traffic signals not
            associated with junction lanelets (required for CARLA compatibility)
        no_junction_lanelet_ids: List of lanelet IDs to exclude from junction
            detection, treating them as regular roads
        junction_id_offset: Offset added to junction IDs to avoid conflicts
            with road IDs (default: 1000)
        traffic_rule: Traffic rule for lanes (RHT: Right-Hand Traffic,
            LHT: Left-Hand Traffic). Defaults to "RHT"
    """

    output_path: Optional[Path] = None
    origin: OriginSpec = field(default_factory=OriginSpec)
    exclude_non_junction_signals: bool = False
    no_junction_lanelet_ids: List[int] = field(default_factory=list)
    junction_id_offset: int = 1000
    traffic_rule: Optional[str] = "RHT"

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


@dataclass
class WidthEstimationConfig:
    """Configuration for lanelet width estimation.

    This dataclass specifies how to calculate lane width as a function of
    arc length along a reference line.

    Attributes:
        num_samples: Number of points to sample along the lanelet for width
            estimation (default: 20)
        num_control_points: Number of control points for width spline
            interpolation (currently unused, kept for compatibility)
        reference: Reference line to use for width measurement
    """

    num_samples: int = 20
    num_control_points: int = 10
    reference: WidthReference = WidthReference.CENTER_LINE
