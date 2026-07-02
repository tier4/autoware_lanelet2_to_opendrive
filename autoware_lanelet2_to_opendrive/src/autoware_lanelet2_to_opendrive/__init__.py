"""Autoware Lanelet2 to OpenDRIVE converter package."""

# Import autoware extensions at package load time to ensure regulatory elements are registered
# This must happen before any lanelet2 map loading
from autoware_lanelet2_extension_python.projection import MGRSProjector  # noqa: F401
import lanelet2  # noqa: F401

from .types import Point2D, Point3D, Point
from .util import (
    ConnectionDirection,
    find_lanelets_without_next,
    find_lanelets_without_previous,
    find_terminal_lanelets,
    find_adjacent_groups,
    split_groups_by_divergent_connections,
    filter_lanelets_by_subtype,
    find_connecting_lanelet_groups,
    RoadLaneletMapping,
)
from .main import convert_lanelet2_to_opendrive
from .preprocess_lanelet import (
    LatLonOrigin,
    PreprocessOperation,
    LaneletPreprocessor,
)
from .map_resolver import resolve_map_to_xodr
from .road_lanelet_geo_mapping import (
    GeoRoadLaneletMapping,
    MappingMismatchError,
    validate_mapping_consistency,
    validate_and_save_mapping,
)

__all__ = [
    "Point2D",
    "Point3D",
    "Point",
    "ConnectionDirection",
    "find_lanelets_without_next",
    "find_lanelets_without_previous",
    "find_terminal_lanelets",
    "find_adjacent_groups",
    "split_groups_by_divergent_connections",
    "filter_lanelets_by_subtype",
    "find_connecting_lanelet_groups",
    "RoadLaneletMapping",
    "convert_lanelet2_to_opendrive",
    "LatLonOrigin",
    "PreprocessOperation",
    "LaneletPreprocessor",
    "resolve_map_to_xodr",
    "GeoRoadLaneletMapping",
    "MappingMismatchError",
    "validate_mapping_consistency",
    "validate_and_save_mapping",
]
