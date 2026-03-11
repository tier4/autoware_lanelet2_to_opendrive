"""Re-export mapping types from autoware_lanelet2_to_opendrive.

The mapping logic (boundary shape comparison, SHA256 caching) lives in the
converter package.  This module re-exports the public API under local names
for use within autoware_carla_scenario.
"""

from autoware_lanelet2_to_opendrive.road_lanelet_geo_mapping import (
    GeoRoadLaneletMapping as RoadLaneletMapping,
    build_mapping,
    load_or_build_mapping,
)

__all__ = [
    "RoadLaneletMapping",
    "build_mapping",
    "load_or_build_mapping",
]
