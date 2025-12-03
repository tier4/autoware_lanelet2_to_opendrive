"""Autoware Lanelet2 to OpenDRIVE converter package."""

# Import autoware extensions at package load time to ensure regulatory elements are registered
# This must happen before any lanelet2 map loading
from autoware_lanelet2_extension_python.projection import MGRSProjector  # noqa: F401
import lanelet2  # noqa: F401

from .util import (
    ConnectionDirection,
    find_lanelets_without_next,
    find_lanelets_without_previous,
    find_terminal_lanelets,
    find_adjacent_groups,
    filter_lanelets_by_subtype,
    find_connecting_lanelet_groups,
)

__all__ = [
    "ConnectionDirection",
    "find_lanelets_without_next",
    "find_lanelets_without_previous",
    "find_terminal_lanelets",
    "find_adjacent_groups",
    "filter_lanelets_by_subtype",
    "find_connecting_lanelet_groups",
]
