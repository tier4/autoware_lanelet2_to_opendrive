"""Coordinate frame definitions for kinematic quantities.

This module re-exports :class:`CoordinateFrame`, :class:`FrameMismatchError`,
and :func:`frame_of` from :mod:`autoware_carla_scenario.coordinate.frames`,
which is the canonical definition site.
"""

from autoware_carla_scenario.coordinate.frames import (
    CoordinateFrame,
    FrameMismatchError,
    frame_of,
)

__all__ = [
    "CoordinateFrame",
    "FrameMismatchError",
    "frame_of",
]
