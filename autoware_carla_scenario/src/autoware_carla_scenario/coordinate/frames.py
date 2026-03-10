"""Coordinate frame definitions shared across coordinate and kinematics modules.

Three coordinate systems are supported:

- **CARLA_WORLD** : Left-handed UE5 world coordinates (x=East, y=South, z=Up).
- **LANELET2** : Right-handed map coordinates (x=East, y=North, z=Up).
- **OPENDRIVE** : Right-handed map coordinates relative to geoReference origin.
"""

from __future__ import annotations

from enum import Enum, auto
from typing import TYPE_CHECKING, Union

if TYPE_CHECKING:
    from .poses import AnyPose


class CoordinateFrame(Enum):
    """Identifier for a coordinate reference frame."""

    CARLA_WORLD = auto()
    LANELET2 = auto()
    OPENDRIVE = auto()


class FrameMismatchError(ValueError):
    """Raised when an operation mixes values from different coordinate frames."""

    def __init__(
        self,
        frame_a: CoordinateFrame,
        frame_b: CoordinateFrame,
        operation: str,
    ) -> None:
        super().__init__(
            f"Cannot {operation} values in different coordinate frames: "
            f"{frame_a.name} and {frame_b.name}"
        )
        self.frame_a = frame_a
        self.frame_b = frame_b


def frame_of(pose: Union[AnyPose, object]) -> CoordinateFrame:
    """Return the :class:`CoordinateFrame` corresponding to a pose type.

    Accepts any pose that carries a ``FRAME`` class variable
    (:class:`~.poses.Lanelet2Pose`, :class:`~.poses.OpenDrivePose`,
    :class:`~.poses.CarlaWorldPose`).

    Raises:
        TypeError: If *pose* does not have a ``FRAME`` attribute.
    """
    frame = getattr(pose, "FRAME", None)
    if isinstance(frame, CoordinateFrame):
        return frame
    raise TypeError(
        f"Cannot determine coordinate frame for {type(pose).__name__}; "
        f"expected a pose type with a FRAME class variable."
    )
