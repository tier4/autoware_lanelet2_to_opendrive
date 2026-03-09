"""Pose dataclasses for coordinate transformation between map systems.

Three coordinate systems:
- Lanelet2: Frenet coordinates along a lanelet centerline (right-hand, North=+y)
- OpenDRIVE: Arc-length + lateral offset along a road reference line (right-hand, North=+y)
- CARLA world: Left-handed UE5 world coordinates (South=+y, i.e. y = −North)
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Union

if TYPE_CHECKING:
    import carla  # noqa: F401  (type stubs only)


@dataclass
class Lanelet2Pose:
    """Frenet coordinates within a single Lanelet2 lanelet.

    The origin of (s, t) is the start of the lanelet centerline.
    Positive t is to the left of the driving direction.
    """

    lanelet_id: int
    s: float  # Arc length along centerline from lanelet start (m)
    t: float = 0.0  # Lateral offset from centerline (positive=left, m)
    heading: float = (
        0.0  # Additional heading offset from centerline direction (radians)
    )


@dataclass
class OpenDrivePose:
    """Position on an OpenDRIVE road in standard (s, t) coordinates.

    s is measured along the road reference line.
    t is the lateral offset from the reference line following the OpenDRIVE standard:
      positive t = left of the reference line direction
      negative t = right of the reference line direction
    lane_id is provided as context but is not used for position calculation.
    """

    road_id: str  # Road ID string as found in the OpenDRIVE XML
    lane_id: int  # Lane ID (negative=right, positive=left) — context only
    s: float  # Arc length along road reference line (m)
    t: float = 0.0  # Lateral offset from road reference line (m)
    heading: float = 0.0  # Additional heading from road direction (radians)


@dataclass
class CarlaWorldPose:
    """Position and orientation in CARLA's left-handed UE5 world coordinate system.

    CARLA uses a left-handed coordinate system where:
      x = East  (same as Lanelet2/OpenDRIVE)
      y = South (opposite of Lanelet2/OpenDRIVE where y = North)
      z = Up    (same)
    Rotations are in degrees (CARLA convention).
    yaw=0 points East; positive yaw rotates clockwise when viewed from above.
    """

    x: float
    y: float
    z: float
    roll: float = field(default=0.0)
    pitch: float = field(default=0.0)
    yaw: float = field(
        default=0.0
    )  # degrees, right-hand rule in CARLA (clockwise from East)

    def to_carla_transform(self) -> "carla.Transform":
        """Convert to a carla.Transform object."""
        import carla  # noqa: PLC0415

        return carla.Transform(
            carla.Location(x=self.x, y=self.y, z=self.z),
            carla.Rotation(roll=self.roll, pitch=self.pitch, yaw=self.yaw),
        )

    @classmethod
    def from_carla_transform(cls, t: "carla.Transform") -> "CarlaWorldPose":
        """Create a CarlaWorldPose from a carla.Transform object."""
        return cls(
            x=t.location.x,
            y=t.location.y,
            z=t.location.z,
            roll=t.rotation.roll,
            pitch=t.rotation.pitch,
            yaw=t.rotation.yaw,
        )


AnyPose = Union[Lanelet2Pose, OpenDrivePose, CarlaWorldPose]
"""Union of all supported pose types for coordinate transformations."""
