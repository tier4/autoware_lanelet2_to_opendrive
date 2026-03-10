"""kinematics – velocity and acceleration types with coordinate-frame safety.

Usage::

    from autoware_carla_scenario.kinematics import (
        # Coordinate frames
        CoordinateFrame, FrameMismatchError,
        # 3-D vector
        Vector3,
        # Velocity types
        AbsoluteVelocity, RelativeVelocity, FrenetVelocity,
        # Acceleration types
        AbsoluteAcceleration, RelativeAcceleration, FrenetAcceleration,
    )

    # Create velocities in a specific frame
    v1 = AbsoluteVelocity.from_components(10, 0, 0, CoordinateFrame.CARLA_WORLD)
    v2 = AbsoluteVelocity.from_components(5, 0, 0, CoordinateFrame.CARLA_WORLD)

    # Affine-space arithmetic
    rel = v1 - v2            # -> RelativeVelocity
    v3  = v2 + rel           # -> AbsoluteVelocity (== v1)
    rel2 = RelativeVelocity.between(v1, v2)  # same as rel

    # Scalar operations
    v_half = v1 / 2          # -> AbsoluteVelocity(5, 0, 0)
    v_double = v1 * 2        # -> AbsoluteVelocity(20, 0, 0)

    # Road-relative (Frenet)
    fv = FrenetVelocity(longitudinal=15.0, lateral=0.5)
    print(fv.speed())        # 15.008...
"""

from .acceleration import (
    AbsoluteAcceleration,
    FrenetAcceleration,
    RelativeAcceleration,
)
from .frames import CoordinateFrame, FrameMismatchError, frame_of
from .vector import Vector3
from .velocity import AbsoluteVelocity, FrenetVelocity, RelativeVelocity

__all__ = [
    # Frames
    "CoordinateFrame",
    "FrameMismatchError",
    "frame_of",
    # Vector
    "Vector3",
    # Velocity
    "AbsoluteVelocity",
    "RelativeVelocity",
    "FrenetVelocity",
    # Acceleration
    "AbsoluteAcceleration",
    "RelativeAcceleration",
    "FrenetAcceleration",
]
