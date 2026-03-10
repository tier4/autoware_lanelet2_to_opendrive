"""Immutable 3-D vector with full arithmetic support.

:class:`Vector3` is the underlying storage for velocity and acceleration
types.  It is a frozen dataclass that implements the standard vector-space
operations: addition, subtraction, scalar multiplication / division, dot
product, cross product, magnitude, and normalisation.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import carla  # noqa: F401


@dataclass(frozen=True)
class Vector3:
    """Immutable 3-D Euclidean vector."""

    x: float
    y: float
    z: float

    # -- vector-space arithmetic ------------------------------------------------

    def __add__(self, other: object) -> Vector3:
        if not isinstance(other, Vector3):
            return NotImplemented
        return Vector3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other: object) -> Vector3:
        if not isinstance(other, Vector3):
            return NotImplemented
        return Vector3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, scalar: object) -> Vector3:
        if not isinstance(scalar, (int, float)):
            return NotImplemented
        return Vector3(self.x * scalar, self.y * scalar, self.z * scalar)

    def __rmul__(self, scalar: object) -> Vector3:
        return self.__mul__(scalar)

    def __truediv__(self, scalar: object) -> Vector3:
        if not isinstance(scalar, (int, float)):
            return NotImplemented
        if scalar == 0:
            raise ZeroDivisionError("Cannot divide vector by zero")
        return Vector3(self.x / scalar, self.y / scalar, self.z / scalar)

    def __neg__(self) -> Vector3:
        return Vector3(-self.x, -self.y, -self.z)

    # -- geometric operations ---------------------------------------------------

    def magnitude(self) -> float:
        """Euclidean length of the vector."""
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)

    def normalized(self) -> Vector3:
        """Unit vector in the same direction.

        Raises:
            ValueError: If the vector has zero magnitude.
        """
        mag = self.magnitude()
        if mag == 0:
            raise ValueError("Cannot normalize a zero vector")
        return self / mag

    def dot(self, other: Vector3) -> float:
        """Dot (inner) product."""
        return self.x * other.x + self.y * other.y + self.z * other.z

    def cross(self, other: Vector3) -> Vector3:
        """Cross product (right-hand rule)."""
        return Vector3(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )

    # -- factory methods --------------------------------------------------------

    @classmethod
    def zero(cls) -> Vector3:
        """Return the zero vector."""
        return cls(0.0, 0.0, 0.0)

    @classmethod
    def from_carla_vector3d(cls, v: carla.Vector3D) -> Vector3:
        """Create from a ``carla.Vector3D`` instance."""
        return cls(x=v.x, y=v.y, z=v.z)
