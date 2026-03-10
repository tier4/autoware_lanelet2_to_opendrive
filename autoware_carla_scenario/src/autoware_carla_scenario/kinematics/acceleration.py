"""Acceleration types with coordinate-frame safety and affine-space semantics.

Mirrors the velocity type hierarchy:

:class:`AbsoluteAcceleration`
    Acceleration of an entity measured in a specific coordinate frame.

:class:`RelativeAcceleration`
    Difference in acceleration between two entities.

:class:`FrenetAcceleration`
    Acceleration decomposed into longitudinal / lateral components along a road.

Arithmetic rules follow the same affine-space model as the velocity types.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING, Union, overload

from .frames import CoordinateFrame, FrameMismatchError
from .vector import Vector3

if TYPE_CHECKING:
    import carla  # noqa: F401


# ---------------------------------------------------------------------------
# Absolute acceleration
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class AbsoluteAcceleration:
    """Acceleration of an entity in a specific coordinate frame (m/s^2)."""

    vector: Vector3
    frame: CoordinateFrame

    # -- arithmetic (affine-space rules) ------------------------------------

    def __add__(self, other: object) -> AbsoluteAcceleration:
        """``AbsoluteAcceleration + RelativeAcceleration -> AbsoluteAcceleration``."""
        if not isinstance(other, RelativeAcceleration):
            return NotImplemented
        if self.frame != other.frame:
            raise FrameMismatchError(self.frame, other.frame, "add")
        return AbsoluteAcceleration(self.vector + other.vector, self.frame)

    @overload
    def __sub__(self, other: AbsoluteAcceleration) -> RelativeAcceleration: ...

    @overload
    def __sub__(self, other: RelativeAcceleration) -> AbsoluteAcceleration: ...

    def __sub__(
        self,
        other: object,
    ) -> Union[RelativeAcceleration, AbsoluteAcceleration]:
        """``Abs - Abs -> Relative``  or  ``Abs - Relative -> Abs``."""
        if isinstance(other, AbsoluteAcceleration):
            if self.frame != other.frame:
                raise FrameMismatchError(self.frame, other.frame, "subtract")
            return RelativeAcceleration(self.vector - other.vector, self.frame)
        if isinstance(other, RelativeAcceleration):
            if self.frame != other.frame:
                raise FrameMismatchError(self.frame, other.frame, "subtract")
            return AbsoluteAcceleration(self.vector - other.vector, self.frame)
        return NotImplemented

    def __mul__(self, scalar: object) -> AbsoluteAcceleration:
        if not isinstance(scalar, (int, float)):
            return NotImplemented
        return AbsoluteAcceleration(self.vector * scalar, self.frame)

    def __rmul__(self, scalar: object) -> AbsoluteAcceleration:
        return self.__mul__(scalar)

    def __truediv__(self, scalar: object) -> AbsoluteAcceleration:
        if not isinstance(scalar, (int, float)):
            return NotImplemented
        return AbsoluteAcceleration(self.vector / scalar, self.frame)

    def __neg__(self) -> AbsoluteAcceleration:
        return AbsoluteAcceleration(-self.vector, self.frame)

    # -- queries ------------------------------------------------------------

    def magnitude(self) -> float:
        """Scalar acceleration magnitude in m/s^2."""
        return self.vector.magnitude()

    # -- factory methods ----------------------------------------------------

    @classmethod
    def zero(cls, frame: CoordinateFrame) -> AbsoluteAcceleration:
        """Zero acceleration in *frame*."""
        return cls(Vector3.zero(), frame)

    @classmethod
    def from_components(
        cls,
        ax: float,
        ay: float,
        az: float,
        frame: CoordinateFrame,
    ) -> AbsoluteAcceleration:
        """Create from individual x/y/z components (m/s^2)."""
        return cls(Vector3(ax, ay, az), frame)

    @classmethod
    def from_carla_vector3d(
        cls,
        v: carla.Vector3D,
    ) -> AbsoluteAcceleration:
        """Create from ``carla.Vector3D`` (always :attr:`CoordinateFrame.CARLA_WORLD`)."""
        return cls(Vector3.from_carla_vector3d(v), CoordinateFrame.CARLA_WORLD)


# ---------------------------------------------------------------------------
# Relative acceleration
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class RelativeAcceleration:
    """Acceleration difference between two entities in a coordinate frame (m/s^2).

    Defined as ``a_target - a_reference``.
    """

    vector: Vector3
    frame: CoordinateFrame

    # -- arithmetic ---------------------------------------------------------

    @overload
    def __add__(self, other: RelativeAcceleration) -> RelativeAcceleration: ...

    @overload
    def __add__(self, other: AbsoluteAcceleration) -> AbsoluteAcceleration: ...

    def __add__(
        self,
        other: object,
    ) -> Union[RelativeAcceleration, AbsoluteAcceleration]:
        """``Rel + Rel -> Rel``  or  ``Rel + Abs -> Abs``."""
        if isinstance(other, RelativeAcceleration):
            if self.frame != other.frame:
                raise FrameMismatchError(self.frame, other.frame, "add")
            return RelativeAcceleration(self.vector + other.vector, self.frame)
        if isinstance(other, AbsoluteAcceleration):
            if self.frame != other.frame:
                raise FrameMismatchError(self.frame, other.frame, "add")
            return AbsoluteAcceleration(self.vector + other.vector, self.frame)
        return NotImplemented

    def __sub__(self, other: object) -> RelativeAcceleration:
        """``Rel - Rel -> Rel``."""
        if not isinstance(other, RelativeAcceleration):
            return NotImplemented
        if self.frame != other.frame:
            raise FrameMismatchError(self.frame, other.frame, "subtract")
        return RelativeAcceleration(self.vector - other.vector, self.frame)

    def __mul__(self, scalar: object) -> RelativeAcceleration:
        if not isinstance(scalar, (int, float)):
            return NotImplemented
        return RelativeAcceleration(self.vector * scalar, self.frame)

    def __rmul__(self, scalar: object) -> RelativeAcceleration:
        return self.__mul__(scalar)

    def __truediv__(self, scalar: object) -> RelativeAcceleration:
        if not isinstance(scalar, (int, float)):
            return NotImplemented
        return RelativeAcceleration(self.vector / scalar, self.frame)

    def __neg__(self) -> RelativeAcceleration:
        """Negate (swaps target and reference)."""
        return RelativeAcceleration(-self.vector, self.frame)

    # -- queries ------------------------------------------------------------

    def magnitude(self) -> float:
        """Scalar acceleration magnitude in m/s^2."""
        return self.vector.magnitude()

    # -- factory methods ----------------------------------------------------

    @classmethod
    def zero(cls, frame: CoordinateFrame) -> RelativeAcceleration:
        """Zero relative acceleration in *frame*."""
        return cls(Vector3.zero(), frame)

    @classmethod
    def from_components(
        cls,
        ax: float,
        ay: float,
        az: float,
        frame: CoordinateFrame,
    ) -> RelativeAcceleration:
        """Create from individual x/y/z components (m/s^2)."""
        return cls(Vector3(ax, ay, az), frame)

    @classmethod
    def between(
        cls,
        target: AbsoluteAcceleration,
        reference: AbsoluteAcceleration,
    ) -> RelativeAcceleration:
        """Compute ``a_target - a_reference``.

        Raises:
            FrameMismatchError: If *target* and *reference* are in different frames.
        """
        if target.frame != reference.frame:
            raise FrameMismatchError(
                target.frame, reference.frame, "compute relative acceleration"
            )
        return cls(target.vector - reference.vector, target.frame)


# ---------------------------------------------------------------------------
# Frenet acceleration (road-relative)
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class FrenetAcceleration:
    """Acceleration decomposed in a road-relative Frenet frame (m/s^2).

    Attributes:
        longitudinal: Acceleration along the road direction (d^2s/dt^2).
        lateral: Acceleration perpendicular to the road (d^2t/dt^2, positive = left).
    """

    longitudinal: float
    lateral: float

    # -- arithmetic ---------------------------------------------------------

    def __add__(self, other: object) -> FrenetAcceleration:
        if not isinstance(other, FrenetAcceleration):
            return NotImplemented
        return FrenetAcceleration(
            self.longitudinal + other.longitudinal,
            self.lateral + other.lateral,
        )

    def __sub__(self, other: object) -> FrenetAcceleration:
        if not isinstance(other, FrenetAcceleration):
            return NotImplemented
        return FrenetAcceleration(
            self.longitudinal - other.longitudinal,
            self.lateral - other.lateral,
        )

    def __mul__(self, scalar: object) -> FrenetAcceleration:
        if not isinstance(scalar, (int, float)):
            return NotImplemented
        return FrenetAcceleration(
            self.longitudinal * scalar,
            self.lateral * scalar,
        )

    def __rmul__(self, scalar: object) -> FrenetAcceleration:
        return self.__mul__(scalar)

    def __truediv__(self, scalar: object) -> FrenetAcceleration:
        if not isinstance(scalar, (int, float)):
            return NotImplemented
        if scalar == 0:
            raise ZeroDivisionError("Cannot divide acceleration by zero")
        return FrenetAcceleration(
            self.longitudinal / scalar,
            self.lateral / scalar,
        )

    def __neg__(self) -> FrenetAcceleration:
        return FrenetAcceleration(-self.longitudinal, -self.lateral)

    # -- queries ------------------------------------------------------------

    def magnitude(self) -> float:
        """Scalar acceleration magnitude in m/s^2."""
        return math.sqrt(self.longitudinal**2 + self.lateral**2)

    @classmethod
    def zero(cls) -> FrenetAcceleration:
        """Zero Frenet acceleration."""
        return cls(0.0, 0.0)
