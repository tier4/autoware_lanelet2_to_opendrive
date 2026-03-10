"""Velocity types with coordinate-frame safety and affine-space semantics.

Three velocity types are provided:

:class:`AbsoluteVelocity`
    Velocity of an entity measured in a specific coordinate frame.

:class:`RelativeVelocity`
    Difference in velocity between two entities (or an entity and a frame).

:class:`FrenetVelocity`
    Velocity decomposed into longitudinal / lateral components along a road.

Arithmetic rules (affine-space model)
--------------------------------------

.. code-block:: text

    AbsoluteVelocity  -  AbsoluteVelocity  =  RelativeVelocity
    AbsoluteVelocity  +  RelativeVelocity   =  AbsoluteVelocity
    AbsoluteVelocity  -  RelativeVelocity   =  AbsoluteVelocity
    RelativeVelocity  +  RelativeVelocity   =  RelativeVelocity
    RelativeVelocity  +  AbsoluteVelocity   =  AbsoluteVelocity
    AbsoluteVelocity  *  scalar             =  AbsoluteVelocity
    RelativeVelocity  *  scalar             =  RelativeVelocity

Operations between values in **different** coordinate frames raise
:class:`~.frames.FrameMismatchError`.
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
# Absolute velocity
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class AbsoluteVelocity:
    """Velocity of an entity in a specific coordinate frame (m/s)."""

    vector: Vector3
    frame: CoordinateFrame

    # -- arithmetic (affine-space rules) ------------------------------------

    def __add__(self, other: object) -> AbsoluteVelocity:
        """``AbsoluteVelocity + RelativeVelocity -> AbsoluteVelocity``."""
        if not isinstance(other, RelativeVelocity):
            return NotImplemented
        if self.frame != other.frame:
            raise FrameMismatchError(self.frame, other.frame, "add")
        return AbsoluteVelocity(self.vector + other.vector, self.frame)

    @overload
    def __sub__(self, other: AbsoluteVelocity) -> RelativeVelocity: ...

    @overload
    def __sub__(self, other: RelativeVelocity) -> AbsoluteVelocity: ...

    def __sub__(
        self,
        other: object,
    ) -> Union[RelativeVelocity, AbsoluteVelocity]:
        """``Abs - Abs -> Relative``  or  ``Abs - Relative -> Abs``."""
        if isinstance(other, AbsoluteVelocity):
            if self.frame != other.frame:
                raise FrameMismatchError(self.frame, other.frame, "subtract")
            return RelativeVelocity(self.vector - other.vector, self.frame)
        if isinstance(other, RelativeVelocity):
            if self.frame != other.frame:
                raise FrameMismatchError(self.frame, other.frame, "subtract")
            return AbsoluteVelocity(self.vector - other.vector, self.frame)
        return NotImplemented

    def __mul__(self, scalar: object) -> AbsoluteVelocity:
        if not isinstance(scalar, (int, float)):
            return NotImplemented
        return AbsoluteVelocity(self.vector * scalar, self.frame)

    def __rmul__(self, scalar: object) -> AbsoluteVelocity:
        return self.__mul__(scalar)

    def __truediv__(self, scalar: object) -> AbsoluteVelocity:
        if not isinstance(scalar, (int, float)):
            return NotImplemented
        return AbsoluteVelocity(self.vector / scalar, self.frame)

    def __neg__(self) -> AbsoluteVelocity:
        return AbsoluteVelocity(-self.vector, self.frame)

    # -- queries ------------------------------------------------------------

    def speed(self) -> float:
        """Scalar speed (magnitude of velocity vector) in m/s."""
        return self.vector.magnitude()

    # -- factory methods ----------------------------------------------------

    @classmethod
    def zero(cls, frame: CoordinateFrame) -> AbsoluteVelocity:
        """Zero velocity in *frame*."""
        return cls(Vector3.zero(), frame)

    @classmethod
    def from_components(
        cls,
        vx: float,
        vy: float,
        vz: float,
        frame: CoordinateFrame,
    ) -> AbsoluteVelocity:
        """Create from individual x/y/z components (m/s)."""
        return cls(Vector3(vx, vy, vz), frame)

    @classmethod
    def from_carla_vector3d(
        cls,
        v: carla.Vector3D,
    ) -> AbsoluteVelocity:
        """Create from ``carla.Vector3D`` (always :attr:`CoordinateFrame.CARLA_WORLD`)."""
        return cls(Vector3.from_carla_vector3d(v), CoordinateFrame.CARLA_WORLD)


# ---------------------------------------------------------------------------
# Relative velocity
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class RelativeVelocity:
    """Velocity difference between two entities in a coordinate frame (m/s).

    Defined as ``v_target - v_reference``.
    """

    vector: Vector3
    frame: CoordinateFrame

    # -- arithmetic ---------------------------------------------------------

    @overload
    def __add__(self, other: RelativeVelocity) -> RelativeVelocity: ...

    @overload
    def __add__(self, other: AbsoluteVelocity) -> AbsoluteVelocity: ...

    def __add__(
        self,
        other: object,
    ) -> Union[RelativeVelocity, AbsoluteVelocity]:
        """``Rel + Rel -> Rel``  or  ``Rel + Abs -> Abs``."""
        if isinstance(other, RelativeVelocity):
            if self.frame != other.frame:
                raise FrameMismatchError(self.frame, other.frame, "add")
            return RelativeVelocity(self.vector + other.vector, self.frame)
        if isinstance(other, AbsoluteVelocity):
            if self.frame != other.frame:
                raise FrameMismatchError(self.frame, other.frame, "add")
            return AbsoluteVelocity(self.vector + other.vector, self.frame)
        return NotImplemented

    def __sub__(self, other: object) -> RelativeVelocity:
        """``Rel - Rel -> Rel``."""
        if not isinstance(other, RelativeVelocity):
            return NotImplemented
        if self.frame != other.frame:
            raise FrameMismatchError(self.frame, other.frame, "subtract")
        return RelativeVelocity(self.vector - other.vector, self.frame)

    def __mul__(self, scalar: object) -> RelativeVelocity:
        if not isinstance(scalar, (int, float)):
            return NotImplemented
        return RelativeVelocity(self.vector * scalar, self.frame)

    def __rmul__(self, scalar: object) -> RelativeVelocity:
        return self.__mul__(scalar)

    def __truediv__(self, scalar: object) -> RelativeVelocity:
        if not isinstance(scalar, (int, float)):
            return NotImplemented
        return RelativeVelocity(self.vector / scalar, self.frame)

    def __neg__(self) -> RelativeVelocity:
        """Negate (swaps target and reference)."""
        return RelativeVelocity(-self.vector, self.frame)

    # -- queries ------------------------------------------------------------

    def speed(self) -> float:
        """Scalar relative speed (magnitude) in m/s."""
        return self.vector.magnitude()

    # -- factory methods ----------------------------------------------------

    @classmethod
    def zero(cls, frame: CoordinateFrame) -> RelativeVelocity:
        """Zero relative velocity in *frame*."""
        return cls(Vector3.zero(), frame)

    @classmethod
    def from_components(
        cls,
        vx: float,
        vy: float,
        vz: float,
        frame: CoordinateFrame,
    ) -> RelativeVelocity:
        """Create from individual x/y/z components (m/s)."""
        return cls(Vector3(vx, vy, vz), frame)

    @classmethod
    def between(
        cls,
        target: AbsoluteVelocity,
        reference: AbsoluteVelocity,
    ) -> RelativeVelocity:
        """Compute ``v_target - v_reference``.

        Raises:
            FrameMismatchError: If *target* and *reference* are in different frames.
        """
        if target.frame != reference.frame:
            raise FrameMismatchError(
                target.frame, reference.frame, "compute relative velocity"
            )
        return cls(target.vector - reference.vector, target.frame)


# ---------------------------------------------------------------------------
# Frenet velocity (road-relative)
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class FrenetVelocity:
    """Velocity decomposed in a road-relative Frenet frame (m/s).

    Attributes:
        longitudinal: Speed along the road direction (ds/dt).
        lateral: Speed perpendicular to the road (dt/dt, positive = left).
    """

    longitudinal: float
    lateral: float

    # -- arithmetic ---------------------------------------------------------

    def __add__(self, other: object) -> FrenetVelocity:
        if not isinstance(other, FrenetVelocity):
            return NotImplemented
        return FrenetVelocity(
            self.longitudinal + other.longitudinal,
            self.lateral + other.lateral,
        )

    def __sub__(self, other: object) -> FrenetVelocity:
        if not isinstance(other, FrenetVelocity):
            return NotImplemented
        return FrenetVelocity(
            self.longitudinal - other.longitudinal,
            self.lateral - other.lateral,
        )

    def __mul__(self, scalar: object) -> FrenetVelocity:
        if not isinstance(scalar, (int, float)):
            return NotImplemented
        return FrenetVelocity(
            self.longitudinal * scalar,
            self.lateral * scalar,
        )

    def __rmul__(self, scalar: object) -> FrenetVelocity:
        return self.__mul__(scalar)

    def __truediv__(self, scalar: object) -> FrenetVelocity:
        if not isinstance(scalar, (int, float)):
            return NotImplemented
        if scalar == 0:
            raise ZeroDivisionError("Cannot divide velocity by zero")
        return FrenetVelocity(
            self.longitudinal / scalar,
            self.lateral / scalar,
        )

    def __neg__(self) -> FrenetVelocity:
        return FrenetVelocity(-self.longitudinal, -self.lateral)

    # -- queries ------------------------------------------------------------

    def speed(self) -> float:
        """Scalar speed (magnitude) in m/s."""
        return math.sqrt(self.longitudinal**2 + self.lateral**2)

    @classmethod
    def zero(cls) -> FrenetVelocity:
        """Zero Frenet velocity."""
        return cls(0.0, 0.0)
