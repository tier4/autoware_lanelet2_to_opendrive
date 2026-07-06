"""Typed values extracted from OpenSCENARIO DSL expressions.

The OpenSCENARIO DSL supports literals (integers, floats, strings, booleans),
physical literals with units (``30kmph``, ``5s``), and symbolic references
(enum members, identifiers).  The classes here provide a small, immutable value
model with unit-aware conversion helpers so the translator can consume argument
values without re-implementing unit maths.
"""

from __future__ import annotations

import re
from dataclasses import dataclass

from .errors import OscTranslationError

# Conversion factors to a canonical unit.
_SPEED_TO_KMH: dict[str, float] = {
    "kmph": 1.0,
    "kph": 1.0,
    "km/h": 1.0,
    "mph": 1.609344,
    "mps": 3.6,
    "m/s": 3.6,
}

_TIME_TO_SECONDS: dict[str, float] = {
    "s": 1.0,
    "sec": 1.0,
    "secs": 1.0,
    "ms": 0.001,
    "min": 60.0,
    "h": 3600.0,
}

_LENGTH_TO_M: dict[str, float] = {
    "m": 1.0,
    "km": 1000.0,
    "cm": 0.01,
    "mm": 0.001,
}

_PHYSICAL_RE = re.compile(r"^([+-]?\d*\.?\d+)\s*([A-Za-z%/]+)$")


class OscValue:
    """Base class for values extracted from a DSL expression."""

    def as_float(self) -> float:
        raise OscTranslationError(f"{self!r} is not a numeric value")

    def as_int(self) -> int:
        raise OscTranslationError(f"{self!r} is not an integer value")

    def as_str(self) -> str:
        raise OscTranslationError(f"{self!r} is not a string value")

    def as_bool(self) -> bool:
        raise OscTranslationError(f"{self!r} is not a boolean value")

    def as_symbol(self) -> str:
        raise OscTranslationError(f"{self!r} is not a symbolic value")


@dataclass(frozen=True)
class IntValue(OscValue):
    """An integer literal (e.g. ``242``)."""

    value: int

    def as_float(self) -> float:
        return float(self.value)

    def as_int(self) -> int:
        return self.value


@dataclass(frozen=True)
class FloatValue(OscValue):
    """A floating-point literal (e.g. ``4.0``)."""

    value: float

    def as_float(self) -> float:
        return self.value

    def as_int(self) -> int:
        return int(self.value)


@dataclass(frozen=True)
class StringValue(OscValue):
    """A string literal with the surrounding quotes removed."""

    value: str

    def as_str(self) -> str:
        return self.value


@dataclass(frozen=True)
class BoolValue(OscValue):
    """A boolean literal (``true`` / ``false``)."""

    value: bool

    def as_bool(self) -> bool:
        return self.value


@dataclass(frozen=True)
class SymbolValue(OscValue):
    """A symbolic reference: an enum member or identifier (e.g. ``left``)."""

    name: str

    def as_str(self) -> str:
        return self.name

    def as_symbol(self) -> str:
        return self.name

    def as_bool(self) -> bool:
        if self.name in ("true", "false"):
            return self.name == "true"
        return super().as_bool()


@dataclass(frozen=True)
class PhysicalValue(OscValue):
    """A physical literal: a magnitude paired with a unit (e.g. ``30kmph``)."""

    magnitude: float
    unit: str

    def as_float(self) -> float:
        return self.magnitude

    def to_kmh(self) -> float:
        """Return the value converted to kilometres per hour."""
        factor = _SPEED_TO_KMH.get(self.unit.lower())
        if factor is None:
            raise OscTranslationError(
                f"unit {self.unit!r} is not a known speed unit "
                f"(expected one of {sorted(_SPEED_TO_KMH)})"
            )
        return self.magnitude * factor

    def to_seconds(self) -> float:
        """Return the value converted to seconds."""
        factor = _TIME_TO_SECONDS.get(self.unit.lower())
        if factor is None:
            raise OscTranslationError(
                f"unit {self.unit!r} is not a known time unit "
                f"(expected one of {sorted(_TIME_TO_SECONDS)})"
            )
        return self.magnitude * factor

    def to_meters(self) -> float:
        """Return the value converted to metres."""
        factor = _LENGTH_TO_M.get(self.unit.lower())
        if factor is None:
            raise OscTranslationError(
                f"unit {self.unit!r} is not a known length unit "
                f"(expected one of {sorted(_LENGTH_TO_M)})"
            )
        return self.magnitude * factor


def parse_physical_literal(text: str) -> PhysicalValue:
    """Split a physical-literal token (``"30kmph"``) into magnitude and unit.

    Args:
        text: The raw token text, optionally containing internal whitespace.

    Returns:
        A :class:`PhysicalValue`.

    Raises:
        OscTranslationError: If *text* is not a recognisable physical literal.
    """
    match = _PHYSICAL_RE.match(text.strip())
    if match is None:
        raise OscTranslationError(f"cannot parse physical literal {text!r}")
    magnitude = float(match.group(1))
    return PhysicalValue(magnitude=magnitude, unit=match.group(2))
