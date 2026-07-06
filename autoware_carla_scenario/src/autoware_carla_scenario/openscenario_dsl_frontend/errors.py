"""Exception types raised by the OpenSCENARIO DSL frontend."""

from __future__ import annotations

from dataclasses import dataclass


class OscError(Exception):
    """Base class for all OpenSCENARIO DSL frontend errors."""


@dataclass(frozen=True)
class SyntaxErrorInfo:
    """A single syntax error reported by the ANTLR parser.

    Attributes:
        line: 1-based line number where the error was detected.
        column: 0-based column offset where the error was detected.
        message: Human-readable description produced by ANTLR.
    """

    line: int
    column: int
    message: str

    def __str__(self) -> str:
        return f"line {self.line}:{self.column} {self.message}"


class OscParseError(OscError):
    """Raised when py-osc2 fails to parse an OpenSCENARIO DSL source.

    Attributes:
        errors: The list of individual :class:`SyntaxErrorInfo` records
            collected from the ANTLR error listener.
    """

    def __init__(self, source_name: str, errors: list[SyntaxErrorInfo]) -> None:
        self.source_name = source_name
        self.errors = errors
        detail = "\n".join(f"  {e}" for e in errors)
        super().__init__(
            f"Failed to parse OpenSCENARIO DSL source {source_name!r} "
            f"({len(errors)} syntax error(s)):\n{detail}"
        )


class OscTranslationError(OscError):
    """Raised when a parsed construct cannot be mapped to the framework.

    This typically means the DSL uses a behavior or modifier name that the
    frontend does not know how to translate into an
    :mod:`autoware_carla_scenario` module call.
    """


class OscDependencyError(OscError):
    """Raised when the optional ``py-osc2`` dependency is not installed."""
