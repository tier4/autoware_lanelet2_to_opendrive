"""Normalised syntax-level model of a parsed OpenSCENARIO DSL source.

The ANTLR parse tree produced by ``py-osc2`` is deeply nested and awkward to
consume directly.  :mod:`.extractor` lifts it into the small, typed model
defined here, which the translator and code generator work against.

This layer is purely syntactic: it records *what was written* (fields,
behaviour invocations, modifiers, ``do`` composition) without attaching any
:mod:`autoware_carla_scenario` semantics.  The semantic mapping happens later,
in :mod:`.translator`.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional, Union

from .values import OscValue


@dataclass(frozen=True)
class OscArgument:
    """A single argument to a behaviour invocation or modifier application.

    Attributes:
        name: The keyword name for a named argument, or ``None`` when the
            argument is positional.
        value: The extracted argument value.
    """

    name: Optional[str]
    value: OscValue


@dataclass(frozen=True)
class OscModifier:
    """A ``modifier_application`` such as ``speed(30kmph)`` or ``timeout(5s)``.

    Attributes:
        name: The modifier name (e.g. ``"speed"``).
        actor: The actor the modifier is scoped to (``ego`` in
            ``ego.speed(...)``), or ``None`` for an unscoped modifier.
        arguments: The modifier's argument list.
    """

    name: str
    actor: Optional[str]
    arguments: list[OscArgument] = field(default_factory=list)


# ---------------------------------------------------------------------------
# Event conditions (used by ``wait`` directives and ``until`` clauses)
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class OscFieldAccess:
    """An attribute reference such as ``ego.speed`` (``<actor>.<attr>``)."""

    actor: str
    attr: str


@dataclass(frozen=True)
class OscComparison:
    """A comparison ``<field> <op> <literal>`` (e.g. ``ego.speed < 5kmph``).

    Attributes:
        lhs: The left-hand attribute reference.
        op: The relational operator (``"<"``, ``"<="``, ``">"``, ``">="``,
            ``"=="``, ``"!="``).
        rhs: The right-hand literal value.
    """

    lhs: OscFieldAccess
    op: str
    rhs: OscValue


@dataclass(frozen=True)
class OscElapsed:
    """An ``elapsed(<duration>)`` event condition."""

    duration: OscValue


#: The event-condition forms the frontend understands.
OscEventCond = Union[OscComparison, OscElapsed]


@dataclass(frozen=True)
class OscInvocation:
    """A ``behavior_invocation`` such as ``ego.drive() with: speed(30kmph)``.

    Attributes:
        behavior: The behaviour name (e.g. ``"drive"``).
        actor: The actor the behaviour is invoked on, or ``None``.
        arguments: The invocation's argument list.
        modifiers: Modifiers attached via a trailing ``with:`` block.
        until: An ``until <event_condition>`` clause in the ``with:`` block that
            defines when the behaviour completes, or ``None``.
        composition: The nearest enclosing composition operator
            (``"serial"`` / ``"parallel"`` / ``"one_of"``), or ``None`` when the
            invocation is the direct ``do`` member.
    """

    behavior: str
    actor: Optional[str]
    arguments: list[OscArgument] = field(default_factory=list)
    modifiers: list[OscModifier] = field(default_factory=list)
    until: Optional[OscEventCond] = None
    composition: Optional[str] = None


@dataclass(frozen=True)
class OscWait:
    """A ``wait <event_condition>`` directive — a step with no action."""

    condition: OscEventCond


@dataclass(frozen=True)
class OscComposition:
    """A ``composition`` block: ``serial:`` / ``parallel:`` / ``one_of:``.

    Attributes:
        operator: The composition operator keyword.
        members: The nested ``do`` members, in source order.
        label: The optional phase label (``get_ahead`` in
            ``get_ahead: parallel(...)``), or ``None``.
        duration: The optional ``duration:`` argument of a timed phase
            (``parallel(duration: 15s)``), or ``None``.
    """

    operator: str
    members: list[OscDoMember] = field(default_factory=list)
    label: Optional[str] = None
    duration: Optional[OscValue] = None


#: A leaf invocation, a ``wait`` step, or a nested composition block.
OscDoMember = Union[OscInvocation, OscWait, OscComposition]


@dataclass(frozen=True)
class OscField:
    """A scenario/struct ``field`` (``parameter_declaration``).

    Attributes:
        name: The field name.
        type_name: The declared type (e.g. ``"vehicle"``, ``"float"``), or
            ``None`` when the type could not be determined.
        default: The default value, or ``None`` when no default is given.
    """

    name: str
    type_name: Optional[str]
    default: Optional[OscValue] = None


@dataclass(frozen=True)
class OscScenario:
    """A ``scenario`` declaration.

    Attributes:
        name: The (possibly qualified) scenario name.
        inherits: The base scenario name, or ``None``.
        fields: Declared fields, in source order.
        modifiers: Top-level ``modifier_application`` members (e.g. a
            scenario-level ``timeout(5s)``).
        do: The scenario's behaviour tree, or ``None`` when it declares no
            ``do`` directive.
    """

    name: str
    inherits: Optional[str] = None
    fields: list[OscField] = field(default_factory=list)
    modifiers: list[OscModifier] = field(default_factory=list)
    do: Optional[OscDoMember] = None


@dataclass(frozen=True)
class OscEnum:
    """An ``enum`` declaration."""

    name: str
    members: list[str] = field(default_factory=list)


@dataclass(frozen=True)
class OscStruct:
    """A ``struct`` declaration."""

    name: str
    fields: list[OscField] = field(default_factory=list)


@dataclass(frozen=True)
class OscProgram:
    """The top-level result of extracting a parsed ``osc_file``.

    Attributes:
        scenarios: All scenario declarations, in source order.
        enums: All enum declarations.
        structs: All struct declarations.
        imports: Raw import targets (file paths or module names).
    """

    scenarios: list[OscScenario] = field(default_factory=list)
    enums: list[OscEnum] = field(default_factory=list)
    structs: list[OscStruct] = field(default_factory=list)
    imports: list[str] = field(default_factory=list)

    def scenario_by_name(self, name: str) -> Optional[OscScenario]:
        """Return the scenario declared under *name*, or ``None``."""
        for scenario in self.scenarios:
            if scenario.name == name:
                return scenario
        return None

    @property
    def main_scenario(self) -> OscScenario:
        """Return the scenario to transpile.

        Prefers the last scenario that declares a ``do`` directive (the
        runnable one); otherwise falls back to the last scenario declared.

        Raises:
            ValueError: If the program declares no scenarios.
        """
        if not self.scenarios:
            raise ValueError("program declares no scenarios")
        with_do = [s for s in self.scenarios if s.do is not None]
        if with_do:
            return with_do[-1]
        return self.scenarios[-1]
