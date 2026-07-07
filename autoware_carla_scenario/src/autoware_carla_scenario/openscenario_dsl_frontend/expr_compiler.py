"""Compile OpenSCENARIO DSL event conditions into framework condition specs.

Used by ``wait`` directives and ``until`` clauses. The supported subset is a
single comparison on a known actor attribute, or an ``elapsed(...)`` call:

* ``<actor>.speed|velocity <op> <speed-literal>`` → :class:`SpeedCondition`
* ``<actor>.lane|lanelet == <int>`` → :class:`EntityLanePositionCondition`
* ``elapsed(<duration>)`` → :class:`ElapsedTimeCondition`

Logical combinators (``and`` / ``or`` / ``not``), distances and other
attributes are not yet handled (see the frontend docs' Roadmap).
"""

from __future__ import annotations

from typing import Callable

from .ast_model import OscComparison, OscElapsed, OscEventCond
from .errors import OscTranslationError
from .plan import CondKind, ConditionSpec
from .values import FloatValue, IntValue, OscValue, PhysicalValue

_SPEED_ATTRS = {"speed", "velocity"}
_LANE_ATTRS = {"lane", "lanelet"}
_ORDER_OPS = {"<", "<=", ">", ">="}
_OP_WORD = {"<": "lt", "<=": "le", ">": "gt", ">=": "ge", "==": "eq", "!=": "ne"}


def _speed_mps(value: OscValue) -> float:
    if isinstance(value, PhysicalValue):
        return value.to_kmh() / 3.6
    if isinstance(value, (IntValue, FloatValue)):
        # Bare number: assume km/h, consistent with the rest of the frontend.
        return value.as_float() / 3.6
    raise OscTranslationError(f"{value!r} is not a valid speed value")


def compile_event_condition(
    condition: OscEventCond, resolve_actor: Callable[[str | None], str]
) -> ConditionSpec:
    """Compile *condition* into a :class:`ConditionSpec`.

    Args:
        condition: The extracted event condition.
        resolve_actor: Callback validating/normalising a DSL actor name.

    Raises:
        OscTranslationError: If the condition uses an unsupported attribute or
            operator.
    """
    if isinstance(condition, OscElapsed):
        seconds = condition.duration.as_float()
        if isinstance(condition.duration, PhysicalValue):
            seconds = condition.duration.to_seconds()
        return ConditionSpec(
            kind=CondKind.ELAPSED,
            value=seconds,
            label=f"elapsed_{seconds:g}s",
        )

    if isinstance(condition, OscComparison):
        actor = resolve_actor(condition.lhs.actor)
        attr = condition.lhs.attr.lower()
        op = condition.op
        if attr in _SPEED_ATTRS:
            if op not in _ORDER_OPS:
                raise OscTranslationError(
                    f"speed comparison needs an ordering operator, got {op!r}"
                )
            return ConditionSpec(
                kind=CondKind.SPEED,
                actor=actor,
                op=op,
                value=_speed_mps(condition.rhs),
                label=f"{actor}_speed_{_OP_WORD[op]}",
            )
        if attr in _LANE_ATTRS:
            if op != "==":
                raise OscTranslationError(f"lane comparison must use '==', got {op!r}")
            lanelet_id = condition.rhs.as_int()
            return ConditionSpec(
                kind=CondKind.LANE,
                actor=actor,
                lanelet_id=lanelet_id,
                label=f"{actor}_on_lane_{lanelet_id}",
            )
        raise OscTranslationError(
            f"unsupported attribute {condition.lhs.actor}.{condition.lhs.attr!r} "
            "(supported: speed/velocity, lane/lanelet)"
        )

    raise OscTranslationError(f"unsupported event condition: {condition!r}")
