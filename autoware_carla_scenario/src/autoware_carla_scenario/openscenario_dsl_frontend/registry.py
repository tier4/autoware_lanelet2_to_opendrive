"""Mapping from OpenSCENARIO DSL vocabulary to framework building blocks.

This is the extensible heart of the frontend: it maps DSL *behaviour* names
(invoked inside a ``do`` directive) and *modifier* names (spawn/speed
parameters and scenario-level conditions) onto :class:`~.plan.Spec` entries and
:class:`~.plan.ActorPlan` fields.

Downstream projects can register additional handlers via
:func:`register_behavior` / :func:`register_modifier` to teach the transpiler
about custom DSL constructs without touching the translator.
"""

from __future__ import annotations

from typing import Callable

from .ast_model import OscArgument
from .errors import OscTranslationError
from .plan import ScenarioPlan, Spec, SpecKind
from .values import FloatValue, IntValue, OscValue, PhysicalValue

#: A handler mutates the plan given the resolved actor name and its arguments.
Handler = Callable[[ScenarioPlan, str, "Arguments"], None]


class Arguments:
    """Positional/named argument accessor for a DSL call."""

    def __init__(self, arguments: list[OscArgument]) -> None:
        self._positional = [a.value for a in arguments if a.name is None]
        self._named = {a.name: a.value for a in arguments if a.name is not None}

    def get(self, *names: str, index: int | None = None) -> OscValue | None:
        """Return the first matching named argument, else the positional one."""
        for name in names:
            if name in self._named:
                return self._named[name]
        if index is not None and index < len(self._positional):
            return self._positional[index]
        return None

    def require(self, *names: str, index: int | None = None) -> OscValue:
        """Like :meth:`get`, but raise when the argument is absent."""
        value = self.get(*names, index=index)
        if value is None:
            raise OscTranslationError(
                f"missing required argument (expected one of {list(names)})"
            )
        return value


def _speed_kmh(value: OscValue) -> float:
    """Coerce a speed argument to km/h (bare numbers are assumed km/h)."""
    if isinstance(value, PhysicalValue):
        return value.to_kmh()
    if isinstance(value, (IntValue, FloatValue)):
        return value.as_float()
    raise OscTranslationError(f"{value!r} is not a valid speed value")


def _seconds(value: OscValue) -> float:
    """Coerce a duration argument to seconds (bare numbers are assumed s)."""
    if isinstance(value, PhysicalValue):
        return value.to_seconds()
    if isinstance(value, (IntValue, FloatValue)):
        return value.as_float()
    raise OscTranslationError(f"{value!r} is not a valid duration value")


def _direction(value: OscValue) -> str:
    symbol = value.as_symbol().lower()
    if symbol not in ("left", "right"):
        raise OscTranslationError(
            f"direction must be 'left' or 'right', got {symbol!r}"
        )
    return symbol


def _traffic_light_state(value: OscValue) -> str:
    symbol = value.as_symbol().lower()
    if symbol not in ("green", "red", "yellow"):
        raise OscTranslationError(
            f"traffic light state must be 'green', 'red' or 'yellow', "
            f"got {symbol!r}"
        )
    return symbol


# ---------------------------------------------------------------------------
# Behaviour handlers (invoked inside `do`)
# ---------------------------------------------------------------------------


def _behavior_noop(plan: ScenarioPlan, actor: str, args: Arguments) -> None:
    """A movement behaviour with no dedicated action (e.g. ``drive``).

    Longitudinal movement is handled by CARLA's TrafficManager autopilot, so
    ``drive()`` / ``follow_lane()`` only serve to attach ``with:`` modifiers.
    """


def _behavior_turn(plan: ScenarioPlan, actor: str, args: Arguments) -> None:
    direction = _direction(args.require("direction", index=0))
    plan.specs.append(
        Spec(
            kind=SpecKind.TURN,
            actor=actor,
            label=f"{actor}_turn_{direction}",
            params={"direction": direction},
        )
    )


def _behavior_lane_change(plan: ScenarioPlan, actor: str, args: Arguments) -> None:
    direction = _direction(args.require("direction", index=0))
    plan.specs.append(
        Spec(
            kind=SpecKind.LANE_CHANGE,
            actor=actor,
            label=f"{actor}_lane_change_{direction}",
            params={"direction": direction},
        )
    )


def _behavior_traffic_signal(plan: ScenarioPlan, actor: str, args: Arguments) -> None:
    state = _traffic_light_state(args.require("state", index=0))
    plan.specs.append(
        Spec(
            kind=SpecKind.TRAFFIC_SIGNAL,
            actor=actor,
            label=f"set_traffic_lights_{state}",
            params={"state": state},
        )
    )


def _behavior_reach_lane(plan: ScenarioPlan, actor: str, args: Arguments) -> None:
    lanelet = args.require("lanelet", "lane", "road", index=0).as_int()
    plan.specs.append(
        Spec(
            kind=SpecKind.REACH_LANE,
            actor=actor,
            label=f"{actor}_reach_lane_{lanelet}",
            params={"lanelet_id": lanelet},
        )
    )


def _behavior_standstill(plan: ScenarioPlan, actor: str, args: Arguments) -> None:
    duration_value = args.get("duration", "for", index=0)
    duration = _seconds(duration_value) if duration_value is not None else 1.0
    plan.specs.append(
        Spec(
            kind=SpecKind.STANDSTILL,
            actor=actor,
            label=f"{actor}_standstill",
            params={"duration": duration},
        )
    )


def _behavior_no_collision(plan: ScenarioPlan, actor: str, args: Arguments) -> None:
    plan.specs.append(
        Spec(
            kind=SpecKind.COLLISION,
            actor=actor,
            label=f"{actor}_no_collision",
            params={},
        )
    )


def _behavior_min_speed(plan: ScenarioPlan, actor: str, args: Arguments) -> None:
    speed = _speed_kmh(args.require("speed", "value", index=0))
    plan.specs.append(
        Spec(
            kind=SpecKind.MIN_SPEED,
            actor=actor,
            label=f"{actor}_min_speed",
            params={"min_speed_kmh": speed},
        )
    )


# ---------------------------------------------------------------------------
# Modifier handlers (spawn/speed parameters and scenario-level conditions)
# ---------------------------------------------------------------------------


def _modifier_speed(plan: ScenarioPlan, actor: str, args: Arguments) -> None:
    plan.actor(actor).initial_speed_kmh = _speed_kmh(args.require("speed", index=0))


def _modifier_spawn_lanelet(plan: ScenarioPlan, actor: str, args: Arguments) -> None:
    plan.actor(actor).spawn_lanelet_id = args.require(
        "lanelet", "lane", "id", index=0
    ).as_int()


def _modifier_spawn_s(plan: ScenarioPlan, actor: str, args: Arguments) -> None:
    plan.actor(actor).spawn_s = args.require("s", "offset", index=0).as_float()


def _modifier_vehicle_type(plan: ScenarioPlan, actor: str, args: Arguments) -> None:
    plan.actor(actor).vehicle_type = args.require(
        "type", "model", "blueprint", index=0
    ).as_str()


def _modifier_timeout(plan: ScenarioPlan, actor: str, args: Arguments) -> None:
    seconds = _seconds(args.require("seconds", "after", index=0))
    plan.specs.append(
        Spec(
            kind=SpecKind.TIMEOUT,
            actor=plan.ego.name,
            label="scenario_timeout",
            params={"seconds": seconds},
        )
    )


BEHAVIOR_HANDLERS: dict[str, Handler] = {
    "drive": _behavior_noop,
    "follow_lane": _behavior_noop,
    "keep_lane": _behavior_noop,
    "follow_route": _behavior_noop,
    "turn": _behavior_turn,
    "change_lane": _behavior_lane_change,
    "lane_change": _behavior_lane_change,
    "set_traffic_lights": _behavior_traffic_signal,
    "set_traffic_light": _behavior_traffic_signal,
    "traffic_signal": _behavior_traffic_signal,
    "reach_lane": _behavior_reach_lane,
    "on_lane": _behavior_reach_lane,
    "reach_position": _behavior_reach_lane,
    "stop": _behavior_standstill,
    "stand_still": _behavior_standstill,
    "standstill": _behavior_standstill,
    "avoid_collision": _behavior_no_collision,
    "no_collision": _behavior_no_collision,
    "keep_speed_above": _behavior_min_speed,
    "min_speed": _behavior_min_speed,
}

MODIFIER_HANDLERS: dict[str, Handler] = {
    "speed": _modifier_speed,
    "initial_speed": _modifier_speed,
    "spawn_lanelet": _modifier_spawn_lanelet,
    "spawn_lane": _modifier_spawn_lanelet,
    "lanelet": _modifier_spawn_lanelet,
    "position": _modifier_spawn_lanelet,
    "spawn_s": _modifier_spawn_s,
    "offset": _modifier_spawn_s,
    "vehicle_type": _modifier_vehicle_type,
    "model": _modifier_vehicle_type,
    "timeout": _modifier_timeout,
    "keep_speed_above": _behavior_min_speed,
    "min_speed": _behavior_min_speed,
}


def register_behavior(name: str, handler: Handler) -> None:
    """Register a custom behaviour handler (overrides any existing entry)."""
    BEHAVIOR_HANDLERS[name] = handler


def register_modifier(name: str, handler: Handler) -> None:
    """Register a custom modifier handler (overrides any existing entry)."""
    MODIFIER_HANDLERS[name] = handler


def behavior_handler(name: str) -> Handler:
    """Look up a behaviour handler by name.

    Raises:
        OscTranslationError: If *name* is unknown.
    """
    handler = BEHAVIOR_HANDLERS.get(name)
    if handler is None:
        raise OscTranslationError(
            f"unsupported behaviour {name!r}. Known behaviours: "
            f"{sorted(BEHAVIOR_HANDLERS)}"
        )
    return handler


def modifier_handler(name: str) -> Handler:
    """Look up a modifier handler by name.

    Raises:
        OscTranslationError: If *name* is unknown.
    """
    handler = MODIFIER_HANDLERS.get(name)
    if handler is None:
        raise OscTranslationError(
            f"unsupported modifier {name!r}. Known modifiers: "
            f"{sorted(MODIFIER_HANDLERS)}"
        )
    return handler
