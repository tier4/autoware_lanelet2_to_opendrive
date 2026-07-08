"""Mapping from OpenSCENARIO DSL vocabulary to framework building blocks.

This is the extensible heart of the frontend. *Behaviour* handlers (invoked
inside a ``do`` directive) return a :class:`~.plan.BehaviorResult` describing the
action/condition specs they contribute plus a *completion gate* used to
sequence serial steps. *Modifier* handlers (spawn/speed parameters and
scenario-level conditions) mutate the plan in place.

Downstream projects can register additional handlers via
:func:`register_behavior` / :func:`register_modifier` to teach the transpiler
about custom DSL constructs without touching the translator.
"""

from __future__ import annotations

from typing import Callable

from .ast_model import OscArgument
from .errors import OscTranslationError
from .plan import BehaviorResult, Gate, ScenarioPlan, Spec, SpecKind
from .values import FloatValue, IntValue, OscValue, PhysicalValue

#: A behaviour handler builds specs + a completion gate from a call's arguments.
BehaviorHandler = Callable[[str, "Arguments"], BehaviorResult]

#: A modifier handler mutates the plan for the resolved actor.
ModifierHandler = Callable[[ScenarioPlan, str, "Arguments"], None]


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


def _meters(value: OscValue) -> float:
    """Coerce a length argument to metres (bare numbers are assumed metres)."""
    if isinstance(value, PhysicalValue):
        return value.to_meters()
    if isinstance(value, (IntValue, FloatValue)):
        return value.as_float()
    raise OscTranslationError(f"{value!r} is not a valid length value")


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
            f"traffic light state must be 'green', 'red' or 'yellow', got {symbol!r}"
        )
    return symbol


# ---------------------------------------------------------------------------
# Behaviour handlers (invoked inside `do`)
# ---------------------------------------------------------------------------


def _behavior_noop(actor: str, args: Arguments) -> BehaviorResult:
    """A movement behaviour with no dedicated action (e.g. ``drive``).

    Longitudinal movement is handled by CARLA's TrafficManager autopilot, so
    ``drive()`` / ``follow_lane()`` only serve to attach ``with:`` modifiers.
    Completion is ``None`` so it does not advance a serial sequence.
    """
    return BehaviorResult()


def _behavior_turn(actor: str, args: Arguments) -> BehaviorResult:
    direction = _direction(args.require("direction", index=0))
    spec = Spec(
        kind=SpecKind.TURN,
        actor=actor,
        label=f"{actor}_turn_{direction}",
        params={"direction": direction},
    )
    return BehaviorResult(actions=[spec], completion=Gate.action_done(spec.label))


def _behavior_lane_change(actor: str, args: Arguments) -> BehaviorResult:
    direction = _direction(args.require("direction", "side", index=0))
    count_value = args.get("lane_changes", "count")
    lane_changes = count_value.as_int() if count_value is not None else 1
    spec = Spec(
        kind=SpecKind.LANE_CHANGE,
        actor=actor,
        label=f"{actor}_lane_change_{direction}",
        params={"direction": direction, "lane_changes": lane_changes},
    )
    return BehaviorResult(actions=[spec], completion=Gate.action_done(spec.label))


def _behavior_traffic_signal(actor: str, args: Arguments) -> BehaviorResult:
    state = _traffic_light_state(args.require("state", index=0))
    spec = Spec(
        kind=SpecKind.TRAFFIC_SIGNAL,
        actor=actor,
        label=f"set_traffic_lights_{state}",
        params={"state": state},
    )
    return BehaviorResult(actions=[spec], completion=Gate.action_done(spec.label))


def _behavior_reach_lane(actor: str, args: Arguments) -> BehaviorResult:
    lanelet = args.require("lanelet", "lane", "road", index=0).as_int()
    spec = Spec(
        kind=SpecKind.REACH_LANE,
        actor=actor,
        label=f"{actor}_reach_lane_{lanelet}",
        params={"lanelet_id": lanelet},
    )
    return BehaviorResult(passes=[spec], completion=Gate.lane_reached(actor, lanelet))


def _behavior_standstill(actor: str, args: Arguments) -> BehaviorResult:
    duration_value = args.get("duration", "for", index=0)
    duration = _seconds(duration_value) if duration_value is not None else 1.0
    spec = Spec(
        kind=SpecKind.STANDSTILL,
        actor=actor,
        label=f"{actor}_standstill",
        params={"duration": duration},
    )
    return BehaviorResult(passes=[spec], completion=Gate.standstill(actor, duration))


def _behavior_no_collision(actor: str, args: Arguments) -> BehaviorResult:
    spec = Spec(
        kind=SpecKind.COLLISION,
        actor=actor,
        label=f"{actor}_no_collision",
        params={},
    )
    # A continuous monitor: does not advance a serial sequence.
    return BehaviorResult(fails=[spec], completion=None)


def _behavior_min_speed(actor: str, args: Arguments) -> BehaviorResult:
    speed = _speed_kmh(args.require("speed", "value", index=0))
    spec = Spec(
        kind=SpecKind.MIN_SPEED,
        actor=actor,
        label=f"{actor}_min_speed",
        params={"min_speed_kmh": speed},
    )
    return BehaviorResult(fails=[spec], completion=None)


# ---------------------------------------------------------------------------
# Modifier handlers (spawn/speed parameters and scenario-level conditions)
# ---------------------------------------------------------------------------


def _modifier_speed(plan: ScenarioPlan, actor: str, args: Arguments) -> None:
    plan.actor(actor).initial_speed_kmh = _speed_kmh(args.require("speed", index=0))


def _modifier_spawn_lanelet(plan: ScenarioPlan, actor: str, args: Arguments) -> None:
    plan.actor(actor).spawn_lanelet_id = args.require(
        "lanelet", "lanelet_id", "id", index=0
    ).as_int()


def _modifier_spawn_s(plan: ScenarioPlan, actor: str, args: Arguments) -> None:
    plan.actor(actor).spawn_s = args.require("s", index=0).as_float()


def _modifier_lanelet_position(plan: ScenarioPlan, actor: str, args: Arguments) -> None:
    """Lanelet2 spawn position: ``lanelet_position(lanelet:, s:)``.

    This is a deliberate *extension* type, not the OpenSCENARIO/OpenDRIVE
    standard ``lane_position`` — the framework addresses spawns by Lanelet2
    lanelet id (:class:`Lanelet2Pose`), which is a distinct concept from an
    OpenDRIVE (road, lane). ``s`` is the standard Frenet arc-length. A lateral
    ``offset`` / ``t`` is rejected because the spawn uses ``(lanelet, s)`` only.
    """
    target = plan.actor(actor)
    target.spawn_lanelet_id = args.require("lanelet", "lanelet_id", index=0).as_int()
    s = args.get("s", index=1)
    if s is not None:
        target.spawn_s = s.as_float()
    if args.get("offset", "t") is not None:
        raise OscTranslationError(
            "lanelet_position lateral 'offset'/'t' is not supported: the spawn "
            "only uses (lanelet, s)"
        )


def _anchor_of(args: Arguments) -> str:
    """Return the ``at:`` time anchor of a placement modifier (default start)."""
    at = args.get("at")
    return at.as_symbol().lower() if at is not None else "start"


def _merge_relative_spawn(
    plan: ScenarioPlan,
    actor: str,
    reference: str,
    *,
    side: str | None = None,
    longitudinal: float | None = None,
    anchor: str = "start",
) -> None:
    """Attach or extend an actor's relative-spawn placement.

    ``lane(right_of: ego)`` and ``position(15m, behind: ego)`` contribute
    complementary lateral / longitudinal parts of the same placement, so they
    are merged into one :class:`~.plan.RelativeSpawn`.
    """
    from .plan import RelativeSpawn

    if reference not in {plan.ego.name, *(n.name for n in plan.npcs)}:
        raise OscTranslationError(
            f"relative placement references unknown actor {reference!r}"
        )
    target = plan.actor(actor)
    current = target.relative_spawn
    target.relative_spawn = RelativeSpawn(
        reference=reference,
        side=side if side is not None else (current.side if current else None),
        longitudinal=(
            longitudinal
            if longitudinal is not None
            else (current.longitudinal if current else 0.0)
        ),
        anchor=anchor,
    )


def _modifier_lane(plan: ScenarioPlan, actor: str, args: Arguments) -> None:
    """Lane placement: ``lane(<index>, at:)`` or ``lane(right_of:/left_of:)``.

    An absolute lane index is map-relative and recorded as advisory metadata;
    a ``right_of`` / ``left_of`` reference is a relational lateral placement.
    """
    anchor = _anchor_of(args)
    for side, key in (("right", "right_of"), ("left", "left_of")):
        ref = args.get(key)
        if ref is not None:
            _merge_relative_spawn(
                plan, actor, ref.as_symbol(), side=side, anchor=anchor
            )
            return
    index = args.get("lane", index=0)
    if index is not None:
        plan.actor(actor).spawn_lane_index = index.as_int()
        plan.notes.append(
            f"{actor} starts in map-relative lane index {index.as_int()} "
            f"(at: {anchor}); set 'spawn_lanelet_id' in the config to pin it."
        )


def _modifier_position(plan: ScenarioPlan, actor: str, args: Arguments) -> None:
    """Longitudinal placement: ``position(<distance>, behind:/ahead_of:, at:)``.

    ``at: start`` placements resolve to a concrete spawn relative to the
    reference actor. An ``at: end`` placement is a *dynamic* relative goal —
    both vehicles move, so it becomes a per-frame ``RelativePositionAction``
    that drives the actor toward the moving target gap.
    """
    anchor = _anchor_of(args)
    distance_value = args.get("distance", index=0)
    distance = _meters(distance_value) if distance_value is not None else 0.0
    behind = args.get("behind")
    ahead = args.get("ahead_of", "ahead")
    if behind is None and ahead is None:
        return
    reference = (behind or ahead).as_symbol()  # type: ignore[union-attr]
    longitudinal = -distance if behind is not None else distance

    if anchor == "start":
        _merge_relative_spawn(
            plan, actor, reference, longitudinal=longitudinal, anchor=anchor
        )
        return

    if reference not in {plan.ego.name, *(n.name for n in plan.npcs)}:
        raise OscTranslationError(
            f"relative placement references unknown actor {reference!r}"
        )
    side = "ahead" if longitudinal > 0 else "behind"
    plan.specs.append(
        Spec(
            kind=SpecKind.RELATIVE_POSITION,
            actor=actor,
            label=f"{actor}_{side}_{reference}",
            params={"reference": reference, "target_gap": longitudinal},
            # Gated by the enclosing invocation; the translator assigns the
            # phase entry gate to modifier-produced action specs.
            gate=Gate.immediate(),
        )
    )


def _modifier_set_map(plan: ScenarioPlan, actor: str, args: Arguments) -> None:
    """``set_map("Town04")`` — advisory only; the map is chosen at run time."""
    plan.map_hint = args.require("map", index=0).as_str()


def _modifier_min_driving_lanes(
    plan: ScenarioPlan, actor: str, args: Arguments
) -> None:
    """``path_min_driving_lanes(n)`` — a pre-run lanelet-constraint sweep.

    Expressed with the ``has_adjacent`` constraint of the Hydra
    lanelet-constraint sweeper: a lane with both a left and a right neighbour
    sits on a road of at least 3 lanes; one neighbour means at least 2. The
    sweeper resolves ``ego.spawn_lanelet_id`` to a lanelet satisfying this
    against whatever map is chosen at run time.
    """
    lanes = args.require("lanes", index=0).as_int()
    left = {"type": "has_adjacent", "value": "left"}
    right = {"type": "has_adjacent", "value": "right"}
    if lanes >= 3:
        node: dict[str, object] = {"type": "and", "constraints": [left, right]}
    elif lanes == 2:
        node = {"type": "or", "constraints": [left, right]}
    else:
        return
    plan.sweep_constraints.setdefault("ego.spawn_lanelet_id", []).append(node)


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


def _modifier_min_speed(plan: ScenarioPlan, actor: str, args: Arguments) -> None:
    speed = _speed_kmh(args.require("speed", "value", index=0))
    plan.specs.append(
        Spec(
            kind=SpecKind.MIN_SPEED,
            actor=actor,
            label=f"{actor}_min_speed",
            params={"min_speed_kmh": speed},
        )
    )


BEHAVIOR_HANDLERS: dict[str, BehaviorHandler] = {
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

MODIFIER_HANDLERS: dict[str, ModifierHandler] = {
    "speed": _modifier_speed,
    "initial_speed": _modifier_speed,
    # Recommended: a Lanelet2 spawn-position extension type (see below).
    "lanelet_position": _modifier_lanelet_position,
    # Aliases that set the individual spawn fields.
    "spawn_lanelet": _modifier_spawn_lanelet,
    "lanelet": _modifier_spawn_lanelet,
    "spawn_s": _modifier_spawn_s,
    "vehicle_type": _modifier_vehicle_type,
    "model": _modifier_vehicle_type,
    "timeout": _modifier_timeout,
    "keep_speed_above": _modifier_min_speed,
    "min_speed": _modifier_min_speed,
    # Relational placement (map-relative / inter-actor).
    "lane": _modifier_lane,
    "position": _modifier_position,
    # Struct-scoped path directives (not bound to a vehicle actor).
    "set_map": _modifier_set_map,
    "path_min_driving_lanes": _modifier_min_driving_lanes,
}

#: Modifiers scoped to a non-vehicle struct (e.g. ``path.set_map(...)``); the
#: translator applies these without resolving their scope as an actor.
PLAN_LEVEL_MODIFIERS: frozenset[str] = frozenset({"set_map", "path_min_driving_lanes"})


def register_behavior(name: str, handler: BehaviorHandler) -> None:
    """Register a custom behaviour handler (overrides any existing entry)."""
    BEHAVIOR_HANDLERS[name] = handler


def register_modifier(name: str, handler: ModifierHandler) -> None:
    """Register a custom modifier handler (overrides any existing entry)."""
    MODIFIER_HANDLERS[name] = handler


def behavior_handler(name: str) -> BehaviorHandler:
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


def modifier_handler(name: str) -> ModifierHandler:
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
