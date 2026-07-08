"""Translate the syntax IR into semantic :class:`~.plan.ScenarioPlan` variants.

Two things happen here beyond the naive mapping:

* **Serial sequencing** — a ``serial`` block threads a *gate* through its steps
  so that each action is triggered only once the previous step has completed
  (``ActionDoneCondition`` for issued manoeuvres, a lane/standstill condition
  for observable steps). ``parallel`` members share the block's entry gate and
  its completion is the conjunction of the members' completions.
* **``one_of`` expansion** — an exclusive choice is expanded into one concrete
  :class:`~.plan.ScenarioPlan` per branch combination (a Cartesian product over
  ``serial``/``parallel`` nesting, a union over ``one_of``), matching the
  "logical scenario → concrete scenarios" model the runner already uses.
"""

from __future__ import annotations

import itertools
import logging
from typing import Any, Callable

from .ast_model import (
    OscComposition,
    OscDoMember,
    OscField,
    OscInvocation,
    OscProgram,
    OscScenario,
    OscWait,
)
from .errors import OscTranslationError
from .expr_compiler import compile_event_condition
from .plan import (
    MAX_VARIANTS,
    ActorPlan,
    CondKind,
    ConditionSpec,
    Gate,
    GateKind,
    ScenarioPlan,
    Spec,
    SpecKind,
    SpecRole,
)
from .registry import (
    BEHAVIOR_HANDLERS,
    MODIFIER_HANDLERS,
    PLAN_LEVEL_MODIFIERS,
    Arguments,
    behavior_handler,
    modifier_handler,
    speed_kmh,
)

logger = logging.getLogger(__name__)

#: Modifiers that set a speed: in the initial context they set the spawn speed,
#: but in a later (gated) phase they become a timed :class:`SpeedAction`.
_SPEED_MODIFIERS = frozenset({"speed", "initial_speed"})

#: Field types that denote a controllable vehicle actor.
_VEHICLE_TYPES = {"vehicle", "car", "ego", "npc"}

#: Non-vehicle struct field types that must never be treated as actors.
_NON_ACTOR_TYPES = {"path", "route", "map"}


def _referenced_actors(node: OscDoMember | None) -> list[str]:
    """Return actor names invoked in a ``do`` tree, in first-seen order."""
    order: list[str] = []

    def visit(member: OscDoMember | None) -> None:
        if isinstance(member, OscInvocation):
            if member.actor and member.actor not in order:
                order.append(member.actor)
        elif isinstance(member, OscComposition):
            for child in member.members:
                visit(child)

    visit(node)
    return order


def _seconds_of(value: object) -> float:
    """Best-effort duration coercion for a phase ``duration:`` value."""
    to_seconds = getattr(value, "to_seconds", None)
    if callable(to_seconds):
        return float(to_seconds())
    as_float = getattr(value, "as_float", None)
    return float(as_float()) if callable(as_float) else 0.0


def _resolve_actors(
    scenario: OscScenario, do_tree: OscDoMember | None
) -> tuple[ActorPlan, list[ActorPlan]]:
    """Split the scenario's vehicle fields into a fresh ego actor and NPCs.

    A field is a vehicle actor when it is driven in the ``do`` tree, has a
    vehicle-like type, or is named after the ego — but never when its type is
    a known non-vehicle struct (e.g. ``path: Path``). The ego is the actor
    whose name mentions ``ego``; otherwise the first actor is promoted.
    """
    referenced = _referenced_actors(do_tree)

    def is_actor(f: OscField) -> bool:
        type_name = (f.type_name or "").lower()
        if type_name in _NON_ACTOR_TYPES:
            return False
        return (
            f.name in referenced
            or type_name in _VEHICLE_TYPES
            or "ego" in f.name.lower()
        )

    vehicle_fields = [f for f in scenario.fields if is_actor(f)]

    ego_names = [f.name for f in vehicle_fields if "ego" in f.name.lower()]
    ego_name = ego_names[0] if ego_names else None
    if ego_name is None and vehicle_fields:
        ego_name = vehicle_fields[0].name

    ego = ActorPlan(name=ego_name or "ego", is_ego=True)
    npcs: list[ActorPlan] = []
    for f in vehicle_fields:
        if f.name == ego.name:
            continue
        npcs.append(ActorPlan(name=f.name, is_ego=False, index=len(npcs) + 1))
    return ego, npcs


def _expand_one_of(node: OscDoMember) -> list[OscDoMember]:
    """Expand ``one_of`` choices into a list of concrete ``do`` trees.

    ``one_of`` contributes alternatives (union); ``serial``/``parallel``
    contribute the Cartesian product of their members, preserving the operator.
    """
    if not isinstance(node, OscComposition):
        return [node]  # leaf: invocation or wait

    member_options = [_expand_one_of(m) for m in node.members]
    if node.operator == "one_of":
        return [opt for options in member_options for opt in options]

    variants: list[OscDoMember] = []
    for combo in itertools.product(*member_options):
        variants.append(
            OscComposition(
                operator=node.operator,
                members=list(combo),
                label=node.label,
                duration=node.duration,
            )
        )
    return variants


def _apply(
    plan: ScenarioPlan,
    node: OscDoMember,
    entry_gate: Gate,
    resolve: Callable[[str | None], str],
    ordered: bool,
) -> Gate:
    """Walk a ``do`` tree, appending specs and returning its completion gate.

    *ordered* is ``True`` while every enclosing composition is ``serial``; it
    marks pass-condition specs that must be satisfied in sequence.
    """
    if isinstance(node, OscWait):
        # A wait registers no action; its condition gates the next serial step.
        cond_gate = Gate.from_condition(
            compile_event_condition(node.condition, resolve)
        )
        if entry_gate.is_immediate:
            return cond_gate
        return Gate.all_of([entry_gate, cond_gate])

    if isinstance(node, OscInvocation):
        actor = resolve(node.actor)
        before = len(plan.specs)
        for modifier in node.modifiers:
            mactor = resolve(modifier.actor) if modifier.actor else actor
            args = Arguments(modifier.arguments)
            if (
                modifier.name in BEHAVIOR_HANDLERS
                and modifier.name not in MODIFIER_HANDLERS
            ):
                # An action-producing behaviour used as a ``with:`` modifier
                # (e.g. ``change_lane(...)``): register it as a gated action.
                mresult = behavior_handler(modifier.name)(mactor, args)
                for maction in mresult.actions:
                    maction.gate = entry_gate
                    plan.specs.append(maction)
                plan.specs.extend(mresult.passes)
                plan.specs.extend(mresult.fails)
            elif modifier.name in _SPEED_MODIFIERS and not entry_gate.is_immediate:
                # A speed() in a later (gated) phase is a *timed* speed change,
                # not the spawn speed: emit a gated SpeedAction.
                speed = speed_kmh(args.require("speed", "value", index=0))
                plan.specs.append(
                    Spec(
                        kind=SpecKind.SPEED_ACTION,
                        actor=mactor,
                        label=f"{mactor}_speed_{speed:g}",
                        params={"speed_kmh": speed},
                        gate=entry_gate,
                    )
                )
            else:
                modifier_handler(modifier.name)(plan, mactor, args)
        # Modifier handlers that emit an action spec (e.g. a dynamic
        # ``position(..., at: end)`` goal) leave it immediate; bind it to this
        # invocation's entry gate so it arms with the enclosing phase.
        for spec in plan.specs[before:]:
            if spec.role is SpecRole.ACTION and spec.gate.is_immediate:
                spec.gate = entry_gate

        result = behavior_handler(node.behavior)(actor, Arguments(node.arguments))
        for action in result.actions:
            action.gate = entry_gate
            plan.specs.append(action)
        for pass_spec in result.passes:
            pass_spec.ordered = ordered
            plan.specs.append(pass_spec)
        plan.specs.extend(result.fails)

        # An explicit `until` clause overrides the inferred completion signal.
        if node.until is not None:
            return Gate.from_condition(compile_event_condition(node.until, resolve))
        return result.completion if result.completion is not None else entry_gate

    if node.operator == "parallel":
        # Members run concurrently, so they are unordered among themselves.
        completions = [
            _apply(plan, m, entry_gate, resolve, ordered=False) for m in node.members
        ]
        advancing = [
            c for c in completions if c is not entry_gate and not c.is_immediate
        ]
        if not advancing:
            return entry_gate
        if len(advancing) == 1:
            return advancing[0]
        return Gate.all_of(advancing)

    # serial (default): thread the gate through the members in order. A member
    # that is a *timed* phase (``parallel(duration: 15s)``) advances the
    # sequence by wall-clock time, so the next phase arms at the cumulative
    # elapsed instant rather than on an observable completion.
    gate = entry_gate
    elapsed = 0.0
    if (
        entry_gate.kind is GateKind.CONDITION
        and entry_gate.condition is not None
        and entry_gate.condition.kind is CondKind.ELAPSED
    ):
        elapsed = entry_gate.condition.value or 0.0
    for member in node.members:
        completion = _apply(plan, member, gate, resolve, ordered=ordered)
        if isinstance(member, OscComposition) and member.duration is not None:
            elapsed += _seconds_of(member.duration)
            gate = Gate.from_condition(
                ConditionSpec(
                    kind=CondKind.ELAPSED,
                    value=elapsed,
                    label=f"after_{member.label or 'phase'}_{elapsed:g}s",
                )
            )
        else:
            gate = completion
    return gate


def _build_plan(
    scenario: OscScenario,
    do_tree: OscDoMember | None,
    variant_index: int,
    variant_count: int,
) -> ScenarioPlan:
    """Build one :class:`ScenarioPlan` for a single ``do`` variant."""
    ego, npcs = _resolve_actors(scenario, do_tree)
    plan = ScenarioPlan(
        name=scenario.name,
        ego=ego,
        npcs=npcs,
        variant_index=variant_index,
        variant_count=variant_count,
    )
    known_actors = {ego.name, *(n.name for n in npcs)}

    def resolve(actor: str | None) -> str:
        if actor is None:
            return ego.name
        if actor not in known_actors:
            raise OscTranslationError(
                f"unknown actor {actor!r} (declared actors: {sorted(known_actors)})"
            )
        return actor

    for modifier in scenario.modifiers:
        # Struct-scoped directives (``path.set_map(...)``) are not bound to a
        # vehicle actor, so they bypass actor resolution.
        actor = "" if modifier.name in PLAN_LEVEL_MODIFIERS else resolve(modifier.actor)
        modifier_handler(modifier.name)(plan, actor, Arguments(modifier.arguments))

    if do_tree is not None:
        _apply(plan, do_tree, Gate.immediate(), resolve, ordered=True)

    _resolve_relative_spawns(plan)
    _finalize_spawn_constraints(plan)
    _ensure_phase_timeout(plan, do_tree)

    return plan


def _has_adjacent(side: str) -> dict[str, str]:
    return {"type": "has_adjacent", "value": side}


def _not(node: dict[str, Any]) -> dict[str, Any]:
    return {"type": "not", "constraint": node}


def _finalize_spawn_constraints(plan: ScenarioPlan) -> None:
    """Build the lanelet-constraint sweep for lateral / lane placement.

    Collapses the ego lane index, the ``path_min_driving_lanes`` count and any
    NPC ``right_of`` / ``left_of`` relation into a single consistent set of
    ``has_adjacent`` constraints on ``ego.spawn_lanelet_id`` plus one
    ``adjacent_lanelet`` binding per relationally-placed NPC. Because
    ``has_adjacent`` only tests the existence of a neighbour (it cannot count
    lanes), an exact lane count from a fixed lane index degrades to "has a
    neighbour on the required side".
    """
    atoms: list[dict[str, Any]] = []

    index = plan.ego.spawn_lane_index
    if index == 1:
        atoms.append(_not(_has_adjacent("left")))  # leftmost lane
    elif index is not None and index >= 2:
        atoms.append(_has_adjacent("left"))  # has lane(s) to the left

    sides: set[str] = set()
    for npc in plan.npcs:
        rel = npc.relative_spawn
        if rel is None or rel.side is None or rel.reference != plan.ego.name:
            continue
        sides.add(rel.side)
        npc.config_spawn_lanelet = True
        plan.sweep_bindings[f"scenario.npc_{npc.index}_spawn_lanelet_id"] = {
            "type": "adjacent_lanelet",
            "side": rel.side,
        }
    for side in sorted(sides):
        # The ego must have a neighbour on that side for the NPC to sit there.
        atoms.append(_has_adjacent(side))

    if not atoms and plan.min_driving_lanes is not None:
        left, right = _has_adjacent("left"), _has_adjacent("right")
        if plan.min_driving_lanes >= 3:
            atoms.append({"type": "and", "constraints": [left, right]})
        elif plan.min_driving_lanes == 2:
            atoms.append({"type": "or", "constraints": [left, right]})

    # De-duplicate while preserving order.
    seen: list[dict[str, Any]] = []
    for atom in atoms:
        if atom not in seen:
            seen.append(atom)
    if seen:
        plan.sweep_constraints.setdefault("ego.spawn_lanelet_id", []).extend(seen)


def _resolve_relative_spawns(plan: ScenarioPlan) -> None:
    """Resolve each NPC's ``at: start`` relative placement.

    A **longitudinal** offset relative to the *ego* is left symbolic — the
    generated ``setup`` derives the NPC pose from the ego's spawn pose
    (``self._spawn_pose``) at runtime, so no map and no transpile-time value are
    needed. A longitudinal offset relative to *another NPC* is resolved
    numerically here. A **lateral** neighbour (``right_of`` / ``left_of``) is
    map-dependent and kept as a note.
    """
    for npc in plan.npcs:
        rel = npc.relative_spawn
        if rel is None or rel.anchor != "start":
            continue
        if rel.reference == plan.ego.name:
            # Longitudinal placement relative to the ego is emitted as code
            # against ``self._spawn_pose``; a lateral neighbour is resolved by
            # the sweeper (see ``_finalize_spawn_constraints``). Nothing to
            # resolve numerically here.
            continue
        # Reference is another NPC: resolve the offset numerically.
        reference = plan.actor(rel.reference)
        if npc.spawn_lanelet_id is None:
            npc.spawn_lanelet_id = reference.spawn_lanelet_id
        npc.spawn_s = max(0.0, reference.spawn_s + rel.longitudinal)
        where = f"{abs(rel.longitudinal):g} m "
        where += "ahead of" if rel.longitudinal > 0 else "behind"
        plan.notes.append(
            f"npc {npc.index} ({npc.name}) spawns {where} {rel.reference} "
            f"(spawn_s={npc.spawn_s:g})."
        )


def _ensure_phase_timeout(plan: ScenarioPlan, do_tree: OscDoMember | None) -> None:
    """Derive a fail-safe timeout from timed phases when none was declared."""
    if any(s.kind is SpecKind.TIMEOUT for s in plan.specs):
        return
    total = _total_duration(do_tree)
    if total <= 0:
        return
    plan.specs.append(
        Spec(
            kind=SpecKind.TIMEOUT,
            actor=plan.ego.name,
            label="scenario_timeout",
            params={"seconds": total},
        )
    )


def _total_duration(node: OscDoMember | None) -> float:
    """Sum the durations of a serial run of timed phases."""
    if not isinstance(node, OscComposition):
        return 0.0
    child_total = sum(
        _seconds_of(m.duration)
        for m in node.members
        if isinstance(m, OscComposition) and m.duration is not None
    )
    if child_total:
        return child_total
    return _seconds_of(node.duration) if node.duration is not None else 0.0


def translate_scenario(scenario: OscScenario) -> list[ScenarioPlan]:
    """Translate a scenario into one plan per ``one_of`` variant."""
    variants: list[OscDoMember | None]
    if scenario.do is None:
        variants = [None]
    else:
        variants = list(_expand_one_of(scenario.do))
        if len(variants) > MAX_VARIANTS:
            logger.warning(
                "scenario %r produced %d one_of variants; truncating to %d",
                scenario.name,
                len(variants),
                MAX_VARIANTS,
            )
            variants = variants[:MAX_VARIANTS]

    count = len(variants)
    return [
        _build_plan(scenario, do_tree, index, count)
        for index, do_tree in enumerate(variants)
    ]


def translate_program(program: OscProgram) -> list[ScenarioPlan]:
    """Translate a program's :attr:`~.OscProgram.main_scenario` into variants.

    Raises:
        ValueError: If the program declares no scenarios.
    """
    return translate_scenario(program.main_scenario)
