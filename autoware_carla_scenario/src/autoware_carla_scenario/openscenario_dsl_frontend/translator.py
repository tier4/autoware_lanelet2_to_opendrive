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
from typing import Callable

from .ast_model import (
    OscComposition,
    OscDoMember,
    OscInvocation,
    OscProgram,
    OscScenario,
)
from .errors import OscTranslationError
from .plan import MAX_VARIANTS, ActorPlan, Gate, ScenarioPlan
from .registry import Arguments, behavior_handler, modifier_handler

logger = logging.getLogger(__name__)

#: Field types that denote a controllable vehicle actor.
_VEHICLE_TYPES = {"vehicle", "car", "ego", "npc"}


def _resolve_actors(scenario: OscScenario) -> tuple[ActorPlan, list[ActorPlan]]:
    """Split the scenario's vehicle fields into a fresh ego actor and NPCs."""
    vehicle_fields = [
        f
        for f in scenario.fields
        if (f.type_name or "").lower() in _VEHICLE_TYPES or f.name == "ego"
    ]

    ego: ActorPlan | None = None
    npcs: list[ActorPlan] = []
    npc_index = 0
    for f in vehicle_fields:
        if f.name == "ego" and ego is None:
            ego = ActorPlan(name="ego", is_ego=True)
        else:
            npc_index += 1
            npcs.append(ActorPlan(name=f.name, is_ego=False, index=npc_index))

    if ego is None:
        if npcs:
            first = npcs.pop(0)
            ego = ActorPlan(name=first.name, is_ego=True)
            for i, npc in enumerate(npcs, start=1):
                npc.index = i
        else:
            ego = ActorPlan(name="ego", is_ego=True)
    return ego, npcs


def _expand_one_of(node: OscDoMember) -> list[OscDoMember]:
    """Expand ``one_of`` choices into a list of concrete ``do`` trees.

    ``one_of`` contributes alternatives (union); ``serial``/``parallel``
    contribute the Cartesian product of their members, preserving the operator.
    """
    if isinstance(node, OscInvocation):
        return [node]

    member_options = [_expand_one_of(m) for m in node.members]
    if node.operator == "one_of":
        return [opt for options in member_options for opt in options]

    variants: list[OscDoMember] = []
    for combo in itertools.product(*member_options):
        variants.append(OscComposition(operator=node.operator, members=list(combo)))
    return variants


def _apply(
    plan: ScenarioPlan,
    node: OscDoMember,
    entry_gate: Gate,
    resolve: Callable[[str | None], str],
) -> Gate:
    """Walk a ``do`` tree, appending specs and returning its completion gate."""
    if isinstance(node, OscInvocation):
        actor = resolve(node.actor)
        for modifier in node.modifiers:
            mactor = resolve(modifier.actor) if modifier.actor else actor
            modifier_handler(modifier.name)(plan, mactor, Arguments(modifier.arguments))

        result = behavior_handler(node.behavior)(actor, Arguments(node.arguments))
        for action in result.actions:
            action.gate = entry_gate
            plan.specs.append(action)
        plan.specs.extend(result.passes)
        plan.specs.extend(result.fails)
        return result.completion if result.completion is not None else entry_gate

    if node.operator == "parallel":
        completions = [_apply(plan, m, entry_gate, resolve) for m in node.members]
        advancing = [
            c for c in completions if c is not entry_gate and not c.is_immediate
        ]
        if not advancing:
            return entry_gate
        if len(advancing) == 1:
            return advancing[0]
        return Gate.all_of(advancing)

    # serial (default): thread the gate through the members in order.
    gate = entry_gate
    for member in node.members:
        gate = _apply(plan, member, gate, resolve)
    return gate


def _build_plan(
    scenario: OscScenario,
    do_tree: OscDoMember | None,
    variant_index: int,
    variant_count: int,
) -> ScenarioPlan:
    """Build one :class:`ScenarioPlan` for a single ``do`` variant."""
    ego, npcs = _resolve_actors(scenario)
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
        modifier_handler(modifier.name)(
            plan, resolve(modifier.actor), Arguments(modifier.arguments)
        )

    if do_tree is not None:
        _apply(plan, do_tree, Gate.immediate(), resolve)

    return plan


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
