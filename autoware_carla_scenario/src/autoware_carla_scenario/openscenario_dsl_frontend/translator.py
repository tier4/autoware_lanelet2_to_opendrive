"""Translate the syntax IR into a semantic :class:`~.plan.ScenarioPlan`.

This is where DSL constructs are resolved into concrete framework intent:
vehicle-typed fields become actors, ``do`` behaviour invocations and their
``with:`` modifiers are dispatched through :mod:`.registry`, and scenario-level
modifiers (such as ``timeout``) become fail conditions.
"""

from __future__ import annotations

from typing import Iterable

from .ast_model import (
    OscComposition,
    OscDoMember,
    OscInvocation,
    OscProgram,
    OscScenario,
)
from .errors import OscTranslationError
from .plan import ActorPlan, ScenarioPlan
from .registry import Arguments, behavior_handler, modifier_handler

#: Field types that denote a controllable vehicle actor.
_VEHICLE_TYPES = {"vehicle", "car", "ego", "npc"}


def _resolve_actors(scenario: OscScenario) -> tuple[ActorPlan, list[ActorPlan]]:
    """Split the scenario's vehicle fields into an ego actor and NPCs."""
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
        # Promote the first NPC to ego, or synthesise a default ego so the
        # generated scenario always has the mandatory ego vehicle.
        if npcs:
            first = npcs.pop(0)
            ego = ActorPlan(name=first.name, is_ego=True)
            for i, npc in enumerate(npcs, start=1):
                npc.index = i
        else:
            ego = ActorPlan(name="ego", is_ego=True)
    return ego, npcs


def _iter_invocations(member: OscDoMember) -> Iterable[OscInvocation]:
    """Yield behaviour invocations from a ``do`` tree in source order."""
    if isinstance(member, OscInvocation):
        yield member
    elif isinstance(member, OscComposition):
        for child in member.members:
            yield from _iter_invocations(child)


def translate_scenario(scenario: OscScenario) -> ScenarioPlan:
    """Translate a single :class:`OscScenario` into a :class:`ScenarioPlan`."""
    ego, npcs = _resolve_actors(scenario)
    plan = ScenarioPlan(name=scenario.name, ego=ego, npcs=npcs)
    known_actors = {ego.name, *(n.name for n in npcs)}

    def resolve(actor: str | None) -> str:
        if actor is None:
            return ego.name
        if actor not in known_actors:
            raise OscTranslationError(
                f"unknown actor {actor!r} (declared actors: {sorted(known_actors)})"
            )
        return actor

    # Scenario-level modifiers (e.g. `timeout(5s)`) apply to the ego.
    for modifier in scenario.modifiers:
        modifier_handler(modifier.name)(
            plan, resolve(modifier.actor), Arguments(modifier.arguments)
        )

    # `do` behaviour invocations and their `with:` modifiers.
    if scenario.do is not None:
        for invocation in _iter_invocations(scenario.do):
            actor = resolve(invocation.actor)
            behavior_handler(invocation.behavior)(
                plan, actor, Arguments(invocation.arguments)
            )
            for modifier in invocation.modifiers:
                modifier_handler(modifier.name)(
                    plan,
                    resolve(modifier.actor) if modifier.actor else actor,
                    Arguments(modifier.arguments),
                )

    return plan


def translate_program(program: OscProgram) -> ScenarioPlan:
    """Translate a program's :attr:`~.OscProgram.main_scenario`.

    Raises:
        ValueError: If the program declares no scenarios.
    """
    return translate_scenario(program.main_scenario)
