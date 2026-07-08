"""Render the body of a transpiled scenario into readable Python code.

Shared helpers used by :mod:`.package_codegen` to emit one
:class:`~autoware_carla_scenario.BaseScenario` subclass per ``one_of`` variant,
whose ``setup`` spawns actors and registers actions and pass/fail conditions
with explicit, constructor-level module calls.

Serial ordering is preserved by giving each action a ``condition=`` trigger
(``ActionDoneCondition`` / lane / standstill / ``AndCondition``), so a later
step only arms once the previous step has completed. Readability is the
priority — the output is meant to be committed, reviewed and edited by hand.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from typing import Optional

from .plan import (
    DEFAULT_SPEED_CHECK_DELAY_SECONDS,
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

_COMPARISON_RULE = {
    "<": "ComparisonRule.LESS_THAN",
    "<=": "ComparisonRule.LESS_THAN_OR_EQUAL",
    ">": "ComparisonRule.GREATER_THAN",
    ">=": "ComparisonRule.GREATER_THAN_OR_EQUAL",
}

_INDENT = "    "

_TRAFFIC_LIGHT_STATE = {
    "green": "carla.TrafficLightState.Green",
    "red": "carla.TrafficLightState.Red",
    "yellow": "carla.TrafficLightState.Yellow",
}


def _sanitize(name: str) -> str:
    return re.sub(r"\W+", "_", name).strip("_")


def _action_var(label: str) -> str:
    return f"{_sanitize(label)}_action"


def _indent(lines: list[str], level: int) -> list[str]:
    pad = _INDENT * level
    return [f"{pad}{line}" if line else "" for line in lines]


@dataclass
class _Emitter:
    """Accumulates import requirements while rendering spec code."""

    imports: set[str] = field(default_factory=set)
    #: Set when the generated ``setup`` references the ``carla`` module.
    needs_carla: bool = False

    def role_expr(self, plan: ScenarioPlan, actor_name: str) -> str:
        """Return the CARLA ``role_name`` expression for *actor_name*."""
        actor = plan.actor(actor_name)
        if actor.is_ego:
            self.imports.add("EGO_ROLE_NAME")
            return "EGO_ROLE_NAME"
        self.imports.add("EntityRole")
        return f"EntityRole.npc({actor.index})"

    # -- trigger gates -------------------------------------------------

    def gate_expr(self, plan: ScenarioPlan, gate: Gate) -> Optional[str]:
        """Render a :class:`Gate` into a single-line condition expression."""
        if gate.is_immediate:
            return None
        if gate.kind is GateKind.ACTION_DONE:
            self.imports.add("ActionDoneCondition")
            var = _action_var(gate.action_label or "")
            return f'ActionDoneCondition({var}, label="after_{gate.action_label}")'
        if gate.kind is GateKind.LANE_REACHED:
            self.imports.update(
                {"EntityLanePositionCondition", "to_opendrive", "Lanelet2Pose"}
            )
            role = self.role_expr(plan, gate.actor or plan.ego.name)
            return (
                f"EntityLanePositionCondition(entity_name={role}, "
                f"road_id=to_opendrive("
                f"Lanelet2Pose(lanelet_id={gate.lanelet_id}, s=0.0)).road_id, "
                f'label="after_reach_lane_{gate.lanelet_id}")'
            )
        if gate.kind is GateKind.STANDSTILL:
            self.imports.add("StandstillCondition")
            role = self.role_expr(plan, gate.actor or plan.ego.name)
            return (
                f"StandstillCondition(entity_name={role}, "
                f"duration={gate.duration!r}, "
                f'label="after_{gate.actor}_standstill")'
            )
        if gate.kind is GateKind.CONDITION:
            assert gate.condition is not None  # noqa: S101
            return self.condition_spec_expr(plan, gate.condition)
        if gate.kind is GateKind.ALL_OF:
            self.imports.add("AndCondition")
            parts = [self.gate_expr(plan, g) for g in gate.members]
            joined = ", ".join(p for p in parts if p)
            return f"AndCondition([{joined}])"
        raise AssertionError(gate.kind)  # pragma: no cover

    def condition_spec_expr(self, plan: ScenarioPlan, spec: ConditionSpec) -> str:
        """Render a compiled event condition into a one-line expression."""
        if spec.kind is CondKind.ELAPSED:
            self.imports.add("ElapsedTimeCondition")
            return f'ElapsedTimeCondition({spec.value!r}, label="{spec.label}")'
        if spec.kind is CondKind.SPEED:
            self.imports.update({"SpeedCondition", "ComparisonRule"})
            role = self.role_expr(plan, spec.actor or plan.ego.name)
            rule = _COMPARISON_RULE[spec.op or "<"]
            return (
                f"SpeedCondition(entity_name={role}, value={spec.value!r}, "
                f'rule={rule}, label="{spec.label}")'
            )
        if spec.kind is CondKind.LANE:
            self.imports.update(
                {"EntityLanePositionCondition", "to_opendrive", "Lanelet2Pose"}
            )
            role = self.role_expr(plan, spec.actor or plan.ego.name)
            return (
                f"EntityLanePositionCondition(entity_name={role}, "
                f"road_id=to_opendrive("
                f"Lanelet2Pose(lanelet_id={spec.lanelet_id}, s=0.0)).road_id, "
                f'label="{spec.label}")'
            )
        raise AssertionError(spec.kind)  # pragma: no cover

    # -- actions ------------------------------------------------------

    def _action_call(self, plan: ScenarioPlan, spec: Spec) -> tuple[str, list[str]]:
        """Return ``(ctor_name, kwarg_lines)`` for an action spec (no gate)."""
        role = self.role_expr(plan, spec.actor)
        if spec.kind is SpecKind.TURN:
            self.imports.update({"TurnAction", "TurnDirection", "TickTiming"})
            direction = spec.params["direction"].upper()
            return "TurnAction", [
                f"entity_name={role}",
                f"direction=TurnDirection.{direction}",
                "client=self.client",
                "timing=TickTiming.PRE_TICK",
                "tm_port=self.tm_port",
            ]
        if spec.kind is SpecKind.LANE_CHANGE:
            self.imports.update(
                {"LaneChangeAction", "LaneChangeDirection", "TickTiming"}
            )
            direction = spec.params["direction"].upper()
            return "LaneChangeAction", [
                f"entity_name={role}",
                f"direction=LaneChangeDirection.{direction}",
                "client=self.client",
                "timing=TickTiming.PRE_TICK",
                "tm_port=self.tm_port",
            ]
        if spec.kind is SpecKind.RELATIVE_POSITION:
            self.imports.update({"RelativePositionAction", "TickTiming"})
            reference = self.role_expr(plan, spec.params["reference"])
            return "RelativePositionAction", [
                f"entity_name={role}",
                f"reference_name={reference}",
                f"target_gap={spec.params['target_gap']!r}",
                "client=self.client",
                "timing=TickTiming.PRE_TICK",
                "tm_port=self.tm_port",
                "once=False",
            ]
        if spec.kind is SpecKind.TRAFFIC_SIGNAL:
            self.imports.update({"TrafficSignalAction", "TrafficLightTarget"})
            self.needs_carla = True
            state = _TRAFFIC_LIGHT_STATE[spec.params["state"]]
            return "TrafficSignalAction", [
                f"state={state}",
                "lanelet2_traffic_light_ids=TrafficLightTarget.ALL",
                f'label="{spec.label}"',
            ]
        raise AssertionError(spec.kind)  # pragma: no cover

    def action_lines(self, plan: ScenarioPlan, spec: Spec, *, named: bool) -> list[str]:
        """Render a full action registration statement (with its gate)."""
        ctor, kwargs = self._action_call(plan, spec)
        gate = self.gate_expr(plan, spec.gate)
        if gate is not None:
            kwargs = [*kwargs, f"condition={gate}"]

        call = [f"{ctor}("]
        call += [f"{_INDENT}{kw}," for kw in kwargs]
        call += [")"]

        if named:
            var = _action_var(spec.label)
            return [
                f"{var} = {call[0]}",
                *call[1:],
                f"self.register_pre_tick({var})",
            ]
        return ["self.register_pre_tick(", *_indent(call, 1), ")"]

    # -- pass conditions ----------------------------------------------

    def reach_lane_expr(self, plan: ScenarioPlan, spec: Spec) -> list[str]:
        self.imports.update(
            {
                "StickyCondition",
                "EntityLanePositionCondition",
                "to_opendrive",
                "Lanelet2Pose",
            }
        )
        role = self.role_expr(plan, spec.actor)
        lanelet = spec.params["lanelet_id"]
        return [
            "StickyCondition(",
            f"{_INDENT}EntityLanePositionCondition(",
            f"{_INDENT * 2}entity_name={role},",
            f"{_INDENT * 2}road_id=to_opendrive("
            f"Lanelet2Pose(lanelet_id={lanelet}, s=0.0)).road_id,",
            f'{_INDENT * 2}label="{spec.label}",',
            f"{_INDENT})",
            ")",
        ]

    def standstill_expr(self, plan: ScenarioPlan, spec: Spec) -> list[str]:
        self.imports.add("StandstillCondition")
        role = self.role_expr(plan, spec.actor)
        return [
            "StandstillCondition(",
            f"{_INDENT}entity_name={role},",
            f"{_INDENT}duration={spec.params['duration']!r},",
            f'{_INDENT}label="{spec.label}",',
            ")",
        ]

    def pass_condition_expr(self, plan: ScenarioPlan, spec: Spec) -> list[str]:
        if spec.kind is SpecKind.REACH_LANE:
            return self.reach_lane_expr(plan, spec)
        if spec.kind is SpecKind.STANDSTILL:
            return self.standstill_expr(plan, spec)
        raise AssertionError(spec.kind)  # pragma: no cover

    def sequential_expr(self, plan: ScenarioPlan, specs: list[Spec]) -> list[str]:
        """Render an ordered pass sequence as a ``SequentialCondition``."""
        self.imports.add("SequentialCondition")
        lines = ["SequentialCondition(", f"{_INDENT}["]
        for spec in specs:
            child = _indent(self.pass_condition_expr(plan, spec), 2)
            child[-1] = child[-1] + ","
            lines.extend(child)
        lines += [f"{_INDENT}],", f'{_INDENT}label="ordered_pass",', ")"]
        return lines

    # -- fail conditions ----------------------------------------------

    def fail_condition_lines(self, plan: ScenarioPlan, spec: Spec) -> list[str]:
        if spec.kind is SpecKind.TIMEOUT:
            self.imports.add("TimeoutCondition")
            return [
                "self.register_fail_condition(",
                f"{_INDENT}TimeoutCondition("
                'self._config.timeout_seconds, label="scenario_timeout")',
                ")",
            ]
        if spec.kind is SpecKind.COLLISION:
            self.imports.add("CollisionCondition")
            return [
                "self.register_fail_condition(",
                f'{_INDENT}CollisionCondition(label="{spec.label}")',
                ")",
            ]
        if spec.kind is SpecKind.MIN_SPEED:
            self.imports.update(
                {
                    "AndCondition",
                    "ElapsedTimeCondition",
                    "SpeedCondition",
                    "ComparisonRule",
                }
            )
            role = self.role_expr(plan, spec.actor)
            speed_kmh = spec.params["min_speed_kmh"]
            delay = DEFAULT_SPEED_CHECK_DELAY_SECONDS
            return [
                "self.register_fail_condition(",
                f"{_INDENT}AndCondition(",
                f"{_INDENT * 2}[",
                f"{_INDENT * 3}ElapsedTimeCondition("
                f'{delay!r}, label="{spec.label}_delay"),',
                f"{_INDENT * 3}SpeedCondition(",
                f"{_INDENT * 4}entity_name={role},",
                f"{_INDENT * 4}value={speed_kmh!r} / 3.6,",
                f"{_INDENT * 4}rule=ComparisonRule.LESS_THAN,",
                f'{_INDENT * 4}label="{spec.label}",',
                f"{_INDENT * 3}),",
                f"{_INDENT * 2}]",
                f"{_INDENT})",
                ")",
            ]
        raise AssertionError(spec.kind)  # pragma: no cover

    # -- npc spawning -------------------------------------------------

    def npc_pose_lines(self, plan: ScenarioPlan, npc: ActorPlan, var: str) -> list[str]:
        """Render the ``<var>_pose = Lanelet2Pose(...)`` statement for an NPC.

        When the NPC is placed a longitudinal distance relative to the *ego*,
        the pose is derived from the ego's spawn pose (``self._spawn_pose``) at
        runtime — no map or transpile-time position needed.
        """
        rel = npc.relative_spawn
        if rel is not None and rel.anchor == "start" and rel.reference == plan.ego.name:
            op = "+" if rel.longitudinal >= 0 else "-"
            return [
                f"{var}_pose = Lanelet2Pose(",
                f"{_INDENT}lanelet_id=self._spawn_pose.lanelet_id,",
                f"{_INDENT}s=max(0.0, self._spawn_pose.s {op} {abs(rel.longitudinal)!r}),",
                ")",
            ]
        lanelet = npc.spawn_lanelet_id if npc.spawn_lanelet_id is not None else 0
        return [f"{var}_pose = Lanelet2Pose(lanelet_id={lanelet}, s={npc.spawn_s!r})"]

    def npc_spawn_lines(self, plan: ScenarioPlan, npc: ActorPlan) -> list[str]:
        self.imports.update(
            {
                "Lanelet2Pose",
                "to_opendrive",
                "snap_to_carla_road",
                "VehicleEntity",
                "VehicleEntityConfig",
                "EntityRole",
                "SpawnTransform",
            }
        )
        var = f"npc_{npc.index}"
        return [
            *self.npc_pose_lines(plan, npc, var),
            f"{var}_od_pose = to_opendrive({var}_pose)",
            f"{var}_snapped = snap_to_carla_road(",
            f"{_INDENT}{var}_od_pose, self.world, "
            "ground_projection=self._ground_projection,",
            ")",
            f"{var} = VehicleEntity(",
            f"{_INDENT}VehicleEntityConfig(",
            f"{_INDENT * 2}role_name=EntityRole.npc({npc.index}),",
            f"{_INDENT * 2}spawn_location=SpawnTransform("
            f"{var}_snapped.to_carla_transform()),",
            f'{_INDENT * 2}vehicle_type="{npc.vehicle_type}",',
            f"{_INDENT * 2}initial_speed_kmh={npc.initial_speed_kmh!r},",
            f"{_INDENT * 2}od_pose={var}_od_pose,",
            f"{_INDENT * 2}ground_projection=self._ground_projection,",
            f"{_INDENT})",
            ")",
            f"{var}.spawn(self.world)",
            f"self.register_entity({var})",
        ]


def _referenced_action_labels(plan: ScenarioPlan) -> set[str]:
    """Return labels of actions referenced by an ``ActionDoneCondition`` gate."""
    refs: set[str] = set()

    def scan(gate: Gate) -> None:
        if gate.kind is GateKind.ACTION_DONE and gate.action_label:
            refs.add(gate.action_label)
        for member in gate.members:
            scan(member)

    for spec in plan.specs_for(SpecRole.ACTION):
        scan(spec.gate)
    return refs


def _wrap_note(note: str) -> list[str]:
    """Wrap a transpiler note into ``# NOTE:`` comment lines (<= 88 cols)."""
    words = f"NOTE: {note}".split()
    lines: list[str] = []
    current = "#"
    for word in words:
        candidate = f"{current} {word}"
        if len(candidate) > 88 and current != "#":
            lines.append(current)
            current = f"#   {word}"
        else:
            current = candidate
    lines.append(current)
    return lines


def _setup_body(plan: ScenarioPlan, emitter: _Emitter) -> list[str]:
    """Build the statements inside ``setup`` (unindented, section by section)."""
    body: list[str] = []
    if plan.map_hint is not None:
        body.extend(
            _wrap_note(
                f"the source targeted map {plan.map_hint!r}; the map is chosen "
                "at run time (map=...), so this is advisory only."
            )
        )
    for note in plan.notes:
        body.extend(_wrap_note(note))
    if body:
        body.append("")
    body += [
        "# Snap the ego spawn pose to the CARLA road surface.",
        "self._setup_ego_spawn()",
    ]

    if plan.npcs:
        body.append("")
        body.append("# Spawn NPC vehicles.")
        for npc in plan.npcs:
            body.extend(emitter.npc_spawn_lines(plan, npc))
            body.append("")
        body.pop()

    actions = plan.specs_for(SpecRole.ACTION)
    if actions:
        referenced = _referenced_action_labels(plan)
        body.append("")
        body.append("# Actions (serial order preserved via trigger conditions).")
        for spec in actions:
            body.extend(
                emitter.action_lines(plan, spec, named=spec.label in referenced)
            )

    pass_specs = plan.specs_for(SpecRole.PASS)
    if pass_specs:
        body.append("")
        body.append("# Pass conditions.")
        ordered = [s for s in pass_specs if s.ordered]
        unordered = [s for s in pass_specs if not s.ordered]

        group_exprs: list[list[str]] = []
        if len(ordered) >= 2:
            # A serial run of pass steps must be satisfied in sequence.
            group_exprs.append(emitter.sequential_expr(plan, ordered))
        elif len(ordered) == 1:
            # A lone ordered step carries no sequence; treat it as a plain one.
            unordered = [ordered[0], *unordered]
        group_exprs.extend(emitter.pass_condition_expr(plan, s) for s in unordered)

        if len(group_exprs) == 1:
            body.append("self.register_pass_condition(")
            body.extend(_indent(group_exprs[0], 1))
            body.append(")")
        else:
            emitter.imports.add("AndCondition")
            body.append("self.register_pass_condition(")
            body.append(f"{_INDENT}AndCondition(")
            body.append(f"{_INDENT * 2}[")
            for expr in group_exprs:
                lines = _indent(expr, 3)
                lines[-1] = lines[-1] + ","
                body.extend(lines)
            body.append(f"{_INDENT * 2}]")
            body.append(f"{_INDENT})")
            body.append(")")

    fail_specs = plan.specs_for(SpecRole.FAIL)
    if fail_specs:
        body.append("")
        body.append("# Fail conditions.")
        for spec in fail_specs:
            body.extend(emitter.fail_condition_lines(plan, spec))

    return body
