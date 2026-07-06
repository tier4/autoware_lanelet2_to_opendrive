"""Render a :class:`~.plan.ScenarioPlan` into readable Python scenario code.

The generated module mirrors the hand-written example scenarios (see
``autoware_carla_scenario/examples/intersection_passing.py``): a
:class:`~autoware_carla_scenario.BaseScenario` subclass whose ``setup`` spawns
actors and registers actions and pass/fail conditions with explicit,
constructor-level module calls.  Readability is the priority — the output is
meant to be committed, reviewed and edited by hand, not treated as an opaque
build artifact.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field

from .plan import (
    DEFAULT_SPEED_CHECK_DELAY_SECONDS,
    ActorPlan,
    ScenarioPlan,
    Spec,
    SpecKind,
    SpecRole,
)

_INDENT = "    "

_TRAFFIC_LIGHT_STATE = {
    "green": "carla.TrafficLightState.Green",
    "red": "carla.TrafficLightState.Red",
    "yellow": "carla.TrafficLightState.Yellow",
}


def _class_name(scenario_name: str) -> str:
    """Turn a DSL scenario name into a ``CamelCaseScenario`` class name."""
    tail = scenario_name.split(".")[-1]
    parts = [p for p in re.split(r"[^0-9A-Za-z]+", tail) if p]
    camel = "".join(p[:1].upper() + p[1:] for p in parts) or "Osc"
    if not camel.endswith("Scenario"):
        camel += "Scenario"
    return camel


def _indent(lines: list[str], level: int) -> list[str]:
    pad = _INDENT * level
    return [f"{pad}{line}" if line else "" for line in lines]


@dataclass
class _Emitter:
    """Accumulates import requirements while rendering spec code."""

    imports: set[str] = field(default_factory=set)

    def role_expr(self, plan: ScenarioPlan, actor_name: str) -> str:
        """Return the CARLA ``role_name`` expression for *actor_name*."""
        actor = plan.actor(actor_name)
        if actor.is_ego:
            self.imports.add("EGO_ROLE_NAME")
            return "EGO_ROLE_NAME"
        self.imports.add("EntityRole")
        return f"EntityRole.npc({actor.index})"

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

    # -- fail conditions ----------------------------------------------

    def fail_condition_lines(self, plan: ScenarioPlan, spec: Spec) -> list[str]:
        if spec.kind is SpecKind.TIMEOUT:
            self.imports.add("TimeoutCondition")
            return [
                "self.register_fail_condition(",
                f"{_INDENT}TimeoutCondition("
                f"{spec.params['seconds']!r}, label=\"{spec.label}\")",
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

    # -- actions ------------------------------------------------------

    def action_lines(self, plan: ScenarioPlan, spec: Spec) -> list[str]:
        if spec.kind is SpecKind.TURN:
            self.imports.update({"TurnAction", "TurnDirection", "TickTiming"})
            role = self.role_expr(plan, spec.actor)
            direction = spec.params["direction"].upper()
            return [
                "self.register_pre_tick(",
                f"{_INDENT}TurnAction(",
                f"{_INDENT * 2}entity_name={role},",
                f"{_INDENT * 2}direction=TurnDirection.{direction},",
                f"{_INDENT * 2}client=self.client,",
                f"{_INDENT * 2}timing=TickTiming.PRE_TICK,",
                f"{_INDENT * 2}tm_port=self.tm_port,",
                f"{_INDENT})",
                ")",
            ]
        if spec.kind is SpecKind.LANE_CHANGE:
            self.imports.update(
                {"LaneChangeAction", "LaneChangeDirection", "TickTiming"}
            )
            role = self.role_expr(plan, spec.actor)
            direction = spec.params["direction"].upper()
            return [
                "self.register_pre_tick(",
                f"{_INDENT}LaneChangeAction(",
                f"{_INDENT * 2}entity_name={role},",
                f"{_INDENT * 2}direction=LaneChangeDirection.{direction},",
                f"{_INDENT * 2}client=self.client,",
                f"{_INDENT * 2}timing=TickTiming.PRE_TICK,",
                f"{_INDENT * 2}tm_port=self.tm_port,",
                f"{_INDENT})",
                ")",
            ]
        if spec.kind is SpecKind.TRAFFIC_SIGNAL:
            self.imports.update({"TrafficSignalAction", "TrafficLightTarget"})
            state = _TRAFFIC_LIGHT_STATE[spec.params["state"]]
            return [
                "TrafficSignalAction(",
                f"{_INDENT}state={state},",
                f"{_INDENT}lanelet2_traffic_light_ids=TrafficLightTarget.ALL,",
                f'{_INDENT}label="{spec.label}",',
                ").execute(self.world)",
            ]
        raise AssertionError(spec.kind)  # pragma: no cover

    # -- npc spawning -------------------------------------------------

    def npc_spawn_lines(self, npc: ActorPlan) -> list[str]:
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
        lanelet = npc.spawn_lanelet_id if npc.spawn_lanelet_id is not None else 0
        return [
            f"{var}_pose = Lanelet2Pose(lanelet_id={lanelet}, s={npc.spawn_s!r})",
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


def _setup_body(plan: ScenarioPlan, emitter: _Emitter) -> list[str]:
    """Build the statements inside ``setup`` (unindented, section by section)."""
    body: list[str] = [
        "# Snap the ego spawn pose to the CARLA road surface.",
        "self._setup_ego_spawn()",
    ]

    if plan.npcs:
        body.append("")
        body.append("# Spawn NPC vehicles.")
        for npc in plan.npcs:
            body.extend(emitter.npc_spawn_lines(npc))
            body.append("")
        body.pop()

    actions = plan.specs_for(SpecRole.ACTION)
    if actions:
        body.append("")
        body.append("# Actions.")
        for spec in actions:
            body.extend(emitter.action_lines(plan, spec))

    pass_specs = plan.specs_for(SpecRole.PASS)
    if pass_specs:
        body.append("")
        body.append("# Pass conditions.")
        exprs = [emitter.pass_condition_expr(plan, s) for s in pass_specs]
        if len(exprs) == 1:
            body.append("self.register_pass_condition(")
            body.extend(_indent(exprs[0], 1))
            body.append(")")
        else:
            emitter.imports.add("AndCondition")
            body.append("self.register_pass_condition(")
            body.append(f"{_INDENT}AndCondition(")
            body.append(f"{_INDENT * 2}[")
            for expr in exprs:
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


def _render_imports(emitter: _Emitter) -> list[str]:
    names = sorted(emitter.imports | {"BaseScenario", "EgoConfig"})
    lines = ["import carla", "", "from autoware_carla_scenario import ("]
    lines.extend(f"{_INDENT}{name}," for name in names)
    lines.append(")")
    return lines


def generate_module(plan: ScenarioPlan, *, source_name: str) -> str:
    """Render *plan* into a complete, importable Python module string.

    Args:
        plan: The semantic scenario plan.
        source_name: The originating ``.osc`` path, recorded in the docstring.

    Returns:
        The generated Python source code.
    """
    emitter = _Emitter()
    class_name = _class_name(plan.name)

    setup_body = _setup_body(plan, emitter)
    import_lines = _render_imports(emitter)

    ego = plan.ego
    spawn_lanelet = ego.spawn_lanelet_id if ego.spawn_lanelet_id is not None else 0
    spawn_comment = (
        ""
        if ego.spawn_lanelet_id is not None
        else "  # NOTE: spawn lanelet not set in DSL"
    )

    lines: list[str] = [
        '"""Scenario transpiled from an OpenSCENARIO DSL source.',
        "",
        f"Source: {source_name}",
        "",
        "Generated by autoware_carla_scenario.openscenario_dsl_frontend.",
        "Edit freely — this file is plain, readable scenario code.",
        '"""',
        "",
        "from __future__ import annotations",
        "",
        *import_lines,
        "",
        "",
        f"class {class_name}(BaseScenario):",
        f'{_INDENT}"""Transpiled scenario {plan.name!r}."""',
        "",
        f"{_INDENT}def setup(self) -> None:",
        *_indent(setup_body, 2),
        "",
        f"{_INDENT}def is_done(self) -> bool:",
        f"{_INDENT * 2}return False",
        "",
        "",
        f"def build_scenario() -> {class_name}:",
        f'{_INDENT}"""Construct the scenario with the ego spawn baked in from the DSL."""',
        f"{_INDENT}ego_config = EgoConfig(",
        f"{_INDENT * 2}spawn_location=SpawnTransform("
        "carla.Transform(carla.Location())),",
        f'{_INDENT * 2}vehicle_type="{ego.vehicle_type}",',
        f"{_INDENT * 2}initial_speed_kmh={ego.initial_speed_kmh!r},",
        f"{_INDENT})",
        f"{_INDENT}spawn_pose = Lanelet2Pose("
        f"lanelet_id={spawn_lanelet}, s={ego.spawn_s!r}){spawn_comment}",
        f"{_INDENT}return {class_name}(ego_config, spawn_pose=spawn_pose)",
        "",
    ]
    # build_scenario relies on these symbols regardless of which specs ran.
    emitter.imports.update({"SpawnTransform", "Lanelet2Pose"})
    # Re-render imports now that build_scenario's needs are known.
    lines[10 : 10 + len(import_lines)] = _render_imports(emitter)

    return "\n".join(lines) + "\n"
