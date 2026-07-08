"""Unit tests for the OpenSCENARIO DSL frontend.

The translator/codegen/values layers are pure Python and exercised directly
from a hand-built IR, so they run in CI without ``py-osc2`` (which cannot be a
project dependency — it pins an incompatible ANTLR runtime). The parser layer
is additionally tested wherever ``py-osc2`` happens to be installed.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

from autoware_carla_scenario.openscenario_dsl_frontend import (
    OscParseError,
    OscTranslationError,
    SpecKind,
    parse_program_from_string,
    transpile_to_package,
)
from autoware_carla_scenario.openscenario_dsl_frontend.package_codegen import (
    generate_package_files,
)
from autoware_carla_scenario.openscenario_dsl_frontend import (
    __file__ as _frontend_file,
)
from autoware_carla_scenario.openscenario_dsl_frontend.ast_model import (
    OscArgument,
    OscComparison,
    OscComposition,
    OscElapsed,
    OscField,
    OscFieldAccess,
    OscInvocation,
    OscModifier,
    OscProgram,
    OscScenario,
    OscWait,
)
from autoware_carla_scenario.openscenario_dsl_frontend.plan import CondKind, GateKind
from autoware_carla_scenario.openscenario_dsl_frontend.translator import (
    translate_program,
)
from autoware_carla_scenario.openscenario_dsl_frontend.values import (
    FloatValue,
    IntValue,
    PhysicalValue,
    StringValue,
    SymbolValue,
    parse_physical_literal,
)

_HAS_OSC2 = importlib.util.find_spec("osc2parser") is not None
_requires_osc2 = pytest.mark.skipif(
    not _HAS_OSC2, reason="py-osc2 is not installed (isolated-env dependency)"
)

_PACKAGE_ROOT = Path(_frontend_file).resolve().parent.parent
_EXAMPLE_OSC = _PACKAGE_ROOT / "examples" / "openscenario" / "intersection_passing.osc"


def _demo_program() -> OscProgram:
    """Build a rich, serial scenario IR directly (no parser required)."""
    scenario = OscScenario(
        name="demo",
        fields=[
            OscField(name="ego", type_name="vehicle"),
            OscField(name="npc", type_name="vehicle"),
        ],
        modifiers=[
            OscModifier(
                name="timeout",
                actor=None,
                arguments=[OscArgument(None, PhysicalValue(8.0, "s"))],
            )
        ],
        do=OscComposition(
            operator="serial",
            members=[
                OscInvocation(
                    behavior="drive",
                    actor="ego",
                    modifiers=[
                        OscModifier(
                            "speed",
                            "ego",
                            [OscArgument(None, PhysicalValue(30.0, "kmph"))],
                        ),
                        OscModifier(
                            "spawn_lanelet", "ego", [OscArgument(None, IntValue(242))]
                        ),
                        OscModifier(
                            "spawn_s", "ego", [OscArgument(None, FloatValue(5.0))]
                        ),
                    ],
                ),
                OscInvocation(
                    behavior="set_traffic_lights",
                    actor=None,
                    arguments=[OscArgument("state", SymbolValue("green"))],
                ),
                OscInvocation(
                    behavior="turn",
                    actor="ego",
                    arguments=[OscArgument("direction", SymbolValue("left"))],
                ),
                OscInvocation(
                    behavior="reach_lane",
                    actor="ego",
                    arguments=[OscArgument("lanelet", IntValue(460))],
                ),
                OscInvocation(
                    behavior="change_lane",
                    actor="ego",
                    arguments=[OscArgument("direction", SymbolValue("right"))],
                ),
                OscInvocation(
                    behavior="keep_speed_above",
                    actor="ego",
                    arguments=[OscArgument(None, PhysicalValue(10.0, "kmph"))],
                ),
                OscInvocation(behavior="no_collision", actor="ego"),
            ],
        ),
    )
    return OscProgram(scenarios=[scenario])


def _one_of_program() -> OscProgram:
    """A serial scenario with a single ``one_of`` branch of two turns."""
    scenario = OscScenario(
        name="choice",
        fields=[OscField(name="ego", type_name="vehicle")],
        do=OscComposition(
            operator="serial",
            members=[
                OscComposition(
                    operator="one_of",
                    members=[
                        OscInvocation(
                            behavior="turn",
                            actor="ego",
                            arguments=[OscArgument("direction", SymbolValue("left"))],
                        ),
                        OscInvocation(
                            behavior="turn",
                            actor="ego",
                            arguments=[OscArgument("direction", SymbolValue("right"))],
                        ),
                    ],
                ),
                OscInvocation(
                    behavior="reach_lane",
                    actor="ego",
                    arguments=[OscArgument("lanelet", IntValue(460))],
                ),
            ],
        ),
    )
    return OscProgram(scenarios=[scenario])


# ---------------------------------------------------------------------------
# values (pure)
# ---------------------------------------------------------------------------


def test_parse_physical_literal_speed() -> None:
    value = parse_physical_literal("30kmph")
    assert value == PhysicalValue(30.0, "kmph")
    assert value.to_kmh() == pytest.approx(30.0)


def test_physical_literal_unit_conversions() -> None:
    assert parse_physical_literal("10mps").to_kmh() == pytest.approx(36.0)
    assert parse_physical_literal("5s").to_seconds() == pytest.approx(5.0)
    assert parse_physical_literal("500ms").to_seconds() == pytest.approx(0.5)


def test_physical_literal_wrong_unit_raises() -> None:
    with pytest.raises(OscTranslationError):
        parse_physical_literal("5s").to_kmh()


# ---------------------------------------------------------------------------
# translator (pure, no py-osc2)
# ---------------------------------------------------------------------------


def test_translate_resolves_ego_and_npc() -> None:
    (plan,) = translate_program(_demo_program())
    assert plan.ego.name == "ego"
    assert plan.ego.spawn_lanelet_id == 242
    assert plan.ego.spawn_s == pytest.approx(5.0)
    assert plan.ego.initial_speed_kmh == pytest.approx(30.0)
    assert [npc.name for npc in plan.npcs] == ["npc"]
    assert plan.npcs[0].index == 1


def test_translate_produces_expected_specs() -> None:
    (plan,) = translate_program(_demo_program())
    kinds = {spec.kind for spec in plan.specs}
    assert {
        SpecKind.TURN,
        SpecKind.LANE_CHANGE,
        SpecKind.TRAFFIC_SIGNAL,
        SpecKind.REACH_LANE,
        SpecKind.MIN_SPEED,
        SpecKind.COLLISION,
        SpecKind.TIMEOUT,
    } <= kinds


def test_serial_ordering_produces_gates() -> None:
    (plan,) = translate_program(_demo_program())
    by_kind = {s.kind: s for s in plan.specs}

    # turn is gated on the preceding traffic-signal action having fired.
    turn = by_kind[SpecKind.TURN]
    assert turn.gate.kind is GateKind.ACTION_DONE
    assert turn.gate.action_label == by_kind[SpecKind.TRAFFIC_SIGNAL].label

    # the lane change waits until the ego has reached lanelet 460.
    lane_change = by_kind[SpecKind.LANE_CHANGE]
    assert lane_change.gate.kind is GateKind.LANE_REACHED
    assert lane_change.gate.lanelet_id == 460

    # the first action (traffic signal) fires immediately.
    assert by_kind[SpecKind.TRAFFIC_SIGNAL].gate.kind is GateKind.IMMEDIATE


def test_unknown_behavior_raises() -> None:
    program = OscProgram(
        scenarios=[
            OscScenario(
                name="x",
                fields=[OscField(name="ego", type_name="vehicle")],
                do=OscInvocation(behavior="teleport", actor="ego"),
            )
        ]
    )
    with pytest.raises(OscTranslationError):
        translate_program(program)


def test_unknown_actor_raises() -> None:
    program = OscProgram(
        scenarios=[
            OscScenario(
                name="x",
                fields=[OscField(name="ego", type_name="vehicle")],
                do=OscInvocation(
                    behavior="turn",
                    actor="ghost",
                    arguments=[OscArgument("direction", SymbolValue("left"))],
                ),
            )
        ]
    )
    with pytest.raises(OscTranslationError):
        translate_program(program)


def test_no_ego_field_synthesises_ego() -> None:
    program = OscProgram(scenarios=[OscScenario(name="empty")])
    (plan,) = translate_program(program)
    assert plan.ego.is_ego
    assert plan.ego.name == "ego"


def _drive_with(modifiers: list[OscModifier]) -> OscProgram:
    return OscProgram(
        scenarios=[
            OscScenario(
                name="p",
                fields=[OscField("ego", "vehicle")],
                do=OscInvocation("drive", "ego", modifiers=modifiers),
            )
        ]
    )


def test_lanelet_position_sets_spawn() -> None:
    program = _drive_with(
        [
            OscModifier(
                "lanelet_position",
                "ego",
                [
                    OscArgument("lanelet", IntValue(242)),
                    OscArgument("s", FloatValue(25.0)),
                ],
            )
        ]
    )
    (plan,) = translate_program(program)
    assert plan.ego.spawn_lanelet_id == 242
    assert plan.ego.spawn_s == pytest.approx(25.0)


def test_lanelet_position_rejects_lateral_offset() -> None:
    program = _drive_with(
        [
            OscModifier(
                "lanelet_position",
                "ego",
                [
                    OscArgument("lanelet", IntValue(1)),
                    OscArgument("offset", FloatValue(0.5)),
                ],
            )
        ]
    )
    with pytest.raises(OscTranslationError):
        translate_program(program)


# ---------------------------------------------------------------------------
# one_of expansion (pure)
# ---------------------------------------------------------------------------


def test_one_of_expands_into_variants() -> None:
    plans = translate_program(_one_of_program())
    assert len(plans) == 2
    directions = {
        spec.params["direction"]
        for plan in plans
        for spec in plan.specs
        if spec.kind is SpecKind.TURN
    }
    assert directions == {"left", "right"}
    assert [p.variant_index for p in plans] == [0, 1]
    assert all(p.variant_count == 2 for p in plans)


def test_nested_one_of_is_cartesian() -> None:
    program = OscProgram(
        scenarios=[
            OscScenario(
                name="m",
                fields=[OscField(name="ego", type_name="vehicle")],
                do=OscComposition(
                    operator="serial",
                    members=[
                        OscComposition(
                            operator="one_of",
                            members=[
                                OscInvocation(
                                    "turn",
                                    "ego",
                                    [OscArgument("direction", SymbolValue("left"))],
                                ),
                                OscInvocation(
                                    "turn",
                                    "ego",
                                    [OscArgument("direction", SymbolValue("right"))],
                                ),
                            ],
                        ),
                        OscComposition(
                            operator="one_of",
                            members=[
                                OscInvocation(
                                    "change_lane",
                                    "ego",
                                    [OscArgument("direction", SymbolValue("left"))],
                                ),
                                OscInvocation(
                                    "change_lane",
                                    "ego",
                                    [OscArgument("direction", SymbolValue("right"))],
                                ),
                            ],
                        ),
                    ],
                ),
            )
        ]
    )
    plans = translate_program(program)
    assert len(plans) == 4


# ---------------------------------------------------------------------------
# codegen (pure, no py-osc2)
# ---------------------------------------------------------------------------


def test_generated_code_is_valid_python() -> None:
    code = _module_from_plans(translate_program(_demo_program()))
    compile(code, "generated_demo.py", "exec")
    assert "class DemoScenario(BaseScenario):" in code
    assert "def __init__(" in code  # framework builder constructor
    assert "self._setup_ego_spawn()" in code


def test_generated_code_contains_gated_module_calls() -> None:
    code = _module_from_plans(translate_program(_demo_program()))
    assert "TurnAction(" in code
    assert "TurnDirection.LEFT" in code
    assert "TrafficSignalAction(" in code
    # turn is triggered by the traffic-signal action having fired.
    assert "ActionDoneCondition(set_traffic_lights_green_action" in code
    # lane change is triggered by reaching lanelet 460.
    assert "condition=EntityLanePositionCondition(" in code
    # timeout is config-driven so it stays overridable via the YAML config.
    assert "TimeoutCondition(self._config.timeout_seconds" in code
    assert "Lanelet2Pose(lanelet_id=460" in code


def test_generated_one_of_has_multiple_classes() -> None:
    code = _module_from_plans(translate_program(_one_of_program()))
    compile(code, "generated_choice.py", "exec")
    assert "class ChoiceScenarioV0(BaseScenario):" in code
    assert "class ChoiceScenarioV1(BaseScenario):" in code


def _two_reach_lanes(operator: str) -> OscProgram:
    return OscProgram(
        scenarios=[
            OscScenario(
                name="x",
                fields=[OscField(name="ego", type_name="vehicle")],
                do=OscComposition(
                    operator=operator,
                    members=[
                        OscInvocation(
                            behavior="reach_lane",
                            actor="ego",
                            arguments=[OscArgument("lanelet", IntValue(1))],
                        ),
                        OscInvocation(
                            behavior="reach_lane",
                            actor="ego",
                            arguments=[OscArgument("lanelet", IntValue(2))],
                        ),
                    ],
                ),
            )
        ]
    )


def test_serial_pass_conditions_are_ordered() -> None:
    (plan,) = translate_program(_two_reach_lanes("serial"))
    assert all(s.ordered for s in plan.specs if s.kind is SpecKind.REACH_LANE)
    code = _module_from_plans([plan])
    assert "SequentialCondition(" in code
    assert "AndCondition(" not in code
    compile(code, "generated_seq.py", "exec")


def test_parallel_pass_conditions_are_unordered() -> None:
    (plan,) = translate_program(_two_reach_lanes("parallel"))
    assert all(not s.ordered for s in plan.specs if s.kind is SpecKind.REACH_LANE)
    code = _module_from_plans([plan])
    assert "SequentialCondition(" not in code
    assert "AndCondition(" in code
    compile(code, "generated_par.py", "exec")


# ---------------------------------------------------------------------------
# scenario package generation (pure; needs jinja2 but not py-osc2)
# ---------------------------------------------------------------------------


def _scenario_module(files: dict[str, str]) -> str:
    key = next(
        k
        for k in files
        if k.startswith("src/")
        and k.endswith(".py")
        and not k.endswith("__init__.py")
        and not k.endswith("configs.py")
    )
    return files[key]


def _module_from_plans(plans: list) -> str:
    """Render the package's scenario module for a set of plans."""
    return _scenario_module(generate_package_files(plans, source_name="t.osc"))


def test_generate_package_files_structure() -> None:
    plans = translate_program(_one_of_program())  # 2 variants
    files = generate_package_files(plans, source_name="choice.osc")

    assert "pyproject.toml" in files
    assert "README.md" in files
    assert any(k.endswith("/__init__.py") for k in files)
    assert any(k.endswith("/configs.py") for k in files)
    # One concrete Hydra config per one_of variant.
    yamls = [k for k in files if k.endswith("default.yaml")]
    assert len(yamls) == 2

    # The package plugs into the framework via the entry point.
    assert 'entry-points."autoware_carla_scenario.scenarios"' in files["pyproject.toml"]
    init = next(files[k] for k in files if k.endswith("/__init__.py"))
    assert init.count("register_scenario(") == 2
    assert "register_conf_dir(CONF_DIR)" in init

    module = _scenario_module(files)
    assert "self._config.timeout_seconds" in module
    assert "def __init__(" in module  # framework builder constructor
    compile(module, "pkg_scenario.py", "exec")


def test_generate_package_files_single_variant() -> None:
    program = OscProgram(
        scenarios=[
            OscScenario(
                name="reach",
                fields=[OscField("ego", "vehicle")],
                do=OscInvocation(
                    "reach_lane",
                    "ego",
                    [OscArgument("lanelet", IntValue(460))],
                ),
                modifiers=[
                    OscModifier("timeout", None, [OscArgument(None, IntValue(9))])
                ],
            )
        ]
    )
    files = generate_package_files(translate_program(program), source_name="r.osc")
    yamls = [k for k in files if k.endswith("default.yaml")]
    assert len(yamls) == 1
    assert "timeout_seconds: float = 9.0" in files["src/reach_package/configs.py"]


# ---------------------------------------------------------------------------
# wait / until + expression compiler (pure)
# ---------------------------------------------------------------------------


def _speed_lt(kmph: float) -> OscComparison:
    return OscComparison(
        lhs=OscFieldAccess("ego", "speed"), op="<", rhs=PhysicalValue(kmph, "kmph")
    )


def test_wait_elapsed_gates_next_action() -> None:
    program = OscProgram(
        scenarios=[
            OscScenario(
                name="w",
                fields=[OscField("ego", "vehicle")],
                do=OscComposition(
                    operator="serial",
                    members=[
                        OscWait(condition=OscElapsed(PhysicalValue(3.0, "s"))),
                        OscInvocation(
                            "turn",
                            "ego",
                            [OscArgument("direction", SymbolValue("left"))],
                        ),
                    ],
                ),
            )
        ]
    )
    (plan,) = translate_program(program)
    turn = next(s for s in plan.specs if s.kind is SpecKind.TURN)
    assert turn.gate.kind is GateKind.CONDITION
    assert turn.gate.condition is not None
    assert turn.gate.condition.kind is CondKind.ELAPSED
    assert turn.gate.condition.value == pytest.approx(3.0)


def test_wait_comparison_compiles_to_speed_condition() -> None:
    program = OscProgram(
        scenarios=[
            OscScenario(
                name="w",
                fields=[OscField("ego", "vehicle")],
                do=OscComposition(
                    operator="serial",
                    members=[
                        OscWait(condition=_speed_lt(5.0)),
                        OscInvocation(
                            "turn",
                            "ego",
                            [OscArgument("direction", SymbolValue("left"))],
                        ),
                    ],
                ),
            )
        ]
    )
    (plan,) = translate_program(program)
    turn = next(s for s in plan.specs if s.kind is SpecKind.TURN)
    assert turn.gate.condition is not None
    assert turn.gate.condition.kind is CondKind.SPEED
    assert turn.gate.condition.value == pytest.approx(5.0 / 3.6)
    code = _module_from_plans([plan])
    assert "SpeedCondition(" in code
    assert "ComparisonRule.LESS_THAN" in code
    compile(code, "gen_wait.py", "exec")


def test_until_overrides_completion_with_lane_condition() -> None:
    program = OscProgram(
        scenarios=[
            OscScenario(
                name="u",
                fields=[OscField("ego", "vehicle")],
                do=OscComposition(
                    operator="serial",
                    members=[
                        OscInvocation(
                            "drive",
                            "ego",
                            until=OscComparison(
                                lhs=OscFieldAccess("ego", "lane"),
                                op="==",
                                rhs=IntValue(460),
                            ),
                        ),
                        OscInvocation(
                            "turn",
                            "ego",
                            [OscArgument("direction", SymbolValue("left"))],
                        ),
                    ],
                ),
            )
        ]
    )
    (plan,) = translate_program(program)
    turn = next(s for s in plan.specs if s.kind is SpecKind.TURN)
    assert turn.gate.kind is GateKind.CONDITION
    assert turn.gate.condition is not None
    assert turn.gate.condition.kind is CondKind.LANE
    assert turn.gate.condition.lanelet_id == 460


def test_unsupported_attribute_raises() -> None:
    program = OscProgram(
        scenarios=[
            OscScenario(
                name="bad",
                fields=[OscField("ego", "vehicle")],
                do=OscWait(
                    condition=OscComparison(
                        lhs=OscFieldAccess("ego", "altitude"),
                        op="<",
                        rhs=IntValue(3),
                    )
                ),
            )
        ]
    )
    with pytest.raises(OscTranslationError):
        translate_program(program)


# ---------------------------------------------------------------------------
# parser (requires py-osc2)
# ---------------------------------------------------------------------------


@_requires_osc2
def test_parse_extract_matches_manual_ir() -> None:
    source = (
        "scenario demo:\n"
        "    ego: vehicle\n"
        "    npc: vehicle\n"
        "    do serial:\n"
        "        ego.turn(direction: left)\n"
        "    timeout(8s)\n"
    )
    program = parse_program_from_string(source)
    scenario = program.main_scenario
    assert scenario.name == "demo"
    assert [f.name for f in scenario.fields] == ["ego", "npc"]
    assert scenario.do is not None
    assert any(m.name == "timeout" for m in scenario.modifiers)


@_requires_osc2
def test_parse_one_of_transpiles_to_variants() -> None:
    source = (
        "scenario junction:\n"
        "    ego: vehicle\n"
        "    do one_of:\n"
        "        ego.turn(direction: left)\n"
        "        ego.turn(direction: right)\n"
    )
    program = parse_program_from_string(source)
    plans = translate_program(program)
    assert len(plans) == 2


@_requires_osc2
def test_parse_wait_and_until_transpile() -> None:
    source = (
        "scenario reactive:\n"
        "    ego: vehicle\n"
        "    do serial:\n"
        "        ego.drive() with:\n"
        "            until(ego.lane == 460)\n"
        "        wait elapsed(3s)\n"
        "        ego.turn(direction: left)\n"
    )
    program = parse_program_from_string(source)
    plans = translate_program(program)
    generated = _module_from_plans(plans)
    compile(generated, "reactive.py", "exec")
    assert "ElapsedTimeCondition(3.0" in generated
    assert "EntityLanePositionCondition(" in generated


@_requires_osc2
def test_parse_error_raises() -> None:
    with pytest.raises(OscParseError) as exc_info:
        parse_program_from_string("scenario broken\n  ego vehicle\n")
    assert exc_info.value.errors


@_requires_osc2
def test_transpile_to_package_writes_installable_package(
    tmp_path: Path,
) -> None:
    root = transpile_to_package(_EXAMPLE_OSC, output_dir=tmp_path)
    assert (root / "pyproject.toml").is_file()
    assert (root / "README.md").is_file()
    modules = list((root / "src").rglob("*.py"))
    assert modules
    for module in modules:
        compile(module.read_text(encoding="utf-8"), str(module), "exec")
    yamls = list((root / "src").rglob("default.yaml"))
    assert yamls


# ---------------------------------------------------------------------------
# Multi-actor, timed phases, and relational placement (scenario_runner dialect)
# ---------------------------------------------------------------------------


def test_catalog_vehicle_types_resolve_actors() -> None:
    """Fields typed with catalog names are actors; ``Path`` structs are not."""
    program = OscProgram(
        scenarios=[
            OscScenario(
                name="top",
                fields=[
                    OscField("path", "Path"),
                    OscField("ego_vehicle", "Model3"),
                    OscField("npc", "Rubicon"),
                ],
                do=OscComposition(
                    operator="parallel",
                    members=[
                        OscInvocation("drive", "ego_vehicle"),
                        OscInvocation("drive", "npc"),
                    ],
                ),
            )
        ]
    )
    (plan,) = translate_program(program)
    assert plan.ego.name == "ego_vehicle"
    assert [n.name for n in plan.npcs] == ["npc"]


def _timed_phase(label: str, seconds: float, member: OscInvocation) -> OscComposition:
    return OscComposition(
        operator="parallel",
        members=[member],
        label=label,
        duration=PhysicalValue(seconds, "s"),
    )


def test_timed_phases_gate_actions_by_elapsed_time() -> None:
    program = OscProgram(
        scenarios=[
            OscScenario(
                name="top",
                fields=[OscField("ego", "vehicle"), OscField("npc", "vehicle")],
                do=OscComposition(
                    operator="serial",
                    members=[
                        _timed_phase("get_ahead", 15.0, OscInvocation("drive", "ego")),
                        _timed_phase(
                            "change",
                            5.0,
                            OscInvocation(
                                "drive",
                                "npc",
                                modifiers=[
                                    OscModifier(
                                        "change_lane",
                                        None,
                                        [OscArgument("side", SymbolValue("left"))],
                                    )
                                ],
                            ),
                        ),
                    ],
                ),
            )
        ]
    )
    (plan,) = translate_program(program)
    lane_change = next(s for s in plan.specs if s.kind is SpecKind.LANE_CHANGE)
    assert lane_change.actor == "npc"
    assert lane_change.gate.kind is GateKind.CONDITION
    assert lane_change.gate.condition is not None
    assert lane_change.gate.condition.kind is CondKind.ELAPSED
    assert lane_change.gate.condition.value == pytest.approx(15.0)
    # No explicit timeout(): a fail-safe is derived from the total phase time.
    timeout = next(s for s in plan.specs if s.kind is SpecKind.TIMEOUT)
    assert timeout.params["seconds"] == pytest.approx(20.0)


def test_relational_position_and_lane_set_relative_spawn() -> None:
    program = OscProgram(
        scenarios=[
            OscScenario(
                name="top",
                fields=[OscField("ego", "vehicle"), OscField("npc", "vehicle")],
                modifiers=[
                    OscModifier(
                        "spawn_lanelet", "ego", [OscArgument("lanelet", IntValue(100))]
                    ),
                    OscModifier("spawn_s", "ego", [OscArgument("s", FloatValue(40.0))]),
                ],
                do=OscInvocation(
                    "drive",
                    "npc",
                    modifiers=[
                        OscModifier(
                            "lane",
                            None,
                            [
                                OscArgument("right_of", SymbolValue("ego")),
                                OscArgument("at", SymbolValue("start")),
                            ],
                        ),
                        OscModifier(
                            "position",
                            None,
                            [
                                OscArgument(None, PhysicalValue(15.0, "m")),
                                OscArgument("behind", SymbolValue("ego")),
                                OscArgument("at", SymbolValue("start")),
                            ],
                        ),
                    ],
                ),
            )
        ]
    )
    (plan,) = translate_program(program)
    npc = plan.npcs[0]
    assert npc.relative_spawn is not None
    assert npc.relative_spawn.side == "right"
    assert npc.relative_spawn.longitudinal == pytest.approx(-15.0)
    # 15 m behind ego (spawn_s=40) on the ego's lanelet.
    assert npc.spawn_lanelet_id == 100
    assert npc.spawn_s == pytest.approx(25.0)
    assert any("right of ego" in n for n in plan.notes)


def test_dynamic_end_position_is_noted_not_transpiled() -> None:
    program = _drive_with(
        [
            OscModifier(
                "position",
                "ego",
                [
                    OscArgument(None, PhysicalValue(20.0, "m")),
                    OscArgument("ahead_of", SymbolValue("ego")),
                    OscArgument("at", SymbolValue("end")),
                ],
            )
        ]
    )
    (plan,) = translate_program(program)
    assert any("dynamic relative goal" in n for n in plan.notes)


def test_set_map_and_min_lanes_are_advisory() -> None:
    program = OscProgram(
        scenarios=[
            OscScenario(
                name="top",
                fields=[OscField("path", "Path"), OscField("ego", "vehicle")],
                modifiers=[
                    OscModifier(
                        "set_map", "path", [OscArgument(None, StringValue("Town04"))]
                    ),
                    OscModifier(
                        "path_min_driving_lanes",
                        "path",
                        [OscArgument(None, IntValue(3))],
                    ),
                ],
                do=OscInvocation("drive", "ego"),
            )
        ]
    )
    (plan,) = translate_program(program)
    assert plan.map_hint == "Town04"
    assert any("3 driving lanes" in n for n in plan.notes)


def test_change_lane_modifier_generates_gated_action_code() -> None:
    program = OscProgram(
        scenarios=[
            OscScenario(
                name="top",
                fields=[OscField("ego", "vehicle"), OscField("npc", "vehicle")],
                do=OscComposition(
                    operator="serial",
                    members=[
                        _timed_phase("run", 10.0, OscInvocation("drive", "ego")),
                        _timed_phase(
                            "change",
                            5.0,
                            OscInvocation(
                                "drive",
                                "npc",
                                modifiers=[
                                    OscModifier(
                                        "change_lane",
                                        None,
                                        [OscArgument("side", SymbolValue("left"))],
                                    )
                                ],
                            ),
                        ),
                    ],
                ),
            )
        ]
    )
    code = _module_from_plans(translate_program(program))
    compile(code, "top.py", "exec")
    assert "LaneChangeAction(" in code
    assert "ElapsedTimeCondition(10.0" in code


@_requires_osc2
def test_parse_scenario_runner_change_lane_transpiles(tmp_path: Path) -> None:
    """A scenario_runner-style two-actor timed choreography transpiles."""
    source = (
        "scenario top:\n"
        "    path: Path\n"
        '    path.set_map("Town04")\n'
        "    path.path_min_driving_lanes(3)\n"
        "    ego_vehicle: Model3\n"
        "    npc: Rubicon\n"
        "    do serial:\n"
        "        get_ahead: parallel(duration: 15s):\n"
        "            ego_vehicle.drive(path) with:\n"
        "                speed(20kph)\n"
        "                lane(1, at: start)\n"
        "            npc.drive(path) with:\n"
        "                lane(right_of: ego_vehicle, at: start)\n"
        "                position(15m, behind: ego_vehicle, at: start)\n"
        "        change_lane: parallel(duration: 5s):\n"
        "            ego_vehicle.drive(path)\n"
        "            npc.drive(path) with:\n"
        "                change_lane(lane_changes: 1, side: left)\n"
    )
    program = parse_program_from_string(source)
    plans = translate_program(program)
    (plan,) = plans
    assert plan.ego.name == "ego_vehicle"
    assert [n.name for n in plan.npcs] == ["npc"]
    assert plan.map_hint == "Town04"

    files = generate_package_files(plans, source_name="change_lane.osc")
    module = _scenario_module(files)
    compile(module, "change_lane.py", "exec")
    assert "LaneChangeAction(" in module
    assert "ElapsedTimeCondition(15.0" in module
    readme = files["README.md"]
    assert "Transpiler notes" in readme
    assert "Town04" in readme
