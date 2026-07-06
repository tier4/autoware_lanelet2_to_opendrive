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
    transpile_file,
)
from autoware_carla_scenario.openscenario_dsl_frontend import (
    __file__ as _frontend_file,
)
from autoware_carla_scenario.openscenario_dsl_frontend.ast_model import (
    OscArgument,
    OscComposition,
    OscField,
    OscInvocation,
    OscModifier,
    OscProgram,
    OscScenario,
)
from autoware_carla_scenario.openscenario_dsl_frontend.codegen import generate_module
from autoware_carla_scenario.openscenario_dsl_frontend.translator import (
    translate_program,
)
from autoware_carla_scenario.openscenario_dsl_frontend.values import (
    FloatValue,
    IntValue,
    PhysicalValue,
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
    """Build a rich scenario IR directly (no parser required)."""
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
                    behavior="turn",
                    actor="ego",
                    arguments=[OscArgument("direction", SymbolValue("left"))],
                ),
                OscInvocation(
                    behavior="set_traffic_lights",
                    actor=None,
                    arguments=[OscArgument("state", SymbolValue("green"))],
                ),
                OscInvocation(
                    behavior="reach_lane",
                    actor="ego",
                    arguments=[OscArgument("lanelet", IntValue(460))],
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
    plan = translate_program(_demo_program())
    assert plan.ego.name == "ego"
    assert plan.ego.spawn_lanelet_id == 242
    assert plan.ego.spawn_s == pytest.approx(5.0)
    assert plan.ego.initial_speed_kmh == pytest.approx(30.0)
    assert [npc.name for npc in plan.npcs] == ["npc"]
    assert plan.npcs[0].index == 1


def test_translate_produces_expected_specs() -> None:
    plan = translate_program(_demo_program())
    kinds = {spec.kind for spec in plan.specs}
    assert {
        SpecKind.TURN,
        SpecKind.TRAFFIC_SIGNAL,
        SpecKind.REACH_LANE,
        SpecKind.MIN_SPEED,
        SpecKind.COLLISION,
        SpecKind.TIMEOUT,
    } <= kinds


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
    plan = translate_program(program)
    assert plan.ego.is_ego
    assert plan.ego.name == "ego"


# ---------------------------------------------------------------------------
# codegen (pure, no py-osc2)
# ---------------------------------------------------------------------------


def test_generated_code_is_valid_python() -> None:
    plan = translate_program(_demo_program())
    code = generate_module(plan, source_name="demo.osc")
    compile(code, "generated_demo.py", "exec")
    assert "class DemoScenario(BaseScenario):" in code
    assert "def build_scenario() -> DemoScenario:" in code
    assert "self._setup_ego_spawn()" in code


def test_generated_code_contains_module_calls() -> None:
    plan = translate_program(_demo_program())
    code = generate_module(plan, source_name="demo.osc")
    assert "TurnAction(" in code
    assert "TurnDirection.LEFT" in code
    assert "TrafficSignalAction(" in code
    assert "EntityLanePositionCondition(" in code
    assert "TimeoutCondition(8.0, label=" in code
    assert "Lanelet2Pose(lanelet_id=242, s=5.0)" in code


def test_multiple_pass_conditions_wrapped_in_and() -> None:
    program = OscProgram(
        scenarios=[
            OscScenario(
                name="x",
                fields=[OscField(name="ego", type_name="vehicle")],
                do=OscComposition(
                    operator="serial",
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
    code = generate_module(translate_program(program), source_name="x.osc")
    assert "AndCondition(" in code
    compile(code, "generated_and.py", "exec")


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
def test_parse_error_raises() -> None:
    with pytest.raises(OscParseError) as exc_info:
        parse_program_from_string("scenario broken\n  ego vehicle\n")
    assert exc_info.value.errors


@_requires_osc2
def test_example_osc_transpiles() -> None:
    assert _EXAMPLE_OSC.is_file(), f"missing example: {_EXAMPLE_OSC}"
    code = transpile_file(_EXAMPLE_OSC)
    compile(code, "example.py", "exec")
    assert "class IntersectionPassingScenario(BaseScenario):" in code
