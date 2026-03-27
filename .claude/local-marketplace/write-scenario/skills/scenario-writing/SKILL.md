---
name: scenario-writing
description: This skill should be used when the user asks to "write a scenario", "create a CARLA scenario", "add a test scenario", "write a concrete scenario", "create a logical scenario", "add constraints", "run multirun", "spawn NPC", "add condition", "add action", mentions scenario writing for autoware_carla_scenario, or discusses CARLA test scenario creation workflow. Provides comprehensive guidance for the full scenario lifecycle from concrete to logical scenarios.
---

# CARLA Scenario Writing Guide

## Purpose

Guide the interactive creation of CARLA test scenarios for the `autoware_carla_scenario` package. Cover the full lifecycle: concrete scenario authoring, parameter tuning, logical scenario conversion, constraint definition, and multirun execution.

## Scenario Writing Workflow

Follow these stages in order. Commit after completing the concrete scenario stage.

### Stage 1: Concrete Scenario

Create a Python class inheriting `BaseScenario` with Lanelet2 spawn coordinates.

**Critical rules:**
- **Always specify spawn positions in Lanelet2 coordinate system** (`Lanelet2Pose(lanelet_id=..., s=...)`). The OpenDRIVE converter auto-assigns Road/Lane IDs, so OpenDRIVE coordinates break when the converter changes.
- **Use `_setup_ego_spawn()` and `snap_to_carla_road()`** for coordinate conversion with ground projection. Without ground projection, ~1 in 30 spawns fail due to clipping into terrain or roadside objects.
- **Guard against infinite loops.** Unlike OpenSCENARIO-DSL, scenarios run as Python — ensure all tick loops have termination conditions (timeout, pass/fail).
- **Prefer composing existing Conditions/Actions** over creating new ones. Proper Condition/Action objects leave viewer logs for debugging.

**File structure for a new scenario:**
1. Python file: `autoware_carla_scenario/src/autoware_carla_scenario/examples/<scenario_name>.py`
2. Dataclass config: Add to `examples/configs.py`
3. YAML config: `examples/conf/scenario/<scenario_name>/<variant>.yaml`
4. Register in `examples/run.py`: Add import, config class, and `build_scenario()` branch

**Scenario class pattern:**
```python
class MyScenario(BaseScenario):
    def __init__(self, ego_config, spawn_pose, config=None, ground_projection=None):
        super().__init__(ego_config, spawn_pose=spawn_pose, ground_projection=ground_projection)
        self._config = config or MyScenarioConfig()

    def setup(self):
        self._setup_ego_spawn()           # Lanelet2 -> OpenDRIVE -> CARLA
        # Register actions (pre_tick / post_tick)
        # Register pass conditions
        # Register fail conditions (always include TimeoutCondition)

    def is_done(self):
        return False  # Driven by conditions
```

### Stage 2: Debug Concrete Scenario

Use the `debug-concrete-scenario` command (or `scenario-debugging` skill) to execute, analyze, and iterate on the scenario until it passes.

This stage autonomously runs the execute → analyze → fix loop. See the `scenario-debugging` skill for log analysis patterns and common failure fixes. Once the user confirms the behavior matches intent, proceed to Stage 3.

### Stage 3: Define Logical Scenario Parameters

Decide which values become parameters. **Prioritize orthogonality** between parameters — orthogonal parameters simplify constraint solving in the sweeper.

**Key rule:** When an NPC spawn depends on the ego's position (e.g., "spawn NPC on the lanelet before ego"), make only the ego's lanelet ID a parameter. Compute the NPC position in Python from the ego parameter.

Write parameters as a dataclass in `configs.py` and corresponding YAML in `conf/scenario/`. Concrete and logical parameters share the same YAML file.

### Stage 4: Discover Constraints/Bindings and Add Sweep Config

**Do not rely on a hardcoded list.** Dynamically discover available Constraint and Binding types:

1. Find all classes inheriting from `BaseConstraint`:
   - Search for `class.*BaseConstraint` in `autoware_carla_scenario/src/autoware_carla_scenario/sweeper/constraints.py`
   - Read the `build_constraint()` factory function to see the `type:` string → class mapping
2. Find all classes inheriting from `BaseBinding`:
   - Search for `class.*BaseBinding` in `autoware_carla_scenario/src/autoware_carla_scenario/sweeper/bindings.py`
   - Read the `build_binding()` factory function to see the `type:` string → class mapping
3. Read constructor signatures to understand YAML parameters for each type

**When existing Constraints/Bindings are insufficient:**
1. Propose the new Constraint/Binding to the user with rationale
2. Wait for approval before implementing
3. Inherit from `BaseConstraint` or `BaseBinding`
4. Implement the required interface (`check()` for constraints, `resolve()` for bindings)
5. Register in the `build_constraint()`/`build_binding()` factory function
6. Export from the appropriate `__init__.py`

Then edit the `sweep:` section in the scenario YAML. Avoid hardcoding specific Lanelet IDs in logical scenario constraints — this breaks cross-map portability.

Map-specific exclusions (like `no_3d_model_lanelet_ids`) are centralized in `conf/map/<map_name>.yaml`.

See `references/constraints-reference.md` for YAML structure examples and patterns.

**Tip:** Insert `type: equals, value: any` constraints as placeholders. After multirun, replace with concrete values from successful runs to aid debugging.

### Stage 5: Run Logical Scenario

```bash
# Logical scenario (parameter sweep)
uv run scenario scenario=<name>/<variant> hydra/sweeper=lanelet_constraint --multirun

# Concrete scenario (single run, no sweeper)
uv run scenario scenario=<name>/<variant>

# Different map
uv run scenario scenario=<name>/<variant> map=nishishinjuku
```

**Warning:** Omitting `hydra/sweeper=lanelet_constraint` with `--multirun` runs only the concrete scenario, not the sweep.

## Discovering Available Conditions and Actions

**Do not rely on a hardcoded list.** Dynamically discover available types:

1. Find all classes inheriting from `BaseCondition`:
   - Search for `class.*BaseCondition` and `class.*Condition.*BaseCondition` in `autoware_carla_scenario/src/`
   - Also check `conditions/composition/` for composite conditions
2. Find all classes inheriting from `BaseAction`:
   - Search for `class.*BaseAction` and `class.*Action.*BaseAction`
3. Read constructor signatures to understand parameters
4. Check `__init__.py` exports for the public API

**When existing Conditions/Actions are insufficient:**
1. Propose the new Condition/Action to the user with rationale
2. Wait for approval before implementing
3. Inherit from `BaseCondition` or `BaseAction`
4. Implement `check()` (conditions) or `execute()` (actions)
5. Export from appropriate `__init__.py`

## Additional Resources

### Reference Files

For detailed patterns and type catalogs, consult:
- **`references/constraints-reference.md`** — Sweeper constraint types, binding types, and YAML examples
- **`references/scenario-patterns.md`** — Complete scenario examples and common composition patterns

### Key Source Locations

- Conditions: `autoware_carla_scenario/src/autoware_carla_scenario/conditions/`
- Actions: `autoware_carla_scenario/src/autoware_carla_scenario/actions/`
- Coordinate transforms: `autoware_carla_scenario/src/autoware_carla_scenario/coordinate/`
- Sweeper: `autoware_carla_scenario/src/autoware_carla_scenario/sweeper/`
- Example scenarios: `autoware_carla_scenario/src/autoware_carla_scenario/examples/`
- Hydra configs: `autoware_carla_scenario/src/autoware_carla_scenario/examples/conf/`
