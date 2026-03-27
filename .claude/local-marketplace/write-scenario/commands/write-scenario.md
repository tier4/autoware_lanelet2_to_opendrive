---
description: Interactively create a CARLA test scenario step by step
argument-hint: [scenario description in natural language]
allowed-tools: Read, Write, Edit, Grep, Glob, Bash(uv:*), Bash(git:*), Agent, AskUserQuestion
---

Guide the user through creating a CARLA test scenario for the `autoware_carla_scenario` package. Use the `scenario-writing` skill for domain knowledge.

## Input

The user describes the scenario they want in natural language: $ARGUMENTS

## Interactive Workflow

Work through these stages sequentially, confirming with the user at each stage before proceeding.

### Stage 1: Understand the Scenario Concept

Parse the user's natural language description and clarify:
1. What behavior is being tested? (e.g., intersection passing, traffic light compliance, lane change)
2. What is the expected outcome? (pass condition)
3. What constitutes failure? (fail conditions)
4. Are NPC vehicles needed? Where relative to ego?

Ask the user focused clarifying questions using AskUserQuestion. Do not assume — confirm the scenario concept before writing code.

### Stage 2: Discover Available Building Blocks

Before writing code, dynamically discover existing Conditions and Actions:
1. Search for all `BaseCondition` subclasses in `autoware_carla_scenario/src/autoware_carla_scenario/conditions/`
2. Search for all `BaseAction` subclasses in `autoware_carla_scenario/src/autoware_carla_scenario/actions/`
3. Read constructor signatures of relevant classes

Present the user with a list of applicable existing Conditions/Actions for their scenario.

If existing building blocks are insufficient:
1. Explain what is missing and why
2. Propose the new Condition/Action with its interface
3. Wait for user approval before implementing

### Stage 3: Write Concrete Scenario

Generate the following files:
1. **Python scenario class** (`examples/<name>.py`) — inheriting BaseScenario
2. **Dataclass config** (add to `examples/configs.py`)
3. **YAML config** (`examples/conf/scenario/<name>/<variant>.yaml`) — with Lanelet2 spawn coordinates
4. **Registration** (update `examples/run.py`) — import, config, build_scenario branch

**Critical: spawn positions must use Lanelet2 coordinates (lanelet_id + s), never OpenDRIVE coordinates.**

Show the user each file before writing. Confirm the overall design, then write all files.

### Stage 4: Debug Concrete Scenario

Use `/debug-concrete-scenario <name>/<variant>` to execute the scenario, analyze results, and iterate until it passes.

This stage is handled by the `debug-concrete-scenario` command, which autonomously runs the execute → analyze → fix loop and reports back. Once the user confirms the behavior matches their intent, proceed to Stage 5.

### Stage 5: Define Logical Scenario Parameters

Once the concrete scenario passes, discuss which values should become parameters:
1. List candidate parameters and explain orthogonality considerations
2. Recommend which parameters to expose vs. compute in Python
3. Confirm the parameter set with the user

Update the YAML config to include concrete parameter values (these serve as the default concrete scenario).

### Stage 6: Discover Available Constraints/Bindings and Add Sweep Config

Before writing sweep constraints, dynamically discover existing Constraint and Binding types:
1. Search for all `BaseConstraint` subclasses in `autoware_carla_scenario/src/autoware_carla_scenario/sweeper/constraints.py`
2. Search for all `BaseBinding` subclasses in `autoware_carla_scenario/src/autoware_carla_scenario/sweeper/bindings.py`
3. Read the `build_constraint()` and `build_binding()` factory functions to understand the `type:` string → class mapping
4. Read constructor signatures of relevant classes to understand YAML parameters

Present the user with applicable existing Constraints/Bindings for their scenario.

If existing Constraints/Bindings are insufficient:
1. Explain what is missing and why existing types cannot express the desired filter
2. Propose the new Constraint/Binding with its interface and `type:` string
3. Wait for user approval before implementing
4. Implement following `BaseConstraint`/`BaseBinding` patterns, register in the factory function, and export properly

Then edit the `sweep:` section of the YAML:
1. Help the user compose constraints that work across maps
2. Add bindings for derived parameters
3. Avoid hardcoding specific Lanelet IDs in constraints

Suggest committing after this stage.

### Stage 7: Guide Multirun Execution

Provide the exact command for logical scenario execution:
```
uv run scenario scenario=<name>/<variant> hydra/sweeper=lanelet_constraint --multirun
```

Remind the user:
- Both `hydra/sweeper=lanelet_constraint` AND `--multirun` are required
- Without the sweeper name, `--multirun` runs only the concrete scenario
- Use `map=<name>` to switch maps (currently only `nishishinjuku` available)
