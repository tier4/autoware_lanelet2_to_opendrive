---
description: Review a CARLA scenario's parameter design for orthogonality and sweeper compatibility
argument-hint: [scenario name or path to YAML config]
allowed-tools: Read, Grep, Glob, Bash(uv:*), Bash(git:*), Agent, AskUserQuestion
---

Review the parameter design of an existing CARLA test scenario. Use the `scenario-review` skill for domain knowledge.

## Input

The user specifies a scenario to review: $ARGUMENTS

## Review Workflow

### Step 1: Read the Scenario

1. Find and read the scenario's YAML config file in `examples/conf/scenario/`
2. Read the corresponding Python scenario class in `examples/`
3. Read the dataclass config in `examples/configs.py`

If no specific scenario is given, ask the user which scenario to review using AskUserQuestion.

### Step 2: Identify All Hardcoded Values

List all values in the YAML and Python code that could potentially be parameters:
- Lanelet IDs, s-values, speeds, distances, timeouts, thresholds
- NPC spawn positions and their relationship to ego
- Traffic light configurations

### Step 3: Analyze Orthogonality

Apply the orthogonality analysis from the `scenario-review` skill:
1. Check each parameter pair for independence
2. Identify derived values that should be bindings or computed in Python
3. Flag any anti-patterns (coupled lanelets, hardcoded s-values, excessive parameters)

### Step 4: Discover Available Constraint/Binding Types

Before recommending sweep configuration, dynamically discover existing types:
1. Search for all `BaseConstraint` subclasses in `sweeper/constraints.py`
2. Search for all `BaseBinding` subclasses in `sweeper/bindings.py`
3. Read `build_constraint()` and `build_binding()` factory functions for type mappings
4. Verify that recommended constraint/binding types actually exist

### Step 5: Review Existing Sweep Config (if present)

If the scenario already has a `sweep:` section:
1. Verify constraints are correct and use existing constraint types
2. Verify bindings are appropriate
3. Check for missing constraints or bindings
4. Identify any anti-patterns in the current configuration

### Step 6: Present Recommendations

Present the review summary using the format from the `scenario-review` skill:
- Parameters vs. bindings vs. constants vs. Python-computed
- Orthogonality check results
- Specific recommendations for improvements
- Any anti-patterns found

Use AskUserQuestion to confirm the recommendations with the user before suggesting changes.

### Step 7: Apply Changes (if requested)

If the user agrees with the recommendations:
1. Update the YAML config with revised parameter/binding/constraint structure
2. Update the Python scenario class if NPC computation needs to move from config to `setup()`
3. Update the dataclass config if parameters are added or removed
