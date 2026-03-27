---
description: Execute a CARLA concrete scenario, analyze results, and iterate until it passes
argument-hint: <scenario/variant> [additional uv run options]
allowed-tools: Read, Write, Edit, Grep, Glob, Bash(uv:*), Agent, AskUserQuestion
---

Debug and iterate on a CARLA concrete scenario until it passes. Use the `scenario-debugging` skill for domain knowledge on log analysis and common failure patterns.

## Input

The scenario to run: $ARGUMENTS

If no argument is provided, ask the user which scenario to run using AskUserQuestion.

## Workflow

### Step 1: Execute

Run the concrete scenario and capture full output:
```bash
uv run scenario scenario=<name>/<variant>
```

If the user provided additional options (e.g., `map=nishishinjuku`), append them to the command.

### Step 2: Analyze Results

Examine the output for:
- **Pass/fail status**: Which condition triggered the result?
- **Error messages**: Spawn failures, import errors, runtime exceptions, missing modules
- **Timing**: Did it timeout? Did conditions trigger too early or too late?
- **Warnings**: Messages that indicate potential issues (e.g., ground projection failures, coordinate conversion warnings)
- **Condition logs**: Which conditions were checked, in what order, and what values they observed

### Step 3: Diagnose and Fix

If the scenario fails or behaves unexpectedly:
1. Identify the root cause from the log output
2. Read the relevant source files (scenario class, config, YAML) to understand the issue
3. Apply the appropriate fix:
   - **Parameter issues**: Adjust timeouts, speeds, spawn positions in YAML or config
   - **Code errors**: Fix imports, logic errors, incorrect condition/action usage in the scenario class
   - **Spawn failures**: Adjust Lanelet2 coordinates or ground projection settings
   - **Condition timing**: Adjust grace periods, thresholds, or condition ordering

### Step 4: Iterate

Repeat Step 1 → Step 2 → Step 3 until the scenario passes as intended.

If you are stuck after 3 iterations on the same issue, stop and ask the user for guidance using AskUserQuestion rather than continuing to loop.

### Step 5: Report

Once the scenario passes (or if you hit an unresolvable issue), present to the user:
1. **Result**: Pass/fail and which condition triggered
2. **Adjustments made**: What was changed and why (list each change)
3. **Concerns**: Any remaining trade-offs or fragile parameters
4. Ask the user to confirm the behavior matches their intent
