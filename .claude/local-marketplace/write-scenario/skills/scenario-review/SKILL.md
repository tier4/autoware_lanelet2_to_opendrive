---
name: scenario-review
description: This skill should be used when reviewing CARLA scenario parameters for orthogonality, analyzing parameter dependencies, deciding which values should be parameters vs. computed, validating the parameter design of a logical scenario, or checking sweep compatibility. Use when "review parameters", "check orthogonality", "parameter design", "review scenario parameters", "parameter independence", or discussing parameter selection for sweeper constraints.
---

# Scenario Parameter Review Guide

## Purpose

Review and validate the parameter design of CARLA test scenarios before converting concrete scenarios to logical scenarios. Ensure parameters are orthogonal, minimal, and compatible with the lanelet constraint sweeper.

## Parameter Review Process

### Step 1: Identify Candidate Parameters

Examine the concrete scenario's YAML config and Python code to list all hardcoded values that could vary across test runs:

**Common candidates:**
- `ego.spawn_lanelet_id` — Ego vehicle spawn lanelet
- `ego.spawn_s` — Ego arc-length position along the lanelet
- `npc.spawn_lanelet_id` — NPC vehicle spawn lanelet
- `npc.spawn_s` — NPC arc-length position
- `scenario.timeout_seconds` — Scenario timeout
- Speed parameters (initial speed, target speed)
- Distance/timing thresholds
- Traffic light states or phases

### Step 2: Analyze Parameter Orthogonality

**Orthogonality principle:** Each parameter should be independently variable. If changing parameter A forces a change in parameter B, they are not orthogonal — one should be derived from the other.

**Checklist for each parameter pair:**
1. Can parameter A change without affecting parameter B's valid range? -> Orthogonal
2. Does parameter B's value depend on parameter A's value? -> Not orthogonal -> Derive B from A
3. Are both parameters constrained to the same lanelet topology? -> Likely not orthogonal

**Common non-orthogonal patterns:**

| Pattern | Problem | Solution |
|---------|---------|----------|
| Ego lanelet + NPC lanelet (NPC relative to ego) | NPC position depends on ego position | Make only ego lanelet a parameter, compute NPC lanelet in Python |
| Spawn lanelet + spawn s (s depends on lanelet geometry) | s valid range varies per lanelet | Use a binding to compute s from the lanelet (e.g., `stop_line_offset`) |
| Two parameters that must satisfy a topological relationship | Sweeper cannot efficiently search coupled parameter spaces | Make one a parameter, derive the other via routing graph |

### Step 3: Classify Each Candidate

For each candidate parameter, classify it as:

1. **Parameter** — Independently variable, meaningful for the sweeper to enumerate
   - Typically: ego spawn lanelet ID, scenario-level config values

2. **Binding** — Derived from a parameter using a deterministic rule
   - Typically: spawn s-values (computed from stop line position), NPC positions relative to ego

3. **Constant** — Fixed value that should not vary
   - Typically: timeout values, speed thresholds, grace periods

**Decision tree:**
```
Is the value independently meaningful across different test runs?
+- YES -> Is the sweeper capable of enumerating it efficiently?
|         +- YES -> PARAMETER (add to sweep.constraints)
|         +- NO  -> Reduce to a simpler parameter + binding
+- NO  -> Does it depend on another parameter?
          +- YES -> BINDING (add to sweep.bindings) or compute in Python
          +- NO  -> CONSTANT (keep hardcoded in dataclass default)
```

### Step 4: Validate NPC Dependency Design

**Critical rule:** When an NPC's spawn depends on the ego's position, **never** make the NPC's lanelet ID an independent parameter.

**Correct pattern:**
```python
# In setup(), compute NPC position from ego's spawn lanelet
map_mgr = MapManager.get_instance()
routing_graph = map_mgr.get_routing_graph()
ego_lanelet = map_mgr.get_lanelet(self._spawn_pose.lanelet_id)
previous = routing_graph.previous(ego_lanelet)
npc_lanelet_id = previous[0].id
npc_pose = Lanelet2Pose(lanelet_id=npc_lanelet_id, s=0.0)
```

**Why:** If both ego and NPC lanelet IDs are parameters, the sweeper must search the Cartesian product of all lanelet pairs. Most combinations are invalid (NPC must be near ego), making the search extremely inefficient. Computing NPC position from ego position eliminates this combinatorial explosion.

### Step 5: Verify Sweeper Compatibility

For each parameter that will be swept:

1. **Can a constraint express the valid set?**
   - Dynamically discover existing constraint types from `build_constraint()` factory
   - If no existing constraint fits, propose a new one to the user

2. **Can a binding compute derived values?**
   - Dynamically discover existing binding types from `build_binding()` factory
   - If no existing binding fits, propose a new one to the user

3. **Is the parameter space finite and reasonable?**
   - Lanelet IDs: finite per map (good)
   - Continuous values (speed, distance): use discrete bins or bindings
   - Avoid parameters that create excessively large search spaces

### Step 6: Present Review Summary

Present the review results to the user in this format:

```
## Parameter Review Summary

### Parameters (swept by sweeper)
- `ego.spawn_lanelet_id` — [rationale]

### Bindings (derived from parameters)
- `ego.spawn_s` -> `stop_line_offset(offset=10.0)` — [rationale]

### Computed in Python (derived in setup())
- NPC lanelet ID -> computed from ego lanelet via routing graph — [rationale]

### Constants (fixed values)
- `timeout_seconds: 10.0` — [rationale]

### Orthogonality Check
- [pass/fail] All parameters are independently variable
- [pass/fail] No NPC positions depend on ego without being derived
- [pass/fail] All derived values use bindings or Python computation
- [pass/fail] Constraint types exist for all parameter constraints
```

## Anti-Patterns to Flag

### Anti-Pattern 1: Coupled Lanelet Parameters

```yaml
# BAD: ego and NPC lanelets as independent parameters
sweep:
  constraints:
    ego.spawn_lanelet_id:
      - type: has_traffic_light_stop_line
    npc.spawn_lanelet_id:
      - type: previous_of
        constraints:
          - type: equals
            value: ${ego.spawn_lanelet_id}  # This doesn't work!
```

**Fix:** Remove `npc.spawn_lanelet_id` from parameters. Compute it in Python from `ego.spawn_lanelet_id`.

### Anti-Pattern 2: Hardcoded s-values as Parameters

```yaml
# BAD: s-value as independent parameter when it depends on stop line
sweep:
  constraints:
    ego.spawn_s:
      - type: range
        min: 0.0
        max: 50.0
```

**Fix:** Use a binding instead:
```yaml
sweep:
  bindings:
    ego.spawn_s:
      type: stop_line_offset
      offset: 10.0
```

### Anti-Pattern 3: Too Many Parameters

If the scenario has more than 2-3 swept parameters, the search space may be too large.

**Fix:** Prioritize the most impactful parameters and derive the rest.

### Anti-Pattern 4: Non-Topological Parameter as Sweep Target

```yaml
# BAD: sweeping over timeout values
sweep:
  constraints:
    scenario.timeout_seconds:
      - type: range
        min: 5.0
        max: 30.0
```

**Fix:** Timeout is a constant, not a topological parameter. Keep it as a fixed dataclass default.

## Integration with Write Workflow

This review should be performed at **Stage 5** of the `/write-scenario` workflow (after the concrete scenario passes, before adding sweep constraints). The review ensures that the transition from concrete to logical scenario maintains correctness and efficiency.

After the review is confirmed by the user, proceed to Stage 6 (Discover Constraints/Bindings and Add Sweep Config) using the agreed parameter design.
