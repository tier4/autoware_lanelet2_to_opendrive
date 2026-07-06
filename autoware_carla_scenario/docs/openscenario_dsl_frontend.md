# OpenSCENARIO DSL Frontend

The `openscenario_dsl_frontend` subpackage parses
[ASAM OpenSCENARIO DSL](https://www.asam.net/standards/detail/openscenario-dsl/)
(OSC2, `.osc`) sources with [py-osc2](https://github.com/PMSFIT/py-osc2) and
**transpiles them into readable Python scenarios** for this framework.

The generated code is ordinary `autoware_carla_scenario` scenario code — a
[`BaseScenario`](api.md) subclass whose `setup()` registers the same actions and
pass/fail conditions you would write by hand. It is meant to be committed,
reviewed and edited, not treated as an opaque build artifact.

## Pipeline

```
.osc source
  → parser.parse_osc_file        # py-osc2 / ANTLR parse tree
  → extractor.extract_program    # syntax IR (OscProgram)
  → translator.translate_program # semantic plan (ScenarioPlan)
  → codegen.generate_module      # readable Python source
```

The parse/extract/translate/codegen layers are pure Python and do **not** import
CARLA, so they run (and are tested) without a CARLA installation. Only the
*generated* code imports CARLA-backed modules at run time.

## Installing py-osc2

!!! warning "py-osc2 must live in an isolated environment"
    `py-osc2` pins `antlr4-python3-runtime==4.7.1`, which conflicts
    irreconcilably with the `==4.9.*` that `hydra-core` / `omegaconf` require —
    and its ANTLR-generated parser is version-coupled to the 4.7.1 runtime. For
    this reason `py-osc2` is **not** a declared dependency (or optional extra)
    of `autoware-carla-scenario`; adding it would break dependency resolution.

    Install it in a dedicated virtual environment and run the transpiler there,
    putting this package's `src/` on `PYTHONPATH`:

    ```bash
    python -m venv .osc-venv
    .osc-venv/bin/pip install py-osc2
    PYTHONPATH=autoware_carla_scenario/src \
        .osc-venv/bin/python -m autoware_carla_scenario.openscenario_dsl_frontend.cli \
        my_scenario.osc -o my_scenario_generated.py
    ```

    The frontend imports `py-osc2` lazily and raises `OscDependencyError` with
    this guidance when it is missing. Only the parse step needs it; the
    *generated* Python has no dependency on `py-osc2` at all.

## Command-line usage

The subpackage installs an `osc-transpile` console script:

```bash
# Print the generated Python to stdout
osc-transpile src/autoware_carla_scenario/examples/openscenario/intersection_passing.osc

# Write it to a file
osc-transpile intersection_passing.osc -o intersection_passing_generated.py

# Syntax-check only
osc-transpile intersection_passing.osc --check
```

## Programmatic usage

```python
from autoware_carla_scenario.openscenario_dsl_frontend import (
    transpile_file,
    transpile_to_file,
    plan_from_file,
)

# Get the generated Python source as a string.
code = transpile_file("intersection_passing.osc")

# Or write it straight to disk.
transpile_to_file("intersection_passing.osc", "generated.py")

# Inspect the semantic plan without generating code.
plan = plan_from_file("intersection_passing.osc")
print(plan.ego, plan.specs)
```

## Example

`examples/openscenario/intersection_passing.osc`:

```
scenario intersection_passing:
    ego: vehicle
    do serial:
        ego.drive() with:
            speed(30kmph)
            spawn_lanelet(242)
        set_traffic_lights(state: green)
        ego.turn(direction: left)
        ego.reach_lane(lanelet: 460)
        ego.reach_lane(lanelet: 265)
    timeout(10s)
```

transpiles to a `IntersectionPassingScenario(BaseScenario)` whose `setup()`:

- snaps the ego spawn to lanelet 242 at 30 km/h,
- sets all traffic lights green via `TrafficSignalAction`,
- registers a `TurnAction` (left) on the pre-tick,
- registers an `AndCondition` of sticky `EntityLanePositionCondition`s for
  lanelets 460 and 265 as the pass condition,
- registers a `TimeoutCondition(10.0)` fail-safe.

## Supported DSL vocabulary

The mapping from DSL names to framework module calls lives in
`openscenario_dsl_frontend/registry.py` and is intentionally small and explicit.

### Actors

Vehicle-typed scenario fields become actors. A field named `ego` is the ego
vehicle; every other vehicle field becomes an NPC (`EntityRole.npc(i)`).

```
scenario s:
    ego: vehicle      # the ego vehicle
    npc: vehicle      # NPC #1
```

### Behaviours (invoked inside `do`)

| DSL behaviour | Maps to |
|---|---|
| `drive()`, `follow_lane()`, `keep_lane()` | no-op (autopilot); a carrier for `with:` modifiers |
| `turn(direction: left\|right)` | `TurnAction` |
| `change_lane(direction: left\|right)` / `lane_change(...)` | `LaneChangeAction` |
| `set_traffic_lights(state: green\|red\|yellow)` | `TrafficSignalAction` |
| `reach_lane(lanelet: N)` / `on_lane(...)` | pass: sticky `EntityLanePositionCondition` |
| `stand_still(Ns)` / `stop(...)` | pass: `StandstillCondition` |
| `keep_speed_above(Nkmph)` / `min_speed(...)` | fail: `SpeedCondition` (below threshold) |
| `no_collision()` / `avoid_collision()` | fail: `CollisionCondition` |

### Modifiers (`with:` blocks and scenario-level)

| DSL modifier | Effect |
|---|---|
| `speed(Nkmph)` | actor initial speed |
| `spawn_lanelet(N)` / `position(N)` | actor spawn lanelet id |
| `spawn_s(N)` / `offset(N)` | longitudinal spawn offset |
| `vehicle_type("...")` / `model(...)` | CARLA blueprint id |
| `timeout(Ns)` | scenario-level fail: `TimeoutCondition` |

Unrecognised behaviour or modifier names raise `OscTranslationError` listing the
supported names, so unsupported constructs fail loudly rather than silently.

## Composition: sequencing and choices

`do` composition is not flattened — the operators carry meaning:

### `serial` — ordered steps via trigger conditions

Each step in a `serial` block is gated on the completion of the previous step,
using the framework's condition-triggered actions
(`BaseAction(..., condition=...)`). The completion signal depends on the step:

- an issued manoeuvre (`turn`, `change_lane`, `set_traffic_lights`) completes
  when it fires → the next action is gated on `ActionDoneCondition(that_action)`;
- `reach_lane(N)` completes when the actor reaches the lanelet → the next action
  is gated on that `EntityLanePositionCondition`;
- `stand_still(Ns)` completes when the standstill condition holds.

So `do serial: set_traffic_lights(green); ego.turn(left); ego.reach_lane(460);
ego.change_lane(right)` generates a `TurnAction` triggered only after the lights
were set, and a `LaneChangeAction` triggered only after lanelet 460 is reached —
rather than arming everything on the first tick.

!!! note "Issued vs. physically complete"
    `ActionDoneCondition` fires when an action *issues* its command (e.g. sets
    the TrafficManager route), not when the manoeuvre has physically finished.
    Where physical completion matters, follow the manoeuvre with an observable
    step (a `reach_lane` / `stand_still`) so the next action gates on that.

### `parallel` — concurrent steps

Members of a `parallel` block share the block's entry gate (they run
concurrently); the block completes when *all* members complete
(`AndCondition` of their completions).

### `one_of` — exclusive choice → concrete variants

A `one_of` is expanded at transpile time into **one concrete scenario per
branch** (a Cartesian product across `serial`/`parallel` nesting, a union over
`one_of`). The generated module then contains several classes
(`FooScenarioV0`, `FooScenarioV1`, …) and a `SCENARIO_VARIANTS` registry mapping
each variant name to its build function — which slots directly into the runner's
existing multirun / batch model. Expansion is capped at 64 variants (excess is
logged and truncated).

## Extending the mapping

Downstream projects can teach the transpiler about new DSL constructs without
editing the translator. Behaviour handlers return a `BehaviorResult`; modifier
handlers mutate the plan:

```python
from autoware_carla_scenario.openscenario_dsl_frontend import register_behavior
from autoware_carla_scenario.openscenario_dsl_frontend.plan import (
    BehaviorResult,
    Gate,
    Spec,
    SpecKind,
)


def _my_turn(actor, args):
    spec = Spec(kind=SpecKind.TURN, actor=actor, label=f"{actor}_custom", params=...)
    # Register the spec and tell the sequencer how this step signals completion.
    return BehaviorResult(actions=[spec], completion=Gate.action_done(spec.label))


register_behavior("my_turn", _my_turn)
```

## Limitations

- Only the subset of the OSC2 standard library listed above is translated;
  `py-osc2` is an alpha-quality parser, so exotic grammar constructs may not be
  supported.
- `one_of` is realised as concrete-variant expansion, not runtime nondeterministic
  choice; combinatorial blow-up is capped at 64 variants.
- Speed values without a unit are assumed to be km/h and durations without a
  unit are assumed to be seconds.
