# OpenSCENARIO DSL Frontend

The `openscenario_dsl_frontend` subpackage parses
[ASAM OpenSCENARIO DSL](https://www.asam.net/standards/detail/openscenario-dsl/)
(OSC2, `.osc`) sources with [py-osc2](https://github.com/PMSFIT/py-osc2) and
**transpiles them into an installable
[scenario package](tutorial.md)** for this framework.

The generated scenario code is ordinary `autoware_carla_scenario` scenario
code — a [`BaseScenario`](api.md) subclass whose `setup()` registers the same
actions and pass/fail conditions you would write by hand. It is meant to be
committed, reviewed and edited, not treated as an opaque build artifact.

## Pipeline

```
.osc source
  → parser.parse_osc_file        # py-osc2 / ANTLR parse tree
  → extractor.extract_program    # syntax IR (OscProgram)
  → translator.translate_program        # semantic plans (ScenarioPlan per one_of variant)
  → package_codegen.generate_package     # installable scenario package
```

The parse/extract/translate/codegen layers are pure Python and do **not** import
CARLA, so they run (and are tested) without a CARLA installation. Only the
*generated* package's scenario module imports CARLA-backed modules at run time.

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
    .osc-venv/bin/pip install py-osc2 jinja2
    PYTHONPATH=autoware_carla_scenario/src \
        .osc-venv/bin/python -m autoware_carla_scenario.openscenario_dsl_frontend.cli \
        my_scenario.osc -o out/
    ```

    The frontend imports `py-osc2` lazily and raises `OscDependencyError` with
    this guidance when it is missing. Only the parse step needs it; the
    *generated* package has no dependency on `py-osc2` at all.

## Command-line usage

The subpackage installs an `osc-transpile` console script that generates an
installable scenario package:

```bash
# Create ./<scenario>_package
osc-transpile intersection_passing.osc

# Create it under out/, choosing the package name
osc-transpile intersection_passing.osc -o out/ --name my_pkg

# Syntax-check only (no output)
osc-transpile intersection_passing.osc --check
```

The package, scenario, class and config names default to the DSL `scenario`
name. When that name is a generic placeholder (`top`, `main`, … — e.g.
scenario_runner names its entry scenario `top`), the source `.osc` **filename**
is used instead, so `change_lane.osc` yields `change_lane` / `ChangeLaneScenario`
rather than `top`. Pass `--name` to override.

## The generated scenario package

The transpiler emits a standalone, installable **scenario package** — the
layout introduced in `autoware_carla_scenario`:

```
<scenario>_package/
├── pyproject.toml                              # autoware_carla_scenario.scenarios entry point
├── README.md
└── src/<scenario>_package/
    ├── __init__.py                             # register() → register_scenario(...) + register_conf_dir(...)
    ├── <scenario>.py                           # the transpiled BaseScenario subclass(es)
    ├── configs.py                              # config dataclass (name, timeout_seconds)
    └── conf/scenario/<variant>/default.yaml    # concrete ego spawn / timeout per variant
```

Installing it makes the scenario runnable through the framework CLI without
editing `autoware_carla_scenario`:

```bash
osc-transpile junction_choice.osc -o .
uv pip install -e junction_choice_package
uv run scenario scenario=junction_choice_v0/default map=nishishinjuku
```

Each `one_of` variant is registered as its own scenario
(`junction_choice_v0`, `junction_choice_v1`, …), so the whole logical scenario
can be swept with the framework's multirun. The ego spawn
(`lanelet_position`, `speed`) and the timeout land in the per-variant
`default.yaml` / config dataclass so they stay overridable via Hydra; the
manoeuvres and conditions are baked into the scenario class.

## Programmatic usage

```python
from autoware_carla_scenario.openscenario_dsl_frontend import (
    transpile_to_package,
    plans_from_file,
)

# Generate a full scenario package under out/.
root = transpile_to_package("intersection_passing.osc", output_dir="out")

# Inspect the semantic plans (one per one_of variant) without generating code.
plans = plans_from_file("intersection_passing.osc")
print(plans[0].ego, plans[0].specs)
```

## Example

`examples/openscenario/intersection_passing.osc`:

```
scenario intersection_passing:
    ego: vehicle
    do serial:
        ego.drive() with:
            speed(30kmph)
            lanelet_position(lanelet: 242)
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

Vehicle scenario fields become actors. A field is treated as a vehicle when it
is *driven* in the `do` tree, has a vehicle-like type (`vehicle`, `car`, …), or
is named after the ego — but never when its type is a known non-vehicle struct
(e.g. `path: Path`). This means catalog model types from the scenario_runner
dialect (`Model3`, `Rubicon`, …) resolve correctly. The actor whose name
mentions `ego` (e.g. `ego` or `ego_vehicle`) is the ego; every other vehicle
field becomes an NPC (`EntityRole.npc(i)`), spawned in `setup()`.

```
scenario s:
    ego: vehicle          # the ego vehicle
    npc: vehicle          # NPC #1
```
```
scenario top:             # scenario_runner dialect
    path: Path            # a struct field, not an actor
    ego_vehicle: Model3   # the ego (name mentions "ego")
    npc: Rubicon          # NPC #1
```

### Behaviours (invoked inside `do`)

| DSL behaviour | Maps to |
|---|---|
| `drive()`, `follow_lane()`, `keep_lane()` | no-op (autopilot); a carrier for `with:` modifiers |
| `turn(direction: left\|right)` | `TurnAction` |
| `change_lane(direction\|side: left\|right, lane_changes: N)` / `lane_change(...)` | `LaneChangeAction` (also usable as a `with:` modifier) |
| `set_traffic_lights(state: green\|red\|yellow)` | `TrafficSignalAction` |
| `reach_lane(lanelet: N)` / `on_lane(...)` | pass: sticky `EntityLanePositionCondition` |
| `stand_still(Ns)` / `stop(...)` | pass: `StandstillCondition` |
| `keep_speed_above(Nkmph)` / `min_speed(...)` | fail: `SpeedCondition` (below threshold) |
| `no_collision()` / `avoid_collision()` | fail: `CollisionCondition` |

### Modifiers (`with:` blocks and scenario-level)

| DSL modifier | Effect |
|---|---|
| `speed(Nkmph)` | actor initial spawn speed, or a timed `SpeedAction` in a later phase (see below) |
| `lanelet_position(lanelet: N, s: X)` | actor spawn position (see note) |
| `spawn_lanelet(N)` / `lanelet(N)` | actor spawn lanelet id (alias) |
| `spawn_s(N)` | longitudinal spawn arc-length (alias) |
| `position(Xm, behind\|ahead_of: actor, at: start)` | NPC longitudinal spawn relative to another actor |
| `lane(right_of\|left_of: actor)` / `lane(N)` | relational / map-relative lane placement (see below) |
| `vehicle_type("...")` / `model(...)` | CARLA blueprint id |
| `set_map("Town")` | binds a Hydra `map` group selection in the package config (see below) |
| `path_min_driving_lanes(N)` | pre-run `sweep.constraints` on `ego.spawn_lanelet_id` (see below) |
| `timeout(Ns)` | scenario-level fail: `TimeoutCondition` |

!!! note "`lanelet_position` is an extension type, not `lane_position`"
    OpenSCENARIO / OpenDRIVE's standard `lane_position` addresses an OpenDRIVE
    `(road, lane, s, offset)`. This framework spawns actors by **Lanelet2**
    lanelet id (`Lanelet2Pose`), a distinct concept the framework keeps
    separate from its OpenDRIVE types. `lanelet_position(lanelet:, s:)` is
    therefore a deliberate *extension* — `s` is the standard Frenet arc-length,
    but the lane is a lanelet, not an OpenDRIVE lane. A lateral `offset`/`t` is
    rejected because the spawn only uses `(lanelet, s)`.

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

Pass-condition steps in a `serial` block are likewise ordered: multiple
`reach_lane` / `stand_still` steps are combined with a `SequentialCondition`, so
the scenario only passes if they are satisfied **in the written order** (visiting
lanelet 265 before 460 does not count). In a `parallel` block the same steps are
combined with an order-insensitive `AndCondition`.

!!! note "Issued vs. physically complete"
    `ActionDoneCondition` fires when an action *issues* its command (e.g. sets
    the TrafficManager route), not when the manoeuvre has physically finished.
    Where physical completion matters, follow the manoeuvre with an observable
    step (a `reach_lane` / `stand_still`) so the next action gates on that.

### `wait` and `until` — explicit event conditions

A `wait <event_condition>` step (no action) and an `until <event_condition>`
clause in a behaviour's `with:` block let the DSL specify *when* to proceed,
rather than relying on the inferred completion signal. The event condition is
compiled to a framework condition and used as the next step's trigger:

```
do serial:
    ego.drive() with:
        until(ego.lane == 460)   # drive completes when the ego reaches lanelet 460
    wait elapsed(3s)             # then wait 3 s
    ego.turn(direction: left)    # then turn
```

Supported event conditions (see `expr_compiler.py`):

| DSL | Compiles to |
|---|---|
| `elapsed(Ns)` | `ElapsedTimeCondition` |
| `<actor>.speed\|velocity <op> <speed>` | `SpeedCondition` (`<`, `<=`, `>`, `>=`) |
| `<actor>.lane\|lanelet == N` | `EntityLanePositionCondition` |

When a `wait` follows another step, its condition is combined (via
`AndCondition`) with that step's completion, so ordering is preserved. Using
`until` gives a **real observable completion** (the "issued vs physically
complete" caveat above disappears for that step).

### `parallel` — concurrent steps

Members of a `parallel` block share the block's entry gate (they run
concurrently); the block completes when *all* members complete
(`AndCondition` of their completions).

### Timed phases — `label: parallel(duration: Ns)`

A named, time-bounded phase advances the sequence by **wall-clock time**: the
actions inside phase *i* arm at the cumulative elapsed instant, gated on an
`ElapsedTimeCondition`. A `serial` of timed phases (the scenario_runner
choreography style) therefore plays out phase-by-phase on a timeline:

```
do serial:
    approach: parallel(duration: 12s):   # phase 1 runs 0–12 s
        ego.drive() with: speed(30kmph)
        npc.drive() with: position(15m, behind: ego, at: start)
    merge: parallel(duration: 6s):       # phase 2 arms at t = 12 s
        ego.drive()
        npc.drive() with: change_lane(side: left)
```

Here the NPC's `change_lane` is emitted as a `LaneChangeAction` triggered by
`ElapsedTimeCondition(12.0)`. When the scenario declares no `timeout(...)`, a
fail-safe timeout is derived from the total phase duration (18 s above).

### `speed(...)` — initial vs. timed

`speed(N)` on the first/initial step sets the actor's **spawn speed**
(`initial_speed_kmh`). In a *later* timed phase it is a **speed change**, not a
spawn speed, so it compiles to a `SpeedAction` (TrafficManager
`set_desired_speed`) gated on that phase's start:

```
do serial:
    approach: parallel(duration: 15s):
        ego.drive() with: speed(30kmph)   # spawn speed
    slow: parallel(duration: 20s):
        npc.drive() with: speed(20kmph)   # SpeedAction at t = 15 s
```

### Relational placement (`behind` / `ahead_of` / `right_of`)

An NPC can be placed relative to another actor. A **longitudinal** offset
relative to the **ego** is emitted as code against the ego's runtime spawn pose,
so it needs no map and no transpile-time position:

```python
# position(15m, behind: ego, at: start)
npc_1_pose = Lanelet2Pose(
    lanelet_id=self._spawn_pose.lanelet_id,
    s=max(0.0, self._spawn_pose.s - 15.0),
)
```

(An offset relative to *another NPC* is instead resolved numerically at
transpile time.)

### Lateral & road-level placement → lanelet-constraint sweep

The **lateral** part of a placement is a map question, so instead of a note it
transpiles to the Hydra lanelet-constraint sweeper, which resolves spawn
lanelets against whatever map is chosen at run time
(`uv run scenario --multirun … hydra/sweeper=lanelet_constraint`). Three DSL
constructs feed one consistent sweep, using the sweeper's `has_adjacent`
primitive (existence of a same-direction neighbour):

| DSL | Sweep effect |
|---|---|
| `lane(1, at: start)` (leftmost) | constraint `not has_adjacent(left)` on `ego.spawn_lanelet_id` |
| `lane(n)` (n ≥ 2) | constraint `has_adjacent(left)` (approx: has lanes to the left) |
| `path_min_driving_lanes(N)` | constraint `and/or(has_adjacent left, right)` (only when no lane / lateral pin already applies) |
| `lane(right_of: ego)` on an NPC | ego constraint `has_adjacent(right)` **+** binding `adjacent_lanelet(side: right)` on `scenario.npc_<i>_spawn_lanelet_id` |

The NPC binding is the inter-actor hook: the sweeper constrains
`ego.spawn_lanelet_id`, then the `adjacent_lanelet` binding derives the NPC's
lanelet as the ego's chosen neighbour. The generated NPC reads that lanelet from
its config field (`self._config.npc_<i>_spawn_lanelet_id`), so a single sweep
run places both actors. For `change_lane.osc` the ego resolves to a leftmost
lane with a right neighbour, and the NPC to that right neighbour:

```yaml
sweep:
  constraints:
    ego.spawn_lanelet_id:
      - {type: not, constraint: {type: has_adjacent, value: left}}
      - {type: has_adjacent, value: right}
  bindings:
    scenario.npc_1_spawn_lanelet_id: {type: adjacent_lanelet, side: right}
```

Because `has_adjacent` only tests neighbour *existence* (it cannot count
lanes), an exact lane count from a fixed lane index degrades to "has a neighbour
on the required side".

### `set_map(...)` → bound `map` group selection

`set_map("Town04")` binds the map the same way the framework's top-level
`config.yaml` does — a Hydra `map` group selection in the generated scenario
config's defaults list, rather than a code comment:

```yaml
# @package _global_
defaults:
  - /map: town04     # from set_map("Town04")
  - _self_
```

So the scenario carries its own map and runs without an explicit `map=` on the
CLI. The package must find a matching `conf/map/town04.yaml` (with the Town's
lanelet2 + OpenDRIVE paths) on the Hydra search path; supply one, or override
with `map=<other>` at run time.

### Dynamic relative goals (`position(..., at: end)`)

A relative placement anchored at `end` — e.g. `position(20m, ahead_of: ego,
at: end)`, "the NPC must *end up* 20 m ahead of the ego" — is a **moving-target
goal**: both vehicles move, so it cannot be a fixed spawn. It transpiles to a
per-frame `RelativePositionAction`:

```python
RelativePositionAction(
    entity_name=EntityRole.npc(1),
    reference_name=EGO_ROLE_NAME,
    target_gap=20.0,          # +ahead / -behind, metres
    client=self.client,
    once=False,               # runs every tick
)
```

The action keeps the NPC on TrafficManager autopilot (lane keeping stays with
TM) and each tick nudges its desired cruise speed by a proportional law on the
longitudinal gap error, measured along the reference actor's forward axis:

```
gap   = (npc_pos - ego_pos) · ego_forward
error = target_gap - gap
desired_speed = clamp(ego_speed + gain * error, 0, max_speed)
```

so the NPC converges to — and holds — the requested gap as the ego moves. The
companion `RelativePositionCondition` checks whether the gap has been reached
(within a tolerance) and can register the goal as a scenario pass condition.
Only the longitudinal channel is controlled; the target lane is still TM's job.

### `one_of` — exclusive choice → concrete variants

A `one_of` is expanded at transpile time into **one concrete scenario per
branch** (a Cartesian product across `serial`/`parallel` nesting, a union over
`one_of`). The generated package then contains several classes
(`FooScenarioV0`, `FooScenarioV1`, …), each registered under its own name
(`foo_v0`, `foo_v1`, …) with its own `default.yaml` — so the whole logical
scenario slots directly into the runner's multirun / batch model. Expansion is
capped at 64 variants (excess is logged and truncated).

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

## Roadmap (not yet supported)

The following OSC2 constructs are recognised as future work. Each entry notes
the extension points involved (parser/extractor → IR → translator → framework
condition/action → codegen).

- [ ] **Richer event conditions** — logical combinators (`and` / `or` / `not`),
      `distance(a, b) <op> X`, time-headway and other attributes. Needs the
      `expr_compiler` to grow a small expression tree and, for distances, a new
      framework condition. (Today: single comparison on `speed`/`lane`, or
      `elapsed`.)
- [x] **Dynamic longitudinal relative goals** — `position(Xm, ahead_of/behind:
      other, at: end)` now transpiles to a per-frame `RelativePositionAction`
      (TrafficManager `set_desired_speed` proportional controller). *Remaining:*
      the **lateral** neighbour of a relational placement (`lane(right_of:
      other)`) still needs map-aware lanelet resolution or the Hydra sweeper,
      and the controller could grow an `apply_control` mode for precise metric
      goals where TM speed control is too coarse.
- [ ] **`event` / `on` handlers** — `event e is <cond>` + `on e: do ...`.
      Extract `event_declaration` / `on` into the IR; compile the event
      condition (reusing `expr_compiler`); emit the handler's actions gated on
      that condition. Mostly lands on the existing gated-action mechanism.
- [ ] **`emit` (manual signalling)** — needs a shared event bus in the
      framework (`EmitAction` sets a flag, an `EventCondition` reads it — a
      generalisation of the existing `ActionDoneCondition`).
- [ ] **`repeat(count)`** — bounded loops. Unroll the body `count` times at
      transpile time (like `one_of` expansion), chaining gates with per-iteration
      labels. No framework change required.
- [ ] **`repeat until <cond>`** — unbounded loops. Needs a runtime primitive
      (`RepeatAction` with `once=False`) and re-armable (non-latching)
      conditions, since `Sticky` / `ActionDone` latch permanently.
