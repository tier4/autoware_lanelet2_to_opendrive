# OSC DSL → scenario package (round-trip sample)

This directory is a worked example of the OpenSCENARIO DSL frontend: a built-in
scenario is re-expressed as OpenSCENARIO DSL and transpiled into an installable
scenario package.

```
osc_transpiled_intersection_passing/
├── intersection_passing.osc          # 1. the DSL (derived from the built-in scenario)
└── intersection_passing_package/     # 2. the transpiled, installable package
```

## 1. Reference scenario

The DSL was written to mirror the framework's built-in `intersection_passing`
scenario (its "straight" variant):

- `autoware_carla_scenario/.../examples/intersection_passing.py`
- `.../examples/conf/scenario/intersection_passing/straight.yaml`

That scenario spawns the ego on lanelet 242 (s = 25) at 5 km/h, sets all
traffic lights green, expects the ego to traverse the roads of lanelets 460
then 265, fails if it drops below 5 km/h, and times out after 5 s.

## 2. The DSL (`intersection_passing.osc`)

```
scenario intersection_passing:
    ego: vehicle
    do serial:
        ego.drive() with:
            speed(5kmph)
            spawn_lanelet(242)
            spawn_s(25.0)
        set_traffic_lights(state: green)
        ego.reach_lane(lanelet: 460)
        ego.reach_lane(lanelet: 265)
        ego.keep_speed_above(5kmph)
    timeout(5s)
```

## 3. Transpile → package

```bash
osc-transpile intersection_passing.osc -o .
```

This generated `intersection_passing_package/` (committed here so you can read
the output without running the transpiler):

```
intersection_passing_package/
├── pyproject.toml                                        # autoware_carla_scenario.scenarios entry point
├── README.md
└── src/intersection_passing_package/
    ├── __init__.py                                       # register() → register_scenario + register_conf_dir
    ├── intersection_passing.py                           # the transpiled BaseScenario
    ├── configs.py                                        # name, timeout_seconds
    └── conf/scenario/intersection_passing/default.yaml   # ego spawn + timeout
```

How the DSL maps into the generated `setup()`:

| DSL | Generated |
|---|---|
| `speed(5kmph)` / `spawn_lanelet(242)` / `spawn_s(25.0)` | `default.yaml` `ego:` block (Hydra-overridable) |
| `set_traffic_lights(state: green)` | `TrafficSignalAction(state=…Green, …)` (pre-tick) |
| `ego.reach_lane(460)` → `ego.reach_lane(265)` (serial) | `SequentialCondition([Sticky(EntityLanePosition 460), Sticky(… 265)])` — ordered pass |
| `ego.keep_speed_above(5kmph)` | fail: `ElapsedTimeCondition` + `SpeedCondition(<5/3.6 m/s)` |
| `timeout(5s)` | fail: `TimeoutCondition(self._config.timeout_seconds)` |

> Note: the built-in scenario combines the two lane checks with an unordered
> `AndCondition`; because the DSL lists them in a `serial` block, the transpiler
> emits an ordered `SequentialCondition` (must reach 460 *before* 265) — a
> stricter, faithful reading of the sequence.

## 4. Run it

```bash
uv pip install -e intersection_passing_package
uv run scenario scenario=intersection_passing/default map=nishishinjuku
```
