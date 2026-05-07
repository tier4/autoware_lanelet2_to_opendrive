# API Reference

This page lists the public surface of `autoware_carla_scenario`. All
symbols exported via the top-level package (`autoware_carla_scenario`)
are stable; symbols only reachable through deeper subpackages should be
considered internal unless explicitly noted.

## Top-level package

```python
from autoware_carla_scenario import (
    BaseScenario, EgoConfig, ScenarioQueue, ScenarioRunner,
    CarlaServerManager, CarlaScenarioFixture,
    # Conditions, actions, kinematics, coordinate poses ...
)
```

The full list of re-exports is defined in `autoware_carla_scenario.__all__`
and includes the conditions, actions, kinematics, coordinate, entity,
and sensor types described below.

## Scenario orchestration

| Symbol | Module | Purpose |
|--------|--------|---------|
| `BaseScenario` | `scenario_base` | Abstract base for user scenarios. Subclasses implement `setup()` and `is_done()`. |
| `EgoConfig` | `scenario_base` | `VehicleEntityConfig` subclass that fixes `role_name` to `EGO_ROLE_NAME`. |
| `ScenarioRunner` | `scenario_runner` | Executes a single `BaseScenario` against a CARLA world (sync mode tick loop, recording, cleanup). |
| `ScenarioQueue` | `scenario_queue` | Context manager that owns a `CarlaServerManager` and runs registered scenarios sequentially with cooldown / retry. |
| `CarlaServerManager` | `server` | Starts, reuses, and stops the CARLA UE5 process. Reads `CARLA_EXECUTABLE`. |
| `CarlaScenarioFixture` | `pytest_fixtures` | Helper that registers a scenario into a queue at import time and exposes a session-scoped pytest fixture for its `ScenarioResult`. |
| `EGO_ROLE_NAME` | `constants` | Reserved CARLA `role_name` used for the ego actor. |
| `EntityRole` | `entity_role` | Validated `role_name` wrapper for CARLA actors. Factories: `EntityRole.ego()`, `EntityRole.npc(n)`. |

`CameraRecorder` (re-exported from the top-level package, source in
`autoware_carla_scenario.camera_recorder`) is the two-pass video
renderer driven by the native CARLA recorder + an RGB camera sensor.
It is used internally by `ScenarioRunner` and can also be instantiated
directly.

## Conditions (`autoware_carla_scenario.conditions`)

All conditions inherit from `BaseCondition` and return a
`ScenarioResult` from `check(world, elapsed)` once triggered (or
`None` when not yet triggered).

| Symbol | Description |
|--------|-------------|
| `BaseCondition` | Abstract base class. Subclasses override `_check()`. |
| `ScenarioResult` | Pass / fail outcome with message, elapsed time, and per-condition statuses. |
| `ConditionStatus` | Per-condition leaf record used for reporting. |
| `AlwaysTrueCondition` | Default trigger for actions. |
| `AndCondition`, `OrCondition`, `NotCondition` | Logical combinators. |
| `StickyCondition`, `PersistentCondition` | Latch / persist a child condition's truth value. |
| `TimeoutCondition`, `ElapsedTimeCondition` | Time-based triggers. |
| `CollisionCondition`, `EntityExistenceCondition` | Safety checks. |
| `TrafficSignalCondition` | Traffic-light state check. |
| `ComparisonRule`, `ScalarComparisonRule`, `compare` | Numeric comparison primitives. `compare(actual, rule, value, tolerance)` is the underlying helper. |
| `find_actor_by_role_name`, `find_actor_in_list` | Helpers for locating CARLA actors by role. `find_actor_in_list` is reachable via `autoware_carla_scenario.conditions`. |

### Composition conditions (`autoware_carla_scenario.conditions.composition`)

These build on `CompositionCondition`, which composes a child condition
tree internally:

- `EntityLanePositionCondition`
- `EntityInAreaCondition`
- `SpeedCondition`, `SpeedDirection`, `SpeedCoordinateSystem`
- `StandstillCondition`
- `TemporaryStopCondition`
- `WaypointCondition`, `WaypointCheckType`

## Actions (`autoware_carla_scenario.actions`)

| Symbol | Description |
|--------|-------------|
| `BaseAction` | Abstract base. Owns a trigger `BaseCondition`, a `TickTiming`, and an `execute(world)` side effect. |
| `TickTiming` | Enum: `PRE_TICK` / `POST_TICK`. |
| `TurnAction`, `TurnDirection` | Steer the ego through left / right turns via the CARLA TrafficManager route hints. |
| `LaneChangeAction`, `LaneChangeDirection` | Trigger a TrafficManager lane change. |
| `TrafficSignalAction`, `TrafficLightTarget` | Set traffic-light states (e.g. all RED, all GREEN, or a specific actor). |
| `AttachCameraSensorAction` | Attach a generic `CameraSensorBase` once a condition fires. |
| `AttachCarlaCameraSensorAction` | CARLA-specific subclass that builds a `CarlaCameraSensor`. |

## Sensors (`autoware_carla_scenario.sensor`)

| Symbol | Description |
|--------|-------------|
| `CameraSensorBase`, `CameraSensorConfig` | Provider-agnostic camera sensor interface. |
| `CarlaCameraSensor`, `CarlaCameraSensorConfig` | CARLA RGB camera implementation, used by the video recorder and `AttachCarlaCameraSensorAction`. |

## Coordinate transforms (`autoware_carla_scenario.coordinate`)

| Symbol | Description |
|--------|-------------|
| `Lanelet2Pose`, `OpenDrivePose`, `CarlaWorldPose`, `AnyPose` | Frame-tagged pose dataclasses. |
| `CoordinateFrame`, `FrameMismatchError`, `frame_of` | Coordinate-frame tagging and validation. |
| `MapManager` | Singleton owning the loaded `LaneletMap`, `pyxodr` road network, MGRS offset, and z offset. |
| `to_carla_world`, `to_carla_location`, `to_lanelet2`, `to_opendrive` | Pose conversion entry points (overload by input frame). |
| `project_onto_road` | Project a `CarlaWorldPose` onto a specified OpenDRIVE road. |
| `snap_to_carla_road` | Ray-cast a pose onto the rendered CARLA ground surface. |
| `GroundProjectionConfig` | Ray-cast tuning for `snap_to_carla_road`. |
| `get_stop_line_poses`, `get_stop_line_poses_with_following` | Resolve stop-line `Lanelet2Pose`s for a lanelet (optionally including following lanelets). |

## Entities (`autoware_carla_scenario.entity`)

| Symbol | Description |
|--------|-------------|
| `VehicleEntity`, `VehicleEntityConfig` | Generic vehicle actor with retry-aware spawn. |
| `EgoVehicle` | Subclass with the fixed `EGO_ROLE_NAME`. |
| `AutowareEntity` | Marker base class for Autoware-managed entities. |
| `SpawnLocation` (Protocol) | Tag interface implemented by spawn-point providers. |
| `SpawnTransform` | Spawn at an explicit `carla.Transform`. |
| `SpawnPointIndex` | Spawn at the N-th map spawn point. |

## Kinematics (`autoware_carla_scenario.kinematics`)

Frame-aware velocity / acceleration types with affine-space arithmetic
(absolute - absolute = relative; absolute + relative = absolute).

| Symbol | Description |
|--------|-------------|
| `Vector3` | Frame-tagged 3-vector. |
| `CoordinateFrame`, `FrameMismatchError`, `frame_of` | Re-exported from `coordinate.frames`. |
| `AbsoluteVelocity`, `RelativeVelocity`, `FrenetVelocity` | Velocity types. |
| `AbsoluteAcceleration`, `RelativeAcceleration`, `FrenetAcceleration` | Acceleration types. |

## Lanelet constraint sweeper (`autoware_carla_scenario.sweeper`)

Powers the Hydra `lanelet_constraint` sweeper plugin. Can also be used
directly from Python:

| Symbol | Description |
|--------|-------------|
| `LaneletConstraintSweeper` | The Hydra `Sweeper` implementation (also re-exposed via `hydra_plugins.autoware_scenario_sweeper`). |
| `Constraint` (Protocol) | Base interface. |
| `EqualsConstraint`, `InSetConstraint`, `LaneletLengthConstraint`, `HasStopLineConstraint`, `HasTrafficLightStopLineConstraint`, `HasAdjacentConstraint`, `IsJunctionConstraint`, `PreviousOfConstraint`, `FollowingOfConstraint` | Atomic constraints. |
| `AndConstraint`, `OrConstraint`, `NotConstraint` | Combinators. |
| `parse_constraint`, `find_matching_lanelets` | YAML-to-`Constraint` parsing and lanelet matching. The corresponding YAML `type:` keys for the atomics above are `equals`, `in_set`, `lanelet_length`, `has_stop_line`, `has_traffic_light_stop_line`, `has_adjacent`, `is_junction`, `previous_of`, `following_of`. |
| `Binding` (Protocol), `StopLineOffsetBinding`, `parse_binding` | Per-match parameter derivation (e.g. compute `ego.spawn_s` from a stop-line offset). |
| `load_lanelet2_map` | Lightweight Lanelet2 loader used outside of CARLA. |

The plugin is registered with Hydra under
`hydra/sweeper=lanelet_constraint`; see
`src/hydra_plugins/autoware_scenario_sweeper/`.

## Result viewer (`autoware_carla_scenario.ui`)

Used internally by the `viewer` CLI. The web app is the supported
surface, but the helper modules are importable for tooling:

| Symbol | Description |
|--------|-------------|
| `ui.app` | FastAPI application object and route handlers. |
| `ui.scanner` | Discover sessions / scenarios under `outputs/` and `multirun/`, build condition trees. |
| `ui.runner` | Background `subprocess.run(["uv", "run", "scenario", ...])` orchestration with thread-safe progress. |
| `ui.sweep_resolver.resolve_sweep` | Resolve a sweep without launching CARLA. |
| `ui.models` | Pydantic models (`SessionSummary`, `SessionItem`, `ConditionNode`, `ScenarioResultView`, `RunProgress`). |

## Utilities (`autoware_carla_scenario.utils`)

| Symbol | Description |
|--------|-------------|
| `find_nearest_traffic_light` | Find the nearest CARLA traffic-light actor for a Lanelet2 traffic-light id. |
| `get_signal_ids_for_controller` | Map an OpenDRIVE controller id to its signal ids. |
| `lanelet2_traffic_light_id_to_opendrive_controller_id` | ID translation between Lanelet2 regulatory elements and OpenDRIVE controllers. |
| `get_stop_line_linestrings` | Collect Lanelet2 stop-line `LineString3d` objects for a lanelet. |

## CLI entry points

Defined in `pyproject.toml`:

| Command | Module |
|---------|--------|
| `scenario` | `autoware_carla_scenario.examples.run:main` |
| `detect-no-3d-model` | `autoware_carla_scenario.tools.detect_no_3d_model_lanelets:main` |
| `viewer` | `autoware_carla_scenario.ui:main` |

The `scenario` command also exposes Python-level helpers in
`autoware_carla_scenario.examples.run` for downstream packages:

| Symbol | Description |
|--------|-------------|
| `register_scenario(name, scenario_cls, config_cls)` | Register a built-in-style scenario class under a Hydra `scenario.name`. |
| `register_scenario_builder(name, builder)` | Register a custom builder when the constructor signature differs. |
| `get_scenario_registry()` | Return a copy of the registry. |
| `build_ego_and_spawn(cfg)` | Build `(EgoConfig, Lanelet2Pose, GroundProjectionConfig)` from a resolved Hydra config. |
| `build_scenario(cfg, *, build_scenario_fn=None)` | Look up the registered builder and instantiate the scenario. |
| `run_scenario(cfg, ...)` / `run_scenario_with_queue(...)` / `run_batch(...)` / `main()` | Programmatic execution paths used by Hydra and the glob batch dispatcher. |
