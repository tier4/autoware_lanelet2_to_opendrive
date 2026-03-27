# Scenario Patterns Reference

## Scenario Class Anatomy

Every scenario inherits from `BaseScenario` and implements `setup()` and `is_done()`.

### Constructor Pattern

```python
class MyScenario(BaseScenario):
    def __init__(
        self,
        ego_config: EgoConfig,
        spawn_pose: Lanelet2Pose,
        config: MyScenarioConfig | None = None,
        ground_projection: GroundProjectionConfig | None = None,
    ) -> None:
        super().__init__(
            ego_config, spawn_pose=spawn_pose, ground_projection=ground_projection
        )
        self._config = config or MyScenarioConfig()
```

### Setup Pattern

```python
def setup(self) -> None:
    world = self.world
    cfg = self._config

    # 1. Convert Lanelet2 -> OpenDRIVE -> CARLA for ego spawn
    od_pose = self._setup_ego_spawn()

    # 2. Set traffic lights (typically all green or all red)
    TrafficSignalAction(
        state=carla.TrafficLightState.Green,
        lanelet2_traffic_light_ids=TrafficLightTarget.ALL,
        label="set_all_green",
    ).execute(world)

    # 3. Spawn NPC vehicles (optional)
    npc_pose = Lanelet2Pose(lanelet_id=npc_lanelet_id, s=npc_s)
    npc_od = to_opendrive(npc_pose)
    npc_snapped = snap_to_carla_road(npc_od, world, ground_projection=self._ground_projection)
    npc_entity = VehicleEntity(VehicleEntityConfig(
        role_name=EntityRole.npc(1),
        spawn_location=SpawnTransform(npc_snapped.to_carla_transform()),
        vehicle_type="vehicle.mini.cooper",
        initial_speed_kmh=0.0,
        spawn_retry_max_count=self.ego_config.spawn_retry_max_count,
        spawn_retry_t_step=self.ego_config.spawn_retry_t_step,
        spawn_retry_z_step=self.ego_config.spawn_retry_z_step,
        od_pose=npc_od,
        ground_projection=self._ground_projection,
    ))
    npc_entity.spawn(world)
    self.register_entity(npc_entity)

    # 4. Register actions (triggered by conditions)
    self.register_pre_tick(some_action)

    # 5. Register pass conditions
    self.register_pass_condition(pass_condition)

    # 6. Register fail conditions (ALWAYS include timeout)
    self.register_fail_condition(TimeoutCondition(cfg.timeout_seconds, label="timeout"))
```

## Common Condition Compositions

### Route Verification (Visit Multiple Roads)

```python
# Verify ego passes through roads in order using sticky conditions
sticky_conditions = [
    StickyCondition(
        EntityLanePositionCondition(
            entity_name=EGO_ROLE_NAME,
            road_id=road_id,
            label=f"ego_on_road_{road_id}",
        )
    )
    for road_id in expected_road_ids
]
self.register_pass_condition(AndCondition(sticky_conditions))
```

### Speed Check with Grace Period

```python
# Fail if ego drops below minimum speed (after warm-up)
self.register_fail_condition(
    AndCondition([
        ElapsedTimeCondition(grace_seconds, label="speed_grace"),
        SpeedCondition(
            entity_name=EGO_ROLE_NAME,
            value=min_speed_kmh / 3.6,
            rule=ComparisonRule.LESS_THAN,
            label="ego_min_speed",
        ),
    ])
)
```

### Temporary Stop Verification

```python
# Verify ego stops then restarts
stopped = StickyCondition(
    StandstillCondition(
        entity_name=EGO_ROLE_NAME,
        duration=stop_duration,
        label="ego_stopped",
    )
)
restarted = StickyCondition(
    AndCondition([
        ElapsedTimeCondition(check_after, label="restart_check"),
        SpeedCondition(
            entity_name=EGO_ROLE_NAME,
            value=restart_speed / 3.6,
            rule=ComparisonRule.GREATER_THAN_OR_EQUAL,
            label="ego_restarted",
        ),
    ])
)
self.register_pass_condition(AndCondition([stopped, restarted]))
```

### Traffic Light Phase Verification

```python
# Red phase: set lights red, verify ego stops
TrafficSignalAction(
    state=carla.TrafficLightState.Red,
    lanelet2_traffic_light_ids=TrafficLightTarget.ALL,
    label="set_red",
).execute(world)

# Green phase: switch after delay
green_action = TrafficSignalAction(
    state=carla.TrafficLightState.Green,
    lanelet2_traffic_light_ids=TrafficLightTarget.ALL,
    condition=ElapsedTimeCondition(delay, label="light_switch"),
    label="switch_green",
)
self.register_pre_tick(green_action)
```

### Time-Windowed Fail Condition

```python
# Fail if ego moves during a specific time window
self.register_fail_condition(
    AndCondition([
        ElapsedTimeCondition(window_start, ComparisonRule.GREATER_THAN_OR_EQUAL, label="window_start"),
        ElapsedTimeCondition(window_end, ComparisonRule.LESS_THAN, label="window_end"),
        SpeedCondition(
            entity_name=EGO_ROLE_NAME,
            value=threshold / 3.6,
            rule=ComparisonRule.GREATER_THAN_OR_EQUAL,
            label="ego_moving_in_window",
        ),
    ])
)
```

## Dataclass Config Pattern

```python
@dataclass
class MyScenarioConfig:
    """Parameters for my scenario."""
    name: str = "my_scenario"
    timeout_seconds: float = 10.0
    # Add scenario-specific parameters with sensible defaults
```

## YAML Config Pattern

```yaml
# @package _global_
scenario:
  name: my_scenario
  timeout_seconds: 10.0

ego:
  initial_speed_kmh: 5.0
  spawn_lanelet_id: 242
  spawn_s: 25.0

# Optional: sweep config for logical scenario
sweep:
  constraints:
    ego.spawn_lanelet_id:
      - type: and
        constraints:
          - <constraints>
  bindings:
    ego.spawn_s:
      type: stop_line_offset
      offset: 10.0
```

## Registration in run.py

Add the new scenario to the factory function:

```python
# In imports
from .my_scenario import MyScenario
from .configs import MyScenarioConfig

# In build_scenario()
if scenario_name == "my_scenario":
    return ego, MyScenario(
        ego,
        config=MyScenarioConfig(**scenario_dict),
        spawn_pose=spawn_pose,
        ground_projection=ground_projection,
    )
```

Also add `MyScenarioConfig` to the `ScenarioRunConfig.scenario` union type in `configs.py`.

## NPC Spawn from Ego Parameter

When NPC position depends on ego position, compute it in Python rather than making it a separate parameter:

```python
def setup(self):
    # Get predecessor lanelet for NPC spawn
    from autoware_carla_scenario.coordinate import MapManager
    map_mgr = MapManager.get_instance()
    routing_graph = map_mgr.get_routing_graph()
    # Find previous lanelet of ego's spawn lanelet
    previous = routing_graph.previous(ego_lanelet)
    npc_lanelet_id = previous[0].id
    npc_pose = Lanelet2Pose(lanelet_id=npc_lanelet_id, s=0.0)
```

This keeps the ego lanelet ID as the single parameter, maintaining orthogonality.
