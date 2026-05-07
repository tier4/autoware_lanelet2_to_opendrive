# Usage Guide

This guide explains how to use the `autoware-carla-scenario` package to run autonomous driving scenario tests in CARLA simulator.

## CLI Commands Overview

The package provides three CLI commands:

| Command | Framework | Purpose |
|---------|-----------|---------|
| [`scenario`](#scenario-scenario-runner) | Hydra | Run autonomous driving scenario tests in CARLA |
| [`detect-no-3d-model`](#detect-no-3d-model-3d-model-detection) | argparse | Detect lanelets without a matching 3D ground model in CARLA |
| [`viewer`](#viewer-scenario-result-viewer) | FastAPI + Uvicorn | Web UI for browsing and monitoring scenario test results |

---

## `scenario` - Scenario Runner

Runs autonomous driving scenario tests in CARLA. Uses [Hydra](https://hydra.cc/) for configuration management. Requires a running CARLA server.

### How It Works

1. Parses the `scenario=...` argument (supports glob patterns for batch execution)
2. Composes a Hydra config from map, server, ego, entity, and scenario configs
3. Connects to the CARLA server and loads the specified map (OpenDRIVE or built-in)
4. Spawns the ego vehicle at the configured Lanelet2 position (converted to CARLA coordinates)
5. Enables synchronous mode (fixed 20 Hz tick rate) and starts the native CARLA recorder
6. Runs a warm-up phase (5 ticks for physics stabilization)
7. Enables autopilot and applies initial speeds
8. Executes the main tick loop, checking pass/fail conditions each tick
9. Writes results as JSON and optionally renders a video from the recording
10. Reloads the CARLA world for clean state between scenarios

### Basic Usage

```bash
# Run a single scenario
uv run scenario scenario=intersection_passing/straight

# Run a left-turn variant
uv run scenario scenario=intersection_passing/left_turn

# Run traffic light compliance scenario
uv run scenario scenario=traffic_light_compliance/traffic_light_compliance

# Run a lane change scenario
uv run scenario scenario=lane_change/left

# Run a temporary stop scenario
uv run scenario scenario=temporary_stop/temporary_stop
```

### Batch Execution with Glob Patterns

When the `scenario=` value contains glob metacharacters (`*`, `?`, `[`), all matching scenario configs are executed sequentially in a single CARLA session:

```bash
# Run all intersection-passing variants
uv run scenario scenario='intersection_passing/*'

# Run all scenarios matching a partial pattern
uv run scenario scenario='intersection_passing/left_*'

# Run all lane-change scenarios
uv run scenario scenario='lane_change/*'

# Run everything
uv run scenario scenario='*/*'
```

**Batch constraints:**

- All scenarios in a batch **must share the same map** (CARLA loads one map per session)
- Results are written to `outputs/YYYY-MM-DD/HH-MM-SS/batch_results.json`
- A summary table is printed at the end with per-scenario pass/fail status

### Lanelet Constraint Sweeping

The package ships a Hydra `Sweeper` plugin
(`hydra/sweeper=lanelet_constraint`) that expands a single scenario
config into one job per lanelet that matches a set of constraints,
optionally deriving per-job parameters via "bindings". Each scenario
YAML may declare its own `sweep:` section (see
`traffic_light_compliance.yaml` for an example).

```bash
# Run a Hydra multirun over every lanelet whose `ego.spawn_lanelet_id`
# satisfies the scenario's `sweep.constraints`.
uv run scenario --multirun scenario=traffic_light_compliance/traffic_light_compliance \
  hydra/sweeper=lanelet_constraint

# Resume a sweep from job number N (1-indexed; 0 disables).
uv run scenario --multirun scenario=traffic_light_compliance/traffic_light_compliance \
  hydra/sweeper=lanelet_constraint sweep.resume_from=10
```

Sweeps write results under `multirun/YYYY-MM-DD/HH-MM-SS/<job-index>/`,
not `outputs/`. The result viewer scans both directories.

### Resume from a Specific Scenario

When running large batches, you can skip already-completed scenarios:

```bash
# Resume from the 3rd scenario (0-based index)
uv run scenario scenario='intersection_passing/*' --resume-from 2
```

### Overriding Configuration Parameters

Any nested Hydra parameter can be overridden from the command line:

```bash
# Select a different map
uv run scenario scenario=intersection_passing/straight map=nishishinjuku

# Connect to a remote CARLA server
uv run scenario scenario=intersection_passing/straight server.host=192.168.1.100 server.port=3000

# Change ego vehicle type and speed
uv run scenario scenario=intersection_passing/straight \
  ego.vehicle_type=vehicle.tesla.model3 \
  ego.initial_speed_kmh=30.0

# Override scenario-specific parameters
uv run scenario scenario=intersection_passing/straight scenario.timeout_seconds=15.0

# Change Traffic Manager port
uv run scenario scenario=intersection_passing/straight traffic_manager.port=8200
```

### Hydra Configuration System

Configuration files are located in `src/autoware_carla_scenario/examples/conf/`:

```
conf/
├── config.yaml              # Root config (defines defaults and top-level overrides)
├── server/
│   └── localhost.yaml       # Server connection settings
├── ego/
│   └── default.yaml         # Ego vehicle settings
├── entity/
│   └── default.yaml         # Entity spawn and ground projection settings
├── traffic_manager/
│   └── default.yaml         # CARLA Traffic Manager settings
├── map/
│   └── nishishinjuku.yaml   # Map file paths and excluded lanelet IDs
└── scenario/
    ├── intersection_passing/
    │   ├── straight.yaml
    │   ├── left_turn.yaml
    │   ├── right_turn.yaml
    │   └── straight_3npc.yaml
    ├── lane_change/
    │   ├── left.yaml
    │   └── right.yaml
    ├── lane_change_fail/
    │   ├── left.yaml
    │   └── right.yaml
    ├── temporary_stop/
    │   └── temporary_stop.yaml
    └── traffic_light_compliance/
        └── traffic_light_compliance.yaml
```

#### Root Configuration (`config.yaml`)

Defines the default config group composition:

| Default Group | Default Value | Description |
|---------------|---------------|-------------|
| `server` | `localhost` | CARLA server connection |
| `ego` | `default` | Ego vehicle configuration |
| `entity` | `default` | Entity spawn and ground projection |
| `traffic_manager` | `default` | CARLA Traffic Manager |
| `scenario` | `intersection_passing/left_turn` | Scenario to run |
| `map` | `nishishinjuku` | Map to load |

#### Server Configuration

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `server.host` | str | `localhost` | CARLA server hostname |
| `server.port` | int | `2000` | CARLA server port |
| `server.cooldown_seconds` | float | `2.0` | Wait time between consecutive scenario runs |
| `server.cooldown_max_retries` | int | `0` | Maximum retries on failure |

#### Ego Vehicle Configuration

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `ego.vehicle_type` | str | `vehicle.mini.cooper` | CARLA blueprint ID for ego vehicle |
| `ego.initial_speed_kmh` | float | `0.0` | Initial speed of ego vehicle (km/h) |
| `ego.spawn_lanelet_id` | int | `242` | Lanelet ID for spawn position |
| `ego.spawn_s` | float | `25.0` | Longitudinal offset along lanelet centerline (meters) |

#### Entity Configuration

Controls ground projection (ray casting) and spawn retry behavior:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `entity.ground_projection_ray_distance_upper` | float | `5.0` | Ray search range above estimated z (meters) |
| `entity.ground_projection_ray_distance_lower` | float | `10.0` | Ray search range below estimated z (meters) |
| `entity.spawn_retry_max_count` | int | `10` | Maximum retry attempts for spawn |
| `entity.spawn_retry_t_step` | float | `0.1` | Lateral offset shift per retry (meters) |
| `entity.spawn_retry_z_step` | float | `0.5` | Vertical shift per retry (meters) |

#### Traffic Manager Configuration

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `traffic_manager.port` | int | `8100` | CARLA Traffic Manager RPC port |

#### Map Configuration

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `map.name` | str | **required** | CARLA map name (e.g., `NishishinjukuMap`) |
| `map.xodr_path` | str \| null | `null` | Path to custom OpenDRIVE file. If `null`, loads built-in CARLA map by name |
| `map.lanelet2_path` | str \| null | `null` | Path to Lanelet2 .osm file (required for lanelet-based spawn) |
| `map.no_3d_model_lanelet_ids` | list[int] | `[]` | Lanelet IDs to exclude from sweeps (no ground geometry) |

Map paths can use environment variables with Hydra's OmegaConf resolver:

```yaml
xodr_path: ${oc.env:NISHISHINJUKU_XODR_PATH,path/to/default.xodr}
```

### Supported Scenarios

#### Intersection Passing

Tests ego vehicle traversal through an intersection. Verifies that the ego passes through the expected OpenDRIVE road sequence.

| Config | Turn Direction | Description |
|--------|---------------|-------------|
| `intersection_passing/straight` | none | Drive straight through intersection |
| `intersection_passing/left_turn` | left | Left turn at intersection |
| `intersection_passing/right_turn` | right | Right turn at intersection |
| `intersection_passing/straight_3npc` | none | Straight with 3 NPC vehicles |

**Scenario-specific parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `scenario.expected_route_lanelet_ids` | list[int] | varies | Lanelet IDs the ego must traverse |
| `scenario.timeout_seconds` | float | `5.0` | Maximum scenario duration |
| `scenario.min_speed_kmh` | float \| null | null | Minimum ego speed (fail if below). `null` disables check |
| `scenario.speed_check_delay_seconds` | float | `0.3` | Grace period before speed check starts |
| `scenario.turn_direction` | str \| null | null | `"left"`, `"right"`, or `null` for straight |
| `scenario.npc_vehicles` | list | `[]` | NPC vehicle configurations (see below) |

**NPC vehicle configuration:**

```yaml
npc_vehicles:
  - spawn_lanelet_id: 100
    spawn_s: 5.0
    vehicle_type: vehicle.mini.cooper
    initial_speed_kmh: 10.0
```

**Behavior:**
- Sets all traffic lights to GREEN
- Registers a TurnAction for left/right turns (applies steering via CARLA autopilot route)
- Pass condition: ego traverses all expected OpenDRIVE road IDs (checked via StickyCondition + AndCondition)
- Fail conditions: timeout, ego destroyed, speed below threshold

#### Lane Change

Tests ego vehicle lane change maneuvers. Can test both successful lane changes and expected failures.

| Config | Direction | Expected Result |
|--------|-----------|-----------------|
| `lane_change/left` | left | success |
| `lane_change/right` | right | success |
| `lane_change_fail/left` | left | fail (no adjacent lane) |
| `lane_change_fail/right` | right | fail (no adjacent lane) |

**Scenario-specific parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `scenario.direction` | str | `"left"` | Lane change direction: `"left"` or `"right"` |
| `scenario.expect` | str | `"success"` | Expected outcome: `"success"` or `"fail"` |
| `scenario.timeout_seconds` | float | `10.0` | Maximum scenario duration |

**Behavior:**
- Sets all traffic lights to GREEN
- Immediately registers a LaneChangeAction
- Derives the target lane from the current spawn position and direction
- Pass condition: `expect=success` requires both `road_id` and `lane_id` to change; `expect=fail` passes when timeout is reached without a lane change
- Checks via EntityLanePositionCondition

#### Traffic Light Compliance

Tests that the ego vehicle correctly stops at a red light and proceeds after green.

| Config |
|--------|
| `traffic_light_compliance/traffic_light_compliance` |

**Scenario-specific parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `scenario.light_switch_delay_seconds` | float | `5.0` | Delay before switching red to green |
| `scenario.merging_time_seconds` | float | `3.0` | Margin subtracted from red phase duration |
| `scenario.timeout_seconds` | float | `8.0` | Maximum scenario duration |
| `scenario.moving_speed_kmh` | float | `1.0` | Speed threshold to detect "moving" (km/h) |

**Behavior:**
- Sets all traffic lights to RED initially
- Registers a TrafficSignalAction to switch to GREEN after `light_switch_delay_seconds`
- Pass conditions: ego is at standstill during red AND moves above `moving_speed_kmh` during green
- Uses StickyCondition + AndCondition for combined checks

#### Temporary Stop

Tests that the ego vehicle stops at a designated stop line for a minimum duration, then restarts.

| Config |
|--------|
| `temporary_stop/temporary_stop` |

**Scenario-specific parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `scenario.s_margin` | float | `5.0` | Arc-length margin around stop position (meters) |
| `scenario.speed_threshold` | float | `0.1` | Max speed considered as "stopped" (m/s) |
| `scenario.stop_duration` | float | `0.3` | Minimum consecutive seconds to remain stopped |
| `scenario.restart_speed_kmh` | float | `3.0` | Speed threshold for restart detection (km/h) |
| `scenario.timeout_seconds` | float | `5.0` | Maximum scenario duration |

**Behavior:**
- Sets all traffic lights to GREEN
- Registers a TemporaryStopCondition at the configured stop position
- Pass condition: ego stays within `s_margin` of stop line at speed below `speed_threshold` for at least `stop_duration` seconds, then accelerates above `restart_speed_kmh`

### Scenario Evaluation

Each scenario registers **pass conditions** and **fail conditions**. The tick loop evaluates them every frame:

- **First pass condition satisfied** → scenario passes
- **First fail condition triggered** → scenario fails
- Common fail conditions: `TimeoutCondition` (exceeded time limit), `EntityExistenceCondition` (ego destroyed)

### Output Files

Each scenario run generates output in `outputs/YYYY-MM-DD/HH-MM-SS/`:

| File | Description |
|------|-------------|
| `{ScenarioName}.log` | CARLA native recording (replay format) |
| `{ScenarioName}_result.json` | Machine-readable result with condition statuses |
| `{ScenarioName}.mp4` | Rendered video from recording (optional, if spectator is configured) |
| `batch_results.json` | Batch summary (glob/batch mode only) |

**Result JSON format:**

```json
{
  "scenario": "intersection_passing/straight",
  "passed": true,
  "message": "All pass conditions satisfied",
  "elapsed_seconds": 3.42,
  "condition_statuses": [
    {"label": "Road 5 visited", "satisfied": true, "message": "..."},
    {"label": "Road 12 visited", "satisfied": true, "message": "..."}
  ]
}
```

### Exit Codes

| Code | Meaning |
|------|---------|
| 0 | All scenarios passed |
| 1 | One or more scenarios failed, or configuration/runtime error |

---

## `detect-no-3d-model` - 3D Model Detection

Detects lanelets that do not have a matching 3D ground model in CARLA. Useful for identifying areas where vehicle ground-projection ray casting will fail. Requires a running CARLA server with the target map loaded.

### How It Works

1. Reads the map configuration YAML to resolve `xodr_path` and `lanelet2_path`
2. Connects to the CARLA server
3. For each lanelet's center point, casts a vertical ray (upward and downward from estimated z) to detect ground geometry
4. Reports lanelets where the ray does not hit any 3D surface
5. Writes the detected IDs to the YAML configuration's `no_3d_model_lanelet_ids` field

### Usage

```bash
# Basic detection using map config YAML
uv run detect-no-3d-model conf/map/nishishinjuku.yaml

# Connect to a remote CARLA server
uv run detect-no-3d-model conf/map/nishishinjuku.yaml --host 192.168.1.100 --port 3000

# Increase ray search range for hilly terrain
uv run detect-no-3d-model conf/map/nishishinjuku.yaml --ray-upper 10.0 --ray-lower 10.0

# Longer timeout for slow connections
uv run detect-no-3d-model conf/map/nishishinjuku.yaml --timeout 30.0

# Debug logging
uv run detect-no-3d-model conf/map/nishishinjuku.yaml -v
```

### Command-Line Options

| Option | Short | Required | Default | Description |
|--------|-------|----------|---------|-------------|
| `yaml_path` | - | Yes | - | Path to map configuration YAML (must contain `map.name`, `xodr_path`, `lanelet2_path`) |
| `--host` | - | No | `localhost` | CARLA server hostname |
| `--port` | - | No | `2000` | CARLA server port |
| `--ray-upper` | - | No | `5.0` | Ray search range above z estimate (meters) |
| `--ray-lower` | - | No | `5.0` | Ray search range below z estimate (meters) |
| `--timeout` | - | No | `10.0` | CARLA client timeout (seconds) |
| `--verbose` | `-v` | No | false | Enable DEBUG logging |

### Input YAML Requirements

The YAML file must contain at minimum:

```yaml
name: NishishinjukuMap        # CARLA map name
xodr_path: path/to/map.xodr  # Path to OpenDRIVE file
lanelet2_path: path/to/map.osm  # Path to Lanelet2 file
```

### Integration with Scenario Runner

The detected lanelet IDs are stored in the map config's `no_3d_model_lanelet_ids` field. The scenario runner's sweep feature uses this list to exclude problematic lanelets from automatic spawn point selection.

---

## `viewer` - Scenario Result Viewer

Web UI for browsing and monitoring scenario test results. Starts a FastAPI server with Uvicorn.

### How It Works

1. Scans the base path directory for scenario output files (`outputs/YYYY-MM-DD/HH-MM-SS/`)
2. Serves a web interface for browsing sessions, viewing individual scenario results, and watching recorded videos
3. Provides an API for triggering scenario runs and monitoring progress in real time

### Usage

```bash
# Start with default settings (0.0.0.0:9000, current directory)
uv run viewer

# Custom host and port
VIEWER_HOST=127.0.0.1 VIEWER_PORT=8080 uv run viewer

# Specify results directory
VIEWER_BASE_PATH=/path/to/scenario/outputs uv run viewer
```

Then open `http://localhost:9000` (or the configured host/port) in a browser.

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `VIEWER_BASE_PATH` | Current working directory | Base path for scanning scenario result files |
| `VIEWER_HOST` | `0.0.0.0` | Server bind address |
| `VIEWER_PORT` | `9000` | Server listen port |

### Web UI Pages

| Route | Description |
|-------|-------------|
| `/` | Session list — displays all test sessions grouped by date/time |
| `/session/{type}/{date}/{time}` | Session detail — shows all scenarios in a session with pass/fail status |
| `/session/{type}/{date}/{time}/{index}` | Scenario detail — condition tree, per-condition status, and video playback |
| `/video/{type}/{date}/{time}/{index}/{filename}` | Video file serving for MP4 playback |

### REST API Endpoints

The viewer also provides API endpoints for programmatic use and the web UI's interactive features:

| Method | Endpoint | Description |
|--------|----------|-------------|
| `POST` | `/api/refresh` | Clear result cache and return updated session table HTML |
| `GET` | `/api/scenarios` | List all available scenario config names |
| `POST` | `/api/run/preview` | Preview the CLI command that would be executed |
| `POST` | `/api/run` | Start a scenario execution in the background |
| `GET` | `/api/run/status` | Get current run status (non-streaming) |
| `GET` | `/api/run/progress` | SSE (Server-Sent Events) stream for real-time progress |

**Run request body format:**

```json
{
  "scenario": "*/*",
  "extra_overrides": ["server.host=192.168.1.100"],
  "sweeper": ""
}
```

**Progress SSE event format:**

```json
{
  "current": 2,
  "total": 5,
  "scenario_name": "intersection_passing/straight",
  "status": "running"
}
```

Where `status` is one of: `"running"`, `"idle"`, or `"done"`.

---

## Environment Variables

| Variable | Used By | Description |
|----------|---------|-------------|
| `CARLA_EXECUTABLE` | scenario runner | Path to CARLA binary executable |
| `NISHISHINJUKU_XODR_PATH` | map config | Override default OpenDRIVE file path for nishishinjuku |
| `NISHISHINJUKU_LANELET2_PATH` | map config | Override default Lanelet2 file path for nishishinjuku |
| `VIEWER_BASE_PATH` | viewer | Base path for scenario results |
| `VIEWER_HOST` | viewer | Viewer server bind address |
| `VIEWER_PORT` | viewer | Viewer server listen port |

---

## Common Use Cases

### Use Case 1: Run a Single Scenario

```bash
# Start CARLA server first, then:
uv run scenario scenario=intersection_passing/straight
```

### Use Case 2: Run All Scenarios for a Map

```bash
uv run scenario scenario='*/*' map=nishishinjuku
```

### Use Case 3: Test on a Remote CARLA Server

```bash
uv run scenario scenario=intersection_passing/left_turn \
  server.host=192.168.1.100 server.port=2000
```

### Use Case 4: Detect Missing 3D Models Before Running Scenarios

```bash
# 1. Detect lanelets without ground geometry
uv run detect-no-3d-model conf/map/nishishinjuku.yaml

# 2. The detected IDs are written to no_3d_model_lanelet_ids in the YAML
# 3. Now run scenarios — sweep will skip these lanelets
uv run scenario scenario='intersection_passing/*' map=nishishinjuku
```

### Use Case 5: Browse Results in the Web UI

```bash
# 1. Run scenarios
uv run scenario scenario='*/*'

# 2. Launch the viewer pointing to the outputs directory
VIEWER_BASE_PATH=outputs uv run viewer

# 3. Open http://localhost:9000 in a browser
```

### Use Case 6: Resume a Failed Batch

```bash
# Original batch run (5 scenarios, failed at index 3)
uv run scenario scenario='intersection_passing/*'

# Resume from the 4th scenario (0-based index 3)
uv run scenario scenario='intersection_passing/*' --resume-from 3
```

---

## Troubleshooting

### "Connection refused" or timeout when connecting to CARLA

**Cause**: CARLA server is not running, or host/port is incorrect.

**Solution**:
- Verify the CARLA server is started and accessible
- Check `server.host` and `server.port` overrides match the server
- Increase `--timeout` for `detect-no-3d-model`

### "batch scenarios must share the same map"

**Cause**: Glob pattern matched scenarios configured for different maps.

**Solution**:
- Use `map=<name>` override to force all scenarios to use the same map
- Or narrow the glob pattern to match scenarios for one map only

### Ego vehicle falls through the ground

**Cause**: The spawn lanelet has no corresponding 3D ground geometry in CARLA.

**Solution**:
- Run `detect-no-3d-model` to identify problematic lanelets
- Add them to `no_3d_model_lanelet_ids` in the map config
- Increase `entity.ground_projection_ray_distance_lower` for deeper ray casting
- Increase `entity.spawn_retry_max_count` for more spawn attempts

### Scenario times out unexpectedly

**Cause**: `scenario.timeout_seconds` is too short, or the ego vehicle is stuck.

**Solution**:
- Increase `scenario.timeout_seconds`
- Check `ego.initial_speed_kmh` — a value of `0.0` requires autopilot to start moving
- Enable verbose logging and check condition statuses in the result JSON

### "Unknown scenario name" error

**Cause**: The `scenario.name` field in the YAML does not match any registered scenario class.

**Solution**:
- Supported scenario names: `intersection_passing`, `lane_change`, `traffic_light_compliance`, `temporary_stop`
- Check the `name` field in your scenario YAML config

---

## Next Steps

- [API Reference](api.md) — Detailed API documentation for programmatic usage
- [Development Guide](development.md) — Contributing and development setup
