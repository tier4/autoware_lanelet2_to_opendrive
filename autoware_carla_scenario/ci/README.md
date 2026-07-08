# CPU-only CARLA traffic-simulation CI

This directory holds the pieces of a **CPU-only** CARLA traffic-simulation
smoke test that runs in GitHub Actions **without a GPU**. It is inspired by
[`boundrivesim/starter-carla-0913`](https://github.com/boundrivesim/starter-carla-0913),
but pins CARLA **0.9.15** and uses the official image.

By default the simulation runs on a **built-in CARLA town map** (`Town03`) that
ships with the image, so CI is fully self-contained — no map data or OpenDRIVE
conversion is required. (A converted `.xodr` can still be simulated via the
`xodr-path` input if you want to exercise a real converted map.)

The moving parts:

| File | Purpose |
|------|---------|
| [`docker-compose.carla.yml`](docker-compose.carla.yml) | Boots a headless CARLA 0.9.15 server with the **Null RHI** (`-nullrhi`, no GPU/Vulkan). |
| [`carla_traffic_simulation.py`](carla_traffic_simulation.py) | Standalone client: loads a built-in town (or an OpenDRIVE map), spawns Traffic-Manager traffic, ticks synchronously, and asserts the traffic moved. |
| [`../../.github/actions/carla-simulation/action.yml`](../../.github/actions/carla-simulation/action.yml) | Reusable **composite action** that wires the two together. |
| [`../../.github/workflows/carla-simulation.yml`](../../.github/workflows/carla-simulation.yml) | Workflow that runs one scenario per matrix entry. |

## How it works

1. The composite action starts the CARLA 0.9.15 container in headless CPU-only
   mode (`-nullrhi -nosound`; `-RenderOffScreen` is avoided because it needs a
   real Vulkan device).
2. It installs the matching `carla==0.9.15` Python client into a throwaway
   virtualenv (the client version must equal the server version).
3. `carla_traffic_simulation.py` connects, loads the built-in town map with
   `load_world` (or, when `xodr-path` is given, generates a world from the
   `.xodr` via `generate_opendrive_world`), spawns autopilot vehicles, advances
   the simulation, and checks that vehicles actually drove.
4. The run passes when enough vehicles moved far enough; the JSON result,
   server logs, and a **native CARLA replay log** (`.log`) are uploaded as
   artifacts.

## Replay log

The native CARLA recorder is started before the vehicles spawn, so the whole
episode is captured. The `.log` is written on the *server* side (inside the
container) and copied out to the `carla_replay_<scenario>.log` artifact.

Download it and replay against a CARLA 0.9.15 server:

```python
import carla

client = carla.Client("localhost", 2000)
client.set_timeout(60.0)
# Print a human-readable summary of the recording:
print(client.show_recorder_file_info("carla_replay_town03-traffic.log", True))
# Or replay it (start, duration, follow-actor-id):
client.replay_file("carla_replay_town03-traffic.log", 0.0, 0.0, 0)
```

Recording is on by default; set the composite action's `record-replay: "false"`
to disable it.

## Why CPU-only works

`-nullrhi` disables the rendering hardware interface entirely, so no GPU is
needed. Physics and the Traffic Manager still run on the CPU, and built-in town
maps load their collision/physics/navigation meshes without rendering.
Camera/LiDAR sensors do **not** work in this mode, but they are not needed for a
traffic smoke test.

## Runner requirements

The workflow runs on a **stock GitHub-hosted `ubuntu-22.04` runner** — no GPU
and no larger/self-hosted runner. To stay within that budget the workflow:

- frees preinstalled toolchains before pulling the multi-GB CARLA image,
- caps shared memory at 1 GB (`shm_size` in the compose file),
- spawns only a handful of vehicles for a short run (`num-vehicles`, `ticks`),
- uses a bundled map, so it never builds the `lanelet2` bindings.

## Adding a scenario

Scenarios are defined declaratively in the workflow's `matrix.include` list.
Because the composite action is scenario-agnostic, adding a scenario is a
single matrix entry — no code changes:

```yaml
# .github/workflows/carla-simulation.yml
matrix:
  include:
    - name: town05-traffic
      map: Town05                       # any built-in CARLA town map
      num-vehicles: "12"
      ticks: "200"
      min-total-distance: "10.0"
      carla-image: carlasim/carla:0.9.15
      carla-client-version: "0.9.15"
```

To simulate a converted OpenDRIVE map instead of a built-in town, drop `map`
and pass `xodr-path` (the action accepts either). The composite action also
accepts finer-grained inputs (`warmup-ticks`, `fixed-delta`,
`min-moved-vehicles`, `seed`, `tm-port`, `extra-args`, …) — see its `inputs:`
block.

## Running locally

You need Docker (with Compose v2):

```bash
# 1. Start the CPU-only CARLA server
CARLA_IMAGE=carlasim/carla:0.9.15 \
  docker compose -f autoware_carla_scenario/ci/docker-compose.carla.yml up -d

# 2. Install the matching client and run the simulation on a built-in town
python3.10 -m venv /tmp/carla-venv
/tmp/carla-venv/bin/pip install "carla==0.9.15"
/tmp/carla-venv/bin/python autoware_carla_scenario/ci/carla_traffic_simulation.py \
  --map Town03 --num-vehicles 8 --ticks 120

# 3. Tear down
docker compose -f autoware_carla_scenario/ci/docker-compose.carla.yml down -v
```

The script exits `0` on pass and `1` on failure, and writes a JSON report
(`--output`, default `carla_traffic_sim_result.json`).
