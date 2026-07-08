# CPU-only CARLA traffic-simulation CI

This directory holds the pieces of a **CPU-only** CARLA traffic-simulation
smoke test that runs in GitHub Actions **without a GPU**. It is inspired by
[`boundrivesim/starter-carla-0913`](https://github.com/boundrivesim/starter-carla-0913),
but pins CARLA **0.9.15** and uses the official image.

The moving parts:

| File | Purpose |
|------|---------|
| [`docker-compose.carla.yml`](docker-compose.carla.yml) | Boots a headless CARLA 0.9.15 server with the **Null RHI** (`-nullrhi`, no GPU/Vulkan). |
| [`carla_traffic_simulation.py`](carla_traffic_simulation.py) | Standalone client: loads an OpenDRIVE map, spawns Traffic-Manager traffic, ticks synchronously, and asserts the traffic moved. |
| [`../../.github/actions/carla-simulation/action.yml`](../../.github/actions/carla-simulation/action.yml) | Reusable **composite action** that wires the two together. |
| [`../../.github/workflows/carla-simulation.yml`](../../.github/workflows/carla-simulation.yml) | Workflow that converts a map and runs one scenario per matrix entry. |

## How it works

1. The workflow converts a Lanelet2 map to OpenDRIVE with the repository's
   `convert` CLI (`target=carla`).
2. The composite action starts the CARLA 0.9.15 container in headless CPU-only
   mode (`-RenderOffScreen -nullrhi -nosound`).
3. It installs the matching `carla==0.9.15` Python client into a throwaway
   virtualenv (the client version must equal the server version).
4. `carla_traffic_simulation.py` connects, generates the world from the
   `.xodr` via `generate_opendrive_world` (a procedural road mesh — no
   pre-baked map assets, so it works under `-nullrhi`), spawns autopilot
   vehicles, advances the simulation, and checks that vehicles actually drove.
5. The run passes when enough vehicles moved far enough; the JSON result and
   server logs are uploaded as artifacts.

## Why CPU-only works

`-nullrhi` disables the rendering hardware interface entirely, so no GPU is
needed. Physics and the Traffic Manager still run on the CPU, and
`generate_opendrive_world` builds the collision/physics road mesh at runtime —
the visual mesh is skipped (`enable_mesh_visibility=False`). Camera/LiDAR
sensors do **not** work in this mode, but they are not needed for a traffic
smoke test.

## Runner requirements

The workflow runs on a **stock GitHub-hosted `ubuntu-22.04` runner** — no GPU
and no larger/self-hosted runner. To stay within that budget the workflow:

- frees preinstalled toolchains before pulling the multi-GB CARLA image,
- caps shared memory at 1 GB (`shm_size` in the compose file),
- spawns only a handful of vehicles for a short run (`num-vehicles`, `ticks`).

`ubuntu-22.04` is pinned rather than `ubuntu-latest` (now 24.04) because the
`lanelet2` bindings the `convert` step depends on only build against Ubuntu
22.04's Boost 1.74.

## Adding a scenario

Scenarios are defined declaratively in the workflow's `matrix.include` list.
Because the composite action is scenario-agnostic, adding a scenario is a
single matrix entry — no code changes:

```yaml
# .github/workflows/carla-simulation.yml
matrix:
  include:
    - name: my-new-scenario
      map: my_map                       # convert CLI `map=` preset
      input-map-path: path/to/my_map.osm
      num-vehicles: "30"
      ticks: "300"
      min-total-distance: "10.0"
      carla-image: carlasim/carla:0.9.15
      carla-client-version: "0.9.15"
```

The composite action also accepts finer-grained inputs
(`warmup-ticks`, `fixed-delta`, `min-moved-vehicles`, `seed`, `tm-port`,
`extra-args`, …) if a scenario needs them — see the action's `inputs:` block.

## Running locally

You need Docker (with Compose v2) and a converted `.xodr`:

```bash
# 1. Convert a map (from the repository root)
uv run convert map=nishishinjuku target=carla \
  input_map_path=autoware_lanelet2_to_opendrive/test/data/nishishinjuku.osm \
  output_map_path=nishishinjuku.xodr

# 2. Start the CPU-only CARLA server
CARLA_IMAGE=carlasim/carla:0.9.15 \
  docker compose -f autoware_carla_scenario/ci/docker-compose.carla.yml up -d

# 3. Install the matching client and run the simulation
python3.10 -m venv /tmp/carla-venv
/tmp/carla-venv/bin/pip install "carla==0.9.15"
/tmp/carla-venv/bin/python autoware_carla_scenario/ci/carla_traffic_simulation.py \
  --xodr nishishinjuku.xodr --num-vehicles 8 --ticks 120

# 4. Tear down
docker compose -f autoware_carla_scenario/ci/docker-compose.carla.yml down -v
```

The script exits `0` on pass and `1` on failure, and writes a JSON report
(`--output`, default `carla_traffic_sim_result.json`).
