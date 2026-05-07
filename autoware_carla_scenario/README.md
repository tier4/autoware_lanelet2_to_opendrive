# Autoware CARLA Scenario

A CARLA scenario testing framework for [Autoware](https://www.autoware.org/). Drives an Autoware ego vehicle through configurable scenarios in the [CARLA simulator](https://carla.org/), evaluates pass/fail conditions tick-by-tick, and produces JSON results plus replayed video.

This package is a workspace member of [`autoware_lanelet2_to_opendrive`](../README.md) and is typically run against OpenDRIVE maps produced by the sibling `convert` CLI.

## Features

- Automated scenario execution against CARLA UE5 (`0.10.0`) and legacy UE4 (`0.9.16`) — selected via mutually exclusive `carla` / `carla-0-9-16` extras.
- [Hydra](https://hydra.cc/)-composed configurations for map, server, ego, entities, and scenario.
- Glob-pattern batch execution of multiple scenarios in a single CARLA session.
- Condition system: timing, collisions, traffic signals, speed/standstill, lane/area position, waypoint crossing, plus logical (`And`/`Or`/`Not`), latching (`Sticky`), and persistent combinators.
- Action system: turns, lane changes, traffic-light state, on-demand camera attachment.
- Coordinate transforms between Lanelet2, OpenDRIVE, and CARLA world frames.
- Hydra `lanelet_constraint` sweeper plugin for parametric map-driven sweeps (resolvable without CARLA).
- FastAPI + Uvicorn web viewer for browsing results, replaying videos, and triggering runs.
- Two-pass video recording (CARLA native log → replayed RGB camera → ffmpeg H.264).
- pytest integration via `CarlaScenarioFixture` (auto-skips when `CARLA_EXECUTABLE` is unset).

## Installation

Python 3.10 is required (`>=3.10,<3.11`, locked by CARLA's bindings). Install via the workspace root:

```bash
# From the repository root
uv sync --dev
```

This pulls the default `carla==0.10.0` extra and the local CARLA wheels under `carla_wheels/`. To use the legacy CARLA 0.9.16 build instead:

```bash
uv sync --dev --extra carla-0-9-16
```

The two CARLA extras are declared as conflicting and cannot be installed simultaneously. CARLA's simulator binary itself must be installed separately — see the [CARLA installation guide](https://carla.readthedocs.io/) and the per-package [installation docs](docs/installation.md).

## Quick usage

The package provides three CLI entry points:

| Command | Framework | Purpose |
|---------|-----------|---------|
| `scenario` | Hydra | Run autonomous-driving scenario tests in CARLA |
| `detect-no-3d-model` | argparse | Detect lanelets without a matching 3D ground model in CARLA |
| `viewer` | FastAPI + Uvicorn | Web UI for browsing and monitoring scenario results |

### Run a scenario

Requires a running CARLA server (with `CARLA_EXECUTABLE` exported or a server already listening on the configured host/port).

```bash
# Single scenario
uv run scenario scenario=intersection_passing/straight

# Batch run via glob pattern (sequential, single CARLA session)
uv run scenario scenario='intersection_passing/*'

# Other built-in scenarios
uv run scenario scenario=lane_change/left
uv run scenario scenario=temporary_stop/temporary_stop
uv run scenario scenario=traffic_light_compliance/traffic_light_compliance
```

Results (JSON + optional video) are written under the configured output directory and can be inspected interactively via:

```bash
uv run viewer
```

### Detect missing 3D models

For diagnosing lanelets that do not project onto a CARLA ground mesh:

```bash
uv run detect-no-3d-model --map /path/to/map.xodr
```

## Docker

The repository's multi-stage Docker image exposes a CARLA-import test profile that mirrors CI:

```bash
docker compose --profile carla run --rm carla-import-test
```

See the root [`docs/docker.md`](../docs/docker.md) for prerequisites, profiles, and troubleshooting.

## Documentation

Full guides live under [`docs/`](docs/) and are served by MkDocs:

- [`docs/installation.md`](docs/installation.md) — system requirements, CARLA setup, `uv` install steps.
- [`docs/usage.md`](docs/usage.md) — every CLI command, configuration overrides, glob batch execution, viewer.
- [`docs/architecture.md`](docs/architecture.md) — module layout, scenario lifecycle, condition/action system.
- [`docs/api.md`](docs/api.md) — generated API reference.
- [`docs/development.md`](docs/development.md) — contributing, testing, pre-commit, release flow.
