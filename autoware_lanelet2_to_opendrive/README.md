# Autoware Lanelet2 to OpenDRIVE

A Python package and Hydra-based CLI for converting [Lanelet2](https://github.com/fzi-forschungszentrum-informatik/Lanelet2) HD maps used by [Autoware](https://www.autoware.org/) into the [OpenDRIVE 1.4](https://www.asam.net/standards/detail/opendrive/) road-network format, with optional Lanelet2 preprocessing, ASAM QC validation, and a [CARLA](https://carla.org/)-specific output overlay.

This package is a workspace member of the [`autoware_lanelet2_to_opendrive` repository](../README.md). The sibling [`autoware_carla_scenario`](../autoware_carla_scenario/) consumes the converted `.xodr` to drive Autoware scenarios in CARLA.

## Features

- Lanelet2 → OpenDRIVE 1.4 conversion targeted at CARLA (with a `target=carla` overlay).
- [Hydra](https://hydra.cc/)-based CLI (`convert`) composed from `conf/config.yaml`, `conf/map/*.yaml`, and `conf/target/*.yaml`.
- Optional Lanelet2 preprocessing pipeline (merge / replace / remove / move-point / delete-point / remove-turn-direction / validate) configurable from the same YAML.
- Reference-line geometry emitted as `<paramPoly3>` chains, optionally classified into `<line>` / `<arc>` / `<paramPoly3>` runs (`arcspiral.enabled`).
- Crosswalk lanelets emitted as `<object type="crosswalk">` with closed outlines.
- Stop-line linestrings emitted as `<object type="stopLine">` (or CARLA `Stencil_STOP`) with optional `<signal type="294">` and dependencies on associated traffic-light, stop-sign, and yield-sign signals.
- Parking-lot `Area`s emitted as synthetic parking roads with `<lane type="parking">` and `<object type="parkingSpace">` per stall.
- Traffic-light extraction from Autoware regulatory elements with arrow-bulb subtype encoding.
- Right-of-way regulatory elements emitted as `<junction><priority>` records.
- Built-in ASAM QC validation and Lanelet2-to-road geometric cross-validation (`analyze`, `qc-validate`).
- CARLA import smoke test (`carla-import-test`).
- Pure Python 3.10 with full type hints (`py.typed`); managed by `uv`.

## Installation

Python 3.10 is required (`>=3.10,<3.11`, locked by CARLA's bindings). Install via the workspace root:

```bash
# From the repository root
uv sync --dev
```

This pulls `lanelet2-python-api-for-autoware`, which builds from source against the system `libboost-python`. Hosts running Ubuntu 22.04 with Boost 1.74 (matching CI) are known to work; newer hosts may fail to compile, in which case use the [Docker route](#docker) instead.

## Quick usage

The package registers five console scripts (see `[project.scripts]` in [`pyproject.toml`](pyproject.toml)):

| Command | Framework | Purpose |
|---------|-----------|---------|
| `convert` | Hydra | Run the full Lanelet2 → OpenDRIVE pipeline |
| `preprocess-lanelet` | argparse | Run preprocessing operations only |
| `analyze` | argparse | ASAM QC + cross-validate `lanelet → (road, lane)` mapping |
| `qc-validate` | argparse | Run the ASAM QC checker on an existing `.xodr` |
| `carla-import-test` | argparse | Smoke-test loading the `.xodr` in a running CARLA server |

Run them via `uv run <script>` (or `<script>` directly inside an active venv).

### Convert a map

```bash
# Minimal: convert with the default map config
uv run convert input_map_path=/path/to/map.osm

# CARLA-targeted output (excludes traffic signals not associated with junctions)
uv run convert \
  input_map_path=/path/to/map.osm \
  target=carla

# Use a registered map config (e.g. nishishinjuku) and override output path
uv run convert \
  input_map_path=/path/to/nishishinjuku.osm \
  output_map_path=/path/to/output.xodr \
  map=nishishinjuku target=carla
```

Map configs live under [`src/autoware_lanelet2_to_opendrive/conf/map/`](src/autoware_lanelet2_to_opendrive/conf/map/) and contain the `mgrs_code` (or lat/lon origin) plus optional preprocessing operations. Target configs live under [`conf/target/`](src/autoware_lanelet2_to_opendrive/conf/target/) and tune simulator-specific behaviour. See [`docs/usage.md`](docs/usage.md) for the full configuration reference, command-line override syntax, and end-to-end examples.

### Validate and analyze

```bash
# Run ASAM QC on an existing .xodr
uv run qc-validate /path/to/map.xodr

# Run QC plus geometric cross-validation against the source Lanelet2 map
uv run analyze /path/to/map.osm /path/to/map.xodr
```

## Docker

The repository's multi-stage Docker image exposes a slim conversion image and a CI-equivalent test profile:

```bash
# One-shot conversion via the slim image (mount the directory holding your map)
docker run --rm -v "$PWD:/io" l2o-convert:local \
  map=nishishinjuku target=carla \
  input_map_path=/io/your-map.osm \
  output_map_path=/io/your-map.xodr

# Run the full pytest suite in the CI-matching environment
docker compose --profile test run --rm pytest
```

See the root [`docs/docker.md`](../docs/docker.md) for prerequisites, image targets, and troubleshooting.

## Documentation

Full guides live under [`docs/`](docs/) and are served by MkDocs:

- [`docs/installation.md`](docs/installation.md) — system requirements and `uv` install steps.
- [`docs/usage.md`](docs/usage.md) — every CLI command, Hydra configuration layout, preprocessing operations, command-line overrides.
- [`docs/conversion-process.md`](docs/conversion-process.md) — the conversion pipeline stage-by-stage and OpenDRIVE tag usage.
- [`docs/carla_opendrive_lanelet2_mapping.md`](docs/carla_opendrive_lanelet2_mapping.md) — Lanelet2 ↔ OpenDRIVE ↔ CARLA tag-mapping reference.
- [`docs/signals.md`](docs/signals.md), [`docs/crosswalk_objects.md`](docs/crosswalk_objects.md), [`docs/stop_line_objects.md`](docs/stop_line_objects.md) — feature-specific encoding details.
- [`docs/limitations/`](docs/limitations/) — known limitations and behavioural differences.
- [`docs/api.md`](docs/api.md) — generated API reference.
- [`docs/development.md`](docs/development.md) — contributing, testing, pre-commit, release flow.
