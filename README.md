# Autoware Lanelet2 to OpenDRIVE

A Python workspace for converting [Lanelet2](https://github.com/fzi-forschungszentrum-informatik/Lanelet2) HD maps used by [Autoware](https://www.autoware.org/) into the [OpenDRIVE](https://www.asam.net/standards/detail/opendrive/) road-network format, with a companion scenario-testing framework for validating Autoware on the [CARLA](https://carla.org/) simulator.

The repository is a [`uv`](https://docs.astral.sh/uv/) workspace with two packages:

- [`autoware_lanelet2_to_opendrive`](autoware_lanelet2_to_opendrive/) — the `convert` CLI that turns a Lanelet2 `.osm` map into an OpenDRIVE `.xodr` file, with optional Lanelet2 preprocessing, ASAM QC validation, and a CARLA-specific output overlay.
- [`autoware_carla_scenario`](autoware_carla_scenario/) — a Hydra-driven scenario runner that loads the converted map into CARLA, drives an Autoware ego vehicle, evaluates pass/fail conditions, and records video/JSON results. Ships a FastAPI viewer for browsing runs.

## Repository layout

```
.
├── autoware_lanelet2_to_opendrive/   # Lanelet2 → OpenDRIVE converter (workspace member)
├── autoware_carla_scenario/          # CARLA scenario testing framework (workspace member)
├── examples/                         # Standalone usage examples
├── docs/                             # Repository-level documentation (Docker, etc.)
├── carla_wheels/                     # Local CARLA Python wheels resolved by uv
├── Dockerfile                        # Multi-stage image: dev / convert
├── docker-compose.yml                # CI-equivalent profiles: test / lint / qc / carla / dev / convert
├── pyproject.toml                    # uv workspace root
└── uv.lock
```

## Quick start (Docker)

The Docker route mirrors CI exactly and avoids host build issues with the `lanelet2-python-api-for-autoware` C++ wrapper. The dependency is cloned from a public repository, so no GitHub token is needed for the default build; if you want to authenticate (for example, to lift unauthenticated rate limits or to substitute a private fork), export `GH_PAT` before building — `Dockerfile` mounts it as an optional BuildKit secret.

```bash
# Optional: export GH_PAT=ghp_xxx  # only needed for authenticated/forked clones

# Build the slim conversion image (only needed once, or after dependency changes)
docker compose --profile convert build convert

# Convert a Lanelet2 map to OpenDRIVE. Mount the directory holding your map at /io.
docker run --rm -v "$PWD:/io" l2o-convert:local \
  map=nishishinjuku target=carla \
  input_map_path=/io/your-map.osm \
  output_map_path=/io/your-map.xodr
```

Arguments are passed verbatim to the `convert` CLI ([Hydra](https://hydra.cc/) syntax). See [`docs/docker.md`](docs/docker.md) for the full Docker reference, image targets, named volumes, and troubleshooting.

## Local development (uv)

For source edits and fast iteration, use `uv` directly. Note that the runtime dependency `lanelet2-python-api-for-autoware` builds from source against system Boost, so host installation is sensitive to the OS — Ubuntu 22.04 with Boost 1.74 (matching CI) is known to work; newer hosts may fail to compile.

```bash
# Install workspace dependencies into a local .venv
uv sync --dev

# Run the converter from source
uv run python -m autoware_lanelet2_to_opendrive.main \
  input_map_path=/path/to/map.osm \
  map=nishishinjuku target=carla

# Run a CARLA scenario (requires CARLA installed via an extra and a running server)
uv sync --dev --extra carla     # or --extra carla-0-9-16 for the legacy build
uv run scenario scenario=intersection_passing/straight
```

For full CLI options, configuration layout, and preprocessing operations, see the per-package READMEs:

- [`autoware_lanelet2_to_opendrive/README.md`](autoware_lanelet2_to_opendrive/README.md)
- [`autoware_carla_scenario/README.md`](autoware_carla_scenario/README.md)

## Development & CI

Each GitHub Actions job has a matching Docker Compose profile so the same checks run identically on a developer machine:

```bash
docker compose --profile test  run --rm pytest          # full pytest suite
docker compose --profile lint  run --rm lint            # pre-commit on all files
docker compose --profile qc    run --rm qc-validate     # ASAM QC against fixture
docker compose --profile carla run --rm carla-import-test
docker compose --profile dev   run --rm dev             # interactive shell
```

Static checks that do not import the workspace (`ruff`, `ruff-format`, `mypy --ignore-missing-imports` on individual files) can also be run on the host for fast feedback.

[`pre-commit`](https://pre-commit.com/) hooks (`ruff`, `ruff-format`, `mypy`, plus standard hygiene checks) are mandatory for every commit; install once with `uv run pre-commit install`. Run `uv run pre-commit run --all-files` before pushing to avoid CI formatting failures.

## Documentation

- Per-package guides served by MkDocs:
  - `autoware_lanelet2_to_opendrive/docs/` — installation, usage, configuration reference, signals, signs, junctions, geometry classification.
  - `autoware_carla_scenario/docs/` — installation, usage, architecture, API reference, development guide.
- Repository-level references:
  - [`docs/docker.md`](docs/docker.md) — Docker build & test environment.
  - [`examples/README_cartesian_to_frenet.md`](examples/README_cartesian_to_frenet.md) — Cartesian ↔ Frenet conversion example.
- [`CLAUDE.md`](CLAUDE.md) — project conventions and guidelines for working with this repository.

## Contributing

Pull requests should follow the project [pull request template](.github/PULL_REQUEST_TEMPLATE.md) and include exactly one of the `bump patch` / `bump minor` / `bump major` labels (enforced by the `Check Version Bump Label` action). Bug reports and feature requests have dedicated templates under [`.github/ISSUE_TEMPLATE/`](.github/ISSUE_TEMPLATE/).
