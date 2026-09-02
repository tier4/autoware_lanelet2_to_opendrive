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
├── carla_wheels/                     # Local CARLA Python wheels resolved by uv
├── docker/scenario/                  # Container image for a generated scenario package
├── pyproject.toml                    # uv workspace root
└── uv.lock
```

## Quick start

Every dependency installs from a wheel — no apt packages, no C++ toolchain, no
container. The Lanelet2 Python API comes from
[`simple-lanelet2`](https://github.com/hakuturu583/simple_lanelet2), a Rust
reimplementation that ships `lanelet2` and `autoware_lanelet2_extension_python`
under their usual import paths.

```bash
# Just the converter, into a plain virtualenv
python -m venv .venv && .venv/bin/pip install ./autoware_lanelet2_to_opendrive

# Convert a Lanelet2 map to OpenDRIVE
.venv/bin/convert \
  map=nishishinjuku target=carla \
  input_map_path=/path/to/your-map.osm \
  output_map_path=/path/to/your-map.xodr
```

Arguments are passed verbatim to the `convert` CLI ([Hydra](https://hydra.cc/) syntax).

The converter runs on Python 3.10 and newer. `autoware_carla_scenario` is capped
at 3.10 because the CARLA 0.10.0 client is only published as a cp310 wheel, so
the workspace lock is resolved for 3.10.

## Local development (uv)

For source edits and fast iteration, use `uv` on the whole workspace.

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

Every GitHub Actions job is the same command you would run locally — CI installs
Python, installs uv, runs `uv sync`, and nothing else:

```bash
uv run pytest -n auto                # full pytest suite
uv run pre-commit run --all-files    # lint & format
uv run convert map=nishishinjuku target=carla \
  input_map_path=autoware_lanelet2_to_opendrive/test/data/nishishinjuku.osm \
  output_map_path=nishishinjuku_carla.xodr
uv run qc-validate nishishinjuku_carla.xodr          # ASAM QC against the fixture
uv run carla-import-test nishishinjuku_carla.xodr --map-name nishishinjuku
```

[`pre-commit`](https://pre-commit.com/) hooks (`ruff`, `ruff-format`, `mypy`, plus standard hygiene checks) are mandatory for every commit; install once with `uv run pre-commit install`. Run `uv run pre-commit run --all-files` before pushing to avoid CI formatting failures.

## Documentation

- Per-package guides served by MkDocs and published to GitHub Pages:
  - [Autoware Lanelet2 to OpenDRIVE](https://tier4.github.io/autoware_lanelet2_to_opendrive/) — installation, usage, configuration reference, signals, signs, junctions, geometry classification.
  - [Autoware CARLA Scenario](https://tier4.github.io/autoware_lanelet2_to_opendrive/carla-scenario/) — installation, usage, architecture, API reference, development guide.
- Repository-level references:
  - [`examples/README_cartesian_to_frenet.md`](examples/README_cartesian_to_frenet.md) — Cartesian ↔ Frenet conversion example.
  - [`autoware_carla_scenario/docs/docker.md`](autoware_carla_scenario/docs/docker.md) — packing a generated scenario package into a container image with a pinned CARLA client.
- [`CLAUDE.md`](CLAUDE.md) — project conventions and guidelines for working with this repository.

## Contributing

Pull requests should follow the project [pull request template](.github/PULL_REQUEST_TEMPLATE.md) and include exactly one of the `bump patch` / `bump minor` / `bump major` labels (enforced by the `Check Version Bump Label` action). Bug reports and feature requests have dedicated templates under [`.github/ISSUE_TEMPLATE/`](.github/ISSUE_TEMPLATE/).
