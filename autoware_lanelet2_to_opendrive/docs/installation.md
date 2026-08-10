# Installation

This guide explains how to install the `autoware-lanelet2-to-opendrive` package.

## System Requirements

### Operating System

Any Linux distribution with a recent `pip`. Nothing is compiled at install
time: every dependency, including the Lanelet2 bindings, resolves to a
prebuilt wheel. macOS and Windows are untested but not excluded by anything in
the dependency set.

### Python Version

- **Python 3.10 or newer** — `requires-python = ">=3.10"` in `pyproject.toml`.

The repository's `uv.lock` is resolved for 3.10 because the sibling
`autoware_carla_scenario` package is capped there by the CARLA 0.10.0 client
wheel, which is published for CPython 3.10 only. That cap does not apply when
this package is installed on its own, and CI exercises it on 3.11 and 3.12.

Python 3.13 works, but is not wheels-only yet: `asam-qc-opendrive` caps `numpy`
below 2.0 and `numpy` 1.26 publishes no cp313 wheel, so a 3.13 install compiles
numpy from source.

### Package Manager

- **pip**, or **uv** 0.9.7 or newer for workspace development.

## Installing the Package

### Standalone (pip)

```bash
python -m venv .venv
.venv/bin/pip install ./autoware_lanelet2_to_opendrive
```

The Lanelet2 Python API comes from
[`simple-lanelet2`](https://github.com/hakuturu583/simple_lanelet2), a Rust
reimplementation of the Lanelet2 Python API that ships `lanelet2` and
`autoware_lanelet2_extension_python` under their usual import paths in one
wheel. There is no Boost, no GeographicLib and no C++ toolchain in the loop.

### From source (uv workspace)

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh   # if uv is not installed

git clone https://github.com/tier4/autoware_lanelet2_to_opendrive.git
cd autoware_lanelet2_to_opendrive

# Sync workspace dependencies from uv.lock (reproducible).
uv sync --dev
```

The repository is a `uv` workspace containing two members
(`autoware_lanelet2_to_opendrive` and `autoware_carla_scenario`); a single
`uv sync` installs both.

To run any of the console scripts:

```bash
uv run convert input_map_path=/path/to/map.osm
uv run preprocess-lanelet config.yaml
uv run analyze output.xodr input.osm
uv run qc-validate output.xodr
uv run carla-import-test output.xodr --map-name my_map
```

### CARLA extra (optional)

The `carla` Python wheel is required only by the `autoware_carla_scenario`
workspace member (used by `carla-import-test`). The optional extra is declared
on that workspace member, not on this package; the bundled wheels live under
`carla_wheels/`:

```bash
# CARLA 0.10.0 (not published to PyPI; the cp310 wheel is vendored in-tree)
uv sync --dev --extra carla

# Or pin to CARLA 0.9.16 (mutually exclusive with `carla`)
uv sync --dev --extra carla-0-9-16
```

`uv sync --dev` (without an extra) is sufficient for converting maps,
running unit tests, building docs, and the QC pipeline; the extra is only
needed when actually importing the `carla` Python package.

## Verifying Installation

```bash
uv run python -c "import autoware_lanelet2_to_opendrive; print('OK')"
uv run convert --help
```

## Dependencies

The runtime dependencies declared in
[`autoware_lanelet2_to_opendrive/pyproject.toml`](https://github.com/tier4/autoware_lanelet2_to_opendrive/blob/master/autoware_lanelet2_to_opendrive/pyproject.toml)
are:

- `simple-lanelet2` (>=1.1.2) — Lanelet2 Python API plus the Autoware regulatory-element extensions, as a single wheel
- `scipy` (>=1.9.0) — spline fitting and numerical primitives
- `lxml` (>=5.2.2) — OpenDRIVE XML serialization
- `mgrs` (>=1.5.0) — MGRS ↔ lat/lon conversion
- `tqdm` (>=4.67.1) — progress bars
- `pyyaml` (>=6.0.0) — YAML parsing for preprocessing configs
- `hydra-core` (>=1.3.2) — CLI configuration
- `asam-qc-opendrive` (>=1.0.0) — ASAM QC checker (used by `analyze` / `qc-validate`)
- `pyxodr` (>=0.1.3) — OpenDRIVE structural validation

Console scripts registered in `[project.scripts]`:

| Script | Module | Purpose |
|--------|--------|---------|
| `convert` | `autoware_lanelet2_to_opendrive.main:main` | Lanelet2 → OpenDRIVE conversion (Hydra CLI) |
| `preprocess-lanelet` | `autoware_lanelet2_to_opendrive.preprocess_lanelet:main` | Standalone preprocessing of `.osm` files |
| `analyze` | `autoware_lanelet2_to_opendrive.analyze_xodr:main` | ASAM QC + lanelet/road mapping cross-validation |
| `qc-validate` | `autoware_lanelet2_to_opendrive.qc_validate:main` | ASAM QC checker on a standalone `.xodr` |
| `carla-import-test` | `autoware_lanelet2_to_opendrive.carla_import_test:main` | Smoke-test `.xodr` import into a running CARLA server |

## Troubleshooting

### `ModuleNotFoundError: No module named 'lanelet2'`

`lanelet2` is provided by `simple-lanelet2`, not by a package of that name.
Reinstall the environment so the wheel is present:

```bash
rm -rf .venv
uv sync --dev
```

### A stale `lanelet2` from PyPI shadows the bindings

Installing the PyPI `lanelet2` distribution alongside `simple-lanelet2` puts two
different implementations at the same import path, and which one wins depends on
install order. Do not add it; if it is already there, reset the venv as above.

## Next Steps

- [Usage Guide](usage.md) — run conversions
- [Development Guide](development.md) — contributing workflow
