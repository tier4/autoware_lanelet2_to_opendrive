# Installation

This guide will help you install the `autoware-carla-scenario` package.

## System Requirements

### Operating System

- **Linux** (Ubuntu 22.04 is the reference; CI uses 22.04 with Boost 1.74)

### Python Version

- **Python 3.10** — `pyproject.toml` pins `requires-python = ">=3.10,<3.11"`
  because the `carla` wheels are built for CPython 3.10 only. Check your
  version with `python --version`.

### CARLA Simulator

The package supports two CARLA Python API versions, exposed as Hydra
extras in `pyproject.toml`:

| Extra | CARLA version | Notes |
|-------|---------------|-------|
| `carla` | `0.10.0` | Default. CARLA UE5 build. |
| `carla-0-9-16` | `0.9.16` | Legacy CARLA UE4 build. |

The two extras are declared as conflicting under `[tool.uv].conflicts`
and cannot be installed simultaneously.

Follow the [CARLA installation guide](https://carla.readthedocs.io/) to
set up the simulator binary itself.

### Package Manager

- **uv** (version 0.9.7+) — modern Python package manager, used as both
  the build backend and the workflow runner.

## Installing uv

If you don't have `uv` installed yet:

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

For more options, see the [official uv documentation](https://docs.astral.sh/uv/).

## Installing the Package

This package is part of a workspace that also contains
`autoware_lanelet2_to_opendrive`. Both are installed together.

### For Developers (Editable Installation)

1. Clone the repository:

    ```bash
    git clone https://github.com/tier4/autoware_lanelet2_to_opendrive.git
    cd autoware_lanelet2_to_opendrive
    ```

2. Sync dependencies from the lock file (uses CARLA `0.10.0` by default):

    ```bash
    uv sync
    ```

3. (Optional) To target legacy CARLA `0.9.16`:

    ```bash
    uv sync --extra carla-0-9-16
    ```

!!! note
    The runtime dependency `lanelet2-python-api-for-autoware` is built
    from source against system Boost. Hosts whose Boost version differs
    from the CI image (Ubuntu 22.04 / Boost 1.74) may fail during
    `uv sync`. In that case, use the Docker workflow described below.

## Container-Based Installation

The repository ships a multi-stage `Dockerfile` and `docker-compose.yml`
at its root. The Docker image pins Ubuntu 22.04 and matches CI exactly,
which avoids the Boost ABI issues mentioned above.

```bash
# Open an interactive development shell (workspace bind-mounted).
docker compose --profile dev run --rm dev

# Run the carla-scenario test suite.
docker compose --profile test run --rm pytest

# Run pre-commit hooks (matches CI's `lint-and-format` job).
docker compose --profile lint run --rm lint
```

A `GH_PAT` environment variable with `repo` scope is required at image
build time because the project depends on the private repository
`tier4/lanelet2_python_api_for_autoware`. See
[`docs/docker.md`](https://github.com/tier4/autoware_lanelet2_to_opendrive/blob/master/docs/docker.md)
in the repository root for the full reference.

## Environment Configuration

Configure your environment by exporting the variables consumed by the
package, or by loading a `.env` file (the package depends on
`python-dotenv`). The most common variables:

| Variable | Required | Description |
|----------|----------|-------------|
| `CARLA_EXECUTABLE` | when `ScenarioQueue` launches its own server | Path to `CarlaUE5.sh`. Pytest tests skip when this is unset. |
| `NISHISHINJUKU_MAP_PATH` | when overwriting the built-in `.xodr` | Path inside the CARLA install where the original `.xodr` lives. |
| `NISHISHINJUKU_XODR_PATH` | optional | Override the default OpenDRIVE file resolved in `conf/map/nishishinjuku.yaml`. |
| `NISHISHINJUKU_LANELET2_PATH` | optional | Override the default Lanelet2 `.osm` path. |
| `VIEWER_BASE_PATH`, `VIEWER_HOST`, `VIEWER_PORT` | optional | `viewer` web-app overrides. |

See the [Usage Guide](usage.md) and [Architecture](architecture.md) for
the full list.

## Verifying Installation

```bash
python -c "import autoware_carla_scenario; print('Installation successful!')"
```

This works without CARLA — running an actual scenario additionally
requires a live CARLA server and a valid `CARLA_EXECUTABLE`.

## Dependencies

The package's runtime dependencies (declared in `pyproject.toml`):

- `pyxodr>=0.1.0` — OpenDRIVE parser used by `MapManager` / `to_opendrive`
- `opencv-python>=4.8` — frame processing for the camera recorder
- `numpy>=1.21`
- `pytest>=9.0.1`
- `python-dotenv>=1.2.2`
- `lanelet2-python-api-for-autoware` — the Lanelet2 binding (provided
  via the workspace; see the note in
  `autoware_lanelet2_to_opendrive/pyproject.toml`)
- `tqdm>=4.67.1`
- `hydra-core>=1.3.2`, `omegaconf>=2.3.0`
- `fastapi>=0.115.0`, `uvicorn[standard]>=0.34.0`, `jinja2>=3.1.0` —
  result viewer
- `pyyaml>=6.0`, `pydantic>=2.0.0`
- `ffmpeg-python>=0.2.0` — H.264 encoder driver for video recording

Plus one of the `carla` / `carla-0-9-16` extras.

## Troubleshooting

### Import Errors

Ensure that:

1. You're using Python 3.10 (`>=3.10,<3.11`).
2. The package was installed in your active environment (`uv sync` from
   the workspace root, or a fresh `uv venv` followed by `uv sync`).
3. If you see `ImportError: ... lanelet2 ...`, your Boost version may
   not match the wheel's expectation — switch to the Docker workflow.

### CARLA Connection Issues

If you cannot connect to the CARLA simulator:

1. Ensure the CARLA server is running.
2. Check that your overrides match: `server.host`, `server.port`,
   `traffic_manager.port`.
3. Verify network connectivity to the CARLA server.

## Next Steps

Once installed, see the [Usage Guide](usage.md) to run scenarios.
