# Development Guide

This guide is for developers who want to contribute to the `autoware-carla-scenario` project.

## Development Environment Setup

### Prerequisites

- Python 3.10
- uv (version 0.9.7+)
- Git
- CARLA Simulator 0.10.0+

### Setting Up Your Environment

1. **Fork and clone the repository**:
```bash
git clone https://github.com/YOUR_USERNAME/autoware_lanelet2_to_opendrive.git
cd autoware_lanelet2_to_opendrive
```

2. **Create a virtual environment and install dependencies**:
```bash
uv venv
source .venv/bin/activate
uv sync
```

3. **Install pre-commit hooks**:
```bash
pre-commit install
```

## Project Structure

```
autoware_carla_scenario/
├── src/
│   ├── autoware_carla_scenario/      # Main package
│   │   ├── actions/                  # Trigger-driven side effects (turn, lane change, traffic signal, attach camera)
│   │   ├── conditions/               # Pass/fail condition evaluators
│   │   │   └── composition/          # Composed conditions (lane position, area, speed, waypoint, ...)
│   │   ├── coordinate/               # Lanelet2 / OpenDRIVE / CARLA-world transforms, MapManager, snap
│   │   ├── entity/                   # VehicleEntity, EgoVehicle, AutowareEntity, spawn helpers
│   │   ├── examples/                 # Built-in scenarios + Hydra config tree (conf/)
│   │   ├── kinematics/               # Frame-tagged Vector3, velocity, acceleration
│   │   ├── sensor/                   # CameraSensorBase + CARLA RGB camera
│   │   ├── sweeper/                  # Lanelet-constraint sweeper logic
│   │   ├── tools/                    # detect-no-3d-model CLI
│   │   ├── ui/                       # Result viewer (FastAPI + scanner + runner + sweep resolver)
│   │   ├── utils/                    # Stop-line / traffic-light helpers
│   │   ├── camera_recorder.py        # Two-pass video renderer
│   │   ├── constants.py, entity_role.py
│   │   ├── pytest_fixtures.py        # CarlaScenarioFixture
│   │   ├── scenario_base.py          # BaseScenario, EgoConfig
│   │   ├── scenario_queue.py         # ScenarioQueue (batch / retry / cooldown)
│   │   ├── scenario_runner.py        # Single-scenario execution
│   │   └── server.py                 # CarlaServerManager
│   └── hydra_plugins/
│       └── autoware_scenario_sweeper/ # Hydra-discoverable sweeper plugin shim
├── test/                              # Tests (use `-m "not integration"` to skip CARLA-dependent tests)
├── docs/                              # Documentation source (this site)
├── pyproject.toml                     # Project configuration
└── mkdocs.yml                         # Documentation config
```

## Running Tests

```bash
# Run unit tests only (skip the `integration` marker)
uv run pytest -v -m "not integration" autoware_carla_scenario/test/

# Run integration tests too (requires a running CARLA server and CARLA_EXECUTABLE)
uv run pytest -v autoware_carla_scenario/test/
```

The repository defines a single custom pytest marker in the workspace
`pyproject.toml`:

- `integration` — tests that need a live CARLA server. The pre-commit
  hook `pytest-carla` and the CI `test` job both run with
  `-m "not integration"`.

For local verification that mirrors CI exactly, use the Docker
profiles defined in the workspace `docker-compose.yml`:

```bash
# Full pytest suite (matches CI's `test` job)
docker compose --profile test run --rm pytest

# Pre-commit (matches CI's `lint-and-format` job)
docker compose --profile lint run --rm lint
```

See [`docs/docker.md`](https://github.com/tier4/autoware_lanelet2_to_opendrive/blob/master/docs/docker.md)
in the repository root for the full reference.

## Code Style and Quality

This project follows the same coding standards as the parent repository:

- **Type Hints**: All functions and methods must include type annotations
- **Docstrings**: Use Google-style docstrings
- **Code Formatting**: Enforced by Ruff formatter via pre-commit hooks
- **Linting**: Ruff linter with auto-fix enabled
- **Type checking**: `mypy --ignore-missing-imports` over both `src/` and `test/`

The full pre-commit configuration lives in
[`.pre-commit-config.yaml`](https://github.com/tier4/autoware_lanelet2_to_opendrive/blob/master/.pre-commit-config.yaml)
at the repository root and runs the following hooks:

- `trailing-whitespace`, `end-of-file-fixer`, `check-yaml`, `check-toml`,
  `check-added-large-files`, `check-merge-conflict`, `debug-statements`,
  `mixed-line-ending`
- `ruff` (with `--fix`) and `ruff-format` (`astral-sh/ruff-pre-commit`)
- `mypy` (local hook, runs over both packages)
- `pytest-lanelet2` and `pytest-carla` (local hooks; the carla suite
  runs with `-m "not integration"`)

## Building Documentation Locally

```bash
# Serve documentation locally
uv run mkdocs serve --config-file autoware_carla_scenario/mkdocs.yml

# Build documentation
uv run mkdocs build --config-file autoware_carla_scenario/mkdocs.yml
```

Visit `http://127.0.0.1:8000` to view the documentation.

## Contributing

### Contribution Guidelines

1. **Create an issue** first to discuss your proposed changes
2. **Fork the repository** and create a feature branch
3. **Write tests** for your changes
4. **Ensure all tests pass** and code is properly formatted
5. **Submit a pull request** with a clear description

### Commit Message Convention

Follow conventional commit format:

- `feat:` - New features
- `fix:` - Bug fixes
- `docs:` - Documentation changes
- `test:` - Test additions or changes
- `refactor:` - Code refactoring
- `chore:` - Maintenance tasks
