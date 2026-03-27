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
│   └── autoware_carla_scenario/  # Main package
│       ├── actions/              # Scenario actions
│       ├── conditions/           # Condition evaluators
│       ├── coordinate/           # Coordinate transforms
│       ├── entity/               # Entity definitions
│       ├── examples/             # Example scenarios
│       ├── kinematics/           # Motion calculations
│       ├── sweeper/              # Lanelet constraint sweeper
│       ├── tools/                # Utility tools
│       ├── ui/                   # Web UI
│       └── utils/                # Utilities
├── test/                         # Tests
├── docs/                         # Documentation
├── pyproject.toml                # Project configuration
└── mkdocs.yml                    # Documentation config
```

## Running Tests

```bash
# Run all tests
uv run pytest -v autoware_carla_scenario/test/

# Run tests in parallel
uv run pytest -v -n auto autoware_carla_scenario/test/
```

## Code Style and Quality

This project follows the same coding standards as the parent repository:

- **Type Hints**: All functions and methods must include type annotations
- **Docstrings**: Use Google-style docstrings
- **Code Formatting**: Enforced by Ruff formatter via pre-commit hooks
- **Linting**: Ruff linter with auto-fix enabled

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
