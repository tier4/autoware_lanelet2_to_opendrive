# Installation

This guide will help you install the `autoware-lanelet2-to-opendrive` package.

## Prerequisites

Before installing, ensure you have:

- **Python 3.10 or higher** - Check your version with `python --version`
- **uv** (version 0.9.7+) - Modern Python package manager

## Installing uv

If you don't have `uv` installed yet, you can install it using one of these methods:

### Using pip
```bash
pip install uv
```

### Using curl (Linux/macOS)
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

### Using PowerShell (Windows)
```powershell
powershell -c "irm https://astral.sh/uv/install.ps1 | iex"
```

For more installation options, visit the [official uv documentation](https://docs.astral.sh/uv/).

## Installing the Package

### From Source

1. Clone the repository:
```bash
git clone https://github.com/tier4/autoware_lanelet2_to_opendrive.git
cd autoware_lanelet2_to_opendrive
```

2. Install the package in editable mode:
```bash
uv pip install -e .
```

### Using uv sync (Recommended for Development)

If you're setting up a development environment:

```bash
# Create and activate virtual environment
uv venv
source .venv/bin/activate  # On Linux/macOS
# or
.venv\Scripts\activate  # On Windows

# Sync dependencies from lock file
uv sync
```

This will install all dependencies with exact versions from `uv.lock`, ensuring a reproducible environment.

## Verifying Installation

To verify that the package is installed correctly:

```bash
python -c "import autoware_lanelet2_to_opendrive; print('Installation successful!')"
```

## Dependencies

The package requires:

- **lanelet2** (>=1.2.2) - Core library for working with Lanelet2 map format

All dependencies will be automatically installed when you install the package.

## Troubleshooting

### Import Errors

If you encounter import errors, ensure that:
1. You're using Python 3.10 or higher
2. The package is properly installed in your current Python environment
3. Your virtual environment is activated (if using one)

### Dependency Issues

If you have problems with dependencies:

```bash
# Re-sync dependencies
uv sync --refresh

# Or reinstall from scratch
uv pip uninstall autoware-lanelet2-to-opendrive
uv pip install -e .
```

## Next Steps

Once installed, check out the [Usage Guide](usage.md) to learn how to use the package.
