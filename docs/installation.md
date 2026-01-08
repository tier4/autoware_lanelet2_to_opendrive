# Installation

This guide will help you install the `autoware-lanelet2-to-opendrive` package.

## System Requirements

### Operating System
This package is compatible with:
- **Linux** (Ubuntu 20.04 or later recommended)

### Python Version
- **Python 3.10 or higher** - Check your version with `python --version`

### Package Manager
- **uv** (version 0.9.7+) - Modern Python package manager

## Installing uv

If you don't have `uv` installed yet, you can install it using curl:

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

For more installation options, visit the [official uv documentation](https://docs.astral.sh/uv/).

## Installing the Package

### Method 1: Using uv (Recommended)

Using `uv` provides faster installation and better dependency resolution:

#### For Users (Standard Installation)

```bash
# Install from GitHub repository
uv pip install git+https://github.com/tier4/autoware_lanelet2_to_opendrive.git
```

#### For Developers (Editable Installation)

1. Clone the repository:
```bash
git clone https://github.com/tier4/autoware_lanelet2_to_opendrive.git
cd autoware_lanelet2_to_opendrive
```

2. Install the package in editable mode:
```bash
uv pip install -e .
```

### Method 2: From Source with uv sync (Best for Development)

If you're setting up a development environment with exact dependency versions:

```bash
# Clone the repository
git clone https://github.com/tier4/autoware_lanelet2_to_opendrive.git
cd autoware_lanelet2_to_opendrive

# Create and activate virtual environment
uv venv
source .venv/bin/activate  # On Linux

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

The package has the following dependencies (automatically installed):

### Core Dependencies
- **lanelet2** (>=1.2.2) - Core library for working with Lanelet2 map format
- **lanelet2-python-api-for-autoware** - Python API for Autoware's Lanelet2
- **scipy** (>=1.9.0) - Scientific computing library
- **lxml** (>=6.0.2) - XML processing library
- **mgrs** (>=1.5.0) - MGRS coordinate conversion
- **tqdm** (>=4.67.1) - Progress bar utility
- **pyyaml** (>=6.0.0) - YAML parser

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
