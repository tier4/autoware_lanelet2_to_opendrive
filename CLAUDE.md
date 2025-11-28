# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a Python package for converting Lanelet2 map format to OpenDRIVE format, designed for use with Autoware autonomous driving software. The project uses the modern Python packaging tool `uv` for dependency management and builds.

## Development Environment Setup

This project uses `uv` (version 0.9.7+) for Python package management. Ensure uv is installed before working with this codebase.

### Key Commands

```bash
# Install dependencies
uv pip install -e .

# Sync dependencies from lock file
uv sync

# Add a new dependency
uv add <package_name>

# Add a development dependency
uv add --dev <package_name>

# Run Python scripts
uv run python <script.py>

# Create virtual environment (if needed)
uv venv
```

## Project Structure

- `src/autoware_lanelet2_to_opendrive/` - Main Python package directory
  - `__init__.py` - Package initialization (currently contains placeholder code)
  - `py.typed` - PEP 561 marker file indicating the package supports type hints
- `pyproject.toml` - Project configuration and dependencies (uses uv as build backend)
- `uv.lock` - Locked dependency versions for reproducible builds

## Dependencies

- **lanelet2** (>=1.2.2) - Core library for working with Lanelet2 map format
- Python 3.10 or higher is required

## Architecture Notes

This is a converter package designed to transform Lanelet2 map data (commonly used in Autoware and other autonomous driving stacks) into the OpenDRIVE format (an open standard for road network descriptions). The actual conversion logic needs to be implemented in the main package directory.

## Development Guidelines

1. Type hints should be used throughout the codebase (package includes py.typed marker)
2. Follow Python 3.10 syntax and features
3. The package name uses hyphens externally (`autoware-lanelet2-to-opendrive`) but underscores internally (`autoware_lanelet2_to_opendrive`)
4. Use uv for all dependency management operations rather than pip directly
