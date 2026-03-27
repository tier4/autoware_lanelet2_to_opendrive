# Installation

This guide will help you install the `autoware-carla-scenario` package.

## System Requirements

### Operating System

- **Linux** (Ubuntu 20.04 or later recommended)

### Python Version

- **Python 3.10** - Check your version with `python --version`

### CARLA Simulator

- **CARLA 0.10.0 or later** - Follow the [CARLA installation guide](https://carla.readthedocs.io/) to set up the simulator

### Package Manager

- **uv** (version 0.9.7+) - Modern Python package manager

## Installing uv

If you don't have `uv` installed yet:

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

For more installation options, visit the [official uv documentation](https://docs.astral.sh/uv/).

## Installing the Package

### For Developers (Editable Installation)

1. Clone the repository:
```bash
git clone https://github.com/tier4/autoware_lanelet2_to_opendrive.git
cd autoware_lanelet2_to_opendrive
```

2. Sync dependencies from lock file:
```bash
uv sync
```

## Environment Configuration

Copy the example environment file and configure it for your setup:

```bash
cp autoware_carla_scenario/.env.example autoware_carla_scenario/.env
```

Edit the `.env` file to match your CARLA installation and Autoware configuration.

## Verifying Installation

To verify that the package is installed correctly:

```bash
python -c "import autoware_carla_scenario; print('Installation successful!')"
```

## Dependencies

The package has the following key dependencies (automatically installed):

### Core Dependencies

- **carla** (>=0.10.0) - CARLA simulator Python API
- **lanelet2** (>=1.2.2) - Lanelet2 map format library
- **pyxodr** (>=0.1.0) - OpenDRIVE format handling
- **hydra-core** (>=1.3.2) - Configuration management
- **fastapi** (>=0.115.0) - Web UI server
- **opencv-python** (>=4.8) - Video processing
- **pydantic** (>=2.0.0) - Data validation

## Troubleshooting

### Import Errors

If you encounter import errors, ensure that:

1. You're using Python 3.10
2. The package is properly installed in your current Python environment
3. Your virtual environment is activated (if using one)

### CARLA Connection Issues

If you cannot connect to the CARLA simulator:

1. Ensure the CARLA server is running
2. Check your `.env` configuration for correct host and port settings
3. Verify network connectivity between your machine and the CARLA server

## Next Steps

Once installed, check out the [Usage Guide](usage.md) to learn how to run scenarios.
