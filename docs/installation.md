# Installation

This guide will help you install the `autoware-lanelet2-to-opendrive` package.

## System Requirements

### Operating System
This package is compatible with:
- **Linux** (Ubuntu 20.04 or later recommended)
- **macOS** (11.0 or later)
- **Windows** (10 or later)

### Python Version
- **Python 3.10 or higher** - Check your version with `python --version`

### Package Manager
- **uv** (version 0.9.7+) - Modern Python package manager (recommended)
- **pip** - Traditional Python package installer (alternative)

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

### Method 1: Using pip (Simple Installation)

If you prefer using traditional pip:

```bash
# Install from GitHub repository
pip install git+https://github.com/tier4/autoware_lanelet2_to_opendrive.git

# Or install in editable mode for development
git clone https://github.com/tier4/autoware_lanelet2_to_opendrive.git
cd autoware_lanelet2_to_opendrive
pip install -e .
```

### Method 2: Using uv (Recommended)

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

### Method 3: From Source with uv sync (Best for Development)

If you're setting up a development environment with exact dependency versions:

```bash
# Clone the repository
git clone https://github.com/tier4/autoware_lanelet2_to_opendrive.git
cd autoware_lanelet2_to_opendrive

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

### Installing lanelet2

The `lanelet2` package is a critical dependency. You can install it using one of the following methods:

#### Method 1: Using apt (Ubuntu/Debian)

For Ubuntu users, lanelet2 is available from the ROS repositories:

```bash
# Add ROS repository (if not already added)
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros2-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Update package list
sudo apt update

# Install lanelet2
sudo apt install ros-humble-lanelet2  # For ROS 2 Humble
# or
sudo apt install ros-rolling-lanelet2  # For ROS 2 Rolling
```

#### Method 2: Building from Source

If you need the latest version or apt installation is not available:

```bash
# Install dependencies
sudo apt install cmake build-essential libboost-all-dev libeigen3-dev libgeographic-dev libpugixml-dev

# Clone and build lanelet2
git clone https://github.com/fzi-forschungszentrum-informatik/Lanelet2.git
cd Lanelet2
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

For more details, see the [official Lanelet2 documentation](https://github.com/fzi-forschungszentrum-informatik/Lanelet2).

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
