# Autoware lanelet2 to OpenDRIVE

## Installation

```bash
uv pip install -e .
```

## Development

### Spec Driven Development

This project uses Spec Driven Development workflow powered by [@pimzino/spec-workflow-mcp](https://www.npmjs.com/package/@pimzino/spec-workflow-mcp). This MCP (Model Context Protocol) server helps manage the specification-driven development process, ensuring systematic feature development from requirements through implementation.

## How to use

This tool uses [Hydra](https://hydra.cc/) for configuration management, allowing you to compose configurations from multiple YAML files.

### Basic usage

```bash
uv run python -m autoware_lanelet2_to_opendrive.main input_map_path=/path/to/map.osm
```

### With CARLA target

For CARLA compatibility (excludes traffic signals not associated with junctions):

```bash
uv run python -m autoware_lanelet2_to_opendrive.main \
  input_map_path=/path/to/map.osm \
  target=carla
```

### With Nishishinjuku map configuration

```bash
uv run python -m autoware_lanelet2_to_opendrive.main \
  input_map_path=/path/to/nishishinjuku.osm \
  map=nishishinjuku
```

### With Nishishinjuku map + CARLA target

```bash
uv run python -m autoware_lanelet2_to_opendrive.main \
  input_map_path=/path/to/nishishinjuku.osm \
  map=nishishinjuku \
  target=carla
```

### Override output path

```bash
uv run python -m autoware_lanelet2_to_opendrive.main \
  input_map_path=/path/to/map.osm \
  output_map_path=/path/to/output.xodr
```

## Configuration structure

The configuration is managed using Hydra with the following structure:

```
conf/
├── config.yaml           # Base configuration
├── map/
│   ├── example.yaml      # Example map template
│   └── nishishinjuku.yaml # Nishishinjuku map configuration
└── target/
    ├── default.yaml      # Default target settings
    └── carla.yaml        # CARLA-specific settings
```

### Base configuration (`conf/config.yaml`)

```yaml
defaults:
  - map: example
  - target: default
  - _self_

input_map_path: ???  # Required: path to input Lanelet2 OSM file
output_map_path: null  # Optional: output file path
dry_run: false
verbose: false
```

### Map configuration (`conf/map/*.yaml`)

Map-specific configurations contain:
- `mgrs_code`: MGRS grid code for coordinate projection (required)
- Preprocessing operations (optional):
  - `merge_operations`: Merge multiple lanelets
  - `remove_operations`: Remove specific lanelets
  - `replace_operations`: Replace lanelets with merged versions
  - `validate_operations`: Validate lanelet continuity
  - `move_point_operations`: Adjust individual point positions
  - `delete_point_operations`: Remove points from linestrings
  - `remove_lanelet_operations`: Completely remove lanelets
  - `remove_turn_direction_operations`: Remove turn_direction attributes

Example (`conf/map/example.yaml`):
```yaml
mgrs_code: 54SUE815501

merge_operations:
  # - lanelet_ids: [100, 101, 102]
  #   validate: true
  #   tolerance: 0.001

remove_operations:
  # - lanelet_ids: [300, 301]
```

### Target configuration (`conf/target/*.yaml`)

Target-specific settings for different simulators:

**Default** (`conf/target/default.yaml`):
```yaml
exclude_non_junction_signals: false
```

**CARLA** (`conf/target/carla.yaml`):
```yaml
exclude_non_junction_signals: true  # CARLA requires signals to be in junctions
```

## Creating custom configurations

### Adding a new map configuration

1. Create a new file in `conf/map/`, e.g., `conf/map/my_city.yaml`
2. Add the required `mgrs_code` and any preprocessing operations
3. Use it with: `map=my_city`

### Adding a new target configuration

1. Create a new file in `conf/target/`, e.g., `conf/target/my_simulator.yaml`
2. Add simulator-specific settings
3. Use it with: `target=my_simulator`

## Command-line overrides

Hydra allows overriding any configuration value from the command line:

```bash
# Override MGRS code
uv run python -m autoware_lanelet2_to_opendrive.main \
  input_map_path=/path/to/map.osm \
  mgrs_code=54SUE123456

# Enable verbose mode
uv run python -m autoware_lanelet2_to_opendrive.main \
  input_map_path=/path/to/map.osm \
  verbose=true

# Dry run (validate without saving)
uv run python -m autoware_lanelet2_to_opendrive.main \
  input_map_path=/path/to/map.osm \
  dry_run=true
```
