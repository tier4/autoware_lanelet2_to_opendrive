# Usage Guide

This guide explains how to use the `autoware-lanelet2-to-opendrive` package to convert Lanelet2 maps to OpenDRIVE format.

## Basic Usage

### CLI Commands

The package includes two CLI commands:

#### 1. `convert` - Convert from Lanelet2 to OpenDRIVE

```bash
convert input.osm --preprocess-config config.yaml
```

#### 2. `preprocess-lanelet` - Preprocess Lanelet2 Maps

```bash
preprocess-lanelet config.yaml
```

## Convert Command Details

### Basic Conversion Example

The simplest usage:

```bash
convert input_map.osm --preprocess-config config.yaml
```

This command will generate `input_map.xodr` (the default output file name).

### Specify Output File Name

```bash
convert input_map.osm --preprocess-config config.yaml -o output_map.xodr
```

or

```bash
convert input_map.osm --preprocess-config config.yaml --output output_map.xodr
```

### Enable Verbose Logging

```bash
convert input_map.osm --preprocess-config config.yaml -v
```

or

```bash
convert input_map.osm --preprocess-config config.yaml --verbose
```

## Command Line Options

### `convert` Command Options

| Option | Short Form | Description | Required |
|--------|------------|-------------|----------|
| `lanelet2_file` | - | Path to the input Lanelet2 OSM file | ✓ |
| `--preprocess-config` | - | Path to the preprocessing configuration YAML file (contains MGRS code) | ✓ |
| `--output` | `-o` | Path to the output OpenDRIVE file (default: input_file.xodr) | |
| `--verbose` | `-v` | Enable verbose log output | |

### `preprocess-lanelet` Command Options

| Option | Short Form | Description | Required |
|--------|------------|-------------|----------|
| `config` | - | Path to the YAML configuration file | ✓ |
| `--mgrs` | - | MGRS code (overrides configuration file) | |
| `--dry-run` | - | Run without saving output (validation only) | |
| `--verbose` | `-v` | Enable verbose log output | |
| `--output-config` | - | Save loaded configuration to a new YAML file | |

## Input File Requirements

### Lanelet2 Map Requirements

A valid Lanelet2 OSM file is required as input:

- **File Format**: Lanelet2 map in `.osm` format
- **Coordinate System**: Map defined in MGRS coordinate system
- **Required Elements**:
  - Lanelet elements
  - Linestring elements
  - Point elements
- **Attributes**: Standard Lanelet2 attributes for Autoware

### Preprocessing Configuration File (YAML)

A preprocessing configuration file is required for conversion. This file must include the MGRS code and can optionally contain preprocessing operations.

#### Minimal Configuration Example

```yaml
input_map_path: /path/to/input.osm
output_map_path: /path/to/preprocessed.osm
mgrs_code: 54SUE815501

# No preprocessing operations (only use MGRS code for conversion)
```

#### Configuration Example with Preprocessing Operations

```yaml
input_map_path: /path/to/input.osm
output_map_path: /path/to/preprocessed.osm
mgrs_code: 54SUE815501

# Merge lanelets
merge_operations:
  - lanelet_ids: [100, 101, 102]
    validate: true
    tolerance: 0.001

# Remove lanelets
remove_lanelet_operations:
  - lanelet_ids: [300, 301]

# Remove turn_direction attributes (from all lanelets)
remove_turn_direction_operations:
  - lanelet_ids: []  # Empty list = remove from all lanelets

# Global settings
dry_run: false
verbose: true
```

#### Available Preprocessing Operations

1. **merge_operations**: Merge multiple lanelets into one
2. **remove_operations**: Remove lanelets (legacy format)
3. **replace_operations**: Replace lanelets
4. **validate_operations**: Validate lanelet continuity
5. **move_point_operations**: Move point coordinates
6. **delete_point_operations**: Delete points
7. **remove_lanelet_operations**: Remove entire lanelets
8. **remove_turn_direction_operations**: Remove turn_direction attributes

## Output Files

### OpenDRIVE Format

After conversion, an OpenDRIVE format (.xodr) file is generated with the following characteristics:

- **OpenDRIVE Version**: 1.4
- **Coordinate System**: MGRS coordinate system (same as input map)
- **Included Elements**:
  - Roads: Normal roads and junction connecting roads
  - Junctions: Intersection areas and connection information
  - Signals: Traffic signals extracted from Lanelet2 map
  - Controllers: Traffic signal controllers
- **Target**: Optimized for CARLA simulator

### Output Structure

```
output.xodr
├── header (header information and geoReference)
├── roads
│   ├── Normal roads (outside junctions)
│   └── Connecting roads (inside junctions)
├── junctions (intersections and their connections)
└── controllers (traffic signal controllers)
```

## Common Use Cases

### Use Case 1: Simple Conversion

Convert a Lanelet2 map to OpenDRIVE without preprocessing:

```bash
# 1. Create a minimal configuration file (config.yaml)
# Include input_map_path, output_map_path, mgrs_code

# 2. Execute conversion
convert my_map.osm --preprocess-config config.yaml -o my_map.xodr
```

### Use Case 2: Conversion with Preprocessing

Fix map issues before conversion:

```bash
# 1. Create a configuration file with preprocessing operations (nishishinjuku_preprocess_config.yaml)
# Include merge_operations, remove_lanelet_operations, etc.

# 2. Execute preprocessing and conversion in one step
convert original_map.osm --preprocess-config nishishinjuku_preprocess_config.yaml -o fixed_map.xodr
```

### Use Case 3: Run Preprocessing Only

Preprocess a map before OpenDRIVE conversion:

```bash
# 1. Run preprocessing only
preprocess-lanelet nishishinjuku_preprocess_config.yaml

# 2. Validate the preprocessed map
preprocess-lanelet nishishinjuku_preprocess_config.yaml --dry-run -v

# 3. Convert the preprocessed map
convert preprocessed_map.osm --preprocess-config simple_config.yaml
```

### Use Case 4: Autoware + CARLA Simulation

Use an Autoware map in CARLA simulator:

```bash
# 1. Convert Autoware Lanelet2 map
convert autoware_map.osm --preprocess-config config.yaml -o carla_map.xodr

# 2. Import the generated carla_map.xodr into CARLA
# (Refer to CARLA documentation)
```

### Use Case 5: Debugging and Validation

Debug the conversion process using verbose logging:

```bash
# Run with verbose logging
convert input.osm --preprocess-config config.yaml -v -o output.xodr

# This will display the following information:
# - Number of loaded lanelets, linestrings, and points
# - Building normal roads and junction roads
# - Building junction connections
# - Extracting signals and controllers
# - Road-Lanelet mapping information
```

## Autoware Integration

This package is designed for use with Autoware autonomous driving software.

### Typical Workflow

1. **Export Autoware Map**: Prepare your Autoware map in Lanelet2 format
2. **Get MGRS Code**: Confirm the MGRS code corresponding to your map's coordinate system
3. **Create Preprocessing Configuration**: Define necessary map corrections
4. **Convert to OpenDRIVE**: Use the `convert` command
5. **Import to Simulator**: Use the generated OpenDRIVE file in CARLA, etc.

## Simulation and Testing

!!! info "Current Support"
    Currently, this package generates OpenDRIVE format for **CARLA simulator**. Support for other simulation platforms may be added in future releases.

### Using with CARLA Simulator

The generated OpenDRIVE map is compatible with:

- **CARLA Simulator** (primary target)
- Custom map import
- Autonomous driving testing in simulation environments

## Best Practices

### Input Validation

Ensure your Lanelet2 map is in the correct format before conversion:

```bash
# Validate with preprocessing dry-run mode
preprocess-lanelet config.yaml --dry-run -v
```

### Output Verification

Verify the generated OpenDRIVE file in your target application:

- Load the map in CARLA simulator
- Visually confirm with an OpenDRIVE viewer
- Validate the XML structure

### Reporting Issues

If you encounter conversion issues, report them at [GitHub Issues](https://github.com/tier4/autoware_lanelet2_to_opendrive/issues).

When reporting, please include:
- Sample of the input Lanelet2 map (if possible)
- The preprocessing configuration file used
- Error messages or unexpected behavior
- Verbose log output (using `--verbose` flag)

## Using as a Python Library

### Importing the Package

```python
import autoware_lanelet2_to_opendrive
from autoware_lanelet2_to_opendrive.main import (
    load_lanelet2_map,
    convert_lanelet2_to_opendrive,
    preprocess_and_convert
)
```

### Programmatic Conversion

```python
from pathlib import Path
from autoware_lanelet2_to_opendrive.main import preprocess_and_convert

# Execute conversion
preprocess_and_convert(
    lanelet2_file=Path("input_map.osm"),
    output_file=Path("output_map.xodr"),
    preprocess_config_path=Path("config.yaml"),
    verbose=True
)
```

### Advanced Usage Example

```python
from pathlib import Path
from autoware_lanelet2_to_opendrive.main import (
    load_lanelet2_map,
    convert_lanelet2_to_opendrive
)

# 1. Load the map
lanelet_map = load_lanelet2_map(
    Path("input.osm"),
    mgrs="54SUE815501"
)

# 2. Convert to OpenDRIVE
opendrive, mapping = convert_lanelet2_to_opendrive(
    lanelet_map=lanelet_map,
    mgrs_code="54SUE815501",
    output_path=Path("output.xodr")
)

# 3. Use mapping information
print(f"Roads: {len(mapping.road_to_lanelets)}")
print(f"Lanelets: {len(mapping.lanelet_to_road)}")

# Check which road a specific lanelet corresponds to
lanelet_id = 100
if lanelet_id in mapping.lanelet_to_road:
    road_id = mapping.lanelet_to_road[lanelet_id]
    print(f"Lanelet {lanelet_id} -> Road {road_id}")
```

## Examples

For practical usage examples, check the `examples/` directory in the repository.

## Next Steps

- Refer to [API Reference](api.md) for detailed API specifications
- Check [Development Guide](development.md) if you want to contribute to development
- See [Signals Documentation](signals.md) for information on signal and traffic rule conversion

## Troubleshooting

### Common Errors

#### "MGRS code must be provided"

**Cause**: The preprocessing configuration file does not contain an MGRS code

**Solution**:
```yaml
# Add the following to config.yaml
mgrs_code: 54SUE815501  # Replace with actual MGRS code
```

#### "Lanelet2 file not found"

**Cause**: The input file path is incorrect

**Solution**:
- Verify the file path
- Correctly specify absolute or relative path

#### "Failed to load Lanelet2 map"

**Cause**: The map file format is incorrect, or the MGRS code is wrong

**Solution**:
- Confirm the map file is in Lanelet2 OSM format
- Ensure the MGRS code matches the map's coordinate system
- Check details with the `--verbose` flag

### Debugging Tips

1. **Enable Verbose Logging**: Always use the `-v` flag
2. **Execute Step by Step**: Separate preprocessing and conversion
3. **Use Dry-Run Mode**: Validate before making actual changes
4. **Test with Small Maps**: Try with a smaller map first
