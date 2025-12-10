# Autoware lanelet2 to OpenDRIVE

## Installation

```bash
uv pip install -e .
```

## Development

### Spec Driven Development

This project uses Spec Driven Development workflow powered by [@pimzino/spec-workflow-mcp](https://www.npmjs.com/package/@pimzino/spec-workflow-mcp). This MCP (Model Context Protocol) server helps manage the specification-driven development process, ensuring systematic feature development from requirements through implementation.

## How to use

### Using the convert command with preprocessing

The `convert` command now supports preprocessing operations through a YAML configuration file:

```bash
uv run convert --preprocess-config <config_file> <lanelet2_map_file>
```

Example:
```bash
uv run convert --preprocess-config test/data/preprocess_config.yaml test/data/lanelet2_map.osm
```

You can also specify a custom output file:
```bash
uv run convert --preprocess-config test/data/preprocess_config.yaml test/data/lanelet2_map.osm -o output.xodr
```

### Configuration file format

The preprocessing configuration file (YAML) must contain:
- `mgrs_code`: MGRS grid code for coordinate projection (required)
- `input_map_path`: Path to the input Lanelet2 map
- `output_map_path`: Path for preprocessed map (optional)
- Preprocessing operations (optional):
  - `merge_operations`: Merge multiple lanelets
  - `remove_operations`: Remove specific lanelets
  - `replace_operations`: Replace lanelets with merged versions
  - `validate_operations`: Validate lanelet continuity

Example configuration file:
```yaml
# Required fields
input_map_path: test/data/lanelet2_map.osm
output_map_path: /tmp/preprocessed.osm  # Optional
mgrs_code: 54SUE815501

# Optional preprocessing operations
merge_operations:
  - lanelet_ids: [100, 101, 102]
    validate: true
    tolerance: 0.001

remove_operations:
  - lanelet_ids: [300, 301]

# Global settings
dry_run: false
verbose: false
```

See `examples/convert_config.yaml` for a complete example.

### Direct Python execution

For direct Python execution:

```bash
python3 src/autoware_lanelet2_to_opendrive/main.py --preprocess-config config.yaml input.osm
```

### Arguments

- `<lanelet2_map_file>`: Path to the input Lanelet2 OSM file
- `--preprocess-config`: Path to preprocessing configuration YAML file (required, contains MGRS code)
- `-o, --output`: Output path for the OpenDRIVE file (default: input_file.xodr)
- `-v, --verbose`: Enable verbose output
