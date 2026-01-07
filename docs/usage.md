# Usage Guide

This guide explains how to use the `autoware-lanelet2-to-opendrive` package to convert Lanelet2 maps to OpenDRIVE format.

## Basic Usage

!!! note
    This section will be updated with usage examples once the conversion functionality is implemented.

### Importing the Package

```python
import autoware_lanelet2_to_opendrive
```

## Conversion Process

The typical workflow for converting a Lanelet2 map to OpenDRIVE involves:

1. **Loading the Lanelet2 map** - Read your input map file
2. **Converting to OpenDRIVE** - Apply the conversion logic
3. **Exporting the result** - Save the OpenDRIVE output

## Example Workflow

```python
# Example code will be added here once the API is implemented

# Step 1: Load Lanelet2 map
# lanelet2_map = load_lanelet2_map("path/to/input.osm")

# Step 2: Convert to OpenDRIVE
# opendrive_map = convert_to_opendrive(lanelet2_map)

# Step 3: Save the result
# save_opendrive(opendrive_map, "path/to/output.xodr")
```

## Command-Line Interface

!!! note
    CLI functionality will be documented here once implemented.

## Configuration Options

Details about configuration options and parameters will be added as the package functionality is developed.

## Use Cases

### Autoware Integration

This package is designed for use with Autoware autonomous driving software. Typical use cases include:

- Converting Autoware maps to OpenDRIVE for simulation environments
- Enabling interoperability with tools that use OpenDRIVE format
- Testing and validation across different map formats

### Simulation and Testing

OpenDRIVE maps can be used in various simulation environments:

- CARLA simulator
- LGSVL simulator
- Other OpenDRIVE-compatible simulation platforms

## Best Practices

- **Validate input**: Ensure your Lanelet2 map is well-formed before conversion
- **Check output**: Verify the generated OpenDRIVE file in your target application
- **Report issues**: If you encounter conversion problems, please report them on [GitHub Issues](https://github.com/tier4/autoware_lanelet2_to_opendrive/issues)

## Examples

For practical examples, check the `examples/` directory in the repository.

## Next Steps

- See the [API Reference](api.md) for detailed API documentation
- Check the [Development Guide](development.md) if you want to contribute
