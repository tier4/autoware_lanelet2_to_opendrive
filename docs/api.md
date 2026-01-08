# API Reference

This page provides detailed documentation for the `autoware-lanelet2-to-opendrive` package API.

!!! note
    API documentation will be automatically generated from docstrings once the implementation is complete. This page uses the `mkdocstrings` plugin to generate API documentation from Python source code.

## Overview

The package provides functionality to convert Lanelet2 map format to OpenDRIVE format. The main components include:

- **Conversion functions** - Core conversion logic
- **Data models** - Representations of map elements
- **Utilities** - Helper functions for map processing

## Core Modules

### Main Package

::: autoware_lanelet2_to_opendrive
    options:
      show_root_heading: true
      show_source: true
      heading_level: 3

## Conversion Functions

!!! info
    Detailed API documentation for conversion functions will appear here once implemented.

### Example Structure

```python
def convert_lanelet2_to_opendrive(
    input_path: str,
    output_path: str,
    options: Optional[ConversionOptions] = None
) -> None:
    """
    Convert a Lanelet2 map file to OpenDRIVE format.

    Args:
        input_path: Path to input Lanelet2 map file
        output_path: Path to output OpenDRIVE file
        options: Optional conversion settings

    Returns:
        None

    Raises:
        FileNotFoundError: If input file doesn't exist
        ValueError: If map format is invalid
    """
    pass
```

## Data Models

Documentation for data models and classes used in the conversion process will be added here.

## Type Definitions

The package includes full type hints support (indicated by the `py.typed` marker). All public APIs are fully typed for better IDE support and type checking.

## Error Handling

### Exception Classes

Details about custom exceptions and error handling will be documented here once implemented.

## Usage Examples

For usage examples, see the [Usage Guide](usage.md).

## Development

For information about contributing to the API, see the [Development Guide](development.md).

## Related Documentation

- [Lanelet2 Documentation](https://github.com/fzi-forschungszentrum-informatik/Lanelet2) - Learn about the source format
- [OpenDRIVE Specification](https://www.asam.net/standards/detail/opendrive/) - Learn about the target format
