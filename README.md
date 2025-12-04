# Autoware lanelet2 to OpenDRIVE

## Installation

```bash
uv pip install -e .
```

## Development

### Spec Driven Development

This project uses Spec Driven Development workflow powered by [@pimzino/spec-workflow-mcp](https://www.npmjs.com/package/@pimzino/spec-workflow-mcp). This MCP (Model Context Protocol) server helps manage the specification-driven development process, ensuring systematic feature development from requirements through implementation.

## How to use

### Using the convert command

After installation, you can use the `convert` command:

```bash
uv run convert <lanelet2_map_file> <mgrs_code>
```

Example:
```bash
uv run convert ./test/data/lanelet2_map.osm 54SUE
```

### Direct Python execution

Alternatively, you can run the script directly:

```bash
python3 main.py ./test/data/lanelet2_map.osm 54SUE
```

### Arguments

- `<lanelet2_map_file>`: Path to the Lanelet2 OSM file
- `<mgrs_code>`: MGRS grid code for coordinate conversion (e.g., 54SUE)
