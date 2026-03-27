# Usage

This guide explains how to use the `autoware-carla-scenario` package to run scenario tests.

## Running Scenarios

### Using the CLI

The package provides a `scenario` command to run predefined scenarios:

```bash
uv run scenario
```

### Available CLI Commands

| Command | Description |
|---------|-------------|
| `scenario` | Run scenario tests |
| `detect-no-3d-model` | Detect lanelets without 3D models |
| `viewer` | Launch the web UI for scenario monitoring |

### Configuration with Hydra

Scenarios are configured using [Hydra](https://hydra.cc/). Configuration files are located in the `examples/conf/` directory.

```bash
# Run with specific configuration overrides
uv run scenario map=nishishinjuku
```

## Web UI

The package includes a web-based UI for monitoring and controlling scenarios:

```bash
uv run viewer
```

## Next Steps

- [API Reference](api.md) - Detailed API documentation
- [Development Guide](development.md) - Contributing and development setup
