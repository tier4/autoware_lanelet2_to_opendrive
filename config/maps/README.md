# Map Configurations

This directory contains map-specific configuration files for the visualization tool and conversion process.

## File Format

Each map configuration is a YAML file with the following structure:

```yaml
# MGRS grid code for the map origin
mgrs_grid: 54SUE

# Offset from MGRS grid origin (in meters)
offset:
  x: 81655.73
  y: 50137.43
  z: 42.49998

# Optional: Traffic rule
traffic_rule: LHT  # LHT (Left-Hand Traffic) or RHT (Right-Hand Traffic)
```

## Usage

### With Visualization Tool

```bash
uv run visualize map.xodr map.osm --map nishishinjuku
```

### With Conversion Tool

Map configs can also be used with the convert command (future feature).

## Creating New Map Configs

1. Determine your map's MGRS grid and offset:
   - Check Lanelet2 OSM file metadata
   - Or use convert command output config

2. Create `{map_name}.yaml` in this directory

3. Use with `--map {map_name}` option

## Available Maps

- **nishishinjuku** - Nishi-Shinjuku (西新宿), Tokyo, Japan
  - MGRS: 54SUE
  - Traffic: LHT (Left-Hand Traffic)
  - Based on test/data/lanelet2_map.osm

## Notes

- Map names should be lowercase with no spaces (use underscores or hyphens)
- MGRS grid codes are case-sensitive (uppercase)
- Offset values are in meters relative to the MGRS grid origin
