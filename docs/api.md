# API Reference

This page provides comprehensive API documentation for the `autoware-lanelet2-to-opendrive` package. All modules use type hints and follow the Google docstring style.

## Overview

The package provides functionality to convert Lanelet2 map format to OpenDRIVE format. The conversion process involves:

1. **Loading** Lanelet2 maps with MGRS projection
2. **Converting** lanelet structures to OpenDRIVE roads, lanes, and junctions
3. **Exporting** the result to OpenDRIVE XML format

## Core Conversion API

### Main Conversion Function

::: autoware_lanelet2_to_opendrive.main.convert_lanelet2_to_opendrive
    options:
      show_root_heading: true
      heading_level: 4

### Map Loading

::: autoware_lanelet2_to_opendrive.main.load_lanelet2_map
    options:
      show_root_heading: true
      heading_level: 4

### Preprocessing and Conversion Pipeline

::: autoware_lanelet2_to_opendrive.main.preprocess_and_convert_with_hydra
    options:
      show_root_heading: true
      heading_level: 4

#### Usage Example

```python
from pathlib import Path
from autoware_lanelet2_to_opendrive import convert_lanelet2_to_opendrive
from autoware_lanelet2_to_opendrive.main import load_lanelet2_map

# Load a Lanelet2 map
lanelet_map = load_lanelet2_map(
    lanelet2_path=Path("input.osm"),
    mgrs="54SUE815501"
)

# Convert to OpenDRIVE format
opendrive, mapping = convert_lanelet2_to_opendrive(
    lanelet_map=lanelet_map,
    mgrs_code="54SUE815501",
    output_path=Path("output.xodr")
)

# The mapping provides bidirectional road-lanelet relationships
road_id = mapping.get_road_for_lanelet(100)
lanelet_ids = mapping.get_lanelets_for_road(0)
```

## OpenDRIVE Data Models

The package provides comprehensive OpenDRIVE data model classes. These classes are dataclasses that can be serialized to OpenDRIVE XML format.

### Main OpenDRIVE Structure

::: autoware_lanelet2_to_opendrive.opendrive.OpenDRIVE
    options:
      show_root_heading: true
      heading_level: 4

### Header

::: autoware_lanelet2_to_opendrive.opendrive.Header
    options:
      show_root_heading: true
      heading_level: 4

### Road

::: autoware_lanelet2_to_opendrive.opendrive.Road
    options:
      show_root_heading: true
      heading_level: 4
      members:
        - construct_from_lanelet_map
        - construct_connecting_roads_from_junctions
        - set_all_lane_links
        - set_connecting_road_links
        - set_incoming_road_junction_links

### Junction

::: autoware_lanelet2_to_opendrive.opendrive.Junction
    options:
      show_root_heading: true
      heading_level: 4
      members:
        - construct_from_lanelet_groups
        - build_connections_from_roads

### Signal Models

::: autoware_lanelet2_to_opendrive.opendrive.Signal
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.opendrive.signals_and_controllers.SignalsAndControllers
    options:
      show_root_heading: true
      heading_level: 4
      members:
        - construct_from_lanelet_map

## Utility Functions

### Road-Lanelet Mapping

::: autoware_lanelet2_to_opendrive.RoadLaneletMapping
    options:
      show_root_heading: true
      heading_level: 4

### Lanelet Analysis Functions

::: autoware_lanelet2_to_opendrive.find_terminal_lanelets
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.find_adjacent_groups
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.filter_lanelets_by_subtype
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.find_connecting_lanelet_groups
    options:
      show_root_heading: true
      heading_level: 4

#### Usage Example

```python
from autoware_lanelet2_to_opendrive import (
    find_terminal_lanelets,
    find_adjacent_groups,
    filter_lanelets_by_subtype
)

# Find terminal lanelets (start and end points)
start_lanelets, end_lanelets = find_terminal_lanelets(lanelet_map)

# Find groups of adjacent lanelets
road_lanelets = filter_lanelets_by_subtype(
    lanelet_map.laneletLayer,
    ["road"]
)
adjacent_groups = find_adjacent_groups(lanelet_map, road_lanelets)

# Each group represents lanelets that should become a single road
for group in adjacent_groups:
    print(f"Road with {len(group)} lanes: {[ll.id for ll in group]}")
```

### MGRS Projection Functions

::: autoware_lanelet2_to_opendrive.util.mgrs_to_lanelet2_origin
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.util.mgrs_to_proj_string
    options:
      show_root_heading: true
      heading_level: 4

#### Usage Example

```python
from autoware_lanelet2_to_opendrive.util import (
    mgrs_to_lanelet2_origin,
    mgrs_to_proj_string
)

# Convert MGRS code to Lanelet2 origin
origin = mgrs_to_lanelet2_origin("54SUE815501")

# Get PROJ string for OpenDRIVE geoReference
proj_string = mgrs_to_proj_string("54SUE815501")
# Returns: "+proj=utm +zone=54 +lat_0=... +lon_0=... +datum=WGS84 +units=m +no_defs"
```

## Preprocessing API

The preprocessing module provides tools for modifying Lanelet2 maps before conversion.

### Preprocessing Configuration

::: autoware_lanelet2_to_opendrive.preprocess_lanelet.PreprocessOperation
    options:
      show_root_heading: true
      heading_level: 4
      members:
        - from_yaml
        - to_yaml

### Preprocessor

::: autoware_lanelet2_to_opendrive.preprocess_lanelet.LaneletPreprocessor
    options:
      show_root_heading: true
      heading_level: 4
      members:
        - load_map
        - save_map
        - process

#### Usage Example

```python
from pathlib import Path
from autoware_lanelet2_to_opendrive.preprocess_lanelet import (
    PreprocessOperation,
    LaneletPreprocessor
)

# Load preprocessing configuration from YAML
config = PreprocessOperation.from_yaml("preprocess_config.yaml")

# Or create configuration programmatically
from autoware_lanelet2_to_opendrive.preprocess_lanelet import (
    MergeOperation,
    RemoveLaneletOperation
)

config = PreprocessOperation(
    input_map_path="input.osm",
    output_map_path="preprocessed.osm",
    mgrs_code="54SUE815501",
    merge_operations=[
        MergeOperation(lanelet_ids=[100, 101, 102], validate=True)
    ],
    remove_lanelet_operations=[
        RemoveLaneletOperation(lanelet_ids=[200, 201])
    ],
    verbose=True
)

# Execute preprocessing
preprocessor = LaneletPreprocessor(config)
processed_map = preprocessor.process()
```

### Preprocessing Operations

::: autoware_lanelet2_to_opendrive.preprocess_lanelet.MergeOperation
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.preprocess_lanelet.RemoveLaneletOperation
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.preprocess_lanelet.MovePointOperation
    options:
      show_root_heading: true
      heading_level: 4

## OpenDRIVE Export Functions

### XML Export

::: autoware_lanelet2_to_opendrive.opendrive.export_to_xml
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.opendrive.save_opendrive_to_file
    options:
      show_root_heading: true
      heading_level: 4

#### Usage Example

```python
from autoware_lanelet2_to_opendrive.opendrive import (
    save_opendrive_to_file,
    export_to_xml
)
from pathlib import Path

# Save OpenDRIVE object to file
save_opendrive_to_file(opendrive, Path("output.xodr"))

# Or get XML string
xml_string = export_to_xml(opendrive)
```

## Enumerations

### Lane Types

::: autoware_lanelet2_to_opendrive.opendrive.LaneType
    options:
      show_root_heading: true
      heading_level: 4

### Road Mark Types

::: autoware_lanelet2_to_opendrive.opendrive.RoadMarkType
    options:
      show_root_heading: true
      heading_level: 4

### Connection Direction

::: autoware_lanelet2_to_opendrive.ConnectionDirection
    options:
      show_root_heading: true
      heading_level: 4

## Command Line Tools

The package provides two command-line tools:

### Conversion Tool

```bash
# Convert with preprocessing config (recommended)
convert input.osm --preprocess-config config.yaml -o output.xodr

# With verbose output
convert input.osm --preprocess-config config.yaml -o output.xodr --verbose
```

See the [Usage Guide](usage.md) for detailed examples.

### Preprocessing Tool

```bash
# Run preprocessing from config
preprocess-lanelet config.yaml

# Override MGRS code
preprocess-lanelet config.yaml --mgrs 54SUE815501

# Dry run (validation only)
preprocess-lanelet config.yaml --dry-run --verbose
```

## Type Definitions

The package includes full type hints support (indicated by the `py.typed` marker). All public APIs are fully typed for better IDE support and type checking.

### Type Aliases

The package uses several type aliases for clarity:

```python
from typing import Set, List, Dict, Optional, Tuple
import lanelet2

# Lanelet collections
LaneletSet = Set[lanelet2.core.Lanelet]
LaneletList = List[lanelet2.core.Lanelet]
AdjacentGroups = List[Set[lanelet2.core.Lanelet]]

# Road-Lanelet mappings
RoadToLanelets = Dict[int, List[int]]  # road_id -> [lanelet_ids]
LaneletToRoad = Dict[int, int]  # lanelet_id -> road_id
```

## Error Handling

The package uses standard Python exceptions for error handling:

- **FileNotFoundError**: When input files don't exist
- **ValueError**: For invalid parameters or map format issues
- **RuntimeError**: For map loading or conversion failures

Example error handling:

```python
from pathlib import Path
from autoware_lanelet2_to_opendrive.main import load_lanelet2_map

try:
    lanelet_map = load_lanelet2_map(
        Path("input.osm"),
        mgrs="54SUE815501"
    )
except FileNotFoundError as e:
    print(f"Input file not found: {e}")
except ValueError as e:
    print(f"Invalid MGRS code: {e}")
except Exception as e:
    print(f"Map loading failed: {e}")
```

## Related Documentation

- [Usage Guide](usage.md) - Practical examples and workflows
- [Signals Documentation](signals.md) - Traffic signal conversion details
- [Development Guide](development.md) - Contributing guidelines
- [Lanelet2 Documentation](https://github.com/fzi-forschungszentrum-informatik/Lanelet2) - Source format reference
- [OpenDRIVE Specification](https://www.asam.net/standards/detail/opendrive/) - Target format reference
