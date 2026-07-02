# API Reference

This page documents the public Python API of
`autoware-lanelet2-to-opendrive`. The package ships a `py.typed` marker;
all public surfaces are typed and follow Google-style docstrings.

The canonical entry point for programmatic conversion is
[`convert_lanelet2_to_opendrive`](#core-conversion-api), which mirrors what
the `convert` console script runs internally. The API is organised by
module below; the highlighted symbols are re-exported from the top-level
`autoware_lanelet2_to_opendrive` package (see
[`__init__.py`](https://github.com/tier4/autoware_lanelet2_to_opendrive/blob/master/autoware_lanelet2_to_opendrive/src/autoware_lanelet2_to_opendrive/__init__.py)).

---

## Core Conversion API

### `convert_lanelet2_to_opendrive`

::: autoware_lanelet2_to_opendrive.main.convert_lanelet2_to_opendrive
    options:
      show_root_heading: true
      heading_level: 4

### `load_lanelet2_map`

::: autoware_lanelet2_to_opendrive.main.load_lanelet2_map
    options:
      show_root_heading: true
      heading_level: 4

### `preprocess_and_convert_with_hydra`

::: autoware_lanelet2_to_opendrive.main.preprocess_and_convert_with_hydra
    options:
      show_root_heading: true
      heading_level: 4

### `parse_origin_from_config`

::: autoware_lanelet2_to_opendrive.main.parse_origin_from_config
    options:
      show_root_heading: true
      heading_level: 4

#### Usage example

```python
from pathlib import Path

from autoware_lanelet2_to_opendrive import convert_lanelet2_to_opendrive
from autoware_lanelet2_to_opendrive.main import load_lanelet2_map
from autoware_lanelet2_to_opendrive.projection import mgrs_to_lanelet2_origin
from autoware_lanelet2_to_opendrive.conversion_config import (
    ConversionConfig,
    OriginSpec,
)

origin = mgrs_to_lanelet2_origin("54SUE815501")
lanelet_map = load_lanelet2_map(Path("input.osm"), origin)

config = ConversionConfig(
    output_path=Path("output.xodr"),
    origin=OriginSpec(mgrs_code="54SUE815501"),
)
opendrive, mapping, lanelet_to_road_and_lane, sl_map, sl_skipped = (
    convert_lanelet2_to_opendrive(lanelet_map, config)
)

print(f"Roads: {len(mapping.road_to_lanelets)}")
print(f"Lanelets: {len(mapping.lanelet_to_road)}")
```

`convert_lanelet2_to_opendrive` returns a 5-tuple:

1. `OpenDRIVE` dataclass instance (already saved to disk if
   `config.output_path` is set);
2. `RoadLaneletMapping` (bidirectional `road_to_lanelets` /
   `lanelet_to_road`);
3. `Dict[int, Tuple[int, int]]` — lanelet ID → `(road_id, lane_id)` for
   every emitted lane;
4. `Dict[int, StopLineMappingEntry]` — successfully exported stop lines;
5. `Dict[int, SkippedStopLineEntry]` — stop lines that were skipped.

---

## Conversion Configuration

The `conversion_config` module groups all converter parameters into typed
dataclasses.

::: autoware_lanelet2_to_opendrive.conversion_config.ConversionConfig
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.conversion_config.OriginSpec
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.conversion_config.ParamPoly3Config
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.conversion_config.ArcSpiralConfig
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.conversion_config.WidthEstimationConfig
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.conversion_config.StopLineConfig
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.conversion_config.TrafficLightConfig
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.conversion_config.ParkingLotConfig
    options:
      show_root_heading: true
      heading_level: 4

The static-tuning constants used internally (numerical tolerances, default
spline weights, preprocessing tolerances) live in
[`config.py`](https://github.com/tier4/autoware_lanelet2_to_opendrive/blob/master/autoware_lanelet2_to_opendrive/src/autoware_lanelet2_to_opendrive/config.py)
and are accessed via the global `DEFAULT_CONFIG` instance — see the
"Constants Configuration" section in `CLAUDE.md` for the rationale.

---

## OpenDRIVE Data Models

The `opendrive` subpackage holds the OpenDRIVE 1.4 dataclasses plus the
serializer. Re-exported names (from
[`opendrive/__init__.py`](https://github.com/tier4/autoware_lanelet2_to_opendrive/blob/master/autoware_lanelet2_to_opendrive/src/autoware_lanelet2_to_opendrive/opendrive/__init__.py))
are used directly; the bigger constructor logic for road and junction
objects lives outside this dataclass module.

### Root document

::: autoware_lanelet2_to_opendrive.opendrive.OpenDRIVE
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.opendrive.Header
    options:
      show_root_heading: true
      heading_level: 4

### Geometry

`PlanView`, `Line`, `Arc`, `Spiral`, and `ParamPoly3` (the latter via the
`Spiral` / `GeometryBase` chain) are exported from the dataclass module
along with `ElevationProfile` / `Elevation`.

::: autoware_lanelet2_to_opendrive.opendrive.PlanView
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.opendrive.Line
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.opendrive.Arc
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.opendrive.Spiral
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.opendrive.ElevationProfile
    options:
      show_root_heading: true
      heading_level: 4

### Roads, lanes, junctions

The construction logic (lateral grouping, ParamPoly3 fitting, lane-link
resolution, junction priority emission) lives in
`autoware_lanelet2_to_opendrive.opendrive.road.Road` and
`autoware_lanelet2_to_opendrive.opendrive.junction.Junction`. The
identically named names re-exported from
`autoware_lanelet2_to_opendrive.opendrive` come from
`opendrive_dataclass.py` — they are the lightweight dataclasses used by
the serializer.

::: autoware_lanelet2_to_opendrive.opendrive.road.Road
    options:
      show_root_heading: true
      heading_level: 4
      members:
        - construct_from_lanelet_map
        - construct_from_lanelet_groups
        - construct_connecting_roads_from_junctions
        - set_all_lane_links
        - set_connecting_road_links
        - set_incoming_road_junction_links
        - get_half_width_at_s

::: autoware_lanelet2_to_opendrive.opendrive.junction.Junction
    options:
      show_root_heading: true
      heading_level: 4
      members:
        - construct_from_lanelet_map
        - construct_from_lanelet_groups
        - build_connections_from_roads
        - build_priorities_from_regulatory_elements

::: autoware_lanelet2_to_opendrive.opendrive.junction.Connection
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.opendrive.junction.Priority
    options:
      show_root_heading: true
      heading_level: 4

### Signals and controllers

::: autoware_lanelet2_to_opendrive.opendrive.Signal
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.opendrive.Validity
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.opendrive.SignalType
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.opendrive.signal.Dependency
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.opendrive.Controller
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.opendrive.signals_and_controllers.SignalsAndControllers
    options:
      show_root_heading: true
      heading_level: 4
      members:
        - construct_from_lanelet_map

### Road-level objects (crosswalks, stop lines, parking spaces)

::: autoware_lanelet2_to_opendrive.opendrive.objects.CrosswalkObject
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.opendrive.objects.StopLineObject
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.opendrive.objects.find_nearest_road
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.opendrive.objects.find_nearest_road_for_linestring
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.opendrive.parking.construct_parking_roads
    options:
      show_root_heading: true
      heading_level: 4

### Enumerations

::: autoware_lanelet2_to_opendrive.opendrive.LaneType
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.opendrive.RoadMarkType
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.opendrive.GeometryType
    options:
      show_root_heading: true
      heading_level: 4

### XML serialization

::: autoware_lanelet2_to_opendrive.opendrive.export_to_xml
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.opendrive.save_opendrive_to_file
    options:
      show_root_heading: true
      heading_level: 4

---

## Lanelet utilities (`util.py`)

::: autoware_lanelet2_to_opendrive.RoadLaneletMapping
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.ConnectionDirection
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.find_terminal_lanelets
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.find_lanelets_without_next
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.find_lanelets_without_previous
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.find_adjacent_groups
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.find_connecting_lanelet_groups
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.filter_lanelets_by_subtype
    options:
      show_root_heading: true
      heading_level: 4

#### Usage example

```python
from autoware_lanelet2_to_opendrive import (
    filter_lanelets_by_subtype,
    find_adjacent_groups,
    find_terminal_lanelets,
)

start_lanelets, end_lanelets = find_terminal_lanelets(lanelet_map)

road_lanelets = filter_lanelets_by_subtype(lanelet_map.laneletLayer, ["road"])
adjacent_groups = find_adjacent_groups(lanelet_map, road_lanelets)
for group in adjacent_groups:
    print(f"Road with {len(group)} lanes: {[ll.id for ll in group]}")
```

---

## Projection helpers (`projection.py`)

::: autoware_lanelet2_to_opendrive.projection.mgrs_to_lanelet2_origin
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.projection.mgrs_grid_with_offset_to_lanelet2_origin
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.projection.mgrs_grid_with_offset_to_latlon
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.projection.latlon_to_lanelet2_origin
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.projection.mgrs_to_proj_string
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.projection.latlon_to_proj_string
    options:
      show_root_heading: true
      heading_level: 4

---

## Preprocessing API (`preprocess_lanelet.py`)

The preprocessing layer is exposed both as a standalone CLI
(`preprocess-lanelet`) and as a Python API. Configurations can be loaded
from YAML, derived from a Hydra map config, or built up programmatically.

::: autoware_lanelet2_to_opendrive.preprocess_lanelet.PreprocessOperation
    options:
      show_root_heading: true
      heading_level: 4
      members:
        - from_yaml
        - from_hydra_config
        - to_yaml

::: autoware_lanelet2_to_opendrive.preprocess_lanelet.LaneletPreprocessor
    options:
      show_root_heading: true
      heading_level: 4
      members:
        - load_map
        - save_map
        - process

`LaneletPreprocessor.process()` returns
`Tuple[lanelet2.core.LaneletMap, PreprocessingLog]` — the (possibly
modified) map and a structured log of every operation applied.

### Operation dataclasses

::: autoware_lanelet2_to_opendrive.preprocess_lanelet.MergeOperation
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.preprocess_lanelet.ReplaceOperation
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.preprocess_lanelet.ValidateOperation
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.preprocess_lanelet.MovePointOperation
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.preprocess_lanelet.DeletePointOperation
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.preprocess_lanelet.RemoveLaneletOperation
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.preprocess_lanelet.RemoveTurnDirectionOperation
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.preprocess_lanelet.RemoveOperation
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.LatLonOrigin
    options:
      show_root_heading: true
      heading_level: 4

#### Usage example

```python
from autoware_lanelet2_to_opendrive.preprocess_lanelet import (
    LaneletPreprocessor,
    MergeOperation,
    PreprocessOperation,
    RemoveLaneletOperation,
)

config = PreprocessOperation(
    input_map_path="input.osm",
    output_map_path="preprocessed.osm",
    mgrs_code="54SUE815501",
    merge_operations=[MergeOperation(lanelet_ids=[100, 101, 102], validate=True)],
    remove_lanelet_operations=[RemoveLaneletOperation(lanelet_ids=[200, 201])],
    verbose=True,
)
processed_map, log = LaneletPreprocessor(config).process()
```

---

## Geometric mapping (`road_lanelet_geo_mapping.py`)

`analyze` cross-validates the persisted lanelet → (road, lane) mapping
against a fresh geometric reprojection. The relevant public API:

::: autoware_lanelet2_to_opendrive.GeoRoadLaneletMapping
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.MappingMismatchError
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.validate_mapping_consistency
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.validate_and_save_mapping
    options:
      show_root_heading: true
      heading_level: 4

::: autoware_lanelet2_to_opendrive.resolve_map_to_xodr
    options:
      show_root_heading: true
      heading_level: 4

---

## Console scripts

The package exposes five `[project.scripts]` entry points (run via
`uv run <name>` or directly inside an active venv):

| Script | Module | Purpose |
|--------|--------|---------|
| `convert` | `autoware_lanelet2_to_opendrive.main:main` | Hydra-driven Lanelet2 → OpenDRIVE conversion |
| `preprocess-lanelet` | `autoware_lanelet2_to_opendrive.preprocess_lanelet:main` | Apply preprocessing operations and write a new `.osm` (argparse CLI) |
| `analyze` | `autoware_lanelet2_to_opendrive.analyze_xodr:main` | ASAM QC + geometric cross-validation of the persisted lanelet → road mapping |
| `qc-validate` | `autoware_lanelet2_to_opendrive.qc_validate:main` | ASAM QC checker only — useful when no source `.osm` is available |
| `carla-import-test` | `autoware_lanelet2_to_opendrive.carla_import_test:main` | Smoke-test importing the `.xodr` into a running CARLA server |

See [Usage Guide](usage.md) for full flag reference.

---

## Type definitions

Lightweight aliases used across the codebase:

```python
import lanelet2
from typing import Dict, List, Set

LaneletSet = Set[lanelet2.core.Lanelet]
LaneletList = List[lanelet2.core.Lanelet]
AdjacentGroups = List[Set[lanelet2.core.Lanelet]]

# Road / lanelet relationships
RoadToLanelets = Dict[int, List[int]]   # road_id -> [lanelet_ids]
LaneletToRoad = Dict[int, int]          # lanelet_id -> road_id
```

Higher-level result containers (`Point2D`, `Point3D`, `Point`,
`RoadLaneletMapping`, `GeoRoadLaneletMapping`, `StopLineMappingEntry`,
`SkippedStopLineEntry`) are dataclasses defined under `types.py`,
`util.py`, and `road_lanelet_geo_mapping.py`.

---

## Error handling

Public APIs raise standard Python exceptions:

- `FileNotFoundError` — input `.osm`, `.xodr`, or YAML path missing
- `ValueError` — invalid origin specification, invalid `traffic_rule`,
  out-of-range latitude/longitude, mutually exclusive Hydra origin
  fields, malformed YAML schema
- `RuntimeError` / `Exception` — wrapping low-level lanelet2 or lxml
  failures (`load_lanelet2_map`, `save_opendrive_to_file`)
- `MappingMismatchError` — `analyze` / `validate_mapping_consistency`
  detected a disagreement between the persisted mapping and the
  geometric reprojection

```python
from pathlib import Path
from autoware_lanelet2_to_opendrive.main import load_lanelet2_map
from autoware_lanelet2_to_opendrive.projection import mgrs_to_lanelet2_origin

try:
    lanelet_map = load_lanelet2_map(
        Path("input.osm"),
        mgrs_to_lanelet2_origin("54SUE815501"),
    )
except FileNotFoundError as e:
    print(f"Input file not found: {e}")
except ValueError as e:
    print(f"Invalid origin: {e}")
except Exception as e:
    print(f"Map loading failed: {e}")
```

---

## Related documentation

- [Usage Guide](usage.md) — practical examples and CLI flag reference
- [Conversion Process](conversion-process.md) — pipeline internals
- [Signals](signals.md) — traffic-signal conversion details
- [Crosswalk Objects](crosswalk_objects.md), [Stop Line Objects](stop_line_objects.md)
- [Development Guide](development.md) — contribution workflow
- [Lanelet2 docs](https://github.com/fzi-forschungszentrum-informatik/Lanelet2)
- [OpenDRIVE specification](https://www.asam.net/standards/detail/opendrive/)
