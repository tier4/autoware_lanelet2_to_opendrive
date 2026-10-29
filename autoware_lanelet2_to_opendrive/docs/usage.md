# Usage Guide

This guide explains how to use the `autoware-lanelet2-to-opendrive` package
to convert Lanelet2 maps to OpenDRIVE format.

## Console Scripts

The package registers five console scripts (see `[project.scripts]` in
[`pyproject.toml`](https://github.com/tier4/autoware_lanelet2_to_opendrive/blob/master/autoware_lanelet2_to_opendrive/pyproject.toml)):

| Script | Purpose |
|--------|---------|
| `convert` | Run the full Lanelet2 → OpenDRIVE pipeline (Hydra CLI) |
| `preprocess-lanelet` | Run preprocessing operations only (argparse CLI) |
| `analyze` | ASAM QC + cross-validate `lanelet → (road, lane)` mapping |
| `qc-validate` | Run ASAM QC checker on an existing `.xodr` |
| `carla-import-test` | Smoke-test loading the `.xodr` in a running CARLA server |

Run them via `uv run <script>` (or `<script>` directly inside an active venv).

## `convert` (Hydra CLI)

`convert` is configured with [Hydra](https://hydra.cc/). Configuration is
composed from three files in
[`src/autoware_lanelet2_to_opendrive/conf/`](https://github.com/tier4/autoware_lanelet2_to_opendrive/blob/master/autoware_lanelet2_to_opendrive/src/autoware_lanelet2_to_opendrive/conf):

```
conf/
├── config.yaml          # base
├── map/                 # map-specific (origin + preprocessing)
│   ├── example.yaml
│   ├── example_mgrs_offset.yaml
│   ├── nishishinjuku.yaml
│   └── odaiba.yaml
└── target/              # simulator-specific
    ├── default.yaml
    └── carla.yaml
```

### Required and Optional Top-Level Keys

| Key | Required | Description |
|-----|----------|-------------|
| `input_map_path` | yes | Path to input Lanelet2 `.osm` file |
| `output_map_path` | no | Output `.xodr` (defaults to `<input>.xodr`) |
| `dry_run` | no | Validate only, no save (default `false`) |
| `verbose` | no | Enable debug logging (default `false`) |
| `map=<name>` | no | Selects `conf/map/<name>.yaml` (default `example`) |
| `target=<name>` | no | Selects `conf/target/<name>.yaml` (default `default`) |

### Basic Conversion

```bash
uv run convert input_map_path=/path/to/map.osm
```

The default `map=example` provides MGRS grid `54SUE815501`. Output path is
`/path/to/map.xodr`.

### Custom Map Configuration

```bash
uv run convert \
  input_map_path=/path/to/nishishinjuku.osm \
  map=nishishinjuku
```

### CARLA-Compatible Output

```bash
uv run convert \
  input_map_path=/path/to/map.osm \
  map=nishishinjuku \
  target=carla
```

The `target=carla` overlay sets `exclude_non_junction_signals: true`, switches
`stopline.carla_stop_line` to `true` (emitting `<object name="Stencil_STOP">`),
and applies traffic-light spawn offsets tuned for CARLA actors.

### Override Output Path

```bash
uv run convert \
  input_map_path=/path/to/map.osm \
  output_map_path=/path/to/output.xodr
```

### Override Individual Keys

Hydra supports dotted overrides:

```bash
uv run convert \
  input_map_path=/path/to/map.osm \
  verbose=true \
  parampoly3.default_segment_length=2.0 \
  stopline.width=0.2
```

### Verbose Output

```bash
uv run convert input_map_path=/path/to/map.osm verbose=true
```

`verbose=true` raises the log level to DEBUG and prints the resolved Hydra
configuration before running.

## Origin Specification

### Canonical: `map_projector_info.yaml`

The preferred way to specify the map origin is a `map_projector_info.yaml`
file placed next to the input `.osm` map (same directory). When present with
a supported `projector_type` (`MGRS` or `TransverseMercator`, matched
case-sensitively as Autoware writes them), it is used as the origin source and
the map's own coordinates are not consulted:

```yaml
projector_type: MGRS
mgrs_grid: 54SUE
```

```yaml
projector_type: TransverseMercator
map_origin:
  latitude: 35.61739731
  longitude: 139.7797546
scale_factor: 0.9996  # optional; defaults to the UTM k0 (0.9996)
```

This is the format Autoware itself writes alongside a map, so pointing the
converter at an Autoware map directory "just works" with no per-map Hydra
config needed for the origin.

### Derived from the map: no configuration at all

With no `map_projector_info.yaml`, the frame is derived from the map's own
node coordinates. Every Lanelet2 `<node>` carries `lat`/`lon`, so the 100 km
MGRS grid square the map occupies follows from the data — the same method
Autoware's own tooling uses. The per-node `mgrs_code` tag that Autoware
tooling writes is *optional*, so it is deliberately not read; only `lat`/`lon`
are.

This is a second canonical route, so it emits no
deprecation notice. It is the only route left for a map without a
projector-info file.

Derivation declines, with a log line saying why, when the map cannot be read
or parsed, carries no georeferenced `<node>` elements, or spans more than one
grid square — the last has no single grid origin, and the log names the
squares it found. With nothing left to fall back to, resolution then fails,
naming both sources it tried.

### Removed: the origin keys

A map config used to be able to declare the frame itself. Those keys are
**gone**, and a config that still carries one is rejected with a message
naming what replaced it ([#564](https://github.com/tier4/autoware_lanelet2_to_opendrive/issues/564);
the soft deprecation that preceded it was
[#539](https://github.com/tier4/autoware_lanelet2_to_opendrive/issues/539)).

| removed key | what to do |
| --- | --- |
| `mgrs_grid` | delete it — the frame comes from `map_projector_info.yaml` or from the map |
| `mgrs_code` | as above; it was the legacy spelling of `mgrs_grid` |
| `lat_lon` | as above |
| `offset` | delete the block; exported coordinates move by exactly the offset it declared |

**They are rejected rather than ignored, deliberately.** A config naming a
grid square its map is not in would otherwise keep converting while the
geoReference it named had no effect, and `offset` was *subtracted from every
exported coordinate*, so ignoring it relocated the map by its own magnitude.

`OriginSpec(mgrs_code=...)` in the Python API is a different thing and is
unaffected: a library caller may still state an origin directly.

## Preprocessing Operations

Preprocessing operations are declared inside a map config and run before
conversion. The execution order is fixed (defined in
`LaneletPreprocessor.process()`):

1. `move_point_operations`
2. `delete_point_operations`
3. `validate_operations`
4. `replace_operations`
5. `merge_operations`
6. `remove_operations` (legacy; prefer `remove_lanelet_operations`)
7. `remove_lanelet_operations`
8. `remove_turn_direction_operations`

Skeleton (commented examples are in `conf/map/example.yaml`):

```yaml
merge_operations:
  - lanelet_ids: [100, 101, 102]
    validate: true
    tolerance: 0.001

remove_lanelet_operations:
  - lanelet_ids: [300, 301]

remove_turn_direction_operations:
  - lanelet_ids: []   # empty list = strip turn_direction from all lanelets
```

See [Conversion Process](conversion-process.md#14-preprocessing-operations-optional)
for the full operation reference.

## Tunable Conversion Parameters

The default target (`conf/target/default.yaml`) exposes the following knobs;
each is also overridable from the command line.

| Section | Key | Default | Effect |
|---------|-----|---------|--------|
| `parampoly3` | `enabled` | `true` | Use dynamic ParamPoly3 segment generation |
| | `min_segment_length` | `0.5` m | Floor for splitting roads into segments; a lanelet shorter than this becomes one straight segment (with a warning) |
| | `default_segment_length` | `1.0` m | Target segment length |
| | `max_segments` / `min_segments` | 100 / 1 | Caps on segment count per road |
| | `coefficient_epsilon` | `1e-8` | Round small coefficients to zero |
| `arcspiral` | `enabled` | `false` | Emit `<line>` / `<arc>` / `<paramPoly3>` mix |
| | `arc_enabled` | `true` | Detect constant-curvature arcs |
| | `min_line_length` | 5.0 m | Reject shorter line runs |
| `stopline` | `width` | `0.1` m | Painted stop-line width |
| | `carla_stop_line` | `false` | Emit `Stencil_STOP` instead of `stopLine` |
| `traffic_light` | `offset_x/y/z`, `hdg_offset` | 0 / 0 / 0 / π/2 | CARLA actor placement |
| `width_estimation` | `adaptive_sampling` | `true` | Sample width based on road length |
| | `default_sample_interval` | 5.0 m | Spacing between width samples |
| `parking_lot` | `enabled` | `true` | Emit synthetic parking-lot roads |
| | `default_stall_width` | `2.5` m | Stall width |
| | `nearest_area_threshold_m` | 30 m | Stall ↔ area association radius |

## `preprocess-lanelet`

A standalone CLI that runs only the preprocessing pipeline and writes the
resulting `.osm`. Useful for validating a config without converting to
OpenDRIVE.

```
preprocess-lanelet config.yaml [--mgrs MGRS] [--origin LAT,LON]
                               [--dry-run] [--verbose]
                               [--output-config OUTPUT_CONFIG]
```

| Flag | Description |
|------|-------------|
| `config` (positional) | YAML config in `PreprocessOperation` schema |
| `--mgrs` | Override MGRS code |
| `--origin LAT,LON` | Override lat/lon origin (comma-separated) |
| `--dry-run` | Run pipeline without saving the `.osm` |
| `-v`, `--verbose` | Enable debug logging |
| `--output-config PATH` | Dump the resolved config to a YAML file |

`--mgrs` and `--origin` are mutually exclusive. The YAML schema mirrors
`PreprocessOperation` (see API reference), e.g.:

```yaml
input_map_path: /path/to/input.osm
output_map_path: /path/to/preprocessed.osm
mgrs_code: 54SUE815501

merge_operations:
  - lanelet_ids: [100, 101, 102]
    validate: true
    tolerance: 0.001
```

## `analyze`

Runs the [ASAM QC OpenDRIVE checker](https://github.com/asam-ev/qc-opendrive)
on a `.xodr` and cross-validates the persisted `lanelet → (road, lane)`
mapping against a live geometric reprojection.

```
analyze <xodr_file> <osm_file> [--output FILE] [--min-severity LEVEL]
                               [--max-issues N] [--ignore-pattern REGEX]
                               [--no-default-ignores]
                               [--fail-on-warning] [--verbose]
```

Both arguments are required. `<osm_file>` is the source Lanelet2 map; the
mapping JSON saved alongside the `.xodr` (named `<stem>_mapping.json`) is
loaded for cross-validation.

Default ignore patterns suppress the known false-positive
`attribute 'rule' is not allowed` reported by the 1.4 schema validator (see
[ASAM Schema Compliance](limitations/asam-schema-compliance.md)). Pass
`--no-default-ignores` to see them.

## `qc-validate`

Thin wrapper that runs only the ASAM QC checker on a standalone `.xodr`. Use
it when you do not have the source `.osm` (or do not need the mapping
cross-check).

```
qc-validate <xodr_file> [--output FILE] [--no-default-ignores] [...]
```

## `carla-import-test`

Connects to a running CARLA server and verifies that the generated `.xodr`
imports cleanly. Only used inside the `carla` docker-compose profile and CI.

```
carla-import-test <xodr_file> --map-name <name> [--host HOST] [--port PORT]
```

## Output File

`convert` writes:

- `<output_map_path>` — the OpenDRIVE 1.4 file
- `<output_map_path stem>_mapping.json` — the persisted mapping used by `analyze`
- `<output_map_path stem>_preprocessed.osm` (only if preprocessing ran) — the
  intermediate Lanelet2 map after preprocessing, so `analyze` can replay the
  same input

OpenDRIVE structure:

```
<OpenDRIVE>
  <header revMajor="1" revMinor="4" .../>      # PROJ geoReference + bounding box
  <road id=... rule="RHT|LHT" .../>            # regular + connecting + parking roads
  <junction id=... .../>                       # intersection + synthetic divergence/merge
  <controller id=... .../>                     # signal groups
</OpenDRIVE>
```

## Using as a Python Library

The package re-exports the main entry points at the top level (see
[`__init__.py`](https://github.com/tier4/autoware_lanelet2_to_opendrive/blob/master/autoware_lanelet2_to_opendrive/src/autoware_lanelet2_to_opendrive/__init__.py)):

```python
from pathlib import Path
import lanelet2

from autoware_lanelet2_to_opendrive import (
    convert_lanelet2_to_opendrive,
    PreprocessOperation,
    LaneletPreprocessor,
    RoadLaneletMapping,
)
from autoware_lanelet2_to_opendrive.main import load_lanelet2_map
from autoware_lanelet2_to_opendrive.projection import mgrs_to_lanelet2_origin
from autoware_lanelet2_to_opendrive.conversion_config import (
    ConversionConfig, OriginSpec,
)

# 1. Load
origin = mgrs_to_lanelet2_origin("54SUE815501")
lanelet_map = load_lanelet2_map(Path("input.osm"), origin)

# 2. Convert
config = ConversionConfig(
    output_path=Path("output.xodr"),
    origin=OriginSpec(mgrs_code="54SUE815501"),
)
opendrive, mapping, lanelet_to_road_and_lane, sl_map, sl_skipped = (
    convert_lanelet2_to_opendrive(lanelet_map, config)
)

# 3. Inspect
print(f"Roads: {len(mapping.road_to_lanelets)}")
print(f"Lanelets: {len(mapping.lanelet_to_road)}")
```

The same five-tuple is what `convert` writes internally before invoking
`analyze`.

## Examples

The repository's `examples/` directory contains:

- `cartesian_to_frenet_demo.py` — driving the `Splines` API to convert
  Cartesian (x, y) into Frenet (s, d) coordinates
- `README_cartesian_to_frenet.md` — companion notes for the demo

## Troubleshooting

### `omegaconf.errors.MissingMandatoryValue: input_map_path` is missing

`config.yaml` marks `input_map_path` as `???` (Hydra "must override"). Pass
it on the command line: `convert input_map_path=/path/to/map.osm`.

### `The map config key '...' is no longer supported`

Delete that key from the map config; the message names what replaced it. See
[Removed: the origin keys](#removed-the-origin-keys).

### `Cannot determine the coordinate frame of ...`

The map has no sibling `map_projector_info.yaml` and no usable node
coordinates. The log above the error says why derivation declined — most
often the map is unparseable, or it spans more than one MGRS grid square.
Add a `map_projector_info.yaml` next to it.

### `projector_type '...' is not supported`

The map's `map_projector_info.yaml` names a projection this converter cannot
build. Only `MGRS` and `TransverseMercator` are wired. The frame is *not*
derived from the map instead, because that would export it in a projection it
does not declare.

### `Failed to load Lanelet2 map`

Verify the `.osm` is a Lanelet2 map (not a vanilla OSM extract) and that the
MGRS / lat-lon origin matches it. Re-run with `verbose=true` for the full
stack trace.

## Next Steps

- [API Reference](api.md) — class-level documentation
- [Conversion Process](conversion-process.md) — pipeline internals
- [Known Limitations](limitations/index.md) — caveats by topic
