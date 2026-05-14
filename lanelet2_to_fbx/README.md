# ll2tofbx

`ll2tofbx` is a CLI tool that converts Lanelet2 `.osm` maps into FBX files.
The converter also writes a JSON report and an execution log so that conversion
results can be checked without opening the FBX file first.

This repository is intended to be used on a shared Linux server through Docker.
Lanelet2 and Blender are installed inside the container, so the host only needs
Docker and access to the input/output files.

## Requirements

- Docker
- Linux `x86_64` is recommended
- Input `.osm` files and output directories should be under the repository
  checkout that contains `lanelet2_to_fbx/`

The Docker helper scripts mount the workspace at the same absolute path inside
the container. Because of that, pass the same host-side absolute paths to
`--input`, `--output`, `--report`, and `--log`.

## Quick Start

Run these commands from the repository root that contains `lanelet2_to_fbx/`.

```bash
REPO_ROOT="$(pwd)"
INPUT_OSM="${REPO_ROOT}/input_map/lanelet2_map.osm"
OUT_DIR="${REPO_ROOT}/out"
mkdir -p "${OUT_DIR}"
```

### 1. Build the Docker image

```bash
bash lanelet2_to_fbx/scripts/docker_build.sh
```

To use a custom image name:

```bash
LL2TOFBX_IMAGE=ll2tofbx:kawai bash lanelet2_to_fbx/scripts/docker_build.sh
```

### 2. Create an output directory

```bash
mkdir -p "${OUT_DIR}"
```

### 3. Export an FBX

```bash
bash lanelet2_to_fbx/scripts/docker_export.sh \
  --input "${INPUT_OSM}" \
  --output "${OUT_DIR}/lanelet2_map.fbx" \
  --report "${OUT_DIR}/lanelet2_map.report.json" \
  --log "${OUT_DIR}/lanelet2_map.log"
```

If the image was built with a custom name, use the same name when exporting:

```bash
LL2TOFBX_IMAGE=ll2tofbx:kawai \
bash lanelet2_to_fbx/scripts/docker_export.sh \
  --input "${INPUT_OSM}" \
  --output "${OUT_DIR}/lanelet2_map.fbx" \
  --report "${OUT_DIR}/lanelet2_map.report.json" \
  --log "${OUT_DIR}/lanelet2_map.log"
```

### 4. Check the result

```bash
ls -lh "${OUT_DIR}"
sed -n '1,240p' "${OUT_DIR}/lanelet2_map.report.json"
```

The main points to check are:

- `success` is `true`
- `validation.errors` is empty
- the `*.fbx` file exists

## Common Conversion Examples

Run these commands from the repository root that contains `lanelet2_to_fbx/`.

```bash
REPO_ROOT="$(pwd)"
INPUT_OSM="${REPO_ROOT}/input_map/lanelet2_map.osm"
ODAIBA_OSM="${REPO_ROOT}/input_map/odaiba_ll2_raw.osm"
OUT_DIR="${REPO_ROOT}/out"
mkdir -p "${OUT_DIR}"
```

### Basic Odaiba conversion

`--lanelet-side-overlap 0.3` keeps the original `lanelet_road` layer and also
adds a `road_extention` layer widened by 0.3 m on both sides. Use
`--keep-intermediate` when the intermediate OBJ/MTL files should be preserved.

```bash
bash lanelet2_to_fbx/scripts/docker_export.sh \
  --input "${ODAIBA_OSM}" \
  --output "${OUT_DIR}/odaiba_ll2_raw_ver4.fbx" \
  --report "${OUT_DIR}/odaiba_ll2_raw_ver4.report.json" \
  --log "${OUT_DIR}/odaiba_ll2_raw_ver4.log" \
  --lanelet-side-overlap 0.3 \
  --keep-intermediate
```

### Export ground layers as flat surfaces

`--surface-style flat` exports ground layers as double-sided surfaces instead
of solid meshes with thickness. This applies to `lanelet_road`,
`road_extention`, `intersection_area`, `hatched_area`, `parking_lot`, and
`shoulder`.

In this mode, `--road-thickness` is ignored.

```bash
bash lanelet2_to_fbx/scripts/docker_export.sh \
  --input "${ODAIBA_OSM}" \
  --output "${OUT_DIR}/odaiba_ll2_raw_flat_ground.fbx" \
  --report "${OUT_DIR}/odaiba_ll2_raw_flat_ground.report.json" \
  --log "${OUT_DIR}/odaiba_ll2_raw_flat_ground.log" \
  --lanelet-side-overlap 0.3 \
  --surface-style flat
```

### Export road markings as flat surfaces

`--marking-style flat` exports `road_marking` as nearly flat surfaces instead
of raised prisms. The default `--marking-offset 0.002` lifts markings 2 mm
above the road surface to reduce z-fighting.

In this mode, `--marking-thickness` is ignored.

```bash
bash lanelet2_to_fbx/scripts/docker_export.sh \
  --input "${ODAIBA_OSM}" \
  --output "${OUT_DIR}/odaiba_ll2_raw_flat_marking.fbx" \
  --report "${OUT_DIR}/odaiba_ll2_raw_flat_marking.report.json" \
  --log "${OUT_DIR}/odaiba_ll2_raw_flat_marking.log" \
  --lanelet-side-overlap 0.3 \
  --marking-style flat
```

### Align the FBX with an existing OpenDRIVE offset

If another tool has already produced an OpenDRIVE map with a known offset, use
`--origin explicit` and pass the same values to
`--shift-x`, `--shift-y`, and `--shift-z`.

The FBX vertices are exported as:

```text
vertex = local_coordinate - shift
```

Use this rule when matching coordinate origins:

```text
OpenDRIVE offset.x/y/z = FBX --shift-x/--shift-y/--shift-z
```

For the Odaiba map, use:

```bash
bash lanelet2_to_fbx/scripts/docker_export.sh \
  --input "${ODAIBA_OSM}" \
  --output "${OUT_DIR}/odaiba_ll2_raw_aligned.fbx" \
  --report "${OUT_DIR}/odaiba_ll2_raw_aligned.report.json" \
  --log "${OUT_DIR}/odaiba_ll2_raw_aligned.log" \
  --origin explicit \
  --shift-x 92008.5 \
  --shift-y 45335.1 \
  --shift-z 0.0
```

If the OpenDRIVE offset is `0, 0, 0`, still use explicit origin mode:

```bash
bash lanelet2_to_fbx/scripts/docker_export.sh \
  --input "${INPUT_OSM}" \
  --output "${OUT_DIR}/lanelet2_map_aligned.fbx" \
  --report "${OUT_DIR}/lanelet2_map_aligned.report.json" \
  --log "${OUT_DIR}/lanelet2_map_aligned.log" \
  --origin explicit \
  --shift-x 0 \
  --shift-y 0 \
  --shift-z 0
```

Do not use `--origin center` when the FBX needs to match another local
coordinate system. `center` recenters the map around the road surface bounds,
so it will not match an external OpenDRIVE offset.

## Common CLI Options

These are the options most commonly used with `ll2tofbx export`.

| Option | Description |
| --- | --- |
| `--input` | Input Lanelet2 OSM file |
| `--output` | Output FBX path |
| `--report` | Output JSON report path |
| `--log` | Log output path. If omitted, a `.log` file is written next to the report |
| `--origin center` | Default. Automatically shifts the map so the road surface is near the origin |
| `--origin explicit` | Uses the given shift values exactly |
| `--shift-x` `--shift-y` `--shift-z` | Shift values used with `--origin explicit` |
| `--surface-style` | `solid` or `flat`. `flat` exports ground layers as double-sided surfaces |
| `--lanelet-side-overlap` | Extra road support surface width added outside lanelet boundaries, in meters |
| `--marking-style` | `prism` or `flat`. `flat` exports markings as nearly flat surfaces |
| `--keep-intermediate` | Keeps the intermediate OBJ/MTL files |

## Lanelet Side Overlap

`--lanelet-side-overlap` adds a widened support surface without changing the
original `lanelet_road` mesh.

In the output FBX:

- `lanelet_road` is generated from the original lanelet width
- `road_extention` is generated by widening both sides by the given distance

For example, `--lanelet-side-overlap 0.3` creates a `road_extention` surface
that extends 0.3 m beyond each side of the original lanelet road. This is useful
when CARLA needs a more continuous drivable surface or when small visual gaps
appear between lanelets.

Recommended starting values:

- `0.0`: no widening
- `0.2` to `0.3`: practical first values to try

Large values can create unintended overlaps with nearby geometry.

## Coordinate Shift Modes

The default `--origin center` mode automatically shifts the road surface so the
map is near the FBX origin. This is convenient when the FBX only needs to be
placed near `(0, 0, 0)`.

Use `--origin explicit` when the FBX must align with another map or simulator
coordinate system:

```bash
--origin explicit \
--shift-x <offset_x> \
--shift-y <offset_y> \
--shift-z <offset_z>
```

`--shift-x`, `--shift-y`, and `--shift-z` are only valid with
`--origin explicit`.

## Outputs

The main outputs are:

- `*.fbx`
- `*.report.json`
- `*.log`

When `--keep-intermediate` is used, the intermediate OBJ/MTL files are also
preserved.

## Troubleshooting

If `success` is `false`, inspect the JSON report first.

Useful report fields include:

- `validation.errors`
- `validation.warnings`
- `filtered_source_counts`
- `blender_command`

For Docker-based operation, the report is usually the fastest way to understand
what happened during conversion.
