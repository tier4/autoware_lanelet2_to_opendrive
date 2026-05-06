# Stop Line Objects

This document describes the conversion of Lanelet2 stop line linestrings to OpenDRIVE `<objects>` elements.

## Overview

The converter transforms Lanelet2 linestrings with `type="stop_line"` into OpenDRIVE `<object type="stopLine">` elements within each road's `<objects>` section. This allows downstream tools and simulators that support the OpenDRIVE object model to recognize and use stop line positions.

## OpenDRIVE Output Structure

Each stop line linestring produces an `<object>` element:

```xml
<road id="5" ...>
  ...
  <objects>
    <object type="stopLine" id="3002478" name="stop_line_3002478"
            s="15.4" t="1.8" zOffset="0.0"
            hdg="1.5707" pitch="0.0" roll="0.0"
            orientation="none" width="0.1" length="3.5"/>
  </objects>
</road>
```

When `target=carla` (or `stopline.carla_stop_line=true`) is selected, the
same linestring is emitted in CARLA's `Stencil_STOP` form instead, and
CARLA actually consumes it:

```xml
<object type="-1" name="Stencil_STOP" id="3002478" s="15.4" t="1.8"
        zOffset="0.0" hdg="1.5707" pitch="0.0" roll="0.0"
        orientation="-" width="2.0" length="3.5"/>
```

### Attribute Mapping

| OpenDRIVE Attribute | Source | Description |
|---------------------|--------|-------------|
| `type` | Fixed | Standard form: `"stopLine"`. CARLA form: `"-1"` (with `name="Stencil_STOP"`). |
| `id` | Linestring ID | The Lanelet2 linestring ID. |
| `name` | Derived | Standard form: `"stop_line_<id>"`. CARLA form: `"Stencil_STOP"`. |
| `s` | Projection | Arc-length position along road reference line. |
| `t` | Projection | Lateral offset from road reference line. |
| `zOffset` | Elevation | Standard form: stop-line absolute elevation minus road elevation at `s`. CARLA form: fixed at `0.0`. |
| `hdg` | Computed | Stop-line heading relative to road direction (radians). |
| `pitch` | Fixed | `0.0`. |
| `roll` | Fixed | `0.0`. |
| `orientation` | Fixed | Standard form: `"none"`. CARLA form: `"-"`. |
| `width` | Config | Painted thickness in the **v-direction** (along the road). Default `0.1` m for the standard form (`stopline.width`); CARLA target overrides this to `2.0` m to match `Stencil_STOP` decals. |
| `length` | Geometry | Span in the **u-direction** (across the road) — the Euclidean distance between the linestring's first and last 2D points. |

## Module Structure

### `opendrive/objects.py`

- **`StopLineObject`** — dataclass for the `<object>` (or
  `Stencil_STOP`) element, with `to_xml()` handling the dual-form
  serialization
- **`StopLineObject.construct_from_linestring()`** — static factory that
  builds a `StopLineObject` from a Lanelet2 linestring and its nearest
  road; takes `width` (painted thickness) and `carla_format` flags
- **`find_nearest_road_for_linestring()`** — shared nearest-road search
- **`_sample_road_points()`** / **`_project_point_onto_road()`** —
  ParamPoly3-aware reference-line sampling helpers reused from the
  crosswalk path

### `main.py` — `_extract_and_assign_stop_lines`

In addition to attaching the `StopLineObject` to its road, this method
emits up to three companion `<signal>` elements per stop line, depending
on what the Lanelet2 regulatory elements say:

- `Signal(@type=294, name="StopLine_<id>")` with one
  `<dependency type="trafficLight">` per associated traffic-light signal
  — emitted when the linestring is referenced by a `traffic_light`
  regulatory element's `stopLine`
- `Signal(@type=206, name="StopSign_<id>")` — emitted when the
  linestring is referenced via a `traffic_sign` regulatory element
  whose `refers` member has `subtype="stop_sign"`
- `Signal(@type=205, name="YieldSign_<id>")` plus
  `Signal(@type=294, name="StopLine_<id>")` with a
  `<dependency type="yieldSign">` — emitted for stop lines that come
  from a `road_marking` regulatory element (skipped when the same
  linestring is also handled by the traffic-light branch above)

After all stop-line signals are emitted, the converter adds back-pointer
`<reference id="<StopLine signal id>" elementType="signal" type="stopLine"/>`
elements to each linked traffic-light signal so the relationship is
expressed in both directions. In CARLA mode
(`stopline.carla_stop_line=true`) the traffic-light → stop-line
dependency emission is suppressed (`Stencil_STOP` already covers what
CARLA needs); the StopSign / YieldSign / stand-alone StopLine signals
are still emitted.

## Conversion Algorithm

For each linestring with `type="stop_line"`, the following steps are performed:

### 1. Point Extraction

All 2D and 3D points are extracted from the linestring:

- 2D points are used for centroid computation, heading, and width calculations
- 3D points are used for elevation (`zOffset`) computation
- Linestrings with fewer than 2 points are skipped with a warning

### 2. Centroid Computation

The centroid is computed as the mean of all 2D points:

```
centroid = mean(pts_2d, axis=0)
```

### 3. Nearest Road Search

The nearest road is found by sampling each road's reference line geometry:

- Each `ParamPoly3` geometry segment is sampled at `_SAMPLE_POINTS_PER_GEOMETRY` (default: 10) equally-spaced arc-length positions
- The road with the sample point closest to the stop line centroid is selected
- If the minimum distance exceeds the threshold (`_NEAREST_ROAD_THRESHOLD_M` = 50 m), the stop line is skipped with a warning

### 4. Road Reference Line Projection

The centroid is projected onto the nearest road's reference line to compute:

- **`s`**: Arc-length position along the road reference line
- **`t`**: Signed lateral offset (positive = left of reference line)
- **`road_hdg`**: Road heading at the closest sample point

### 5. Elevation Computation

The `zOffset` is computed from 3D geometry:

```
stop_line_z   = mean of all 3D point z-coordinates
road_elev_z   = road.get_elevation_at_s(s)
zOffset       = stop_line_z - road_elev_z
```

### 6. Heading Computation

The stop line heading is the direction from the first point to the last point, expressed relative to the road direction:

```
stop_line_angle = atan2(pts[-1].y - pts[0].y,
                        pts[-1].x - pts[0].x)
hdg = normalize_to_pi(stop_line_angle - road_hdg)
```

### 7. Width and Length Computation

OpenDRIVE's object model places `length` along the local `u`-axis
(aligned with the object's heading) and `width` along the local
`v`-axis. For a stop line, the heading runs across the road, so:

```
length = ‖pts_2d[-1] − pts_2d[0]‖     # span across the road (u-axis)
width  = stopline.width                # painted thickness along the road (v-axis)
```

`width` is therefore a configurable painted-thickness value, not a
geometric measurement: `0.1` m by default, and `2.0` m when the
`target=carla` overlay swaps to `Stencil_STOP` so the stencil decal has
the right footprint.

## Coordinate System

```
                 road direction (s-axis →)
                            ↑ road_hdg

  pts[0] ─────●───────────── pts[-1]      ← stop-line span = `length` (u-axis)
            centroid
                            ↕ painted thickness = `width` (v-axis, along road)
```

Object-local conventions:

- `s` / `t` are road-curve-relative coordinates of the stop-line
  centroid;
- `hdg` is the angle between the stop-line direction (`pts[0] →
  pts[-1]`) and the road direction at `s`;
- in object-local frame, `u` runs along `hdg` (across the road) and `v`
  runs perpendicular to `hdg` (along the road).

## Lanelet2 Input Requirements

For reliable stop line conversion, the Lanelet2 map should:

- Tag stop line linestrings with `type="stop_line"` in the `lineStringLayer`
- Ensure each linestring has at least 2 points
- Place stop lines within 50 m of a road reference line

## Integration with Conversion Pipeline

Stop line extraction runs as **Step 6.6** in the main conversion pipeline, after crosswalk extraction (Step 6.5) and before XML output (Step 7):

```
Step 1: Build regular roads
Step 2: Build junction structure
Step 3: Build road-lanelet mappings
Step 4: Set up road and lane connections
Step 5: Extract and assign signals
Step 6: Create and assign controllers
Step 6.5: Extract crosswalks and assign as road objects
Step 6.6: Extract stop lines and assign as road objects  ← This step
Step 7: Write OpenDRIVE output
```

**Code location:** [`main.py` – `_extract_and_assign_stop_lines()`](https://github.com/tier4/autoware_lanelet2_to_opendrive/blob/master/src/autoware_lanelet2_to_opendrive/main.py)

## Verification

After conversion, verify the output contains stop line objects:

```bash
# Check for stopLine objects
grep 'type="stopLine"' output.xodr

# Count stop line objects
grep -c 'type="stopLine"' output.xodr
```

Expected output format:

```xml
<objects>
  <object type="stopLine" id="..." name="stop_line_..." s="..." t="..."
          zOffset="..." hdg="..." pitch="0.0" roll="0.0"
          orientation="none" width="..." length="0.0"/>
</objects>
```

## Known Limitations

- Only the **nearest road** (within 50 m) is associated with each stop
  line; stop lines near road boundaries may be assigned to the closest
  road rather than the most semantically correct one.
- `width` is a painted-thickness constant (`stopline.width`) rather than
  a geometric measurement; the actual painted-stop-line thickness in the
  source map is not preserved.
- `orientation` is always `"none"` for the standard form (`"-"` for the
  CARLA `Stencil_STOP` form).
- Only `ParamPoly3` and simple `Line` geometry are fully supported for
  road sampling; other geometry types fall back to straight-line
  approximation.
- CARLA's stock OpenDRIVE object parser does **not** consume
  `<object type="stopLine">`; use `target=carla` (or
  `stopline.carla_stop_line=true`) to emit the `Stencil_STOP` form CARLA
  actually reads. See
  [Stop Line Position Discrepancies](limitations/stop-line-position.md)
  for the architectural background.

## See Also

- [Conversion Process](conversion-process.md) – Full pipeline overview including Stage 8.6
- [Crosswalk Objects](crosswalk_objects.md) – Similar conversion for crosswalk lanelets
- [Stop Line Position Discrepancies](limitations/stop-line-position.md) – Known CARLA limitation regarding stop line position accuracy
