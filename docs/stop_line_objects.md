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
            orientation="none" width="3.5" length="0.0"/>
  </objects>
</road>
```

### Attribute Mapping

| OpenDRIVE Attribute | Source | Description |
|---------------------|--------|-------------|
| `type` | Fixed value | Always `"stopLine"` |
| `id` | Linestring ID | The Lanelet2 linestring ID |
| `name` | Derived | `"stop_line_<id>"` |
| `s` | Projection | Arc-length position along road reference line |
| `t` | Projection | Lateral offset from road reference line |
| `zOffset` | Elevation | Absolute stop line elevation minus road elevation at `s` |
| `hdg` | Computed | Stop line heading relative to road direction (radians) |
| `pitch` | Fixed value | Always `0.0` |
| `roll` | Fixed value | Always `0.0` |
| `orientation` | Fixed value | Always `"none"` |
| `width` | Geometry | Distance from first to last point of the linestring |
| `length` | Fixed value | Always `0.0` (zero thickness) |

## Module Structure

### `opendrive/objects.py`

Key definitions added for stop line support:

- **`StopLineObject`**: OpenDRIVE stop line object dataclass with position, dimensions, and `to_xml()` serialization
- **`StopLineObject.construct_from_linestring()`**: Static factory method that builds a `StopLineObject` from a Lanelet2 linestring and its nearest road
- **`find_nearest_road_for_linestring()`**: Finds the nearest road for a given linestring centroid (shared distance-search logic)

The following helpers defined for crosswalk support are also reused:

- **`_sample_road_points()`**: Samples points along the road reference line (ParamPoly3-aware)
- **`_project_point_onto_road()`**: Projects a point onto the road reference line to obtain `s`/`t`/`hdg`

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

### 7. Width Computation

The stop line width (transversal span) is the Euclidean distance between the first and last 2D points:

```
width = ‖pts_2d[-1] − pts_2d[0]‖
```

`length` is always set to `0.0` because a stop line has no meaningful thickness.

## Coordinate System

```
         road direction (s-axis →)
              ↑ road_hdg
  pts[0] ─────────────────── pts[-1]
              ↕ width
         ● centroid
```

Object-local coordinates follow the standard OpenDRIVE convention:
- `s`/`t` are road-curve-relative coordinates
- `hdg` is the angle between the stop line direction and the road direction

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

- Only the **nearest road** (within 50 m) is associated with each stop line; stop lines near road boundaries may be assigned to the closest road rather than the most semantically correct one
- `length` is always `0.0`; the stop line is modeled as a zero-thickness line across the road
- `orientation` is always `"none"`
- Only `ParamPoly3` and simple `Line` geometry are fully supported for road sampling; other geometry types fall back to straight-line approximation
- CARLA's current OpenDRIVE object parser does not specifically handle `type="stopLine"` objects; see [Stop Line Position Discrepancies](limitations/stop-line-position.md) for details

## See Also

- [Conversion Process](conversion-process.md) – Full pipeline overview including Stage 8.6
- [Crosswalk Objects](crosswalk_objects.md) – Similar conversion for crosswalk lanelets
- [Stop Line Position Discrepancies](limitations/stop-line-position.md) – Known CARLA limitation regarding stop line position accuracy
