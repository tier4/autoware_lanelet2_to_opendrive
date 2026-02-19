# Crosswalk Objects

This document describes the conversion of Lanelet2 crosswalk lanelets to OpenDRIVE `<objects>` elements.

## Overview

The converter transforms Lanelet2 lanelets with `subtype="crosswalk"` into OpenDRIVE `<object type="crosswalk">` elements within each road's `<objects>` section. CARLA Simulator reads these objects to include pedestrian crossings in its navigation mesh.

## OpenDRIVE Output Structure

Each crosswalk lanelet produces an `<object>` element with a closed polygon outline:

```xml
<road id="5" ...>
  ...
  <objects>
    <object type="crosswalk" id="12345" name="crosswalk_12345"
            s="42.3" t="-3.1" zOffset="0.0"
            hdg="1.5707" pitch="0.0" roll="0.0"
            orientation="none" width="4.5" length="6.0">
      <outline>
        <cornerLocal u="-3.0" v="-2.25" z="0.0"/>
        <cornerLocal u="3.0"  v="-2.25" z="0.0"/>
        <cornerLocal u="3.0"  v="2.25"  z="0.0"/>
        <cornerLocal u="-3.0" v="2.25"  z="0.0"/>
        <cornerLocal u="-3.0" v="-2.25" z="0.0"/>
      </outline>
    </object>
  </objects>
</road>
```

## Module Structure

### `opendrive/objects.py`

Core definitions:

- **`CornerLocal`**: Single vertex in the object-local coordinate system
- **`CrosswalkObject`**: OpenDRIVE crosswalk object with position, dimensions, and outline
- **`find_nearest_road()`**: Finds the nearest road for a given crosswalk lanelet
- **`_sample_road_points()`**: Samples points along the road reference line (ParamPoly3-aware)
- **`_project_point_onto_road()`**: Projects a point onto the road reference line to obtain `s`/`t`/`hdg`
- **`_compute_corner_locals()`**: Transforms world-space polygon vertices to object-local coordinates

## Conversion Algorithm

For each crosswalk lanelet, the following steps are performed:

### 1. Polygon Extraction

Four vertices are extracted from the lanelet boundaries using coordinate-offset-adjusted coordinates:

| Vertex | Source |
|--------|--------|
| `p0` | `leftBound[0]` (left-start) |
| `p1` | `leftBound[-1]` (left-end) |
| `p2` | `rightBound[-1]` (right-end) |
| `p3` | `rightBound[0]` (right-start) |

### 2. Centroid Computation

The centroid is computed as the average of the four vertices:

```
centroid = (p0 + p1 + p2 + p3) / 4
```

### 3. Nearest Road Search

The nearest road is found by sampling each road's reference line geometry:

- Each `ParamPoly3` geometry segment is sampled at `_SAMPLE_POINTS_PER_GEOMETRY` (default: 10) equally-spaced arc-length positions
- The world-space position at each sample is computed from the polynomial coefficients
- The road with the sample point closest to the crosswalk centroid is selected
- If the minimum distance exceeds the threshold (`_NEAREST_ROAD_THRESHOLD_M` = 50 m), the crosswalk is skipped with a warning

### 4. Road Reference Line Projection

The centroid is projected onto the nearest road's reference line to compute:

- **`s`**: Arc-length position along the road reference line
- **`t`**: Signed lateral offset (positive = left of reference line)
- **`road_hdg`**: Road heading at the closest sample point

### 5. Heading Computation

The crosswalk heading is computed relative to the road direction:

```
cw_angle = atan2(leftBound[-1].y - leftBound[0].y,
                 leftBound[-1].x - leftBound[0].x)
hdg = normalize_to_pi(cw_angle - road_hdg)
```

### 6. Width and Length Computation

| Attribute | Formula | Physical Meaning |
|-----------|---------|-----------------|
| `width` | `‖p3 − p0‖` | Distance between left and right bounds at entry (parallel to road) |
| `length` | `(‖p1 − p0‖ + ‖p2 − p3‖) / 2` | Average length of left and right bounds (crossing distance) |

### 7. Corner Local Coordinates

Each vertex is transformed to the object-local coordinate system:

- **Origin**: centroid
- **u-axis**: along `cw_dir` (crosswalk heading direction)
- **v-axis**: perpendicular to `cw_dir` (right-hand rule)

The polygon is closed by repeating the first vertex as the fifth point.

## Coordinate System

```
         cw_dir (u-axis)
            ↑
p1 ─────────┼───────── p2
|           │ centroid  |
|           ●           |   v-axis →
|           │           |
p0 ─────────┼───────── p3
```

Object-local coordinates:
- `u` > 0: ahead of centroid in crosswalk direction
- `v` > 0: to the right of the crosswalk direction
- `z` = 0: flush with road surface

## Elevation

The `zOffset` attribute is set to the average elevation of the crosswalk:

```
zOffset = mean([mean(leftBound[:].z), mean(rightBound[:].z)])
```

Coordinate offset (from `COORDINATE_OFFSET`) is applied before this computation.

## Lanelet2 Input Requirements

For reliable crosswalk conversion, the Lanelet2 map should:

- Tag crosswalk lanelets with `subtype="crosswalk"`
- Ensure `leftBound` and `rightBound` each have at least 2 points
- Place crosswalk lanelets within 50 m of a road reference line

!!! note "Lanelet type vs. area type"
    This converter processes `subtype="crosswalk"` **lanelets** (not Lanelet2 `area` elements).
    The mapping is from `lanelet[@subtype='crosswalk']` → `<object type="crosswalk">`.

## Integration with Conversion Pipeline

Crosswalk extraction runs as **Step 6.5** in the main conversion pipeline, after traffic signal extraction (Step 6) and before XML output (Step 7):

```
Step 1: Build regular roads
Step 2: Build junction structure
Step 3: Build road-lanelet mappings
Step 4: Set up road and lane connections
Step 5: Extract and assign signals
Step 6: Create and assign controllers
Step 6.5: Extract crosswalks and assign as road objects
Step 6.6: Extract stop lines and assign as road objects
Step 7: Write OpenDRIVE output
```

**Code location:** [`main.py` – `_extract_and_assign_crosswalks()`](https://github.com/tier4/autoware_lanelet2_to_opendrive/blob/master/src/autoware_lanelet2_to_opendrive/main.py)

## Verification

After conversion, verify the output contains crosswalk objects:

```bash
# Check for objects section
grep -A 20 '<objects>' output.xodr

# Count crosswalk objects
grep -c 'type="crosswalk"' output.xodr
```

Expected output format:

```xml
<objects>
  <object type="crosswalk" id="..." name="crosswalk_..." s="..." t="..." ...>
    <outline>
      <cornerLocal u="..." v="..." z="0.0"/>
      ...
    </outline>
  </object>
</objects>
```

## CARLA Behavior

When CARLA loads an OpenDRIVE file containing `<object type="crosswalk">` elements:

1. The `ObjectParser` reads the object type, position (`s`, `t`, `zOffset`), and outline geometry
2. The navigation mesh includes the crosswalk polygon as a pedestrian crossing area
3. NPC pedestrians route through crosswalk regions
4. NPC vehicles recognize crosswalks and may yield to pedestrians (depending on CARLA version and settings)

See [CARLA OpenDRIVE and Lanelet2 Tag Mapping](carla_opendrive_lanelet2_mapping.md#objectparser) for detailed CARLA parser behavior.

## Known Limitations

- Only the **nearest road** (within 50 m) is associated with each crosswalk; crosswalks spanning multiple roads are assigned to the single closest road
- `height` attribute is not set (defaults to 0); CARLA does not require it for crosswalk detection
- `orientation` is always `"none"`; CARLA does not use this field for crosswalks
- Only `ParamPoly3` and simple `Line` geometry are fully supported for road sampling; other geometry types fall back to straight-line approximation
