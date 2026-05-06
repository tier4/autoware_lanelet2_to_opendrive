# Lanelet2 + Autoware Profile

This document describes the slice of Lanelet2 (with Autoware extensions) that
this converter needs to understand. It is written against real AWSIM
西新宿 data (`autoware_lanelet2_to_opendrive/test/data/nishishinjuku.osm`) so
counts and attribute shapes are grounded in an actual production map.

Sources: Lanelet2 upstream documentation, Autoware `autoware_lanelet2_extension`,
`docs.pilot.auto` vector map requirements, and direct code-reading of the
converter.

## Data model primitives

Lanelet2 is an attributed graph of three geometric primitives (`Point3d`,
`LineString3d`, `Polygon3d`), a few semantic primitives (`Lanelet`, `Area`),
and first-class `RegulatoryElement`s. Every primitive carries an integer `id`
and an attribute map (`attributes`).

| Primitive | Purpose | Accessed here |
|---|---|---|
| `Point3d` | 3D coordinate `(x, y, z)` with optional attributes. | `.x`, `.y`, `.z`, `.id`, `attributes` keys `ele`, `local_x`, `local_y`, `mgrs_code`. |
| `LineString3d` | Ordered list of Point3d. | Iteration, `.attributes`, `.id`. |
| `Polygon3d` | Closed polygon of Point3d. | Not consumed today (would carry `Area` outlines). |
| `Lanelet` | Left + right bound (both LineString3d) with attributes and regulatory element list. | `.leftBound`, `.rightBound`, `.centerline`, `.attributes`, `.regulatoryElements`, `.id`. |
| `Area` | Bounded polygon plus inner rings and attributes. | Parking-lot Areas consumed since P2-1 (#448): each `Area subtype="parking_lot"` produces one synthetic road with two `PARKING`-typed lanes plus one `<object type="parkingSpace">` per child `parking_space` LineString. Other Area subtypes (traffic islands, etc.) remain skipped. |
| `RegulatoryElement` | Polymorphic rule attached to one or more lanelets. | `.attributes` (`subtype` discriminator), `.parameters` (`refers`, `ref_line`, ...), `.id`. Strongly-typed subclasses (`TrafficLight`) expose properties like `.trafficLights`, `.stopLine`. |

### Coordinate origin

Lanelet2 maps are loaded through `lanelet2.io.load(path, projector)`. The
projector converts WGS84 to a local metric frame. This converter uses
`autoware_lanelet2_extension_python.projection.MGRSProjector`, constructed from
either an MGRS grid + offset or a latitude/longitude origin (see
`projection.py`). After loading, every `Point3d` carries `x`/`y`/`z` in the
projected local metric frame; the original MGRS code is preserved on each point
as the attribute `mgrs_code`.

## Lanelet subtype vocabulary

The `subtype` attribute on a `Lanelet` discriminates between driving surface
and other semantic regions. AWSIM 新宿 uses (counts approximate):

| Subtype | Count | Meaning | Converter behavior |
|---|---|---|---|
| `road` | 884 | Generic driving lane. | Emitted as OpenDRIVE `lane type="driving"`. |
| `highway` | 0 in 新宿, common on expressway maps | Driving lane with highway semantics. | Same as `road`, with optional `roadType="motorway"`. |
| `crosswalk` | 84 | Pedestrian crosswalk polygon. | Emitted as OpenDRIVE `object type="crosswalk"` with `outline`. |
| `walkway` | 8 | Pedestrian walkway. | Emitted as OpenDRIVE `lane type="sidewalk"` since P0-1 (#427). Note: walkways are disconnected from crosswalks in 新宿, so CARLA pedestrian NavMesh still has no usable network — the remaining gap is on the Lanelet2 authoring side. |
| `road_shoulder` | 3 | Narrow lane at the edge of a road. | Currently treated as `road`; should map to `lane type="shoulder"`. |
| `bicycle_lane` | 0 in 新宿 | Dedicated bicycle lane. | Maps to `lane type="biking"` (implemented but not exercised). |
| `parking_lot` | 0 in 新宿 | Parking area (usually an `Area`). | Not emitted. |
| `play_street` | 0 | Shared-use street. | Not specified. |
| `unknown` / missing | 455 | Data cleanliness problem. | Skipped with a warning. |

The Autoware vector map requirements additionally recognize `bus_lane`,
`emergency_lane`, `pedestrian_marking`, and `stop_line` as LineString or
RegulatoryElement subtypes, but not as Lanelet subtypes.

## LineString types and subtypes

LineStrings in Lanelet2 are used for three orthogonal purposes:

1. **Boundaries of lanelets** (left/right bounds). Their `type` describes the
   physical marking.
2. **Free-standing semantic linestrings** (e.g. stop lines, traffic-sign
   refers). Their `type` is the semantic role.
3. **Children of TrafficLight regulatory elements** (bulb geometry). Their
   `type` is `traffic_light`.

### Boundary marking types (from AWSIM / Autoware vector map guidelines)

| `type` | `subtype` | OpenDRIVE roadMark equivalent |
|---|---|---|
| `line_thin` | `solid` | `solid`, `color="white"`, `weight="standard"` |
| `line_thin` | `dashed` | `broken`, `color="white"`, `weight="standard"` |
| `line_thin` | `solid_solid` | `solid solid` |
| `line_thin` | `solid_dashed` | `solid broken` |
| `line_thin` | `dashed_solid` | `broken solid` |
| `line_thin` | `dashed_dashed` | `broken broken` |
| `line_thick` | `solid` | `solid`, `weight="bold"` |
| `line_thick` | `dashed` | `broken`, `weight="bold"` |
| `road_border` | (none) | `curb` or `edge` |
| `curbstone` | (none) | `curb` |
| `virtual` | (none) | `none` |
| `guard_rail` | (none) | `edge` (no direct equivalent) |
| `fence` | (none) | (no direct equivalent; could be emitted as `object type="barrier"`) |
| `wall` | (none) | (same as fence) |
| `pedestrian_marking` | (none) | Not a lane boundary; sometimes appears on crosswalks. |

Colour is encoded on the LineString as `color` (`white`, `yellow`, ...).
Lane-change permissibility is encoded as `lane_change` (`yes`/`no` or `left`,
`right`). Both are preserved on the LineString's attribute map.

### Semantic free-standing LineStrings

| `type` | `subtype` | Purpose |
|---|---|---|
| `stop_line` | (none) | Transverse line where vehicles must stop (used by traffic lights and stop signs). |
| `traffic_sign` | `stop_sign`, `yield_sign`, `maximum_speed`, ... | Represents the sign's physical frame; the actual rule comes from its owning RegulatoryElement. |
| `traffic_light` | `red_yellow_green`, `pedestrian`, `arrow`, ... | Polyline along the traffic light housing; bulbs are stored in nested `light_bulbs` LineStrings. |
| `light_bulbs` | subtype like `red_yellow_green` | Sub-linestring grouping individual bulbs with the `color` attribute per point. |
| `pedestrian_marking` | (none) | Crosswalk stripes geometry. |
| `detection_area` | (none) | Polygon surrounding a zone for detection events. |

## Regulatory elements

Lanelet2 models traffic rules as `RegulatoryElement`s discriminated by
`attributes.subtype`. A lanelet attaches to one or more REs via
`.regulatoryElements`.

| `subtype` | Count in 新宿 | Shape | Converter usage |
|---|---|---|---|
| `traffic_light` (Autoware `AutowareTrafficLight`) | 164 | `parameters["refers"]` → traffic_light LineStrings; `parameters["ref_line"]` → stop_line LineString; `.trafficLights` property. | Emits `signal type=1000001` with `positionInertial` at bulb mean; emits `dependency` to the stop-line signal. |
| `traffic_sign` | 450 | `parameters["refers"]` → traffic_sign LineStrings; sub-`subtype` on those LineStrings (e.g. `stop_sign`). | Generates `signal type=206` or `205` depending on sub-subtype. |
| `road_marking` | 32 | `parameters["refers"]` → stop_line LineStrings. | Emits `signal type=294` stop-line marking. |
| `right_of_way` | 85 | `parameters["right_of_way"]` lanelets vs `parameters["yield"]` lanelets. | Emitted as `<junction><priority high="..." low="..."/>` since P1-1 (#457). CARLA TrafficManager ignores the value. |
| `speed_limit` | 0 in 新宿 (lanelet-level attribute preferred) | Scalar value with unit. | Converter reads the lanelet attribute `speed_limit` instead. |
| `detection_area` | 1 | Polygon plus `ref_line`. | Not consumed. |
| `all_way_stop` | 0 in 新宿 | Group of stop lines. | Not consumed. |
| `no_stopping_area`, `no_parking_area`, `bus_stop`, `pedestrian_traffic_light` | 0 | Various. | Not consumed. |

### Autoware extensions

The fork `autoware_lanelet2_extension` registers extra classes with Lanelet2's
C++ factory. The most relevant are:

- **`AutowareTrafficLight`** (`subtype=traffic_light`). Adds the `.trafficLights`
  list (one LineString per physical traffic-light housing; children
  `light_bulbs` describe individual coloured bulbs) and the `.stopLine`
  reference.
- **`VirtualTrafficLight`** (`subtype=virtual_traffic_light`). Used for
  intersections without physical signals, e.g. intelligent intersection support.
  Not consumed here.
- **`DetectionArea`** (`subtype=detection_area`). Attaches a polygon to a
  `ref_line`.
- **`NoStoppingArea`**, **`BusStopArea`**, etc.

The converter triggers Autoware class registration simply by importing
`autoware_lanelet2_extension_python.regulatory_elements` — after that, any
`lanelet2.io.load()` call produces the strongly-typed subclasses.

## Lanelet attribute vocabulary

Attributes accessed on `Lanelet` objects (counts are observed occurrences in
新宿):

| Attribute | Example values | Purpose | Count |
|---|---|---|---|
| `subtype` | `road`, `crosswalk`, `walkway`, `road_shoulder`. | Discriminator for lanelet role. | 3,900 |
| `turn_direction` | `straight`, `left`, `right`. | Presence indicates the lanelet is inside a junction. | 387 |
| `one_way` | `yes`, `no`. | Directionality. | 979 |
| `speed_limit` | numeric string. | km/h for the lanelet. | 979 |
| `location` | `urban`, `highway`, `rural`, `private`. | Maps to OpenDRIVE `road/type`. | 979 |
| `participant:vehicle`, `participant:pedestrian`, `participant:bicycle` | `yes`. | Who may use this lanelet. Emitted as `<lane><access rule="allow|deny" restriction="..."/>` since #477. | 602 / 92 / — |
| `lane_change` | `yes`, `no`, `left`, `right`. | Lane-change permissibility. | 251 |
| `width`, `height` | numeric. | Explicit width/height overrides. | 1,350 / 602 |
| `turn_signal_distance` | numeric. | Autoware planner hint. | 15 |

Absence of `turn_direction` implies the lanelet is on a straight road segment
(not inside a junction). Junction detection in this converter relies on the
presence of `turn_direction` rather than `right_of_way` REs.

## Point attribute vocabulary

All points in an AWSIM map carry:

- `ele` — elevation (float, absolute metres). This is authoritative; the
  Point's `.z` is already assigned from it after projection.
- `local_x`, `local_y` — cached projected coordinates.
- `mgrs_code` — identifier for the MGRS tile the point belongs to.

Inside `light_bulbs` LineStrings, each bulb point also carries `color`
(`red`, `yellow`, `green`, or arrow-shape strings) and sometimes `arrow`
(`left`, `right`, `straight`). The arrow direction is now encoded as a
bitmask in the emitted `signal@subtype` so it is preserved end-to-end
(#474/#475).

## Information the converter can and cannot infer

The following facts are **available** from a well-formed AWSIM Lanelet2 map:

- Per-lanelet bounds, center line, turn direction, one-way flag, speed limit,
  participants, location class.
- Per-lanelet regulatory elements including co-phased traffic lights (because
  each TrafficLight RE references a specific `stop_line` LineString).
- Per-point 3D coordinates, allowing z and slope calculation.
- Per-LineString type/subtype/colour for road markings.

The following facts are **not directly encoded** in AWSIM maps and must be
inferred or supplied externally:

- **Cross-slope / superelevation** — only inferrable from asymmetric z between
  left and right bounds.
- **Pedestrian sidewalks** (`walkway`) are sparse and not connected into a
  sidewalk network. Generating CARLA-compatible pedestrian NavMesh will
  require extending the map, not just the converter.
- **Junction priority** — emitted from `right_of_way` REs (P1-1 / #457).
  CARLA TrafficManager ignores the value, so it is informational only on
  CARLA today.
- **Lane change rules between adjacent lanelets of the same road** — encoded
  on boundary LineStrings as the `lane_change` attribute; emitted as
  `roadMark@laneChange` since P0-3 (#426).
- **Signal controllers / phase groups** — Lanelet2 models one traffic light
  per RE; grouping into OpenDRIVE `<controller>` entities is a converter-side
  decision based on shared stop lines.

## Known data-quality issues observed in 新宿

- **455 lanelets with `subtype` missing or `unknown`**. These currently slip
  through subtype filters; they should be either skipped or flagged by
  preprocessing.
- **Trapezoidal lanelets at IDs 146, 113, etc.** — shapes with large left/right
  asymmetry whose auto-computed centerline is unreliable. Since #461 the
  converter no longer raises `AsymmetryLaneletException` on these; the
  width spline absorbs the asymmetry and the lanelet stays in the output.
  `preprocess_lanelet` is still useful for shape clean-up but is no longer
  load-bearing for keeping a road from being silently dropped.
- **Lanelet IDs 301033, 554** — possible authoring mistakes (disconnected from
  predecessors, or left-turn attribute on a non-junction segment). These are
  patched via the preprocessing pipeline.

## References

- Lanelet2 core documentation — `https://github.com/fzi-forschungszentrum-informatik/Lanelet2`
- Autoware vector map requirements — `https://docs.pilot.auto/reference-design/common/map-requirements/vector-map-requirements/category_lane`
- `autoware_lanelet2_extension` — `https://github.com/autowarefoundation/autoware_tools/tree/main/map/autoware_lanelet2_extension`
- `autoware_lanelet2_extension_python` (fork used here) — pyproject dependency
  `lanelet2-python-api-for-autoware` pinned to
  `7745347645fd04aa8e4230cfa8793e253aad6753`.
- Confluence — 追加仕様が必要と推察されるLanelet (page id `4659740708`).
