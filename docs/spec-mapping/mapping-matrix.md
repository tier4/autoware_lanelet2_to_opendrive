# Lanelet2 → OpenDRIVE Mapping Matrix

Element-by-element conversion matrix for the Lanelet2 → OpenDRIVE (CARLA
ue5-dev profile) converter. Every row carries:

- the Lanelet2 side (primitive, attribute or regulatory element),
- the OpenDRIVE side that should emit from it,
- a **status label** describing the converter's current state,
- the existing source module when something is implemented,
- a note that explains information loss, heuristics, or open questions.

## Status labels

| Label | Meaning |
|---|---|
| **FULL** | 1:1 mapping; the converter emits all relevant OpenDRIVE fields from the available Lanelet2 data without loss. |
| **LOSSY** | Mapping exists but drops or approximates information (shape smoothing, attribute collapse, etc.). |
| **MANUAL** | Requires human input or external configuration (YAML, preprocessing rules). The automatic converter alone is not sufficient. |
| **UNSUPPORTED_BY_CARLA** | Fields are defined by the OpenDRIVE spec and can be emitted, but CARLA ue5-dev ignores them. Emit for spec conformance only. |
| **MISSING_IN_LANELET2** | CARLA needs data that Lanelet2 does not encode. Requires a map extension, not just converter work. |
| **NOT_IMPLEMENTED** | Mapping is possible in principle but not yet implemented. |
| **OUT_OF_SCOPE** | Intentionally excluded (e.g. rail, CRG). |

## 1. Header and coordinate system

| Lanelet2 side | OpenDRIVE side | Status | Module | Notes |
|---|---|---|---|---|
| `lanelet2.io.Origin(lat, lon)` built from MGRS grid or `lat_lon` config. | `header/geoReference` with `+proj=tmerc +lat_0 +lon_0 +k=1 +x_0=0 +y_0=0 +ellps=WGS84`. | FULL | `opendrive/header.py`, `projection.py` | CARLA only reads `+lat_0` / `+lon_0`; omitting them teleports the map to Barcelona. |
| Projected map extent (bounding box of all Point3d). | `header@north/@south/@east/@west`. | FULL | `opendrive/header.py`, `main.py`. | #473 (B-1, #465). Computed from the `(min_x, max_x, min_y, max_y)` of every projected `Point3d`. |
| Map date, version, vendor. | `header@date`, `@version`, `@vendor`. | LOSSY | `opendrive/header.py` | Fixed strings. CARLA ignores. |

## 2. Road geometry

| Lanelet2 side | OpenDRIVE side | Status | Module | Notes |
|---|---|---|---|---|
| Adjacency graph of `Lanelet` primitives, grouped by shared boundaries and direction. | `road/@id`, `@length`, `@junction`. | FULL | `opendrive/road.py::Road.construct_from_lanelet_map`. | Groups use `subtype != crosswalk/walkway` and `turn_direction` absence. |
| Centerline of the reference lanelet in the group (Frenet-lifted from left/right bounds). | `road/planView/geometry` as a series of `paramPoly3` with `pRange="arcLength"`. | LOSSY | `spline.py::Splines`, `opendrive/geometry.py::ParamPoly3.from_spline`. | B-spline fit with hard-constraint boundaries. Typical fit error < 2 m avg, < 8 m peak; see `SplineConstants`. `min_segment_length` ≥ 0.5 m to avoid CARLA crashes. |
| Straight segments with zero curvature. | `geometry/line`. | FULL | `opendrive/geometry.py::Line`. | Emitted only when centerline is strictly linear within tolerance. |
| Constant-curvature arcs. | `geometry/arc@curvature`. | FULL | `opendrive/geometry.py::Arc`. | #476 (B-2, #466). Detection enabled; long-radius constant-curvature segments emit as `<arc>` instead of paramPoly3 chains. |
| Curvature transitions. | `geometry/spiral@curvStart, @curvEnd`. | NOT_IMPLEMENTED | `opendrive/geometry.py::Spiral` stub. | Detection not enabled — the shared evaluation kernel needs to be extended before the stub becomes usable. Tracked alongside #466 follow-ups. |
| Z values of centerline points. | `road/elevationProfile/elevation@s, a, b, c, d`. | LOSSY | `opendrive/elevation.py`. | Cubic polynomial per geometry segment; stepped at boundaries. |
| Asymmetric left/right z (banked curves). | `road/lateralProfile/superelevation`. | NOT_IMPLEMENTED | — | Not computed; roads are always emitted with zero bank. |
| Cross-section deformation. | `road/lateralProfile/shape`. | OUT_OF_SCOPE | — | Rarely needed; skip unless a specific consumer requires it. |

## 3. Road connectivity and links

| Lanelet2 side | OpenDRIVE side | Status | Module | Notes |
|---|---|---|---|---|
| Previous lanelet of the reference lanelet in a road group. | `road/link/predecessor@elementId, @elementType, @contactPoint`. | FULL | `opendrive/road_links.py`. | `elementType` is `road` or `junction` depending on whether the predecessor belongs to a junction group. |
| Next lanelet. | `road/link/successor@*`. | FULL | `opendrive/road_links.py`. | Same as above. |
| Lanelet2 `location` attribute on the reference lanelet. | `road/type@s, @type`. | FULL | `opendrive/road.py`. | Mapping `urban`→`town`, `highway`→`motorway`, `private`→`town`, default `town`. |
| Lanelet2 `speed_limit` attribute. | `road/type/speed@max, @unit`. | FULL | `main.py` (RoadTypeSpeed). | CARLA TrafficManager currently ignores; PR #9566 will enforce. |
| Lanelet2 `speed_limit` per-lane. | `lane/speed@sOffset, @max, @unit`. | FULL | `opendrive/lane.py:252-259`, `opendrive/lane_elements.py::LaneSpeed`. | P0-4 (#428). One `<speed>` per lane at `sOffset=0` with `unit="km/h"`. CARLA TrafficManager still ignores the value; PR `carla-simulator/carla#9566` will enforce. |

## 4. Lane structure

| Lanelet2 side | OpenDRIVE side | Status | Module | Notes |
|---|---|---|---|---|
| Lanelet group → lane arrangement. | `road/lanes/laneSection` (one section per lane-count change). | LOSSY | `opendrive/lane_sections.py`, `opendrive/lane_section.py`. | Complex merges/splits may fall back to a new road instead of a new section. |
| Centerline of the reference lanelet vs. planView reference line. | `road/lanes/laneOffset@s, a, b, c, d`. | NOT_IMPLEMENTED | `opendrive/lane_section.py:298-303` (XML emission only). | Infrastructure exists but no caller assigns `LaneSection.lane_offset`. Today the reference line *is* the driving centerline so `<laneOffset>` is unnecessary; the field becomes load-bearing only if planView extraction shifts to a different curve (e.g. for multi-section roads with offset center lanes). |
| Adjacent lanelet chain within a road group. | `lane@id` (negative right, positive left). | FULL | `opendrive/lane.py:87-111, 393-396`. | #481 (B-8, #471). `Lane.lane_id` is `Optional[int]` with `None` as the sentinel; `to_xml()` raises if it is unresolved at serialisation time. RHT/LHT directionality continues to be applied at the section level. |
| Lanelet `subtype`. | `lane@type`. | FULL | `opendrive/lane.py:178`. | Mapping: `road`→`driving`, `crosswalk`→`none` (emitted as object instead), `walkway`→`sidewalk` (P0-1, #427), `bicycle_lane`→`biking`, `road_shoulder`→`shoulder`. |
| Per-point distance between left and right bounds. | `lane/width@sOffset, a, b, c, d`. | LOSSY | `centerline.py`, `opendrive/lane_elements.py::LaneWidth`. | Polynomial fit. As of #461 (commit `8b2f7f7`, follow-up to `88c59ab`) the converter no longer raises `AsymmetryLaneletException`; asymmetric / non-monotonic lanelets succeed via the width spline alone. |
| Explicit `width` / `border` curves for asymmetric lanes. | `lane/border`. | NOT_IMPLEMENTED | — | Spec requires either `width` or `border`, not both. P1-5 (#440) was closed without implementing this — the underlying user-visible defect (silent skips) was already fixed by the AsymmetryLaneletException removal. |
| Boundary LineString `type`/`subtype`/`color`/`lane_change`. | `lane/roadMark@type, @weight, @color, @laneChange, @material`. | FULL | `opendrive/lane_elements.py:118-175` (`road_mark_from_linestring_attrs`), `opendrive/lane_section.py:205-227`. | P0-3 (#426). Lookup tables at `opendrive/lane_elements.py:86-115` cover `line_thin`/`line_thick` × `solid`/`dashed`/etc. and `road_border`/`curbstone`/`virtual`. `material` not emitted (CARLA ignores). |
| Predecessor / successor lanelets at the lane level. | `lane/link/predecessor@id`, `lane/link/successor@id`. | FULL | `opendrive/road_links.py`. | Uses the road-level predecessor/successor mapping plus lane-id alignment. |
| Lanelet `lane_change` attribute on boundaries. | `roadMark@laneChange`. | FULL | `opendrive/lane_elements.py`, P0-3 (#426). | Same lookup as the row above. |
| Lanelet `level` (flat override). | `lane@level`. | LOSSY | `opendrive/lane.py`. | Always emitted as false. |
| Curb vs road-border LineStrings. | `lane/height@sOffset, @inner, @outer`. | FULL | `opendrive/lane.py:42-74` (`_compute_sidewalk_height`), `config.py::GeometryConstants.sidewalk_height`. | #480 (B-6, #469). Sidewalk and shoulder lanes raised by `DEFAULT_CONFIG.geometry.sidewalk_height` whenever the inner/outer boundary is a `curbstone` / `road_border` LineString. |
| Participant filters (`participant:vehicle`, etc.). | `lane/access@sOffset, @rule, @restriction`. | FULL | `opendrive/lane.py:147-149` (`_add_access`), `opendrive/lane_elements.py:249-264` (`LaneAccess`). | #477 (B-5, #468). `participant:X=yes` → `<access rule="allow" restriction="X"/>`; `participant:X=no` → deny. Restriction values mapped to OpenDRIVE 1.4 `e_accessRestrictionType`. |

## 5. Junctions

| Lanelet2 side | OpenDRIVE side | Status | Module | Notes |
|---|---|---|---|---|
| Lanelets with `turn_direction` attribute, grouped by shared incoming/outgoing roads. | `junction@id, @name, @type`. | FULL | `junction.py`, `opendrive/junction.py`. | `@type="default"` always. Direct/virtual types not emitted. |
| Incoming road → connecting road linking. | `junction/connection@id, @incomingRoad, @connectingRoad, @contactPoint`. | FULL | `opendrive/junction.py`. | One connection per (incomingRoad, connectingRoad) pair. |
| Connecting-road reference-line endpoints aligned with linked regular roads. | `road/planView/geometry` start/end at the rendered linked-road endpoint, no `<laneOffset>` inflation. | FULL | `opendrive/road.py:81-228` (`_evaluate_planview_endpoint_with_heading`, `_lane_aware_endpoint`), `opendrive/road.py:1197-1363`, `opendrive/reference_line.py:206-271`. | P0-2 root cause (#437/#453, default-on with lateral gate; lane-aware endpoint pinning in #464). Hard constraint weight 1e4 keeps endpoint error sub-millimetre. |
| Lane-level mapping inside the junction. | `junction/connection/laneLink@from, @to`. | FULL | `opendrive/junction.py` (`_build_lane_links`). | P1-2 (#460). Multi-lane merges and splits emit N:M `<laneLink>` per direct predecessor/successor of the connecting road. |
| `right_of_way` RegulatoryElement. | `junction/priority@high, @low`. | FULL | `opendrive/junction.py:39-54` (`Priority`), `:70-144` (`_build_priorities_from_records`), `:202-236` (`extract_right_of_way_records`). | P1-1 (#457). 新宿's 85 REs expand to per-junction `<priority>` entries; conflicts are warned. CARLA TrafficManager still ignores the value (UNSUPPORTED_BY_CARLA at runtime). |
| Traffic-light controllers associated with the junction. | `junction/controller@id, @type, @sequence`. | FULL | `opendrive/signals_and_controllers.py`. | Controllers are created from co-phased traffic lights; junction references them. |
| Overhead crossings (3D intersections without a traffic junction). | `junction@type="crossing"`. | MANUAL | — | Per Confluence, must be specified by hand (no automatic Lanelet2 signal distinguishes overhead crossings from at-grade intersections). |

## 6. Traffic signals (dynamic)

| Lanelet2 side | OpenDRIVE side | Status | Module | Notes |
|---|---|---|---|---|
| `TrafficLight` RE with a traffic_light LineString and bulbs. | `signals/signal@type="1000001"`, `@dynamic="yes"`, `positionInertial@x,@y,@z,@hdg`. | FULL | `opendrive/signals_and_controllers.py`, `opendrive/signal.py`. | `@country="OpenDRIVE"`. Position taken from bulb centroid. `zOffset` computed from elevation profile. |
| Traffic light `stopLine` reference. | `signal/dependency@id, @type`. | FULL | `opendrive/signals_and_controllers.py`. | Links traffic-light signal to its stop-line signal. |
| Back-reference from stop-line signal to traffic light. | `signal/reference` (1.5+) | LOSSY | `opendrive/signals_and_controllers.py`. | Issue #135 emits this; some 1.4-strict parsers may reject it, but CARLA accepts. |
| Traffic-light aspect colours (red/yellow/green) and shapes (arrow). | Signal `@subtype` / `@value`. | FULL | `opendrive/signal.py`, `opendrive/signals_and_controllers.py`. | #474 / #475 (B-3, #467). `@subtype` carries a bitmask derived from `lightBulbs()` so arrow direction (left / right / straight) is preserved per signal. CARLA TrafficManager still does not consume `@subtype`; benefit is for SUMO / RoadRunner / scenario tooling. |
| Co-phased traffic lights (same junction, same phase). | Top-level `<controller>` with multiple `<control@signalId>` children; junction `<controller>` reference. | FULL | `opendrive/signals_and_controllers.py`. | Grouping heuristic: same intersection, same stop line, same orientation. |
| Validity on specific lanes. | `signal/validity@fromLane, @toLane`. | FULL | `opendrive/signal.py`. | Defaults to all driving lanes of the road. |
| Pedestrian traffic lights. | Separate signal type. | NOT_IMPLEMENTED | — | Lanelet2 encodes these as a `pedestrian_traffic_light` RE (absent in 新宿); no converter path today. |

## 7. Static traffic signs

| Lanelet2 side | OpenDRIVE side | Status | Module | Notes |
|---|---|---|---|---|
| `traffic_sign` RE with refers LineString `subtype="stop_sign"`. | `signal@type="206"` (German StVO stop). | FULL | `opendrive/signals_and_controllers.py`. | `@dynamic="no"`. |
| `traffic_sign` RE with refers LineString `subtype="yield_sign"`. | `signal@type="205"`. | FULL | same. | CARLA reduces behaviour to "slow to 10 km/h" regardless of priority. |
| `traffic_sign` RE with other refers subtypes (e.g. `maximum_speed`, `no_entry`). | Various type codes. | NOT_IMPLEMENTED | — | Not required for current scenarios. |
| Country-specific numbering. | `signal@country, @type`. | LOSSY | — | Always emitted as German StVO; CARLA ignores `country` anyway. |
| Sign physical position from LineString points. | `signal@s, @t, @zOffset, @hOffset, @pitch, @roll` and/or `positionInertial`. | FULL | `opendrive/signal.py::signal_position_from_*`. | Preferred path uses `positionInertial` for precision. |

## 8. Road markings and stop lines

| Lanelet2 side | OpenDRIVE side | Status | Module | Notes |
|---|---|---|---|---|
| Free-standing `stop_line` LineString referenced by a TrafficLight RE. | `signal@type="294"` with dependency to the traffic light. | FULL | `opendrive/signals_and_controllers.py`. | UNSUPPORTED_BY_CARLA for stop behaviour — TrafficManager does not stop here. |
| Free-standing `stop_line` LineString referenced by a `road_marking` RE. | `signal@type="205"` + `signal@type="294"`. | FULL | `opendrive/signals_and_controllers.py`. | Used for stop markings without a physical sign. |
| Stop line geometry. | `road/objects/object type="stopLine"` with `length, width, height`. | LOSSY | `opendrive/objects.py::StopLineObject`. | Not a standard OpenDRIVE object type; CARLA ignores for behaviour but it renders the box. Optional via `stopline.carla_stop_line` config. |
| Lane boundary `type="line_thin" subtype="solid"` etc. | `lane/roadMark@type, @color, @weight, @laneChange`. | FULL | `opendrive/lane_elements.py:118-175`, `opendrive/lane_section.py:205-227`. | P0-3 (#426). See section 4 for the full attribute coverage. |
| Arrow markings (left/right/through). | `roadMark/type/line@*` with named patterns. | NOT_IMPLEMENTED | — | Not part of current scope. |

## 9. Objects (static infrastructure)

| Lanelet2 side | OpenDRIVE side | Status | Module | Notes |
|---|---|---|---|---|
| Lanelet `subtype="crosswalk"`. | `road/objects/object type="crosswalk"` with `outline/cornerLocal`. | FULL | `opendrive/objects.py::CrosswalkObject`. | Outline derived from left+right bounds. |
| Lanelet `subtype="walkway"` or adjacent sidewalk polygon. | `lane type="sidewalk"` within the road, plus optional `object type="pedestrianCrossing"`. | LOSSY (converter side) / MISSING_IN_LANELET2 (topology side) | `opendrive/lane.py:178`. | P0-1 (#427). The converter now emits `lane[type=sidewalk]` for walkway lanelets, but AWSIM 新宿 only has 8 walkways and they are disconnected from the 84 crosswalks; CARLA pedestrian NavMesh therefore still has no usable network. The remaining gap is on the Lanelet2 authoring side. |
| `Area subtype="parking_lot"`. | `lane type="parking"` + `object type="parkingSpace"`. | FULL | `opendrive/parking.py`, `opendrive/enums.py::LaneType.PARKING`. | P2-1 (#448, closes #441). One synthetic road per `parking_lot` Area, two `PARKING`-typed lanes flanking the reference line, one `<object type="parkingSpace">` per `parking_space` LineString in the lot. AWSIM 新宿 has 0 parking lots so this is exercised only by other maps. |
| Guard rail / fence / wall LineStrings. | `object type="barrier"` / `object type="fence"`. | NOT_IMPLEMENTED | — | Not currently required. |
| Buildings, poles, trees. | `object type="building"` / `object type="pole"` / `object type="tree"`. | MISSING_IN_LANELET2 | — | Not part of Lanelet2's data model. |

## 10. Miscellaneous

| Lanelet2 side | OpenDRIVE side | Status | Module | Notes |
|---|---|---|---|---|
| Lanelet `one_way` attribute. | Implicit in OpenDRIVE lane direction (lanes on the left side go in one direction, lanes on the right in another). | FULL | `opendrive/lane_sections.py`. | Bidirectional lanes (`one_way="no"`) are modelled by emitting lanes on both sides of the reference line. |
| Traffic rule configuration (RHT vs LHT). | `road@rule`. | FULL | `main.py`, `opendrive/road.py`. | CARLA ue5-dev honors this (PR #8954); older builds do not. |
| `lane_change` boundary attribute. | `roadMark@laneChange`. | FULL | `opendrive/lane_elements.py`. | P0-3 (#426). |
| `detection_area` RE. | (no direct equivalent). | OUT_OF_SCOPE | — | Autoware-specific planning aid; no OpenDRIVE consumer. |
| `no_parking_area`, `no_stopping_area`. | Could be emitted as `object type="restrictedArea"` or `lane/access@rule="deny"`. | OUT_OF_SCOPE | — | Out of current scope. |
| Rail / tram. | `lane type="rail"` / `"tram"`. | OUT_OF_SCOPE | — | CARLA parses but NPCs don't use them. |

## 11. Cross-cutting concerns

### Geometry fidelity

- OpenDRIVE encodes lane shape via a single reference line plus width
  polynomials. Lanelet2 encodes left and right bounds independently. When
  left-right width is asymmetric or changes non-monotonically, the
  `paramPoly3 + width` representation cannot match the Lanelet2 polygon
  exactly. The converter's B-spline fit keeps the average error below 2 m and
  the peak below 8 m, but end-point tangents can drift; this is the root cause
  of the "mesh has holes" problem tracked in Confluence page 4666163223.

### ID scheme

- OpenDRIVE requires road/junction IDs to be unique across the entire map,
  signal IDs globally unique, lane IDs unique within a laneSection, and object
  IDs unique within a road. This converter partitions the integer space using
  `OpenDriveConstants.junction_id_offset = 1000` to avoid road/junction
  collisions.

### Conversion pipeline

The order below matches `main.py::_Lanelet2ToOpenDRIVEConverter.convert`:

1. Preprocess Lanelet2 map (merge / split / move / delete points, strip
   `turn_direction` from lanelets that must not be junctions).
2. Build non-junction roads (FULL / LOSSY rows in sections 2–4).
3. Build junction-internal connecting roads (sections 5, 2–4).
4. Fit `planView/geometry` per road (section 2).
5. Compute elevation profile per road (section 2).
6. Assign lane widths + roadMarks per laneSection (section 4).
7. Cross-link roads and lanes (section 3, 5).
8. Attach traffic lights, stop signs, stop lines and controllers (sections
   6–8).
9. Attach crosswalk and stop-line objects (sections 8–9).
10. Validate (`opendrive/validation.py`) and serialize (`opendrive/opendrive.py`).

### Information loss summary

| Area | Kind of loss |
|---|---|
| Geometry | `<line>`, `<arc>` and `<paramPoly3>` ship; clothoid (`<spiral>`) detection is still a stub so curvature transitions still fall back to paramPoly3. Elevation stepped at segment boundaries; superelevation discarded. |
| Lane shape | Asymmetric boundaries collapsed into width polynomial. Since #461 the converter no longer raises `AsymmetryLaneletException`; the worst-case loss is residual width error in the polynomial fit. |
| Road marks | None at the type/color/weight/laneChange level since P0-3 (#426). `material` still ignored. |
| Signals | Aspect / arrow info preserved in `@subtype` since #474/#475; CARLA TrafficManager continues to ignore `@subtype` so the loss is consumer-side only. |
| Static signs | Country-specific sign numbering forced to German StVO (P2-4 / #443 will fix). |
| Regulatory | `right_of_way` shipped as `<priority>` since P1-1 (#457). `detection_area`, `no_stopping_area`, `no_parking_area` discarded permanently. |
| Objects | `crosswalk`, `stopLine`, and `parkingSpace` (P2-1 / #448) ship today. Other static infrastructure (poles, fences, buildings) remains discarded. |

### Information gap summary (things CARLA needs that Lanelet2 does not provide)

| CARLA need | Gap | Mitigation |
|---|---|---|
| Pedestrian NavMesh | Sidewalks connected to crosswalks. | Converter side shipped: `lane[type=sidewalk]` (P0-1 / #427) plus `<lane><height>` curb step (#480). Topology gap remains on the Lanelet2 side: extend AWSIM 新宿 with `walkway`/`sidewalk` chains, or synthesize in preprocessing. |
| Junction priority phase | `right_of_way` on every junction. | Consumed since P1-1 (#457). Still UNSUPPORTED_BY_CARLA at runtime — TrafficManager ignores `<priority>`. |
| Road-level successor for diverging roads | Diverging roads must point `<successor>` at a junction so per-lane links are honoured. | Fixed (#472, closes #291). Diverging / merging road groups now generate a junction successor and per-lane link table. |
| Actor props (street-light 3D models) | Pole, building, fence geometries. | Out of scope for this converter. Supply via CARLA 3D assets separately. |
| TrafficManager stop enforcement | Actionable stop-line representation. | Emit dependencies between traffic lights and stop lines; upstream CARLA PR needed for TrafficManager to honor type 294. |
