# OpenDRIVE 1.4 + CARLA ue5-dev Profile

This document captures the subset of the ASAM OpenDRIVE specification that CARLA
ue5-dev (commit `fc52f323c1f05d615f0dce0e250bb235c8d8d39b`, the reference target
of this converter) actually parses and uses. It is the "profile" we must emit
against; anything outside this profile is either ignored by CARLA or handled
through CARLA-specific extensions.

Sources: CARLA `LibCarla/source/carla/opendrive/` parsers, ASAM OpenDRIVE 1.4H
and 1.8.1 specifications, Confluence page `4573266027`
("OpenDRIVEのなんのタグを読んでそうかリストアップする"), and direct inspection
of CARLA's MapBuilder.

## Overall posture

- **Declared version:** CARLA uses OpenDRIVE 1.4 as its baseline. Individual
  features from 1.5–1.6 (e.g. some `signalReference` attributes) are accepted
  but should not be relied upon.
- **XML parser:** pugixml. No schema validation; unknown tags are ignored
  silently.
- **Coordinate system:** `header/geoReference` string is parsed for
  `+lat_0=` / `+lon_0=` only. Projection strings other than lat/lon
  (UTM, MGRS) are ignored; if parsing fails, CARLA falls back to
  `(lat=42.0, lon=2.0)` (Barcelona).
- **ID uniqueness:** per ASAM spec:
  - `road@id`, `junction@id`, `controller@id`, `signal@id`: globally unique.
  - `lane@id`: unique within its `laneSection`.
  - `object@id`: unique within its road.
- **Unknown extensions:** CARLA accepts vendor `userData` children (RoadRunner
  UUIDs, `vectorLane`, `vectorSignal`) but does not use them for behavior.

## Header

| Element / Attribute | CARLA parses | Notes |
|---|---|---|
| `header@revMajor`, `@revMinor` | No | Read but unused. We emit `"1"`/`"4"`. |
| `header@name`, `@version`, `@date` | No | Ignored. |
| `header@north`, `@south`, `@east`, `@west` | No | Bounds currently written as `"0.0"` — CARLA never checks them. |
| `header/geoReference` (CDATA) | Yes | Only `+lat_0=` / `+lon_0=` tokens. Other PROJ4 tokens ignored. |
| `header/offset` (1.5+) | No | Not parsed. |
| `header/userData` | No | Ignored. |

## Road

### Attributes

| Attribute | CARLA parses | Notes |
|---|---|---|
| `road@id` | Yes | Integer-parseable string; CARLA stores as `RoadId`. |
| `road@name` | Yes (stored, unused for behavior) | |
| `road@length` | Yes | Must equal sum of `geometry@length` within tolerance. |
| `road@junction` | Yes | `"-1"` for regular road, otherwise the junction id. |
| `road@rule` | No | `RHT` / `LHT` tag. CARLA 0.10.0 ignored it; ue5-dev (post-PR #8954) respects it. |

### `road/link`

| Element | CARLA parses | Notes |
|---|---|---|
| `predecessor/successor@elementId`, `@elementType`, `@contactPoint` | Yes | `elementType` ∈ {road, junction}. `contactPoint` ∈ {start, end}. |
| `predecessor/successor@elementDir` (1.5+) | No | |
| `neighbor` (<1.5) | Parser reads but CARLA routing uses only predecessor/successor. | |

### `road/type`

| Attribute | CARLA parses | Notes |
|---|---|---|
| `type@s`, `@type` | Yes | Type values ∈ {town, rural, motorway, lowSpeed, pedestrian, bicycle, etc.}. |
| `type/speed@max`, `@unit` | Yes (parses) | **CARLA TrafficManager currently ignores road speed and hard-codes 30 km/h.** A PR (`carla-simulator/carla#9566`) adds enforcement. |

### `road/planView/geometry`

Primary road reference line. Each `<geometry>` sits at `@s` with pose
`(@x, @y, @hdg)` and `@length`.

| Child | CARLA parses | Notes |
|---|---|---|
| `line` | Yes | |
| `arc@curvature` | Yes | Constant curvature; curvature sign per OpenDRIVE convention. |
| `spiral@curvStart`, `@curvEnd` | Yes | Clothoid transition. |
| `poly3@a, b, c, d` | Yes | Deprecated in 1.6; CARLA still reads. |
| `paramPoly3@aU, bU, cU, dU, aV, bV, cV, dV, pRange` | Yes | `pRange ∈ {"arcLength", "normalized"}`. CARLA crashes if consecutive segments are very short; enforce `min_segment_length` (configured here to 0.5 m). |

### `road/elevationProfile/elevation`

| Attribute | CARLA parses | Notes |
|---|---|---|
| `@s, @a, @b, @c, @d` | Yes | Cubic polynomial `z = a + b·ds + c·ds² + d·ds³` with `ds = s − @s`. Omitting this block forces the road to lie on z=0. |

### `road/lateralProfile`

| Element | CARLA parses | Notes |
|---|---|---|
| `superelevation@s, a, b, c, d` | Yes (parses, applied to mesh generation) | Banking rotation around the reference line. Missing → zero bank. |
| `shape@s, t, a, b, c, d` | Yes (parses) | Cross-section deformation. Rarely needed for urban maps. |
| `crg` (1.5+) | No | Curved-regular-grid road surface. |

### `road/lanes`

#### `laneOffset`

`laneOffset@s, a, b, c, d` — shifts the center line of lanes orthogonally to the
reference line. **Parsed by CARLA; emitting it is required whenever the driving
center line does not coincide with the reference line.**

#### `laneSection`

- Attributes: `@s`, `@singleSide` (1.5+, CARLA ignores).
- Children: `left`, `center`, `right`. Within each, an ordered list of
  `<lane>` entries.

#### `lane`

| Attribute / Child | CARLA parses | Notes |
|---|---|---|
| `lane@id` | Yes | Negative on right, positive on left, 0 on center (must be exactly one center lane). |
| `lane@type` | Yes | See **lane types** below. |
| `lane@level` | Yes (default false) | If true, lane ignores superelevation and elevation. |
| `lane/link/predecessor@id`, `successor@id` | Yes | Connects to lanes in neighbouring sections or junction connections. |
| `lane/width@sOffset, a, b, c, d` | Yes | Cubic polynomial. `center` lane has no width. |
| `lane/border@sOffset, a, b, c, d` | Yes | Parsed but **must not coexist with `width` in the same lane** (spec rule). CARLA behavior with mixed widths/borders is untested; prefer `width`. |
| `lane/roadMark` | Yes | Attributes `@sOffset`, `@type`, `@weight`, `@color`, `@material`, `@width`, `@laneChange`, `@height`. Children `roadMark/type/line@*` define compound patterns. |
| `lane/material@sOffset, surface, friction, roughness` | Yes (parses; friction used by physics when present) | |
| `lane/speed@sOffset, max, unit` | Yes (parses) | Same TrafficManager caveat as road speed. |
| `lane/access@sOffset, rule, restriction` | Yes | `rule ∈ {allow, deny}`. |
| `lane/height@sOffset, inner, outer` | Yes | Lifts the lane surface above the base road mesh. Useful for sidewalks. |
| `lane/rule@sOffset, value` | Yes | Text attribute. |
| `lane/visibility` | No | Ignored. |
| `lane/userData` | No (routing), Yes (stored) | |

#### Lane types accepted by CARLA

CARLA's `LaneParser` accepts the following `lane@type` values; others map to
`"none"`:

```
driving, bidirectional, stop, shoulder, biking, sidewalk, parking, border,
restricted, median, entry, exit, onRamp, offRamp, rail, tram, roadWorks,
special1, special2, special3, none
```

For AWSIM-derived maps the relevant values are `driving`, `sidewalk`, `biking`,
`parking`, `shoulder`, `median`, `none`. `rail` and `tram` are parsed but not
usable by CARLA NPCs.

#### Road marks (`lane/roadMark`)

`roadMark@type` ∈ {
  `none`, `solid`, `broken`, `solid solid`, `solid broken`, `broken solid`,
  `broken broken`, `botts dots`, `grass`, `curb`, `custom`, `edge`
}.

`roadMark@weight` ∈ {`standard`, `bold`}. `roadMark@color` ∈
{`standard`, `yellow`, `white`, `red`, `green`, `blue`, `orange`}.
`roadMark@laneChange` ∈ {`increase`, `decrease`, `both`, `none`}.

CARLA uses only `type`, `color` and `laneChange` for behavior (lane-change
eligibility).

## Junctions

`junction@id`, `@name`, `@type` (ue5-dev also understands
`@type="direct"` and `@type="virtual"`; default is `default`).

### `junction/connection`

| Attribute | CARLA parses | Notes |
|---|---|---|
| `@id` | Yes | |
| `@incomingRoad` | Yes | Road connecting into the junction. |
| `@connectingRoad` | Yes (default/virtual) | Road inside the junction. |
| `@linkedRoad` | Yes (direct junction) | Used only for `type="direct"`. |
| `@contactPoint` | Yes | `start` / `end` on the connecting road. |
| `@connectionMaster` | No | |
| `@type` (1.5+) | No | |

### `junction/connection/laneLink@from, @to`

Pairs incoming lane id with connecting lane id. Multiple laneLinks per connection
are allowed.

### `junction/priority@high, @low`

Parsed by CARLA's JunctionParser but **TrafficManager does not honor it**. Emit
for spec compliance only.

### `junction/controller@id, @type, @sequence`

Reference from the junction to a top-level `<controller>`. Used for traffic
light coordination.

## Signals and controllers

### `<road>/<signals>/<signal>`

Top-level attributes:

| Attribute | CARLA | Notes |
|---|---|---|
| `@id` | globally unique | |
| `@s, @t` | lateral/longitudinal position on the road | |
| `@zOffset` | height above road surface | |
| `@name` | stored | |
| `@dynamic` | yes/no; `yes` means traffic-light-like | |
| `@orientation` | `+`, `-`, `none` | direction of travel the signal applies to |
| `@country` | `OpenDRIVE`, `DE`, `US`, `JP`, ... | **CARLA interprets the type codes as German StVO regardless of country.** |
| `@type`, `@subtype`, `@value`, `@unit` | See **signal type codes** below. | |
| `@width`, `@height` | dimensions of the physical sign | |
| `@hOffset`, `@pitch`, `@roll` | orientation offsets | |
| `@text` | auxiliary text, unused | |

Children:

- `signal/validity@fromLane, @toLane` — lane range where the signal applies.
- `signal/dependency@id, @type` — references another signal id (e.g., stop
  line type 294 depends on traffic light type 1000001).
- `signal/positionInertial@x, @y, @z, @hdg, @pitch, @roll` — world-frame
  override of `(s, t, zOffset)`. CARLA prefers this when present.
- `signal/positionRoad` — not used by the converter.
- `signal/reference@elementType, @elementId, @type` — back-reference to other
  signals (emitted from 1.5; parsed by ue5-dev CARLA).
- `signal/userData/vectorSignal@signalId` — RoadRunner UUID, stored.

#### Signal type codes used by the converter

CARLA's `SignalType.cpp` defines the semantic buckets below. Type numbers are
StVO codes; the converter therefore always emits `country="OpenDRIVE"` and
the German IDs regardless of the input map country.

| Semantic | type | subtype | Notes |
|---|---|---|---|
| Traffic light (3-aspect) | `1000001` | `-1` | Dynamic. CARLA spawns a 3D actor by default. |
| Stop sign | `206` | `-1` | Static, mandatory stop. |
| Yield sign | `205` | `-1` | Static. |
| Stop line (marking) | `294` | `-1` | Parsed but **TrafficManager does not stop at these**. Use dependency to a traffic light signal instead. |

`<signalReference>` may appear alongside `<signal>` to mirror a signal's effect
onto another road without duplicating the physical definition.

### `<controller>` (root-level)

Coordinates traffic light phases.

| Element / Attribute | CARLA parses | Notes |
|---|---|---|
| `controller@id, @name, @sequence` | Yes | |
| `controller/control@signalId, @type` | Yes | Type string is free-form; CARLA mainly uses signalId grouping. |

### `<junction>/<controller>@id, @type, @sequence`

Reference from a junction to a root controller. CARLA binds the referenced
controller to the junction for signal-phase management.

## Objects

`<road>/<objects>/<object>`.

| Attribute | Purpose | CARLA uses |
|---|---|---|
| `@type` | `crosswalk`, `barrier`, `parkingSpace`, `pole`, `building`, ... | Only `crosswalk` drives behavior (pedestrian NavMesh). |
| `@subtype`, `@name`, `@s`, `@t`, `@zOffset`, `@hdg`, `@pitch`, `@roll`, `@orientation`, `@validLength`, `@length`, `@width`, `@height`, `@radius` | Standard placement | Parsed |
| `@perpToRoad` | Billboard flag | Parsed |
| `object/outline/cornerLocal@u, @v, @z, @height` | Polygonal extent in road-local frame | Used by crosswalk pedestrian NavMesh. |
| `object/outline/cornerRoad` | Alternate outline mode | Parsed |
| `object/validity` | Lane validity | Parsed |
| `object/repeat`, `object/markings`, `object/surface`, `object/borders` | Repeating / surface features | Parsed but rarely used for AWSIM-class maps. |

### Object types exercised by this converter

- `crosswalk` — polygon outline derived from `Lanelet subtype="crosswalk"`.
- `stopLine` — simple rectangle derived from `Linestring type="stop_line"`.
  Not a standard OpenDRIVE object type; CARLA currently ignores it for
  TrafficManager behavior.

### Types not yet emitted but candidates for future work

`parkingSpace`, `barrier`, `building`, `pole`, `fence`, `pedestrianCrossing`
(traffic island). All are read by CARLA but have no navigation impact; they
are purely decorative or used for collision meshes.

## CARLA-specific extensions currently observed

- **`trafficGroup` tag** — present in CARLA's `TrafficGroupParser.cpp` but
  entirely commented out as of the reference commit. Treat as "not supported".
- **`userData/vectorLane` / `userData/vectorSignal`** — RoadRunner identifiers,
  accepted but not used.
- **`road@rule="LHT"`** — supported from CARLA PR #8954 (2025-06). Required for
  Japanese maps. CARLA 0.10.0 binaries do not include this; use a freshly
  built ue5-dev Python wheel.

## Tags that CARLA ue5-dev does **not** parse

Including, but not limited to:

- `railroad/switch`
- `surface/CRG`
- `include` (external file references)
- `station`, `stationGroup`
- `road/surface`
- `road/link/neighbor` (beyond predecessor/successor routing)
- Most 1.5+ additions: `signalGroup`, `station`, `objectReference` attributes
  beyond `@id`.

Emitting any of the above is harmless (CARLA ignores) but adds no value for
the ue5-dev target.

## Implications for the converter

1. **Stay in 1.4 wire format**. Emit `revMajor="1"` / `revMinor="4"` and avoid
   1.5+-only children unless CARLA's parser is known to accept them.
2. **Always emit `geoReference`** with valid `+lat_0` and `+lon_0`. Missing
   values silently relocate the map to Barcelona.
3. **Use `paramPoly3` as the default geometry primitive**, segmented so every
   segment is ≥ 0.5 m. CARLA-sourced crashes have been observed for shorter
   segments.
4. **Elevation is mandatory** for anything non-flat; omitting `elevationProfile`
   forces the road onto `z=0`. `superelevation`/`shape` are optional but
   should be considered for banked curves.
5. **For traffic behavior we have to emit, in this order**:
   - `signal type=1000001` for each traffic light bulb group, with
     `positionInertial` populated from Lanelet2.
   - `controller` elements binding co-phased traffic lights; junction
     `<controller>` references to link them.
   - `signal type=206` (stop sign) / `205` (yield) for static signs.
   - `signal type=294` as optional metadata. Do not rely on TrafficManager
     obeying it; stop behavior must be expressed via `dependency` from the
     traffic light back to the stop line or via the junction boundary.
6. **Lane types must come from the German/StVO worldview** CARLA expects.
   Japanese-specific lane types (e.g. Japanese expressway shoulder) must be
   coerced to one of the CARLA-recognized values.
7. **`roadMark` details are honored for lane-change permissibility only.** Fine
   granularity (e.g., double yellow vs single white) is purely cosmetic in
   CARLA today.
8. **Expect `priority` and fine-grained `validity` to be ignored** by
   TrafficManager. Include them for spec compliance, not for behavior.

## References

- ASAM OpenDRIVE 1.8.1 specification — `https://publications.pages.asam.net/standards/ASAM_OpenDRIVE/`
- ASAM OpenDRIVE 1.4H (original release) — `https://www.asam.net/standards/detail/opendrive/`
- CARLA parsers — `https://github.com/carla-simulator/carla/tree/ue5-dev/LibCarla/source/carla/opendrive`
- CARLA PR #8954 (LHT support) — `https://github.com/carla-simulator/carla/pull/8954`
- CARLA PR #9566 (road speed limit) — `https://github.com/carla-simulator/carla/pull/9566`
- Confluence — OpenDRIVEのなんのタグを読んでそうかリストアップする
  (page id `4573266027`).
