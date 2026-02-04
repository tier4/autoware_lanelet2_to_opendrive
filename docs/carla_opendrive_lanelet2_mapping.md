# CARLA OpenDRIVE and Lanelet2 Tag Mapping

This document describes how OpenDRIVE tags are used within CARLA and how Lanelet2 tags are processed and mapped to OpenDRIVE format.

## Overview

CARLA UE5 uses a modular OpenDRIVE parser system located in `LibCarla/source/carla/opendrive/` directory. The parser consists of specialized components that handle different aspects of the OpenDRIVE specification:

- **XML Parsing**: Uses the `pugixml` library for XML processing
- **Modular Architecture**: Separate parser classes for different OpenDRIVE elements
- **Map Building**: Constructs CARLA's internal road network representation from parsed data

This document focuses on which OpenDRIVE tags CARLA reads and how they are used internally, which is essential for understanding what needs to be generated when converting from Lanelet2 format.

## OpenDRIVE Tags Used in CARLA

CARLA's OpenDRIVE parser reads the following tags and attributes. All tags are organized in a single comprehensive table:

| Parser Module | OpenDRIVE Tag/Attribute | Purpose | Used For | CARLA Code Location |
|---------------|-------------------------|---------|----------|---------------------|
| GeoReferenceParser | `header/geoReference` | PROJ format georeference string | Geographic coordinate system definition | [`GeoReferenceParser.cpp` L62](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/GeoReferenceParser.cpp#L62) |
| GeoReferenceParser | `+lat_0=` | Latitude origin | Origin point for coordinate transformation | [`GeoReferenceParser.cpp` L28-L50](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/GeoReferenceParser.cpp#L28-L50) |
| GeoReferenceParser | `+lon_0=` | Longitude origin | Origin point for coordinate transformation | [`GeoReferenceParser.cpp` L28-L50](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/GeoReferenceParser.cpp#L28-L50) |
| RoadParser | `road@id` | Road identifier | Unique road identification | [`RoadParser.cpp` L113-L120](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L113-L120) |
| RoadParser | `road@name` | Road name | Road naming/labeling | [`RoadParser.cpp` L113-L120](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L113-L120) |
| RoadParser | `road@length` | Road length (meters) | Road geometry calculation | [`RoadParser.cpp` L113-L120](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L113-L120) |
| RoadParser | `road@junction` | Junction ID reference | Links road to junction | [`RoadParser.cpp` L113-L120](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L113-L120) |
| RoadParser | `road/link/predecessor@elementId` | Previous road ID | Road connectivity | [`RoadParser.cpp` L122-L130](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L122-L130) |
| RoadParser | `road/link/successor@elementId` | Next road ID | Road connectivity | [`RoadParser.cpp` L122-L130](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L122-L130) |
| RoadParser | `road/type@s` | Start position | Road type section position | [`RoadParser.cpp` L133-L145](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L133-L145) |
| RoadParser | `road/type@type` | Road type | Road classification (town, highway, etc.) | [`RoadParser.cpp` L133-L145](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L133-L145) |
| RoadParser | `road/type/speed@max` | Maximum speed | Speed limit | [`RoadParser.cpp` L133-L145](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L133-L145) |
| RoadParser | `road/type/speed@unit` | Speed unit | Speed unit (km/h, mph, etc.) | [`RoadParser.cpp` L133-L145](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L133-L145) |
| RoadParser | `road/lanes/laneOffset@s,a,b,c,d` | Lane offset parameters | Lane lateral offset polynomial | [`RoadParser.cpp` L147-L155](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L147-L155) |
| RoadParser | `road/lanes/laneSection@s` | Lane section start | Lane section definition | [`RoadParser.cpp` L157-L180](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L157-L180) |
| RoadParser | `lane@id` | Lane identifier | Lane identification | [`RoadParser.cpp` L157-L180](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L157-L180) |
| RoadParser | `lane@type` | Lane type | Lane classification | [`RoadParser.cpp` L157-L180](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L157-L180) |
| RoadParser | `lane@level` | Lane level | Stacked lane handling | [`RoadParser.cpp` L157-L180](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L157-L180) |
| RoadParser | `lane/link/predecessor` | Previous lane | Lane connectivity | [`RoadParser.cpp` L157-L180](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L157-L180) |
| RoadParser | `lane/link/successor` | Next lane | Lane connectivity | [`RoadParser.cpp` L157-L180](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L157-L180) |
| GeometryParser | `geometry@s` | Start position along road | Geometry segment positioning | [`GeometryParser.cpp` L77-L84](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/GeometryParser.cpp#L77-L84) |
| GeometryParser | `geometry@x` | X coordinate | Geometry start point | [`GeometryParser.cpp` L77-L84](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/GeometryParser.cpp#L77-L84) |
| GeometryParser | `geometry@y` | Y coordinate | Geometry start point | [`GeometryParser.cpp` L77-L84](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/GeometryParser.cpp#L77-L84) |
| GeometryParser | `geometry@hdg` | Heading angle | Geometry orientation | [`GeometryParser.cpp` L77-L84](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/GeometryParser.cpp#L77-L84) |
| GeometryParser | `geometry@length` | Geometry length | Segment length | [`GeometryParser.cpp` L77-L84](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/GeometryParser.cpp#L77-L84) |
| GeometryParser | `geometry/line` | Straight line | Linear road segment | [`GeometryParser.cpp` L89, L120](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/GeometryParser.cpp#L89) |
| GeometryParser | `geometry/arc@curvature` | Arc curvature | Curved road segment | [`GeometryParser.cpp` L88-91, L122-123](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/GeometryParser.cpp#L88-L91) |
| GeometryParser | `geometry/spiral@curvStart` | Spiral start curvature | Clothoid transition | [`GeometryParser.cpp` L91-94, L124-131](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/GeometryParser.cpp#L91-L94) |
| GeometryParser | `geometry/spiral@curvEnd` | Spiral end curvature | Clothoid transition | [`GeometryParser.cpp` L91-94, L124-131](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/GeometryParser.cpp#L91-L94) |
| GeometryParser | `geometry/poly3@a,b,c,d` | Cubic polynomial coefficients | Parametric road shape | [`GeometryParser.cpp` L94-99, L133-144](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/GeometryParser.cpp#L94-L99) |
| GeometryParser | `geometry/paramPoly3@aU,bU,cU,dU` | U-direction polynomial | Parametric curve U | [`GeometryParser.cpp` L99-110, L144-159](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/GeometryParser.cpp#L99-L110) |
| GeometryParser | `geometry/paramPoly3@aV,bV,cV,dV` | V-direction polynomial | Parametric curve V | [`GeometryParser.cpp` L99-110, L144-159](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/GeometryParser.cpp#L99-L110) |
| GeometryParser | `geometry/paramPoly3@pRange` | Parameter range | Parametric curve range | [`GeometryParser.cpp` L99-110, L144-159](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/GeometryParser.cpp#L99-L110) |
| LaneParser | `lane/width@sOffset,a,b,c,d` | Lane width polynomial | Lane width calculation | [`LaneParser.cpp` L17-L185](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/LaneParser.cpp#L17-L185) |
| LaneParser | `lane/border@sOffset,a,b,c,d` | Lane border polynomial | Lane boundary definition | [`LaneParser.cpp` L17-L185](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/LaneParser.cpp#L17-L185) |
| LaneParser | `lane/roadMark@*` | Road marking attributes | Lane markings/striping | [`LaneParser.cpp` L17-L185](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/LaneParser.cpp#L17-L185) |
| LaneParser | `lane/roadMark/type/line@*` | Line marking details | Detailed marking geometry | [`LaneParser.cpp` L17-L185](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/LaneParser.cpp#L17-L185) |
| LaneParser | `lane/material@*` | Lane surface material | Surface properties | [`LaneParser.cpp` L17-L185](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/LaneParser.cpp#L17-L185) |
| LaneParser | `lane/speed@sOffset` | Speed limit start position | Lane-specific speed limit | [`LaneParser.cpp` L17-L185](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/LaneParser.cpp#L17-L185) |
| LaneParser | `lane/speed@max` | Maximum speed | Lane speed restriction | [`LaneParser.cpp` L17-L185](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/LaneParser.cpp#L17-L185) |
| LaneParser | `lane/speed@unit` | Speed unit | Speed unit specification | [`LaneParser.cpp` L17-L185](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/LaneParser.cpp#L17-L185) |
| LaneParser | `lane/access@sOffset` | Access rule start position | Lane access restrictions | [`LaneParser.cpp` L17-L185](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/LaneParser.cpp#L17-L185) |
| LaneParser | `lane/access@rule` | Access rule type | Access control | [`LaneParser.cpp` L17-L185](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/LaneParser.cpp#L17-L185) |
| LaneParser | `lane/access@restriction` | Restriction details | Detailed access rules | [`LaneParser.cpp` L17-L185](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/LaneParser.cpp#L17-L185) |
| LaneParser | `lane/height@sOffset` | Height change position | Vertical lane offset | [`LaneParser.cpp` L17-L185](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/LaneParser.cpp#L17-L185) |
| LaneParser | `lane/height@inner` | Inner edge height | Height at inner boundary | [`LaneParser.cpp` L17-L185](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/LaneParser.cpp#L17-L185) |
| LaneParser | `lane/height@outer` | Outer edge height | Height at outer boundary | [`LaneParser.cpp` L17-L185](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/LaneParser.cpp#L17-L185) |
| LaneParser | `lane/rule@sOffset,value` | Lane rule position/value | Lane-specific rules | [`LaneParser.cpp` L17-L185](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/LaneParser.cpp#L17-L185) |
| LaneParser | `lane/visibility@*` | Visibility attributes | Lane visibility properties | [`LaneParser.cpp` L17-L185](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/LaneParser.cpp#L17-L185) |
| ProfilesParser | `elevationProfile/elevation@s` | Elevation start position | Vertical profile positioning | [`ProfilesParser.cpp` L46-L80](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/ProfilesParser.cpp#L46-L80) |
| ProfilesParser | `elevationProfile/elevation@a,b,c,d` | Elevation polynomial | Vertical road shape | [`ProfilesParser.cpp` L46-L80](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/ProfilesParser.cpp#L46-L80) |
| ProfilesParser | `lateralProfile/superelevation@s` | Superelevation start | Banking start position | [`ProfilesParser.cpp` L46-L80](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/ProfilesParser.cpp#L46-L80) |
| ProfilesParser | `lateralProfile/superelevation@a,b,c,d` | Superelevation polynomial | Road banking/tilt | [`ProfilesParser.cpp` L46-L80](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/ProfilesParser.cpp#L46-L80) |
| ProfilesParser | `lateralProfile/shape@s` | Shape change start | Lateral shape positioning | [`ProfilesParser.cpp` L46-L80](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/ProfilesParser.cpp#L46-L80) |
| ProfilesParser | `lateralProfile/shape@t` | Lateral offset | Transverse position | [`ProfilesParser.cpp` L46-L80](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/ProfilesParser.cpp#L46-L80) |
| ProfilesParser | `lateralProfile/shape@a,b,c,d` | Shape polynomial | Lateral profile shape | [`ProfilesParser.cpp` L46-L80](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/ProfilesParser.cpp#L46-L80) |
| JunctionParser | `junction@id` | Junction identifier | Unique junction identification | [`JunctionParser.cpp` L17-L50](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/JunctionParser.cpp#L17-L50) |
| JunctionParser | `junction@name` | Junction name | Junction naming/labeling | [`JunctionParser.cpp` L17-L50](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/JunctionParser.cpp#L17-L50) |
| JunctionParser | `junction/connection@id` | Connection identifier | Connection identification | [`JunctionParser.cpp` L17-L50](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/JunctionParser.cpp#L17-L50) |
| JunctionParser | `junction/connection@incomingRoad` | Incoming road ID | Junction entry point | [`JunctionParser.cpp` L17-L50](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/JunctionParser.cpp#L17-L50) |
| JunctionParser | `junction/connection@connectingRoad` | Connecting road ID | Junction internal path | [`JunctionParser.cpp` L17-L50](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/JunctionParser.cpp#L17-L50) |
| JunctionParser | `junction/connection@contactPoint` | Connection point type | Start/end specification | [`JunctionParser.cpp` L17-L50](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/JunctionParser.cpp#L17-L50) |
| JunctionParser | `junction/connection/laneLink@from` | Source lane ID | Lane-level connection source | [`JunctionParser.cpp` L17-L50](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/JunctionParser.cpp#L17-L50) |
| JunctionParser | `junction/connection/laneLink@to` | Target lane ID | Lane-level connection target | [`JunctionParser.cpp` L17-L50](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/JunctionParser.cpp#L17-L50) |
| JunctionParser | `junction/controller@id` | Controller ID | Traffic signal reference | [`JunctionParser.cpp` L17-L50](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/JunctionParser.cpp#L17-L50) |
| JunctionParser | `junction/controller@type` | Controller type | Controller classification | [`JunctionParser.cpp` L17-L50](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/JunctionParser.cpp#L17-L50) |

### Parser-Specific Notes

#### GeoReferenceParser
- CARLA only reads latitude/longitude origins; UTM/MGRS zones are not directly supported
- Geographic reference is optional in OpenDRIVE but recommended for proper geo-location

#### RoadParser
**Supported Lane Types** ([`RoadParser.cpp` L66-L110](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L66-L110)):
- `driving`, `bidirectional`, `stop`, `shoulder`, `biking`, `sidewalk`
- `parking`, `border`, `restricted`, `median`, `entry`, `exit`
- `onRamp`, `offRamp`, `rail`, `tram`, `roadWorks`
- `special1`, `special2`, `special3`, `none`

**Important Considerations:**
- **Road Concept Challenge**: Lanelet2 doesn't have a "Road" concept; conversion must group Lanelets into Roads
- **Lane Speed Override**: Lane speed limits override road speed limits (OpenDRIVE spec 11.7)
- **Signal Priority**: Speed limits from signals always have preference over road/lane speed limits

#### GeometryParser
**Lanelet2 Conversion Notes:**
- **Simple Case**: Lanelet2 centerlines can be converted to `<line>` geometries (straight segments)
- **Advanced Case**: For smoother roads, consider using `<spiral>` or `<paramPoly3>` with B-spline fitting

#### LaneParser
**Width vs Border Conflict** (OpenDRIVE Spec 1.8c):
- `<width>` and `<border>` are mutually exclusive within the same lane group
- If both exist, applications **must use** `<width>` elements
- **Recommendation**: Use `<border>` for Lanelet2 conversion (more direct mapping)
- CommonRoadScenarioDesigner uses `<width>`; this converter should choose based on requirements

**Lanelet2 Challenges:**
- **No Width Concept**: Lanelet2 doesn't have lane width calculation functions
- **Road+Lane ID Required**: Physical lane shape requires both Road ID and Lane ID
- **Relative Positioning**: Lane IDs are relative within road sections

#### ProfilesParser
**Important Notes:**
- **Optional but Critical**: While optional in OpenDRIVE spec, `<elevationProfile>` is essential for proper height representation in CARLA
- **Lanelet2 Integration**: Lanelet2 has elevation data (z-coordinates); should be converted to elevation profile

#### JunctionParser
**Related MapBuilder Functions:**
- `AddJunction`: [`MapBuilder.cpp` L566](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/road/MapBuilder.cpp#L566)
- `AddConnection`: [`MapBuilder.cpp` L570](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/road/MapBuilder.cpp#L570)
- `AddLaneLink`: [`MapBuilder.cpp` L580](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/road/MapBuilder.cpp#L580)

**Junction Semantics:**
- **laneLink**: Defines lane-level connectivity within junction
- **connection**: Defines road-level connectivity (which roads connect to junction)
- **Road Assignment**: Roads must specify which junction they belong to via `road@junction` attribute

## Lanelet2 to OpenDRIVE Tag Mapping

This section describes how Lanelet2 tags and attributes are converted to OpenDRIVE format by this converter.

### Road-Level Mapping

| Lanelet2 Tag | Lanelet2 Value | OpenDRIVE Element | OpenDRIVE Value | Conversion Notes |
|--------------|----------------|-------------------|-----------------|------------------|
| `location` | `"urban"` | `road/type@type` | `TOWN` | Road type classification |
| `location` | `"highway"` | `road/type@type` | `MOTORWAY` | Highway classification |
| `location` | `"rural"` | `road/type@type` | `RURAL` | Rural road classification |
| `location` | `"private"` (≤10 km/h) | `road/type@type` | `LOW_SPEED` | Private/slow roads |
| `speed_limit` | numeric (km/h) | `road/type/speed@max` | float | Road speed limit |
| (N/A) | (inferred) | `road@id` | auto-generated | No direct Lanelet2 equivalent |
| (N/A) | (inferred) | `road@length` | calculated | Computed from geometry |
| (connectivity) | successor/predecessor | `road/link/predecessor` | lanelet ID mapping | Connectivity conversion |
| (connectivity) | successor/predecessor | `road/link/successor` | lanelet ID mapping | Connectivity conversion |

**Speed Limit Fallback** (when `location` tag is missing):
- `speed ≤ 10 km/h` → `LOW_SPEED`
- `10 < speed ≤ 40 km/h` → `TOWN`
- `40 < speed ≤ 90 km/h` → `RURAL`
- `speed > 90 km/h` → `MOTORWAY`

**Road Grouping Algorithm** (for Lanelet2 → OpenDRIVE conversion):
1. Exclude junction lanelets (those with `turn_direction` tag)
2. Find lanelets adjacent to junctions or with no successor
3. Perform adjacency testing and group lanelets
4. Select one lanelet per group as reference line
5. Convert each group to an OpenDRIVE Road

### Lane-Level Mapping

| Lanelet2 Tag | Lanelet2 Value | OpenDRIVE Element | OpenDRIVE Value | Conversion Notes |
|--------------|----------------|-------------------|-----------------|------------------|
| `subtype` | `"road"` | `lane@type` | `driving` | Standard driving lane |
| `subtype` | `"highway"` | `lane@type` | `driving` | Highway driving lane |
| `subtype` | `"walkway"` | `lane@type` | `sidewalk` | Pedestrian sidewalk |
| `subtype` | `"bicycle_lane"` | `lane@type` | `biking` | Bicycle lane |
| (default) | (none) | `lane@type` | `driving` | Default lane type |
| `speed_limit` | numeric (km/h) | `lane/speed@max` | float | Lane-specific speed (overrides road) |
| (left/right boundary) | LineString3d | `lane/border` | polynomial | Lanelet boundaries → lane borders |
| (centerline) | LineString3d | `geometry/line` | points | Lanelet centerline → road geometry |

**Lane Width Considerations:**
- Lanelet2 has no built-in lane width concept
- Width must be computed from left/right boundary distances
- Converter uses `lane/border` for more direct geometric mapping

### Geometry Mapping

| Lanelet2 Element | OpenDRIVE Element | Conversion Method | Implementation Notes |
|------------------|-------------------|-------------------|----------------------|
| Lanelet centerline (LineString3d) | `road/planView/geometry/line` | Direct point-to-point conversion | Simplest approach; results in many short segments |
| Lanelet centerline (LineString3d) | `road/planView/geometry/paramPoly3` | B-spline fitting | Smoother roads; requires spline algorithm |
| Lanelet left/right boundaries | `lane/border@a,b,c,d` | Polynomial fitting | Cubic polynomial fitting to boundary points |
| Lanelet elevation (z-coordinates) | `road/elevationProfile/elevation` | Z-value extraction and fitting | Critical for 3D road representation |

**Geometry Conversion Approach:**
- **Phase 1**: Use `<line>` elements for straightforward conversion
- **Phase 2** (optional): Implement B-spline fitting for smoother geometry using `<paramPoly3>` or `<spiral>`

### Junction Mapping

| Lanelet2 Tag/Attribute | OpenDRIVE Element | Conversion Method | Implementation Notes |
|------------------------|-------------------|-------------------|----------------------|
| `turn_direction` (presence) | `junction` element | Lanelet classified as junction | Lanelets with this tag → junction connections |
| `turn_direction` | (classification only) | Not directly mapped | Used for lanelet classification, not stored in OpenDRIVE |
| successor/predecessor (junction lanelet) | `junction/connection` | Connectivity mapping | Maps incoming/outgoing roads to junction |
| lane ID (junction lanelet) | `junction/connection/laneLink` | Lane-level connections | Maps specific lane connections through junction |

**Junction Conversion Process:**
1. Identify all lanelets with `turn_direction` tag
2. Group spatially overlapping junction lanelets
3. Create `<junction>` element for each group
4. Generate `<connection>` elements for each path through junction
5. Create connecting roads (roads with `@junction` attribute)
6. Establish `<laneLink>` for lane-level connectivity

### Traffic Signal Mapping

| Lanelet2 Regulatory Element | OpenDRIVE Element | Conversion Method | Implementation Notes |
|------------------------------|-------------------|-------------------|----------------------|
| Traffic light `type` or `subtype` | `signal@type` | Signal type classification | Maps to OpenDRIVE signal types |
| `"red_yellow_green"` or `"3_lights"` | `TRAFFIC_LIGHT_3_LIGHTS` | Direct mapping | Standard 3-color traffic light |
| `"pedestrian"` | `TRAFFIC_LIGHT_PEDESTRIAN` | Direct mapping | Pedestrian signal |
| `"arrow"` | `TRAFFIC_LIGHT_ARROW` | Direct mapping | Directional arrow signal |
| `trafficLights` (LineString3d) | `signal@s,t` | Position extraction | Signal position in road coordinates |
| Regulatory element → lanelet | `signal@orientation` | Affected lane determination | Which lanes signal controls |

**Signal Positioning:**
- Extract 3D position from traffic light LineString geometry
- Convert to road-relative (s, t) coordinates
- `s`: distance along road reference line
- `t`: lateral offset from reference line

### Coordinate System Mapping

| Lanelet2 Element | OpenDRIVE Element | Conversion Method | Implementation Notes |
|------------------|-------------------|-------------------|----------------------|
| MGRS grid code | `header/geoReference` | MGRS → lat/lon → PROJ string | Convert MGRS to geographic coordinates |
| Latitude/Longitude origin | `header/geoReference` | Direct PROJ string generation | `+lat_0=` and `+lon_0=` parameters |
| Local XY coordinates | `geometry@x,y` | Direct use or offset subtraction | May apply coordinate offset transformation |
| Point z-coordinate (`ele`) | `elevationProfile/elevation` | Polynomial fitting | Extract elevation profile from 3D points |

**Coordinate Transformation Workflow:**
1. Parse origin specification (MGRS or lat/lon)
2. Create PROJ string for `<geoReference>`
3. Apply coordinate offset (if specified)
4. Generate road geometry in local coordinates
5. Ensure elevation data is captured in `<elevationProfile>`

## CARLA-Specific Considerations

??? info "CARLA Compatibility Requirements and Recommendations"
    Based on the CARLA parser analysis, the following considerations are important for CARLA compatibility:

    ### Required Elements

    CARLA **requires** these elements for proper map loading:

    1. **Geographic Reference** (`header/geoReference`):
       - Must include `+lat_0=` and `+lon_0=` parameters
       - CARLA uses this for coordinate system initialization

    2. **Road Geometry** (`road/planView/geometry`):
       - At least one geometry element per road
       - `<line>` is the simplest and always supported

    3. **Lane Sections** (`road/lanes/laneSection`):
       - At least one lane section per road
       - Lane count changes require new lane sections

    4. **Elevation Profile** (`road/elevationProfile`):
       - Optional in OpenDRIVE spec but critical for CARLA
       - Without this, roads may appear at incorrect heights

    ### Optional but Recommended

    1. **Lane Borders** (`lane/border`):
       - Preferred over `lane/width` for geometric accuracy
       - More direct mapping from Lanelet2 boundaries

    2. **Speed Limits**:
       - Can be specified at road level (`road/type/speed`) or lane level (`lane/speed`)
       - Signal speed limits override road/lane speeds

    3. **Junction Connections**:
       - Required for intersection navigation
       - Must include `laneLink` for lane-level routing

    ### Elements Not Used by CARLA

    The following OpenDRIVE elements are **not** currently parsed by CARLA (based on parser code analysis):

    - `road/objects` (roadside objects) - Has ObjectParser but usage unclear
    - `road/signals` (road-mounted signals) - Has SignalParser but only traffic lights confirmed
    - `road/surface` (road surface properties)
    - `lane/userData` (custom lane data)
    - Complex geometry types beyond those listed (e.g., `<spiral>` support uncertain)

## Conversion Challenges and Solutions

| Challenge | Description | Solution Approach | Implementation Status |
|-----------|-------------|-------------------|----------------------|
| Road Grouping | Lanelet2 has no "Road" concept | Group connected lanelets by connectivity | ✅ Implemented |
| Lane Width | Lanelet2 lacks width calculation | Compute from boundary separation | ✅ Implemented |
| Smooth Geometry | Line segments vs. curves | B-spline fitting to centerline | ✅ Implemented (optional) |
| Junction Detection | Must identify intersections | Use `turn_direction` tag | ✅ Implemented |
| Elevation Profile | Convert z-coordinates to polynomial | Fit cubic polynomial to elevation data | ⚠️ Basic implementation |
| Signal Positioning | Convert to (s, t) coordinates | Project signal position onto road | ✅ Implemented |
| Lane Connectivity | OpenDRIVE uses Road+Lane IDs | Map Lanelet IDs to Road+Lane pairs | ✅ Implemented |

**Legend:**
- ✅ Fully implemented
- ⚠️ Partially implemented or basic support
- ❌ Not implemented

## References

### CARLA Documentation

- [CARLA OpenDRIVE Parser Source Code](https://github.com/carla-simulator/carla/tree/master/LibCarla/source/carla/opendrive/parser)
- [CARLA UE5 OpenDRIVE Integration](https://tier4.atlassian.net/wiki/spaces/T4DC/pages/4573266027/OpenDRIVE) (Internal Tier4 documentation)

### OpenDRIVE Specification

- [ASAM OpenDRIVE Specification v1.8](https://publications.pages.asam.net/standards/ASAM_OpenDRIVE/ASAM_OpenDRIVE_Specification/v1.8.1/specification/)
- [OpenDRIVE Coordinate Systems](https://publications.pages.asam.net/standards/ASAM_OpenDRIVE/ASAM_OpenDRIVE_Specification/v1.8.1/specification/08_coordinate_systems/08_05_geo_referencing.html)
- [OpenDRIVE Geometries](https://publications.pages.asam.net/standards/ASAM_OpenDRIVE/ASAM_OpenDRIVE_Specification/v1.8.1/specification/09_geometries/09_01_introduction.html)
- [OpenDRIVE Junctions](https://publications.pages.asam.net/standards/ASAM_OpenDRIVE/ASAM_OpenDRIVE_Specification/v1.8.1/specification/12_junctions/12_04_connecting_roads.html)
- [OpenDRIVE Lane Properties](https://publications.pages.asam.net/standards/ASAM_OpenDRIVE/ASAM_OpenDRIVE_Specification/v1.8.1/specification/11.7_additional_lane_properties/README.html)

### Converter Documentation

- [Conversion Process Flow](conversion-process.md) - Detailed conversion pipeline documentation
- [Signals Documentation](signals.md) - Traffic signal conversion details
- [Known Limitations](limitations.md) - Current converter limitations and CARLA compatibility notes

### Related Tools

- [Lanelet2](https://github.com/fzi-forschungszentrum-informatik/Lanelet2) - Lanelet2 library and format specification
- [CommonRoad Scenario Designer](https://commonroad.in.tum.de/) - Alternative Lanelet2 to OpenDRIVE converter
