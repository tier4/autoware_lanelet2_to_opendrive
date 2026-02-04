# CARLA OpenDRIVE and Lanelet2 Tag Mapping

This document describes how OpenDRIVE tags are used within CARLA and how Lanelet2 tags are processed and mapped to OpenDRIVE format.

## Overview

CARLA UE5 uses a modular OpenDRIVE parser system located in [`LibCarla/source/carla/opendrive/`](https://github.com/carla-simulator/carla/tree/master/LibCarla/source/carla/opendrive) directory. The parser consists of specialized components that handle different aspects of the OpenDRIVE specification:

- **XML Parsing**: Uses the `pugixml` library for XML processing
- **Modular Architecture**: Separate parser classes for different OpenDRIVE elements
- **Map Building**: Constructs CARLA's internal road network representation from parsed data

This document focuses on which OpenDRIVE tags CARLA reads and how they are used internally, which is essential for understanding what needs to be generated when converting from Lanelet2 format.


## OpenDRIVE Tags in CARLA

This section provides a comprehensive reference of OpenDRIVE tags, showing how they are used in CARLA's parser and how Lanelet2 elements map to these tags during conversion.

**Table columns:**

- **Parser Module**: CARLA parser component (click to jump to detailed notes)
- **OpenDRIVE Tag/Attribute**: Tag name in OpenDRIVE format
- **CARLA Purpose**: How CARLA uses this tag
- **CARLA Code Location**: Link to source code
- **Lanelet2 Mapping**: How this tag is generated from Lanelet2 data
- **Conversion Notes**: Important considerations for conversion

| Parser Module | OpenDRIVE Tag/Attribute | CARLA Purpose | CARLA Code Location | Lanelet2 Mapping | Conversion Notes |
|---------------|-------------------------|---------------|---------------------|------------------|------------------|
| [GeoReferenceParser](#georeferenceparser) | `header/geoReference` | PROJ format georeference string, Geographic coordinate system definition | [`GeoReferenceParser.cpp` L62](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/GeoReferenceParser.cpp#L62) | Lanelet2 origin (MGRS or lat/lon) | Convert MGRS to lat/lon, generate PROJ string |
| [GeoReferenceParser](#georeferenceparser) | `+lat_0=` | Latitude origin, Origin point for coordinate transformation | [`GeoReferenceParser.cpp` L28-L50](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/GeoReferenceParser.cpp#L28-L50) | Lanelet2 origin latitude | Direct extraction from origin |
| [GeoReferenceParser](#georeferenceparser) | `+lon_0=` | Longitude origin, Origin point for coordinate transformation | [`GeoReferenceParser.cpp` L28-L50](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/GeoReferenceParser.cpp#L28-L50) | Lanelet2 origin longitude | Direct extraction from origin |
| [RoadParser](#roadparser) | `road@id` | Road identifier, Unique road identification | [`RoadParser.cpp` L113-L120](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L113-L120) | Auto-generated | Group adjacent lanelets into roads |
| [RoadParser](#roadparser) | `road@name` | Road name, Road naming/labeling | [`RoadParser.cpp` L113-L120](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L113-L120) | Optional | Can use lanelet IDs or custom names |
| [RoadParser](#roadparser) | `road@length` | Road length (meters), Road geometry calculation | [`RoadParser.cpp` L113-L120](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L113-L120) | Calculated from L2 centerline | Sum of geometry segment lengths |
| [RoadParser](#roadparser) | `road@junction` | Junction ID reference, Links road to junction | [`RoadParser.cpp` L113-L120](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L113-L120) | L2 `turn_direction` tag | Lanelets with turn_direction → junction roads |
| [RoadParser](#roadparser) | `road/link/predecessor@elementId` | Previous road ID, Road connectivity | [`RoadParser.cpp` L122-L130](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L122-L130) | L2 predecessor connectivity | Map lanelet to road connectivity |
| [RoadParser](#roadparser) | `road/link/successor@elementId` | Next road ID, Road connectivity | [`RoadParser.cpp` L122-L130](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L122-L130) | L2 successor connectivity | Map lanelet to road connectivity |
| [RoadParser](#roadparser) | `road/type@s` | Start position, Road type section position | [`RoadParser.cpp` L133-L145](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L133-L145) | Generated | Segment start positions |
| [RoadParser](#roadparser) | `road/type@type` | Road type, Road classification (town, highway, etc.) | [`RoadParser.cpp` L133-L145](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L133-L145) | L2 `location` tag | urban→TOWN, highway→MOTORWAY, rural→RURAL |
| [RoadParser](#roadparser) | `road/type/speed@max` | Maximum speed, Speed limit | [`RoadParser.cpp` L133-L145](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L133-L145) | L2 `speed_limit` tag | Direct value (km/h) |
| [RoadParser](#roadparser) | `road/type/speed@unit` | Speed unit, Speed unit (km/h, mph, etc.) | [`RoadParser.cpp` L133-L145](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L133-L145) | Default km/h | Lanelet2 uses km/h |
| [RoadParser](#roadparser) | `road/lanes/laneOffset@s,a,b,c,d` | Lane offset parameters, Lane lateral offset polynomial | [`RoadParser.cpp` L147-L155](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L147-L155) | Default 0 | L2 has no lane offset concept |
| [RoadParser](#roadparser) | `road/lanes/laneSection@s` | Lane section start, Lane section definition | [`RoadParser.cpp` L157-L180](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L157-L180) | Generated | Section start positions from lanelet groups |
| [RoadParser](#roadparser) | `lane@id` | Lane identifier, Lane identification | [`RoadParser.cpp` L157-L180](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L157-L180) | Generated from ordering | Sequential lane numbering |
| [RoadParser](#roadparser) | `lane@type` | Lane type, Lane classification | [`RoadParser.cpp` L157-L180](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L157-L180) | L2 `subtype` tag | road→driving, walkway→sidewalk, bicycle_lane→biking |
| [RoadParser](#roadparser) | `lane@level` | Lane level, Stacked lane handling | [`RoadParser.cpp` L157-L180](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L157-L180) | Default 0 | L2 has no stacked lanes |
| [RoadParser](#roadparser) | `lane/link/predecessor` | Previous lane, Lane connectivity | [`RoadParser.cpp` L157-L180](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L157-L180) | L2 predecessor lanelet | Lane connectivity mapping |
| [RoadParser](#roadparser) | `lane/link/successor` | Next lane, Lane connectivity | [`RoadParser.cpp` L157-L180](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L157-L180) | L2 successor lanelet | Lane connectivity mapping |
| [GeometryParser](#geometryparser) | `geometry@s` | Start position along road, Geometry segment positioning | [`GeometryParser.cpp` L77-L84](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/GeometryParser.cpp#L77-L84) | L2 centerline segments | Cumulative distance along reference line |
| [GeometryParser](#geometryparser) | `geometry@x` | X coordinate, Geometry start point | [`GeometryParser.cpp` L77-L84](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/GeometryParser.cpp#L77-L84) | L2 centerline points | Extract x-coordinate from points |
| [GeometryParser](#geometryparser) | `geometry@y` | Y coordinate, Geometry start point | [`GeometryParser.cpp` L77-L84](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/GeometryParser.cpp#L77-L84) | L2 centerline points | Extract y-coordinate from points |
| [GeometryParser](#geometryparser) | `geometry@hdg` | Heading angle, Geometry orientation | [`GeometryParser.cpp` L77-L84](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/GeometryParser.cpp#L77-L84) | L2 centerline direction | Calculate heading from consecutive points |
| [GeometryParser](#geometryparser) | `geometry@length` | Geometry length, Segment length | [`GeometryParser.cpp` L77-L84](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/GeometryParser.cpp#L77-L84) | L2 centerline segments | Distance between consecutive points |
| [GeometryParser](#geometryparser) | `geometry/line` | Straight line, Linear road segment | [`GeometryParser.cpp` L89, L120](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/GeometryParser.cpp#L89) | L2 centerline (simple) | Direct point-to-point conversion |
| [GeometryParser](#geometryparser) | `geometry/arc@curvature` | Arc curvature, Curved road segment | [`GeometryParser.cpp` L88-91, L122-123](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/GeometryParser.cpp#L88-L91) | Optional | Requires curve fitting |
| [GeometryParser](#geometryparser) | `geometry/spiral@curvStart` | Spiral start curvature, Clothoid transition | [`GeometryParser.cpp` L91-94, L124-131](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/GeometryParser.cpp#L91-L94) | Optional | Requires clothoid fitting |
| [GeometryParser](#geometryparser) | `geometry/spiral@curvEnd` | Spiral end curvature, Clothoid transition | [`GeometryParser.cpp` L91-94, L124-131](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/GeometryParser.cpp#L91-L94) | Optional | Requires clothoid fitting |
| [GeometryParser](#geometryparser) | `geometry/poly3@a,b,c,d` | Cubic polynomial coefficients, Parametric road shape | [`GeometryParser.cpp` L94-99, L133-144](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/GeometryParser.cpp#L94-L99) | Optional | Requires polynomial fitting |
| [GeometryParser](#geometryparser) | `geometry/paramPoly3@aU,bU,cU,dU` | U-direction polynomial, Parametric curve U | [`GeometryParser.cpp` L99-110, L144-159](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/GeometryParser.cpp#L99-L110) | L2 centerline (advanced) | B-spline fitting for smoother roads |
| [GeometryParser](#geometryparser) | `geometry/paramPoly3@aV,bV,cV,dV` | V-direction polynomial, Parametric curve V | [`GeometryParser.cpp` L99-110, L144-159](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/GeometryParser.cpp#L99-L110) | L2 centerline (advanced) | B-spline fitting for smoother roads |
| [GeometryParser](#geometryparser) | `geometry/paramPoly3@pRange` | Parameter range, Parametric curve range | [`GeometryParser.cpp` L99-110, L144-159](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/GeometryParser.cpp#L99-L110) | Generated | arcLength for parameter range |
| [LaneParser](#laneparser) | `lane/width@sOffset,a,b,c,d` | Lane width polynomial, Lane width calculation | [`LaneParser.cpp` L17-L185](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/LaneParser.cpp#L17-L185) | Calculated from L2 boundaries | Distance between left/right bounds (not preferred) |
| [LaneParser](#laneparser) | `lane/border@sOffset,a,b,c,d` | Lane border polynomial, Lane boundary definition | [`LaneParser.cpp` L17-L185](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/LaneParser.cpp#L17-L185) | L2 left/right boundaries | Polynomial fit of boundary points (preferred) |
| [LaneParser](#laneparser) | `lane/roadMark@*` | Road marking attributes, Lane markings/striping | [`LaneParser.cpp` L17-L185](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/LaneParser.cpp#L17-L185) | Enhancement required | L2 has no road marking data |
| [LaneParser](#laneparser) | `lane/roadMark/type/line@*` | Line marking details, Detailed marking geometry | [`LaneParser.cpp` L17-L185](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/LaneParser.cpp#L17-L185) | Enhancement required | L2 has no detailed marking geometry |
| [LaneParser](#laneparser) | `lane/material@*` | Lane surface material, Surface properties | [`LaneParser.cpp` L17-L185](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/LaneParser.cpp#L17-L185) | Enhancement required | L2 has no material data |
| [LaneParser](#laneparser) | `lane/speed@sOffset` | Speed limit start position, Lane-specific speed limit | [`LaneParser.cpp` L17-L185](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/LaneParser.cpp#L17-L185) | Generated | Section start positions |
| [LaneParser](#laneparser) | `lane/speed@max` | Maximum speed, Lane speed restriction | [`LaneParser.cpp` L17-L185](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/LaneParser.cpp#L17-L185) | L2 `speed_limit` tag | Lane-specific speed (overrides road) |
| [LaneParser](#laneparser) | `lane/speed@unit` | Speed unit, Speed unit specification | [`LaneParser.cpp` L17-L185](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/LaneParser.cpp#L17-L185) | Default km/h | Lanelet2 uses km/h |
| [LaneParser](#laneparser) | `lane/access@sOffset` | Access rule start position, Lane access restrictions | [`LaneParser.cpp` L17-L185](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/LaneParser.cpp#L17-L185) | Enhancement required | L2 has no access restriction data |
| [LaneParser](#laneparser) | `lane/access@rule` | Access rule type, Access control | [`LaneParser.cpp` L17-L185](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/LaneParser.cpp#L17-L185) | Enhancement required | L2 has no access rules |
| [LaneParser](#laneparser) | `lane/access@restriction` | Restriction details, Detailed access rules | [`LaneParser.cpp` L17-L185](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/LaneParser.cpp#L17-L185) | Enhancement required | L2 has no detailed restrictions |
| [LaneParser](#laneparser) | `lane/height@sOffset` | Height change position, Vertical lane offset | [`LaneParser.cpp` L17-L185](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/LaneParser.cpp#L17-L185) | Enhancement required | L2 has no height offset data |
| [LaneParser](#laneparser) | `lane/height@inner` | Inner edge height, Height at inner boundary | [`LaneParser.cpp` L17-L185](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/LaneParser.cpp#L17-L185) | Enhancement required | L2 has no height data |
| [LaneParser](#laneparser) | `lane/height@outer` | Outer edge height, Height at outer boundary | [`LaneParser.cpp` L17-L185](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/LaneParser.cpp#L17-L185) | Enhancement required | L2 has no height data |
| [LaneParser](#laneparser) | `lane/rule@sOffset,value` | Lane rule position/value, Lane-specific rules | [`LaneParser.cpp` L17-L185](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/LaneParser.cpp#L17-L185) | Enhancement required | L2 has no lane rules |
| [LaneParser](#laneparser) | `lane/visibility@*` | Visibility attributes, Lane visibility properties | [`LaneParser.cpp` L17-L185](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/LaneParser.cpp#L17-L185) | Enhancement required | L2 has no visibility data |
| [ProfilesParser](#profilesparser) | `elevationProfile/elevation@s` | Elevation start position, Vertical profile positioning | [`ProfilesParser.cpp` L46-L80](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/ProfilesParser.cpp#L46-L80) | Generated | Segment start positions |
| [ProfilesParser](#profilesparser) | `elevationProfile/elevation@a,b,c,d` | Elevation polynomial, Vertical road shape | [`ProfilesParser.cpp` L46-L80](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/ProfilesParser.cpp#L46-L80) | L2 z-coordinates | Polynomial fit of elevation points |
| [ProfilesParser](#profilesparser) | `lateralProfile/superelevation@s` | Superelevation start, Banking start position | [`ProfilesParser.cpp` L46-L80](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/ProfilesParser.cpp#L46-L80) | Enhancement required | L2 has no superelevation data |
| [ProfilesParser](#profilesparser) | `lateralProfile/superelevation@a,b,c,d` | Superelevation polynomial, Road banking/tilt | [`ProfilesParser.cpp` L46-L80](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/ProfilesParser.cpp#L46-L80) | Enhancement required | L2 has no banking data |
| [ProfilesParser](#profilesparser) | `lateralProfile/shape@s` | Shape change start, Lateral shape positioning | [`ProfilesParser.cpp` L46-L80](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/ProfilesParser.cpp#L46-L80) | Enhancement required | L2 has no lateral shape data |
| [ProfilesParser](#profilesparser) | `lateralProfile/shape@t` | Lateral offset, Transverse position | [`ProfilesParser.cpp` L46-L80](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/ProfilesParser.cpp#L46-L80) | Enhancement required | L2 has no lateral offset |
| [ProfilesParser](#profilesparser) | `lateralProfile/shape@a,b,c,d` | Shape polynomial, Lateral profile shape | [`ProfilesParser.cpp` L46-L80](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/ProfilesParser.cpp#L46-L80) | Enhancement required | L2 has no shape profile |
| [JunctionParser](#junctionparser) | `junction@id` | Junction identifier, Unique junction identification | [`JunctionParser.cpp` L17-L50](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/JunctionParser.cpp#L17-L50) | Generated | From lanelets with turn_direction tag |
| [JunctionParser](#junctionparser) | `junction@name` | Junction name, Junction naming/labeling | [`JunctionParser.cpp` L17-L50](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/JunctionParser.cpp#L17-L50) | Optional | Can use custom names |
| [JunctionParser](#junctionparser) | `junction/connection@id` | Connection identifier, Connection identification | [`JunctionParser.cpp` L17-L50](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/JunctionParser.cpp#L17-L50) | Generated | Sequential connection numbering |
| [JunctionParser](#junctionparser) | `junction/connection@incomingRoad` | Incoming road ID, Junction entry point | [`JunctionParser.cpp` L17-L50](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/JunctionParser.cpp#L17-L50) | L2 junction predecessor | Map incoming lanelet group to road |
| [JunctionParser](#junctionparser) | `junction/connection@connectingRoad` | Connecting road ID, Junction internal path | [`JunctionParser.cpp` L17-L50](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/JunctionParser.cpp#L17-L50) | L2 junction lanelet | Map junction lanelet to connecting road |
| [JunctionParser](#junctionparser) | `junction/connection@contactPoint` | Connection point type, Start/end specification | [`JunctionParser.cpp` L17-L50](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/JunctionParser.cpp#L17-L50) | Generated | Determined from junction geometry |
| [JunctionParser](#junctionparser) | `junction/connection/laneLink@from` | Source lane ID, Lane-level connection source | [`JunctionParser.cpp` L17-L50](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/JunctionParser.cpp#L17-L50) | L2 source lane | Lane connectivity through junction |
| [JunctionParser](#junctionparser) | `junction/connection/laneLink@to` | Target lane ID, Lane-level connection target | [`JunctionParser.cpp` L17-L50](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/JunctionParser.cpp#L17-L50) | L2 target lane | Lane connectivity through junction |
| [JunctionParser](#junctionparser) | `junction/controller@id` | Controller ID, Traffic signal reference | [`JunctionParser.cpp` L17-L50](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/JunctionParser.cpp#L17-L50) | Enhancement required | L2 has controller concept but needs mapping |
| [JunctionParser](#junctionparser) | `junction/controller@type` | Controller type, Controller classification | [`JunctionParser.cpp` L17-L50](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/JunctionParser.cpp#L17-L50) | Enhancement required | Controller type classification |

### Parser-Specific Notes

#### GeoReferenceParser

[`GeoReferenceParser.cpp`](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/GeoReferenceParser.cpp)

- CARLA only reads latitude/longitude origins; UTM/MGRS zones are not directly supported
- Geographic reference is optional in OpenDRIVE but recommended for proper geo-location

#### RoadParser

[`RoadParser.cpp`](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp)

##### Supported Lane Types

([`RoadParser.cpp` L66-L110](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L66-L110))

- `driving`, `bidirectional`, `stop`, `shoulder`, `biking`, `sidewalk`
- `parking`, `border`, `restricted`, `median`, `entry`, `exit`
- `onRamp`, `offRamp`, `rail`, `tram`, `roadWorks`
- `special1`, `special2`, `special3`, `none`

##### Speed Limit Hierarchy

- Lane speed limits override road speed limits (OpenDRIVE spec 11.7)
- Signal speed limits have highest priority
- See Conversion Challenges for road grouping from Lanelet2

#### GeometryParser

[`GeometryParser.cpp`](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/GeometryParser.cpp)

##### Supported Geometry Types

- `<line>`: Straight segments (always supported)
- `<arc>`: Circular arcs
- `<spiral>`: Clothoid curves
- `<poly3>`, `<paramPoly3>`: Polynomial curves

See Conversion Challenges for Lanelet2 centerline conversion approaches

#### LaneParser

[`LaneParser.cpp`](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/LaneParser.cpp)

##### Width vs Border Specification

(OpenDRIVE Spec 1.8c)

- `<width>` and `<border>` are mutually exclusive within the same lane group
- If both exist, applications **must use** `<width>` elements per spec
- See CARLA-Specific Considerations for Lanelet2 conversion recommendations

##### Additional Attributes

- Road markings, material, speed limits, access rules parsed but usage in CARLA varies
- Height and visibility attributes have limited CARLA support

#### ProfilesParser

[`ProfilesParser.cpp`](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/ProfilesParser.cpp)

##### Parsing Details

- Reads elevation, superelevation, and lateral shape profiles
- Polynomial coefficients (a, b, c, d) define profile shape
- See CARLA-Specific Considerations for elevation profile requirements

#### JunctionParser

[`JunctionParser.cpp`](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/JunctionParser.cpp)

##### Related MapBuilder Functions

- `AddJunction`: [`MapBuilder.cpp` L566](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/road/MapBuilder.cpp#L566)
- `AddConnection`: [`MapBuilder.cpp` L570](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/road/MapBuilder.cpp#L570)
- `AddLaneLink`: [`MapBuilder.cpp` L580](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/road/MapBuilder.cpp#L580)

##### Junction Semantics

- **laneLink**: Defines lane-level connectivity within junction
- **connection**: Defines road-level connectivity (which roads connect to junction)
- **Road Assignment**: Roads must specify which junction they belong to via `road@junction` attribute

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
