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
- **Status**: Implementation status (see legend below)
- **Conversion Notes**: Important considerations for conversion
- **CARLA NPC Behavior Impact**: How this tag affects NPC vehicle/pedestrian behavior in simulation

**Legend:**

- ✅ Fully implemented (direct mapping from Lanelet2)
- ⚠️ Partially implemented or requires generation/calculation
- ❌ Not implemented (no Lanelet2 equivalent, enhancement required)

| Parser Module | OpenDRIVE Tag/Attribute | CARLA Purpose | CARLA Code Location | Lanelet2 Mapping | Status | Conversion Notes | CARLA NPC Behavior Impact |
|---------------|-------------------------|---------------|---------------------|------------------|--------|------------------|------------------------|
| [GeoReferenceParser](#georeferenceparser) | `header/geoReference` | PROJ format georeference string, Geographic coordinate system definition | [`GeoReferenceParser.cpp` L62](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/GeoReferenceParser.cpp#L62) | Lanelet2 origin (MGRS or lat/lon) | ✅ | Convert MGRS to lat/lon, generate PROJ string | - |
| [GeoReferenceParser](#georeferenceparser) | `+lat_0=` | Latitude origin, Origin point for coordinate transformation | [`GeoReferenceParser.cpp` L28-L50](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/GeoReferenceParser.cpp#L28-L50) | Lanelet2 origin latitude | ✅ | Direct extraction from origin | - |
| [GeoReferenceParser](#georeferenceparser) | `+lon_0=` | Longitude origin, Origin point for coordinate transformation | [`GeoReferenceParser.cpp` L28-L50](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/GeoReferenceParser.cpp#L28-L50) | Lanelet2 origin longitude | ✅ | Direct extraction from origin | - |
| [RoadParser](#roadparser) | `road@id` | Road identifier, Unique road identification | [`RoadParser.cpp` L113-L120](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L113-L120) | Auto-generated | ⚠️ | Group adjacent lanelets into roads | - |
| [RoadParser](#roadparser) | `road@name` | Road name, Road naming/labeling | [`RoadParser.cpp` L113-L120](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L113-L120) | Optional | ⚠️ | Can use lanelet IDs or custom names | - |
| [RoadParser](#roadparser) | `road@length` | Road length (meters), Road geometry calculation | [`RoadParser.cpp` L113-L120](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L113-L120) | Calculated from Lanelet2 centerline | ⚠️ | Sum of geometry segment lengths | - |
| [RoadParser](#roadparser) | `road@junction` | Junction ID reference, Links road to junction | [`RoadParser.cpp` L113-L120](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L113-L120) | Lanelet2 `turn_direction` tag | ✅ | Lanelets with turn_direction → junction roads | NPC uses junction navigation logic for turns |
| [RoadParser](#roadparser) | `road/link/predecessor@elementId` | Previous road ID, Road connectivity | [`RoadParser.cpp` L122-L130](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L122-L130) | Lanelet2 predecessor connectivity | ✅ | Map lanelet to road connectivity | Affects route planning and road connectivity |
| [RoadParser](#roadparser) | `road/link/successor@elementId` | Next road ID, Road connectivity | [`RoadParser.cpp` L122-L130](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L122-L130) | Lanelet2 successor connectivity | ✅ | Map lanelet to road connectivity | Affects route planning and road connectivity |
| [RoadParser](#roadparser) | `road/type@s` | Start position, Road type section position | [`RoadParser.cpp` L133-L145](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L133-L145) | Generated | ⚠️ | Segment start positions | - |
| [RoadParser](#roadparser) | `road/type@type` | Road type, Road classification (town, highway, etc.) | [`RoadParser.cpp` L133-L145](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L133-L145) | Lanelet2 `location` tag | ✅ | urban→TOWN, highway→MOTORWAY, rural→RURAL | Highway: higher cruise speed; Town: slower, more cautious |
| [RoadParser](#roadparser) | `road/type/speed@max` | Maximum speed, Speed limit | [`RoadParser.cpp` L133-L145](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L133-L145) | Lanelet2 `speed_limit` tag | ✅ | Direct value (km/h) | Sets NPC cruise speed limit |
| [RoadParser](#roadparser) | `road/type/speed@unit` | Speed unit, Speed unit (km/h, mph, etc.) | [`RoadParser.cpp` L133-L145](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L133-L145) | Default km/h | ⚠️ | Lanelet2 uses km/h | - |
| [RoadParser](#roadparser) | `road/lanes/laneOffset@s,a,b,c,d` | Lane offset parameters, Lane lateral offset polynomial | [`RoadParser.cpp` L147-L155](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L147-L155) | Default 0 | ⚠️ | Lanelet2 has no lane offset concept | - |
| [RoadParser](#roadparser) | `road/lanes/laneSection@s` | Lane section start, Lane section definition | [`RoadParser.cpp` L157-L180](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L157-L180) | Generated | ⚠️ | Section start positions from lanelet groups | - |
| [RoadParser](#roadparser) | `lane@id` | Lane identifier, Lane identification | [`RoadParser.cpp` L157-L180](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L157-L180) | Generated from ordering | ⚠️ | Sequential lane numbering | - |
| [RoadParser](#roadparser) | `lane@type` | Lane type, Lane classification | [`RoadParser.cpp` L157-L180](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L157-L180) | Lanelet2 `subtype` tag | ✅ | road→driving, walkway→sidewalk, bicycle_lane→biking | NPCs only drive in 'driving' lanes; ignore sidewalk/biking |
| [RoadParser](#roadparser) | `lane@level` | Lane level, Stacked lane handling | [`RoadParser.cpp` L157-L180](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L157-L180) | Default 0 | ⚠️ | Lanelet2 has no stacked lanes | - |
| [RoadParser](#roadparser) | `lane/link/predecessor` | Previous lane, Lane connectivity | [`RoadParser.cpp` L157-L180](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L157-L180) | Lanelet2 predecessor lanelet | ✅ | Lane connectivity mapping | Used for lane-level route planning |
| [RoadParser](#roadparser) | `lane/link/successor` | Next lane, Lane connectivity | [`RoadParser.cpp` L157-L180](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/RoadParser.cpp#L157-L180) | Lanelet2 successor lanelet | ✅ | Lane connectivity mapping | Used for lane-level route planning |
| [GeometryParser](#geometryparser) | `geometry@s` | Start position along road, Geometry segment positioning | [`GeometryParser.cpp` L77-L84](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/GeometryParser.cpp#L77-L84) | Lanelet2 centerline segments | ✅ | Cumulative distance along reference line | - |
| [GeometryParser](#geometryparser) | `geometry@x` | X coordinate, Geometry start point | [`GeometryParser.cpp` L77-L84](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/GeometryParser.cpp#L77-L84) | Lanelet2 centerline points | ✅ | Extract x-coordinate from points | - |
| [GeometryParser](#geometryparser) | `geometry@y` | Y coordinate, Geometry start point | [`GeometryParser.cpp` L77-L84](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/GeometryParser.cpp#L77-L84) | Lanelet2 centerline points | ✅ | Extract y-coordinate from points | - |
| [GeometryParser](#geometryparser) | `geometry@hdg` | Heading angle, Geometry orientation | [`GeometryParser.cpp` L77-L84](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/GeometryParser.cpp#L77-L84) | Lanelet2 centerline direction | ✅ | Calculate heading from consecutive points | - |
| [GeometryParser](#geometryparser) | `geometry@length` | Geometry length, Segment length | [`GeometryParser.cpp` L77-L84](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/GeometryParser.cpp#L77-L84) | Lanelet2 centerline segments | ✅ | Distance between consecutive points | - |
| [GeometryParser](#geometryparser) | `geometry/line` | Straight line, Linear road segment | [`GeometryParser.cpp` L89, L120](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/GeometryParser.cpp#L89) | Lanelet2 centerline (simple) | ✅ | Direct point-to-point conversion | - |
| [GeometryParser](#geometryparser) | `geometry/arc@curvature` | Arc curvature, Curved road segment | [`GeometryParser.cpp` L88-91, L122-123](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/GeometryParser.cpp#L88-L91) | Optional | ⚠️ | Requires curve fitting | Affects steering behavior on curves |
| [GeometryParser](#geometryparser) | `geometry/spiral@curvStart` | Spiral start curvature, Clothoid transition | [`GeometryParser.cpp` L91-94, L124-131](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/GeometryParser.cpp#L91-L94) | Optional | ⚠️ | Requires clothoid fitting | Affects steering behavior on transitions |
| [GeometryParser](#geometryparser) | `geometry/spiral@curvEnd` | Spiral end curvature, Clothoid transition | [`GeometryParser.cpp` L91-94, L124-131](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/GeometryParser.cpp#L91-L94) | Optional | ⚠️ | Requires clothoid fitting | Affects steering behavior on transitions |
| [GeometryParser](#geometryparser) | `geometry/poly3@a,b,c,d` | Cubic polynomial coefficients, Parametric road shape | [`GeometryParser.cpp` L94-99, L133-144](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/GeometryParser.cpp#L94-L99) | Optional | ⚠️ | Requires polynomial fitting | - |
| [GeometryParser](#geometryparser) | `geometry/paramPoly3@aU,bU,cU,dU` | U-direction polynomial, Parametric curve U | [`GeometryParser.cpp` L99-110, L144-159](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/GeometryParser.cpp#L99-L110) | Lanelet2 centerline (advanced) | ✅ | B-spline fitting for smoother roads | - |
| [GeometryParser](#geometryparser) | `geometry/paramPoly3@aV,bV,cV,dV` | V-direction polynomial, Parametric curve V | [`GeometryParser.cpp` L99-110, L144-159](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/GeometryParser.cpp#L99-L110) | Lanelet2 centerline (advanced) | ✅ | B-spline fitting for smoother roads | - |
| [GeometryParser](#geometryparser) | `geometry/paramPoly3@pRange` | Parameter range, Parametric curve range | [`GeometryParser.cpp` L99-110, L144-159](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/GeometryParser.cpp#L99-L110) | Generated | ⚠️ | arcLength for parameter range | - |
| [LaneParser](#laneparser) | `lane/width@sOffset,a,b,c,d` | Lane width polynomial, Lane width calculation | [`LaneParser.cpp` L17-L185](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/LaneParser.cpp#L17-L185) | Calculated from Lanelet2 boundaries | ⚠️ | Distance between left/right bounds (not preferred) | Narrow lanes: NPCs drive more cautiously |
| [LaneParser](#laneparser) | `lane/border@sOffset,a,b,c,d` | Lane border polynomial, Lane boundary definition | [`LaneParser.cpp` L17-L185](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/LaneParser.cpp#L17-L185) | Lanelet2 left/right boundaries | ✅ | Polynomial fit of boundary points (preferred) | Defines drivable area boundaries |
| [LaneParser](#laneparser) | `lane/roadMark@*` | Road marking attributes, Lane markings/striping | [`LaneParser.cpp` L17-L185](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/LaneParser.cpp#L17-L185) | Enhancement required | ❌ | Lanelet2 has no road marking data | - |
| [LaneParser](#laneparser) | `lane/roadMark/type/line@*` | Line marking details, Detailed marking geometry | [`LaneParser.cpp` L17-L185](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/LaneParser.cpp#L17-L185) | Enhancement required | ❌ | Lanelet2 has no detailed marking geometry | - |
| [LaneParser](#laneparser) | `lane/material@*` | Lane surface material, Surface properties | [`LaneParser.cpp` L17-L185](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/LaneParser.cpp#L17-L185) | Enhancement required | ❌ | Lanelet2 has no material data | - |
| [LaneParser](#laneparser) | `lane/speed@sOffset` | Speed limit start position, Lane-specific speed limit | [`LaneParser.cpp` L17-L185](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/LaneParser.cpp#L17-L185) | Generated | ⚠️ | Section start positions | - |
| [LaneParser](#laneparser) | `lane/speed@max` | Maximum speed, Lane speed restriction | [`LaneParser.cpp` L17-L185](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/LaneParser.cpp#L17-L185) | Lanelet2 `speed_limit` tag | ✅ | Lane-specific speed (overrides road) | Overrides road speed; sets NPC cruise speed for this lane |
| [LaneParser](#laneparser) | `lane/speed@unit` | Speed unit, Speed unit specification | [`LaneParser.cpp` L17-L185](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/LaneParser.cpp#L17-L185) | Default km/h | ⚠️ | Lanelet2 uses km/h | - |
| [LaneParser](#laneparser) | `lane/access@sOffset` | Access rule start position, Lane access restrictions | [`LaneParser.cpp` L17-L185](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/LaneParser.cpp#L17-L185) | Enhancement required | ❌ | Lanelet2 has no access restriction data | - |
| [LaneParser](#laneparser) | `lane/access@rule` | Access rule type, Access control | [`LaneParser.cpp` L17-L185](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/LaneParser.cpp#L17-L185) | Enhancement required | ❌ | Lanelet2 has no access rules | Could restrict NPC vehicle types (not fully implemented) |
| [LaneParser](#laneparser) | `lane/access@restriction` | Restriction details, Detailed access rules | [`LaneParser.cpp` L17-L185](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/LaneParser.cpp#L17-L185) | Enhancement required | ❌ | Lanelet2 has no detailed restrictions | - |
| [LaneParser](#laneparser) | `lane/height@sOffset` | Height change position, Vertical lane offset | [`LaneParser.cpp` L17-L185](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/LaneParser.cpp#L17-L185) | Enhancement required | ❌ | Lanelet2 has no height offset data | - |
| [LaneParser](#laneparser) | `lane/height@inner` | Inner edge height, Height at inner boundary | [`LaneParser.cpp` L17-L185](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/LaneParser.cpp#L17-L185) | Enhancement required | ❌ | Lanelet2 has no height data | - |
| [LaneParser](#laneparser) | `lane/height@outer` | Outer edge height, Height at outer boundary | [`LaneParser.cpp` L17-L185](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/LaneParser.cpp#L17-L185) | Enhancement required | ❌ | Lanelet2 has no height data | - |
| [LaneParser](#laneparser) | `lane/rule@sOffset,value` | Lane rule position/value, Lane-specific rules | [`LaneParser.cpp` L17-L185](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/LaneParser.cpp#L17-L185) | Enhancement required | ❌ | Lanelet2 has no lane rules | - |
| [LaneParser](#laneparser) | `lane/visibility@*` | Visibility attributes, Lane visibility properties | [`LaneParser.cpp` L17-L185](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/LaneParser.cpp#L17-L185) | Enhancement required | ❌ | Lanelet2 has no visibility data | - |
| [ProfilesParser](#profilesparser) | `elevationProfile/elevation@s` | Elevation start position, Vertical profile positioning | [`ProfilesParser.cpp` L46-L80](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/ProfilesParser.cpp#L46-L80) | Generated | ⚠️ | Segment start positions | - |
| [ProfilesParser](#profilesparser) | `elevationProfile/elevation@a,b,c,d` | Elevation polynomial, Vertical road shape | [`ProfilesParser.cpp` L46-L80](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/ProfilesParser.cpp#L46-L80) | Lanelet2 z-coordinates | ✅ | Polynomial fit of elevation points | Steep grades: NPCs adjust speed accordingly |
| [ProfilesParser](#profilesparser) | `lateralProfile/superelevation@s` | Superelevation start, Banking start position | [`ProfilesParser.cpp` L46-L80](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/ProfilesParser.cpp#L46-L80) | Enhancement required | ❌ | Lanelet2 has no superelevation data | - |
| [ProfilesParser](#profilesparser) | `lateralProfile/superelevation@a,b,c,d` | Superelevation polynomial, Road banking/tilt | [`ProfilesParser.cpp` L46-L80](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/ProfilesParser.cpp#L46-L80) | Enhancement required | ❌ | Lanelet2 has no banking data | - |
| [ProfilesParser](#profilesparser) | `lateralProfile/shape@s` | Shape change start, Lateral shape positioning | [`ProfilesParser.cpp` L46-L80](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/ProfilesParser.cpp#L46-L80) | Enhancement required | ❌ | Lanelet2 has no lateral shape data | - |
| [ProfilesParser](#profilesparser) | `lateralProfile/shape@t` | Lateral offset, Transverse position | [`ProfilesParser.cpp` L46-L80](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/ProfilesParser.cpp#L46-L80) | Enhancement required | ❌ | Lanelet2 has no lateral offset | - |
| [ProfilesParser](#profilesparser) | `lateralProfile/shape@a,b,c,d` | Shape polynomial, Lateral profile shape | [`ProfilesParser.cpp` L46-L80](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/ProfilesParser.cpp#L46-L80) | Enhancement required | ❌ | Lanelet2 has no shape profile | - |
| [JunctionParser](#junctionparser) | `junction@id` | Junction identifier, Unique junction identification | [`JunctionParser.cpp` L17-L50](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/JunctionParser.cpp#L17-L50) | Generated | ⚠️ | From lanelets with turn_direction tag | - |
| [JunctionParser](#junctionparser) | `junction@name` | Junction name, Junction naming/labeling | [`JunctionParser.cpp` L17-L50](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/JunctionParser.cpp#L17-L50) | Optional | ⚠️ | Can use custom names | - |
| [JunctionParser](#junctionparser) | `junction/connection@id` | Connection identifier, Connection identification | [`JunctionParser.cpp` L17-L50](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/JunctionParser.cpp#L17-L50) | Generated | ⚠️ | Sequential connection numbering | - |
| [JunctionParser](#junctionparser) | `junction/connection@incomingRoad` | Incoming road ID, Junction entry point | [`JunctionParser.cpp` L17-L50](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/JunctionParser.cpp#L17-L50) | Lanelet2 junction predecessor | ✅ | Map incoming lanelet group to road | Defines NPC turning options at intersections |
| [JunctionParser](#junctionparser) | `junction/connection@connectingRoad` | Connecting road ID, Junction internal path | [`JunctionParser.cpp` L17-L50](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/JunctionParser.cpp#L17-L50) | Lanelet2 junction lanelet | ✅ | Map junction lanelet to connecting road | NPC follows connecting road through junction |
| [JunctionParser](#junctionparser) | `junction/connection@contactPoint` | Connection point type, Start/end specification | [`JunctionParser.cpp` L17-L50](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/JunctionParser.cpp#L17-L50) | Generated | ⚠️ | Determined from junction geometry | - |
| [JunctionParser](#junctionparser) | `junction/connection/laneLink@from` | Source lane ID, Lane-level connection source | [`JunctionParser.cpp` L17-L50](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/JunctionParser.cpp#L17-L50) | Lanelet2 source lane | ✅ | Lane connectivity through junction | Determines valid lane-to-lane turns for NPCs |
| [JunctionParser](#junctionparser) | `junction/connection/laneLink@to` | Target lane ID, Lane-level connection target | [`JunctionParser.cpp` L17-L50](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/JunctionParser.cpp#L17-L50) | Lanelet2 target lane | ✅ | Lane connectivity through junction | Determines valid lane-to-lane turns for NPCs |
| [SignalParser](#signalparser) | `road/signals/signal@s` | Signal S-coordinate position, Longitudinal position along road | [`SignalParser.cpp` L47](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/SignalParser.cpp#L47) | Lanelet2 regulatory_element position | ✅ | Calculate from regulatory_element refers | - |
| [SignalParser](#signalparser) | `road/signals/signal@t` | Signal T-coordinate position, Lateral offset from reference line | [`SignalParser.cpp` L48](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/SignalParser.cpp#L48) | Generated | ⚠️ | Default lateral offset for signal placement | - |
| [SignalParser](#signalparser) | `road/signals/signal@id` | Signal identifier, Unique signal identification | [`SignalParser.cpp` L49](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/SignalParser.cpp#L49) | Lanelet2 regulatory_element ID | ✅ | Use regulatory_element ID directly | - |
| [SignalParser](#signalparser) | `road/signals/signal@name` | Signal name, Signal naming/labeling | [`SignalParser.cpp` L50](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/SignalParser.cpp#L50) | Optional | ⚠️ | Can use regulatory_element subtype | - |
| [SignalParser](#signalparser) | `road/signals/signal@dynamic` | Dynamic signal flag, Indicates if signal state changes | [`SignalParser.cpp` L51](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/SignalParser.cpp#L51) | Lanelet2 traffic_light type | ✅ | yes for traffic_light, no for traffic_sign | Dynamic signals (traffic lights): NPCs obey state changes |
| [SignalParser](#signalparser) | `road/signals/signal@orientation` | Signal orientation, Signal facing direction (+/- for lane direction) | [`SignalParser.cpp` L52](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/SignalParser.cpp#L52) | Generated | ⚠️ | Determine from lane direction | - |
| [SignalParser](#signalparser) | `road/signals/signal@zOffset` | Signal Z-offset, Vertical position above road surface | [`SignalParser.cpp` L53](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/SignalParser.cpp#L53) | Default height | ⚠️ | Use standard signal height (e.g., 5m) | - |
| [SignalParser](#signalparser) | `road/signals/signal@country` | Country code, Signal standard specification (e.g., OpenDRIVE) | [`SignalParser.cpp` L54](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/SignalParser.cpp#L54) | Default OpenDRIVE | ⚠️ | Use OpenDRIVE standard codes | - |
| [SignalParser](#signalparser) | `road/signals/signal@type` | Signal type code, Signal classification (1000003=red/yellow/green) | [`SignalParser.cpp` L55](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/SignalParser.cpp#L55) | Lanelet2 subtype mapping | ✅ | Map Lanelet2 subtypes to OpenDRIVE codes | Determines signal meaning: traffic light, speed limit, stop, etc. |
| [SignalParser](#signalparser) | `road/signals/signal@subtype` | Signal subtype, Detailed signal classification | [`SignalParser.cpp` L56](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/SignalParser.cpp#L56) | Lanelet2 subtype details | ✅ | Detailed type from regulatory_element | - |
| [SignalParser](#signalparser) | `road/signals/signal@value` | Signal value, Numeric value (e.g., speed limit number) | [`SignalParser.cpp` L57](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/SignalParser.cpp#L57) | Lanelet2 speed_limit or other values | ✅ | Extract value from regulatory_element | Speed limit value: NPCs adjust speed to comply |
| [SignalParser](#signalparser) | `road/signals/signal@unit` | Signal unit, Unit of measurement for value | [`SignalParser.cpp` L58](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/SignalParser.cpp#L58) | Default unit | ⚠️ | km/h for speed, m for distance | - |
| [SignalParser](#signalparser) | `road/signals/signal@height` | Signal height, Physical signal height | [`SignalParser.cpp` L59](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/SignalParser.cpp#L59) | Default dimensions | ⚠️ | Standard signal dimensions | - |
| [SignalParser](#signalparser) | `road/signals/signal@width` | Signal width, Physical signal width | [`SignalParser.cpp` L60](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/SignalParser.cpp#L60) | Default dimensions | ⚠️ | Standard signal dimensions | - |
| [SignalParser](#signalparser) | `road/signals/signal@text` | Signal text, Displayed text on signal | [`SignalParser.cpp` L61](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/SignalParser.cpp#L61) | Optional | ⚠️ | Textual signal information if available | - |
| [SignalParser](#signalparser) | `road/signals/signal@hOffset` | Signal horizontal offset, Additional horizontal displacement | [`SignalParser.cpp` L62](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/SignalParser.cpp#L62) | Default 0 | ⚠️ | Typically 0 unless specific placement needed | - |
| [SignalParser](#signalparser) | `road/signals/signal@pitch` | Signal pitch angle, Vertical tilt angle | [`SignalParser.cpp` L63](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/SignalParser.cpp#L63) | Default 0 | ⚠️ | Default vertical orientation | - |
| [SignalParser](#signalparser) | `road/signals/signal@roll` | Signal roll angle, Horizontal tilt angle | [`SignalParser.cpp` L64](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/SignalParser.cpp#L64) | Default 0 | ⚠️ | Default roll orientation | - |
| [SignalParser](#signalparser) | `signal/validity@fromLane` | Validity start lane, Signal applies from this lane ID | [`SignalParser.cpp` L25](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/SignalParser.cpp#L25) | Lanelet2 refers lanelets | ✅ | Map referenced lanelets to lane IDs | Signals only affect NPCs in specified lane range |
| [SignalParser](#signalparser) | `signal/validity@toLane` | Validity end lane, Signal applies up to this lane ID | [`SignalParser.cpp` L26](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/SignalParser.cpp#L26) | Lanelet2 refers lanelets | ✅ | Map referenced lanelets to lane IDs | Signals only affect NPCs in specified lane range |
| [SignalParser](#signalparser) | `signal/dependency@id` | Dependency signal ID, References another signal | [`SignalParser.cpp` L110](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/SignalParser.cpp#L110) | Enhancement required | ❌ | Lanelet2 has no signal dependency concept | - |
| [SignalParser](#signalparser) | `signal/dependency@type` | Dependency type, Type of signal dependency | [`SignalParser.cpp` L111](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/SignalParser.cpp#L111) | Enhancement required | ❌ | Lanelet2 has no dependency types | - |
| [SignalParser](#signalparser) | `signal/positionInertial@x` | Inertial X position, Absolute X coordinate | [`SignalParser.cpp` L116](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/SignalParser.cpp#L116) | Calculate from road coordinates | ⚠️ | Transform from s,t to absolute X,Y | - |
| [SignalParser](#signalparser) | `signal/positionInertial@y` | Inertial Y position, Absolute Y coordinate | [`SignalParser.cpp` L117](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/SignalParser.cpp#L117) | Calculate from road coordinates | ⚠️ | Transform from s,t to absolute X,Y | - |
| [SignalParser](#signalparser) | `signal/positionInertial@z` | Inertial Z position, Absolute Z coordinate | [`SignalParser.cpp` L118](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/SignalParser.cpp#L118) | Calculate from elevation + zOffset | ⚠️ | Road elevation + signal zOffset | - |
| [SignalParser](#signalparser) | `signal/positionInertial@hdg` | Inertial heading, Absolute heading angle | [`SignalParser.cpp` L119](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/SignalParser.cpp#L119) | Calculate from road heading | ⚠️ | Road heading at signal position | - |
| [SignalParser](#signalparser) | `signal/positionInertial@pitch` | Inertial pitch, Absolute pitch angle | [`SignalParser.cpp` L120](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/SignalParser.cpp#L120) | Default 0 | ⚠️ | Combined with signal pitch | - |
| [SignalParser](#signalparser) | `signal/positionInertial@roll` | Inertial roll, Absolute roll angle | [`SignalParser.cpp` L121](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/SignalParser.cpp#L121) | Default 0 | ⚠️ | Combined with signal roll | - |
| [SignalParser](#signalparser) | `road/signals/signalReference@s` | Signal reference S position, Reference to signal at position | [`SignalParser.cpp` L129](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/SignalParser.cpp#L129) | Same as signal@s | ⚠️ | Cross-reference to signal definition | - |
| [SignalParser](#signalparser) | `road/signals/signalReference@t` | Signal reference T position, Lateral reference position | [`SignalParser.cpp` L130](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/SignalParser.cpp#L130) | Same as signal@t | ⚠️ | Cross-reference lateral offset | - |
| [SignalParser](#signalparser) | `road/signals/signalReference@id` | Signal reference ID, References signal by ID | [`SignalParser.cpp` L131](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/SignalParser.cpp#L131) | Lanelet2 regulatory_element ref | ✅ | Links road to signal definition | Links signal to road segment for NPC awareness |
| [SignalParser](#signalparser) | `road/signals/signalReference@orientation` | Signal reference orientation, Reference direction | [`SignalParser.cpp` L133](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/SignalParser.cpp#L133) | Generated | ⚠️ | Match signal orientation | - |
| [ControllerParser](#controllerparser) | `controller@id` | Controller identifier, Unique controller identification | [`ControllerParser.cpp` L27](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/ControllerParser.cpp#L27) | Generated from traffic_light groups | ⚠️ | Group related traffic_lights | - |
| [ControllerParser](#controllerparser) | `controller@name` | Controller name, Controller naming/labeling | [`ControllerParser.cpp` L28](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/ControllerParser.cpp#L28) | Optional | ⚠️ | Can use intersection name | - |
| [ControllerParser](#controllerparser) | `controller@sequence` | Controller sequence, Signal phase sequence number | [`ControllerParser.cpp` L29](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/ControllerParser.cpp#L29) | Enhancement required | ❌ | Lanelet2 has no phase sequence data | Signal phase sequence: affects NPC waiting time at lights |
| [ControllerParser](#controllerparser) | `controller/control@signalId` | Controlled signal ID, References signal under control | [`ControllerParser.cpp` L39](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/ControllerParser.cpp#L39) | Lanelet2 traffic_light IDs in group | ✅ | Map traffic_light refs to signal IDs | Groups signals for coordinated NPC traffic flow |
| [ControllerParser](#controllerparser) | `controller/control@type` | Control type, Type of controller (not yet used in CARLA) | [`ControllerParser.cpp` L41](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/ControllerParser.cpp#L41) | Optional | ⚠️ | Currently not used by CARLA | - |
| [ObjectParser](#objectparser) | `road/objects/object@type` | Object type classification, Type of road object (crosswalk, etc.) | [`ObjectParser.cpp` L34](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/ObjectParser.cpp#L34) | Lanelet2 area/polygon type | ✅ | crosswalk, speed sign, stop line | Crosswalk: NPCs yield to pedestrians; Stop: NPCs stop |
| [ObjectParser](#objectparser) | `road/objects/object@name` | Object name, Object identification name | [`ObjectParser.cpp` L35](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/ObjectParser.cpp#L35) | Lanelet2 area/polygon ID or name | ✅ | Speed_*, Stencil_STOP patterns | - |
| [ObjectParser](#objectparser) | `road/objects/object@id` | Object identifier, Unique object identification | [`ObjectParser.cpp` L78](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/ObjectParser.cpp#L78) | Lanelet2 area/polygon ID | ✅ | Direct ID mapping | - |
| [ObjectParser](#objectparser) | `road/objects/object@s` | Object S position, Longitudinal position along road | [`ObjectParser.cpp` L79](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/ObjectParser.cpp#L79) | Calculate from area center | ⚠️ | Project area center to road reference | - |
| [ObjectParser](#objectparser) | `road/objects/object@t` | Object T position, Lateral offset from reference line | [`ObjectParser.cpp` L80](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/ObjectParser.cpp#L80) | Calculate from area center | ⚠️ | Lateral offset calculation | - |
| [ObjectParser](#objectparser) | `road/objects/object@zOffset` | Object Z offset, Vertical position above road | [`ObjectParser.cpp` L84](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/ObjectParser.cpp#L84) | Default 0 | ⚠️ | Road surface level | - |
| [ObjectParser](#objectparser) | `road/objects/object@hdg` | Object heading, Orientation angle | [`ObjectParser.cpp` L93](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/ObjectParser.cpp#L93) | Calculate from area orientation | ⚠️ | Area principal axis direction | - |
| [ObjectParser](#objectparser) | `road/objects/object@pitch` | Object pitch, Vertical tilt angle | [`ObjectParser.cpp` L94](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/ObjectParser.cpp#L94) | Default 0 | ⚠️ | Flat orientation | - |
| [ObjectParser](#objectparser) | `road/objects/object@roll` | Object roll, Horizontal tilt angle | [`ObjectParser.cpp` L95](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/ObjectParser.cpp#L95) | Default 0 | ⚠️ | Flat orientation | - |
| [ObjectParser](#objectparser) | `road/objects/object@orientation` | Object orientation, Direction relative to road (+/-/none) | [`ObjectParser.cpp` L83](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/ObjectParser.cpp#L83) | Generated | ⚠️ | Determine from area position | - |
| [ObjectParser](#objectparser) | `road/objects/object@width` | Object width, Physical width dimension | [`ObjectParser.cpp` L91](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/ObjectParser.cpp#L91) | Calculate from area bounds | ⚠️ | Bounding box width | - |
| [ObjectParser](#objectparser) | `road/objects/object@length` | Object length, Physical length dimension | [`ObjectParser.cpp` L63](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/ObjectParser.cpp#L63) | Calculate from area bounds | ⚠️ | Bounding box length | - |
| [ObjectParser](#objectparser) | `road/objects/object@height` | Object height, Physical height dimension | [`ObjectParser.cpp` L90](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/ObjectParser.cpp#L90) | Default height | ⚠️ | Standard object height | - |
| [ObjectParser](#objectparser) | `object/outline/cornerLocal@u` | Corner U coordinate, Local U coordinate of outline point | [`ObjectParser.cpp` L43](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/ObjectParser.cpp#L43) | Lanelet2 area vertices | ✅ | Transform area vertices to local U,V | - |
| [ObjectParser](#objectparser) | `object/outline/cornerLocal@v` | Corner V coordinate, Local V coordinate of outline point | [`ObjectParser.cpp` L44](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/ObjectParser.cpp#L44) | Lanelet2 area vertices | ✅ | Transform area vertices to local U,V | - |
| [ObjectParser](#objectparser) | `object/outline/cornerLocal@z` | Corner Z coordinate, Local Z coordinate of outline point | [`ObjectParser.cpp` L45](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/ObjectParser.cpp#L45) | Lanelet2 area vertex Z | ⚠️ | Use Z from vertices if 3D | - |
| [TrafficGroupParser](#trafficgroupparser) | `userData/trafficGroup@id` | Traffic group identifier, Signal group identification (STUBBED) | [`TrafficGroupParser.cpp` L37](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/TrafficGroupParser.cpp#L37) | Enhancement required | ❌ | Feature currently stubbed in CARLA | - |
| [TrafficGroupParser](#trafficgroupparser) | `userData/trafficGroup@redTime` | Red signal duration, Red phase timing in seconds (STUBBED) | [`TrafficGroupParser.cpp` L38](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/TrafficGroupParser.cpp#L38) | Enhancement required | ❌ | Timing data not in Lanelet2 | Red duration: NPCs wait at red light (if implemented) |
| [TrafficGroupParser](#trafficgroupparser) | `userData/trafficGroup@yellowTime` | Yellow signal duration, Yellow phase timing in seconds (STUBBED) | [`TrafficGroupParser.cpp` L39](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/TrafficGroupParser.cpp#L39) | Enhancement required | ❌ | Timing data not in Lanelet2 | Yellow duration: NPCs prepare to stop (if implemented) |
| [TrafficGroupParser](#trafficgroupparser) | `userData/trafficGroup@greenTime` | Green signal duration, Green phase timing in seconds (STUBBED) | [`TrafficGroupParser.cpp` L40](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/TrafficGroupParser.cpp#L40) | Enhancement required | ❌ | Timing data not in Lanelet2 | Green duration: NPCs proceed through intersection |
| [JunctionParser](#junctionparser) | `junction/controller@id` | Controller ID, Traffic signal reference | [`JunctionParser.cpp` L17-L50](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/JunctionParser.cpp#L17-L50) | Enhancement required | ❌ | Lanelet2 has controller concept but needs mapping | Links junction to traffic signal controller |
| [JunctionParser](#junctionparser) | `junction/controller@type` | Controller type, Controller classification | [`JunctionParser.cpp` L17-L50](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/JunctionParser.cpp#L17-L50) | Enhancement required | ❌ | Controller type classification | - |

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

#### SignalParser

[`SignalParser.cpp`](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/SignalParser.cpp)

##### Supported Signal Types

- **Traffic Signals**: Dynamic signals (traffic lights with changing states)
- **Traffic Signs**: Static signals (speed limits, stop signs, etc.)
- **Signal Validity**: Lane-specific signal applicability via `fromLane`/`toLane`
- **Signal Dependencies**: Cross-referencing between related signals

##### Position Specification

Signals can be positioned using two methods:

- **Road coordinates** (`s`, `t`, `zOffset`): Position relative to road reference line
- **Inertial coordinates** (`x`, `y`, `z`, `hdg`, `pitch`, `roll`): Absolute world position

##### Signal References

- **`<signal>`**: Full signal definition with all attributes
- **`<signalReference>`**: Lightweight reference to existing signal by ID
- References allow multiple roads to share the same signal definition

##### Lanelet2 Mapping

- `regulatory_element` with `traffic_light` or `traffic_sign` subtypes map to signals
- Signal ID: Use regulatory_element ID
- Signal type: Map Lanelet2 subtype to OpenDRIVE type codes (e.g., 1000003 for red/yellow/green)
- Signal position: Calculate s-coordinate from regulatory_element refers
- Signal validity: Map refers lanelets to fromLane/toLane range

#### ControllerParser

[`ControllerParser.cpp`](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/ControllerParser.cpp)

##### Purpose

Controllers manage groups of traffic signals, coordinating their states and sequencing (e.g., traffic light phases at an intersection).

##### Key Elements

- **Controller ID**: Unique identifier for signal group
- **Sequence**: Phase sequence number (order of signal state changes)
- **Control**: List of signal IDs managed by this controller

##### Lanelet2 Mapping

- Lanelet2 has no built-in signal controller concept
- Controllers can be generated from traffic_light groups at intersections
- Group all traffic_lights affecting the same junction into one controller
- Sequence number requires external phase timing data (not in Lanelet2)

##### Current Status

Controllers are parsed but signal phase timing logic is not fully integrated in CARLA's map builder.

#### ObjectParser

[`ObjectParser.cpp`](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/ObjectParser.cpp)

##### Supported Object Types

CARLA recognizes specific object types for autonomous driving scenarios:

- **`crosswalk`**: Pedestrian crossing areas with detailed outline geometry
- **Speed signs**: Objects named `Speed_*` (e.g., `Speed_30`, `Speed_50`)
- **Stop lines**: Objects named `Stencil_STOP` for stop sign markings

##### Object Geometry

- **Simple objects**: Defined by position, dimensions (width, length, height), and orientation
- **Outline objects**: Defined by polygon vertices using `<outline>/<cornerLocal>` with U,V,Z coordinates

##### Lanelet2 Mapping

- **Crosswalks**: Map from Lanelet2 `area` with `subtype=crosswalk`
- **Speed signs**: Generate from `speed_limit` regulatory_elements
- **Stop lines**: Map from `stop_line` regulatory_elements
- Object position: Project area/element center onto road to get s,t coordinates
- Object outline: Transform area polygon vertices to local U,V coordinates

##### RoadRunner Integration

This parser is designed to work with RoadRunner-generated OpenDRIVE files, which use object naming patterns like `Speed_*` and `Stencil_STOP` for signal-like elements.

#### TrafficGroupParser

[`TrafficGroupParser.cpp`](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/TrafficGroupParser.cpp)

##### Current Status: STUBBED

**Important**: This parser exists in CARLA's codebase but is **currently disabled** (all parsing code is commented out).

##### Intended Purpose

Parse traffic signal timing groups with phase durations:

- **Red time**: Duration of red signal phase (seconds)
- **Yellow time**: Duration of yellow signal phase (seconds)
- **Green time**: Duration of green signal phase (seconds)

##### XML Structure

Located in `<userData>` section as custom extension:

```xml
<userData>
  <trafficGroup id="group1" redTime="30" yellowTime="5" greenTime="25"/>
</userData>
```

##### Lanelet2 Mapping

Not applicable - Lanelet2 has no signal timing data. This would require external timing configuration or simulation-specific data.

##### Future Implementation

If enabled, this would allow CARLA to simulate realistic traffic signal timing patterns, but requires:

1. Uncommenting parser code in TrafficGroupParser.cpp
2. Integrating timing data into MapBuilder's signal management
3. External timing configuration source (Lanelet2 doesn't provide this)

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

    ### Signal and Object Support

    CARLA **does support** the following elements:

    1. **Traffic Signals** (`road/signals/signal`):
       - Fully parsed via SignalParser
       - Supports both static signs and dynamic traffic lights
       - Includes signal validity ranges, dependencies, and positioning
       - See SignalParser section for detailed mapping

    2. **Road Objects** (`road/objects/object`):
       - Parsed via ObjectParser for specific types
       - Supported: crosswalks, speed signs (Speed_*), stop lines (Stencil_STOP)
       - Objects can have outline geometry for precise boundaries

    3. **Traffic Controllers** (`controller`):
       - Parsed via ControllerParser
       - Manages signal groups and sequencing
       - Phase timing integration is partial

    ### Elements Not Used by CARLA

    The following OpenDRIVE elements are **not** currently parsed or have limited support:

    - `road/surface` (road surface properties) - Not parsed
    - `lane/userData` (custom lane data) - Not parsed
    - `userData/trafficGroup` (signal timing) - Parser exists but stubbed out
    - Complex geometry types (e.g., `<spiral>` support varies by CARLA version)

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
