# Stop Line Position Discrepancies

## Issue

Stop line positions may not be accurately preserved in the conversion.

## Cause

CARLA simulator (the primary target platform for this tool) uses **TriggerVolume-based collision detection** for traffic signals. This means:

- Traffic signals in CARLA use invisible 3D trigger volumes to detect vehicles
- Stop line positions are automatically determined by the trigger volume placement
- The automatic positioning can differ from explicit stop line positions defined in Lanelet2 maps

### Technical Analysis with Code References

**1. Traffic Light Trigger Volume Implementation**

CARLA uses bounding box-based trigger volumes for traffic light detection:

- **Traffic Light Class**: [`LibCarla/source/carla/trafficmanager/TrafficLight.cpp`](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/trafficmanager/TrafficLight.cpp)
  - Uses `GetTriggerVolume()` to get the detection area
  - Stop line position is **implicitly defined** by the trigger volume boundary
  - **No explicit stop line coordinate** is stored or read from OpenDRIVE

- **Traffic Light Manager**: [`PythonAPI/carla/source/libcarla/TrafficLight.cpp`](https://github.com/carla-simulator/carla/blob/master/PythonAPI/carla/source/libcarla/TrafficLight.cpp)
  - `GetStopWaypoints()` returns waypoints within the trigger volume
  - These waypoints are **approximations**, not precise stop line positions from the map

**2. OpenDRIVE Signal Parsing: Stop Line Position Ignored**

CARLA's OpenDRIVE parser does not extract stop line positions:

- **Signal Parser**: [`LibCarla/source/carla/opendrive/parser/SignalParser.cpp`](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/SignalParser.cpp)
  - Parses signal `type`, `value`, `s` (longitudinal position), `t` (lateral offset)
  - **Does not parse** `<validity>` elements that could define precise stop line locations
  - **Does not parse** stop line geometries from road objects

**3. Road Object Parser: Stop Lines Not Extracted**

Stop lines can be defined as road objects in OpenDRIVE, but CARLA ignores them:

- **Object Parser**: [`LibCarla/source/carla/opendrive/parser/ObjectParser.cpp`](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/ObjectParser.cpp)
  - Parses road objects like poles, barriers, etc.
  - **Does not specifically handle** stop line road objects
  - Stop line geometries are not converted into game world coordinates

## Impact

- Stop lines in the resulting OpenDRIVE map may be **shifted** from their original Lanelet2 positions
- The shift amount depends on CARLA's trigger volume configuration
- This is a CARLA architectural limitation, not a converter bug
- Precise stop line positions from Lanelet2 maps are **lost in translation**

## Workaround

### Temporary Solutions (No CARLA Modification)

If precise stop line positioning is critical for your use case:

1. **Manual trigger volume adjustment**: Adjust trigger volumes in CARLA after importing the map
2. **Post-processing scripts**: Modify the OpenDRIVE file to adjust signal positions
3. **Use alternative simulator**: Consider simulators that support explicit stop line positioning

### Root Solution: CARLA Source Modification

<details>
<summary><strong>Advanced: Modify CARLA source code for precise stop line support</strong> (click to expand)</summary>

!!! warning "Out of Scope"
    The following information is provided for reference only. **Modifying CARLA's source code is not within the scope of this converter project**. This is a CARLA-side limitation that requires changes to the CARLA simulator itself.

#### Required Modifications

To support precise stop line positioning from OpenDRIVE, the following changes are needed:

**1. Extend Signal Data Structure**

File: `LibCarla/source/carla/road/element/RoadInfoSignal.h`

```cpp
// Add stop line position field
class RoadInfoSignal : public RoadInfo {
public:
  // Existing fields...

  // NEW: Add explicit stop line position
  boost::optional<geom::Location> _stop_line_position;

  // NEW: Add getter/setter
  const boost::optional<geom::Location>& GetStopLinePosition() const {
    return _stop_line_position;
  }

  void SetStopLinePosition(const geom::Location& position) {
    _stop_line_position = position;
  }
};
```

**2. Parse Stop Line Position from OpenDRIVE**

File: `LibCarla/source/carla/opendrive/parser/SignalParser.cpp`

```cpp
void SignalParser::Parse(/* ... */) {
  // Existing signal parsing code...

  // NEW: Parse validity element for stop line position
  pugi::xml_node validity_node = signal_node.child("validity");
  if (validity_node) {
    double from_lane = validity_node.attribute("fromLane").as_double();
    double to_lane = validity_node.attribute("toLane").as_double();

    // Calculate stop line position from validity range
    // This requires road geometry calculations
    auto stop_line_pos = CalculateStopLinePosition(
      road_id, s_position, from_lane, to_lane);

    signal_info->SetStopLinePosition(stop_line_pos);
  }

  // NEW: Parse road object for stop line geometry (alternative method)
  // OpenDRIVE can define stop lines as <object type="stopLine">
  pugi::xml_node objects_node = road_node.child("objects");
  for (pugi::xml_node object_node : objects_node.children("object")) {
    std::string type = object_node.attribute("type").as_string();
    if (type == "stopLine") {
      // Parse stop line geometry
      double s = object_node.attribute("s").as_double();
      double t = object_node.attribute("t").as_double();

      // Convert to world coordinates
      auto stop_line_pos = ConvertToWorldCoords(road_id, s, t);

      // Associate with nearby traffic signal
      AssociateStopLineWithSignal(signal_info, stop_line_pos);
    }
  }
}
```

**3. Use Explicit Stop Line in Traffic Manager**

File: `LibCarla/source/carla/trafficmanager/TrafficLight.cpp`

```cpp
// Modify vehicle detection to use explicit stop line position

bool TrafficLight::ShouldStop(const Vehicle& vehicle) {
  // OLD: Use trigger volume boundary
  // return vehicle.IsInsideTriggerVolume(GetTriggerVolume());

  // NEW: Use explicit stop line position if available
  if (signal_info->GetStopLinePosition()) {
    geom::Location stop_line = *signal_info->GetStopLinePosition();
    double distance = geom::Distance(vehicle.GetLocation(), stop_line);

    // Check if vehicle has crossed the stop line
    double stop_threshold = 0.5; // meters
    return distance < stop_threshold;
  }

  // Fallback to trigger volume if no explicit position
  return vehicle.IsInsideTriggerVolume(GetTriggerVolume());
}
```

**4. Update Python API**

File: `PythonAPI/carla/source/libcarla/TrafficLight.cpp`

```cpp
// Expose stop line position to Python API

static auto GetStopLinePosition(const csd::TrafficLight &self) {
  auto signal_info = self.GetSignalInfo();
  if (signal_info && signal_info->GetStopLinePosition()) {
    return *signal_info->GetStopLinePosition();
  }
  throw std::runtime_error("No explicit stop line position available");
}

// Register in Python bindings
class_<csd::TrafficLight, ...>
  // ...
  .def("get_stop_line_position", &GetStopLinePosition)
```

#### Implementation Steps

1. **Fork CARLA repository**: `git clone https://github.com/carla-simulator/carla.git`
2. **Create feature branch**: `git checkout -b feature/explicit-stop-lines`
3. **Apply modifications**: Implement the changes above
4. **Update OpenDRIVE exporter**: Ensure stop lines are properly exported from Lanelet2
5. **Test with sample maps**: Verify stop line positions match Lanelet2 source
6. **Rebuild CARLA**: Follow [CARLA Build Guide](https://carla.readthedocs.io/en/latest/build_linux/)
7. **Submit PR to CARLA**: Propose changes to CARLA maintainers

#### Expected Benefits

- ✅ Stop line positions accurately preserved from Lanelet2 maps
- ✅ No manual trigger volume adjustment needed
- ✅ Consistent behavior across different map formats
- ✅ Better compliance with OpenDRIVE specification

!!! danger "Complex Modification"
    This requires deep understanding of:
    - CARLA's C++ codebase architecture
    - OpenDRIVE road geometry calculations
    - Traffic signal-road association logic
    - Python binding system (pybind11)

</details>

---

[← Back to Limitations Overview](index.md)
