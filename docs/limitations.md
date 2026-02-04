# Known Limitations

This page documents implementation limitations and behavioral differences when converting from Lanelet2 to OpenDRIVE format.

!!! warning "Important"
    Understanding these limitations is crucial for setting realistic expectations when using this converter tool.

## Overview

The conversion from Lanelet2 to OpenDRIVE involves transforming between two fundamentally different map representation formats. Due to architectural differences between the formats and target platform constraints, some behavioral differences are expected and unavoidable.

---

## Stop Line Position Discrepancies

### Issue

Stop line positions may not be accurately preserved in the conversion.

### Cause

CARLA simulator (the primary target platform for this tool) uses **TriggerVolume-based collision detection** for traffic signals. This means:

- Traffic signals in CARLA use invisible 3D trigger volumes to detect vehicles
- Stop line positions are automatically determined by the trigger volume placement
- The automatic positioning can differ from explicit stop line positions defined in Lanelet2 maps

### Impact

- Stop lines in the resulting OpenDRIVE map may be **shifted** from their original Lanelet2 positions
- The shift amount depends on CARLA's trigger volume configuration
- This is a CARLA architectural limitation, not a converter bug

### Workaround

If precise stop line positioning is critical for your use case:

1. Manually adjust trigger volumes in CARLA after importing the map
2. Use post-processing scripts to modify the OpenDRIVE file
3. Consider using a different simulator that supports explicit stop line positioning

---

## Lane Width Inconsistencies

### Issue

Lane widths in the converted OpenDRIVE map may not exactly match the original Lanelet2 map.

### Cause

OpenDRIVE and Lanelet2 use **fundamentally different geometric representations**:

| Aspect | OpenDRIVE | Lanelet2 |
|--------|-----------|----------|
| **Base Representation** | Reference line-based | Lane boundary-based |
| **Lane Definition** | Lanes defined relative to center reference line | Lanes defined by explicit left and right boundaries |
| **Width Calculation** | Offset distances from reference line | Direct boundary-to-boundary measurement |

The conversion process requires **mathematical interpolation** to transform between these representations:

- **Spline fitting**: Used to generate smooth reference lines from discrete boundary points
- **Geometric algorithms**: Calculate lane offsets and widths from fitted splines
- **Numerical approximation**: Inherent in all spline-based interpolation methods

### Impact

- Lane widths may **vary slightly** from the original Lanelet2 specification
- The variation depends on:
    - Complexity of the lane geometry (curves vs straight sections)
    - Number and distribution of boundary points in the original Lanelet2 map
    - Spline fitting parameters and interpolation method
- Exact width preservation is **mathematically impossible** due to format differences

### Typical Variation Range

In most cases:

- **Straight sections**: Width variation < 5 cm
- **Curved sections**: Width variation < 10 cm
- **Complex junctions**: Width variation < 20 cm

!!! info "Design Trade-off"
    The converter prioritizes **smooth, drivable geometry** over exact width preservation. This ensures generated maps work well in simulation environments.

### Workaround

If exact lane widths are critical:

1. Validate converted maps against width requirements in your target simulator
2. Adjust width tolerances in your validation scripts to account for interpolation errors
3. Use higher-resolution boundary point data in your Lanelet2 maps for better approximation

---

## Special Traffic Signals Not Supported

### Issue

Special traffic signal types are not supported in the converter.

### Unsupported Signal Types

The following signal types are **not supported**:

- ❌ **Arrow signals** (left arrow, right arrow, straight arrow)
- ❌ **Pedestrian crossing signals** (walk/don't walk)
- ❌ **Directional signals** (lane-specific arrows)
- ❌ **Conditional signals** (time-based, vehicle-type-based)
- ❌ **Flashing signals** (yellow caution, red stop-then-go)
- ❌ **Railroad crossing signals**

### Supported Signal Types

Currently supported:

- ✅ **Standard traffic lights** (red, yellow, green)
- ✅ **Basic signal positioning**
- ✅ **Signal-junction association**

### Cause

This tool was originally developed to support **CARLA simulator integration**. CARLA has the following limitations:

- CARLA does not support special traffic signal types
- CARLA only implements basic 3-state traffic lights (red, yellow, green)
- Advanced signal behaviors are not part of CARLA's core traffic system

Since the primary use case was CARLA compatibility, these signal types were not implemented in the converter.

### Impact

When converting maps with special signals:

- Special signal types will be **ignored or simplified** to basic traffic lights
- Directional information (arrows) will be **lost**
- Lane-specific signal rules will **not be preserved**

### Future Support

Support for additional traffic signal types may be added based on:

- Community demand for other simulators
- Development of standardized signal representation in OpenDRIVE
- Maintainer availability and project priorities

### Requesting Support

If you require support for special traffic signals:

1. **Open a GitHub Issue**: [Create new issue](https://github.com/tier4/autoware_lanelet2_to_opendrive/issues/new/choose)
2. **Describe your use case**:
    - Which simulator or platform you're targeting
    - Which signal types you need
    - Expected behavior and semantics
3. **Provide examples**:
    - Sample Lanelet2 maps with the signal types
    - Expected OpenDRIVE representation
4. **Contact maintainers**: Tag maintainers in the issue for discussion

!!! tip "Community Contributions"
    Pull requests implementing support for additional signal types are welcome! Check the [Development Guide](development.md) for contribution guidelines.

---

## Priority-Based Right-of-Way Control Not Supported

### Issue

Priority tags (`priority` attribute in regulatory elements) for controlling intersection entry order are **not implemented** in the converter.

### Cause

While **technically feasible** to implement in the converter, this feature is not supported because **CARLA simulator does not recognize or utilize priority attributes** for traffic management at junctions.

#### CARLA's Complete Ignorance of Priority Attributes

CARLA simulator completely ignores priority information at every processing stage:

| Processing Stage | Result |
|------------------|--------|
| **① Parsing** | ❌ Priority attributes are not read from OpenDRIVE files |
| **② Data Storage** | ❌ No internal fields exist to store priority values |
| **③ Traffic Control** | ❌ Uses FIFO queue only for junction management; priority values are never consulted |

#### Technical Analysis with Code References

**1. Parsing Stage: Priority Not Read**

CARLA's OpenDRIVE parser does not extract `priority` attributes from regulatory elements:

- **OpenDRIVE Parser**: [`LibCarla/source/carla/opendrive/parser/SignalParser.cpp`](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/opendrive/parser/SignalParser.cpp)
  - Only parses signal `type`, `subtype`, `value`, `orientation`, and `position`
  - **No code exists** to parse `priority` attributes from `<signal>` or `<controller>` elements

**2. Data Storage: No Priority Field**

CARLA's internal traffic signal representation lacks priority storage:

- **Signal Class**: [`LibCarla/source/carla/road/element/RoadInfoSignal.h`](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/road/element/RoadInfoSignal.h)
  - Contains: `type`, `value`, `orientation`, `heading`, `pitch`, `roll`, `width`, `height`, `text`
  - **Does not contain**: `priority` field

- **Controller Class**: [`LibCarla/source/carla/road/SignalController.h`](https://github.com/carla-simulator/carla/blob/master/LibCarla/source/carla/road/SignalController.h)
  - Contains: `signal_ids`, `junction_id`
  - **Does not contain**: `priority` field

**3. Traffic Control: FIFO Queue Only**

CARLA's Traffic Manager uses a simple FIFO (First-In-First-Out) queue for junction management:

- **Traffic Manager Junction Logic**: [`LibCarla/source/carla/trafficmanager/`](https://github.com/carla-simulator/carla/tree/master/LibCarla/source/carla/trafficmanager)
  - Junction entry order is determined **solely by arrival time**
  - **No priority-based decision making** exists in the codebase
  - All vehicles are treated equally regardless of any potential priority values

### Impact

When converting Lanelet2 maps with `priority` regulatory elements:

- Priority information is **completely discarded** during conversion
- Intersection right-of-way behavior will **not match** the original Lanelet2 specification
- All vehicles will follow **FIFO queue behavior** in CARLA, regardless of intended priority

### Workaround

If priority-based right-of-way control is needed in CARLA:

#### Option 1: Signal Controller (Recommended)

Use CARLA's traffic signal controller system:

```xml
<!-- In OpenDRIVE file -->
<controller id="1" name="intersection_controller">
  <control signalId="10" type="traffic_light"/>
  <control signalId="11" type="traffic_light"/>
</controller>
```

- Define signal groups with controllers
- Use `sequence` attribute in signals to control timing
- Manually adjust signal phases to simulate priority behavior

**Reference**: [CARLA Signal Controller Documentation](https://carla.readthedocs.io/en/latest/core_map/#traffic-signs-and-lights)

#### Option 2: Python API Manual Control

Control traffic signals programmatically:

```python
# Get traffic light
traffic_light = world.get_traffic_light(traffic_light_id)

# Manually set state to implement priority logic
traffic_light.set_state(carla.TrafficLightState.Green)
```

**Reference**: [CARLA Traffic Light API](https://carla.readthedocs.io/en/latest/python_api/#carla.TrafficLight)

#### Option 3: Custom CARLA Source Modification

If priority support is critical:

1. Fork CARLA repository
2. Modify `SignalParser.cpp` to parse priority attributes
3. Add `priority` field to `RoadInfoSignal` and `SignalController` classes
4. Implement priority-based logic in Traffic Manager
5. Rebuild CARLA from source

**Reference**: [CARLA Build Documentation](https://carla.readthedocs.io/en/latest/build_linux/)

!!! warning "CARLA Source Modification Required"
    Implementing priority-based right-of-way control requires **modifying CARLA's C++ source code**. This is a non-trivial undertaking requiring deep understanding of CARLA's architecture.

### Requesting Support

If CARLA priority support is important for your use case:

1. **Open a CARLA Issue**: [CARLA GitHub Issues](https://github.com/carla-simulator/carla/issues)
2. **Describe the use case**:
    - Why priority-based right-of-way is needed
    - Expected behavior at intersections
    - Real-world traffic scenarios requiring priority
3. **Propose implementation**:
    - Suggest changes to data structures
    - Outline Traffic Manager modifications
4. **Engage CARLA community**: Discuss with CARLA maintainers

!!! info "Future Support"
    If CARLA adds priority attribute support in the future, this converter can be updated to include priority information in the generated OpenDRIVE files.

---

## Geometric Approximation Limitations

### Issue

Complex curved geometries may be simplified during conversion.

### Cause

- Lanelet2 uses discrete point sequences for geometry representation
- OpenDRIVE uses parametric curves (lines, arcs, spirals, cubic polynomials)
- Fitting parametric curves to discrete points involves approximation

### Impact

- Very tight curves may lose some precision
- Sharp corners may be slightly smoothed
- Small geometric features may be simplified

### Mitigation

The converter uses **high-quality spline fitting algorithms** to minimize approximation errors while maintaining smooth, drivable geometry.

---

## Coordinate System Considerations

### MGRS Projection

The converter requires a valid **MGRS (Military Grid Reference System)** code for coordinate transformation.

### Limitations

- Maps spanning multiple MGRS grid zones are **not supported**
- Very large maps (> 100 km²) may have accumulated projection errors
- High-latitude regions (> 84°N or < 80°S) are outside MGRS coverage

### Workaround

For maps spanning multiple grid zones:

1. Split the map into separate grid zone sections
2. Convert each section independently
3. Manually merge the resulting OpenDRIVE files (advanced)

---

## Summary

| Limitation | Severity | Workaround Available |
|------------|----------|---------------------|
| Stop line position discrepancies | Medium | Manual adjustment in CARLA |
| Lane width inconsistencies | Low | Validation with tolerances |
| Special signals not supported | High | Community contribution needed |
| Priority-based right-of-way not supported | Low | Signal controllers or Python API |
| Geometric approximation | Low | Use high-resolution input data |
| MGRS projection limitations | Medium | Split large maps by grid zone |

!!! info "Feedback Welcome"
    If you encounter limitations not documented here, please [report them](https://github.com/tier4/autoware_lanelet2_to_opendrive/issues) so we can improve this documentation.

---

## Next Steps

- Return to [Usage Guide](usage.md) for conversion instructions
- Check [API Reference](api.md) for programmatic access
- Visit [Development Guide](development.md) to contribute improvements
