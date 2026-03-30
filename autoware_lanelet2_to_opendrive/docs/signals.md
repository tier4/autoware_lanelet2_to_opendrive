# OpenDRIVE Signals Implementation

This document describes the implementation of traffic signals (traffic lights, signs, etc.) for OpenDRIVE format conversion.

## Overview

The signals implementation follows the ASAM OpenDRIVE v1.8.1 specification (Section 14: Signals) and provides a framework for creating traffic signal objects.

## Module Structure

### `signal.py`
Core signal definitions:
- **`Signal`**: Main signal class representing a single traffic signal
- **`Validity`**: Defines which lanes a signal applies to
- **`SignalUserData`**: Container for custom user-defined data
- **`SignalType`**: Constants for common signal types (traffic lights, etc.)

## Usage

### Basic Example

```python
from autoware_lanelet2_to_opendrive.opendrive import (
    Signal,
    Validity,
    SignalType,
)

# Create a traffic light using Signal constructor
traffic_light = Signal(
    id=100,
    name="TrafficLight_1",
    s=50.0,  # Position along road (m)
    t=-4.5,  # Lateral offset (m)
    z_offset=0.0,
    h_offset=0.0,
    roll=0.0,
    pitch=0.0,
    orientation="-",
    dynamic="yes",  # Traffic lights are dynamic
    country="DE",
    type=SignalType.TRAFFIC_LIGHT_3_LIGHTS,
    subtype=-1,
    value=-1.0,
    text="",
    height=1.2,
    width=0.6,
    validities=[Validity(from_lane=-1, to_lane=-1)],
)

# Generate XML
xml = traffic_light.to_xml()
```

### Creating Multiple Signals

```python
from autoware_lanelet2_to_opendrive.opendrive import Signal, Validity

# Create a signal with full control over all attributes
signal = Signal(
    id=101,
    name="CustomSignal",
    s=75.0,
    t=-5.0,
    z_offset=0.5,
    h_offset=0.0,
    roll=0.0,
    pitch=0.0,
    orientation="-",
    dynamic="yes",  # Changes during simulation
    country="DE",
    type=1000001,  # 3-light traffic signal
    subtype=-1,
    value=-1.0,
    text="",
    height=1.5,
    width=0.7,
    validities=[
        Validity(from_lane=-2, to_lane=-1)  # Applies to lanes -2 and -1
    ],
)
```

## Signal Types

The `SignalType` class provides constants for common signal types:

- **`TRAFFIC_LIGHT_3_LIGHTS`** (1000001): Standard 3-light traffic signal (red, yellow, green)
- **`TRAFFIC_LIGHT_PEDESTRIAN`** (1000002): Pedestrian traffic light
- **`TRAFFIC_LIGHT_ARROW`** (1000003): Arrow traffic light

These types use `country="DE"` (German StVO). For other national regulations, use appropriate country codes (e.g., "US", "JP").

## Signal Attributes

### Required Attributes

- **`id`**: Unique signal identifier
- **`name`**: Signal name
- **`s`**: Position along road reference line (m)
- **`t`**: Lateral offset from road reference line (m)
- **`orientation`**: "+" or "-" (orientation with respect to road direction)
- **`dynamic`**: "yes" or "no" (whether signal changes during simulation)
- **`country`**: Country code (e.g., "DE", "US", "JP")
- **`type`**: Signal type ID
- **`subtype`**: Signal subtype ID

### Optional Attributes

- **`z_offset`**: Height offset above road surface (m)
- **`h_offset`**: Heading offset (rad)
- **`roll`**: Roll angle (rad)
- **`pitch`**: Pitch angle (rad)
- **`value`**: Signal value (e.g., speed limit value)
- **`text`**: Signal text content
- **`height`**: Signal physical height (m)
- **`width`**: Signal physical width (m)

## Lane Validity

The `Validity` class defines which lanes a signal applies to:

```python
from autoware_lanelet2_to_opendrive.opendrive import Validity

# Apply to single lane
validity = Validity(from_lane=-1, to_lane=-1)

# Apply to multiple lanes
validity = Validity(from_lane=-3, to_lane=-1)  # Lanes -3, -2, -1

# Apply to center lane
validity = Validity(from_lane=0, to_lane=0)
```

**Note**: In OpenDRIVE, lane IDs are:
- **Positive** for left lanes (1, 2, 3, ...)
- **Zero** for center lane
- **Negative** for right lanes (-1, -2, -3, ...)

## XML Output Format

The implementation generates OpenDRIVE-compliant XML:

```xml
<signal id="100" name="TrafficLight_1"
        s="5.0000000000000000e+01"
        t="-4.5000000000000000e+00"
        zOffset="0.0000000000000000e+00"
        orientation="-"
        dynamic="yes"
        country="DE"
        type="1000001"
        subtype="-1"
        height="1.2000000000000000e+00"
        width="6.0000000000000000e-01">
  <validity fromLane="-1" toLane="-1"/>
</signal>
```

## Testing

Comprehensive tests are provided in `test/test_signals.py`:

```bash
uv run pytest test/test_signals.py -v
```

Tests cover:
- Signal creation and XML conversion
- Validity and SignalUserData functionality
- Signal conversion from lanelet2 traffic lights
- XML formatting and structure
- Error handling for invalid inputs

## Signal References

Signal references (`<signalReference>`) are companion elements to signals that specify the signal's position on the road reference line. While a `<signal>` element can be placed at any lateral offset (t-coordinate), a `<signalReference>` is always placed at t=0.0 (on the reference line) to facilitate lane assignment.

### Implementation

For each `<signal>` element generated, a corresponding `<signalReference>` element is automatically created with:

- **id**: Same as the signal's id
- **s**: Same s-coordinate as the signal
- **t**: Always "0.0" (on the reference line)
- **orientation**: Same orientation as the signal
- **validity**: Copy of the signal's validity elements

### XML Example

```xml
<signals>
  <!-- Original signal at lateral offset -->
  <signal id="59" name="TrafficLight_3002245"
          s="44.13" t="6.51" orientation="+"
          dynamic="yes" type="1000001">
    <validity fromLane="-1" toLane="-1"/>
  </signal>

  <!-- Corresponding signalReference on reference line -->
  <signalReference id="59" s="44.13" t="0.0" orientation="+">
    <validity fromLane="-1" toLane="-1"/>
  </signalReference>
</signals>
```

### Usage

Signal references are generated automatically by the conversion process. No manual intervention is required.

## References

- **ASAM OpenDRIVE v1.8.1 Specification**: [Section 14 - Signals](https://publications.pages.asam.net/standards/ASAM_OpenDRIVE/ASAM_OpenDRIVE_Specification/v1.8.1/specification/14_signals/14_01_introduction.html)
- **OpenDRIVE Signal Reference**: [Signal Catalog](https://publications.pages.asam.net/standards/ASAM_OpenDRIVE/ASAM_OpenDRIVE_Signal_reference/latest/signal-catalog/00_preface/00_introduction.html)

## Converting from Lanelet2

The `Signal.construct_from_lanelet2_traffic_signal()` method converts lanelet2 traffic light regulatory elements to OpenDRIVE signals:

```python
import lanelet2
from autoware_lanelet2_to_opendrive.opendrive import Signal

# Load lanelet2 map
lanelet_map = lanelet2.io.load("map.osm", projector)

# Get traffic light from lanelet
lanelet = lanelet_map.laneletLayer.get(lanelet_id)
traffic_lights = [reg for reg in lanelet.regulatoryElements
                  if isinstance(reg, lanelet2.core.TrafficLight)]
traffic_light = traffic_lights[0]

# Convert to OpenDRIVE Signal
signal = Signal.construct_from_lanelet2_traffic_signal(
    traffic_light=traffic_light,
    signal_id=100,
    s=50.0,      # Position along road reference line
    t=-4.5,      # Lateral offset from reference line
    lane_ids=[-1]  # Lanes this signal applies to
)
```

## Future Enhancements

Potential future enhancements:
- Signal dependencies and relationships
- ✅ Signal references (signalReference element) - Implemented in Issue #135
- More signal type constants for different countries
- Signal controllers for coordinated traffic light control
