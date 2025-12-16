# OpenDRIVE Signals Implementation

This document describes the implementation of traffic signals (traffic lights, signs, etc.) for OpenDRIVE format conversion.

## Overview

The signals implementation follows the ASAM OpenDRIVE v1.8.1 specification (Section 14: Signals) and provides a complete framework for adding traffic signals to OpenDRIVE roads.

## Module Structure

### `signal.py`
Core signal definitions:
- **`Signal`**: Main signal class representing a single traffic signal
- **`Validity`**: Defines which lanes a signal applies to
- **`SignalUserData`**: Container for custom user-defined data
- **`SignalType`**: Constants for common signal types (traffic lights, etc.)

### `signals.py`
Container class:
- **`Signals`**: Container class for managing multiple signals on a road

## Usage

### Basic Example

```python
from autoware_lanelet2_to_opendrive.opendrive import (
    Road,
    Signal,
    Signals,
    Validity,
    SignalType,
)

# Create a road
road = Road(id=1, name="Main Street", length=200.0)

# Create signals container
signals = Signals()

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
    country="OpenDRIVE",
    type=SignalType.TRAFFIC_LIGHT_3_LIGHTS,
    subtype=-1,
    value=-1.0,
    text="",
    height=1.2,
    width=0.6,
    validities=[Validity(from_lane=-1, to_lane=-1)],
)
signals.add_signal(traffic_light)

# Attach signals to road
road.signals = signals

# Generate XML
xml = road.to_xml()
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
    country="OpenDRIVE",
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

These types use `country="OpenDRIVE"` for simulation-specific signals. For national regulations, use appropriate country codes (e.g., "US", "DE", "JP").

## Signal Attributes

### Required Attributes

- **`id`**: Unique signal identifier
- **`name`**: Signal name
- **`s`**: Position along road reference line (m)
- **`t`**: Lateral offset from road reference line (m)
- **`orientation`**: "+" or "-" (orientation with respect to road direction)
- **`dynamic`**: "yes" or "no" (whether signal changes during simulation)
- **`country`**: Country code or "OpenDRIVE"
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
<road id="1" length="200.0" junction="-1" name="Main Street">
  <!-- ... other road elements ... -->
  <signals>
    <signal id="100" name="TrafficLight_1"
            s="5.0000000000000000e+01"
            t="-4.5000000000000000e+00"
            zOffset="0.0000000000000000e+00"
            orientation="-"
            dynamic="yes"
            country="OpenDRIVE"
            type="1000001"
            subtype="-1"
            height="1.2000000000000000e+00"
            width="6.0000000000000000e-01">
      <validity fromLane="-1" toLane="-1"/>
    </signal>
  </signals>
</road>
```

## Running the Demo

A complete demo script is provided:

```bash
uv run python examples/signal_demo.py
```

This demonstrates:
1. Creating traffic lights using the Signal constructor
2. Creating signals with custom attributes
3. Creating pedestrian traffic lights
4. Generating OpenDRIVE XML output

## Testing

Comprehensive tests are provided in `test/test_signals.py`:

```bash
uv run pytest test/test_signals.py -v
```

Tests cover:
- Signal creation and XML conversion
- Signals container operations
- Road integration
- XML formatting and structure

## References

- **ASAM OpenDRIVE v1.8.1 Specification**: [Section 14 - Signals](https://publications.pages.asam.net/standards/ASAM_OpenDRIVE/ASAM_OpenDRIVE_Specification/v1.8.1/specification/14_signals/14_01_introduction.html)
- **OpenDRIVE Signal Reference**: [Signal Catalog](https://publications.pages.asam.net/standards/ASAM_OpenDRIVE/ASAM_OpenDRIVE_Signal_reference/latest/signal-catalog/00_preface/00_introduction.html)

## Integration with Existing Code

The signals functionality integrates seamlessly with the existing `Road` class:

1. Signals are optional - roads without signals work as before
2. Signals are added via the `road.signals` attribute
3. XML output automatically includes signals when present
4. All existing road functionality remains unchanged

## Future Enhancements

Potential future enhancements:
- Signal dependencies and relationships
- Signal controllers
- Signal references (signalReference element)
- More signal type constants for different countries
- Automated signal placement from Lanelet2 regulatory elements
