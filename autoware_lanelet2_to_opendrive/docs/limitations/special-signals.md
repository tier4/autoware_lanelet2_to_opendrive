# Special Traffic Signals Not Supported

## Issue

Special traffic signal types are not supported in the converter.

## Unsupported Signal Types

The following signal types are **not supported**:

- ❌ **Arrow signals** (left arrow, right arrow, straight arrow)
- ❌ **Pedestrian crossing signals** (walk/don't walk)
- ❌ **Directional signals** (lane-specific arrows)
- ❌ **Conditional signals** (time-based, vehicle-type-based)
- ❌ **Flashing signals** (yellow caution, red stop-then-go)
- ❌ **Railroad crossing signals**

## Supported Signal Types

Currently supported:

- ✅ **Standard traffic lights** (red, yellow, green)
- ✅ **Basic signal positioning**
- ✅ **Signal-junction association**

## Cause

This tool was originally developed to support **CARLA simulator integration**. CARLA has the following limitations:

- CARLA does not support special traffic signal types
- CARLA only implements basic 3-state traffic lights (red, yellow, green)
- Advanced signal behaviors are not part of CARLA's core traffic system

Since the primary use case was CARLA compatibility, these signal types were not implemented in the converter.

## Impact

When converting maps with special signals:

- Special signal types will be **ignored or simplified** to basic traffic lights
- Directional information (arrows) will be **lost**
- Lane-specific signal rules will **not be preserved**

## Future Support

Support for additional traffic signal types may be added based on:

- Community demand for other simulators
- Development of standardized signal representation in OpenDRIVE
- Maintainer availability and project priorities

## Requesting Support

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
    Pull requests implementing support for additional signal types are welcome! Check the [Development Guide](../development.md) for contribution guidelines.

---

[← Back to Limitations Overview](index.md)
