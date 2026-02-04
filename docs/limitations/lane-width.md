# Lane Width Inconsistencies

## Issue

Lane widths in the converted OpenDRIVE map may not exactly match the original Lanelet2 map.

## Cause

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

## Impact

- Lane widths may **vary slightly** from the original Lanelet2 specification
- The variation depends on:
    - Complexity of the lane geometry (curves vs straight sections)
    - Number and distribution of boundary points in the original Lanelet2 map
    - Spline fitting parameters and interpolation method
- Exact width preservation is **mathematically impossible** due to format differences

## Typical Variation Range

In most cases:

- **Straight sections**: Width variation < 5 cm
- **Curved sections**: Width variation < 10 cm
- **Complex junctions**: Width variation < 20 cm

!!! info "Design Trade-off"
    The converter prioritizes **smooth, drivable geometry** over exact width preservation. This ensures generated maps work well in simulation environments.

## Workaround

If exact lane widths are critical:

1. Validate converted maps against width requirements in your target simulator
2. Adjust width tolerances in your validation scripts to account for interpolation errors
3. Use higher-resolution boundary point data in your Lanelet2 maps for better approximation

---

[← Back to Limitations Overview](index.md)
