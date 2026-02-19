# Cartesian to Frenet Coordinate Conversion

This example demonstrates how to use the `Splines.cartesian_to_frenet()` method to convert Cartesian coordinates (x, y, z) to Frenet coordinates (s, d).

## Overview

Frenet coordinates are a natural way to describe positions relative to a reference path:
- **s**: Arc length along the reference line (longitudinal position)
- **d**: Lateral offset from the reference line (positive = left, negative = right)

## Usage

### Basic Example

```python
import numpy as np
from autoware_lanelet2_to_opendrive.spline import Splines

# Create waypoints for a road centerline
wx = np.array([0.0, 20.0, 40.0, 60.0, 80.0])
wy = np.array([0.0, 0.0, 20.0, 20.0, 40.0])
wz = np.zeros_like(wx)
points = np.column_stack([wx, wy, wz])

# Create a spline (reference line)
spline = Splines(points, num_control_points=12)

# Convert a Cartesian point to Frenet coordinates
x, y, z = 10.0, 2.0, 0.0
s, d = spline.cartesian_to_frenet(x, y, z)

print(f"Cartesian: ({x}, {y}, {z})")
print(f"Frenet: s={s:.2f}m, d={d:.2f}m")
# Output: Frenet: s=10.12m, d=2.00m
```

### For 2D Applications

For 2D applications, you can omit the z parameter (defaults to 0.0):

```python
# 2D conversion (z defaults to 0.0)
x, y = 50.0, 5.0
s, d = spline.cartesian_to_frenet(x, y)
```

### Sign Convention

The lateral offset `d` follows this sign convention:
- **Positive d**: Point is on the LEFT side of the reference line
- **Negative d**: Point is on the RIGHT side of the reference line

Left/right is determined based on the tangent direction in the XY plane using a 2D cross product.

## Running the Demo

Run the visualization demo to see the conversion in action:

```bash
# From the repository root
uv run python examples/cartesian_to_frenet_demo.py
```

The demo will:
1. Create an S-curve reference line using waypoints
2. Convert several test points to Frenet coordinates
3. Display a visualization showing:
   - The reference line (black curve)
   - Query points (red circles)
   - Perpendicular distances to the reference line (red dashed lines)
   - s and d values with left/right indicators

## Use Cases

This functionality is useful for:

- **Lane tracking**: Determine a vehicle's position relative to the lane centerline
- **Path planning**: Generate trajectories in Frenet coordinates and convert back to Cartesian
- **Collision detection**: Check if objects are within lane boundaries
- **Map matching**: Snap GPS coordinates to road centerlines
- **Autonomous driving**: Track lateral and longitudinal positions of vehicles

## Implementation Details

The `cartesian_to_frenet()` method:
1. Uses numerical optimization (`scipy.optimize.minimize_scalar`) to find the closest point on the spline
2. Converts the spline parameter to arc length using pre-computed lookup tables
3. Calculates the signed lateral distance using vector geometry

The method handles:
- ✓ 2D and 3D coordinates
- ✓ Large coordinate values (e.g., UTM coordinates) with numerical stability
- ✓ Curved and straight paths
- ✓ Points at the start, end, or anywhere along the spline

## Performance Notes

- The arc length lookup table is computed once and cached
- Point-to-spline distance minimization uses bounded scalar optimization
- Typical conversion time: < 1ms per point

## See Also

- `Splines.evaluate(s)`: Convert arc length to Cartesian coordinates
- `test/test_cartesian_to_frenet.py`: Comprehensive test suite
