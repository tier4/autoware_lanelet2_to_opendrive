# Geometric Approximation Limitations

## Issue

Complex curved geometries may be simplified during conversion.

## Cause

- Lanelet2 uses discrete point sequences for geometry representation
- OpenDRIVE uses parametric curves (lines, arcs, spirals, cubic polynomials)
- Fitting parametric curves to discrete points involves approximation

## Impact

- Very tight curves may lose some precision
- Sharp corners may be slightly smoothed
- Small geometric features may be simplified

## Mitigation

The converter uses **high-quality spline fitting algorithms** to minimize approximation errors while maintaining smooth, drivable geometry.

---

[← Back to Limitations Overview](index.md)
