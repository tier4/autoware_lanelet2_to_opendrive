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

The converter uses **high-quality spline fitting algorithms** to minimize
approximation errors while maintaining smooth, drivable geometry.

### Optional `<line>` / `<arc>` / `<paramPoly3>` classification

The default emitter writes a chain of `<paramPoly3>` segments. As of
issue #466, an optional classifier can split each fitted reference line
into runs of `<line>`, `<arc>`, and `<paramPoly3>` primitives, reducing
approximation error on geometries that are actually straight or
constant-curvature. Enable it via:

```yaml
arcspiral:
  enabled: true        # off by default for byte-stable output
  arc_enabled: true
  min_line_length: 5.0
  min_arc_length: 5.0
```

(see
[`conversion_config.py:ArcSpiralConfig`](https://github.com/tier4/autoware_lanelet2_to_opendrive/blob/master/autoware_lanelet2_to_opendrive/src/autoware_lanelet2_to_opendrive/conversion_config.py)
for the full set of tolerances). Spiral / clothoid emission
(`spiral_enabled`) is reserved for follow-up issue #466b and is
currently a no-op.

---

[← Back to Limitations Overview](index.md)
