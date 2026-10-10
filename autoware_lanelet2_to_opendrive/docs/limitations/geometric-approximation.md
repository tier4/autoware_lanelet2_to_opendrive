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

### The emitted curve *is* the fitted curve

The fitted reference line is a cubic B-spline. Inside one knot span a cubic
B-spline is a single cubic polynomial, so the emitter cuts the reference line
at the spline's own breakpoints and writes each span out as one
`<paramPoly3>`. The emitted chain therefore reproduces the fit exactly
(measured on Nishi-Shinjuku: max position deviation ~1e-12 m, curvature step
between consecutive segments ~4e-9 1/m) and the only approximation left is the
spline fit against the Lanelet2 points. The previous emitter re-fitted the
spline onto a uniform grid with a cubic Hermite construction that discarded
second derivatives; that added up to 0.26 m of its own error and broke the C2
continuity of the fit at every segment boundary.

Subdivision, when configured (`DEFAULT_CONFIG.parampoly3.knot_span_max_length`),
only ever cuts *inside* a knot span, so the pieces stay exact. Setting
`DEFAULT_CONFIG.parampoly3.knot_aligned = False` restores the old uniform
split for A/B comparison.

### `pRange="normalized"`, not `"arcLength"`

Each emitted `<paramPoly3>` declares `pRange="normalized"`: `p` runs over
`[0, 1]` and is the spline's own parameter, **not** travelled distance. A
cubic polynomial cannot be unit-speed unless it is a straight line, so the
former `pRange="arcLength"` declaration was never true and left ASAM's
`road.geometry.parampoly3.arclength_range` reporting 277 of 9,428 segments
(worst 0.98 m on a 2.75 m segment). `geometry@length` is now the curve's true
arc length, computed by Gauss-Legendre quadrature over the emitted
coefficients, which is what both that rule and its `normalized_range` sibling
actually check.

Consumers must not treat `p` as a distance. CARLA does not
(`GeometryParamPoly3::PreComputeSpline` tabulates real arc length and honours
the `pRange` attribute); inside this converter, use
`opendrive.geometry.param_for_offset` / `end_param` to go from a distance
along the geometry to `p`.

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
