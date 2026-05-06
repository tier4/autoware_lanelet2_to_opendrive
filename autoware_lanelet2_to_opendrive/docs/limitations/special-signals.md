# Special Traffic Signals

This page tracks the traffic-signal types the converter handles today and
the ones still missing.

## Currently supported

- **Vehicle 3-aspect traffic lights** (`@type=1000001`, `country="DE"`)
- **Pedestrian traffic lights** (`@type=1000002`, `country="DE"`)
- **Arrow encoding for vehicle traffic lights** — the per-bulb `arrow`
  attribute on the Lanelet2 `light_bulbs` LineString is encoded into the
  signal's `@subtype` as a bitmask (`left=1`, `right=2`, `up=4`); see
  [Signals](../signals.md#subtype-encoding-for-vehicle-traffic-lights)
  for the full table.
- **StopSign signals** (`@type=206`) — emitted for stop lines associated
  with a `traffic_sign` regulatory element whose `refers` member has
  `subtype="stop_sign"`
- **YieldSign signals** (`@type=205`) — emitted for stop lines from a
  `road_marking` regulatory element
- **StopLine signals** (`@type=294`) — emitted alongside the supported
  `<object type="stopLine">` and carrying `<dependency>` references back
  to the controlling traffic light, stop sign, or yield sign
- **Signal-junction association** via `<controller>` and the existing
  `<signalReference>` mechanism (issue #135)

## Not yet supported

- **Conditional or time-based signals** (e.g. variable-message signs,
  time-of-day speed limits) — OpenDRIVE 1.4 has no first-class
  representation; we emit only static signals
- **Flashing-only signals** (e.g. flashing-yellow caution beacons) — no
  dedicated signal subtype is emitted
- **Railroad-crossing signals**
- **General traffic signs beyond stop / yield** (speed-limit signs,
  warning signs, regulatory signs, etc.) — Lanelet2's
  `traffic_sign` regulatory element is read only for the `stop_sign`
  case; other subtypes are silently skipped
- **Per-lane signal validity** beyond a single `<validity>` row covering
  the lanelets the regulatory element references — the converter does
  not currently split `<validity>` rows when only a subset of the lanes
  is affected

## Cause

The converter targets CARLA, whose OpenDRIVE parser only reads a small
subset of the `<signal>` schema (see the table in
[CARLA OpenDRIVE Mapping](../carla_opendrive_lanelet2_mapping.md#signalparser)):
the type, value, position, and orientation fields. Anything CARLA does
not consume has no end-to-end test coverage today, so the converter
implements only what other features in the project depend on.

## Impact

When converting maps that contain unsupported special signals:

- The signal is silently skipped (no warning is emitted today)
- Directional or conditional information that does not map to the
  arrow-bulb subtype encoding is lost
- Lane-specific signal phases that depend on a finer `<validity>` split
  fall back to the regulatory element's full set of referring lanelets

## Future Support

Support for more signal types may be added based on:

- Demand from other simulators (RoadRunner, esmini, SUMO) which
  consume more of the OpenDRIVE signal schema than CARLA does
- Standardisation work in newer OpenDRIVE revisions
- Maintainer availability and project priorities

## Requesting Support

If you need an unsupported signal type:

1. **Open a GitHub issue**: [Create new issue](https://github.com/tier4/autoware_lanelet2_to_opendrive/issues/new/choose)
2. **Describe the use case** — which simulator/platform, which Lanelet2
   regulatory element, and the expected OpenDRIVE representation
3. **Attach a sample** — a small `.osm` fixture and the expected
   `.xodr` snippet make implementation much faster
4. **Engage maintainers** in the issue thread

!!! tip "Community Contributions"
    Pull requests adding new signal types are welcome. See the
    [Development Guide](../development.md) for the contribution
    workflow.

---

[← Back to Limitations Overview](index.md)
