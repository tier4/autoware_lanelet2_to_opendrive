# Known Limitations

This page documents implementation limitations and behavioral differences when converting from Lanelet2 to OpenDRIVE format.

!!! warning "Important"
    Understanding these limitations is crucial for setting realistic expectations when using this converter tool.

## Overview

The conversion from Lanelet2 to OpenDRIVE involves transforming between two fundamentally different map representation formats. Due to architectural differences between the formats and target platform constraints, some behavioral differences are expected and unavoidable.

### Quick Navigation

Jump to specific limitations:

1. [Stop Line Position Discrepancies](stop-line-position.md) — CARLA's trigger-volume-based detection causes position shifts even though the converter emits stop lines correctly
2. [Lane Width Inconsistencies](lane-width.md) — mathematical interpolation between different geometric representations
3. [Special Traffic Signals](special-signals.md) — vehicle/pedestrian traffic lights with arrow encoding plus stop-line / stop-sign / yield-sign signals are supported; everything else (variable-message signs, flashing-only beacons, railroad crossings, general traffic signs) is not
4. [Priority-Based Right-of-Way Control](priority-right-of-way.md) — converter now emits `<junction><priority>` from `right_of_way` REs, but CARLA still ignores them
5. [Geometric Approximation Limitations](geometric-approximation.md) — parametric curve fitting to discrete points
6. [Coordinate System Considerations](coordinate-system.md) — MGRS projection constraints
7. [ASAM Schema Compliance](asam-schema-compliance.md) — known QC-checker false positives from CARLA's LHT `rule` extension on top of OpenDRIVE 1.4

For a summary comparison, see the [Summary](#summary) table below.

---

## Summary

| Limitation | Severity | CARLA Source Modification Required | Workaround Available |
|------------|----------|------------------------------------|---------------------|
| [Stop line position discrepancies](stop-line-position.md) | Medium | Yes (for precise positioning) | Manual trigger volume adjustment in CARLA |
| [Lane width inconsistencies](lane-width.md) | Low | No | Validate widths with a tolerance band |
| [Special signals partially supported](special-signals.md) | Medium | No (CARLA already consumes the supported ones) | Most needs already covered; see page for unsupported types |
| [Priority emitted but ignored by CARLA](priority-right-of-way.md) | Low | Yes (CARLA-side) | None on the converter side; non-CARLA consumers already work |
| [Geometric approximation](geometric-approximation.md) | Low | No | Use higher-resolution input data |
| [MGRS projection limitations](coordinate-system.md) | Medium | No | Split large maps by grid zone |
| [ASAM schema compliance](asam-schema-compliance.md) | Low | No | Suppressed automatically by the `analyze` command |

!!! info "Feedback Welcome"
    If you encounter limitations not documented here, please [report them](https://github.com/tier4/autoware_lanelet2_to_opendrive/issues) so we can improve this documentation.

---

## Next Steps

- Return to [Usage Guide](../usage.md) for conversion instructions
- Check [API Reference](../api.md) for programmatic access
- Visit [Development Guide](../development.md) to contribute improvements
