# Known Limitations

This page documents implementation limitations and behavioral differences when converting from Lanelet2 to OpenDRIVE format.

!!! warning "Important"
    Understanding these limitations is crucial for setting realistic expectations when using this converter tool.

## Overview

The conversion from Lanelet2 to OpenDRIVE involves transforming between two fundamentally different map representation formats. Due to architectural differences between the formats and target platform constraints, some behavioral differences are expected and unavoidable.

### Quick Navigation

Jump to specific limitations:

1. [Stop Line Position Discrepancies](stop-line-position.md) - CARLA trigger volume-based detection causes position shifts
2. [Lane Width Inconsistencies](lane-width.md) - Mathematical interpolation between different geometric representations
3. [Special Traffic Signals Not Supported](special-signals.md) - Arrow signals, pedestrian signals, and directional signals
4. [Priority-Based Right-of-Way Control Not Supported](priority-right-of-way.md) - CARLA does not support priority attributes
5. [Geometric Approximation Limitations](geometric-approximation.md) - Parametric curve fitting to discrete points
6. [Coordinate System Considerations](coordinate-system.md) - MGRS projection constraints
7. [ASAM Schema Compliance](asam-schema-compliance.md) - Known QC checker false positives due to CARLA's LHT extension on OpenDRIVE 1.4

For a summary comparison, see the [Summary](#summary) table below.

---

## Summary

| Limitation | Severity | CARLA Source Modification Required | Workaround Available |
|------------|----------|------------------------------------|---------------------|
| [Stop line position discrepancies](stop-line-position.md) | Medium | Yes (for precise positioning) | Manual trigger volume adjustment |
| [Lane width inconsistencies](lane-width.md) | Low | No | Validation with tolerances |
| [Special signals not supported](special-signals.md) | High | Yes (for full support) | Community contribution needed |
| [Priority-based right-of-way not supported](priority-right-of-way.md) | Low | Yes | CARLA source modification only |
| [Geometric approximation](geometric-approximation.md) | Low | No | Use high-resolution input data |
| [MGRS projection limitations](coordinate-system.md) | Medium | No | Split large maps by grid zone |
| [ASAM schema compliance](asam-schema-compliance.md) | Low | No | Suppressed automatically by `analyze` command |

!!! info "Feedback Welcome"
    If you encounter limitations not documented here, please [report them](https://github.com/tier4/autoware_lanelet2_to_opendrive/issues) so we can improve this documentation.

---

## Next Steps

- Return to [Usage Guide](../usage.md) for conversion instructions
- Check [API Reference](../api.md) for programmatic access
- Visit [Development Guide](../development.md) to contribute improvements
