# Autoware Lanelet2 to OpenDRIVE

Welcome to the documentation for `autoware-lanelet2-to-opendrive`, a Python package for converting Lanelet2 map format to OpenDRIVE format.

## Overview

This package provides tools to convert Lanelet2 maps, commonly used in Autoware and other autonomous driving platforms, into the OpenDRIVE format, an open standard for road network descriptions.

## Features

- Lanelet2 → OpenDRIVE 1.4 conversion targeted at CARLA (with a `target=carla` overlay)
- Hydra-based CLI (`convert`) composable from `conf/config.yaml`, `conf/map/*.yaml`, and `conf/target/*.yaml`
- Optional Lanelet2 preprocessing pipeline (merge / replace / remove / move-point / delete-point / remove-turn-direction / validate) configurable from the same YAML
- Reference-line geometry emitted as `<paramPoly3>` chains, optionally classified into `<line>` / `<arc>` / `<paramPoly3>` runs (`arcspiral.enabled`)
- Crosswalk lanelets (`subtype="crosswalk"`) emitted as `<object type="crosswalk">` with closed outlines
- Stop-line linestrings (`type="stop_line"`) emitted as `<object type="stopLine">` (or CARLA `Stencil_STOP`) with optional `<signal type="294">` plus dependencies on associated traffic-light, stop-sign and yield-sign signals
- Parking-lot `Area`s emitted as synthetic parking roads with `<lane type="parking">` and `<object type="parkingSpace">` per stall
- Traffic-light extraction from Autoware regulatory elements with arrow-bulb subtype encoding (see [Signals](signals.md))
- Right-of-way regulatory elements emitted as `<junction><priority>` records (`high`/`low` road IDs)
- ASAM QC validation and Lanelet2-to-road geometric cross-validation built in (`analyze`, `qc-validate`)
- CARLA import smoke test (`carla-import-test`)
- Pure Python 3.10 with full type hints (`py.typed`); managed by `uv`

## Quick Links

- [Installation Guide](installation.md) - Get started with installing the package
- [Usage Guide](usage.md) - Learn how to use the converter
- [CARLA OpenDRIVE Mapping](carla_opendrive_lanelet2_mapping.md) - OpenDRIVE tags used by CARLA and Lanelet2 conversion mapping
- [Conversion Process](conversion-process.md) - Detailed conversion pipeline and tag usage
- [Known Limitations](limitations/index.md) - Important limitations and behavioral differences
- [Signals](signals.md) - Signal handling documentation
- [Crosswalk Objects](crosswalk_objects.md) - Crosswalk lanelet to OpenDRIVE objects conversion
- [Stop Line Objects](stop_line_objects.md) - Stop line linestring to OpenDRIVE objects conversion
- [API Reference](api.md) - Detailed API documentation
- [Development Guide](development.md) - Contributing and development setup

## Project Information

- **Repository**: [tier4/autoware_lanelet2_to_opendrive](https://github.com/tier4/autoware_lanelet2_to_opendrive)
- **Release Notes**: [View all releases on GitHub](https://github.com/tier4/autoware_lanelet2_to_opendrive/releases)
- **License**: Check the repository for license information
- **Python Version**: exactly 3.10 (pinned to `>=3.10,<3.11` because `lanelet2-python-api-for-autoware` ships a CPython-3.10 ABI-tagged wheel)

## Getting Help

If you encounter any issues or have questions:

1. Check the documentation sections above
2. Search existing [GitHub Issues](https://github.com/tier4/autoware_lanelet2_to_opendrive/issues)
3. Create a new issue if your problem hasn't been reported

## About Lanelet2 and OpenDRIVE

**Lanelet2** is a map format designed for autonomous driving, focusing on lane-level topology and traffic rules. It's widely used in the Autoware platform.

**OpenDRIVE** is an open standard for road network descriptions, providing a detailed representation of road geometry, lane topology, and traffic infrastructure.

This converter bridges these two formats, enabling interoperability between different autonomous driving tools and simulation environments.
