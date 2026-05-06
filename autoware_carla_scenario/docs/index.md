# Autoware CARLA Scenario

Welcome to the documentation for `autoware-carla-scenario`, a CARLA scenario testing framework for Autoware autonomous driving software.

## Overview

This package provides a framework for creating and running automated scenario tests in the [CARLA simulator](https://carla.org/), designed for validating Autoware's autonomous driving capabilities.

## Features

- Automated scenario execution in CARLA simulator (UE5 / `0.10.0` and legacy `0.9.16`)
- Configurable scenarios using Hydra `compose` + structured configs
- Glob-pattern batch execution of multiple scenarios in a single CARLA session
- Condition-based scenario evaluation: timing, collisions, traffic signals,
  speed / standstill, lane / area position, waypoint crossing, plus
  logical (`And` / `Or` / `Not`), latching (`Sticky`), and persistent combinators
- Action system for side effects: turns, lane changes, traffic-light state,
  on-demand camera attachment
- Entity management for ego (`EgoVehicle`) and NPC vehicles with spawn
  retry and ground projection
- Coordinate transforms between Lanelet2, OpenDRIVE, and CARLA world
- Lanelet-based constraint sweeping via a Hydra `lanelet_constraint`
  sweeper plugin (also resolvable without CARLA from the viewer)
- Web UI (FastAPI + Uvicorn) for browsing results, replaying videos,
  and triggering runs
- Two-pass video recording (CARLA native log + replayed RGB camera +
  ffmpeg H.264)
- pytest integration via `CarlaScenarioFixture` (auto-skip when
  `CARLA_EXECUTABLE` is unset)

## Quick Links

- [Installation Guide](installation.md) - Get started with installing the package
- [Usage Guide](usage.md) - Learn how to run scenarios
- [Architecture](architecture.md) - Software architecture and design decisions
- [API Reference](api.md) - Detailed API documentation
- [Development Guide](development.md) - Contributing and development setup

## Project Information

- **Repository**: [tier4/autoware_lanelet2_to_opendrive](https://github.com/tier4/autoware_lanelet2_to_opendrive)
- **Release Notes**: [View all releases on GitHub](https://github.com/tier4/autoware_lanelet2_to_opendrive/releases)
- **License**: Check the repository for license information
- **Python Version**: 3.10 required (`>=3.10,<3.11`, locked by CARLA's bindings)

## Getting Help

If you encounter any issues or have questions:

1. Check the documentation sections above
2. Search existing [GitHub Issues](https://github.com/tier4/autoware_lanelet2_to_opendrive/issues)
3. Create a new issue if your problem hasn't been reported
