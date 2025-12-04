# Product Overview

## Product Purpose

This tool converts Lanelet2 map format to OpenDRIVE format for use with Autoware and other autonomous driving software stacks. It bridges the gap between two major HD map standards, enabling seamless integration of map data across different autonomous driving platforms and simulation environments.

## Target Users

The primary users of this product are:

* **Autoware developers** who need to work with OpenDRIVE-compatible tools and simulators
* **Autonomous vehicle engineers** requiring map format conversion for testing and validation
* **Simulation engineers** who need to import Lanelet2 maps into OpenDRIVE-based simulators
* **Map data providers** who need to deliver maps in lanelet2 formats

## Key Features

1. **Format Conversion**: Accurate conversion from Lanelet2 (.osm) to OpenDRIVE (.xodr) format
2. **Coordinate System Support**: MGRS grid code support for proper coordinate transformation
3. **Geometry Preservation**: Maintains road geometry, lane connections, and regulatory elements
4. **Command-line Interface**: Simple CLI for batch processing and integration into pipelines
5. **Type Safety**: Full type hints and validation for reliable conversions

## Business Objectives

* Enable interoperability between Lanelet2-based systems and OpenDRIVE-based tools
* Reduce manual effort in map format conversion workflows
* Support Autoware's ecosystem growth by providing essential tooling
* Facilitate testing and validation across multiple simulation platforms

## Success Metrics

* **Conversion Accuracy**: 100% preservation of essential map elements
* **Performance**: Convert typical urban maps (\< 10MB) in under 30 seconds
* **Reliability**: Zero data loss for supported Lanelet2 elements
* **Adoption**: Integration into Autoware toolchain and documentation
* **Coverage**: Support for all core Lanelet2 primitives and regulatory elements

## Product Principles

1. **Data Integrity First**: Never lose or corrupt map data during conversion
2. **Standards Compliance**: Strictly adhere to both Lanelet2 and OpenDRIVE specifications
3. **Developer Experience**: Provide clear error messages and debugging information
4. **Open Source Excellence**: Maintain high code quality for community contributions

## Monitoring & Visibility

* **Progress Reporting**: CLI progress indicators for large file conversions
* **Validation Output**: Detailed reports on converted elements and any warnings
* **Error Handling**: Clear error messages with actionable remediation steps
* **Logging**: Configurable logging levels for debugging conversion issues

## Future Vision

### Near-term Enhancements

* **Validation Suite**: Comprehensive validation of converted maps
* **CARLA Support**: Convert to OpenDRIVE 1.4 Map for CARLA.&#x20;

### Long-term Goals

* **Integration Platform**: Direct integration with major simulation platforms
