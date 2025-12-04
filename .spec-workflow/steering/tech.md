# Technology Stack

## Project Type

Command-line tool and Python library for converting HD map formats, specifically from Lanelet2 to OpenDRIVE format, designed for autonomous driving applications.

## Core Technologies

### Primary Language(s)

* **Language**: Python 3.10+
* **Runtime**: CPython interpreter
* **Language-specific tools**: `uv` package manager (version 0.9.7+), pip for fallback

### Key Dependencies/Libraries

* **lanelet2** (>\=1.2.2): Core library for reading and manipulating Lanelet2 map data
* **lanelet2-python-api-for-autoware**: TIER IV's custom Python bindings for Autoware-specific Lanelet2 functionality
* **scipy** (>\=1.9.0): Scientific computing library for mathematical operations and interpolation
* **splines** (>\=0.3.3): Spline interpolation for smooth road geometry generation
* **scenariogeneration** (>\=0.16.3): OpenDRIVE generation and manipulation library
* **lxml** (>\=6.0.2): XML processing for OpenDRIVE file generation
* **mgrs** (>\=1.5.0): Military Grid Reference System coordinate conversion

### Application Architecture

Modular converter architecture with:

* **Input layer**: Lanelet2 OSM file parser
* **Transformation core**: Geometry conversion, coordinate transformation, element mapping
* **Output layer**: OpenDRIVE XML generation
* **Utility modules**: Shared functions for geometry calculations and data structures

### Data Storage

* **Primary storage**: File-based I/O (no database required)
* **Input formats**: Lanelet2 OSM (XML-based)
* **Output formats**: OpenDRIVE XODR (XML format)
* **Intermediate representation**: Python data structures (dataclasses)

### External Integrations

* **Map Standards**:
  * Lanelet2 v1.2+ specification
  * OpenDRIVE 1.4/1.6 specification
* **Coordinate Systems**: WGS84, MGRS, local Cartesian
* **Autoware Ecosystem**: Compatible with Autoware.Universe toolchain

### Monitoring & Dashboard Technologies

Not applicable - Command-line tool without dashboard requirements. Progress reporting via terminal output.

## Development Environment

### Build & Development Tools

* **Build System**: `uv` build backend
* **Package Management**: `uv` for dependencies, lock file management
* **Development workflow**: Local development with `uv pip install -e .`
* **Project configuration**: pyproject.toml (PEP 621 compliant)

### Code Quality Tools

* **Static Analysis**:
  * `mypy` (>\=1.19.0): Type checking
  * `ruff`: Fast Python linter
* **Formatting**:
  * `black` (>\=25.11.0): Code formatting
  * `pre-commit`: Git hooks for automated checks
* **Testing Framework**:
  * `pytest` (>\=9.0.1): Unit and integration testing
  * Test data in `test/data/` directory
* **Documentation**:
  * Type hints with `py.typed` marker
  * Docstrings following Google style

### Version Control & Collaboration

* **VCS**: Git with GitHub hosting
* **Branching Strategy**: Feature branches with PR-based workflow
* **Code Review Process**: GitHub Pull Requests with CI checks
* **CI/CD**: GitHub Actions for automated testing and validation

### Dashboard Development

Not applicable - CLI tool without dashboard.

## Deployment & Distribution

* **Target Platform(s)**: Linux, macOS, Windows (Python 3.10+ required)
* **Distribution Method**:
  * PyPI package (future)
  * Git installation via `uv`
  * Docker container (planned)
* **Installation Requirements**:
  * Python 3.10 or higher
  * GEOS library for spatial operations
  * C++ compiler for lanelet2 dependencies
* **Update Mechanism**: Package manager updates (uv/pip)

## Technical Requirements & Constraints

### Performance Requirements

* Process maps up to 10MB in under 30 seconds
* Memory usage proportional to map size (typical \< 500MB)
* Support for maps with 10,000+ lane elements
* Efficient coordinate transformation for large point sets

### Compatibility Requirements

* **Platform Support**: Linux
* **Python Versions**: 3.10
* **Lanelet2 Format**: Version 1.2.0 and higher
* **OpenDRIVE Output**: Version 1.4 (CARLA compatible)

### Security & Compliance

* **Security Requirements**:
  * Input validation for untrusted map files
  * No network access required
  * File system permissions respected
* **Data Protection**: No sensitive data handling
* **Open Source License**: Apache 2.0 (Autoware standard)

### Scalability & Reliability

* **Expected Load**: Single-user CLI tool, batch processing supported
* **Memory Model**: Streaming processing for large files
* **Error Handling**: Graceful degradation with detailed error messages
* **Validation**: Pre and post-conversion validation checks

## Technical Decisions & Rationale

### Decision Log

1. **Python over C++**: Chose Python for rapid development and ease of contribution, despite C++ being common in Autoware. Python bindings for lanelet2 provide sufficient performance.
2. **uv Package Manager**: Modern, fast Python package manager with lock file support, replacing traditional pip/poetry/pipenv for better reproducibility.
3. **scenariogeneration Library**: Leverages existing OpenDRIVE generation capabilities rather than implementing XML generation from scratch.
4. **Dataclass-based Architecture**: Using Python dataclasses for type safety and clean data structures throughout the conversion pipeline.
5. **MGRS Coordinate System**: Supporting MGRS as primary coordinate input due to its use in Japanese mapping systems and Autoware deployments.
6. **Modular Design**: Separate modules for geometry, lanes, and road elements to enable incremental feature addition and testing.

## Known Limitations

* **Partial Element Support**: Currently supports core road geometry and lanes; traffic signs, signals, and objects pending implementation
* **One-way Conversion**: Lanelet2 to OpenDRIVE only; reverse conversion not yet supported
* **Coordinate Precision**: Some coordinate transformations may introduce minor precision loss (\< 1cm)
* **Performance on Large Maps**: Maps over 50MB may require optimization for memory usage
* **Limited Validation**: Post-conversion validation against OpenDRIVE schema not yet comprehensive
* **Junction Complexity**: Complex intersection geometries may require manual adjustment
