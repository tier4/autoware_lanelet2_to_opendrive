# Project Structure

## Directory Organization

```
autoware_lanelet2_to_opendrive/
├── src/                                # Source code root
│   └── autoware_lanelet2_to_opendrive/ # Main package directory
│       ├── __init__.py                 # Package initialization
│       ├── main.py                     # CLI entry point
│       ├── centerline.py               # Centerline calculation utilities
│       ├── geometry.py                 # Core geometry operations
│       ├── junction.py                 # Junction/intersection handling
│       ├── util.py                     # Shared utility functions
│       ├── py.typed                    # PEP 561 type hint marker
│       └── opendrive/                  # OpenDRIVE format modules
│           ├── __init__.py            # Subpackage initialization
│           ├── opendrive.py           # Main OpenDRIVE generator
│           ├── opendrive_dataclass.py # OpenDRIVE data structures
│           ├── road.py                # Road element generation
│           ├── reference_line.py     # Reference line calculations
│           ├── geometry.py            # OpenDRIVE geometry primitives
│           ├── lane_section.py       # Lane section handling
│           ├── lane_sections.py      # Multiple lane sections
│           ├── lane.py                # Individual lane properties
│           ├── lane_elements.py      # Lane-specific elements
│           ├── elevation.py           # Elevation profile handling
│           ├── header.py              # OpenDRIVE header generation
│           └── enums.py               # OpenDRIVE enumerations
├── test/                              # Test directory
│   ├── conftest.py                   # PyTest configuration
│   ├── data/                          # Test data files
│   │   ├── nishisinjyuku.osm        # Sample Lanelet2 input
│   │   └── lanelet2_map.xodr        # Expected OpenDRIVE output
│   ├── test_util.py                  # Utility function tests
│   ├── test_geometry.py              # Geometry module tests
│   ├── test_centerline.py            # Centerline calculation tests
│   ├── test_junction.py              # Junction handling tests
│   ├── test_opendrive_road.py       # Road generation tests
│   ├── test_opendrive_lane.py       # Lane generation tests
│   ├── test_opendrive_lane_section.py # Lane section tests
│   └── test_opendrive_reference_line.py # Reference line tests
├── .spec-workflow/                    # Spec-driven development workspace
│   ├── steering/                     # Project steering documents
│   │   ├── product.md               # Product vision
│   │   ├── tech.md                  # Technical architecture
│   │   └── structure.md             # This document
│   └── templates/                    # Document templates
├── pyproject.toml                     # Project configuration (PEP 621)
├── uv.lock                           # Dependency lock file
├── README.md                         # Project documentation
└── CLAUDE.md                         # Claude Code guidance
```

## Naming Conventions

### Files
- **Python modules**: `snake_case.py` (e.g., `reference_line.py`, `lane_section.py`)
- **Test files**: `test_[module_name].py` (e.g., `test_geometry.py`)
- **Data files**: Descriptive names with format extension (e.g., `nishisinjyuku.osm`)
- **Configuration**: Standard names (`pyproject.toml`, `uv.lock`)

### Code
- **Classes**: `PascalCase` (e.g., `OpenDrive`, `RoadGeometry`, `LaneSection`)
- **Functions/Methods**: `snake_case` (e.g., `convert_to_opendrive`, `calculate_centerline`)
- **Constants**: `UPPER_SNAKE_CASE` (e.g., `DEFAULT_LANE_WIDTH`, `MAX_ITERATIONS`)
- **Variables**: `snake_case` (e.g., `road_id`, `lane_sections`)
- **Type aliases**: `PascalCase` (e.g., `Point3D`, `LaneletMap`)
- **Enum values**: `UPPER_SNAKE_CASE` or `PascalCase` depending on context

## Import Patterns

### Import Order
1. Standard library imports (`import os`, `import sys`, `from typing import ...`)
2. Third-party imports (`import lanelet2`, `import numpy`, `from scipy import ...`)
3. Local package imports (`from autoware_lanelet2_to_opendrive import ...`)
4. Relative imports within submodules (`from .geometry import ...`, `from .enums import ...`)

### Module/Package Organization
- **Absolute imports** for main package: `from autoware_lanelet2_to_opendrive.util import ...`
- **Relative imports** within subpackages: `from .lane_section import LaneSection`
- **Explicit imports** preferred over wildcard: `from .enums import LaneType, RoadType`
- **Type imports** separated when needed: `from typing import TYPE_CHECKING`

## Code Structure Patterns

### Module/Class Organization
```python
# 1. Module docstring
"""Module description and purpose."""

# 2. Future imports (if needed)
from __future__ import annotations

# 3. Standard library imports
import math
from typing import List, Optional

# 4. Third-party imports
import numpy as np
from lanelet2 import core

# 5. Local imports
from .geometry import Point3D
from .enums import LaneType

# 6. Constants and configuration
DEFAULT_LANE_WIDTH = 3.5
EPSILON = 1e-6

# 7. Type definitions and dataclasses
@dataclass
class RoadElement:
    ...

# 8. Main classes
class OpenDriveConverter:
    ...

# 9. Utility functions
def calculate_distance(p1: Point3D, p2: Point3D) -> float:
    ...

# 10. Module-level execution (if __name__ == "__main__")
```

### Function/Method Organization
```python
def convert_lanelet_to_road(lanelet: Lanelet) -> Road:
    """Convert a Lanelet2 lanelet to an OpenDRIVE road.

    Args:
        lanelet: Input Lanelet2 lanelet object

    Returns:
        Converted OpenDRIVE road object

    Raises:
        ValueError: If lanelet geometry is invalid
    """
    # 1. Input validation
    if not lanelet.centerline:
        raise ValueError("Lanelet missing centerline")

    # 2. Data preparation
    points = extract_points(lanelet)

    # 3. Core logic
    road = Road()
    road.geometry = calculate_geometry(points)
    road.lanes = convert_lanes(lanelet)

    # 4. Post-processing
    validate_road(road)

    # 5. Return result
    return road
```

### File Organization Principles
- **Single responsibility**: Each module handles one aspect (e.g., `geometry.py` for geometry operations)
- **Clear interfaces**: Public API functions at module level, internal helpers prefixed with underscore
- **Dataclass separation**: Complex data structures in dedicated files (e.g., `opendrive_dataclass.py`)
- **Subpackage isolation**: OpenDRIVE-specific code contained in `opendrive/` subpackage

## Code Organization Principles

1. **Single Responsibility**: Each module has one clear purpose
   - `geometry.py`: Geometric calculations only
   - `junction.py`: Junction/intersection logic only
   - `main.py`: CLI interface only

2. **Modularity**: Reusable components across the conversion pipeline
   - Utility functions in `util.py` shared across modules
   - OpenDRIVE primitives in subpackage for composition

3. **Testability**: Structure supports comprehensive testing
   - Each module has corresponding test file
   - Test data in dedicated directory
   - Fixtures in `conftest.py`

4. **Consistency**: Uniform patterns throughout codebase
   - All modules follow same import ordering
   - Consistent error handling patterns
   - Standardized docstring format

## Module Boundaries

### Core vs Format-Specific
- **Core modules** (`geometry.py`, `util.py`): Format-agnostic operations
- **Format modules** (`opendrive/`): OpenDRIVE-specific implementation
- **Conversion layer** (`main.py`): Orchestrates between formats

### Public API vs Internal
- **Public API**: `convert` command, main conversion functions
- **Internal utilities**: Helper functions prefixed with underscore
- **Package exports**: Controlled through `__init__.py`

### Dependencies Direction
```
main.py
   ↓
[centerline.py, geometry.py, junction.py, util.py]
   ↓
opendrive/
   ├── opendrive.py (orchestrator)
   └── [individual element modules]
```

## Code Size Guidelines

### File Size
- **Maximum lines per file**: 500 lines (excluding tests)
- **Preferred range**: 100-300 lines for maintainability
- **Split criteria**: When file handles multiple responsibilities

### Function/Method Size
- **Maximum lines per function**: 50 lines
- **Preferred range**: 10-30 lines
- **Complexity limit**: Cyclomatic complexity < 10

### Class/Module Complexity
- **Methods per class**: Maximum 20 public methods
- **Nesting depth**: Maximum 4 levels of indentation
- **Import limit**: Maximum 15 imports per module

## Documentation Standards

### Module Documentation
- Every module must have a docstring explaining its purpose
- Complex algorithms should include implementation notes
- External references (e.g., OpenDRIVE spec) should be cited

### Function/Method Documentation
```python
def function_name(param1: Type1, param2: Type2) -> ReturnType:
    """Brief description of function purpose.

    Longer description if needed, explaining algorithm or approach.

    Args:
        param1: Description of first parameter
        param2: Description of second parameter

    Returns:
        Description of return value

    Raises:
        ExceptionType: When this exception occurs

    Example:
        >>> result = function_name(value1, value2)
    """
```

### Type Hints
- All public functions must have complete type hints
- Use `Optional[T]` for nullable types
- Use `Union[T1, T2]` sparingly (prefer single types)
- Import from `typing` for complex types

### Inline Comments
- Explain "why" not "what" (code should be self-documenting)
- Complex mathematical operations should include formulas
- Reference external specifications where applicable

## Testing Structure

### Test Organization
- **Unit tests**: Test individual functions/methods in isolation
- **Integration tests**: Test module interactions
- **Data-driven tests**: Use test data files for realistic scenarios
- **Parametrized tests**: Use pytest.mark.parametrize for multiple cases

### Test Naming
```python
def test_[function_name]_[scenario]_[expected_outcome]():
    """Test that [function] correctly handles [scenario]."""
```

### Test Coverage
- Minimum 80% code coverage target
- Critical path coverage: 100%
- Edge cases and error conditions must be tested
