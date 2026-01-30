"""Pytest configuration and fixtures."""

from pathlib import Path

import pytest

# Import autoware extensions before any tests to ensure proper registration
from autoware_lanelet2_extension_python.projection import MGRSProjector  # noqa: F401
import lanelet2  # noqa: F401

# The above imports ensure that Autoware lanelet2 extensions are registered
# before any test modules are loaded. This prevents issues with missing
# regulatory elements like 'road_marking' and 'detection_area'.


@pytest.fixture(scope="session")
def lanelet_map():
    """Load test map once and cache for entire test session.

    This fixture loads the large lanelet2_map.osm file (11MB, 307k lines) once
    per test session and reuses it across all tests. This significantly reduces
    test execution time by avoiding repeated file I/O and parsing.

    Returns:
        lanelet2.core.LaneletMap: The loaded lanelet2 map.
    """
    test_data_path = Path(__file__).parent / "data" / "lanelet2_map.osm"
    projector = MGRSProjector(
        lanelet2.io.Origin(35.23, 139.16)
    )  # MGRS origin for Tokyo area (54SUE)
    return lanelet2.io.load(str(test_data_path), projector)
