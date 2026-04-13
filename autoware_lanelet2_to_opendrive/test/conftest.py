"""Pytest configuration and fixtures."""

from pathlib import Path

import pytest

# Import autoware extensions before any tests to ensure proper registration.
# Both projection AND regulatory_elements must be imported so that the
# lanelet2 C++ factory recognises custom types (AutowareTrafficLight,
# RoadMarking, DetectionArea, etc.) when loading maps.
from autoware_lanelet2_extension_python.projection import MGRSProjector  # noqa: F401
import autoware_lanelet2_extension_python.regulatory_elements as _ll2_ext_reg  # noqa: F401
import lanelet2  # noqa: F401


@pytest.fixture(scope="session")
def lanelet_map():
    """Load test map once and cache for entire test session.

    This fixture loads the large nishishinjuku.osm file (11MB, 307k lines) once
    per test session and reuses it across all tests. This significantly reduces
    test execution time by avoiding repeated file I/O and parsing.

    Returns:
        lanelet2.core.LaneletMap: The loaded lanelet2 map.
    """
    test_data_path = Path(__file__).parent / "data" / "nishishinjuku.osm"
    projector = MGRSProjector(
        lanelet2.io.Origin(35.23, 139.16)
    )  # MGRS origin for Tokyo area (54SUE)
    return lanelet2.io.load(str(test_data_path), projector)
