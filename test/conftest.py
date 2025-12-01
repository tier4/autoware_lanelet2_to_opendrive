"""Pytest configuration and fixtures."""

# Import autoware extensions before any tests to ensure proper registration
from autoware_lanelet2_extension_python.projection import MGRSProjector  # noqa: F401
import lanelet2  # noqa: F401

# The above imports ensure that Autoware lanelet2 extensions are registered
# before any test modules are loaded. This prevents issues with missing
# regulatory elements like 'road_marking' and 'detection_area'.
