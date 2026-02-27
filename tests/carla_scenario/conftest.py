"""Session-scoped fixtures for CARLA integration tests.

Tests in this directory require a running CARLA server.  If the
``CARLA_UE5_EXECUTABLE`` environment variable is not set the entire
test session is skipped automatically.
"""

from __future__ import annotations

import os
from collections.abc import Generator

import pytest

from autoware_carla_scenario import CarlaAutowareScenario, CarlaServerManager


@pytest.fixture(scope="session")
def carla_server() -> Generator[CarlaServerManager, None, None]:
    """Start a CARLA server for the full test session.

    Skips if ``CARLA_UE5_EXECUTABLE`` is not configured.
    """
    if not os.environ.get(CarlaServerManager.ENV_VAR):
        pytest.skip(
            f"Environment variable '{CarlaServerManager.ENV_VAR}' is not set. "
            "Skipping CARLA integration tests."
        )
    with CarlaServerManager() as server:
        yield server


@pytest.fixture(scope="session")
def carla_runner(carla_server: CarlaServerManager) -> CarlaAutowareScenario:
    """Provide a :class:`CarlaAutowareScenario` connected to the session server."""
    runner = CarlaAutowareScenario(carla_server)
    try:
        runner.load_map_by_name("Town01")
    except Exception as exc:
        pytest.skip(f"Failed to load CARLA map 'Town01': {exc}")
    return runner
