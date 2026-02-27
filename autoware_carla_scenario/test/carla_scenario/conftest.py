"""Session-scoped fixtures for CARLA integration tests.

Architecture
------------
All scenarios are registered into ``_queue`` at module import time (before
any fixture runs).  The single session-scoped ``carla_queue`` fixture starts
the CARLA server once, runs every registered scenario via
:meth:`ScenarioQueue.run_all`, and yields the queue.  Individual test fixtures
then simply call :meth:`ScenarioQueue.result_for` — CARLA is never restarted
between test cases.

Skipping
--------
If ``CARLA_UE5_EXECUTABLE`` is not set the entire session is skipped.
If the map cannot be loaded the session is skipped as well.
"""

from __future__ import annotations

import os
from collections.abc import Generator

import pytest

import carla as _carla

from autoware_carla_scenario import (
    BaseScenario,
    CarlaScenarioFixture,
    CarlaServerManager,
    EgoConfig,
    ScenarioQueue,
)


# ---------------------------------------------------------------------------
# Minimal scenarios used by integration tests
# ---------------------------------------------------------------------------


class _ImmediateScenario(BaseScenario):
    """Scenario that completes immediately (is_done returns True on first call)."""

    def setup(self, world: object) -> None:  # type: ignore[override]
        pass

    def is_done(self) -> bool:
        return True


# ---------------------------------------------------------------------------
# Queue – populated at import time so all scenarios are registered before
# the session fixture starts the CARLA server.
# ---------------------------------------------------------------------------

_INTEGRATION_EGO = EgoConfig(
    transform=_carla.Transform(_carla.Location(x=0.0, y=0.0, z=0.0)),
    vehicle_type="vehicle.tesla.model3",
)

_queue = ScenarioQueue(map_name="Town01")

# Register integration scenarios.  Add more CarlaScenarioFixture calls here
# as needed; they will all run in the same CARLA session.
immediate_scenario_result = CarlaScenarioFixture(
    _ImmediateScenario,
    _INTEGRATION_EGO,
    queue=_queue,
).as_fixture()

another_immediate_result = CarlaScenarioFixture(
    _ImmediateScenario,
    _INTEGRATION_EGO,
    queue=_queue,
).as_fixture()


# ---------------------------------------------------------------------------
# Session fixture – starts server and runs all registered scenarios once
# ---------------------------------------------------------------------------


@pytest.fixture(scope="session")
def carla_queue() -> Generator[ScenarioQueue, None, None]:
    """Start CARLA, run every registered scenario, yield the queue.

    Skips if ``CARLA_UE5_EXECUTABLE`` is not set or if the map fails to load.
    """
    if not os.environ.get(CarlaServerManager.ENV_VAR):
        pytest.skip(
            f"Environment variable '{CarlaServerManager.ENV_VAR}' is not set. "
            "Skipping CARLA integration tests."
        )
    try:
        with _queue:
            _queue.run_all()
            yield _queue
    except Exception as exc:
        pytest.skip(f"CARLA session failed: {exc}")
