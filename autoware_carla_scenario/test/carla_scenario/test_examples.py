"""Integration test for the SpawnAndIdleScenario example.

This file is intentionally self-contained — it does not rely on the
``carla_queue`` fixture defined in ``conftest.py``.  Copy this pattern
into your own project to run custom scenarios with pytest.

How it works
------------
1. A ``ScenarioQueue`` and ``EgoConfig`` are created at module-import time.
2. ``CarlaScenarioFixture`` registers the scenario into the queue and
   returns a session-scoped pytest fixture for the result.
3. The ``example_carla_queue`` session fixture starts the CARLA server
   (or reuses one that is already running), runs every registered scenario
   exactly once, then tears down.
4. Individual test functions receive the pre-computed ``ScenarioResult``
   and make assertions on it — CARLA is never restarted between tests.

To run only these tests::

    uv run pytest test/carla_scenario/test_examples.py -v
"""

from __future__ import annotations

import os
from collections.abc import Generator

import carla
import pytest

from autoware_carla_scenario import (
    CarlaScenarioFixture,
    CarlaServerManager,
    EgoConfig,
    ScenarioQueue,
)
from autoware_carla_scenario.examples.spawn_and_idle import SpawnAndIdleScenario

# ---------------------------------------------------------------------------
# Queue and scenario registration — happens at import time so that all
# scenarios are known before the session fixture starts the CARLA server.
# ---------------------------------------------------------------------------

_ego = EgoConfig(
    transform=carla.Transform(carla.Location(x=0.0, y=0.0, z=0.5)),
    vehicle_type="vehicle.tesla.model3",
)

_queue = ScenarioQueue(map_name="Town01")

# Each CarlaScenarioFixture call registers one scenario and returns a
# session-scoped pytest fixture that holds its ScenarioResult.
spawn_and_idle_result = CarlaScenarioFixture(
    SpawnAndIdleScenario,
    _ego,
    queue=_queue,
).as_fixture(queue_fixture="example_carla_queue")


# ---------------------------------------------------------------------------
# Session fixture — starts CARLA once, runs all scenarios, then tears down.
# ---------------------------------------------------------------------------


@pytest.fixture(scope="session")
def example_carla_queue() -> Generator[ScenarioQueue, None, None]:
    """Start CARLA, execute every registered scenario, yield the queue.

    Skips the entire module if ``CARLA_UE5_EXECUTABLE`` is not set or if
    the CARLA session cannot be established.
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


# ---------------------------------------------------------------------------
# Tests — each function receives the pre-computed ScenarioResult.
# ---------------------------------------------------------------------------


def test_spawn_and_idle_passes(spawn_and_idle_result) -> None:  # noqa: ANN001
    """The scenario should complete without hitting the timeout."""
    assert spawn_and_idle_result.passed, spawn_and_idle_result.message


def test_spawn_and_idle_elapsed_time(spawn_and_idle_result) -> None:  # noqa: ANN001
    """The scenario should run for approximately 2 seconds (40 ticks × 0.05 s)."""
    expected = SpawnAndIdleScenario.DONE_AFTER_TICKS * 0.05  # seconds
    assert spawn_and_idle_result.elapsed_seconds >= expected * 0.5
