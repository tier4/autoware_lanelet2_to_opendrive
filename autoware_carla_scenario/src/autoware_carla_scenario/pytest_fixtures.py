"""pytest fixture helpers for CARLA scenario testing."""

from __future__ import annotations

from collections.abc import Callable
from typing import Type

import pytest

from .conditions import ScenarioResult
from .scenario_base import BaseScenario, EgoConfig
from .scenario_queue import ScenarioQueue


class CarlaScenarioFixture:
    """Registers a scenario into a :class:`ScenarioQueue` and generates a fixture.

    The scenario instance is created and added to the queue **at construction
    time** (i.e. when the conftest.py module is imported, before any fixture
    executes).  This ensures that all scenarios are registered before the
    session-scoped *queue_fixture* starts the CARLA server and calls
    :meth:`~ScenarioQueue.run_all`.

    Typical usage::

        # tests/my_tests/conftest.py
        import os
        import pytest
        import carla
        from autoware_carla_scenario import (
            CarlaScenarioFixture,
            CarlaServerManager,
            EgoConfig,
            ScenarioQueue,
            SpawnTransform,
        )
        from .my_scenarios import MyScenario, AnotherScenario

        ego = EgoConfig(
            spawn_location=SpawnTransform(carla.Transform(carla.Location(x=0, y=0, z=0))),
            vehicle_type="vehicle.fuso.mitsubishi",
        )

        # 1. Create the queue (no server started yet)
        _queue = ScenarioQueue(map_name="Town10HD_Opt")

        # 2. Register scenarios at import time
        my_result     = CarlaScenarioFixture(MyScenario,     ego, queue=_queue).as_fixture()
        another_result = CarlaScenarioFixture(AnotherScenario, ego, queue=_queue).as_fixture()

        # 3. Session fixture that starts CARLA and runs every scenario once
        @pytest.fixture(scope="session")
        def carla_queue():
            if not os.environ.get(CarlaServerManager.ENV_VAR):
                pytest.skip("CARLA_UE5_EXECUTABLE not set")
            with _queue:
                _queue.run_all()
                yield _queue

        # tests/my_tests/test_my.py
        def test_passes(my_result):
            assert my_result.passed

        def test_another(another_result):
            assert another_result.passed

    The ``carla_queue`` session fixture starts the server and runs **all**
    registered scenarios exactly once.  Individual result fixtures simply
    fetch their pre-computed :class:`~autoware_carla_scenario.ScenarioResult`
    from the queue — CARLA is never restarted between tests.
    """

    def __init__(
        self,
        scenario_cls: Type[BaseScenario],
        ego_config: EgoConfig,
        queue: ScenarioQueue,
    ) -> None:
        """Register the scenario into *queue*.

        Args:
            scenario_cls: The :class:`BaseScenario` subclass to instantiate.
            ego_config: Ego vehicle spawn configuration.
            queue: The :class:`ScenarioQueue` that will execute this scenario.
                   :meth:`~ScenarioQueue.add` is called immediately so that the
                   scenario is registered before any fixture runs.
        """
        self._scenario = scenario_cls(ego_config)
        self._queue = queue
        queue.add(self._scenario)

    def as_fixture(
        self, queue_fixture: str = "carla_queue"
    ) -> Callable[..., ScenarioResult]:
        """Return a session-scoped pytest fixture for this scenario's result.

        The generated fixture depends on *queue_fixture* (default:
        ``"carla_queue"``), which must be a session-scoped fixture that starts
        the server, calls :meth:`~ScenarioQueue.run_all`, and yields the queue.
        The dependency is resolved via :meth:`pytest.FixtureRequest.getfixturevalue`
        so the queue always runs before any individual result is accessed.

        Args:
            queue_fixture: Name of the session-scoped fixture that drives the
                queue.  Override if you use a custom fixture name.

        Returns:
            A function decorated with ``@pytest.fixture(scope="session")``.
        """
        scenario = self._scenario
        queue = self._queue

        @pytest.fixture(scope="session")
        def _fixture(request: pytest.FixtureRequest) -> ScenarioResult:
            # Trigger the queue fixture (starts server + run_all) if not yet done.
            request.getfixturevalue(queue_fixture)
            return queue.result_for(scenario)

        return _fixture
