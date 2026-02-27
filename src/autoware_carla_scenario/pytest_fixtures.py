"""pytest fixture helpers for CARLA scenario testing."""

from __future__ import annotations

from collections.abc import Callable, Generator
from pathlib import Path
from typing import Optional, Type

import pytest

from .carla_autoware_scenario import CarlaAutowareScenario
from .conditions import ScenarioResult
from .scenario_base import BaseScenario, EgoConfig
from .server import CarlaServerManager


class CarlaScenarioFixture:
    """Helper that wraps a scenario class into a reusable pytest fixture.

    Example::

        # conftest.py
        from autoware_carla_scenario import CarlaScenarioFixture, EgoConfig
        import carla

        ego = EgoConfig(
            transform=carla.Transform(carla.Location(x=0, y=0, z=0)),
            vehicle_type="vehicle.tesla.model3",
        )
        my_result = CarlaScenarioFixture(
            MyScenario, ego, map_name="Town01"
        ).as_fixture()

        # test_my_scenario.py
        def test_passes(my_result):
            assert my_result.passed
    """

    def __init__(
        self,
        scenario_cls: Type[BaseScenario],
        ego_config: EgoConfig,
        xodr_path: Optional[Path] = None,
        map_name: Optional[str] = None,
        timeout_seconds: float = 60.0,
        output_dir: Path = Path("scenario_outputs"),
    ) -> None:
        """Create the fixture helper.

        Either *xodr_path* or *map_name* must be provided so the runner can
        load a map before running the scenario.

        Args:
            scenario_cls: The :class:`BaseScenario` subclass to instantiate.
            ego_config: Ego vehicle spawn configuration.
            xodr_path: Path to an OpenDRIVE map file (optional).
            map_name: Built-in CARLA map name, e.g. ``"Town01"`` (optional).
            timeout_seconds: Per-scenario timeout in seconds.
            output_dir: Directory where MP4 recordings are stored.
        """
        if xodr_path is None and map_name is None:
            raise ValueError("Provide either xodr_path or map_name.")

        self._scenario_cls = scenario_cls
        self._ego_config = ego_config
        self._xodr_path = xodr_path
        self._map_name = map_name
        self._timeout_seconds = timeout_seconds
        self._output_dir = output_dir

    def as_fixture(self) -> Callable[[], ScenarioResult]:
        """Return a zero-argument pytest fixture function.

        The fixture starts a ``CarlaServerManager`` as a context manager,
        creates a :class:`CarlaAutowareScenario` runner, loads the map, and
        yields the :class:`ScenarioResult` returned by :meth:`run_scenario`.

        Returns:
            A function decorated with ``@pytest.fixture``.
        """
        scenario_cls = self._scenario_cls
        ego_config = self._ego_config
        xodr_path = self._xodr_path
        map_name = self._map_name
        timeout_seconds = self._timeout_seconds
        output_dir = self._output_dir

        @pytest.fixture
        def _fixture() -> Generator[ScenarioResult, None, None]:
            with CarlaServerManager() as server:
                runner = CarlaAutowareScenario(
                    server,
                    timeout_seconds=timeout_seconds,
                    output_dir=output_dir,
                )
                if xodr_path is not None:
                    runner.load_map_from_xodr(xodr_path)
                else:
                    runner.load_map_by_name(map_name)  # type: ignore[arg-type]

                scenario = scenario_cls(ego_config)
                yield runner.run_scenario(scenario)

        return _fixture
