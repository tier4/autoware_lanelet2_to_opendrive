"""Scenario queue bound to a CarlaServerManager."""

from __future__ import annotations

import os
from collections.abc import Callable, Generator
from pathlib import Path
from typing import List, Optional

from .scenario_runner import ScenarioRunner
from .conditions import ScenarioResult
from .coordinate.map_manager import MapManager
from .scenario_base import BaseScenario
from .server import CarlaServerManager


class ScenarioQueue:
    """Owns a :class:`CarlaServerManager` and runs scenarios sequentially.

    The queue acts as a context manager that starts the CARLA server on
    entry and stops it on exit.  All scenarios must be :meth:`add`-ed
    *before* calling :meth:`run_all`.

    When used together with :class:`~autoware_carla_scenario.CarlaScenarioFixture`,
    scenarios are registered at module import time so the full list is known
    before the session fixture starts the server.


    Example – minimal usage::

        queue = ScenarioQueue(map_name="Town10HD_Opt")
        queue.add(MyScenario(ego_config))
        queue.add(AnotherScenario(ego_config))

        with queue:
            results = queue.run_all()

    Example – with an externally-managed server::

        with CarlaServerManager() as server:
            queue = ScenarioQueue(server=server, map_name="Town10HD_Opt")
            queue.add(MyScenario(ego_config))
            results = queue.run_all()
    """

    def __init__(
        self,
        server: Optional[CarlaServerManager] = None,
        *,
        xodr_path: Optional[Path] = None,
        lanelet2_path: Optional[Path] = None,
        map_name: Optional[str] = None,
        host: str = "localhost",
        port: int = 2000,
        timeout_seconds: float = 60.0,
        output_dir: Path = Path("scenario_outputs"),
        server_extra_args: Optional[List[str]] = None,
    ) -> None:
        """Create a scenario queue.

        Args:
            server: An externally-managed :class:`CarlaServerManager`.  When
                provided the queue borrows it (does not start/stop it).  When
                *None* a new manager is created and owned by this queue.
            xodr_path: OpenDRIVE map file path (optional).  Must be used
                together with *map_name*: the file at the
                ``<MAP_NAME_PATH>`` env var is replaced with *xodr_path*,
                then the map is loaded by name (retains full CARLA assets).
            lanelet2_path: Lanelet2 map file path (optional).  When provided
                together with *xodr_path*, :class:`MapManager` is initialised
                so that coordinate transforms (Lanelet2 ↔ OpenDRIVE) work.
            map_name: Built-in CARLA map name to load on start (optional).
                Used alone or together with *xodr_path*.
            host: CARLA RPC host.
            port: CARLA RPC port.
            timeout_seconds: Default per-scenario timeout.
            output_dir: Directory for MP4 recordings.
            server_extra_args: Extra CLI arguments for CarlaUE5.sh (only used
                when the queue creates its own server).
        """
        if server is not None:
            self._server = server
            self._owns_server = False
        else:
            self._server = CarlaServerManager(
                host=host,
                port=port,
                extra_args=server_extra_args,
            )
            self._owns_server = True

        self._xodr_path = xodr_path
        self._lanelet2_path = lanelet2_path
        self._map_name = map_name
        self._host = host
        self._port = port
        self._timeout_seconds = timeout_seconds
        self._output_dir = output_dir

        self._scenarios: List[BaseScenario] = []
        self._results: List[ScenarioResult] = []
        self._scenario_results: dict[int, ScenarioResult] = {}
        self._runner: Optional[ScenarioRunner] = None

    # ------------------------------------------------------------------
    # Scenario registration
    # ------------------------------------------------------------------

    def add(self, scenario: BaseScenario) -> None:
        """Append a scenario to the queue.

        Args:
            scenario: A :class:`BaseScenario` instance to enqueue.
        """
        self._scenarios.append(scenario)

    # ------------------------------------------------------------------
    # Execution
    # ------------------------------------------------------------------

    def run_all(self) -> List[ScenarioResult]:
        """Execute every registered scenario in order.

        Must be called while the queue is started (inside the context manager
        or after :meth:`start`).

        Returns:
            A list of :class:`ScenarioResult` objects in registration order.

        Raises:
            RuntimeError: If called before :meth:`start` / outside the context
                manager.
        """
        if self._runner is None:
            raise RuntimeError(
                "ScenarioQueue must be started before run_all(). "
                "Use the context manager ('with queue:') or call start() first."
            )
        results: List[ScenarioResult] = []
        for scenario in self._scenarios:
            result = self._runner.run_scenario(scenario)
            self._scenario_results[id(scenario)] = result
            results.append(result)
        self._results = results
        return results

    def result_for(self, scenario: BaseScenario) -> ScenarioResult:
        """Return the :class:`ScenarioResult` for a specific scenario instance.

        Args:
            scenario: The exact scenario object that was passed to :meth:`add`.

        Returns:
            The scenario's execution result.

        Raises:
            KeyError: If no result exists for *scenario* (run_all not called yet).
        """
        key = id(scenario)
        if key not in self._scenario_results:
            raise KeyError(
                f"No result found for {type(scenario).__name__}. "
                "Has run_all() been called?"
            )
        return self._scenario_results[key]

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Ensure the CARLA server is running, then initialise the runner.

        :meth:`CarlaServerManager.start` is called unconditionally so that
        the server is always reachable by the time the map is loaded.
        When the server is already running (``reuse_if_running=True``, the
        default) the call is effectively a no-op — it just records that the
        server was reused and returns immediately.  Only the queue that
        *owns* its server will stop it in :meth:`stop`.
        """
        self._server.start()
        self._runner = ScenarioRunner(
            self._server,
            host=self._host,
            port=self._port,
            timeout_seconds=self._timeout_seconds,
            output_dir=self._output_dir,
        )
        if self._xodr_path is not None and self._map_name is None:
            raise ValueError(
                "xodr_path requires map_name: standalone OpenDRIVE mode is not "
                "supported. Provide map_name together with xodr_path to use "
                "overwrite mode (retains full CARLA assets)."
            )
        if self._xodr_path is not None and self._map_name is not None:
            self._runner.load_map_by_overwriting_xodr(self._xodr_path, self._map_name)
        elif self._map_name is not None:
            self._runner.load_map_by_name(self._map_name)

        # Initialise MapManager when both map files are available.
        # Pass the CARLA world so that spawn-point-based z_offset averaging
        # is used instead of single-point sampling.
        if self._xodr_path is not None and self._lanelet2_path is not None:
            MapManager.reset()
            MapManager.get_instance().initialize(
                xodr_path=self._xodr_path,
                lanelet2_path=self._lanelet2_path,
                carla_world=self._runner._world,
            )

    def stop(self) -> None:
        """Stop the server if owned by this queue."""
        self._runner = None
        if self._owns_server:
            self._server.stop()

    # ------------------------------------------------------------------
    # pytest fixture factory
    # ------------------------------------------------------------------

    def as_fixture(
        self, fixture_name: str = "carla_queue"
    ) -> Callable[[], Generator["ScenarioQueue", None, None]]:
        """Return a session-scoped pytest fixture that manages this queue.

        The generated fixture automatically skips when
        ``CARLA_UE5_EXECUTABLE`` is not set, so callers do not need to
        add a manual ``pytest.skip`` guard.

        Args:
            fixture_name: Name under which the fixture is registered.
                Defaults to ``"carla_queue"``.

        Returns:
            A ``@pytest.fixture(scope="session")``-decorated function.
            Assign it to a module-level name in ``conftest.py`` or a test
            file so that pytest discovers it.

        Example::

            # conftest.py
            from autoware_carla_scenario import (
                CarlaScenarioFixture, EgoConfig, ScenarioQueue, SpawnTransform,
            )
            import carla

            ego = EgoConfig(
                spawn_location=SpawnTransform(carla.Transform(carla.Location(x=0.0, y=0.0, z=0.5))),
                vehicle_type="vehicle.mini.cooper",
            )
            _queue = ScenarioQueue(map_name="Town10HD_Opt")
            carla_queue  = _queue.as_fixture()              # auto-skip baked in
            my_result    = CarlaScenarioFixture(MyScenario, ego, queue=_queue).as_fixture()

            # test_my.py
            def test_passes(my_result):
                assert my_result.passed
        """
        import pytest

        queue = self

        @pytest.fixture(scope="session", name=fixture_name)
        def _queue_fixture() -> Generator["ScenarioQueue", None, None]:
            if not os.environ.get(CarlaServerManager.ENV_VAR):
                pytest.skip(
                    f"Environment variable '{CarlaServerManager.ENV_VAR}' is not set. "
                    "Skipping CARLA integration tests."
                )
            try:
                with queue:
                    queue.run_all()
                    yield queue
            except Exception as exc:
                pytest.skip(f"CARLA session failed: {exc}")

        return _queue_fixture

    # ------------------------------------------------------------------
    # Context manager
    # ------------------------------------------------------------------

    def __enter__(self) -> "ScenarioQueue":
        self.start()
        return self

    def __exit__(self, *args: object) -> None:
        self.stop()

    def __len__(self) -> int:
        return len(self._scenarios)
