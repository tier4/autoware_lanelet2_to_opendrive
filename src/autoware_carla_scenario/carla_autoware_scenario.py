"""Top-level scenario runner integrating all sub-components."""

from __future__ import annotations

import time
from pathlib import Path
from typing import Optional

from .conditions import ScenarioResult, TimeoutCondition
from .ego import EgoVehicle
from .recording import ScenarioRecorder
from .scenario_base import BaseScenario
from .server import CarlaServerManager


class CarlaAutowareScenario:
    """Orchestrates scenario execution: map loading, tick loop, and recording.

    A single instance can run multiple scenarios sequentially. Each call to
    :meth:`run_scenario` spawns/destroys the ego vehicle and saves a recording.

    Example::

        with CarlaServerManager() as server:
            runner = CarlaAutowareScenario(server)
            runner.load_map_by_name("Town01")
            result = runner.run_scenario(MyScenario(ego_config))
            assert result.passed
    """

    def __init__(
        self,
        server: CarlaServerManager,
        host: str = "localhost",
        port: int = 2000,
        timeout_seconds: float = 60.0,
        output_dir: Path = Path("scenario_outputs"),
    ) -> None:
        """Initialize the scenario runner.

        Args:
            server: An already-started (or context-managed) server manager.
            host: CARLA server hostname.
            port: CARLA server RPC port.
            timeout_seconds: Default timeout applied to every scenario.
            output_dir: Directory where MP4 recordings are saved.
        """
        import carla

        self.timeout_seconds = timeout_seconds
        self.output_dir = output_dir

        self._client = carla.Client(host, port)
        self._client.set_timeout(10.0)
        self._world: Optional["carla.World"] = None

    # ------------------------------------------------------------------
    # Map loading
    # ------------------------------------------------------------------

    def load_map_from_xodr(self, xodr_path: Path) -> None:
        """Load a map from an OpenDRIVE file.

        Uses ``pyxodr`` to parse the file and
        ``carla.Client.generate_opendrive_world`` to instantiate it.

        Args:
            xodr_path: Path to the ``.xodr`` file.
        """
        xodr_content = xodr_path.read_text(encoding="utf-8")
        self._world = self._client.generate_opendrive_world(
            xodr_content,
            self._client.get_world().get_settings(),
        )

    def load_map_by_name(self, map_name: str) -> None:
        """Load a built-in CARLA map by name (e.g. ``"Town01"``).

        Args:
            map_name: Name of the CARLA map asset.
        """
        self._world = self._client.load_world(map_name)

    # ------------------------------------------------------------------
    # Scenario execution
    # ------------------------------------------------------------------

    def run_scenario(self, scenario: BaseScenario) -> ScenarioResult:
        """Execute a single scenario from setup to teardown.

        Steps:
        1. Call ``scenario.setup(world)``
        2. Spawn the ego vehicle
        3. Register the default timeout fail condition
        4. Run the tick loop
        5. Save the MP4 recording
        6. Destroy the ego vehicle
        7. Return the :class:`ScenarioResult`

        Args:
            scenario: The scenario to run.

        Returns:
            The outcome of the scenario execution.
        """
        if self._world is None:
            self._world = self._client.get_world()

        world = self._world

        # Enable synchronous mode so we control the simulation tick rate
        settings = world.get_settings()
        original_synchronous = settings.synchronous_mode
        original_delta = settings.fixed_delta_seconds
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05  # 20 Hz
        world.apply_settings(settings)

        ego = EgoVehicle()
        recorder = ScenarioRecorder()
        result: Optional[ScenarioResult] = None

        try:
            scenario.setup(world)
            ego.spawn(world, scenario.ego_config)

            # Register default timeout fail condition
            scenario.register_fail_condition(TimeoutCondition(self.timeout_seconds))

            start_time = time.monotonic()

            # Tick loop
            while not scenario.is_done():
                elapsed = time.monotonic() - start_time

                # Pre-tick callbacks
                for cb in scenario._pre_tick_callbacks:
                    cb(world)

                world.tick()

                # Post-tick callbacks
                for cb in scenario._post_tick_callbacks:
                    cb(world)

                # Collect camera frames
                for frame in ego.get_camera_frames():
                    recorder.add_frame(frame)

                # Check pass conditions
                for condition in scenario._pass_conditions:
                    check = condition.check(world, elapsed)
                    if check is not None:
                        result = check
                        break

                if result is not None:
                    break

                # Check fail conditions
                for condition in scenario._fail_conditions:
                    check = condition.check(world, elapsed)
                    if check is not None:
                        result = check
                        break

                if result is not None:
                    break

            # If the loop exited via is_done() with no result, treat as passed
            if result is None:
                elapsed = time.monotonic() - start_time
                result = ScenarioResult(
                    passed=True,
                    message="Scenario completed successfully",
                    elapsed_seconds=elapsed,
                )

        finally:
            # Save recording
            scenario_name = type(scenario).__name__
            output_path = self.output_dir / f"{scenario_name}.mp4"
            try:
                recorder.save(output_path)
            except Exception:
                pass  # Recording failure must not shadow the scenario result

            ego.destroy()

            # Restore original world settings
            settings.synchronous_mode = original_synchronous
            settings.fixed_delta_seconds = original_delta
            world.apply_settings(settings)

        return result
