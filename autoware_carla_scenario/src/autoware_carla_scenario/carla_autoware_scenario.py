"""Top-level scenario runner integrating all sub-components."""

from __future__ import annotations

import os
import re
import shutil
import time
from pathlib import Path
from typing import Optional

from .conditions import ScenarioResult, TimeoutCondition
from .ego import EgoVehicle
from .recording import ScenarioRecorder
from .scenario_base import BaseScenario
from .server import CarlaServerManager


def _map_name_to_env_var(map_name: str) -> str:
    """Convert a CamelCase map name to an UPPER_SNAKE_CASE environment variable name.

    A ``_PATH`` suffix is appended so callers can use the variable to locate
    the ``.xodr`` file inside the CARLA installation.

    Examples::

        _map_name_to_env_var("NishishinjyukuMap")  # -> "NISHISHINJYUKU_MAP_PATH"
        _map_name_to_env_var("Town01")             # -> "TOWN01_PATH"
        _map_name_to_env_var("Town10HD_Opt")       # -> "TOWN10_HD_OPT_PATH"

    Args:
        map_name: CamelCase CARLA map name.

    Returns:
        The derived environment variable name.
    """
    # Insert underscore between a lowercase/digit and the following uppercase letter
    snake = re.sub(r"(?<=[a-z0-9])([A-Z])", r"_\1", map_name)
    return snake.upper() + "_PATH"


class CarlaAutowareScenario:
    """Orchestrates scenario execution: map loading, tick loop, and recording.

    A single instance can run multiple scenarios sequentially. Each call to
    :meth:`run_scenario` spawns/destroys the ego vehicle and saves a recording.

    Example::

        with CarlaServerManager() as server:
            runner = CarlaAutowareScenario(server)
            runner.load_map_by_name("Town10HD_Opt")
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

    def load_map_by_overwriting_xodr(self, xodr_path: Path, map_name: str) -> None:
        """Load a built-in CARLA map after overwriting its internal ``.xodr`` file.

        The destination path is read from an environment variable derived from
        *map_name* by converting CamelCase to ``UPPER_SNAKE_CASE_PATH``
        (e.g. ``NishishinjyukuMap`` → ``NISHISHINJYUKU_MAP_PATH``).

        This allows using the full CARLA map assets (meshes, textures, etc.)
        while replacing only the road network definition.

        Args:
            xodr_path: Path to the ``.xodr`` file that will overwrite the
                internal map file.
            map_name: Built-in CARLA map name (e.g. ``"NishishinjyukuMap"``).
                Must exist in the server's available maps.

        Raises:
            RuntimeError: If the derived environment variable is not set.
            FileNotFoundError: If *xodr_path* does not exist.
        """
        env_var = _map_name_to_env_var(map_name)
        dest_str = os.environ.get(env_var)
        if not dest_str:
            raise RuntimeError(
                f"Environment variable '{env_var}' is not set. "
                f"Set it to the path of the internal .xodr file for map '{map_name}' "
                f"inside the CARLA installation "
                f"(e.g. /opt/carla/CarlaUE5/Content/Carla/Maps/{map_name}.xodr)."
            )
        dest = Path(dest_str)
        shutil.copy2(xodr_path, dest)
        self.load_map_by_name(map_name)

    def load_map_by_name(self, map_name: str) -> None:
        """Load a built-in CARLA map by name (e.g. ``"Town10HD_Opt"``).

        Available maps are checked before loading.  Both short names
        (``"Town10HD_Opt"``) and full asset paths (``"/Game/Carla/Maps/Town10HD_Opt"``)
        are accepted.

        Args:
            map_name: Name of the CARLA map asset.

        Raises:
            ValueError: If *map_name* is not found in the server's available
                maps, with the full list of valid names included in the message.
        """
        available: list[str] = self._client.get_available_maps()

        # Accept either a full asset path or the short name (last path segment).
        def _matches(candidate: str) -> bool:
            return candidate == map_name or candidate.split("/")[-1] == map_name

        if not any(_matches(m) for m in available):
            short_names = sorted(m.split("/")[-1] for m in available)
            raise ValueError(
                f"Map {map_name!r} is not available on the CARLA server. "
                f"Available maps: {short_names}"
            )

        self._world = self._client.load_world(map_name)

    # ------------------------------------------------------------------
    # Scenario execution
    # ------------------------------------------------------------------

    def run_scenario(self, scenario: BaseScenario) -> ScenarioResult:
        """Execute a single scenario from setup to teardown.

        Steps:
        1. Call ``scenario.setup(world)``
        2. Spawn the ego vehicle
        3. Start the CARLA native recorder
        4. Register the default timeout fail condition
        5. Run the tick loop
        6. Stop the recorder
        7. Destroy the ego vehicle
        8. Return the :class:`ScenarioResult`

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

            # Start native CARLA recorder
            scenario_name = type(scenario).__name__
            output_path = self.output_dir / f"{scenario_name}.log"
            recorder.start(self._client, output_path)

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
            recorder.stop()
            ego.destroy()

            # Restore original world settings
            settings.synchronous_mode = original_synchronous
            settings.fixed_delta_seconds = original_delta
            world.apply_settings(settings)

        return result
