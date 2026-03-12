"""Top-level scenario runner integrating all sub-components."""

from __future__ import annotations

import logging
import os
import re
import shutil
import time
from pathlib import Path
from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    import carla

from .conditions import ActorExistenceCondition, ScenarioResult, TimeoutCondition
from .conditions.base import find_actor_by_role_name
from .constants import EGO_ROLE_NAME
from .coordinate.poses import CarlaWorldPose
from .coordinate.transform import to_opendrive
from .entity import EgoVehicle
from .entity import vehicle_entity as _vehicle_entity_module
from .scenario_base import BaseScenario
from .server import CarlaServerManager

logger = logging.getLogger(__name__)


def _map_name_to_env_var(map_name: str) -> str:
    """Convert a CamelCase map name to an UPPER_SNAKE_CASE environment variable name.

    A ``_PATH`` suffix is appended so callers can use the variable to locate
    the ``.xodr`` file inside the CARLA installation.

    Examples::

        _map_name_to_env_var("NishishinjukuMap")  # -> "NISHISHINJUKU_MAP_PATH"
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


#: Log ego OpenDRIVE position every N ticks (~1 s at 20 Hz).
_CONDITION_LOG_INTERVAL: int = 20


def _log_ego_opendrive_position(
    world: "carla.World",
    scenario_name: str,
    elapsed: float,
    tick_count: int,
) -> None:
    """Log the ego vehicle's current OpenDRIVE road/lane position."""
    ego = find_actor_by_role_name(world, EGO_ROLE_NAME)
    if ego is None:
        logger.warning(
            "[%s] tick=%d t=%.2fs ego actor NOT FOUND (destroyed?)",
            scenario_name,
            tick_count,
            elapsed,
        )
        return

    loc = ego.get_location()
    try:
        od = to_opendrive(CarlaWorldPose(x=loc.x, y=loc.y, z=loc.z))
        logger.info(
            "[%s] tick=%d t=%.2fs ego CARLA(%.1f, %.1f, %.1f) -> "
            "OpenDRIVE road='%s' lane=%d s=%.1f",
            scenario_name,
            tick_count,
            elapsed,
            loc.x,
            loc.y,
            loc.z,
            od.road_id,
            od.lane_id,
            od.s,
        )
    except (ValueError, KeyError, IndexError, RuntimeError) as exc:
        logger.warning(
            "[%s] tick=%d t=%.2fs ego at (%.1f, %.1f, %.1f) — "
            "failed to convert to OpenDRIVE: %s",
            scenario_name,
            tick_count,
            elapsed,
            loc.x,
            loc.y,
            loc.z,
            exc,
        )


class ScenarioRunner:
    """Orchestrates scenario execution: map loading, tick loop, and recording.

    A single instance can run multiple scenarios sequentially. Each call to
    :meth:`run_scenario` spawns/destroys the ego vehicle and saves a recording.

    Example::

        with CarlaServerManager() as server:
            runner = ScenarioRunner(server)
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
            output_dir: Directory where CARLA recording logs are saved.
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

    def load_map_by_overwriting_xodr(self, xodr_path: Path, map_name: str) -> None:
        """Load a built-in CARLA map after overwriting its internal ``.xodr`` file.

        The destination path is read from an environment variable derived from
        *map_name* by converting CamelCase to ``UPPER_SNAKE_CASE_PATH``
        (e.g. ``NishishinjukuMap`` → ``NISHISHINJUKU_MAP_PATH``).

        This allows using the full CARLA map assets (meshes, textures, etc.)
        while replacing only the road network definition.

        Args:
            xodr_path: Path to the ``.xodr`` file that will overwrite the
                internal map file.
            map_name: Built-in CARLA map name (e.g. ``"NishishinjukuMap"``).
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
        1. Enable synchronous mode on World and TrafficManager
        2. Seed the TrafficManager with ``scenario.random_seed``
        3. Call ``scenario.setup(world)``
        4. Spawn the ego vehicle
        5. Start the CARLA native recorder
        6. Register the default timeout fail condition
        7. Run the tick loop
        8. Stop the recorder and destroy the ego vehicle
        9. Restore original World / TrafficManager settings

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

        # Configure TrafficManager for deterministic behaviour.
        # Must be synchronous with the world and seeded before any
        # set_autopilot() call so NPC decisions are reproducible.
        tm = self._client.get_trafficmanager()
        tm.set_synchronous_mode(True)
        tm.set_random_device_seed(scenario.random_seed)

        ego = EgoVehicle()
        recording_started = False
        result: Optional[ScenarioResult] = None

        try:
            scenario_name = type(scenario).__name__
            logger.info("[%s] === Setup start ===", scenario_name)
            scenario.set_client(self._client)
            scenario.setup()
            logger.info("[%s] Spawning ego vehicle ...", scenario_name)
            ego_actor = ego.spawn(world, scenario.ego_config)
            logger.info(
                "[%s] Ego spawned: id=%d blueprint=%s",
                scenario_name,
                ego_actor.id,
                ego_actor.type_id,
            )

            # Register ego existence fail condition so the scenario fails
            # immediately if the ego is destroyed (e.g. falls through map).
            scenario.register_fail_condition(ActorExistenceCondition(EGO_ROLE_NAME))

            # Warm-up ticks: let physics and TrafficManager stabilise
            # before the main loop begins.
            from tqdm import tqdm  # noqa: PLC0415

            for _ in tqdm(
                range(scenario.STABILIZE_TICKS),
                desc="Warm-up",
                unit="tick",
            ):
                world.tick()

            # Enable autopilot on every vehicle (all NPCs use TrafficManager)
            n_autopilot = 0
            for actor in world.get_actors().filter("vehicle.*"):
                actor.set_autopilot(True)
                n_autopilot += 1
            if n_autopilot:
                logger.info(
                    "Autopilot enabled on %d vehicle(s) after %d warm-up ticks",
                    n_autopilot,
                    scenario.STABILIZE_TICKS,
                )

            # Apply initial speeds after warm-up stabilisation
            scenario.set_initial_speed(ego_actor)

            _vehicle_entity_module._warmup_done = True

            # Start native CARLA recorder
            output_path = self.output_dir / f"{scenario_name}.log"
            output_path.parent.mkdir(parents=True, exist_ok=True)
            self._client.start_recorder(str(output_path))
            recording_started = True
            logger.info("[%s] Recording to %s", scenario_name, output_path)

            # Register default timeout fail condition
            scenario.register_fail_condition(TimeoutCondition(self.timeout_seconds))

            logger.info("[%s] === Tick loop start ===", scenario_name)
            start_time = time.monotonic()
            tick_count = 0

            # Tick loop
            while not scenario.is_done():
                elapsed = time.monotonic() - start_time
                tick_count += 1

                # Pre-tick callbacks
                for cb in scenario._pre_tick_callbacks:
                    cb(world)

                world.tick()

                # Post-tick callbacks
                for cb in scenario._post_tick_callbacks:
                    cb(world)

                # Periodic ego OpenDRIVE position log
                if tick_count % _CONDITION_LOG_INTERVAL == 0:
                    _log_ego_opendrive_position(
                        world, scenario_name, elapsed, tick_count
                    )

                # Check pass conditions
                for i, condition in enumerate(scenario._pass_conditions):
                    check = condition.check(world, elapsed)
                    if check is not None:
                        logger.info(
                            "[%s] Pass condition [%d] SATISFIED: %s",
                            scenario_name,
                            i,
                            check.message,
                        )
                        result = ScenarioResult(
                            passed=True,
                            message=check.message,
                            elapsed_seconds=check.elapsed_seconds,
                        )
                        break
                    if tick_count % _CONDITION_LOG_INTERVAL == 0:
                        logger.info(
                            "[%s] Pass condition [%d] pending at t=%.2fs (tick %d)",
                            scenario_name,
                            i,
                            elapsed,
                            tick_count,
                        )

                if result is not None:
                    break

                # Check fail conditions
                for i, condition in enumerate(scenario._fail_conditions):
                    check = condition.check(world, elapsed)
                    if check is not None:
                        logger.info(
                            "[%s] Fail condition [%d] TRIGGERED: %s",
                            scenario_name,
                            i,
                            check.message,
                        )
                        result = ScenarioResult(
                            passed=False,
                            message=check.message,
                            elapsed_seconds=check.elapsed_seconds,
                        )
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
            logger.info("[%s] === Cleanup start ===", scenario_name)
            _vehicle_entity_module._warmup_done = False
            if recording_started:
                self._client.stop_recorder()
                logger.info("[%s] Recorder stopped", scenario_name)
            logger.info("[%s] Destroying ego vehicle ...", scenario_name)
            ego.destroy()
            logger.info("[%s] Ego destroyed", scenario_name)

            # Restore original world and TrafficManager settings
            tm.set_synchronous_mode(original_synchronous)
            settings.synchronous_mode = original_synchronous
            settings.fixed_delta_seconds = original_delta
            world.apply_settings(settings)
            logger.info("[%s] World settings restored", scenario_name)
            logger.info("[%s] === Cleanup done ===", scenario_name)

        if result is not None:
            status = "PASSED" if result.passed else "FAILED"
            logger.info(
                "[%s] Result: %s — %s (%.2fs)",
                scenario_name,
                status,
                result.message,
                result.elapsed_seconds,
            )

        return result
