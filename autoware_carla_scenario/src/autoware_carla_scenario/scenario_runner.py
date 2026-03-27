"""Top-level scenario runner integrating all sub-components."""

from __future__ import annotations

import logging
import os
import re
import shutil
import time
from pathlib import Path
from typing import TYPE_CHECKING, Optional

import carla

if TYPE_CHECKING:
    from .scenario_base import SpectatorCameraConfig

from .camera_recorder import CameraRecorder
from .conditions import EntityExistenceCondition, ScenarioResult, TimeoutCondition
from .conditions.base import BaseCondition, ConditionStatus, find_actor_by_role_name
from .constants import DEFAULT_TM_PORT, EGO_ROLE_NAME
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


def _build_condition_status(
    cond: "BaseCondition",
    index: int,
    role: str,
    world: "carla.World",
    elapsed: float,
) -> ConditionStatus:
    """Build a :class:`ConditionStatus` for a single condition.

    Args:
        cond: The condition instance.
        index: Positional index within its role group (pass or fail).
        role: ``"pass"`` or ``"fail"``.
        world: CARLA world.
        elapsed: Elapsed time in seconds.
    """
    check = cond.check(world, elapsed)
    cond_type = type(cond).__name__
    details = cond.get_details()

    if check is not None:
        satisfied = True
        message = check.message
    else:
        satisfied = False
        message = "not yet satisfied" if role == "pass" else "not triggered"

    return ConditionStatus(
        label=f"{role}[{index}]({cond.label})",
        satisfied=satisfied,
        message=message,
        condition_type=cond_type,
        role=role,
        details=details,
    )


def _collect_condition_statuses(
    scenario: "BaseScenario",
    world: "carla.World",
    elapsed: float,
    scenario_name: str,
) -> list[ConditionStatus]:
    """Snapshot current status of all pass and fail conditions.

    Each condition is checked once and the result is recorded as a
    :class:`ConditionStatus`.  The statuses are also logged for
    real-time visibility.
    """
    statuses: list[ConditionStatus] = []

    for i, cond in enumerate(scenario._pass_conditions):
        status = _build_condition_status(cond, i, "pass", world, elapsed)
        if status.satisfied:
            logger.info(
                "[%s]   %s: OK — %s", scenario_name, status.label, status.message
            )
        else:
            logger.info("[%s]   %s: PENDING", scenario_name, status.label)
        statuses.append(status)

    for i, cond in enumerate(scenario._fail_conditions):
        status = _build_condition_status(cond, i, "fail", world, elapsed)
        if status.satisfied:
            logger.info(
                "[%s]   %s: TRIGGERED — %s",
                scenario_name,
                status.label,
                status.message,
            )
        statuses.append(status)

    return statuses


def _unique_path(path: Path) -> Path:
    """Return *path* if it does not exist, otherwise append ``_1``, ``_2``, … ."""
    if not path.exists():
        return path
    stem = path.stem
    suffix = path.suffix
    counter = 1
    while True:
        candidate = path.with_name(f"{stem}_{counter}{suffix}")
        if not candidate.exists():
            return candidate
        counter += 1


def _destroy_all_dynamic_actors(world: "carla.World", scenario_name: str) -> None:
    """Destroy all vehicles and sensors in the world for a clean state."""
    destroyed = 0
    actors = world.get_actors()
    for actor in [*actors.filter("vehicle.*"), *actors.filter("sensor.*")]:
        try:
            actor.destroy()
            destroyed += 1
        except RuntimeError:
            pass
    if destroyed:
        world.tick()
        logger.info("[%s] Destroyed %d leftover actor(s)", scenario_name, destroyed)


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
        tm_port: int = DEFAULT_TM_PORT,
        timeout_seconds: float = 60.0,
        output_dir: Path = Path("scenario_outputs"),
    ) -> None:
        """Initialize the scenario runner.

        Args:
            server: An already-started (or context-managed) server manager.
            host: CARLA server hostname.
            port: CARLA server RPC port.
            tm_port: CARLA TrafficManager port.
            timeout_seconds: Default timeout applied to every scenario.
            output_dir: Directory where CARLA recording logs are saved.
        """
        self.timeout_seconds = timeout_seconds
        self.output_dir = output_dir
        self._tm_port = tm_port

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
    # Video rendering from recording
    # ------------------------------------------------------------------

    def _render_video_from_recording(
        self,
        log_path: Path,
        mp4_path: Path,
        spectator_config: "SpectatorCameraConfig",
        tick_count: int,
        scenario_name: str,
    ) -> None:
        """Replay a CARLA recording and render video with an attached RGB camera.

        The ``.log`` file produced by the native CARLA recorder is replayed in
        synchronous mode.  An RGB camera sensor is attached to the ego vehicle
        at the same offset as the spectator camera, and the captured frames are
        written to an MP4 file via :class:`CameraRecorder`.

        Args:
            log_path: Path to the CARLA ``.log`` recording file.
            mp4_path: Destination path for the rendered ``.mp4`` video.
            spectator_config: Camera offset / pitch matching the spectator.
            tick_count: Number of ticks recorded during the scenario.
            scenario_name: Scenario class name (used for logging).
        """
        world = self._client.get_world()

        # Destroy any leftover actors (NPCs, sensors, etc.) so the
        # replay starts from a completely clean state.
        _destroy_all_dynamic_actors(world, scenario_name)

        # Enable synchronous mode for controlled replay
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05  # 20 Hz – same as scenario execution
        world.apply_settings(settings)

        camera_recorder: Optional[CameraRecorder] = None

        try:
            logger.info(
                "[%s] Starting replay of %s for video rendering",
                scenario_name,
                log_path,
            )
            # replay_file(filename, start, duration, follow_id)
            # duration=0.0 replays the entire recording.
            self._client.replay_file(str(log_path), 0.0, 0.0, 0)

            # Tick once so the replayer spawns actors from the first frame.
            world.tick()

            # Find the ego vehicle among replayed actors
            ego = find_actor_by_role_name(world, EGO_ROLE_NAME)
            if ego is None:
                logger.warning(
                    "[%s] Could not find ego actor during replay — "
                    "skipping video render",
                    scenario_name,
                )
                return

            mp4_path.parent.mkdir(parents=True, exist_ok=True)
            camera_recorder = CameraRecorder(
                world,
                ego,
                mp4_path,
                offset_back=spectator_config.offset_back,
                offset_up=spectator_config.offset_up,
                pitch=spectator_config.pitch,
            )

            # Tick through the remaining replay frames.  We already consumed
            # one tick above for actor spawn, so replay (tick_count - 1) more.
            # Each tick is followed by a synchronous frame write so no
            # frames are lost to async delivery delays.
            remaining = max(tick_count - 1, 0)
            for _ in range(remaining):
                world.tick()
                camera_recorder.write_frame()

            logger.info("[%s] Video rendered to %s", scenario_name, mp4_path)

        except Exception:
            logger.warning("[%s] Video rendering failed", scenario_name, exc_info=True)
        finally:
            if camera_recorder is not None:
                camera_recorder.stop()
            # Stop the replayer; leftover actors are cleaned up by the
            # reload_world() call at the end of run_scenario().
            self._client.stop_replayer(keep_actors=False)

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
        scenario_name = type(scenario).__name__

        # Destroy any leftover actors from a previous scenario that may
        # have survived a failed reload_world().  On a clean world this
        # is a no-op.
        _destroy_all_dynamic_actors(world, scenario_name)

        # Enable synchronous mode so we control the simulation tick rate.
        # Original settings are not saved because reload_world() at the
        # end of this method resets everything to defaults.
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05  # 20 Hz
        world.apply_settings(settings)

        # Configure TrafficManager for deterministic behaviour.
        # Must be synchronous with the world and seeded before any
        # set_autopilot() call so NPC decisions are reproducible.
        tm = self._client.get_trafficmanager(self._tm_port)
        tm.set_synchronous_mode(True)
        tm.set_random_device_seed(scenario.random_seed)

        ego = EgoVehicle()
        recording_started = False
        tick_count = 0
        result: Optional[ScenarioResult] = None

        try:
            logger.info("[%s] === Setup start ===", scenario_name)
            scenario.set_client(self._client, tm_port=self._tm_port)
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
            scenario.register_fail_condition(
                EntityExistenceCondition(EGO_ROLE_NAME, label="ego_existence")
            )

            # Warm-up ticks: let physics and TrafficManager stabilise
            # before the main loop begins.
            for _ in range(scenario.STABILIZE_TICKS):
                world.tick()

            # Enable autopilot on every vehicle (all NPCs use TrafficManager)
            n_autopilot = 0
            for actor in world.get_actors().filter("vehicle.*"):
                actor.set_autopilot(True, self._tm_port)
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
            scenario.register_fail_condition(
                TimeoutCondition(self.timeout_seconds, label="default_timeout")
            )

            logger.info("[%s] === Tick loop start ===", scenario_name)
            start_time = time.monotonic()

            # Tick loop
            while not scenario.is_done():
                elapsed = time.monotonic() - start_time
                tick_count += 1

                # Pre-tick actions (receive elapsed)
                for action in scenario._pre_tick_actions:
                    action.tick(world, elapsed)

                # Pre-tick callbacks
                for cb in scenario._pre_tick_callbacks:
                    cb(world)

                world.tick()

                # Post-tick actions (receive elapsed)
                for action in scenario._post_tick_actions:
                    action.tick(world, elapsed)

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
                            "[%s] Pass condition [%d](%s) SATISFIED: %s",
                            scenario_name,
                            i,
                            condition.label,
                            check.message,
                        )
                        result = ScenarioResult(
                            passed=True,
                            message=check.message,
                            elapsed_seconds=check.elapsed_seconds,
                            condition_statuses=_collect_condition_statuses(
                                scenario, world, elapsed, scenario_name
                            ),
                        )
                        break
                    if tick_count % _CONDITION_LOG_INTERVAL == 0:
                        logger.info(
                            "[%s] Pass condition [%d](%s) pending at t=%.2fs (tick %d)",
                            scenario_name,
                            i,
                            condition.label,
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
                            "[%s] Fail condition [%d](%s) TRIGGERED: %s",
                            scenario_name,
                            i,
                            condition.label,
                            check.message,
                        )
                        result = ScenarioResult(
                            passed=False,
                            message=check.message,
                            elapsed_seconds=check.elapsed_seconds,
                            condition_statuses=_collect_condition_statuses(
                                scenario, world, elapsed, scenario_name
                            ),
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

            # Explicitly destroy the ego vehicle so it does not persist
            # if reload_world() fails later.
            try:
                ego.destroy()
                logger.info("[%s] Ego vehicle destroyed", scenario_name)
            except Exception:
                logger.warning(
                    "[%s] Failed to destroy ego vehicle",
                    scenario_name,
                    exc_info=True,
                )

            if recording_started:
                self._client.stop_recorder()
                logger.info("[%s] Recorder stopped", scenario_name)

            # Shut down the TrafficManager so the next run starts with a
            # fresh instance (resets InMemoryMap cache and internal state).
            tm.shut_down()
            logger.info("[%s] TrafficManager shut down", scenario_name)
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
            json_path = self.output_dir / f"{scenario_name}_result.json"
            json_path.parent.mkdir(parents=True, exist_ok=True)
            json_path.write_text(result.to_json(indent=2), encoding="utf-8")
            logger.info("[%s] Result JSON written to: %s", scenario_name, json_path)

        # Render video from the CARLA recording after scenario cleanup
        if (
            recording_started
            and tick_count > 0
            and scenario._spectator_camera_config is not None
        ):
            log_path = self.output_dir / f"{scenario_name}.log"
            mp4_path = _unique_path(self.output_dir / f"{scenario_name}.mp4")
            self._render_video_from_recording(
                log_path=log_path,
                mp4_path=mp4_path,
                spectator_config=scenario._spectator_camera_config,
                tick_count=tick_count,
                scenario_name=scenario_name,
            )

        # Reload the world to guarantee a completely clean state (all
        # actors, sensors, and physics state are reset).  This is more
        # reliable than destroying actors individually, which can fail
        # with "failed to destroy actor" errors from the CARLA server.
        # Flush any pending server-side operations (e.g. replayer
        # shutdown) before reloading to avoid std::exception errors.
        try:
            world = self._client.get_world()
            world.tick()
            self._client.reload_world()
            # After reload, get the fresh world and tick it once so that
            # CARLA fully initialises the new world state.
            self._world = self._client.get_world()
            self._world.tick()
            logger.info("[%s] World reloaded and ticked", scenario_name)
        except Exception:
            logger.warning(
                "[%s] reload_world failed — world may retain residual state",
                scenario_name,
                exc_info=True,
            )
            # Ensure self._world is refreshed even on failure so that the
            # next scenario does not operate on a stale world reference.
            try:
                self._world = self._client.get_world()
            except Exception:
                logger.warning(
                    "[%s] Failed to refresh world reference after reload failure",
                    scenario_name,
                    exc_info=True,
                )

        return result
