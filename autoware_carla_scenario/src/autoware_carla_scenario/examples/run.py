"""Hydra-based unified entry point for all example scenarios.

Usage examples::

    # Run intersection-passing scenario (straight-through)
    uv run scenario scenario=intersection_passing/straight

    # Run left-turn variant (uses intersection_passing with turn_direction=left)
    uv run scenario scenario=intersection_passing/left_turn

    # Run traffic-light-compliance scenario
    uv run scenario scenario=traffic_light_compliance/traffic_light_compliance

    # Run all intersection-passing variants in a single batch
    uv run scenario scenario='intersection_passing/*'

    # Glob patterns also work with ? and [
    uv run scenario scenario='intersection_passing/left_*'

    # Select a different map
    uv run scenario scenario=intersection_passing/straight map=nishishinjuku

    # Override individual parameters
    uv run scenario scenario=intersection_passing/straight scenario.timeout_seconds=15.0

    # Override server connection
    uv run scenario scenario=intersection_passing/straight server.host=192.168.1.100 server.port=3000
"""

from __future__ import annotations

import logging
import os
import sys
from pathlib import Path

import carla
import hydra
from hydra import compose, initialize_config_dir
from hydra.core.global_hydra import GlobalHydra
from hydra.core.hydra_config import HydraConfig
from omegaconf import DictConfig, OmegaConf

from autoware_carla_scenario import (
    BaseScenario,
    EgoConfig,
    EgoVehicle,
    GroundProjectionConfig,
    Lanelet2Pose,
    ScenarioQueue,
    SpawnTransform,
)
from autoware_carla_scenario.conditions import ScenarioResult
from autoware_carla_scenario.registry import (
    BuildScenarioFn,
    get_conf_dirs,
    get_scenario_builder,
    get_scenario_registry,
    load_scenario_plugins,
    register_conf_dir,
    register_scenario,
)

from .configs import (
    IntersectionPassingConfig,
    LaneChangeConfig,
    TemporaryStopConfig,
    TrafficLightComplianceConfig,
)
from .intersection_passing import IntersectionPassingScenario
from .lane_change import LaneChangeScenario
from .temporary_stop import TemporaryStopScenario
from .traffic_light_compliance import TrafficLightComplianceScenario

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Scenario Registry
# ---------------------------------------------------------------------------
#
# The registry itself (``register_scenario``, ``register_scenario_builder``,
# ``get_scenario_registry``, the conf-dir registry, and entry-point plugin
# discovery) lives in :mod:`autoware_carla_scenario.registry` (and is
# re-exported from the top-level package) so it can be imported without pulling
# in CARLA.  This runner only imports the pieces it actually uses.

# Directory containing the built-in Hydra config files (conf/ next to this
# module).  Registered so it becomes the primary entry in the search path.
_CONF_DIR = Path(__file__).resolve().parent / "conf"
register_conf_dir(_CONF_DIR)


# Register built-in scenarios.
register_scenario(
    "intersection_passing", IntersectionPassingScenario, IntersectionPassingConfig
)
register_scenario(
    "traffic_light_compliance",
    TrafficLightComplianceScenario,
    TrafficLightComplianceConfig,
)
register_scenario("lane_change", LaneChangeScenario, LaneChangeConfig)
register_scenario("temporary_stop", TemporaryStopScenario, TemporaryStopConfig)


# ---------------------------------------------------------------------------
# Reusable helpers
# ---------------------------------------------------------------------------


def build_ego_and_spawn(
    cfg: DictConfig,
) -> tuple[EgoConfig, Lanelet2Pose, GroundProjectionConfig]:
    """Extract :class:`EgoConfig`, spawn pose, and ground-projection config.

    This is the common preamble shared by all built-in scenarios.  Downstream
    projects can call this helper and then instantiate their own scenario class
    without duplicating the boilerplate.
    """
    ground_projection = GroundProjectionConfig(
        ray_distance_upper=float(cfg.entity.ground_projection_ray_distance_upper),
        ray_distance_lower=float(cfg.entity.ground_projection_ray_distance_lower),
    )
    ego = EgoConfig(
        spawn_location=SpawnTransform(
            carla.Transform(carla.Location(x=0.0, y=0.0, z=0.0))
        ),
        vehicle_type=cfg.ego.vehicle_type,
        initial_speed_kmh=float(cfg.ego.initial_speed_kmh),
        spawn_retry_max_count=int(cfg.entity.spawn_retry_max_count),
        spawn_retry_t_step=float(cfg.entity.spawn_retry_t_step),
        spawn_retry_z_step=float(cfg.entity.spawn_retry_z_step),
    )
    spawn_pose = Lanelet2Pose(
        lanelet_id=cfg.ego.spawn_lanelet_id,
        s=cfg.ego.spawn_s,
    )
    return ego, spawn_pose, ground_projection


def build_ego_entity(cfg: DictConfig) -> EgoVehicle | None:
    """Build the ego entity selected by ``cfg.ego.entity``.

    Returns ``None`` for ``"autopilot"``, letting the scenario fall back to its default
    :class:`~autoware_carla_scenario.entity.ego.EgoVehicle`. A config with no ``ego``
    group at all selects ``"autopilot"`` too: an injected ``build_scenario_fn`` supplies
    its own :class:`EgoConfig`, so it has no reason to carry the group Hydra would.

    Raises:
        ValueError: If ``cfg.ego.entity`` names an unknown entity.
    """
    ego_cfg = cfg.get("ego") or {}
    entity = str(ego_cfg.get("entity", "autopilot"))

    if entity == "autopilot":
        return None

    if entity == "autoware":
        from autoware_carla_scenario import AutowareEntity  # noqa: PLC0415

        return AutowareEntity()

    if entity == "autoware_ego":
        return _build_autoware_ego_entity(cfg)

    if entity == "carla_driver":
        from autoware_carla_scenario import CarlaDriverEntity  # noqa: PLC0415
        from autoware_carla_scenario.driver import (  # noqa: PLC0415
            ControlConfig,
            DriverClientConfig,
        )

        driver_cfg = cfg.get("driver")
        if driver_cfg is None:
            msg = (
                "ego.entity=carla_driver requires the 'driver' config group. "
                "Add 'driver: default' to the defaults list."
            )
            raise ValueError(msg)

        driver_dict = _to_dict(driver_cfg)
        control = ControlConfig.from_mapping(driver_dict.get("control", {}))
        return CarlaDriverEntity(DriverClientConfig.from_mapping(driver_dict), control)

    msg = (
        f"Unknown ego.entity: {entity!r}. Expected one of: "
        "'autopilot', 'autoware', 'autoware_ego', 'carla_driver'."
    )
    raise ValueError(msg)


def _build_autoware_ego_entity(cfg: DictConfig) -> EgoVehicle:
    """Build the closed-loop :class:`AutowareEgoEntity` from the Hydra config.

    Loads the map (so the spawn/goal Lanelet2 poses can be converted to map-frame
    ``BridgePose``), hosts the ``AutowareBridge`` gRPC server, and hands both the
    mission and the server to the entity.  The map is a singleton the scenario
    reloads (reset + initialize) before it runs, so this early load is independent
    -- it only needs the map to derive the (plain) poses now.

    Raises:
        ValueError: If the map lacks the files needed to derive map-frame poses.
    """
    from autoware_carla_scenario import (  # noqa: PLC0415
        AutowareBridgeConfig,
        AutowareEgoEntity,
        GrpcAutowareBridgeServer,
    )
    from autoware_carla_scenario.autoware_bridge import BridgePose  # noqa: PLC0415
    from autoware_carla_scenario.coordinate import (  # noqa: PLC0415
        MapManager,
        lanelet2_to_map,
    )
    from autoware_carla_scenario.coordinate.poses import Lanelet2Pose  # noqa: PLC0415

    ego_cfg = cfg.ego
    map_cfg = cfg.get("map") or {}
    if not map_cfg.get("lanelet2_path"):
        msg = (
            "ego.entity=autoware_ego needs a map with a 'lanelet2_path' to "
            "derive the Autoware initial pose and goal (OpenDRIVE is optional -- "
            "the poses come from the Lanelet2 centerline)."
        )
        raise ValueError(msg)

    # Load the map so the poses can be derived now.  Only the Lanelet2 centerline
    # is needed (lanelet2_to_map), so a lanelet2-only map (no 'xodr_path') is
    # fine.  The scenario reloads the map (reset + initialize) before it runs,
    # so this early load is independent.
    MapManager.reset()
    MapManager.get_instance().initialize(
        xodr_path=Path(map_cfg["xodr_path"]) if map_cfg.get("xodr_path") else None,
        lanelet2_path=Path(map_cfg["lanelet2_path"]),
    )

    def _bridge_pose(pose: Lanelet2Pose) -> BridgePose:
        x, y, z, yaw = lanelet2_to_map(pose)
        return BridgePose.from_yaw(x=x, y=y, z=z, yaw=yaw)

    spawn = Lanelet2Pose(
        lanelet_id=int(ego_cfg["spawn_lanelet_id"]), s=float(ego_cfg["spawn_s"])
    )
    goal_lanelet_id = ego_cfg.get("goal_lanelet_id")
    goal_s = ego_cfg.get("goal_s")
    goal = Lanelet2Pose(
        lanelet_id=int(goal_lanelet_id)
        if goal_lanelet_id is not None
        else spawn.lanelet_id,
        s=float(goal_s) if goal_s is not None else spawn.s,
    )

    bridge_node = cfg.get("bridge")
    bridge_map = _to_dict(bridge_node) if bridge_node is not None else {}
    bridge_config = AutowareBridgeConfig.from_mapping(bridge_map)
    server = GrpcAutowareBridgeServer(bridge_config)
    return AutowareEgoEntity(
        bridge_config,
        bridge=server,
        initial_pose=_bridge_pose(spawn),
        goal_pose=_bridge_pose(goal),
    )


def run_scenario_with_queue(
    scenario: BaseScenario,
    *,
    host: str = "localhost",
    port: int = 2000,
    tm_port: int = 8000,
    xodr_path: Path | None = None,
    lanelet2_path: Path | None = None,
    map_name: str | None = None,
    cooldown_seconds: float = 0.0,
    cooldown_max_retries: int = 0,
    output_dir: Path = Path("scenario_outputs"),
) -> ScenarioResult:
    """Run a single pre-built scenario using :class:`ScenarioQueue`.

    This is the extracted orchestration logic that was previously embedded
    inside :func:`run_scenario`.  Downstream projects can use this to execute
    their own ``BaseScenario`` subclasses without duplicating the queue setup::

        scenario = MyCustomScenario(ego, config=my_cfg, ...)
        result = run_scenario_with_queue(
            scenario, host="localhost", port=2000,
            output_dir=Path("outputs"),
        )
    """
    queue = ScenarioQueue(
        host=host,
        port=port,
        tm_port=tm_port,
        xodr_path=xodr_path,
        lanelet2_path=lanelet2_path,
        map_name=map_name,
        cooldown_seconds=cooldown_seconds,
        cooldown_max_retries=cooldown_max_retries,
        output_dir=output_dir,
    )
    queue.add(scenario)
    with queue:
        results = queue.run_all()
    return results[0]


def _to_dict(cfg_node: DictConfig) -> dict:  # type: ignore[type-arg]
    """Convert an OmegaConf node to a plain dict (typed helper)."""
    container = OmegaConf.to_container(cfg_node, resolve=True)
    assert isinstance(container, dict)  # noqa: S101
    return container


def _is_glob_pattern(value: str) -> bool:
    """Return ``True`` if *value* contains glob metacharacters."""
    return any(ch in value for ch in ("*", "?", "["))


def _is_multirun() -> bool:
    """Return ``True`` when running under Hydra ``--multirun``."""
    return "--multirun" in sys.argv or "-m" in sys.argv


def _extract_scenario_override(argv: list[str]) -> tuple[str | None, list[str]]:
    """Parse *argv* to extract the ``scenario=…`` value.

    Returns:
        A 2-tuple of ``(scenario_value, remaining_overrides)``.
        *scenario_value* is ``None`` when no ``scenario=`` argument is found.
    """
    scenario_value: str | None = None
    remaining: list[str] = []
    for arg in argv[1:]:  # skip argv[0] (program name)
        if arg.startswith("scenario="):
            scenario_value = arg[len("scenario=") :]
        else:
            remaining.append(arg)
    return scenario_value, remaining


def _find_scenario_yaml(name: str) -> Path:
    """Return the YAML path for scenario *name* across registered conf dirs.

    Falls back to the built-in dir (for display only) when the file is not
    found on disk, e.g. for a config coming from Hydra's ConfigStore.
    """
    for conf_dir in get_conf_dirs():
        candidate = conf_dir / "scenario" / f"{name}.yaml"
        if candidate.is_file():
            return candidate
    return _CONF_DIR / "scenario" / f"{name}.yaml"


def _resolve_scenario_glob(pattern: str) -> list[str]:
    """Glob ``conf/scenario/{pattern}.yaml`` across every registered conf dir.

    Each returned name is a Hydra config path relative to ``scenario/`` without
    the ``.yaml`` suffix (e.g. ``"intersection_passing/left_turn"``).  Matches
    from external scenario packages are included alongside the built-ins.
    """
    names: set[str] = set()
    searched_dirs: list[Path] = []
    for conf_dir in get_conf_dirs():
        scenario_dir = conf_dir / "scenario"
        if not scenario_dir.is_dir():
            continue
        searched_dirs.append(scenario_dir)
        for m in scenario_dir.glob(f"{pattern}.yaml"):
            rel = m.relative_to(scenario_dir).with_suffix("")
            # Hydra config names always use forward slashes, so normalise
            # away any OS-specific separator.
            names.add(rel.as_posix())
    if not names:
        print(  # noqa: T201
            f"Error: no scenario configs match pattern '{pattern}' "
            f"under {[str(d) for d in searched_dirs]}"
        )
        sys.exit(1)
    return sorted(names)


def _compose_config(scenario_name: str, overrides: list[str]) -> DictConfig:
    """Build a resolved Hydra config for *scenario_name* using the Compose API."""
    GlobalHydra.instance().clear()
    with initialize_config_dir(config_dir=str(_CONF_DIR), version_base=None):
        cfg = compose(
            config_name="config",
            overrides=[f"scenario={scenario_name}", *overrides],
        )
    return cfg


def _write_batch_result_json(
    names: list[str],
    results: list[ScenarioResult],
    output_dir: Path,
) -> Path:
    """Write a machine-readable JSON summary to *output_dir* and return the path."""
    import json  # noqa: PLC0415

    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = output_dir / "batch_results.json"
    json_results = [
        {"scenario": name, **result.to_dict()} for name, result in zip(names, results)
    ]
    json_path.write_text(
        json.dumps(json_results, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    return json_path


def _print_summary(
    names: list[str],
    results: list[ScenarioResult],
    output_dir: Path = Path("scenario_outputs"),
) -> bool:
    """Print a formatted result table and return ``True`` if all passed.

    A machine-readable JSON file is also written to *output_dir*.
    """
    sep = "=" * 60
    thin = "-" * 60
    print(f"\n{sep}")  # noqa: T201
    print("Batch Scenario Results")  # noqa: T201
    print(sep)  # noqa: T201
    for name, result in zip(names, results):
        tag = "PASS" if result.passed else "FAIL"
        print(f"  [{tag}] {name} ({result.elapsed_seconds:.1f}s)")  # noqa: T201
        if not result.passed:
            for line in result.message.splitlines():
                print(f"         {line}")  # noqa: T201
        if result.condition_statuses:
            max_label_len = max(len(cs.label) for cs in result.condition_statuses)
            for cs in result.condition_statuses:
                mark = "OK" if cs.satisfied else "NG"
                padded = cs.label.ljust(max_label_len)
                print(f"    [{mark}] {padded} : {cs.message}")  # noqa: T201

    json_path = _write_batch_result_json(names, results, output_dir)
    print(thin)  # noqa: T201
    print(f"Result JSON: {json_path}")  # noqa: T201

    passed = sum(1 for r in results if r.passed)
    total = len(results)
    print(thin)  # noqa: T201
    print(f"{passed}/{total} scenarios passed")  # noqa: T201
    print(sep)  # noqa: T201
    return passed == total


def _log_batch_plan(
    scenario_names: list[str],
    configs: list[DictConfig],
    overrides: list[str],
) -> None:
    """Log which YAML configs will be loaded and their resolved parameters."""
    sep = "=" * 60
    thin = "-" * 60
    logger.info(sep)
    logger.info(
        "Batch execution plan: %d scenario(s)%s",
        len(scenario_names),
        f"  (extra overrides: {overrides})" if overrides else "",
    )
    logger.info(sep)

    for i, (name, cfg) in enumerate(zip(scenario_names, configs), 1):
        yaml_path = _find_scenario_yaml(name)
        logger.info(thin)
        logger.info("[%d/%d] %s", i, len(scenario_names), name)
        logger.info("  config file : %s", yaml_path)
        logger.info("  map         : %s", cfg.map.name)
        logger.info("  server      : %s:%s", cfg.server.host, cfg.server.port)
        logger.info("  TM port     : %s", cfg.traffic_manager.port)
        logger.info(
            "  ego         : %s (%.1f km/h) spawn=lanelet:%d s:%.1f",
            cfg.ego.vehicle_type,
            cfg.ego.initial_speed_kmh,
            cfg.ego.spawn_lanelet_id,
            cfg.ego.spawn_s,
        )
        # Log all scenario-specific parameters.
        logger.info("  scenario parameters:")
        scenario_dict = OmegaConf.to_container(cfg.scenario, resolve=True)
        assert isinstance(scenario_dict, dict)  # noqa: S101
        for key, value in scenario_dict.items():
            logger.info("    %-30s = %s", key, value)

    logger.info(sep)


def _make_batch_output_dir() -> Path:
    """Create a Hydra-style timestamped output directory for batch runs.

    Returns:
        Absolute path to the created directory
        (e.g. ``outputs/2026-03-13/12-00-00/``).
    """
    from datetime import datetime  # noqa: PLC0415

    now = datetime.now()  # noqa: DTZ005
    output_dir = Path("outputs") / now.strftime("%Y-%m-%d") / now.strftime("%H-%M-%S")
    output_dir.mkdir(parents=True, exist_ok=True)
    return output_dir.resolve()


def run_batch(
    scenario_names: list[str],
    overrides: list[str],
    *,
    build_scenario_fn: BuildScenarioFn | None = None,
) -> None:
    """Compose configs, build scenarios, and run them in a single queue."""
    configs = [_compose_config(name, overrides) for name in scenario_names]

    # Validate all configs share the same map (shared CARLA server constraint).
    map_names = {str(cfg.map.name) for cfg in configs}
    if len(map_names) > 1:
        print(  # noqa: T201
            f"Error: batch scenarios must share the same map, but found: {map_names}"
        )
        sys.exit(1)

    # Log detailed execution plan before building anything.
    _log_batch_plan(scenario_names, configs, overrides)

    first_cfg = configs[0]

    xodr_path = (
        Path(first_cfg.map.xodr_path) if first_cfg.map.get("xodr_path") else None
    )
    lanelet2_path = (
        Path(first_cfg.map.lanelet2_path)
        if first_cfg.map.get("lanelet2_path")
        else None
    )

    cooldown = float(first_cfg.server.get("cooldown_seconds", 0.0))
    cooldown_max_retries = int(first_cfg.server.get("cooldown_max_retries", 0))

    # Batch mode bypasses @hydra.main, so we create the output directory
    # ourselves following Hydra's timestamped convention.
    output_dir = _make_batch_output_dir()

    queue = ScenarioQueue(
        host=first_cfg.server.host,
        port=first_cfg.server.port,
        tm_port=first_cfg.traffic_manager.port,
        xodr_path=xodr_path,
        lanelet2_path=lanelet2_path,
        map_name=first_cfg.map.name,
        cooldown_seconds=cooldown,
        cooldown_max_retries=cooldown_max_retries,
        output_dir=output_dir,
    )

    for i, (name, cfg) in enumerate(zip(scenario_names, configs), 1):
        logger.info("Building scenario [%d/%d]: %s", i, len(scenario_names), name)
        _ego, scenario = build_scenario(cfg, build_scenario_fn=build_scenario_fn)
        queue.add(scenario)

    logger.info("All %d scenario(s) built. Starting execution...", len(scenario_names))

    with queue:
        results = queue.run_all()

    all_passed = _print_summary(scenario_names, results, output_dir=output_dir)
    sys.exit(0 if all_passed else 1)


def build_scenario(
    cfg: DictConfig,
    *,
    build_scenario_fn: BuildScenarioFn | None = None,
) -> tuple[EgoConfig, BaseScenario]:
    """Instantiate the correct scenario class based on ``cfg.scenario.name``.

    Parameters
    ----------
    cfg:
        Resolved Hydra config containing ``scenario.name`` and related keys.
    build_scenario_fn:
        Optional callable that completely replaces the default registry
        lookup.  When provided, it is called as ``build_scenario_fn(cfg)``
        and its return value is forwarded to the caller.
    """
    if build_scenario_fn is not None:
        ego, scenario = build_scenario_fn(cfg)
        scenario.ego_entity = build_ego_entity(cfg)
        return ego, scenario

    # Validate the name before doing any expensive work.
    scenario_name: str = cfg.scenario.name
    builder = get_scenario_builder(scenario_name)
    if builder is None:
        registered = sorted(get_scenario_registry())
        msg = (
            f"Unknown scenario name: {scenario_name!r}. "
            f"Registered scenarios: {registered}"
        )
        raise ValueError(msg)

    ego, spawn_pose, ground_projection = build_ego_and_spawn(cfg)
    scenario_dict = _to_dict(cfg.scenario)
    scenario = builder(ego, scenario_dict, spawn_pose, ground_projection)
    scenario.ego_entity = build_ego_entity(cfg)
    return ego, scenario


def run_scenario(
    cfg: DictConfig,
    *,
    build_scenario_fn: BuildScenarioFn | None = None,
) -> ScenarioResult:
    """Build and execute a scenario from a resolved Hydra config.

    Parameters
    ----------
    cfg:
        Resolved Hydra config.
    build_scenario_fn:
        Optional callable forwarded to :func:`build_scenario`.

    Returns the :class:`ScenarioResult` so that callers (including Hydra
    multirun) can inspect it without the process being terminated.

    Hydra changes the working directory to its output directory
    (e.g. ``outputs/YYYY-MM-DD/HH-MM-SS/``) before this function is
    called, so all relative paths resolve inside that directory.
    """
    logger.info("Resolved config:\n%s", OmegaConf.to_yaml(cfg))

    _ego, scenario = build_scenario(cfg, build_scenario_fn=build_scenario_fn)

    xodr_path = Path(cfg.map.xodr_path) if cfg.map.get("xodr_path") else None
    lanelet2_path = (
        Path(cfg.map.lanelet2_path) if cfg.map.get("lanelet2_path") else None
    )

    if xodr_path is None:
        # Lanelet2-only map (no .xodr): the map loads without OpenDRIVE, so fail
        # fast -- before CARLA -- if the scenario uses OpenDRIVE-based symbols.
        # They only register at setup() (post-CARLA), so this scans the source.
        import inspect  # noqa: PLC0415

        from autoware_carla_scenario.opendrive_lint import (  # noqa: PLC0415
            check_scenario_source,
        )

        module = inspect.getmodule(type(scenario))
        try:
            source = inspect.getsource(module) if module is not None else None
        except (OSError, TypeError):
            source = None
        if source is not None:
            check_scenario_source(source, type(scenario).__name__)

    cooldown = float(cfg.server.get("cooldown_seconds", 0.0))
    cooldown_max_retries = int(cfg.server.get("cooldown_max_retries", 0))

    # Retrieve the Hydra output directory (works regardless of
    # ``hydra.job.chdir`` which defaults to False since Hydra 1.2).
    output_dir = Path(HydraConfig.get().runtime.output_dir)

    result = run_scenario_with_queue(
        scenario,
        host=cfg.server.host,
        port=cfg.server.port,
        tm_port=cfg.traffic_manager.port,
        xodr_path=xodr_path,
        lanelet2_path=lanelet2_path,
        map_name=cfg.map.name,
        cooldown_seconds=cooldown,
        cooldown_max_retries=cooldown_max_retries,
        output_dir=output_dir,
    )

    status = "PASSED" if result.passed else "FAILED"
    print(f"{status}: {result.message} ({result.elapsed_seconds:.2f}s)")  # noqa: T201
    scenario_name = type(scenario).__name__
    json_path = (output_dir / f"{scenario_name}_result.json").resolve()
    print(f"Result JSON: {json_path}")  # noqa: T201
    return result


@hydra.main(version_base=None, config_path="conf", config_name="config")
def _hydra_main(cfg: DictConfig) -> None:
    """Hydra entry point that dispatches to the selected scenario."""
    logging.basicConfig(
        level=logging.INFO, format="%(levelname)s %(name)s: %(message)s"
    )
    result = run_scenario(cfg)
    # Only exit for single-run mode. In --multirun, Hydra calls this
    # function repeatedly; sys.exit() would kill the entire sweep.
    if not _is_multirun():
        sys.exit(0 if result.passed else 1)


def _extract_resume_from(argv: list[str]) -> tuple[int, list[str]]:
    """Extract ``--resume-from N`` from *argv* and return the value and cleaned argv.

    Returns:
        A 2-tuple of ``(resume_from, remaining_argv)``.
        *resume_from* is 0 when the flag is absent.
    """
    resume_from = 0
    remaining: list[str] = []
    skip_next = False
    for i, arg in enumerate(argv):
        if skip_next:
            skip_next = False
            continue
        if arg == "--resume-from":
            if i + 1 < len(argv):
                try:
                    resume_from = int(argv[i + 1])
                except ValueError:
                    print(  # noqa: T201
                        f"Error: --resume-from requires an integer, got '{argv[i + 1]}'"
                    )
                    sys.exit(1)
                skip_next = True
            else:
                print("Error: --resume-from requires a value")  # noqa: T201
                sys.exit(1)
        elif arg.startswith("--resume-from="):
            try:
                resume_from = int(arg.split("=", 1)[1])
            except ValueError:
                print(  # noqa: T201
                    f"Error: --resume-from requires an integer, got '{arg.split('=', 1)[1]}'"
                )
                sys.exit(1)
        else:
            remaining.append(arg)
    return resume_from, remaining


def main() -> None:
    """CLI entry point: detect glob patterns and dispatch accordingly."""
    # Discover external scenario packages (entry-point plugins) before we touch
    # the config search path or the scenario registry, so their scenarios and
    # conf dirs are available to both the glob and single-run code paths.
    load_scenario_plugins()

    # Extract --resume-from before Hydra sees the argv.
    resume_from, cleaned_argv = _extract_resume_from(sys.argv)
    sys.argv = cleaned_argv

    # Pass via environment variable so the sweeper can read it without
    # going through Hydra's CLI parser (which rejects unknown overrides).
    if resume_from > 0:
        os.environ["SWEEP_RESUME_FROM"] = str(resume_from)

    scenario_value, remaining = _extract_scenario_override(sys.argv)
    if scenario_value is not None and _is_glob_pattern(scenario_value):
        logging.basicConfig(
            level=logging.INFO, format="%(levelname)s %(name)s: %(message)s"
        )
        scenario_names = _resolve_scenario_glob(scenario_value)
        logger.info(
            "Glob matched %d scenario(s): %s",
            len(scenario_names),
            scenario_names,
        )
        run_batch(scenario_names, remaining)
    else:
        # Single run goes through @hydra.main.  External conf dirs are added to
        # the search path by AutowareScenarioSearchPathPlugin (discovered via
        # the hydra_plugins namespace), so nothing needs threading here.
        _hydra_main()


if __name__ == "__main__":
    main()
