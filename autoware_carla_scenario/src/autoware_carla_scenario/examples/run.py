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
from typing import TYPE_CHECKING

import carla
import hydra
from hydra import compose, initialize_config_dir
from hydra.core.global_hydra import GlobalHydra
from hydra.core.hydra_config import HydraConfig
from omegaconf import DictConfig, OmegaConf

from autoware_carla_scenario import (
    BaseScenario,
    EgoConfig,
    GroundProjectionConfig,
    Lanelet2Pose,
    ScenarioQueue,
    SpawnTransform,
)
from autoware_carla_scenario.conditions import ScenarioResult

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

if TYPE_CHECKING:
    pass

logger = logging.getLogger(__name__)

# Directory containing Hydra config files (conf/ next to this module).
_CONF_DIR = Path(__file__).resolve().parent / "conf"


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


def _resolve_scenario_glob(pattern: str) -> list[str]:
    """Glob ``conf/scenario/{pattern}.yaml`` and return sorted config names.

    Each returned name is a Hydra config path relative to ``conf/scenario/``
    without the ``.yaml`` suffix (e.g. ``"intersection_passing/left_turn"``).
    """
    scenario_dir = _CONF_DIR / "scenario"
    matches = sorted(scenario_dir.glob(f"{pattern}.yaml"))
    if not matches:
        print(  # noqa: T201
            f"Error: no scenario configs match pattern '{pattern}' "
            f"under {scenario_dir}"
        )
        sys.exit(1)
    names: list[str] = []
    for m in matches:
        rel = m.relative_to(scenario_dir).with_suffix("")
        names.append(str(rel))
    return names


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
    scenario_dir = _CONF_DIR / "scenario"
    logger.info(sep)
    logger.info(
        "Batch execution plan: %d scenario(s)%s",
        len(scenario_names),
        f"  (extra overrides: {overrides})" if overrides else "",
    )
    logger.info(sep)

    for i, (name, cfg) in enumerate(zip(scenario_names, configs), 1):
        yaml_path = scenario_dir / f"{name}.yaml"
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


def run_batch(scenario_names: list[str], overrides: list[str]) -> None:
    """Compose configs, build scenarios, and run them in a single queue."""
    configs = [_compose_config(name, overrides) for name in scenario_names]

    # Validate all configs share the same map (shared CARLA server constraint).
    map_names = {str(cfg.map.name) for cfg in configs}
    if len(map_names) > 1:
        print(  # noqa: T201
            f"Error: batch scenarios must share the same map, "
            f"but found: {map_names}"
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
        _ego, scenario = build_scenario(cfg)
        queue.add(scenario)

    logger.info("All %d scenario(s) built. Starting execution...", len(scenario_names))

    with queue:
        results = queue.run_all()

    all_passed = _print_summary(scenario_names, results, output_dir=output_dir)
    sys.exit(0 if all_passed else 1)


def build_scenario(cfg: DictConfig) -> tuple[EgoConfig, BaseScenario]:
    """Instantiate the correct scenario class based on ``cfg.scenario.name``."""
    ground_projection = GroundProjectionConfig(
        ray_distance_upper=float(cfg.entity.ground_projection_ray_distance_upper),
        ray_distance_lower=float(cfg.entity.ground_projection_ray_distance_lower),
    )

    scenario_name: str = cfg.scenario.name
    scenario_dict = _to_dict(cfg.scenario)

    # For lanelet-based scenarios, use a dummy spawn that setup() overwrites.
    ego = EgoConfig(
        spawn_location=SpawnTransform(
            carla.Transform(carla.Location(x=0.0, y=0.0, z=0.0))
        ),
        vehicle_type=cfg.ego.vehicle_type,
        initial_speed_kmh=float(cfg.ego.initial_speed_kmh),
        spawn_retry_max_count=int(cfg.entity.spawn_retry_max_count),
        spawn_retry_t_step=float(cfg.entity.spawn_retry_t_step),
    )

    # Build spawn pose from ego config (lanelet2-specific, kept in examples layer).
    spawn_pose = Lanelet2Pose(
        lanelet_id=cfg.ego.spawn_lanelet_id,
        s=cfg.ego.spawn_s,
    )

    # --- intersection_passing (also handles left_turn / right_turn via turn_direction) ---
    if scenario_name == "intersection_passing":
        return ego, IntersectionPassingScenario(
            ego,
            config=IntersectionPassingConfig(**scenario_dict),
            spawn_pose=spawn_pose,
            ground_projection=ground_projection,
        )

    # --- traffic_light_compliance ---
    if scenario_name == "traffic_light_compliance":
        return ego, TrafficLightComplianceScenario(
            ego,
            config=TrafficLightComplianceConfig(**scenario_dict),
            spawn_pose=spawn_pose,
            ground_projection=ground_projection,
        )

    # --- lane_change ---
    if scenario_name == "lane_change":
        return ego, LaneChangeScenario(
            ego,
            config=LaneChangeConfig(**scenario_dict),
            spawn_pose=spawn_pose,
            ground_projection=ground_projection,
        )

    # --- temporary_stop ---
    if scenario_name == "temporary_stop":
        return ego, TemporaryStopScenario(
            ego,
            config=TemporaryStopConfig(**scenario_dict),
            spawn_pose=spawn_pose,
            ground_projection=ground_projection,
        )

    msg = f"Unknown scenario name: {scenario_name!r}"
    raise ValueError(msg)


def run_scenario(cfg: DictConfig) -> ScenarioResult:
    """Build and execute a scenario from a resolved Hydra config.

    Returns the :class:`ScenarioResult` so that callers (including Hydra
    multirun) can inspect it without the process being terminated.

    Hydra changes the working directory to its output directory
    (e.g. ``outputs/YYYY-MM-DD/HH-MM-SS/``) before this function is
    called, so all relative paths resolve inside that directory.
    """
    logger.info("Resolved config:\n%s", OmegaConf.to_yaml(cfg))

    _ego, scenario = build_scenario(cfg)

    # Build optional paths
    xodr_path = Path(cfg.map.xodr_path) if cfg.map.get("xodr_path") else None
    lanelet2_path = (
        Path(cfg.map.lanelet2_path) if cfg.map.get("lanelet2_path") else None
    )

    cooldown = float(cfg.server.get("cooldown_seconds", 0.0))
    cooldown_max_retries = int(cfg.server.get("cooldown_max_retries", 0))

    # Retrieve the Hydra output directory (works regardless of
    # ``hydra.job.chdir`` which defaults to False since Hydra 1.2).
    output_dir = Path(HydraConfig.get().runtime.output_dir)

    queue = ScenarioQueue(
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
    queue.add(scenario)

    with queue:
        results = queue.run_all()

    result = results[0]
    status = "PASSED" if result.passed else "FAILED"
    print(f"{status}: {result.message} ({result.elapsed_seconds:.2f}s)")  # noqa: T201
    # JSON is already written by ScenarioRunner; print the absolute path.
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
        _hydra_main()


if __name__ == "__main__":
    main()
