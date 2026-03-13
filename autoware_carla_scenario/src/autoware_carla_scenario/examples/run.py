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
import sys
from pathlib import Path
from typing import TYPE_CHECKING

import carla
import hydra
from hydra import compose, initialize_config_dir
from hydra.core.global_hydra import GlobalHydra
from omegaconf import DictConfig, OmegaConf

from autoware_carla_scenario import (
    BaseScenario,
    EgoConfig,
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


def _print_summary(names: list[str], results: list[ScenarioResult]) -> bool:
    """Print a formatted result table and return ``True`` if all passed."""
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
            for cs in result.condition_statuses:
                mark = "OK" if cs.satisfied else "NG"
                print(f"    [{mark}] {cs.label}: {cs.message}")  # noqa: T201
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

    queue = ScenarioQueue(
        host=first_cfg.server.host,
        port=first_cfg.server.port,
        tm_port=first_cfg.traffic_manager.port,
        xodr_path=xodr_path,
        lanelet2_path=lanelet2_path,
        map_name=first_cfg.map.name,
    )

    for i, (name, cfg) in enumerate(zip(scenario_names, configs), 1):
        logger.info("Building scenario [%d/%d]: %s", i, len(scenario_names), name)
        _ego, scenario = build_scenario(cfg)
        queue.add(scenario)

    logger.info("All %d scenario(s) built. Starting execution...", len(scenario_names))

    with queue:
        results = queue.run_all()

    all_passed = _print_summary(scenario_names, results)
    sys.exit(0 if all_passed else 1)


def build_scenario(cfg: DictConfig) -> tuple[EgoConfig, BaseScenario]:
    """Instantiate the correct scenario class based on ``cfg.scenario.name``."""
    scenario_name: str = cfg.scenario.name
    scenario_dict = _to_dict(cfg.scenario)

    # For lanelet-based scenarios, use a dummy spawn that setup() overwrites.
    ego = EgoConfig(
        spawn_location=SpawnTransform(
            carla.Transform(carla.Location(x=0.0, y=0.0, z=0.0))
        ),
        vehicle_type=cfg.ego.vehicle_type,
        initial_speed_kmh=float(cfg.ego.initial_speed_kmh),
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
        )

    # --- traffic_light_compliance ---
    if scenario_name == "traffic_light_compliance":
        return ego, TrafficLightComplianceScenario(
            ego,
            config=TrafficLightComplianceConfig(**scenario_dict),
            spawn_pose=spawn_pose,
        )

    # --- lane_change ---
    if scenario_name == "lane_change":
        return ego, LaneChangeScenario(
            ego,
            config=LaneChangeConfig(**scenario_dict),
            spawn_pose=spawn_pose,
        )

    # --- temporary_stop ---
    if scenario_name == "temporary_stop":
        return ego, TemporaryStopScenario(
            ego,
            config=TemporaryStopConfig(**scenario_dict),
            spawn_pose=spawn_pose,
        )

    msg = f"Unknown scenario name: {scenario_name!r}"
    raise ValueError(msg)


def run_scenario(cfg: DictConfig) -> None:
    """Build and execute a scenario from a resolved Hydra config."""
    logger.info("Resolved config:\n%s", OmegaConf.to_yaml(cfg))

    _ego, scenario = build_scenario(cfg)

    # Build optional paths
    xodr_path = Path(cfg.map.xodr_path) if cfg.map.get("xodr_path") else None
    lanelet2_path = (
        Path(cfg.map.lanelet2_path) if cfg.map.get("lanelet2_path") else None
    )

    queue = ScenarioQueue(
        host=cfg.server.host,
        port=cfg.server.port,
        tm_port=cfg.traffic_manager.port,
        xodr_path=xodr_path,
        lanelet2_path=lanelet2_path,
        map_name=cfg.map.name,
    )
    queue.add(scenario)

    with queue:
        results = queue.run_all()

    result = results[0]
    print(result)  # noqa: T201
    sys.exit(0 if result.passed else 1)


@hydra.main(version_base=None, config_path="conf", config_name="config")
def _hydra_main(cfg: DictConfig) -> None:
    """Hydra entry point that dispatches to the selected scenario."""
    logging.basicConfig(
        level=logging.INFO, format="%(levelname)s %(name)s: %(message)s"
    )
    run_scenario(cfg)


def main() -> None:
    """CLI entry point: detect glob patterns and dispatch accordingly."""
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
