"""Hydra-based unified entry point for all example scenarios.

Usage examples::

    # Basic: run spawn-and-idle with a specific map
    uv run scenario scenario=spawn_and_idle map.name=Town10HD_Opt

    # Select a different scenario
    uv run scenario scenario=left_turn map.name=NishishinjyukuMap

    # Override a target environment
    uv run scenario scenario=left_turn target=nishishinjuku

    # Override individual parameters
    uv run scenario scenario=left_turn map.name=NishishinjyukuMap scenario.timeout_seconds=15.0

    # Override server connection
    uv run scenario scenario=left_turn map.name=X server.host=192.168.1.100 server.port=3000
"""

from __future__ import annotations

import logging
import sys
from pathlib import Path
from typing import TYPE_CHECKING

import carla
import hydra
from omegaconf import DictConfig, OmegaConf

from autoware_carla_scenario import (
    BaseScenario,
    EgoConfig,
    ScenarioQueue,
    SpawnPointIndex,
    SpawnTransform,
)

from .configs import (
    IntersectionPassingConfig,
    LeftTurnConfig,
    SpawnAndIdleConfig,
    TrafficLightComplianceConfig,
)
from .intersection_passing import IntersectionPassingScenario
from .left_turn import LeftTurnScenario
from .spawn_and_idle import SpawnAndIdleScenario
from .traffic_light_compliance import TrafficLightComplianceScenario

if TYPE_CHECKING:
    pass

logger = logging.getLogger(__name__)


def _to_dict(cfg_node: DictConfig) -> dict:  # type: ignore[type-arg]
    """Convert an OmegaConf node to a plain dict (typed helper)."""
    container = OmegaConf.to_container(cfg_node, resolve=True)
    assert isinstance(container, dict)  # noqa: S101
    return container


def build_scenario(cfg: DictConfig) -> tuple[EgoConfig, BaseScenario]:
    """Instantiate the correct scenario class based on ``cfg.scenario.name``."""
    scenario_name: str = cfg.scenario.name
    scenario_dict = _to_dict(cfg.scenario)

    # --- spawn_and_idle ---
    if scenario_name == "spawn_and_idle":
        spawn_index = int(cfg.scenario.get("spawn_index", 0))
        ego = EgoConfig(
            spawn_location=SpawnPointIndex(spawn_index),
            vehicle_type=cfg.ego.vehicle_type,
            initial_speed_kmh=float(cfg.ego.initial_speed_kmh),
        )
        return ego, SpawnAndIdleScenario(
            ego, config=SpawnAndIdleConfig(**scenario_dict)
        )

    # For lanelet-based scenarios, use a dummy spawn that setup() overwrites.
    ego = EgoConfig(
        spawn_location=SpawnTransform(
            carla.Transform(carla.Location(x=0.0, y=0.0, z=0.0))
        ),
        vehicle_type=cfg.ego.vehicle_type,
        initial_speed_kmh=float(cfg.ego.initial_speed_kmh),
    )

    # --- left_turn ---
    if scenario_name == "left_turn":
        return ego, LeftTurnScenario(
            ego,
            host=cfg.server.host,
            port=cfg.server.port,
            config=LeftTurnConfig(**scenario_dict),
        )

    # --- intersection_passing ---
    if scenario_name == "intersection_passing":
        return ego, IntersectionPassingScenario(
            ego, config=IntersectionPassingConfig(**scenario_dict)
        )

    # --- traffic_light_compliance ---
    if scenario_name == "traffic_light_compliance":
        return ego, TrafficLightComplianceScenario(
            ego, config=TrafficLightComplianceConfig(**scenario_dict)
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
def main(cfg: DictConfig) -> None:
    """Hydra entry point that dispatches to the selected scenario."""
    logging.basicConfig(
        level=logging.INFO, format="%(levelname)s %(name)s: %(message)s"
    )
    run_scenario(cfg)


if __name__ == "__main__":
    main()
