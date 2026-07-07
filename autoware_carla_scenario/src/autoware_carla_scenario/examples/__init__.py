"""Example scenarios for autoware_carla_scenario."""

from .configs import (
    EgoVehicleConfig,
    IntersectionPassingConfig,
    LaneChangeConfig,
    MapConfig,
    ScenarioRunConfig,
    ServerConfig,
    TrafficLightComplianceConfig,
)
from .run import (
    build_ego_and_spawn,
    build_scenario,
    get_conf_dirs,
    get_scenario_registry,
    load_scenario_plugins,
    register_conf_dir,
    register_scenario,
    register_scenario_builder,
    run_scenario,
    run_scenario_with_queue,
)

__all__ = [
    "EgoVehicleConfig",
    "IntersectionPassingConfig",
    "LaneChangeConfig",
    "MapConfig",
    "ScenarioRunConfig",
    "ServerConfig",
    "TrafficLightComplianceConfig",
    # Scenario registry (Issue #420)
    "build_ego_and_spawn",
    "build_scenario",
    "get_conf_dirs",
    "get_scenario_registry",
    "load_scenario_plugins",
    "register_conf_dir",
    "register_scenario",
    "register_scenario_builder",
    "run_scenario",
    "run_scenario_with_queue",
]
