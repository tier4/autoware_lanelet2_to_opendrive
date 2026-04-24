"""Example scenarios for autoware_carla_scenario."""

from .configs import (
    EgoVehicleConfig,
    IntersectionPassingConfig,
    LaneChangeConfig,
    MapConfig,
    ScenarioRunConfig,
    ServerConfig,
    SimulationConfig,
    TrafficLightComplianceConfig,
)
from .run import (
    build_ego_and_spawn,
    build_scenario,
    get_scenario_registry,
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
    "SimulationConfig",
    "TrafficLightComplianceConfig",
    # Scenario registry (Issue #420)
    "build_ego_and_spawn",
    "build_scenario",
    "get_scenario_registry",
    "register_scenario",
    "register_scenario_builder",
    "run_scenario",
    "run_scenario_with_queue",
]
