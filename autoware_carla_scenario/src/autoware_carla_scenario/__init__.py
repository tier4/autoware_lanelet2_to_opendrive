"""autoware_carla_scenario – CARLA scenario testing framework.

Public API re-exported for convenience::

    from autoware_carla_scenario import (
        CarlaAutowareScenario,
        CarlaScenarioFixture,
        CarlaServerManager,
        BaseCondition,
        BaseScenario,
        EgoConfig,
        EgoVehicle,
        ScenarioQueue,
        ScenarioRecorder,
        ScenarioResult,
        TimeoutCondition,
    )
"""

from .carla_autoware_scenario import CarlaAutowareScenario
from .conditions import BaseCondition, ScenarioResult, TimeoutCondition
from .ego import EgoVehicle
from .pytest_fixtures import CarlaScenarioFixture
from .recording import ScenarioRecorder
from .scenario_base import BaseScenario, EgoConfig
from .scenario_queue import ScenarioQueue
from .server import CarlaServerManager

__all__ = [
    "CarlaAutowareScenario",
    "CarlaScenarioFixture",
    "CarlaServerManager",
    "BaseCondition",
    "BaseScenario",
    "EgoConfig",
    "EgoVehicle",
    "ScenarioQueue",
    "ScenarioRecorder",
    "ScenarioResult",
    "TimeoutCondition",
]
