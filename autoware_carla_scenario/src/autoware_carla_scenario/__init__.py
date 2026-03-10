"""autoware_carla_scenario – CARLA scenario testing framework.

Public API re-exported for convenience::

    from autoware_carla_scenario import (
        CarlaAutowareScenario,
        CarlaScenarioFixture,
        CarlaServerManager,
        AndCondition,
        BaseCondition,
        BaseScenario,
        CarlaWorldPose,
        CollisionCondition,
        EntityInAreaCondition,
        EntityLanePositionCondition,
        OrCondition,
        EgoConfig,
        EgoVehicle,
        Lanelet2Pose,
        MapManager,
        OpenDrivePose,
        ScenarioQueue,
        ScenarioRecorder,
        ScenarioResult,
        TimeoutCondition,
        to_carla_location,
        to_carla_world,
        to_lanelet2,
        to_opendrive,
        find_nearest_traffic_light,
        frame_of,
        lanelet2_traffic_light_id_to_opendrive_controller_id,
        set_group_traffic_light_state,
        # Kinematics
        CoordinateFrame,
        FrameMismatchError,
        Vector3,
        AbsoluteVelocity,
        RelativeVelocity,
        FrenetVelocity,
        AbsoluteAcceleration,
        RelativeAcceleration,
        FrenetAcceleration,
    )
"""

from .carla_autoware_scenario import CarlaAutowareScenario
from .conditions import (
    AndCondition,
    BaseCondition,
    CollisionCondition,
    EntityInAreaCondition,
    EntityLanePositionCondition,
    OrCondition,
    ScenarioResult,
    TimeoutCondition,
)
from .coordinate import (
    CarlaWorldPose,
    CoordinateFrame,
    FrameMismatchError,
    Lanelet2Pose,
    MapManager,
    OpenDrivePose,
    frame_of,
    to_carla_location,
    to_carla_world,
    to_lanelet2,
    to_opendrive,
)
from .ego import EgoVehicle
from .kinematics import (
    AbsoluteAcceleration,
    AbsoluteVelocity,
    FrenetAcceleration,
    FrenetVelocity,
    RelativeAcceleration,
    RelativeVelocity,
    Vector3,
)
from .pytest_fixtures import CarlaScenarioFixture
from .recording import ScenarioRecorder
from .scenario_base import BaseScenario, EgoConfig
from .scenario_queue import ScenarioQueue
from .server import CarlaServerManager
from .utils import (
    find_nearest_traffic_light,
    lanelet2_traffic_light_id_to_opendrive_controller_id,
    set_group_traffic_light_state,
)

__all__ = [
    "CarlaAutowareScenario",
    "CarlaScenarioFixture",
    "CarlaServerManager",
    "AndCondition",
    "BaseCondition",
    "BaseScenario",
    "CarlaWorldPose",
    "CollisionCondition",
    "EntityInAreaCondition",
    "EntityLanePositionCondition",
    "OrCondition",
    "EgoConfig",
    "EgoVehicle",
    "Lanelet2Pose",
    "MapManager",
    "OpenDrivePose",
    "ScenarioQueue",
    "ScenarioRecorder",
    "ScenarioResult",
    "TimeoutCondition",
    "to_carla_location",
    "to_carla_world",
    "to_lanelet2",
    "to_opendrive",
    "find_nearest_traffic_light",
    "frame_of",
    "lanelet2_traffic_light_id_to_opendrive_controller_id",
    "set_group_traffic_light_state",
    # Kinematics
    "CoordinateFrame",
    "FrameMismatchError",
    "Vector3",
    "AbsoluteVelocity",
    "RelativeVelocity",
    "FrenetVelocity",
    "AbsoluteAcceleration",
    "RelativeAcceleration",
    "FrenetAcceleration",
]
