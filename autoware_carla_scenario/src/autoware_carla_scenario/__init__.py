"""autoware_carla_scenario – CARLA scenario testing framework.

Public API re-exported for convenience::

    from autoware_carla_scenario import (
        AutowareEntity,
        EGO_ROLE_NAME,
        EntityRole,
        ScenarioRunner,
        CarlaScenarioFixture,
        CarlaServerManager,
        AndCondition,
        BaseCondition,
        BaseScenario,
        CarlaWorldPose,
        CollisionCondition,
        ComparisonRule,
        ElapsedTimeCondition,
        EntityInAreaCondition,
        EntityLanePositionCondition,
        NotCondition,
        OrCondition,
        SpeedCondition,
        SpeedCoordinateSystem,
        SpeedDirection,
        StandstillCondition,
        StickyCondition,
        EgoConfig,
        EgoVehicle,
        Lanelet2Pose,
        MapManager,
        OpenDrivePose,
        ScenarioQueue,
        ScenarioResult,
        TimeoutCondition,
        to_carla_location,
        to_carla_world,
        to_lanelet2,
        to_opendrive,
        find_nearest_traffic_light,
        frame_of,
        lanelet2_traffic_light_id_to_opendrive_controller_id,
        find_actor_by_role_name,
        TrafficLightTarget,
        TrafficSignalAction,
        # Entity
        SpawnLocation,
        SpawnPointIndex,
        SpawnTransform,
        VehicleEntity,
        VehicleEntityConfig,
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

Imports are deferred (PEP 562) so that lightweight subpackages such as
``autoware_carla_scenario.ui`` can be imported without pulling in heavy
native dependencies (CARLA, lanelet2).
"""

from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .actions import (
        AttachCameraSensorAction as AttachCameraSensorAction,
        AttachCarlaCameraSensorAction as AttachCarlaCameraSensorAction,
        BaseAction as BaseAction,
        LaneChangeAction as LaneChangeAction,
        LaneChangeDirection as LaneChangeDirection,
        TickTiming as TickTiming,
        TrafficLightTarget as TrafficLightTarget,
        TrafficSignalAction as TrafficSignalAction,
        TurnAction as TurnAction,
        TurnDirection as TurnDirection,
    )
    from .camera_recorder import CameraRecorder as CameraRecorder
    from .conditions import (
        AlwaysTrueCondition as AlwaysTrueCondition,
        AndCondition as AndCondition,
        BaseCondition as BaseCondition,
        CollisionCondition as CollisionCondition,
        ComparisonRule as ComparisonRule,
        ElapsedTimeCondition as ElapsedTimeCondition,
        EntityExistenceCondition as EntityExistenceCondition,
        EntityInAreaCondition as EntityInAreaCondition,
        EntityLanePositionCondition as EntityLanePositionCondition,
        NotCondition as NotCondition,
        OrCondition as OrCondition,
        PersistentCondition as PersistentCondition,
        ScalarComparisonRule as ScalarComparisonRule,
        ScenarioResult as ScenarioResult,
        SpeedCondition as SpeedCondition,
        SpeedCoordinateSystem as SpeedCoordinateSystem,
        SpeedDirection as SpeedDirection,
        StandstillCondition as StandstillCondition,
        StickyCondition as StickyCondition,
        TemporaryStopCondition as TemporaryStopCondition,
        TimeoutCondition as TimeoutCondition,
        TrafficSignalCondition as TrafficSignalCondition,
        WaypointCheckType as WaypointCheckType,
        WaypointCondition as WaypointCondition,
        find_actor_by_role_name as find_actor_by_role_name,
    )
    from .constants import EGO_ROLE_NAME as EGO_ROLE_NAME
    from .coordinate import (
        CarlaWorldPose as CarlaWorldPose,
        CoordinateFrame as CoordinateFrame,
        FrameMismatchError as FrameMismatchError,
        GroundProjectionConfig as GroundProjectionConfig,
        Lanelet2Pose as Lanelet2Pose,
        MapManager as MapManager,
        OpenDrivePose as OpenDrivePose,
        frame_of as frame_of,
        get_stop_line_poses as get_stop_line_poses,
        get_stop_line_poses_with_following as get_stop_line_poses_with_following,
        snap_to_carla_road as snap_to_carla_road,
        to_carla_location as to_carla_location,
        to_carla_world as to_carla_world,
        to_lanelet2 as to_lanelet2,
        to_opendrive as to_opendrive,
    )
    from .entity import (
        AutowareEntity as AutowareEntity,
        EgoVehicle as EgoVehicle,
        SpawnLocation as SpawnLocation,
        SpawnPointIndex as SpawnPointIndex,
        SpawnTransform as SpawnTransform,
        VehicleEntity as VehicleEntity,
        VehicleEntityConfig as VehicleEntityConfig,
    )
    from .entity_role import EntityRole as EntityRole
    from .kinematics import (
        AbsoluteAcceleration as AbsoluteAcceleration,
        AbsoluteVelocity as AbsoluteVelocity,
        FrenetAcceleration as FrenetAcceleration,
        FrenetVelocity as FrenetVelocity,
        RelativeAcceleration as RelativeAcceleration,
        RelativeVelocity as RelativeVelocity,
        Vector3 as Vector3,
    )
    from .pytest_fixtures import CarlaScenarioFixture as CarlaScenarioFixture
    from .scenario_base import BaseScenario as BaseScenario, EgoConfig as EgoConfig
    from .scenario_queue import ScenarioQueue as ScenarioQueue
    from .scenario_runner import ScenarioRunner as ScenarioRunner
    from .sensor import (
        CameraSensorBase as CameraSensorBase,
        CameraSensorConfig as CameraSensorConfig,
        CarlaCameraSensor as CarlaCameraSensor,
        CarlaCameraSensorConfig as CarlaCameraSensorConfig,
    )
    from .server import CarlaServerManager as CarlaServerManager
    from .utils import (
        find_nearest_traffic_light as find_nearest_traffic_light,
        get_stop_line_linestrings as get_stop_line_linestrings,
        lanelet2_traffic_light_id_to_opendrive_controller_id as lanelet2_traffic_light_id_to_opendrive_controller_id,
    )

__all__ = [
    "EGO_ROLE_NAME",
    "EntityRole",
    "ScenarioRunner",
    "CarlaScenarioFixture",
    "CarlaServerManager",
    "EntityExistenceCondition",
    "AlwaysTrueCondition",
    "AndCondition",
    "BaseCondition",
    "BaseScenario",
    "CarlaWorldPose",
    "CollisionCondition",
    "ComparisonRule",
    "ElapsedTimeCondition",
    "EntityInAreaCondition",
    "EntityLanePositionCondition",
    "NotCondition",
    "OrCondition",
    "PersistentCondition",
    "ScalarComparisonRule",
    "SpeedCondition",
    "SpeedCoordinateSystem",
    "SpeedDirection",
    "StandstillCondition",
    "StickyCondition",
    "TemporaryStopCondition",
    "AutowareEntity",
    "EgoConfig",
    "EgoVehicle",
    "SpawnLocation",
    "SpawnPointIndex",
    "SpawnTransform",
    "VehicleEntity",
    "VehicleEntityConfig",
    "Lanelet2Pose",
    "MapManager",
    "OpenDrivePose",
    "ScenarioQueue",
    "ScenarioResult",
    "TimeoutCondition",
    "to_carla_location",
    "to_carla_world",
    "to_lanelet2",
    "to_opendrive",
    "find_nearest_traffic_light",
    "frame_of",
    "get_stop_line_linestrings",
    "get_stop_line_poses",
    "get_stop_line_poses_with_following",
    "GroundProjectionConfig",
    "snap_to_carla_road",
    "lanelet2_traffic_light_id_to_opendrive_controller_id",
    "find_actor_by_role_name",
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
    # Actions
    "AttachCameraSensorAction",
    "AttachCarlaCameraSensorAction",
    "BaseAction",
    "LaneChangeAction",
    "LaneChangeDirection",
    "TickTiming",
    "TrafficLightTarget",
    "TrafficSignalAction",
    "TrafficSignalCondition",
    "WaypointCheckType",
    "WaypointCondition",
    "TurnAction",
    "TurnDirection",
    # Sensors
    "CameraRecorder",
    "CameraSensorBase",
    "CameraSensorConfig",
    "CarlaCameraSensor",
    "CarlaCameraSensorConfig",
]

# ---------------------------------------------------------------------------
# Lazy import mapping: name -> (module_path, attribute_name)
# ---------------------------------------------------------------------------

_LAZY_IMPORTS: dict[str, tuple[str, str]] = {
    # actions
    "AttachCameraSensorAction": (".actions", "AttachCameraSensorAction"),
    "AttachCarlaCameraSensorAction": (".actions", "AttachCarlaCameraSensorAction"),
    "BaseAction": (".actions", "BaseAction"),
    "LaneChangeAction": (".actions", "LaneChangeAction"),
    "LaneChangeDirection": (".actions", "LaneChangeDirection"),
    "TickTiming": (".actions", "TickTiming"),
    "TrafficLightTarget": (".actions", "TrafficLightTarget"),
    "TrafficSignalAction": (".actions", "TrafficSignalAction"),
    "TurnAction": (".actions", "TurnAction"),
    "TurnDirection": (".actions", "TurnDirection"),
    # camera / sensor
    "CameraRecorder": (".camera_recorder", "CameraRecorder"),
    "CameraSensorBase": (".sensor", "CameraSensorBase"),
    "CameraSensorConfig": (".sensor", "CameraSensorConfig"),
    "CarlaCameraSensor": (".sensor", "CarlaCameraSensor"),
    "CarlaCameraSensorConfig": (".sensor", "CarlaCameraSensorConfig"),
    # scenario runner / queue / server
    "ScenarioRunner": (".scenario_runner", "ScenarioRunner"),
    "ScenarioQueue": (".scenario_queue", "ScenarioQueue"),
    "CarlaServerManager": (".server", "CarlaServerManager"),
    # conditions
    "EntityExistenceCondition": (".conditions", "EntityExistenceCondition"),
    "AlwaysTrueCondition": (".conditions", "AlwaysTrueCondition"),
    "AndCondition": (".conditions", "AndCondition"),
    "BaseCondition": (".conditions", "BaseCondition"),
    "CollisionCondition": (".conditions", "CollisionCondition"),
    "ComparisonRule": (".conditions", "ComparisonRule"),
    "ElapsedTimeCondition": (".conditions", "ElapsedTimeCondition"),
    "EntityInAreaCondition": (".conditions", "EntityInAreaCondition"),
    "EntityLanePositionCondition": (".conditions", "EntityLanePositionCondition"),
    "NotCondition": (".conditions", "NotCondition"),
    "OrCondition": (".conditions", "OrCondition"),
    "PersistentCondition": (".conditions", "PersistentCondition"),
    "ScalarComparisonRule": (".conditions", "ScalarComparisonRule"),
    "ScenarioResult": (".conditions", "ScenarioResult"),
    "SpeedCondition": (".conditions", "SpeedCondition"),
    "SpeedCoordinateSystem": (".conditions", "SpeedCoordinateSystem"),
    "SpeedDirection": (".conditions", "SpeedDirection"),
    "StandstillCondition": (".conditions", "StandstillCondition"),
    "StickyCondition": (".conditions", "StickyCondition"),
    "TemporaryStopCondition": (".conditions", "TemporaryStopCondition"),
    "TimeoutCondition": (".conditions", "TimeoutCondition"),
    "TrafficSignalCondition": (".conditions", "TrafficSignalCondition"),
    "WaypointCheckType": (".conditions", "WaypointCheckType"),
    "WaypointCondition": (".conditions", "WaypointCondition"),
    "find_actor_by_role_name": (".conditions", "find_actor_by_role_name"),
    # constants / entity_role
    "EGO_ROLE_NAME": (".constants", "EGO_ROLE_NAME"),
    "EntityRole": (".entity_role", "EntityRole"),
    # coordinate
    "CarlaWorldPose": (".coordinate", "CarlaWorldPose"),
    "CoordinateFrame": (".coordinate", "CoordinateFrame"),
    "FrameMismatchError": (".coordinate", "FrameMismatchError"),
    "GroundProjectionConfig": (".coordinate", "GroundProjectionConfig"),
    "Lanelet2Pose": (".coordinate", "Lanelet2Pose"),
    "MapManager": (".coordinate", "MapManager"),
    "OpenDrivePose": (".coordinate", "OpenDrivePose"),
    "frame_of": (".coordinate", "frame_of"),
    "get_stop_line_poses": (".coordinate", "get_stop_line_poses"),
    "get_stop_line_poses_with_following": (
        ".coordinate",
        "get_stop_line_poses_with_following",
    ),
    "snap_to_carla_road": (".coordinate", "snap_to_carla_road"),
    "to_carla_location": (".coordinate", "to_carla_location"),
    "to_carla_world": (".coordinate", "to_carla_world"),
    "to_lanelet2": (".coordinate", "to_lanelet2"),
    "to_opendrive": (".coordinate", "to_opendrive"),
    # entity
    "AutowareEntity": (".entity", "AutowareEntity"),
    "EgoVehicle": (".entity", "EgoVehicle"),
    "SpawnLocation": (".entity", "SpawnLocation"),
    "SpawnPointIndex": (".entity", "SpawnPointIndex"),
    "SpawnTransform": (".entity", "SpawnTransform"),
    "VehicleEntity": (".entity", "VehicleEntity"),
    "VehicleEntityConfig": (".entity", "VehicleEntityConfig"),
    # kinematics
    "AbsoluteAcceleration": (".kinematics", "AbsoluteAcceleration"),
    "AbsoluteVelocity": (".kinematics", "AbsoluteVelocity"),
    "FrenetAcceleration": (".kinematics", "FrenetAcceleration"),
    "FrenetVelocity": (".kinematics", "FrenetVelocity"),
    "RelativeAcceleration": (".kinematics", "RelativeAcceleration"),
    "RelativeVelocity": (".kinematics", "RelativeVelocity"),
    "Vector3": (".kinematics", "Vector3"),
    # pytest
    "CarlaScenarioFixture": (".pytest_fixtures", "CarlaScenarioFixture"),
    # scenario base
    "BaseScenario": (".scenario_base", "BaseScenario"),
    "EgoConfig": (".scenario_base", "EgoConfig"),
    # utils
    "find_nearest_traffic_light": (".utils", "find_nearest_traffic_light"),
    "get_stop_line_linestrings": (".utils", "get_stop_line_linestrings"),
    "lanelet2_traffic_light_id_to_opendrive_controller_id": (
        ".utils",
        "lanelet2_traffic_light_id_to_opendrive_controller_id",
    ),
}


def __getattr__(name: str) -> object:
    """Lazily import public symbols on first access (PEP 562)."""
    if name in _LAZY_IMPORTS:
        module_path, attr = _LAZY_IMPORTS[name]
        import importlib  # noqa: PLC0415

        mod = importlib.import_module(module_path, __name__)
        value = getattr(mod, attr)
        # Cache on the module so subsequent lookups are fast.
        globals()[name] = value
        return value
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
