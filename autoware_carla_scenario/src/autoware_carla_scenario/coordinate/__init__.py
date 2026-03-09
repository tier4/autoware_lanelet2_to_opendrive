"""coordinate – mutual conversion between Lanelet2, OpenDRIVE, and CARLA world poses.

Usage::

    from autoware_carla_scenario.coordinate import (
        CarlaWorldPose, Lanelet2Pose, OpenDrivePose,
        MapManager,
        to_carla_world, to_lanelet2, to_opendrive,
    )

    # Initialize maps once
    mm = MapManager.get_instance()
    mm.initialize(xodr_path=Path("map.xodr"), lanelet2_path=Path("map.osm"))

    # Convert poses
    carla_pose = to_carla_world(Lanelet2Pose(lanelet_id=1234, s=10.0, t=0.5))
    od_pose    = to_opendrive(carla_pose)
    ll2_pose   = to_lanelet2(od_pose)
"""

from .map_manager import MapManager
from .poses import AnyPose, CarlaWorldPose, Lanelet2Pose, OpenDrivePose
from .transform import to_carla_world, to_lanelet2, to_opendrive

__all__ = [
    "AnyPose",
    "CarlaWorldPose",
    "Lanelet2Pose",
    "OpenDrivePose",
    "MapManager",
    "to_carla_world",
    "to_lanelet2",
    "to_opendrive",
]
