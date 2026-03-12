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

from .frames import CoordinateFrame, FrameMismatchError, frame_of
from .map_manager import MapManager
from .poses import AnyPose, CarlaWorldPose, Lanelet2Pose, OpenDrivePose
from .snap import snap_to_carla_road
from .stop_line import get_stop_line_poses
from .transform import (
    project_onto_road,
    to_carla_location,
    to_carla_world,
    to_lanelet2,
    to_opendrive,
)

__all__ = [
    "AnyPose",
    "CarlaWorldPose",
    "CoordinateFrame",
    "FrameMismatchError",
    "Lanelet2Pose",
    "OpenDrivePose",
    "MapManager",
    "frame_of",
    "get_stop_line_poses",
    "project_onto_road",
    "snap_to_carla_road",
    "to_carla_location",
    "to_carla_world",
    "to_lanelet2",
    "to_opendrive",
]
