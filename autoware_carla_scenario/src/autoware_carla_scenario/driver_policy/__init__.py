"""driver_policy – drive the ego from an alpasim-compatible egodriver policy.

The ego is controlled by an external driver process that serves
``egodriver.EgodriverService`` (the alpasim driver contract), for example one
started with ``carla_driver_interface``::

    carla-driver-interface serve --policy route_follower --port 50051

The scenario side plays alpasim's Runtime role: once per policy step it sends
camera images, egomotion, the route ahead and CARLA ground truth, asks the
driver for a plan, and tracks it with pure pursuit and a speed PID.

Usage::

    from autoware_carla_scenario.driver_policy import (
        EgoDriverEntity,
        EgoDriverPolicyConfig,
    )

    scenario = MyScenario(
        ego_config,
        ego_type=EgoDriverEntity.factory(
            EgoDriverPolicyConfig(driver_address="localhost:50051")
        ),
    )
"""

from .config import CameraConfig, EgoDriverPolicyConfig, default_camera_rig
from .control import ControlConfig
from .entity import EgoDriverEntity
from .policy import EgoDriverPolicy

__all__ = [
    "CameraConfig",
    "ControlConfig",
    "EgoDriverEntity",
    "EgoDriverPolicy",
    "EgoDriverPolicyConfig",
    "default_camera_rig",
]
