"""Sensor abstractions for CARLA scenario testing."""

from .base import CameraSensorBase, CameraSensorConfig
from .carla_camera import CarlaCameraSensor, CarlaCameraSensorConfig

__all__ = [
    "CameraSensorBase",
    "CameraSensorConfig",
    "CarlaCameraSensor",
    "CarlaCameraSensorConfig",
]
