"""Abstract base class and configuration dataclass for LiDAR sensors.

This module defines the simulator-agnostic LiDAR interface. Concrete
implementations subclass :class:`LidarSensorBase` and extend
:class:`LidarSensorConfig` with simulator-specific attributes.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import TYPE_CHECKING, Optional

import numpy as np
from numpy.typing import NDArray

if TYPE_CHECKING:
    import carla


@dataclass(frozen=True)
class LidarSensorConfig:
    """Simulator-agnostic spinning-LiDAR configuration.

    Groups the parameters common to *any* spinning LiDAR sensor:

    * **Line count** -- ``n_rows`` beam channels per revolution.
    * **Azimuth resolution** -- ``n_columns`` samples per full revolution.
    * **Frame rate** -- ``fps`` (full revolutions per second).
    * **Range gating** -- ``min_range_m`` and ``max_range_m`` in metres.
    * **Extrinsics** -- 6-DOF pose relative to the vehicle ``base_link``
      frame expressed as ``(position_x, position_y, position_z)`` in metres
      and ``(roll, pitch, yaw)`` in degrees.
    """

    # -- Beam geometry --------------------------------------------------------
    n_rows: int = 128
    """Number of beam channels (vertical resolution)."""

    n_columns: int = 2048
    """Number of azimuth samples per full revolution (horizontal resolution)."""

    # -- Timing ---------------------------------------------------------------
    fps: float = 10.0
    """Full revolutions per second."""

    # -- Range gating ---------------------------------------------------------
    min_range_m: float = 0.3
    """Minimum valid return range in metres (returns closer are discarded)."""

    max_range_m: float = 120.0
    """Maximum valid return range in metres (returns farther are discarded)."""

    # -- Extrinsics (base_link -> sensor) -------------------------------------
    position_x: float = 0.0
    """Forward offset from ``base_link`` in metres (positive = forward)."""

    position_y: float = 0.0
    """Lateral offset from ``base_link`` in metres (positive = left)."""

    position_z: float = 0.0
    """Vertical offset from ``base_link`` in metres (positive = up)."""

    roll: float = 0.0
    """Roll angle in degrees."""

    pitch: float = 0.0
    """Pitch angle in degrees (negative = look down)."""

    yaw: float = 0.0
    """Yaw angle in degrees."""


class LidarSensorBase(ABC):
    """Abstract base for LiDAR sensors attached to a CARLA actor.

    Subclasses must implement :meth:`attach`, :meth:`destroy`, and
    :meth:`get_point_cloud`.

    Args:
        config: LiDAR configuration dataclass.
    """

    def __init__(self, config: LidarSensorConfig) -> None:
        self._config = config
        self._attached = False

    @property
    def config(self) -> LidarSensorConfig:
        """Return the LiDAR configuration."""
        return self._config

    @property
    def attached(self) -> bool:
        """Whether the sensor is currently attached to an actor."""
        return self._attached

    @abstractmethod
    def attach(self, world: "carla.World", actor: "carla.Actor") -> None:
        """Attach the sensor to *actor* in *world*.

        Args:
            world: The CARLA world instance.
            actor: The actor to attach the sensor to.
        """
        ...

    @abstractmethod
    def destroy(self) -> None:
        """Destroy the sensor actor and release resources."""
        ...

    @abstractmethod
    def get_point_cloud(self) -> Optional[NDArray[np.float32]]:
        """Return the latest point cloud, or ``None`` if none is available yet.

        Returns:
            A NumPy array of shape ``(N, 4)`` with columns
            ``[x, y, z, intensity]`` in the sensor frame (metres for xyz,
            unitless in ``[0, 1]`` for intensity), or ``None``.
        """
        ...
