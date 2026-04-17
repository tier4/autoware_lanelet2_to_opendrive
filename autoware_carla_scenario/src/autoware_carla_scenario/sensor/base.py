"""Abstract base class and configuration dataclass for camera sensors.

This module defines the simulator-agnostic camera interface.  Concrete
implementations (e.g. :class:`CarlaCameraSensor`) subclass
:class:`CameraSensorBase` and extend :class:`CameraSensorConfig` with
simulator-specific attributes.
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import TYPE_CHECKING, Optional

import numpy as np
from numpy.typing import NDArray

if TYPE_CHECKING:
    import carla


@dataclass(frozen=True)
class CameraSensorConfig:
    """Simulator-agnostic camera configuration.

    Groups the parameters common to *any* pinhole camera sensor:

    * **Resolution** -- ``image_width`` and ``image_height`` in pixels.
    * **Field of view** -- horizontal ``fov`` in degrees.
    * **Frame rate** -- ``fps`` (frames per second).
    * **Extrinsics** -- 6-DOF pose relative to the vehicle ``base_link``
      frame expressed as ``(position_x, position_y, position_z)`` in metres
      and ``(roll, pitch, yaw)`` in degrees.

    Intrinsic matrix entries (``fx``, ``fy``, ``cx``, ``cy``) are derived
    automatically from resolution and FOV via :attr:`intrinsic_matrix`.
    """

    # -- Resolution -----------------------------------------------------------
    image_width: int = 1920
    """Image width in pixels."""

    image_height: int = 1080
    """Image height in pixels."""

    # -- Optics ---------------------------------------------------------------
    fov: float = 90.0
    """Horizontal field of view in degrees."""

    # -- Timing ---------------------------------------------------------------
    fps: float = 20.0
    """Sensor capture rate in frames per second."""

    # -- Extrinsics (base_link -> camera) -------------------------------------
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

    # -- Derived intrinsic helpers -------------------------------------------

    @property
    def fx(self) -> float:
        """Horizontal focal length in pixels derived from FOV and width."""
        return self.image_width / (2.0 * math.tan(math.radians(self.fov / 2.0)))

    @property
    def fy(self) -> float:
        """Vertical focal length in pixels (square-pixel assumption)."""
        return self.fx

    @property
    def cx(self) -> float:
        """Principal point x-coordinate (image centre)."""
        return self.image_width / 2.0

    @property
    def cy(self) -> float:
        """Principal point y-coordinate (image centre)."""
        return self.image_height / 2.0

    @property
    def intrinsic_matrix(self) -> NDArray[np.float64]:
        """3x3 pinhole camera intrinsic matrix (K).

        .. math::

            K = \\begin{bmatrix}
                f_x & 0   & c_x \\\\
                0   & f_y & c_y \\\\
                0   & 0   & 1
            \\end{bmatrix}
        """
        return np.array(
            [
                [self.fx, 0.0, self.cx],
                [0.0, self.fy, self.cy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )


class CameraSensorBase(ABC):
    """Abstract base for camera sensors attached to a CARLA actor.

    Subclasses must implement :meth:`attach` and :meth:`destroy`.

    Args:
        config: Camera configuration dataclass.
    """

    def __init__(self, config: CameraSensorConfig) -> None:
        self._config = config
        self._attached = False

    @property
    def config(self) -> CameraSensorConfig:
        """Return the camera configuration."""
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
    def get_image(self) -> Optional[NDArray[np.uint8]]:
        """Return the latest captured frame as an HxWx3 BGR array, or ``None``.

        Returns:
            A NumPy array of shape ``(height, width, 3)`` in BGR order, or
            ``None`` if no frame is available yet.
        """
        ...
