"""Concrete CARLA RGB camera sensor implementation.

Extends :class:`CameraSensorBase` with CARLA-specific attributes such as
post-processing effects (bloom, motion blur, lens flare) and exposure
control.
"""

from __future__ import annotations

import logging
import queue
from dataclasses import dataclass
from typing import TYPE_CHECKING, Optional

import numpy as np
from numpy.typing import NDArray

from .base import CameraSensorBase, CameraSensorConfig

if TYPE_CHECKING:
    import carla

logger = logging.getLogger(__name__)

#: Timeout (seconds) for waiting on a single frame from the sensor queue.
_FRAME_TIMEOUT: float = 1.0


@dataclass(frozen=True)
class CarlaCameraSensorConfig(CameraSensorConfig):
    """CARLA ``sensor.camera.rgb`` configuration.

    Inherits all fields from :class:`CameraSensorConfig` (resolution, FOV,
    extrinsics, etc.) and adds attributes specific to CARLA's RGB camera
    blueprint.

    See `CARLA sensor reference
    <https://carla.readthedocs.io/en/latest/ref_sensors/#rgb-camera>`_
    for full attribute documentation.
    """

    # -- CARLA blueprint selection --------------------------------------------
    sensor_type: str = "sensor.camera.rgb"
    """CARLA blueprint name for the camera sensor."""

    # -- Post-processing effects ----------------------------------------------
    bloom_intensity: float = 0.675
    """Intensity of the bloom post-processing effect (0.0 to 1.0)."""

    fstop: float = 1.4
    """Simulated f-stop number controlling depth of field."""

    iso: float = 100.0
    """Simulated film ISO sensitivity."""

    gamma: float = 2.2
    """Gamma correction value applied to the output image."""

    lens_flare_intensity: float = 0.1
    """Intensity of the lens flare effect (0.0 to 1.0)."""

    motion_blur_intensity: float = 0.45
    """Intensity of motion blur (0.0 to 1.0)."""

    motion_blur_max_distortion: float = 0.35
    """Maximum distortion percentage from motion blur."""

    motion_blur_min_object_screen_size: float = 0.1
    """Minimum screen-space fraction for an object to trigger motion blur."""

    # -- Exposure control -----------------------------------------------------
    exposure_mode: str = "histogram"
    """Exposure mode: ``"histogram"`` (auto) or ``"manual"``."""

    exposure_compensation: float = 0.0
    """Logarithmic exposure compensation (EV)."""

    exposure_min_bright: float = 7.0
    """Minimum brightness for auto-exposure (histogram mode)."""

    exposure_max_bright: float = 9.0
    """Maximum brightness for auto-exposure (histogram mode)."""

    exposure_speed_up: float = 3.0
    """Speed of adaptation when scene brightens."""

    exposure_speed_down: float = 1.0
    """Speed of adaptation when scene darkens."""

    # -- Chromatic aberration -------------------------------------------------
    chromatic_aberration_intensity: float = 0.0
    """Intensity of chromatic aberration fringing (0.0 = disabled)."""

    chromatic_aberration_offset: float = 0.0
    """Offset applied to chromatic aberration channels."""

    # -- Lens distortion ------------------------------------------------------
    lens_circle_falloff: float = 5.0
    """Vignette falloff factor (higher = sharper falloff)."""

    lens_circle_multiplier: float = 0.0
    """Vignette radius multiplier (0.0 = disabled)."""

    lens_k: float = -1.0
    """Radial distortion coefficient k (negative = barrel distortion).

    Set to ``-1.0`` to use CARLA's default (no user override).
    """

    lens_kcube: float = 0.0
    """Cubic radial distortion coefficient."""

    lens_x_size: float = 0.08
    """Horizontal size of the lens distortion grid."""

    lens_y_size: float = 0.08
    """Vertical size of the lens distortion grid."""


class CarlaCameraSensor(CameraSensorBase):
    """CARLA ``sensor.camera.rgb`` sensor implementation.

    Spawns the CARLA RGB camera blueprint with attributes driven by a
    :class:`CarlaCameraSensorConfig`.  Frames are delivered asynchronously
    via ``sensor.listen()`` into a :class:`queue.Queue` and retrieved with
    :meth:`get_image`.

    Args:
        config: CARLA camera sensor configuration.
    """

    def __init__(self, config: CarlaCameraSensorConfig) -> None:
        super().__init__(config)
        self._carla_config = config
        self._sensor: Optional[carla.Actor] = None
        self._frame_queue: queue.Queue[carla.Image] = queue.Queue(maxsize=2)

    # ------------------------------------------------------------------
    # CameraSensorBase interface
    # ------------------------------------------------------------------

    def attach(self, world: "carla.World", actor: "carla.Actor") -> None:
        """Spawn and attach the CARLA RGB camera to *actor*.

        Args:
            world: The CARLA world instance.
            actor: The actor to attach the camera to.

        Raises:
            RuntimeError: If the sensor is already attached.
        """
        if self._attached:
            raise RuntimeError("CarlaCameraSensor is already attached")

        import carla as _carla

        cfg = self._carla_config
        bp_lib = world.get_blueprint_library()
        camera_bp = bp_lib.find(cfg.sensor_type)

        # -- Resolution & optics
        camera_bp.set_attribute("image_size_x", str(cfg.image_width))
        camera_bp.set_attribute("image_size_y", str(cfg.image_height))
        camera_bp.set_attribute("fov", str(cfg.fov))
        camera_bp.set_attribute("sensor_tick", str(1.0 / cfg.fps))

        # -- Post-processing
        camera_bp.set_attribute("bloom_intensity", str(cfg.bloom_intensity))
        camera_bp.set_attribute("fstop", str(cfg.fstop))
        camera_bp.set_attribute("iso", str(cfg.iso))
        camera_bp.set_attribute("gamma", str(cfg.gamma))
        camera_bp.set_attribute("lens_flare_intensity", str(cfg.lens_flare_intensity))
        camera_bp.set_attribute("motion_blur_intensity", str(cfg.motion_blur_intensity))
        camera_bp.set_attribute(
            "motion_blur_max_distortion", str(cfg.motion_blur_max_distortion)
        )
        camera_bp.set_attribute(
            "motion_blur_min_object_screen_size",
            str(cfg.motion_blur_min_object_screen_size),
        )

        # -- Exposure
        camera_bp.set_attribute("exposure_mode", cfg.exposure_mode)
        camera_bp.set_attribute("exposure_compensation", str(cfg.exposure_compensation))
        camera_bp.set_attribute("exposure_min_bright", str(cfg.exposure_min_bright))
        camera_bp.set_attribute("exposure_max_bright", str(cfg.exposure_max_bright))
        camera_bp.set_attribute("exposure_speed_up", str(cfg.exposure_speed_up))
        camera_bp.set_attribute("exposure_speed_down", str(cfg.exposure_speed_down))

        # -- Chromatic aberration
        camera_bp.set_attribute(
            "chromatic_aberration_intensity", str(cfg.chromatic_aberration_intensity)
        )
        camera_bp.set_attribute(
            "chromatic_aberration_offset", str(cfg.chromatic_aberration_offset)
        )

        # -- Lens distortion / vignette
        camera_bp.set_attribute("lens_circle_falloff", str(cfg.lens_circle_falloff))
        camera_bp.set_attribute(
            "lens_circle_multiplier", str(cfg.lens_circle_multiplier)
        )
        if cfg.lens_k >= 0.0:
            camera_bp.set_attribute("lens_k", str(cfg.lens_k))
        camera_bp.set_attribute("lens_kcube", str(cfg.lens_kcube))
        camera_bp.set_attribute("lens_x_size", str(cfg.lens_x_size))
        camera_bp.set_attribute("lens_y_size", str(cfg.lens_y_size))

        # -- Transform (base_link -> camera)
        transform = _carla.Transform(
            _carla.Location(x=cfg.position_x, y=cfg.position_y, z=cfg.position_z),
            _carla.Rotation(roll=cfg.roll, pitch=cfg.pitch, yaw=cfg.yaw),
        )

        self._sensor = world.spawn_actor(camera_bp, transform, attach_to=actor)
        self._sensor.listen(self._frame_queue.put)
        self._attached = True

        logger.info(
            "CarlaCameraSensor attached: %dx%d fov=%.1f @ %.0f fps -> %s",
            cfg.image_width,
            cfg.image_height,
            cfg.fov,
            cfg.fps,
            actor.type_id,
        )

    def destroy(self) -> None:
        """Stop and destroy the CARLA sensor actor."""
        if self._sensor is not None:
            self._sensor.stop()
            self._sensor.destroy()
            self._sensor = None
        self._attached = False
        logger.info("CarlaCameraSensor destroyed")

    def get_image(self) -> Optional[NDArray[np.uint8]]:
        """Return the latest frame as an HxWx3 BGR NumPy array.

        The method blocks for up to :data:`_FRAME_TIMEOUT` seconds waiting
        for a frame.  Returns ``None`` on timeout.
        """
        try:
            image: carla.Image = self._frame_queue.get(timeout=_FRAME_TIMEOUT)
        except queue.Empty:
            return None

        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((image.height, image.width, 4))  # BGRA
        return np.ascontiguousarray(array[:, :, :3])  # BGR

    # ------------------------------------------------------------------
    # CARLA-specific helpers
    # ------------------------------------------------------------------

    @property
    def sensor_actor(self) -> Optional["carla.Actor"]:
        """Return the underlying CARLA sensor actor, or ``None``."""
        return self._sensor
