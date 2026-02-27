"""Ego vehicle spawning and camera management."""

from __future__ import annotations

import queue
from typing import TYPE_CHECKING, List, Optional

import numpy as np

if TYPE_CHECKING:
    import carla

from .scenario_base import EgoConfig


class EgoVehicle:
    """Manages the ego vehicle actor and its rear RGB camera.

    The camera is mounted behind and above the vehicle at a fixed offset
    so that overview recordings are always available.
    """

    # Camera offset: 8 m behind, 3 m above, pitched 15° down
    CAMERA_RELATIVE_TRANSFORM_X: float = -8.0
    CAMERA_RELATIVE_TRANSFORM_Y: float = 0.0
    CAMERA_RELATIVE_TRANSFORM_Z: float = 3.0
    CAMERA_RELATIVE_PITCH: float = -15.0

    def __init__(self) -> None:
        self._vehicle: Optional["carla.Actor"] = None
        self._camera: Optional["carla.Actor"] = None
        self._frame_queue: queue.Queue[np.ndarray] = queue.Queue()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def spawn(self, world: "carla.World", config: EgoConfig) -> "carla.Actor":
        """Spawn the ego vehicle and attach a rear RGB camera.

        Args:
            world: The CARLA world instance.
            config: Ego vehicle spawn configuration.

        Returns:
            The spawned vehicle actor.

        Raises:
            RuntimeError: If the vehicle blueprint is not found or spawn fails.
        """
        import carla

        bp_lib = world.get_blueprint_library()

        # Spawn the vehicle
        vehicle_bp = bp_lib.find(config.vehicle_type)
        if vehicle_bp is None:
            raise RuntimeError(f"Blueprint not found: {config.vehicle_type}")
        vehicle_bp.set_attribute("role_name", "Ego")

        actor = world.spawn_actor(vehicle_bp, config.transform)
        if actor is None:
            raise RuntimeError("Failed to spawn ego vehicle")
        self._vehicle = actor

        # Attach rear RGB camera
        camera_bp = bp_lib.find("sensor.camera.rgb")
        camera_transform = carla.Transform(
            carla.Location(
                x=self.CAMERA_RELATIVE_TRANSFORM_X,
                y=self.CAMERA_RELATIVE_TRANSFORM_Y,
                z=self.CAMERA_RELATIVE_TRANSFORM_Z,
            ),
            carla.Rotation(pitch=self.CAMERA_RELATIVE_PITCH),
        )
        self._camera = world.spawn_actor(
            camera_bp, camera_transform, attach_to=self._vehicle
        )
        self._camera.listen(self._on_image)

        return self._vehicle

    def get_camera_frames(self) -> List[np.ndarray]:
        """Drain and return all accumulated camera frames.

        Returns:
            List of RGB images as NumPy arrays (H×W×3, uint8).
        """
        frames: List[np.ndarray] = []
        while not self._frame_queue.empty():
            try:
                frames.append(self._frame_queue.get_nowait())
            except queue.Empty:
                break
        return frames

    def destroy(self) -> None:
        """Stop the camera and destroy both actors."""
        if self._camera is not None:
            self._camera.stop()
            self._camera.destroy()
            self._camera = None
        if self._vehicle is not None:
            self._vehicle.destroy()
            self._vehicle = None

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _on_image(self, image: "carla.Image") -> None:
        """Camera callback – convert BGRA to RGB and enqueue the frame."""
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = array.reshape((image.height, image.width, 4))
        rgb = array[:, :, :3][:, :, ::-1].copy()  # BGRA → RGB
        self._frame_queue.put(rgb)
