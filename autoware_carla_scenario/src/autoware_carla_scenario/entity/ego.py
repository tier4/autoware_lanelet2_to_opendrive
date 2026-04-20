"""Ego vehicle spawning."""

from __future__ import annotations

from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    import carla

from ..constants import EGO_ROLE_NAME
from ..scenario_base import EgoConfig
from ._spawn import spawn_vehicle_actor


class EgoVehicle:
    """Manages the ego vehicle actor."""

    #: When ``True`` (default), :class:`ScenarioRunner` enables
    #: TrafficManager autopilot on this actor after warm-up.
    use_autopilot: bool = True

    def __init__(self) -> None:
        self._vehicle: Optional["carla.Actor"] = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def spawn(self, world: "carla.World", config: EgoConfig) -> "carla.Actor":
        """Spawn the ego vehicle.

        Args:
            world: The CARLA world instance.
            config: Ego vehicle spawn configuration.

        Returns:
            The spawned vehicle actor.

        Raises:
            ValueError: If the vehicle blueprint is not found or spawn index
                is out of range.
            RuntimeError: If the vehicle could not be spawned at the
                requested location.
        """
        self._vehicle = spawn_vehicle_actor(
            world,
            config.vehicle_type,
            str(EGO_ROLE_NAME),
            config.spawn_location,
            od_pose=config.od_pose,
            spawn_retry_max_count=config.spawn_retry_max_count,
            spawn_retry_t_step=config.spawn_retry_t_step,
            spawn_retry_z_step=config.spawn_retry_z_step,
            ground_projection=config.ground_projection,
        )
        return self._vehicle

    def destroy(self) -> None:
        """Destroy the vehicle actor."""
        if self._vehicle is not None:
            self._vehicle.destroy()
            self._vehicle = None
