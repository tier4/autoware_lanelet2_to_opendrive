"""NPC vehicle entity for CARLA scenarios."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    import carla

from ._spawn import SpawnLocation, spawn_vehicle_actor


@dataclass
class VehicleEntityConfig:
    """Configuration for spawning an NPC vehicle entity.

    *spawn_location* determines where the vehicle is placed — either an
    explicit :class:`SpawnTransform` or a :class:`SpawnPointIndex`.
    """

    role_name: str
    spawn_location: SpawnLocation
    vehicle_type: str = "vehicle.fuso.mitsubishi"


class VehicleEntity:
    """Manages the lifecycle of an NPC vehicle actor in CARLA.

    This class handles spawning, destroying, and providing access to an NPC
    vehicle actor.  It mirrors the :class:`EgoVehicle` API but is intended
    for non-ego vehicles that participate in a scenario.
    """

    def __init__(self, config: VehicleEntityConfig) -> None:
        self._config = config
        self._vehicle: Optional["carla.Actor"] = None

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def role_name(self) -> str:
        """Return the role name that identifies this entity."""
        return self._config.role_name

    @property
    def vehicle_type(self) -> str:
        """Return the CARLA blueprint ID for this vehicle."""
        return self._config.vehicle_type

    @property
    def actor(self) -> Optional["carla.Actor"]:
        """Return the underlying CARLA actor, or ``None`` if not spawned."""
        return self._vehicle

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def spawn(self, world: "carla.World") -> "carla.Actor":
        """Spawn the NPC vehicle in the CARLA world.

        Args:
            world: The CARLA world instance.

        Returns:
            The spawned vehicle actor.

        Raises:
            ValueError: If the vehicle blueprint is not available or the
                spawn index is out of range.
            RuntimeError: If the vehicle could not be spawned at the
                requested location.
        """
        self._vehicle = spawn_vehicle_actor(
            world,
            self._config.vehicle_type,
            self._config.role_name,
            self._config.spawn_location,
        )
        return self._vehicle

    def destroy(self) -> None:
        """Destroy the vehicle actor and release resources."""
        if self._vehicle is not None:
            self._vehicle.destroy()
            self._vehicle = None
