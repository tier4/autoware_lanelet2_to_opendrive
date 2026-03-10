"""NPC vehicle entity for CARLA scenarios."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    import carla

from ._spawn import spawn_vehicle_actor, validate_exclusive_spawn_params


@dataclass
class VehicleEntityConfig:
    """Configuration for spawning an NPC vehicle entity.

    Exactly one of *transform* or *spawn_index* must be provided.

    * ``transform``: an explicit :class:`carla.Transform` for the spawn pose.
    * ``spawn_index``: index into the map's ``get_spawn_points()`` list.
      The transform is resolved at spawn time so a world connection is required.
    """

    role_name: str
    vehicle_type: str = "vehicle.tesla.model3"
    transform: Optional["carla.Transform"] = None
    spawn_index: Optional[int] = None
    autopilot: bool = False

    def __post_init__(self) -> None:
        validate_exclusive_spawn_params(self.transform, self.spawn_index)


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
            self._config.transform,
            self._config.spawn_index,
        )

        if self._config.autopilot:
            self._vehicle.set_autopilot(True)

        return self._vehicle

    def destroy(self) -> None:
        """Destroy the vehicle actor and release resources."""
        if self._vehicle is not None:
            self._vehicle.destroy()
            self._vehicle = None
