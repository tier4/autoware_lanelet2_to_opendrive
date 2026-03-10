"""NPC vehicle entity for CARLA scenarios."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    import carla


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
        if self.transform is None and self.spawn_index is None:
            raise ValueError("Either 'transform' or 'spawn_index' must be provided.")
        if self.transform is not None and self.spawn_index is not None:
            raise ValueError(
                "Only one of 'transform' or 'spawn_index' may be provided."
            )


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
        bp_lib = world.get_blueprint_library()

        available_vehicles = sorted(bp.id for bp in bp_lib.filter("vehicle.*"))
        if self._config.vehicle_type not in available_vehicles:
            raise ValueError(
                f"Vehicle blueprint {self._config.vehicle_type!r} is not available. "
                f"Available vehicles: {available_vehicles}"
            )

        spawn_points = world.get_map().get_spawn_points()
        if self._config.spawn_index is not None:
            if self._config.spawn_index >= len(spawn_points):
                raise ValueError(
                    f"Spawn index {self._config.spawn_index} is out of range. "
                    f"The map has {len(spawn_points)} spawn points "
                    f"(valid indices: 0\u2013{len(spawn_points) - 1})."
                )
            transform = spawn_points[self._config.spawn_index]
        else:
            transform = self._config.transform  # type: ignore[assignment]

        vehicle_bp = bp_lib.find(self._config.vehicle_type)
        vehicle_bp.set_attribute("role_name", self._config.role_name)

        actor = world.try_spawn_actor(vehicle_bp, transform)
        if actor is None:
            suggestions = "\n".join(
                f"  [{i}] x={sp.location.x:.1f}, y={sp.location.y:.1f}, "
                f"z={sp.location.z:.1f}"
                for i, sp in enumerate(spawn_points[:5])
            )
            raise RuntimeError(
                f"Failed to spawn NPC vehicle '{self._config.role_name}' at "
                f"x={transform.location.x}, "
                f"y={transform.location.y}, "
                f"z={transform.location.z}. "
                "The location may be occupied or out of map bounds. "
                f"Try one of these spawn points:\n{suggestions}"
            )

        self._vehicle = actor

        if self._config.autopilot:
            self._vehicle.set_autopilot(True)

        return self._vehicle

    def destroy(self) -> None:
        """Destroy the vehicle actor and release resources."""
        if self._vehicle is not None:
            self._vehicle.destroy()
            self._vehicle = None
