"""Ego vehicle spawning."""

from __future__ import annotations

from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    import carla

from ..constants import EGO_ROLE_NAME
from ..scenario_base import EgoConfig


class EgoVehicle:
    """Manages the ego vehicle actor."""

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
            RuntimeError: If the vehicle blueprint is not found or spawn fails.
        """

        bp_lib = world.get_blueprint_library()

        # Validate the vehicle blueprint before calling find() which raises an
        # opaque std::exception when the ID does not exist.
        available_vehicles = sorted(bp.id for bp in bp_lib.filter("vehicle.*"))
        if config.vehicle_type not in available_vehicles:
            raise ValueError(
                f"Vehicle blueprint {config.vehicle_type!r} is not available. "
                f"Available vehicles: {available_vehicles}"
            )

        # Resolve spawn transform from index or explicit transform.
        spawn_points = world.get_map().get_spawn_points()
        if config.spawn_index is not None:
            if config.spawn_index >= len(spawn_points):
                raise ValueError(
                    f"Spawn index {config.spawn_index} is out of range. "
                    f"The map has {len(spawn_points)} spawn points "
                    f"(valid indices: 0–{len(spawn_points) - 1})."
                )
            transform = spawn_points[config.spawn_index]
        else:
            transform = config.transform  # type: ignore[assignment]

        # Spawn the vehicle
        vehicle_bp = bp_lib.find(config.vehicle_type)
        if vehicle_bp is None:
            raise RuntimeError(f"Blueprint not found: {config.vehicle_type}")
        vehicle_bp.set_attribute("role_name", EGO_ROLE_NAME)

        # try_spawn_actor returns None on collision / out-of-bounds instead of
        # raising an opaque std::exception like spawn_actor does.
        actor = world.try_spawn_actor(vehicle_bp, transform)
        if actor is None:
            suggestions = "\n".join(
                f"  [{i}] x={sp.location.x:.1f}, y={sp.location.y:.1f}, z={sp.location.z:.1f}"
                for i, sp in enumerate(spawn_points[:5])
            )
            raise RuntimeError(
                f"Failed to spawn ego vehicle at "
                f"x={transform.location.x}, "
                f"y={transform.location.y}, "
                f"z={transform.location.z}. "
                "The location may be occupied or out of map bounds. "
                f"Try one of these spawn points:\n{suggestions}"
            )
        self._vehicle = actor

        return self._vehicle

    def destroy(self) -> None:
        """Destroy the vehicle actor."""
        if self._vehicle is not None:
            self._vehicle.destroy()
            self._vehicle = None
