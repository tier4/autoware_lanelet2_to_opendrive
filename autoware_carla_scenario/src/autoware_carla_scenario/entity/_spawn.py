"""Shared spawn helpers for vehicle entities."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Union

if TYPE_CHECKING:
    import carla


@dataclass(frozen=True)
class SpawnTransform:
    """Spawn at an explicit :class:`carla.Transform`."""

    value: "carla.Transform"


@dataclass(frozen=True)
class SpawnPointIndex:
    """Spawn at a map spawn-point by index."""

    value: int


SpawnLocation = Union[SpawnTransform, SpawnPointIndex]
"""Where to place a vehicle.  Exactly one variant must be chosen."""


def spawn_vehicle_actor(
    world: "carla.World",
    vehicle_type: str,
    role_name: str,
    spawn_location: SpawnLocation,
) -> "carla.Actor":
    """Spawn a vehicle actor in the CARLA world.

    This is the shared implementation used by both :class:`EgoVehicle` and
    :class:`VehicleEntity`.

    Args:
        world: The CARLA world instance.
        vehicle_type: CARLA blueprint ID (e.g. ``"vehicle.tesla.model3"``).
        role_name: Value for the ``role_name`` actor attribute.
        spawn_location: Where to place the vehicle — either an explicit
            :class:`SpawnTransform` or a :class:`SpawnPointIndex`.

    Returns:
        The spawned vehicle actor.

    Raises:
        ValueError: If the blueprint is unavailable or the spawn index is
            out of range.
        RuntimeError: If the actor could not be placed at the location.
    """
    bp_lib = world.get_blueprint_library()

    # Validate blueprint – bp_lib.find() raises an opaque C++ exception when
    # the ID does not exist, so we pre-check with a set (O(n), no sort).
    vehicle_ids = {bp.id for bp in bp_lib.filter("vehicle.*")}
    if vehicle_type not in vehicle_ids:
        raise ValueError(
            f"Vehicle blueprint {vehicle_type!r} is not available. "
            f"Available vehicles: {sorted(vehicle_ids)}"
        )

    # Resolve spawn transform.  Only fetch spawn points when needed so that
    # the explicit-transform path avoids the get_map() RPC call.
    spawn_points = None
    if isinstance(spawn_location, SpawnPointIndex):
        spawn_points = world.get_map().get_spawn_points()
        if spawn_location.value >= len(spawn_points):
            raise ValueError(
                f"Spawn index {spawn_location.value} is out of range. "
                f"The map has {len(spawn_points)} spawn points "
                f"(valid indices: 0\u2013{len(spawn_points) - 1})."
            )
        resolved_transform = spawn_points[spawn_location.value]
    else:
        resolved_transform = spawn_location.value

    vehicle_bp = bp_lib.find(vehicle_type)
    if vehicle_bp is None:
        raise RuntimeError(f"Blueprint not found: {vehicle_type}")
    vehicle_bp.set_attribute("role_name", role_name)

    # try_spawn_actor returns None on collision / out-of-bounds instead of
    # raising an opaque std::exception like spawn_actor does.
    actor = world.try_spawn_actor(vehicle_bp, resolved_transform)
    if actor is None:
        if spawn_points is None:
            spawn_points = world.get_map().get_spawn_points()
        suggestions = "\n".join(
            f"  [{i}] x={sp.location.x:.1f}, y={sp.location.y:.1f}, "
            f"z={sp.location.z:.1f}"
            for i, sp in enumerate(spawn_points[:5])
        )
        raise RuntimeError(
            f"Failed to spawn vehicle '{role_name}' at "
            f"x={resolved_transform.location.x}, "
            f"y={resolved_transform.location.y}, "
            f"z={resolved_transform.location.z}. "
            "The location may be occupied or out of map bounds. "
            f"Try one of these spawn points:\n{suggestions}"
        )

    return actor
