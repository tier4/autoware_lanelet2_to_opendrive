"""Shared spawn helpers for vehicle entities."""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import TYPE_CHECKING, Union

if TYPE_CHECKING:
    import carla

logger = logging.getLogger(__name__)


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

    # Refine z via ground_projection before spawning
    resolved_transform = _refine_spawn_z(world, resolved_transform)

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


def _refine_spawn_z(
    world: "carla.World",
    transform: "carla.Transform",
) -> "carla.Transform":
    """Refine the spawn transform's z using ``world.ground_projection``.

    Casts a ray downward from slightly above the transform's location.  If
    the ray hits the ground mesh, a new transform with the corrected z is
    returned; otherwise the original transform is returned unchanged.
    """
    from ..coordinate.snap import (  # noqa: PLC0415
        _refine_z_with_ground_projection,
    )

    loc = transform.location
    refined_z = _refine_z_with_ground_projection(loc.x, loc.y, loc.z, world)

    if refined_z == loc.z:
        return transform

    import carla as _carla  # noqa: PLC0415

    logger.info(
        "Spawn z refined via ground_projection: %.3f -> %.3f at (%.1f, %.1f)",
        loc.z,
        refined_z,
        loc.x,
        loc.y,
    )
    return _carla.Transform(
        _carla.Location(x=loc.x, y=loc.y, z=refined_z),
        transform.rotation,
    )
