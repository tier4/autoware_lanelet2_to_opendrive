"""Shared spawn helpers for vehicle entities."""

from __future__ import annotations

import logging
from dataclasses import dataclass, replace
from typing import TYPE_CHECKING, Optional, Union

if TYPE_CHECKING:
    import carla

    from ..coordinate.poses import OpenDrivePose
    from ..coordinate.snap import GroundProjectionConfig

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
    *,
    od_pose: Optional["OpenDrivePose"] = None,
    spawn_retry_max_count: int = 0,
    spawn_retry_t_step: float = 0.1,
    ground_projection: Optional["GroundProjectionConfig"] = None,
) -> "carla.Actor":
    """Spawn a vehicle actor in the CARLA world.

    This is the shared implementation used by both :class:`EgoVehicle` and
    :class:`VehicleEntity`.

    When the initial spawn fails and *spawn_retry_max_count* > 0 and
    *od_pose* is provided, the function retries by shifting the lateral
    offset *t* in the OpenDRIVE coordinate system in increments of
    *spawn_retry_t_step* metres, recomputing the CARLA position via
    :func:`snap_to_carla_road` for each attempt.

    Args:
        world: The CARLA world instance.
        vehicle_type: CARLA blueprint ID (e.g. ``"vehicle.tesla.model3"``).
        role_name: Value for the ``role_name`` actor attribute.
        spawn_location: Where to place the vehicle \u2014 either an explicit
            :class:`SpawnTransform` or a :class:`SpawnPointIndex`.
        od_pose: Optional :class:`OpenDrivePose` used to recompute the
            spawn position with a shifted *t* on retry.
        spawn_retry_max_count: Maximum number of lateral-shift retries
            when the initial spawn fails.  0 disables retries.
        spawn_retry_t_step: Lateral shift in OpenDRIVE *t* (metres) per
            retry attempt.
        ground_projection: Ground projection config used when snapping
            the shifted pose on retry.  Defaults to
            :class:`~autoware_carla_scenario.coordinate.snap.GroundProjectionConfig`
            with its default values.

    Returns:
        The spawned vehicle actor.

    Raises:
        ValueError: If the blueprint is unavailable or the spawn index is
            out of range.
        RuntimeError: If the actor could not be placed at the location
            even after all retry attempts.
    """
    bp_lib = world.get_blueprint_library()

    # Validate blueprint \u2013 bp_lib.find() raises an opaque C++ exception when
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
    logger.info(
        "Spawning '%s' at x=%.3f, y=%.3f, z=%.3f",
        role_name,
        resolved_transform.location.x,
        resolved_transform.location.y,
        resolved_transform.location.z,
    )
    actor = world.try_spawn_actor(vehicle_bp, resolved_transform)

    # Retry with lateral t-shift in OpenDRIVE coordinates when spawn fails.
    # Each step tries both +t and -t so the vehicle is placed on whichever
    # side of the lane centre is free first.
    if actor is None and spawn_retry_max_count > 0 and od_pose is not None:
        from ..coordinate.snap import GroundProjectionConfig, snap_to_carla_road  # noqa: PLC0415

        gp = (
            ground_projection
            if ground_projection is not None
            else GroundProjectionConfig()
        )
        original_t = od_pose.t
        for attempt in range(1, spawn_retry_max_count + 1):
            delta = spawn_retry_t_step * attempt
            for sign, label in ((+1, "+"), (-1, "-")):
                new_t = original_t + sign * delta
                shifted_pose = replace(od_pose, t=new_t)
                snapped = snap_to_carla_road(shifted_pose, world, ground_projection=gp)
                retry_transform = snapped.to_carla_transform()
                logger.info(
                    "Spawn retry %d/%d (%st) for '%s': t=%.3f -> %.3f "
                    "(x=%.3f, y=%.3f, z=%.3f)",
                    attempt,
                    spawn_retry_max_count,
                    label,
                    role_name,
                    original_t,
                    new_t,
                    retry_transform.location.x,
                    retry_transform.location.y,
                    retry_transform.location.z,
                )
                actor = world.try_spawn_actor(vehicle_bp, retry_transform)
                if actor is not None:
                    logger.info(
                        "Spawn succeeded on retry %d (%st) for '%s' at t=%.3f",
                        attempt,
                        label,
                        role_name,
                        new_t,
                    )
                    break
            if actor is not None:
                break

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
