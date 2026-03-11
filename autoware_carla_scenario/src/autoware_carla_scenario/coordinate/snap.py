"""Snap a pose onto the actual CARLA road surface.

The coordinate transform pipeline (Lanelet2 → OpenDRIVE → CARLA) can produce
positions that are slightly off the drivable surface due to geometry mismatches
between the Lanelet2 map and the XODR road network.  :func:`snap_to_carla_road`
corrects the position via OpenDRIVE projection (for :class:`Lanelet2Pose` input)
or the CARLA waypoint API (for :class:`CarlaWorldPose` input), and z via the
nearest spawn point.
"""

from __future__ import annotations

import logging
import math
from typing import TYPE_CHECKING, Union, overload

from .poses import CarlaWorldPose, Lanelet2Pose

if TYPE_CHECKING:
    import carla

logger = logging.getLogger(__name__)

#: Maximum distance (m) to the nearest spawn point before a warning is emitted.
_SPAWN_POINT_WARN_DISTANCE: float = 10.0


@overload
def snap_to_carla_road(
    pose: Lanelet2Pose,
    world: "carla.World",
) -> CarlaWorldPose: ...


@overload
def snap_to_carla_road(
    pose: CarlaWorldPose,
    world: "carla.World",
) -> CarlaWorldPose: ...


def snap_to_carla_road(
    pose: Union[CarlaWorldPose, Lanelet2Pose],
    world: "carla.World",
) -> CarlaWorldPose:
    """Snap a pose onto the CARLA drivable surface.

    When given a :class:`Lanelet2Pose`, the function converts to an approximate
    CARLA position via the Lanelet2 centerline, projects it onto the nearest
    OpenDRIVE road, then re-converts through the XODR geometry.  This
    round-trip ensures the final position lies on the XODR road surface that
    CARLA trusts.

    When given a :class:`CarlaWorldPose`, the function uses the CARLA waypoint
    API to project x/y onto the nearest road.

    In both cases z is corrected using the nearest CARLA spawn point (spawn-point
    elevations match the physics-engine ground plane).

    If the pose cannot be matched to any road, a warning is logged and the
    original pose (or its CARLA equivalent) is returned unchanged.

    Parameters
    ----------
    pose:
        A Lanelet2 or CARLA world pose to snap.
    world:
        An active ``carla.World`` instance.

    Returns
    -------
    CarlaWorldPose
        A new pose snapped to the road surface.
    """
    if isinstance(pose, Lanelet2Pose):
        return _snap_lanelet2_via_opendrive(pose, world)
    return _snap_carla_via_waypoint(pose, world)


# ---------------------------------------------------------------------------
# Lanelet2Pose path – snap via OpenDRIVE projection
# ---------------------------------------------------------------------------


def _snap_lanelet2_via_opendrive(
    pose: Lanelet2Pose,
    world: "carla.World",
) -> CarlaWorldPose:
    """Snap a Lanelet2 pose by round-tripping through OpenDRIVE geometry.

    1. Convert to an approximate CARLA position via Lanelet2 centerline.
    2. Project that position onto the nearest OpenDRIVE road to obtain
       the correct (road_id, s, t) in the XODR coordinate system.
    3. Re-convert the projected OpenDRIVE pose to CARLA world coordinates
       (uses XODR geometry, which CARLA trusts).
    4. Replace z with the nearest spawn-point elevation.
    """
    from .transform import to_carla_world, to_opendrive  # noqa: PLC0415

    # Approximate CARLA position → project onto nearest OpenDRIVE road
    carla_approx = to_carla_world(pose)
    od_projected = to_opendrive(carla_approx)

    logger.info(
        "Lanelet2Pose(lanelet_id=%d, s=%.2f, t=%.2f) -> "
        "approx CARLA (%.2f, %.2f) -> "
        "OpenDrivePose(road='%s', lane=%d, s=%.2f, t=%.2f)",
        pose.lanelet_id,
        pose.s,
        pose.t,
        carla_approx.x,
        carla_approx.y,
        od_projected.road_id,
        od_projected.lane_id,
        od_projected.s,
        od_projected.t,
    )

    # Re-convert via XODR geometry (position now lies on the XODR road surface)
    carla_from_od = to_carla_world(od_projected)

    # Z correction
    snapped_z = _z_from_nearest_spawn_point(carla_from_od.x, carla_from_od.y, world)

    result = CarlaWorldPose(
        x=carla_from_od.x,
        y=carla_from_od.y,
        z=snapped_z if snapped_z is not None else carla_from_od.z,
        roll=carla_from_od.roll,
        pitch=carla_from_od.pitch,
        yaw=carla_from_od.yaw,
    )

    logger.debug(
        "snap (Lanelet2): approx=(%.2f, %.2f, %.2f) -> " "snapped=(%.2f, %.2f, %.2f)",
        carla_approx.x,
        carla_approx.y,
        carla_approx.z,
        result.x,
        result.y,
        result.z,
    )

    return result


# ---------------------------------------------------------------------------
# CarlaWorldPose path – snap via waypoint API
# ---------------------------------------------------------------------------


def _snap_carla_via_waypoint(
    pose: CarlaWorldPose,
    world: "carla.World",
) -> CarlaWorldPose:
    """Snap a CARLA world pose using the waypoint API.

    Uses ``get_waypoint()`` to project x/y onto the nearest road and the
    nearest spawn point for z.  The original yaw is preserved.
    """
    import carla as _carla  # noqa: PLC0415

    carla_map = world.get_map()

    waypoint = carla_map.get_waypoint(
        _carla.Location(x=pose.x, y=pose.y, z=0.0),
    )
    if waypoint is None:
        logger.warning(
            "get_waypoint() returned None for (%.2f, %.2f); "
            "returning original pose unchanged",
            pose.x,
            pose.y,
        )
        return pose

    snapped_x = waypoint.transform.location.x
    snapped_y = waypoint.transform.location.y

    snapped_z = _z_from_nearest_spawn_point(snapped_x, snapped_y, world)

    result = CarlaWorldPose(
        x=snapped_x,
        y=snapped_y,
        z=snapped_z if snapped_z is not None else pose.z,
        roll=pose.roll,
        pitch=pose.pitch,
        yaw=pose.yaw,
    )

    logger.debug(
        "snap (CARLA): (%.2f, %.2f, %.2f) -> (%.2f, %.2f, %.2f)",
        pose.x,
        pose.y,
        pose.z,
        result.x,
        result.y,
        result.z,
    )

    return result


# ---------------------------------------------------------------------------
# Shared helper
# ---------------------------------------------------------------------------


def _z_from_nearest_spawn_point(
    x: float, y: float, world: "carla.World"
) -> float | None:
    """Return z from the nearest CARLA spawn point, or ``None`` if unavailable.

    Emits a warning when the nearest spawn point is more than
    :data:`_SPAWN_POINT_WARN_DISTANCE` metres away.
    """
    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        return None

    best_dist2 = float("inf")
    best_z: float | None = None
    for sp in spawn_points:
        d2 = (sp.location.x - x) ** 2 + (sp.location.y - y) ** 2
        if d2 < best_dist2:
            best_dist2 = d2
            best_z = sp.location.z

    if best_z is not None:
        dist = math.sqrt(best_dist2)
        if dist > _SPAWN_POINT_WARN_DISTANCE:
            logger.warning(
                "Nearest spawn point is %.1fm away from (%.1f, %.1f); "
                "z=%.2f may be inaccurate",
                dist,
                x,
                y,
                best_z,
            )

    return best_z
