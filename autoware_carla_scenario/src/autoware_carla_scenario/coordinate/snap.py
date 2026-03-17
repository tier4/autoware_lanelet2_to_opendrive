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
from dataclasses import dataclass
from typing import TYPE_CHECKING, Union, overload

from .poses import CarlaWorldPose, Lanelet2Pose, OpenDrivePose

if TYPE_CHECKING:
    import carla

logger = logging.getLogger(__name__)

#: Maximum distance (m) to the nearest spawn point before a warning is emitted.
_SPAWN_POINT_WARN_DISTANCE: float = 10.0


@dataclass
class GroundProjectionConfig:
    """Tunable parameters for the ground-projection z-refinement.

    The ray is cast downward from ``z_estimate + ray_distance_upper`` and
    searches a total of ``ray_distance_upper + ray_distance_lower`` metres,
    i.e. from ``z_estimate + upper`` to ``z_estimate - lower``.
    """

    #: Search range (m) above the estimated z.
    ray_distance_upper: float = 5.0

    #: Search range (m) below the estimated z.
    ray_distance_lower: float = 5.0

    #: Minimum absolute z delta (m) to log an info message.
    z_log_threshold: float = 0.01


_ground_projection_config = GroundProjectionConfig()


def configure_ground_projection(
    *,
    ray_distance_upper: float | None = None,
    ray_distance_lower: float | None = None,
) -> None:
    """Update the module-level :class:`GroundProjectionConfig`.

    Only the parameters that are explicitly passed (not ``None``) are changed.
    Call this before any :func:`snap_to_carla_road` invocation so that the
    snap pipeline uses the desired values.
    """
    if ray_distance_upper is not None:
        _ground_projection_config.ray_distance_upper = ray_distance_upper
    if ray_distance_lower is not None:
        _ground_projection_config.ray_distance_lower = ray_distance_lower
    logger.info(
        "Ground projection config updated: ray_distance_upper=%.2f, ray_distance_lower=%.2f",
        _ground_projection_config.ray_distance_upper,
        _ground_projection_config.ray_distance_lower,
    )


@overload
def snap_to_carla_road(
    pose: Lanelet2Pose,
    world: "carla.World",
) -> CarlaWorldPose: ...


@overload
def snap_to_carla_road(
    pose: OpenDrivePose,
    world: "carla.World",
) -> CarlaWorldPose: ...


@overload
def snap_to_carla_road(
    pose: CarlaWorldPose,
    world: "carla.World",
) -> CarlaWorldPose: ...


def snap_to_carla_road(
    pose: Union[CarlaWorldPose, Lanelet2Pose, OpenDrivePose],
    world: "carla.World",
) -> CarlaWorldPose:
    """Snap a pose onto the CARLA drivable surface.

    When given a :class:`Lanelet2Pose`, the function converts to an approximate
    CARLA position via the Lanelet2 centerline, projects it onto the nearest
    OpenDRIVE road, then re-converts through the XODR geometry.  This
    round-trip ensures the final position lies on the XODR road surface that
    CARLA trusts.

    When given an :class:`OpenDrivePose`, the function uses
    ``carla.Map.get_waypoint_xodr(road_id, lane_id, s)`` to obtain the exact
    position on the CARLA road surface.  This is the most accurate path
    because it uses CARLA's own OpenDRIVE projection—no spawn-point z
    approximation is needed.

    When given a :class:`CarlaWorldPose`, the function uses the CARLA waypoint
    API to project x/y onto the nearest road.

    For Lanelet2 and CARLA-world inputs z is corrected using the nearest CARLA
    spawn point (spawn-point elevations match the physics-engine ground plane).

    If the pose cannot be matched to any road, a warning is logged and the
    original pose (or its CARLA equivalent) is returned unchanged.

    Parameters
    ----------
    pose:
        A Lanelet2, OpenDRIVE, or CARLA world pose to snap.
    world:
        An active ``carla.World`` instance.

    Returns
    -------
    CarlaWorldPose
        A new pose snapped to the road surface.
    """
    if isinstance(pose, Lanelet2Pose):
        return _snap_lanelet2_via_opendrive(pose, world)
    if isinstance(pose, OpenDrivePose):
        return _snap_opendrive_via_waypoint_xodr(pose, world)
    return _snap_carla_via_waypoint(pose, world)


# ---------------------------------------------------------------------------
# Lanelet2Pose path – snap via OpenDRIVE projection
# ---------------------------------------------------------------------------


def _snap_lanelet2_via_opendrive(
    pose: Lanelet2Pose,
    world: "carla.World",
) -> CarlaWorldPose:
    """Snap a Lanelet2 pose by projecting through OpenDRIVE geometry.

    1. Convert directly to OpenDRIVE (s, t) using the cached lanelet-to-road
       mapping.  Lanelet2's reference line is the centerline; OpenDRIVE's is
       the right boundary (LHT).  The mapping provides road_id and lane_id;
       s and t are computed by projecting the centerline point onto the road
       reference line.
    2. Correct t to the lane centre.
    3. Convert the OpenDRIVE pose to CARLA world coordinates (XODR geometry).
    4. Replace z with the nearest spawn-point elevation.
    """
    from .map_manager import MapManager  # noqa: PLC0415
    from .transform import _lane_center_t, to_carla_world, to_opendrive  # noqa: PLC0415

    # Direct Lanelet2 → OpenDRIVE (uses cached mapping for road_id/lane_id,
    # projects centerline point onto the road reference line for s/t).
    od_projected = to_opendrive(pose)

    # Correct t to the lane centre (the projected t reflects the offset
    # between the Lanelet2 centreline and the XODR reference line, which
    # does not correspond to the lane centre).
    mm = MapManager.get_instance()
    road = mm.road_network.road_ids_to_object[od_projected.road_id]
    center_t = _lane_center_t(road, od_projected.s, od_projected.lane_id)

    if center_t is not None:
        corrected_t = center_t
    else:
        corrected_t = od_projected.t
        logger.warning(
            "Could not compute lane centre t for road '%s' lane %d; "
            "using projected t=%.2f",
            od_projected.road_id,
            od_projected.lane_id,
            od_projected.t,
        )

    logger.info(
        "Lanelet2Pose(lanelet_id=%d, s=%.2f, t=%.2f) -> "
        "OpenDrivePose(road='%s', lane=%d, s=%.2f, t=%.2f -> centre_t=%.2f)",
        pose.lanelet_id,
        pose.s,
        pose.t,
        od_projected.road_id,
        od_projected.lane_id,
        od_projected.s,
        od_projected.t,
        corrected_t,
    )

    od_corrected = OpenDrivePose(
        road_id=od_projected.road_id,
        lane_id=od_projected.lane_id,
        s=od_projected.s,
        t=corrected_t,
        heading=od_projected.heading,
    )

    # Re-convert via XODR geometry (position now lies on the lane centre)
    carla_from_od = to_carla_world(od_corrected)

    # Z correction: first from nearest spawn point, then refine via ground projection
    snapped_z = _z_from_nearest_spawn_point(carla_from_od.x, carla_from_od.y, world)
    base_z = snapped_z if snapped_z is not None else carla_from_od.z
    refined_z = refine_z_with_ground_projection(
        carla_from_od.x, carla_from_od.y, base_z, world
    )

    result = CarlaWorldPose(
        x=carla_from_od.x,
        y=carla_from_od.y,
        z=refined_z,
        roll=carla_from_od.roll,
        pitch=carla_from_od.pitch,
        yaw=carla_from_od.yaw,
    )

    logger.debug(
        "snap (Lanelet2): od=(road=%s, s=%.2f, t=%.2f) -> snapped=(%.2f, %.2f, %.2f)",
        od_corrected.road_id,
        od_corrected.s,
        od_corrected.t,
        result.x,
        result.y,
        result.z,
    )

    return result


# ---------------------------------------------------------------------------
# OpenDrivePose path – snap via get_waypoint_xodr
# ---------------------------------------------------------------------------


def _snap_opendrive_via_waypoint_xodr(
    pose: OpenDrivePose,
    world: "carla.World",
) -> CarlaWorldPose:
    """Snap an OpenDRIVE pose using ``carla.Map.get_waypoint_xodr``.

    This is the most accurate snap path because CARLA resolves
    ``(road_id, lane_id, s)`` directly against its internal OpenDRIVE
    geometry, producing exact x/y/z and yaw on the road surface.
    """
    carla_map = world.get_map()

    waypoint = carla_map.get_waypoint_xodr(
        int(pose.road_id),
        pose.lane_id,
        pose.s,
    )
    if waypoint is None:
        logger.warning(
            "get_waypoint_xodr(road=%s, lane=%d, s=%.2f) returned None; "
            "falling back to coordinate transform",
            pose.road_id,
            pose.lane_id,
            pose.s,
        )
        from .transform import to_carla_world  # noqa: PLC0415

        return to_carla_world(pose)

    tf = waypoint.transform
    refined_z = refine_z_with_ground_projection(
        tf.location.x, tf.location.y, tf.location.z, world
    )

    result = CarlaWorldPose(
        x=tf.location.x,
        y=tf.location.y,
        z=refined_z,
        yaw=tf.rotation.yaw,
    )

    logger.info(
        "snap (OpenDRIVE): road='%s' lane=%d s=%.2f -> "
        "CARLA (%.2f, %.2f, %.3f) yaw=%.1f",
        pose.road_id,
        pose.lane_id,
        pose.s,
        result.x,
        result.y,
        result.z,
        result.yaw,
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
    base_z = snapped_z if snapped_z is not None else pose.z
    refined_z = refine_z_with_ground_projection(snapped_x, snapped_y, base_z, world)

    result = CarlaWorldPose(
        x=snapped_x,
        y=snapped_y,
        z=refined_z,
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


def refine_z_with_ground_projection(
    x: float,
    y: float,
    z_estimate: float,
    world: "carla.World",
) -> float:
    """Refine *z_estimate* by casting a downward ray via ``world.ground_projection``.

    A ray is cast from ``(x, y, z_estimate + offset)`` downward.  If the ray
    hits the physics-mesh ground surface, the hit-point's z coordinate is
    returned.

    Parameters
    ----------
    x, y:
        CARLA world horizontal coordinates.
    z_estimate:
        The best z estimate from existing logic (spawn-point / waypoint).
    world:
        An active ``carla.World`` instance.

    Returns
    -------
    float
        The refined z coordinate.

    Raises
    ------
    RuntimeError
        If ``ground_projection`` is unavailable, fails, or returns no hit.
        A missing ground hit indicates that the spawn location is outside the
        physics mesh (e.g. off-road or the map is not loaded correctly).
    """
    import carla as _carla  # noqa: PLC0415

    cfg = _ground_projection_config
    origin = _carla.Location(
        x=x,
        y=y,
        z=z_estimate + cfg.ray_distance_upper,
    )
    search_distance = cfg.ray_distance_upper + cfg.ray_distance_lower

    try:
        result = world.ground_projection(origin, search_distance)
    except AttributeError:
        raise RuntimeError(
            "world.ground_projection() is not available in this CARLA version. "
            "A CARLA build that supports ground_projection is required."
        ) from None
    except RuntimeError as exc:
        raise RuntimeError(
            f"world.ground_projection() failed at ({x:.2f}, {y:.2f}, "
            f"{origin.z:.2f}): {exc}. "
            "The physics mesh may not be loaded."
        ) from exc

    if result is None:
        raise RuntimeError(
            f"ground_projection returned no hit at ({x:.2f}, {y:.2f}) "
            f"with z_estimate={z_estimate:.3f}. "
            "The spawn location is likely outside the drivable surface "
            "or the CARLA map is not loaded correctly."
        )

    if result.label == _carla.CityObjectLabel.NONE:
        logger.warning(
            "ground_projection hit at (%.2f, %.2f) has CityObjectLabel.NONE. "
            "Enable semantic tags on the map's ground meshes for more "
            "reliable ground detection.",
            x,
            y,
        )

    ground_z = result.location.z
    delta = ground_z - z_estimate
    if abs(delta) > cfg.z_log_threshold:
        logger.info(
            "ground_projection refined z: %.3f -> %.3f (delta=%.3f) at (%.1f, %.1f)",
            z_estimate,
            ground_z,
            delta,
            x,
            y,
        )
    return ground_z
