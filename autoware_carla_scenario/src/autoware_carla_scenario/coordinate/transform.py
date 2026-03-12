"""6-direction mutual conversion between Lanelet2, OpenDRIVE, and CARLA world coordinates.

When a lanelet-to-road mapping is available (see :mod:`.road_lanelet_mapping`),
Lanelet2 -> OpenDRIVE conversion uses a direct path that avoids the O(n) road
search and the unnecessary CARLA y-flip.  The indirect path through CARLA world
coordinates is kept as a fallback.

Coordinate systems
------------------
Lanelet2 (MGRS absolute, right-hand):  x=East, y=North, z=Up
OpenDRIVE / CARLA (XODR-relative):     origin = geoReference (lat_0, lon_0)
CARLA world (left-hand):               x≈East, y≈South, z=Up  (y flipped from XODR)

Key relationships
-----------------
  mgrs_xy  = xodr_xy + mgrs_offset
  carla_x  = xodr_x                     (same origin as XODR)
  carla_y  = −xodr_y                    (flip North→South)
  carla_z  = xodr_z

  carla_x  = ll2_x − mgrs_offset_x     (MGRS absolute → XODR relative)
  carla_y  = −(ll2_y − mgrs_offset_y)   (MGRS absolute → XODR relative, then flip)
  carla_yaw_deg = −degrees(heading_rad)  (right-hand → left-hand)
"""

from __future__ import annotations

import logging
import math
from typing import TYPE_CHECKING, Any, Optional, Union, overload

import numpy as np

# autoware_lanelet2_extension_python must be imported before lanelet2
from autoware_lanelet2_extension_python.projection import MGRSProjector as _  # noqa: F401
import lanelet2.core
import lanelet2.geometry

from .map_manager import MapManager
from .poses import AnyPose, CarlaWorldPose, Lanelet2Pose, OpenDrivePose

if TYPE_CHECKING:
    import carla

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Public API (overloaded)
# ---------------------------------------------------------------------------


def to_carla_location(pose: Union[AnyPose, "carla.Location"]) -> "carla.Location":
    """Convert any pose type to a ``carla.Location``.

    Accepts ``Lanelet2Pose``, ``OpenDrivePose``, ``CarlaWorldPose``, or a raw
    ``carla.Location``.  Lanelet2/OpenDRIVE poses are first converted to CARLA
    world coordinates via :func:`to_carla_world`.

    Args:
        pose: The source pose or location.

    Returns:
        A ``carla.Location`` instance.
    """
    if isinstance(pose, (Lanelet2Pose, OpenDrivePose)):
        pose = to_carla_world(pose)
    if isinstance(pose, CarlaWorldPose):
        import carla as _carla  # noqa: PLC0415

        return _carla.Location(x=pose.x, y=pose.y, z=pose.z)
    # Assume carla.Location
    return pose


@overload
def to_carla_world(pose: Lanelet2Pose) -> CarlaWorldPose: ...


@overload
def to_carla_world(pose: OpenDrivePose) -> CarlaWorldPose: ...


def to_carla_world(pose: Union[Lanelet2Pose, OpenDrivePose]) -> CarlaWorldPose:
    """Convert a Lanelet2Pose or OpenDrivePose to a CARLA world pose."""
    if isinstance(pose, Lanelet2Pose):
        return _lanelet2_to_carla(pose)
    if isinstance(pose, OpenDrivePose):
        return _opendrive_to_carla(pose)
    raise TypeError(f"Unsupported pose type: {type(pose)}")


@overload
def to_opendrive(pose: Lanelet2Pose) -> OpenDrivePose: ...


@overload
def to_opendrive(pose: CarlaWorldPose) -> OpenDrivePose: ...


def to_opendrive(pose: Union[Lanelet2Pose, CarlaWorldPose]) -> OpenDrivePose:
    """Convert a Lanelet2Pose or CarlaWorldPose to an OpenDrivePose.

    When a lanelet-to-road mapping is available, Lanelet2 -> OpenDRIVE uses a
    direct path (no CARLA y-flip, no O(n) road search).  Falls back to the
    indirect path through CARLA world coordinates otherwise.
    """
    if isinstance(pose, CarlaWorldPose):
        return _carla_to_opendrive(pose)
    if isinstance(pose, Lanelet2Pose):
        direct = _lanelet2_to_opendrive_direct(pose)
        if direct is not None:
            return direct
        return _carla_to_opendrive(_lanelet2_to_carla(pose))
    raise TypeError(f"Unsupported pose type: {type(pose)}")


@overload
def to_lanelet2(pose: OpenDrivePose) -> Lanelet2Pose: ...


@overload
def to_lanelet2(pose: CarlaWorldPose) -> Lanelet2Pose: ...


def to_lanelet2(pose: Union[OpenDrivePose, CarlaWorldPose]) -> Lanelet2Pose:
    """Convert an OpenDrivePose or CarlaWorldPose to a Lanelet2Pose.

    When a road-lanelet mapping is available, OpenDRIVE → Lanelet2 uses a
    direct path (XODR + mgrs_offset, no CARLA y-flip).  Falls back to the
    indirect path through CARLA world coordinates otherwise.
    """
    if isinstance(pose, CarlaWorldPose):
        return _carla_to_lanelet2(pose)
    if isinstance(pose, OpenDrivePose):
        direct = _opendrive_to_lanelet2_direct(pose)
        if direct is not None:
            return direct
        return _carla_to_lanelet2(_opendrive_to_carla(pose))
    raise TypeError(f"Unsupported pose type: {type(pose)}")


# ---------------------------------------------------------------------------
# Direct: Lanelet2Pose → OpenDrivePose (no CARLA intermediate)
# ---------------------------------------------------------------------------


def _lanelet2_to_opendrive_direct(pose: Lanelet2Pose) -> Optional[OpenDrivePose]:
    """Convert a Lanelet2 pose directly to OpenDRIVE using the cached mapping.

    Returns ``None`` when the mapping is unavailable or the lanelet is not
    in the mapping, signalling the caller should fall back to the indirect
    path through CARLA world coordinates.
    """
    mm = MapManager.get_instance()
    mapping = mm.road_lanelet_mapping
    if mapping is None:
        return None

    result = mapping.lanelet_to_road_and_lane.get(pose.lanelet_id)
    if result is None:
        logger.debug(
            "Lanelet %d not in mapping; falling back to indirect path",
            pose.lanelet_id,
        )
        return None

    road_id, lane_id = result

    # Compute Lanelet2 centerline point at pose.s, then apply lateral offset
    lanelet = mm.lanelet_map.laneletLayer[pose.lanelet_id]
    points = [(p.x, p.y, p.z) for p in lanelet.centerline]
    x_cl, y_cl, _z_cl, heading_cl = _interpolate_at_s(points, pose.s)
    total_heading = heading_cl + pose.heading

    x = x_cl + pose.t * (-math.sin(total_heading))
    y = y_cl + pose.t * math.cos(total_heading)

    # Convert MGRS -> XODR: subtract offset, NO y-flip
    offset_x, offset_y = mm.mgrs_offset
    xodr_x = x - offset_x
    xodr_y = y - offset_y

    # Project onto this ONE road's reference line
    road = mm.road_network.road_ids_to_object[str(road_id)]
    ref_line: np.ndarray = road.reference_line
    if len(ref_line) < 2:
        return None

    arc_lengths = _compute_arc_lengths_2d(ref_line)
    diffs = ref_line - np.array([xodr_x, xodr_y])
    dists = np.linalg.norm(diffs, axis=1)
    nearest_idx = int(np.argmin(dists))

    s_road = float(arc_lengths[nearest_idx])
    heading_ref = _heading_at_s(ref_line, arc_lengths, s_road)
    t_road = _signed_perp_distance(xodr_x, xodr_y, ref_line, nearest_idx, heading_ref)
    heading_diff = _normalize_angle(total_heading - heading_ref)

    return OpenDrivePose(
        road_id=str(road_id),
        lane_id=lane_id,
        s=s_road,
        t=t_road,
        heading=heading_diff,
    )


# ---------------------------------------------------------------------------
# Direct: OpenDrivePose → Lanelet2Pose (no CARLA intermediate)
# ---------------------------------------------------------------------------


def _opendrive_to_lanelet2_direct(pose: OpenDrivePose) -> Optional[Lanelet2Pose]:
    """Convert an OpenDRIVE pose directly to Lanelet2 using the cached mapping.

    The road reference line point at ``(s, t)`` is converted to MGRS
    coordinates (``xodr_xy + mgrs_offset``, no y-flip) and projected onto
    the lanelet centerline.

    Returns ``None`` when the mapping is unavailable or the (road, lane)
    pair is not in the reverse index, signalling the caller should fall
    back to the indirect path through CARLA world coordinates.
    """
    mm = MapManager.get_instance()
    mapping = mm.road_lanelet_mapping
    if mapping is None:
        return None

    road_id_int = int(pose.road_id)
    lanelet_id = mapping.road_lane_to_lanelet.get((road_id_int, pose.lane_id))
    if lanelet_id is None:
        logger.debug(
            "Road %s lane %d not in reverse mapping; falling back to indirect path",
            pose.road_id,
            pose.lane_id,
        )
        return None

    # Compute XODR world point from road reference line at (s, t)
    road = mm.road_network.road_ids_to_object[pose.road_id]
    ref_line: np.ndarray = road.reference_line
    if len(ref_line) < 2:
        return None

    arc_lengths = _compute_arc_lengths_2d(ref_line)
    x_ref = float(np.interp(pose.s, arc_lengths, ref_line[:, 0]))
    y_ref = float(np.interp(pose.s, arc_lengths, ref_line[:, 1]))
    heading_ref = _heading_at_s(ref_line, arc_lengths, pose.s)
    total_heading = heading_ref + pose.heading

    # Apply lateral offset t
    xodr_x = x_ref + pose.t * (-math.sin(heading_ref))
    xodr_y = y_ref + pose.t * math.cos(heading_ref)

    # XODR → MGRS: add offset (no y-flip, both right-hand systems)
    offset_x, offset_y = mm.mgrs_offset
    mgrs_x = xodr_x + offset_x
    mgrs_y = xodr_y + offset_y

    # Project onto the lanelet centerline
    lanelet = mm.lanelet_map.laneletLayer[lanelet_id]
    points = [(p.x, p.y, p.z) for p in lanelet.centerline]
    s_ll, t_ll, heading_cl = _project_to_centerline(points, mgrs_x, mgrs_y)
    heading_diff = _normalize_angle(total_heading - heading_cl)

    return Lanelet2Pose(lanelet_id=lanelet_id, s=s_ll, t=t_ll, heading=heading_diff)


# ---------------------------------------------------------------------------
# Forward: Lanelet2Pose → CarlaWorldPose
# ---------------------------------------------------------------------------


def _lanelet2_to_carla(pose: Lanelet2Pose) -> CarlaWorldPose:
    """Convert a Lanelet2 Frenet pose to CARLA world coordinates."""
    mm = MapManager.get_instance()
    lanelet = mm.lanelet_map.laneletLayer[pose.lanelet_id]
    points = [(p.x, p.y, p.z) for p in lanelet.centerline]

    x_cl, y_cl, z_cl, heading_cl = _interpolate_at_s(points, pose.s)
    total_heading = heading_cl + pose.heading

    # Apply lateral offset (positive t = left of heading direction)
    x = x_cl + pose.t * (-math.sin(total_heading))
    y = y_cl + pose.t * math.cos(total_heading)

    # Lanelet2 centerline uses MGRS absolute coords; CARLA world uses
    # XODR-relative coords.  Subtract the MGRS offset for x/y, and the
    # vertical offset for z (Lanelet2 stores absolute elevation).
    offset_x, offset_y = mm.mgrs_offset
    carla_yaw_deg = -math.degrees(total_heading)
    return CarlaWorldPose(
        x=x - offset_x,
        y=-(y - offset_y),
        z=z_cl - mm.z_offset,
        yaw=carla_yaw_deg,
    )


# ---------------------------------------------------------------------------
# Forward: OpenDrivePose → CarlaWorldPose
# ---------------------------------------------------------------------------


def _opendrive_to_carla(pose: OpenDrivePose) -> CarlaWorldPose:
    """Convert an OpenDRIVE (s, t) pose to CARLA world coordinates."""
    mm = MapManager.get_instance()
    road = mm.road_network.road_ids_to_object[pose.road_id]
    ref_line: np.ndarray = road.reference_line  # shape (N, 2)

    z_coords: np.ndarray = road.z_coordinates  # shape (N,)
    arc_lengths = _compute_arc_lengths_2d(ref_line)

    x_ref = float(np.interp(pose.s, arc_lengths, ref_line[:, 0]))
    y_ref = float(np.interp(pose.s, arc_lengths, ref_line[:, 1]))
    z_ref = float(np.interp(pose.s, arc_lengths, z_coords))

    heading_ref = _heading_at_s(ref_line, arc_lengths, pose.s)
    total_heading = heading_ref + pose.heading

    # Apply lateral offset (OpenDRIVE: positive t = left of reference line direction)
    x = x_ref + pose.t * (-math.sin(total_heading))
    y = y_ref + pose.t * math.cos(total_heading)

    # XODR coords are already in CARLA's coordinate frame (same origin);
    # just flip y for CARLA's left-hand system (South=+y).
    carla_yaw_deg = -math.degrees(total_heading)
    return CarlaWorldPose(x=x, y=-y, z=z_ref, yaw=carla_yaw_deg)


# ---------------------------------------------------------------------------
# Inverse: CarlaWorldPose → OpenDrivePose
# ---------------------------------------------------------------------------


def _carla_to_opendrive(pose: CarlaWorldPose) -> OpenDrivePose:
    """Convert a CARLA world pose to the nearest OpenDRIVE (s, t) pose.

    Uses ``carla.Map.get_waypoint()`` when available for accurate road/lane
    identification; falls back to brute-force reference-line search otherwise.
    """
    mm = MapManager.get_instance()
    carla_map = mm.carla_map

    if carla_map is not None:
        result = _carla_to_opendrive_via_waypoint(pose, carla_map, mm)
        if result is not None:
            return result
        logger.warning(
            "Waypoint lookup failed for (%.2f, %.2f); "
            "falling back to brute-force search",
            pose.x,
            pose.y,
        )

    return _carla_to_opendrive_bruteforce(pose, mm)


def _carla_to_opendrive_via_waypoint(
    pose: CarlaWorldPose,
    carla_map: Any,
    mm: MapManager,
) -> Optional[OpenDrivePose]:
    """Use ``carla.Map.get_waypoint()`` for accurate road/lane identification.

    Returns ``None`` when the waypoint cannot be obtained or the road is not
    present in the pyxodr ``RoadNetwork``.
    """
    import carla  # noqa: PLC0415

    try:
        wp = carla_map.get_waypoint(
            carla.Location(x=pose.x, y=pose.y, z=pose.z),
        )
    except Exception:
        return None

    if wp is None:
        return None

    road_id = str(wp.road_id)
    lane_id: int = wp.lane_id

    if road_id not in mm.road_network.road_ids_to_object:
        return None

    road = mm.road_network.road_ids_to_object[road_id]
    ref_line: np.ndarray = road.reference_line
    if len(ref_line) < 2:
        return None

    # CARLA world coords share the same origin as XODR; just flip y back.
    od_x = pose.x
    od_y = -pose.y
    od_heading = -math.radians(pose.yaw)

    arc_lengths = _compute_arc_lengths_2d(ref_line)
    diffs = ref_line - np.array([od_x, od_y])
    dists = np.linalg.norm(diffs, axis=1)
    nearest_idx = int(np.argmin(dists))

    s = float(arc_lengths[nearest_idx])
    heading_ref = _heading_at_s(ref_line, arc_lengths, s)
    t = _signed_perp_distance(od_x, od_y, ref_line, nearest_idx, heading_ref)
    heading_diff = _normalize_angle(od_heading - heading_ref)

    return OpenDrivePose(
        road_id=road_id,
        lane_id=lane_id,
        s=s,
        t=t,
        heading=heading_diff,
    )


def _carla_to_opendrive_bruteforce(
    pose: CarlaWorldPose,
    mm: MapManager,
) -> OpenDrivePose:
    """Brute-force nearest road search across all roads in the network.

    Used as a fallback when ``carla.Map`` is unavailable or waypoint lookup
    fails.
    """
    # CARLA world coords share the same origin as XODR; just flip y back.
    od_x = pose.x
    od_y = -pose.y
    od_heading = -math.radians(pose.yaw)

    best_road_id: str = ""
    best_s: float = 0.0
    best_t: float = 0.0
    best_heading_diff: float = 0.0
    best_lane_id: int = 0
    best_dist: float = float("inf")

    for road_id, road in mm.road_network.road_ids_to_object.items():
        ref_line: np.ndarray = road.reference_line  # shape (N, 2)
        if len(ref_line) < 2:
            continue

        arc_lengths = _compute_arc_lengths_2d(ref_line)
        diffs = ref_line - np.array([od_x, od_y])
        dists = np.linalg.norm(diffs, axis=1)
        nearest_idx = int(np.argmin(dists))
        dist = float(dists[nearest_idx])

        if dist < best_dist:
            best_dist = dist
            best_road_id = road_id
            s = float(arc_lengths[nearest_idx])
            heading_ref = _heading_at_s(ref_line, arc_lengths, s)
            t = _signed_perp_distance(od_x, od_y, ref_line, nearest_idx, heading_ref)
            heading_diff = _normalize_angle(od_heading - heading_ref)
            lane_id = _find_lane_at_t(road, s, t)

            best_s = s
            best_t = t
            best_heading_diff = heading_diff
            best_lane_id = lane_id

    return OpenDrivePose(
        road_id=best_road_id,
        lane_id=best_lane_id,
        s=best_s,
        t=best_t,
        heading=best_heading_diff,
    )


# ---------------------------------------------------------------------------
# Inverse: CarlaWorldPose → Lanelet2Pose
# ---------------------------------------------------------------------------


def _carla_to_lanelet2(pose: CarlaWorldPose) -> Lanelet2Pose:
    """Convert a CARLA world pose to the nearest Lanelet2 Frenet pose."""
    mm = MapManager.get_instance()

    # CARLA world uses XODR-relative coords; Lanelet2 uses MGRS absolute.
    # Add the MGRS offset for x/y, and the vertical offset for z.
    offset_x, offset_y = mm.mgrs_offset
    ll2_x = pose.x + offset_x
    ll2_y = -pose.y + offset_y
    ll2_heading = -math.radians(pose.yaw)

    query = lanelet2.core.BasicPoint2d(ll2_x, ll2_y)
    results = lanelet2.geometry.findNearest(mm.lanelet_map.laneletLayer, query, 1)
    lanelet = results[0][1]

    points = [(p.x, p.y, p.z) for p in lanelet.centerline]
    s, t, heading_cl = _project_to_centerline(points, ll2_x, ll2_y)
    heading_diff = _normalize_angle(ll2_heading - heading_cl)

    return Lanelet2Pose(lanelet_id=lanelet.id, s=s, t=t, heading=heading_diff)


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------


def _compute_arc_lengths_2d(points: np.ndarray) -> np.ndarray:
    """Compute cumulative arc lengths along a 2D polyline (shape N×2)."""
    deltas = np.diff(points, axis=0)
    seg_lengths = np.linalg.norm(deltas, axis=1)
    arc_lengths = np.zeros(len(points))
    arc_lengths[1:] = np.cumsum(seg_lengths)
    return arc_lengths


def _interpolate_at_s(
    points: list[tuple[float, float, float]], s: float
) -> tuple[float, float, float, float]:
    """Interpolate position and heading at arc length s along a 3D polyline.

    Returns
    -------
    (x, y, z, heading)  where heading is the tangent direction in radians.
    """
    xs = np.array([p[0] for p in points])
    ys = np.array([p[1] for p in points])
    zs = np.array([p[2] for p in points])

    arc = _compute_arc_lengths_2d(np.column_stack([xs, ys]))
    s_clamped = float(np.clip(s, arc[0], arc[-1]))

    x = float(np.interp(s_clamped, arc, xs))
    y = float(np.interp(s_clamped, arc, ys))
    z = float(np.interp(s_clamped, arc, zs))

    # Heading via finite difference of nearest segment
    idx = int(np.searchsorted(arc, s_clamped, side="right")) - 1
    idx = max(0, min(idx, len(points) - 2))
    heading = math.atan2(ys[idx + 1] - ys[idx], xs[idx + 1] - xs[idx])

    return x, y, z, heading


def _heading_at_s(ref_line: np.ndarray, arc_lengths: np.ndarray, s: float) -> float:
    """Return the tangent heading (radians) at arc length s along a 2D polyline."""
    s_clamped = float(np.clip(s, arc_lengths[0], arc_lengths[-1]))
    idx = int(np.searchsorted(arc_lengths, s_clamped, side="right")) - 1
    idx = max(0, min(idx, len(ref_line) - 2))
    dx = ref_line[idx + 1, 0] - ref_line[idx, 0]
    dy = ref_line[idx + 1, 1] - ref_line[idx, 1]
    return math.atan2(dy, dx)


def _signed_perp_distance(
    x: float,
    y: float,
    ref_line: np.ndarray,
    nearest_idx: int,
    heading_ref: float,
) -> float:
    """Signed perpendicular distance from point (x, y) to the reference line.

    Positive = left of the reference line direction (OpenDRIVE convention).
    Uses the cross product of the heading vector and the displacement vector.
    """
    rx = ref_line[nearest_idx, 0]
    ry = ref_line[nearest_idx, 1]
    # 2D cross product: heading_vec × displacement_vec
    # heading_vec = (cos θ, sin θ), displacement = (x − rx, y − ry)
    # cross = cos θ * (y − ry) − sin θ * (x − rx)
    t = math.cos(heading_ref) * (y - ry) - math.sin(heading_ref) * (x - rx)
    return t


def _project_to_centerline(
    points: list[tuple[float, float, float]], x: float, y: float
) -> tuple[float, float, float]:
    """Project (x, y) onto a 3D polyline centerline.

    Returns
    -------
    (s, t, heading)
        s       – arc length to nearest point
        t       – signed lateral offset (positive=left)
        heading – centerline tangent direction at nearest point (radians)
    """
    xs = np.array([p[0] for p in points])
    ys = np.array([p[1] for p in points])

    arc = _compute_arc_lengths_2d(np.column_stack([xs, ys]))
    dists = np.sqrt((xs - x) ** 2 + (ys - y) ** 2)
    idx = int(np.argmin(dists))
    s = float(arc[idx])

    seg_idx = max(0, min(idx, len(points) - 2))
    heading = math.atan2(ys[seg_idx + 1] - ys[seg_idx], xs[seg_idx + 1] - xs[seg_idx])

    t = _signed_perp_distance(x, y, np.column_stack([xs, ys]), idx, heading)

    return s, t, heading


def _normalize_angle(angle: float) -> float:
    """Normalize angle to (−π, π]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _lane_width_at_ds(lane, ds: float) -> float:
    """Return the width of *lane* at offset *ds* from the lane-section start.

    Lane widths in OpenDRIVE are cubic polynomials:
    ``width(ds) = a + b·ds + c·ds² + d·ds³``.
    Multiple ``<width>`` elements may define piece-wise polynomials; this
    function selects the active segment for the given *ds*.
    """
    width_elements = lane.lane_xml.findall("width")
    if not width_elements:
        return 0.0

    active = width_elements[0]
    for elem in width_elements:
        s_offset = float(elem.attrib.get("sOffset", 0.0))
        if s_offset <= ds:
            active = elem
        else:
            break

    s_offset = float(active.attrib.get("sOffset", 0.0))
    a = float(active.attrib["a"])
    b = float(active.attrib["b"])
    c = float(active.attrib["c"])
    d = float(active.attrib["d"])
    local = ds - s_offset
    return a + b * local + c * local**2 + d * local**3


def _find_lane_section_at_s(road, s: float):
    """Return the active lane section covering arc length *s*.

    Returns ``None`` when the road has no lane sections.
    """
    lane_sections = road.lane_sections
    if not lane_sections:
        return None

    active_section = lane_sections[0]
    for section in lane_sections:
        s_start = float(section.lane_section_xml.attrib.get("s", 0.0))
        if s_start <= s:
            active_section = section
        else:
            break
    return active_section


def _lane_center_t(road, s: float, lane_id: int) -> float | None:
    """Return the *t* offset that places a point at the centre of *lane_id*.

    OpenDRIVE measures *t* from the road reference line.  For lane −1 the
    centre is at ``t = −(width_of_lane_−1 / 2)``; for lane −2 it is
    ``t = −(width_of_lane_−1 + width_of_lane_−2 / 2)``; and so on for the
    left side (positive lane IDs, positive *t*).

    Returns ``None`` when the requested lane cannot be found in the active
    lane section.
    """
    if lane_id == 0:
        return 0.0

    active_section = _find_lane_section_at_s(road, s)
    if active_section is None:
        return None

    section_s_start = float(active_section.lane_section_xml.attrib.get("s", 0.0))
    ds = s - section_s_start

    lanes_by_id = {lane.id: lane for lane in active_section.lanes}

    sign = 1.0 if lane_id > 0 else -1.0
    target_abs = abs(lane_id)

    total = 0.0
    for abs_id in range(1, target_abs + 1):
        current_id = abs_id if lane_id > 0 else -abs_id
        lane = lanes_by_id.get(current_id)
        if lane is None:
            return None
        width = _lane_width_at_ds(lane, ds)
        if abs_id == target_abs:
            total += width / 2.0
        else:
            total += width

    return sign * total


def _find_lane_at_t(road, s: float, t: float) -> int:
    """Find the lane ID at lateral offset t for a given s on a road.

    Returns the first lane on the correct side (positive t = left lanes with
    positive IDs; negative t = right lanes with negative IDs).  Lane ID is
    context-only information, so a best-effort match is sufficient.

    Returns 0 if no lane sections are found.
    """
    active_section = _find_lane_section_at_s(road, s)
    if active_section is None:
        return 0

    is_left = t >= 0.0
    for lane in active_section.lanes:
        if is_left and lane.id > 0:
            return lane.id
        if not is_left and lane.id < 0:
            return lane.id

    return 0
