"""6-direction mutual conversion between Lanelet2, OpenDRIVE, and CARLA world coordinates.

All cross-system conversions (Lanelet2 ↔ OpenDRIVE) go through CARLA world coordinates
as an intermediate representation, which avoids the need for an explicit lanelet-to-road
ID mapping.

Coordinate axes
---------------
Lanelet2 local UTM (right-hand):  x=East, y=North, z=Up
OpenDRIVE world   (right-hand):   x=East, y=North, z=Up  ← same origin as Lanelet2
CARLA world       (left-hand):    x=East, y=South, z=Up

Key transforms
--------------
  carla_x = ll2_x
  carla_y = −ll2_y          (flip North→South)
  carla_z = ll2_z
  carla_yaw_deg = −degrees(ll2_heading_rad)   (right-hand → left-hand)
"""

from __future__ import annotations

import math
from typing import Union, overload

import numpy as np

# autoware_lanelet2_extension_python must be imported before lanelet2
from autoware_lanelet2_extension_python.projection import MGRSProjector as _  # noqa: F401
import lanelet2.core
import lanelet2.geometry

from .map_manager import MapManager
from .poses import CarlaWorldPose, Lanelet2Pose, OpenDrivePose


# ---------------------------------------------------------------------------
# Public API (overloaded)
# ---------------------------------------------------------------------------


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

    Lanelet2 → OpenDRIVE conversion goes through CARLA world coordinates.
    """
    if isinstance(pose, CarlaWorldPose):
        return _carla_to_opendrive(pose)
    if isinstance(pose, Lanelet2Pose):
        return _carla_to_opendrive(_lanelet2_to_carla(pose))
    raise TypeError(f"Unsupported pose type: {type(pose)}")


@overload
def to_lanelet2(pose: OpenDrivePose) -> Lanelet2Pose: ...


@overload
def to_lanelet2(pose: CarlaWorldPose) -> Lanelet2Pose: ...


def to_lanelet2(pose: Union[OpenDrivePose, CarlaWorldPose]) -> Lanelet2Pose:
    """Convert an OpenDrivePose or CarlaWorldPose to a Lanelet2Pose.

    OpenDRIVE → Lanelet2 conversion goes through CARLA world coordinates.
    """
    if isinstance(pose, CarlaWorldPose):
        return _carla_to_lanelet2(pose)
    if isinstance(pose, OpenDrivePose):
        return _carla_to_lanelet2(_opendrive_to_carla(pose))
    raise TypeError(f"Unsupported pose type: {type(pose)}")


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

    # Convert right-hand (North=+y) → CARLA left-hand (South=+y)
    carla_yaw_deg = -math.degrees(total_heading)
    return CarlaWorldPose(x=x, y=-y, z=z_cl, yaw=carla_yaw_deg)


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

    # XODR coords are relative to geoReference origin; add MGRS offset to get
    # absolute MGRS coords (= Lanelet2 local UTM coords), then flip y for CARLA.
    offset_x, offset_y = mm.mgrs_offset
    carla_yaw_deg = -math.degrees(total_heading)
    return CarlaWorldPose(x=x + offset_x, y=-(y + offset_y), z=z_ref, yaw=carla_yaw_deg)


# ---------------------------------------------------------------------------
# Inverse: CarlaWorldPose → OpenDrivePose
# ---------------------------------------------------------------------------


def _carla_to_opendrive(pose: CarlaWorldPose) -> OpenDrivePose:
    """Convert a CARLA world pose to the nearest OpenDRIVE (s, t) pose."""
    mm = MapManager.get_instance()

    # Convert CARLA left-hand → absolute MGRS → XODR (relative to geoRef origin)
    offset_x, offset_y = mm.mgrs_offset
    od_x = pose.x - offset_x
    od_y = -pose.y - offset_y
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

    # Convert CARLA left-hand → Lanelet2 right-hand
    ll2_x = pose.x
    ll2_y = -pose.y
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


def _find_lane_at_t(road, s: float, t: float) -> int:
    """Find the lane ID at lateral offset t for a given s on a road.

    Returns the first lane on the correct side (positive t = left lanes with
    positive IDs; negative t = right lanes with negative IDs).  Lane ID is
    context-only information, so a best-effort match is sufficient.

    Returns 0 if no lane sections are found.
    """
    lane_sections = road.lane_sections
    if not lane_sections:
        return 0

    # Find the lane section covering s (assumes sections are sorted by s)
    active_section = lane_sections[0]
    for section in lane_sections:
        s_start = float(section.lane_section_xml.attrib.get("s", 0.0))
        if s_start <= s:
            active_section = section
        else:
            break

    is_left = t >= 0.0
    for lane in active_section.lanes:
        if is_left and lane.id > 0:
            return lane.id
        if not is_left and lane.id < 0:
            return lane.id

    return 0
