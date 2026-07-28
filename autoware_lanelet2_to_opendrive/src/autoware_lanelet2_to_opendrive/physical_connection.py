"""Shared physical geometry planning for OpenDRIVE road connections."""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from enum import Enum
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import lanelet2
import numpy as np

from .config import DEFAULT_CONFIG
from .opendrive.enums import ContactPoint, ElementType
from .opendrive.lane_elements import LaneWidth
from .opendrive.road import Road
from .util import extract_points_3d


class PhysicalConnectionType(str, Enum):
    """Logical road transition represented by one physical seam."""

    ORDINARY_CONTINUATION = "ordinary_continuation"
    ORDINARY_MULTI_LANE_CONTINUATION = "ordinary_multi_lane_continuation"
    JUNCTION_INCOMING = "junction_incoming"
    JUNCTION_OUTGOING = "junction_outgoing"
    SPLIT = "split"
    MERGE = "merge"
    LANE_ADDITION_OR_DROP = "lane_addition_or_drop"


@dataclass(frozen=True)
class LogicalLaneCorrespondence:
    """Lane-level ownership on both sides of a physical seam."""

    from_lanelet_id: int
    from_lane_id: int
    to_lanelet_id: int
    to_lane_id: int


@dataclass(frozen=True)
class SharedPhysicalCrossSection:
    """Source-backed cross-section shared by both emitted roads."""

    reference_xyz: Tuple[float, float, float]
    heading: float
    boundary_xyz: Tuple[Tuple[float, float, float], ...]
    lane_widths: Tuple[float, ...]


@dataclass(frozen=True)
class RoadEndpointConstraint:
    """One road-side view of a shared physical cross-section."""

    road_id: int
    at_start: bool
    reference_xyz: Tuple[float, float, float]
    heading: float


@dataclass(frozen=True)
class PhysicalConnectionPlan:
    """Atomic geometry contract for one OpenDRIVE road seam."""

    from_road_id: int
    to_road_id: int
    connection_type: PhysicalConnectionType
    lane_correspondences: Tuple[LogicalLaneCorrespondence, ...]
    cross_section: SharedPhysicalCrossSection
    from_endpoint: RoadEndpointConstraint
    to_endpoint: RoadEndpointConstraint
    source_backed: bool = True
    require_c0: bool = True
    require_c1: bool = True
    require_width_continuity: bool = True


def _non_center_lanes(road: Road, *, at_start: bool) -> Dict[int, object]:
    if road.lanes is None or not road.lanes.lane_sections:
        return {}
    section = road.lanes.lane_sections[0 if at_start else -1]
    return {**section.left_lanes, **section.right_lanes}


def _ordered_one_side_lanes(road: Road, *, at_start: bool) -> List[object]:
    lanes = _non_center_lanes(road, at_start=at_start)
    if not lanes:
        return []
    signs = {1 if lane_id > 0 else -1 for lane_id in lanes}
    if len(signs) != 1:
        return []
    return [lanes[lane_id] for lane_id in sorted(lanes, key=abs)]


def _nearest_endpoint(
    points: np.ndarray,
    reference_xy: np.ndarray,
) -> Tuple[np.ndarray, bool]:
    start_distance = float(np.linalg.norm(points[0, :2] - reference_xy))
    end_distance = float(np.linalg.norm(points[-1, :2] - reference_xy))
    if start_distance <= end_distance:
        return points[0], True
    return points[-1], False


def _lane_cap(
    lanelet: lanelet2.core.Lanelet,
    lane_id: int,
    reference_xy: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    left, _left_at_start = _nearest_endpoint(
        extract_points_3d(lanelet.leftBound),
        reference_xy,
    )
    right, _right_at_start = _nearest_endpoint(
        extract_points_3d(lanelet.rightBound),
        reference_xy,
    )
    return (right, left) if lane_id > 0 else (left, right)


def _oriented_reference_points(
    road: Road,
    lanelet_by_id: Dict[int, lanelet2.core.Lanelet],
) -> Optional[np.ndarray]:
    """Return the source inner boundary in emitted road travel direction."""
    lanes = _ordered_one_side_lanes(road, at_start=True)
    if not lanes:
        return None
    reference_lane = lanes[0]
    if reference_lane.lane_id is None or reference_lane.lanelet_id is None:
        return None
    lanelet = lanelet_by_id.get(reference_lane.lanelet_id)
    if lanelet is None:
        return None
    boundary = lanelet.rightBound if reference_lane.lane_id > 0 else lanelet.leftBound
    points = extract_points_3d(boundary)
    if len(points) < 2:
        return None

    clean_points = [points[0]]
    for point in points[1:]:
        if (
            float(np.linalg.norm(point[:2] - clean_points[-1][:2]))
            > DEFAULT_CONFIG.geometry.epsilon
        ):
            clean_points.append(point)
    oriented = np.asarray(clean_points, dtype=float)
    if len(oriented) < 2:
        return None

    if road.reference_start_xyz is not None and road.reference_end_xyz is not None:
        start = np.asarray(road.reference_start_xyz[:2], dtype=float)
        end = np.asarray(road.reference_end_xyz[:2], dtype=float)
        direct = float(
            np.linalg.norm(oriented[0, :2] - start)
            + np.linalg.norm(oriented[-1, :2] - end)
        )
        reverse = float(
            np.linalg.norm(oriented[-1, :2] - start)
            + np.linalg.norm(oriented[0, :2] - end)
        )
        if reverse + DEFAULT_CONFIG.geometry.epsilon < direct:
            oriented = oriented[::-1].copy()
    return oriented


def _fixed_endpoint_headings(
    plans: Sequence[PhysicalConnectionPlan],
    roads: Sequence[Road],
    lanelet_by_id: Dict[int, lanelet2.core.Lanelet],
) -> Tuple[Dict[Tuple[int, bool], float], set[Tuple[int, bool]]]:
    """Find endpoint tangents that cannot change without moving another seam."""
    constrained_sides: Dict[int, set[bool]] = {}
    for plan in plans:
        constrained_sides.setdefault(plan.from_road_id, set()).add(False)
        constrained_sides.setdefault(plan.to_road_id, set()).add(True)

    roads_by_id = {road.id: road for road in roads}
    fixed: Dict[Tuple[int, bool], float] = {}
    curve_capable: set[Tuple[int, bool]] = set()
    for road_id, sides in constrained_sides.items():
        road = roads_by_id.get(road_id)
        if road is None:
            continue
        points = _oriented_reference_points(road, lanelet_by_id)
        if points is None:
            continue

        # With two points, changing either endpoint segment changes the whole
        # road. With three points and both ends constrained, both adjustments
        # compete for the same middle point. Preserve the source tangents and
        # propagate them to a neighbouring road that has local freedom.
        if len(points) == 2:
            fixed[(road_id, True)] = math.atan2(
                float(points[1, 1] - points[0, 1]),
                float(points[1, 0] - points[0, 0]),
            )
            fixed[(road_id, False)] = fixed[(road_id, True)]
            curve_capable.update(((road_id, True), (road_id, False)))
        elif len(points) == 3 and sides == {True, False}:
            fixed[(road_id, True)] = math.atan2(
                float(points[1, 1] - points[0, 1]),
                float(points[1, 0] - points[0, 0]),
            )
            fixed[(road_id, False)] = math.atan2(
                float(points[2, 1] - points[1, 1]),
                float(points[2, 0] - points[1, 0]),
            )
    return fixed, curve_capable


def _widths_for_heading(
    cross_section: SharedPhysicalCrossSection,
    heading: float,
    lane_sign: int,
) -> Optional[Tuple[float, ...]]:
    tangent = np.array([math.cos(heading), math.sin(heading)], dtype=float)
    side_direction = lane_sign * np.array([-tangent[1], tangent[0]], dtype=float)
    boundaries = np.asarray(cross_section.boundary_xyz, dtype=float)
    widths = tuple(
        float(np.dot(end[:2] - start[:2], side_direction))
        for start, end in zip(boundaries[:-1], boundaries[1:])
    )
    if any(
        not math.isfinite(width) or width <= DEFAULT_CONFIG.geometry.epsilon
        for width in widths
    ):
        return None
    return widths


def _reconcile_source_fixed_headings(
    plans: Sequence[PhysicalConnectionPlan],
    roads: Sequence[Road],
    lanelet_by_id: Dict[int, lanelet2.core.Lanelet],
) -> List[PhysicalConnectionPlan]:
    """Propagate source-fixed tangents without deforming under-sampled roads."""
    fixed, curve_capable = _fixed_endpoint_headings(
        plans,
        roads,
        lanelet_by_id,
    )
    reconciled = []
    for plan in plans:
        from_heading = fixed.get((plan.from_road_id, False))
        to_heading = fixed.get((plan.to_road_id, True))
        shared_heading = plan.cross_section.heading
        require_c1 = plan.require_c1

        if from_heading is not None and to_heading is not None:
            cap_matches_fixed_tangents = all(
                abs(
                    math.atan2(
                        math.sin(fixed_heading - shared_heading),
                        math.cos(fixed_heading - shared_heading),
                    )
                )
                <= DEFAULT_CONFIG.geometry.terminal_micro_kink_support_heading_tolerance
                for fixed_heading in (from_heading, to_heading)
            )
            require_c1 = (
                cap_matches_fixed_tangents
                and (plan.from_road_id, False) in curve_capable
                and (plan.to_road_id, True) in curve_capable
            )
            if require_c1:
                from_heading = shared_heading
                to_heading = shared_heading
        elif from_heading is not None:
            if (plan.from_road_id, False) in curve_capable and abs(
                math.atan2(
                    math.sin(from_heading - shared_heading),
                    math.cos(from_heading - shared_heading),
                )
            ) <= DEFAULT_CONFIG.geometry.terminal_micro_kink_support_heading_tolerance:
                from_heading = shared_heading
                to_heading = shared_heading
            else:
                shared_heading = from_heading
                to_heading = from_heading
        elif to_heading is not None:
            if (plan.to_road_id, True) in curve_capable and abs(
                math.atan2(
                    math.sin(to_heading - shared_heading),
                    math.cos(to_heading - shared_heading),
                )
            ) <= DEFAULT_CONFIG.geometry.terminal_micro_kink_support_heading_tolerance:
                from_heading = shared_heading
                to_heading = shared_heading
            else:
                shared_heading = to_heading
                from_heading = to_heading
        else:
            from_heading = shared_heading
            to_heading = shared_heading

        lane_sign = 1 if plan.lane_correspondences[0].from_lane_id > 0 else -1
        widths = _widths_for_heading(
            plan.cross_section,
            shared_heading,
            lane_sign,
        )
        if widths is None:
            continue
        cross_section = replace(
            plan.cross_section,
            heading=shared_heading,
            lane_widths=widths,
        )
        reconciled.append(
            replace(
                plan,
                cross_section=cross_section,
                from_endpoint=replace(
                    plan.from_endpoint,
                    heading=from_heading,
                ),
                to_endpoint=replace(
                    plan.to_endpoint,
                    heading=to_heading,
                ),
                require_c1=require_c1,
            )
        )
    return reconciled


def _caps_match(
    from_cap: Tuple[np.ndarray, np.ndarray],
    to_cap: Tuple[np.ndarray, np.ndarray],
) -> bool:
    tolerance = DEFAULT_CONFIG.geometry.point_distance_threshold
    return all(
        float(np.linalg.norm(from_point[:2] - to_point[:2])) <= tolerance
        for from_point, to_point in zip(from_cap, to_cap)
    )


def _lane_correspondences(
    from_road: Road,
    to_road: Road,
) -> Optional[Tuple[LogicalLaneCorrespondence, ...]]:
    from_lanes = _ordered_one_side_lanes(from_road, at_start=False)
    to_lanes = _non_center_lanes(to_road, at_start=True)
    if not from_lanes or len(from_lanes) != len(to_lanes):
        return None

    correspondences = []
    matched_to_ids = set()
    for from_lane in from_lanes:
        if (
            from_lane.lane_id is None
            or from_lane.lanelet_id is None
            or from_lane.successor is None
        ):
            return None
        to_lane = to_lanes.get(from_lane.successor.id)
        if (
            to_lane is None
            or to_lane.lane_id is None
            or to_lane.lanelet_id is None
            or to_lane.predecessor is None
            or to_lane.predecessor.id != from_lane.lane_id
        ):
            return None
        correspondences.append(
            LogicalLaneCorrespondence(
                from_lanelet_id=from_lane.lanelet_id,
                from_lane_id=from_lane.lane_id,
                to_lanelet_id=to_lane.lanelet_id,
                to_lane_id=to_lane.lane_id,
            )
        )
        matched_to_ids.add(to_lane.lane_id)

    if matched_to_ids != set(to_lanes):
        return None
    return tuple(correspondences)


def _junction_incoming_lane_correspondences(
    from_road: Road,
    to_road: Road,
) -> Optional[Tuple[LogicalLaneCorrespondence, ...]]:
    """Map the incoming lane subset used by one junction connecting road."""
    from_lanes = _non_center_lanes(from_road, at_start=False)
    to_lanes = _ordered_one_side_lanes(to_road, at_start=True)
    if not from_lanes or not to_lanes:
        return None

    correspondences = []
    for to_lane in to_lanes:
        if (
            to_lane.lane_id is None
            or to_lane.lanelet_id is None
            or to_lane.predecessor is None
        ):
            return None
        from_lane = from_lanes.get(to_lane.predecessor.id)
        if (
            from_lane is None
            or from_lane.lane_id is None
            or from_lane.lanelet_id is None
            or from_lane.successor is None
            or from_lane.successor.id != to_lane.lane_id
        ):
            return None
        correspondences.append(
            LogicalLaneCorrespondence(
                from_lanelet_id=from_lane.lanelet_id,
                from_lane_id=from_lane.lane_id,
                to_lanelet_id=to_lane.lanelet_id,
                to_lane_id=to_lane.lane_id,
            )
        )
    return tuple(correspondences)


def _shared_cross_section(
    from_road: Road,
    to_road: Road,
    correspondences: Sequence[LogicalLaneCorrespondence],
    lanelet_by_id: Dict[int, lanelet2.core.Lanelet],
) -> Optional[SharedPhysicalCrossSection]:
    if from_road.reference_end_xyz is None or to_road.reference_start_xyz is None:
        return None
    from_reference_xy = np.asarray(from_road.reference_end_xyz[:2], dtype=float)
    to_reference_xy = np.asarray(to_road.reference_start_xyz[:2], dtype=float)

    boundary_samples: List[List[np.ndarray]] = [
        [] for _ in range(len(correspondences) + 1)
    ]
    lane_sign = 0
    for index, correspondence in enumerate(correspondences):
        from_lanelet = lanelet_by_id.get(correspondence.from_lanelet_id)
        to_lanelet = lanelet_by_id.get(correspondence.to_lanelet_id)
        if from_lanelet is None or to_lanelet is None:
            return None
        from_cap = _lane_cap(
            from_lanelet,
            correspondence.from_lane_id,
            from_reference_xy,
        )
        to_cap = _lane_cap(
            to_lanelet,
            correspondence.to_lane_id,
            to_reference_xy,
        )
        if not _caps_match(from_cap, to_cap):
            return None
        current_sign = 1 if correspondence.from_lane_id > 0 else -1
        if lane_sign not in (0, current_sign):
            return None
        lane_sign = current_sign
        boundary_samples[index].extend((from_cap[0], to_cap[0]))
        boundary_samples[index + 1].extend((from_cap[1], to_cap[1]))

    boundaries = np.asarray(
        [
            np.mean(np.asarray(samples, dtype=float), axis=0)
            for samples in boundary_samples
        ],
        dtype=float,
    )
    lateral = boundaries[-1, :2] - boundaries[0, :2]
    lateral_norm = float(np.linalg.norm(lateral))
    if lateral_norm <= DEFAULT_CONFIG.geometry.epsilon:
        return None
    tangent = lane_sign * np.array([lateral[1], -lateral[0]], dtype=float)
    tangent /= float(np.linalg.norm(tangent))

    road_direction = (
        np.asarray(from_road.reference_end_xyz[:2], dtype=float)
        - np.asarray(from_road.reference_start_xyz[:2], dtype=float)
        if from_road.reference_start_xyz is not None
        else tangent
    )
    if float(np.dot(tangent, road_direction)) < 0.0:
        tangent = -tangent
    heading = math.atan2(float(tangent[1]), float(tangent[0]))
    side_direction = lane_sign * np.array([-tangent[1], tangent[0]], dtype=float)

    widths = []
    for start, end in zip(boundaries[:-1], boundaries[1:]):
        width = float(np.dot(end[:2] - start[:2], side_direction))
        if not math.isfinite(width) or width <= DEFAULT_CONFIG.geometry.epsilon:
            return None
        widths.append(width)

    return SharedPhysicalCrossSection(
        reference_xyz=tuple(float(value) for value in boundaries[0]),
        heading=heading,
        boundary_xyz=tuple(
            tuple(float(value) for value in boundary) for boundary in boundaries
        ),
        lane_widths=tuple(widths),
    )


def _road_source_cross_section(
    road: Road,
    lanelet_by_id: Dict[int, lanelet2.core.Lanelet],
    *,
    at_start: bool,
) -> Optional[SharedPhysicalCrossSection]:
    lanes = _ordered_one_side_lanes(road, at_start=at_start)
    reference_xyz = road.reference_start_xyz if at_start else road.reference_end_xyz
    if not lanes or reference_xyz is None:
        return None
    reference_xy = np.asarray(reference_xyz[:2], dtype=float)
    boundary_samples: List[List[np.ndarray]] = [[] for _ in range(len(lanes) + 1)]
    lane_sign = 0
    for index, lane in enumerate(lanes):
        if lane.lane_id is None or lane.lanelet_id is None:
            return None
        lanelet = lanelet_by_id.get(lane.lanelet_id)
        if lanelet is None:
            return None
        cap = _lane_cap(lanelet, lane.lane_id, reference_xy)
        current_sign = 1 if lane.lane_id > 0 else -1
        if lane_sign not in (0, current_sign):
            return None
        lane_sign = current_sign
        boundary_samples[index].append(cap[0])
        boundary_samples[index + 1].append(cap[1])

    boundaries = np.asarray(
        [
            np.mean(np.asarray(samples, dtype=float), axis=0)
            for samples in boundary_samples
        ],
        dtype=float,
    )
    lateral = boundaries[-1, :2] - boundaries[0, :2]
    if float(np.linalg.norm(lateral)) <= DEFAULT_CONFIG.geometry.epsilon:
        return None
    tangent = lane_sign * np.array([lateral[1], -lateral[0]], dtype=float)
    tangent /= float(np.linalg.norm(tangent))
    if road.reference_start_xyz is not None and road.reference_end_xyz is not None:
        road_direction = np.asarray(
            road.reference_end_xyz[:2], dtype=float
        ) - np.asarray(road.reference_start_xyz[:2], dtype=float)
        if float(np.dot(tangent, road_direction)) < 0.0:
            tangent = -tangent
    heading = math.atan2(float(tangent[1]), float(tangent[0]))
    side_direction = lane_sign * np.array([-tangent[1], tangent[0]], dtype=float)
    widths = tuple(
        float(np.dot(end[:2] - start[:2], side_direction))
        for start, end in zip(boundaries[:-1], boundaries[1:])
    )
    if any(
        not math.isfinite(width) or width <= DEFAULT_CONFIG.geometry.epsilon
        for width in widths
    ):
        return None
    return SharedPhysicalCrossSection(
        reference_xyz=tuple(float(value) for value in boundaries[0]),
        heading=heading,
        boundary_xyz=tuple(
            tuple(float(value) for value in boundary) for boundary in boundaries
        ),
        lane_widths=widths,
    )


def _unify_junction_incoming_cross_sections(
    plans: Sequence[PhysicalConnectionPlan],
    roads: Sequence[Road],
    lanelet_by_id: Dict[int, lanelet2.core.Lanelet],
) -> List[PhysicalConnectionPlan]:
    roads_by_id = {road.id: road for road in roads}
    unified = []
    for from_road_id in sorted({plan.from_road_id for plan in plans}):
        from_road = roads_by_id.get(from_road_id)
        if from_road is None:
            continue
        full_section = _road_source_cross_section(
            from_road,
            lanelet_by_id,
            at_start=False,
        )
        lanes = _ordered_one_side_lanes(from_road, at_start=False)
        lane_ids = [lane.lane_id for lane in lanes]
        if full_section is None or any(lane_id is None for lane_id in lane_ids):
            continue
        lane_index = {int(lane_id): index for index, lane_id in enumerate(lane_ids)}
        lane_sign = 1 if int(lane_ids[0]) > 0 else -1
        tangent = np.array(
            [math.cos(full_section.heading), math.sin(full_section.heading)],
            dtype=float,
        )
        side_direction = lane_sign * np.array(
            [-tangent[1], tangent[0]],
            dtype=float,
        )
        reference = np.asarray(full_section.reference_xyz, dtype=float)
        offsets = np.concatenate(([0.0], np.cumsum(full_section.lane_widths)))
        canonical_boundaries = tuple(
            (
                float(reference[0] + offset * side_direction[0]),
                float(reference[1] + offset * side_direction[1]),
                full_section.boundary_xyz[index][2],
            )
            for index, offset in enumerate(offsets)
        )

        for plan in plans:
            if plan.from_road_id != from_road_id:
                continue
            indices = [
                lane_index.get(correspondence.from_lane_id)
                for correspondence in plan.lane_correspondences
            ]
            if any(index is None for index in indices):
                continue
            concrete_indices = [int(index) for index in indices]
            first = min(concrete_indices)
            last = max(concrete_indices)
            if sorted(concrete_indices) != list(range(first, last + 1)):
                continue
            cross_section = SharedPhysicalCrossSection(
                reference_xyz=canonical_boundaries[first],
                heading=full_section.heading,
                boundary_xyz=canonical_boundaries[first : last + 2],
                lane_widths=full_section.lane_widths[first : last + 1],
            )
            unified.append(
                replace(
                    plan,
                    cross_section=cross_section,
                    from_endpoint=RoadEndpointConstraint(
                        road_id=plan.from_road_id,
                        at_start=False,
                        reference_xyz=full_section.reference_xyz,
                        heading=full_section.heading,
                    ),
                    to_endpoint=RoadEndpointConstraint(
                        road_id=plan.to_road_id,
                        at_start=True,
                        reference_xyz=cross_section.reference_xyz,
                        heading=cross_section.heading,
                    ),
                )
            )
    return unified


def build_ordinary_physical_connection_plans(
    roads: Sequence[Road],
    lanelet_by_id: Dict[int, lanelet2.core.Lanelet],
    *,
    protected_road_endpoints: Optional[Iterable[Tuple[int, bool]]] = None,
) -> List[PhysicalConnectionPlan]:
    """Plan all complete source-backed end-to-start ordinary continuations."""
    protected = set(protected_road_endpoints or ())
    roads_by_id = {road.id: road for road in roads}
    plans = []

    for from_road in roads:
        successor = from_road.link.successor if from_road.link else None
        if (
            (from_road.id, False) in protected
            or from_road.junction >= 0
            or successor is None
            or successor.element_type is not ElementType.ROAD
            or successor.contact_point is not ContactPoint.START
        ):
            continue
        to_road = roads_by_id.get(successor.element_id)
        if to_road is None or (to_road.id, True) in protected or to_road.junction >= 0:
            continue
        predecessor = to_road.link.predecessor if to_road.link else None
        if (
            predecessor is None
            or predecessor.element_type is not ElementType.ROAD
            or predecessor.element_id != from_road.id
            or predecessor.contact_point is not ContactPoint.END
        ):
            continue
        correspondences = _lane_correspondences(from_road, to_road)
        if correspondences is None:
            continue
        cross_section = _shared_cross_section(
            from_road,
            to_road,
            correspondences,
            lanelet_by_id,
        )
        if cross_section is None:
            continue

        connection_type = (
            PhysicalConnectionType.ORDINARY_MULTI_LANE_CONTINUATION
            if len(correspondences) > 1
            else PhysicalConnectionType.ORDINARY_CONTINUATION
        )
        endpoint = RoadEndpointConstraint(
            road_id=from_road.id,
            at_start=False,
            reference_xyz=cross_section.reference_xyz,
            heading=cross_section.heading,
        )
        plans.append(
            PhysicalConnectionPlan(
                from_road_id=from_road.id,
                to_road_id=to_road.id,
                connection_type=connection_type,
                lane_correspondences=correspondences,
                cross_section=cross_section,
                from_endpoint=endpoint,
                to_endpoint=RoadEndpointConstraint(
                    road_id=to_road.id,
                    at_start=True,
                    reference_xyz=cross_section.reference_xyz,
                    heading=cross_section.heading,
                ),
            )
        )
    return _reconcile_source_fixed_headings(plans, roads, lanelet_by_id)


def build_junction_incoming_physical_connection_plans(
    roads: Sequence[Road],
    lanelet_by_id: Dict[int, lanelet2.core.Lanelet],
    *,
    protected_road_endpoints: Optional[Iterable[Tuple[int, bool]]] = None,
) -> List[PhysicalConnectionPlan]:
    """Plan source-backed incoming-to-connecting-road lane subsets."""
    protected = set(protected_road_endpoints or ())
    roads_by_id = {road.id: road for road in roads}
    plans = []
    for to_road in roads:
        predecessor = to_road.link.predecessor if to_road.link else None
        if (
            to_road.junction < 0
            or (to_road.id, True) in protected
            or predecessor is None
            or predecessor.element_type is not ElementType.ROAD
            or predecessor.contact_point is not ContactPoint.END
        ):
            continue
        from_road = roads_by_id.get(predecessor.element_id)
        if (
            from_road is None
            or from_road.junction >= 0
            or (from_road.id, False) in protected
        ):
            continue
        correspondences = _junction_incoming_lane_correspondences(
            from_road,
            to_road,
        )
        if correspondences is None:
            continue
        cross_section = _shared_cross_section(
            from_road,
            to_road,
            correspondences,
            lanelet_by_id,
        )
        if cross_section is None:
            continue
        plans.append(
            PhysicalConnectionPlan(
                from_road_id=from_road.id,
                to_road_id=to_road.id,
                connection_type=PhysicalConnectionType.JUNCTION_INCOMING,
                lane_correspondences=correspondences,
                cross_section=cross_section,
                from_endpoint=RoadEndpointConstraint(
                    road_id=from_road.id,
                    at_start=False,
                    reference_xyz=cross_section.reference_xyz,
                    heading=cross_section.heading,
                ),
                to_endpoint=RoadEndpointConstraint(
                    road_id=to_road.id,
                    at_start=True,
                    reference_xyz=cross_section.reference_xyz,
                    heading=cross_section.heading,
                ),
            )
        )
    return _unify_junction_incoming_cross_sections(
        plans,
        roads,
        lanelet_by_id,
    )


def _source_terminal_heading(
    road: Road,
    lanelet_by_id: Dict[int, lanelet2.core.Lanelet],
    *,
    at_start: bool,
) -> Optional[float]:
    """Source reference tangent at one road end over a non-degenerate span."""
    points = _oriented_reference_points(road, lanelet_by_id)
    if points is None:
        return None
    min_span = DEFAULT_CONFIG.parampoly3.min_segment_length
    anchor = points[0] if at_start else points[-1]
    ordered = points if at_start else points[::-1]
    for other in ordered[1:]:
        segment = other[:2] - anchor[:2]
        span = float(np.linalg.norm(segment))
        if span >= min_span:
            if at_start:
                return math.atan2(float(segment[1]), float(segment[0]))
            return math.atan2(float(-segment[1]), float(-segment[0]))
    return None


@dataclass(frozen=True)
class _StubEdge:
    """One lane-level hop through a synthetic divergence/merge stub road."""

    stub_road_id: int
    from_road_id: int
    from_lane_id: int
    to_road_id: int
    to_lane_id: int


def _divergence_stub_edge(road: Road) -> Optional[_StubEdge]:
    """Recognize a #291 synthetic stub road and return its lane-level edge.

    Synthetic divergence/merge connectors are single-lane junction roads with
    no source lanelet, at most ``divergence_endpoint_tolerance`` long, whose
    road link joins one road end to one road start. Anything else (real
    junction connecting roads, source-backed connectors) is rejected.
    """
    if (
        road.junction < 0
        or road.length > DEFAULT_CONFIG.geometry.divergence_endpoint_tolerance
        or road.link is None
        or road.link.predecessor is None
        or road.link.successor is None
        or road.link.predecessor.element_type is not ElementType.ROAD
        or road.link.successor.element_type is not ElementType.ROAD
        or road.link.predecessor.contact_point is not ContactPoint.END
        or road.link.successor.contact_point is not ContactPoint.START
    ):
        return None
    lanes = list(_non_center_lanes(road, at_start=True).values())
    if len(lanes) != 1:
        return None
    lane = lanes[0]
    if (
        getattr(lane, "lanelet_id", None) is not None
        or lane.predecessor is None
        or lane.successor is None
    ):
        return None
    return _StubEdge(
        stub_road_id=road.id,
        from_road_id=road.link.predecessor.element_id,
        from_lane_id=lane.predecessor.id,
        to_road_id=road.link.successor.element_id,
        to_lane_id=lane.successor.id,
    )


def _divergence_shared_cross_section(
    trunk: Road,
    branch_edges: Sequence[Tuple[Road, _StubEdge]],
    lanelet_by_id: Dict[int, lanelet2.core.Lanelet],
) -> Optional[
    Tuple[SharedPhysicalCrossSection, Tuple[Tuple[float, float, float], ...]]
]:
    """Average source caps across the trunk start and all branch ends.

    Returns the full trunk-start cross-section plus straightened canonical
    boundary anchors (reference + cumulative widths along the shared normal)
    so branch blocks tile the trunk surface exactly.
    """
    if trunk.reference_start_xyz is None or trunk.reference_end_xyz is None:
        return None
    trunk_lanes = _ordered_one_side_lanes(trunk, at_start=True)
    trunk_lane_index = {lane.lane_id: index for index, lane in enumerate(trunk_lanes)}
    trunk_reference_xy = np.asarray(trunk.reference_start_xyz[:2], dtype=float)

    boundary_samples: List[List[np.ndarray]] = [[] for _ in range(len(trunk_lanes) + 1)]
    lane_sign = 0
    for branch, edge in branch_edges:
        if branch.reference_end_xyz is None:
            return None
        branch_reference_xy = np.asarray(branch.reference_end_xyz[:2], dtype=float)
        branch_lane = _non_center_lanes(branch, at_start=False).get(edge.from_lane_id)
        trunk_index = trunk_lane_index.get(edge.to_lane_id)
        if branch_lane is None or trunk_index is None:
            return None
        trunk_lane = trunk_lanes[trunk_index]
        if branch_lane.lanelet_id is None or trunk_lane.lanelet_id is None:
            return None
        branch_lanelet = lanelet_by_id.get(branch_lane.lanelet_id)
        trunk_lanelet = lanelet_by_id.get(trunk_lane.lanelet_id)
        if branch_lanelet is None or trunk_lanelet is None:
            return None
        branch_cap = _lane_cap(
            branch_lanelet,
            edge.from_lane_id,
            branch_reference_xy,
        )
        trunk_cap = _lane_cap(trunk_lanelet, edge.to_lane_id, trunk_reference_xy)
        if not _caps_match(branch_cap, trunk_cap):
            return None
        current_sign = 1 if edge.from_lane_id > 0 else -1
        if (edge.to_lane_id > 0) != (edge.from_lane_id > 0):
            return None
        if lane_sign not in (0, current_sign):
            return None
        lane_sign = current_sign
        boundary_samples[trunk_index].extend((branch_cap[0], trunk_cap[0]))
        boundary_samples[trunk_index + 1].extend((branch_cap[1], trunk_cap[1]))

    if any(not samples for samples in boundary_samples):
        return None
    boundaries = np.asarray(
        [
            np.mean(np.asarray(samples, dtype=float), axis=0)
            for samples in boundary_samples
        ],
        dtype=float,
    )
    lateral = boundaries[-1, :2] - boundaries[0, :2]
    if float(np.linalg.norm(lateral)) <= DEFAULT_CONFIG.geometry.epsilon:
        return None
    tangent = lane_sign * np.array([lateral[1], -lateral[0]], dtype=float)
    tangent /= float(np.linalg.norm(tangent))
    trunk_direction = np.asarray(trunk.reference_end_xyz[:2], dtype=float) - np.asarray(
        trunk.reference_start_xyz[:2], dtype=float
    )
    if float(np.dot(tangent, trunk_direction)) < 0.0:
        tangent = -tangent
    heading = math.atan2(float(tangent[1]), float(tangent[0]))
    side_direction = lane_sign * np.array([-tangent[1], tangent[0]], dtype=float)

    widths = tuple(
        float(np.dot(end[:2] - start[:2], side_direction))
        for start, end in zip(boundaries[:-1], boundaries[1:])
    )
    if any(
        not math.isfinite(width) or width <= DEFAULT_CONFIG.geometry.epsilon
        for width in widths
    ):
        return None

    offsets = np.concatenate(([0.0], np.cumsum(widths)))
    reference = boundaries[0]
    canonical_boundaries = tuple(
        (
            float(reference[0] + offset * side_direction[0]),
            float(reference[1] + offset * side_direction[1]),
            float(boundaries[index][2]),
        )
        for index, offset in enumerate(offsets)
    )
    cross_section = SharedPhysicalCrossSection(
        reference_xyz=canonical_boundaries[0],
        heading=heading,
        boundary_xyz=canonical_boundaries,
        lane_widths=widths,
    )
    return cross_section, canonical_boundaries


def build_divergence_physical_connection_plans(
    roads: Sequence[Road],
    lanelet_by_id: Dict[int, lanelet2.core.Lanelet],
    *,
    protected_road_endpoints: Optional[Iterable[Tuple[int, bool]]] = None,
) -> List[PhysicalConnectionPlan]:
    """Plan seams where branch road ends tile one trunk road start exactly.

    Synthetic divergence/merge junctions (#291) join roads through near
    zero-length stub connectors. When the stub lane links partition the
    trunk's start cross-section — every trunk lane fed by exactly one branch
    lane, every branch lane used exactly once anywhere — the interface is a
    physical continuation split only by road grouping, and both sides must
    share one source-backed cross-section. True forks (a branch lane feeding
    several trunks, or several branches feeding one trunk lane) keep their
    genuine apex geometry and are skipped.
    """
    protected = set(protected_road_endpoints or ())
    roads_by_id = {road.id: road for road in roads}

    edges: List[_StubEdge] = []
    for road in roads:
        edge = _divergence_stub_edge(road)
        if edge is not None:
            edges.append(edge)
    if not edges:
        return []

    from_use_count: Dict[Tuple[int, int], int] = {}
    to_use_count: Dict[Tuple[int, int], int] = {}
    for edge in edges:
        from_key = (edge.from_road_id, edge.from_lane_id)
        to_key = (edge.to_road_id, edge.to_lane_id)
        from_use_count[from_key] = from_use_count.get(from_key, 0) + 1
        to_use_count[to_key] = to_use_count.get(to_key, 0) + 1

    groups: Dict[int, List[_StubEdge]] = {}
    for edge in edges:
        groups.setdefault(edge.to_road_id, []).append(edge)

    plans: List[PhysicalConnectionPlan] = []
    for trunk_id in sorted(groups):
        group = groups[trunk_id]
        trunk = roads_by_id.get(trunk_id)
        if trunk is None or trunk.junction >= 0 or (trunk_id, True) in protected:
            continue
        trunk_lanes = _ordered_one_side_lanes(trunk, at_start=True)
        if not trunk_lanes or any(lane.lane_id is None for lane in trunk_lanes):
            continue
        trunk_lane_index = {
            lane.lane_id: index for index, lane in enumerate(trunk_lanes)
        }
        covered = [edge.to_lane_id for edge in group]
        if sorted(covered, key=abs) != sorted(trunk_lane_index, key=abs) or len(
            set(covered)
        ) != len(covered):
            continue
        if any(
            from_use_count[(edge.from_road_id, edge.from_lane_id)] != 1
            or to_use_count[(edge.to_road_id, edge.to_lane_id)] != 1
            for edge in group
        ):
            continue

        branch_edges: List[Tuple[Road, _StubEdge]] = []
        valid = True
        for edge in group:
            branch = roads_by_id.get(edge.from_road_id)
            if (
                branch is None
                or branch.junction >= 0
                or (branch.id, False) in protected
            ):
                valid = False
                break
            branch_edges.append((branch, edge))
        if not valid or len({branch.id for branch, _edge in branch_edges}) < 2:
            continue

        # Each branch must contribute all of its lanes as one contiguous,
        # order-preserving block of trunk lanes.
        edges_by_branch: Dict[int, List[_StubEdge]] = {}
        for branch, edge in branch_edges:
            edges_by_branch.setdefault(branch.id, []).append(edge)
        for branch_id, branch_group in edges_by_branch.items():
            branch = roads_by_id[branch_id]
            branch_lanes = _ordered_one_side_lanes(branch, at_start=False)
            branch_group.sort(key=lambda edge: abs(edge.from_lane_id))
            if [edge.from_lane_id for edge in branch_group] != [
                lane.lane_id for lane in branch_lanes
            ]:
                valid = False
                break
            indices = [trunk_lane_index[edge.to_lane_id] for edge in branch_group]
            if indices != list(range(indices[0], indices[0] + len(indices))):
                valid = False
                break
        if not valid:
            continue

        shared = _divergence_shared_cross_section(
            trunk,
            branch_edges,
            lanelet_by_id,
        )
        if shared is None:
            continue
        full_section, canonical_boundaries = shared

        # Every participating road must already point close enough to the
        # shared cross-section normal that the emitted terminal Bezier can
        # blend the residual smoothly. Larger mismatches would force the
        # point-move fallback, which folds the terminal boundary instead of
        # closing the seam — leave those interfaces unplanned.
        tolerance = (
            DEFAULT_CONFIG.geometry.terminal_micro_kink_support_heading_tolerance
        )
        terminal_headings = [
            _source_terminal_heading(trunk, lanelet_by_id, at_start=True)
        ]
        terminal_headings.extend(
            _source_terminal_heading(branch, lanelet_by_id, at_start=False)
            for branch, _edge in branch_edges
        )
        if any(
            heading is None
            or abs(
                math.atan2(
                    math.sin(heading - full_section.heading),
                    math.cos(heading - full_section.heading),
                )
            )
            > tolerance
            for heading in terminal_headings
        ):
            continue

        group_plans: List[PhysicalConnectionPlan] = []
        for branch_id in sorted(edges_by_branch):
            branch_group = edges_by_branch[branch_id]
            branch = roads_by_id[branch_id]
            first = trunk_lane_index[branch_group[0].to_lane_id]
            last = trunk_lane_index[branch_group[-1].to_lane_id]
            correspondences = []
            for edge in branch_group:
                branch_lane = _non_center_lanes(branch, at_start=False)[
                    edge.from_lane_id
                ]
                trunk_lane = trunk_lanes[trunk_lane_index[edge.to_lane_id]]
                correspondences.append(
                    LogicalLaneCorrespondence(
                        from_lanelet_id=branch_lane.lanelet_id,
                        from_lane_id=edge.from_lane_id,
                        to_lanelet_id=trunk_lane.lanelet_id,
                        to_lane_id=edge.to_lane_id,
                    )
                )
            block_section = SharedPhysicalCrossSection(
                reference_xyz=canonical_boundaries[first],
                heading=full_section.heading,
                boundary_xyz=canonical_boundaries[first : last + 2],
                lane_widths=full_section.lane_widths[first : last + 1],
            )
            group_plans.append(
                PhysicalConnectionPlan(
                    from_road_id=branch.id,
                    to_road_id=trunk.id,
                    connection_type=PhysicalConnectionType.MERGE,
                    lane_correspondences=tuple(correspondences),
                    cross_section=block_section,
                    from_endpoint=RoadEndpointConstraint(
                        road_id=branch.id,
                        at_start=False,
                        reference_xyz=canonical_boundaries[first],
                        heading=full_section.heading,
                    ),
                    to_endpoint=RoadEndpointConstraint(
                        road_id=trunk.id,
                        at_start=True,
                        reference_xyz=canonical_boundaries[0],
                        heading=full_section.heading,
                    ),
                )
            )

        reconciled = _reconcile_source_fixed_headings(
            group_plans,
            roads,
            lanelet_by_id,
        )
        if len(reconciled) != len(group_plans):
            continue
        headings = [plan.to_endpoint.heading for plan in reconciled]
        if any(
            abs(math.atan2(math.sin(h - headings[0]), math.cos(h - headings[0])))
            > DEFAULT_CONFIG.geometry.terminal_micro_kink_support_heading_tolerance
            for h in headings[1:]
        ):
            continue
        plans.extend(reconciled)
    return plans


def endpoint_constraints_by_road(
    plans: Sequence[PhysicalConnectionPlan],
) -> Dict[int, Dict[str, RoadEndpointConstraint]]:
    """Index compatible atomic connection constraints by road and side."""
    constraints: Dict[int, Dict[str, RoadEndpointConstraint]] = {}
    for plan in plans:
        for endpoint in (plan.from_endpoint, plan.to_endpoint):
            side = "start" if endpoint.at_start else "end"
            existing = constraints.setdefault(endpoint.road_id, {}).get(side)
            if existing is not None:
                position_gap = float(
                    np.linalg.norm(
                        np.asarray(existing.reference_xyz[:2])
                        - np.asarray(endpoint.reference_xyz[:2])
                    )
                )
                heading_gap = abs(
                    math.atan2(
                        math.sin(existing.heading - endpoint.heading),
                        math.cos(existing.heading - endpoint.heading),
                    )
                )
                if (
                    position_gap > DEFAULT_CONFIG.geometry.point_distance_threshold
                    or heading_gap
                    > DEFAULT_CONFIG.geometry.terminal_micro_kink_support_heading_tolerance
                ):
                    continue
            constraints[endpoint.road_id][side] = endpoint
    return constraints


def _width_value_and_derivative(lane, station: float) -> Tuple[float, float]:
    if not lane.widths:
        return 0.0, 0.0
    active = lane.widths[0]
    for record in lane.widths:
        if record.s_offset <= station + DEFAULT_CONFIG.geometry.epsilon:
            active = record
        else:
            break
    ds = station - active.s_offset
    value = active.a + active.b * ds + active.c * ds**2 + active.d * ds**3
    derivative = active.b + 2.0 * active.c * ds + 3.0 * active.d * ds**2
    return float(value), float(derivative)


def _shape_preserving_transition_length(
    lane,
    road_length: float,
    *,
    at_start: bool,
    width: float,
    derivative: float,
    minimum_length: float,
) -> float:
    """Choose the shortest width-knot span admitting monotone Hermite blending."""
    maximum_length = min(
        road_length,
        DEFAULT_CONFIG.geometry.physical_connection_bezier_handle_length,
    )
    candidates = {minimum_length, maximum_length}
    candidates.update(
        (record.s_offset if at_start else road_length - record.s_offset)
        for record in lane.widths
        if minimum_length - DEFAULT_CONFIG.geometry.epsilon
        <= (record.s_offset if at_start else road_length - record.s_offset)
        <= maximum_length + DEFAULT_CONFIG.geometry.epsilon
    )
    for transition_length in sorted(candidates):
        if transition_length <= DEFAULT_CONFIG.geometry.epsilon:
            continue
        support_station = (
            transition_length if at_start else road_length - transition_length
        )
        support_value, support_derivative = _width_value_and_derivative(
            lane,
            support_station,
        )
        if at_start:
            delta = support_value - width
            start_derivative = derivative
            end_derivative = support_derivative
        else:
            delta = width - support_value
            start_derivative = support_derivative
            end_derivative = derivative
        secant = delta / transition_length
        if abs(secant) <= DEFAULT_CONFIG.geometry.epsilon:
            if (
                abs(start_derivative) <= DEFAULT_CONFIG.geometry.epsilon
                and abs(end_derivative) <= DEFAULT_CONFIG.geometry.epsilon
            ):
                return transition_length
            continue
        alpha = start_derivative / secant
        beta = end_derivative / secant
        if (
            alpha >= -DEFAULT_CONFIG.geometry.epsilon
            and beta >= -DEFAULT_CONFIG.geometry.epsilon
            and alpha + beta <= 3.0 + DEFAULT_CONFIG.geometry.epsilon
        ):
            return transition_length
    return minimum_length


def _constrain_width_endpoint(
    lane,
    road_length: float,
    *,
    at_start: bool,
    width: float,
    derivative: float,
    transition_length: float = 0.0,
) -> bool:
    if not lane.widths or road_length <= DEFAULT_CONFIG.geometry.epsilon:
        return False
    records = sorted(lane.widths, key=lambda record: record.s_offset)
    transition_length = min(max(0.0, transition_length), road_length)
    if transition_length > DEFAULT_CONFIG.geometry.epsilon:
        transition_length = _shape_preserving_transition_length(
            lane,
            road_length,
            at_start=at_start,
            width=width,
            derivative=derivative,
            minimum_length=transition_length,
        )
        original = lane.widths
        if at_start:
            support_station = transition_length
            support_value, support_derivative = _width_value_and_derivative(
                lane,
                support_station,
            )
            span = support_station
            delta = support_value - width - derivative * span
            derivative_delta = support_derivative - derivative
            c = 3.0 * delta / span**2 - derivative_delta / span
            d = derivative_delta / span**2 - 2.0 * delta / span**3

            active = records[0]
            for record in records:
                if record.s_offset <= support_station + DEFAULT_CONFIG.geometry.epsilon:
                    active = record
                else:
                    break
            ds = support_station - active.s_offset
            shifted = LaneWidth(
                support_station,
                support_value,
                support_derivative,
                active.c + 3.0 * active.d * ds,
                active.d,
            )
            records = [
                LaneWidth(0.0, width, derivative, c, d),
                shifted,
                *[
                    record
                    for record in records
                    if record.s_offset
                    > support_station + DEFAULT_CONFIG.geometry.epsilon
                ],
            ]
        else:
            support_station = road_length - transition_length
            support_value, support_derivative = _width_value_and_derivative(
                lane,
                support_station,
            )
            span = transition_length
            delta = width - support_value - support_derivative * span
            derivative_delta = derivative - support_derivative
            c = 3.0 * delta / span**2 - derivative_delta / span
            d = derivative_delta / span**2 - 2.0 * delta / span**3
            records = [
                *[
                    record
                    for record in records
                    if record.s_offset
                    < support_station - DEFAULT_CONFIG.geometry.epsilon
                ],
                LaneWidth(
                    support_station,
                    support_value,
                    support_derivative,
                    c,
                    d,
                ),
            ]

        samples = np.linspace(
            0.0,
            road_length,
            DEFAULT_CONFIG.geometry.emission_validation_samples,
        )
        lane.widths = records
        valid = all(
            math.isfinite(value) and value >= -DEFAULT_CONFIG.geometry.epsilon
            for value, _derivative in (
                _width_value_and_derivative(lane, float(station)) for station in samples
            )
        )
        if not valid:
            lane.widths = original
            return False
        return True

    if at_start:
        record = records[0]
        next_station = records[1].s_offset if len(records) > 1 else road_length
        span = float(next_station - record.s_offset)
        if span <= DEFAULT_CONFIG.geometry.epsilon:
            return False
        end_value, end_derivative = _width_value_and_derivative(lane, next_station)
        a = width
        b = derivative
        delta = end_value - a - b * span
        derivative_delta = end_derivative - b
        c = 3.0 * delta / span**2 - derivative_delta / span
        d = derivative_delta / span**2 - 2.0 * delta / span**3
        records[0] = LaneWidth(record.s_offset, a, b, c, d)
    else:
        record = records[-1]
        span = float(road_length - record.s_offset)
        if span <= DEFAULT_CONFIG.geometry.epsilon:
            return False
        a = record.a
        b = record.b
        delta = width - a - b * span
        derivative_delta = derivative - b
        c = 3.0 * delta / span**2 - derivative_delta / span
        d = derivative_delta / span**2 - 2.0 * delta / span**3
        records[-1] = LaneWidth(record.s_offset, a, b, c, d)

    samples = np.linspace(
        0.0,
        road_length,
        DEFAULT_CONFIG.geometry.emission_validation_samples,
    )
    original = lane.widths
    lane.widths = records
    valid = all(
        math.isfinite(value) and value >= -DEFAULT_CONFIG.geometry.epsilon
        for value, _derivative in (
            _width_value_and_derivative(lane, float(station)) for station in samples
        )
    )
    if not valid:
        lane.widths = original
        return False
    return True


def apply_physical_connection_width_constraints(
    plans: Sequence[PhysicalConnectionPlan],
    roads: Sequence[Road],
) -> int:
    """Apply shared endpoint width and slope conditions without new records."""
    roads_by_id = {road.id: road for road in roads}
    changed = 0
    for plan in plans:
        from_road = roads_by_id.get(plan.from_road_id)
        to_road = roads_by_id.get(plan.to_road_id)
        if from_road is None or to_road is None:
            continue
        from_lanes = _non_center_lanes(from_road, at_start=False)
        to_lanes = _non_center_lanes(to_road, at_start=True)
        tangent = np.array(
            [
                math.cos(plan.cross_section.heading),
                math.sin(plan.cross_section.heading),
            ],
            dtype=float,
        )
        boundaries = np.asarray(plan.cross_section.boundary_xyz, dtype=float)
        longitudinal_offsets = boundaries[:, :2] @ tangent
        endpoint_stagger = float(
            np.max(longitudinal_offsets) - np.min(longitudinal_offsets)
        )
        transition_length = 0.0
        if endpoint_stagger > DEFAULT_CONFIG.geometry.epsilon:
            transition_length = min(
                endpoint_stagger
                + DEFAULT_CONFIG.geometry.emission_width_refinement_min_interval,
                DEFAULT_CONFIG.geometry.physical_connection_bezier_handle_length,
            )
        for correspondence, width in zip(
            plan.lane_correspondences,
            plan.cross_section.lane_widths,
        ):
            from_lane = from_lanes.get(correspondence.from_lane_id)
            to_lane = to_lanes.get(correspondence.to_lane_id)
            if from_lane is None or to_lane is None:
                continue
            if _constrain_width_endpoint(
                from_lane,
                from_road.length,
                at_start=False,
                width=width,
                derivative=0.0,
                transition_length=transition_length,
            ):
                changed += 1
            if _constrain_width_endpoint(
                to_lane,
                to_road.length,
                at_start=True,
                width=width,
                derivative=0.0,
                transition_length=transition_length,
            ):
                changed += 1
    return changed
