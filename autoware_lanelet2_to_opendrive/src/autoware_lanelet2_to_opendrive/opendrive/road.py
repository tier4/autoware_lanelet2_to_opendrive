"""OpenDRIVE road definitions."""

import logging
from dataclasses import dataclass
from typing import (
    TYPE_CHECKING,
    Dict,
    List,
    NamedTuple,
    Optional,
    Set,
    Tuple,
    Union,
    cast,
)

import lanelet2
import lxml.etree as ET
import numpy as np
from lanelet2.routing import RoutingGraph, RoutingCostDistance
from tqdm import tqdm

from ..conversion_config import (
    ArcSpiralConfig,
    LaneLinksContext,
    ParamPoly3Config,
    WidthEstimationConfig,
)
from ..util import LaneletInput, filter_lanelets_by_subtype, to_lanelet_list
from .elevation import ElevationProfile
from .enums import ContactPoint, ElementType, RoadType, TrafficRule
from .geometry import (
    Arc,
    GeometryBase,
    Line,
    ParamPoly3,
    PlanView,
    evaluate_plan_view_world,
)
from .lane_elements import LaneLink, RoadTypeDefinition, RoadTypeSpeed, SpeedUnit
from .lane_sections import Lanes
from .reference_line import ReferenceLine
from .road_links import Predecessor, RoadLink, Successor

logger = logging.getLogger(__name__)


def _build_planview_geometries(
    spline,
    parampoly3_config: Optional[ParamPoly3Config],
    arcspiral_config: Optional[ArcSpiralConfig],
) -> List[GeometryBase]:
    """Emit a list of planView geometry primitives for ``spline``.

    Honours ``arcspiral_config.enabled`` — when False (or None), behaviour
    is bit-for-bit identical to the previous ParamPoly3-only path.
    """
    from .geometry_classifier import (
        ArcRun,
        ClassifiedSegment,
        LineRun,
        ParamPoly3Run,
        classify_spline,
    )
    from ..config import DEFAULT_CONFIG

    if arcspiral_config is None or not arcspiral_config.enabled:
        return cast(
            List[GeometryBase],
            ParamPoly3.from_spline(spline, config=parampoly3_config),
        )

    pp3_cfg = parampoly3_config or ParamPoly3Config()
    runs: List[ClassifiedSegment] = classify_spline(
        spline,
        config=arcspiral_config,
        constants=DEFAULT_CONFIG.arcspiral,
    )

    out: List[GeometryBase] = []
    for run in runs:
        if isinstance(run, LineRun):
            out.append(Line.from_spline_window(spline, run.s_start, run.s_end))
        elif isinstance(run, ArcRun):
            out.append(
                Arc.from_spline_window(spline, run.s_start, run.s_end, run.curvature)
            )
        elif isinstance(run, ParamPoly3Run):
            # Re-use existing per-window splitting (CARLA min length).
            out.extend(_emit_paramPoly3_run(spline, run, pp3_cfg))
        else:
            raise TypeError(f"Unknown ClassifiedSegment type: {type(run)!r}")
    return out


def _emit_paramPoly3_run(
    spline,
    run: "ParamPoly3Run",
    pp3_cfg: ParamPoly3Config,
) -> List[ParamPoly3]:
    """Emit one or more ParamPoly3 segments covering a ParamPoly3Run.

    Honours ``pp3_cfg.min_segment_length`` by sub-dividing the run if
    needed (mirrors the per-segment loop in ``ParamPoly3.from_spline``).
    """
    length = run.s_end - run.s_start
    target = pp3_cfg.default_segment_length if pp3_cfg.enabled else length
    n = max(1, int(np.ceil(length / max(target, pp3_cfg.min_segment_length))))
    out: List[ParamPoly3] = []
    for i in range(n):
        s0 = run.s_start + (i / n) * length
        s1 = run.s_start + ((i + 1) / n) * length
        if s1 - s0 < pp3_cfg.min_segment_length:
            continue
        out.append(
            ParamPoly3.from_spline_window(
                spline,
                s0,
                s1,
                coefficient_epsilon=pp3_cfg.coefficient_epsilon,
            )
        )
    return out


def _resolve_candidate_road_ids(
    groups: List[Set["lanelet2.core.Lanelet"]],
    mapping: Dict[int, int],
) -> List[int]:
    """Return the distinct regular-road IDs covered by ``groups``.

    Used by both ``Road.construct_from_lanelet_map`` (to decide whether to
    set a road-level link or defer to the divergence/merge synthesis pass)
    and the synthesis pass itself. Returns ``[]`` when any group contains a
    ``turn_direction`` lanelet — that signals a real-junction lanelet group
    whose link is owned by the existing junction pipeline (issue #291).
    Order of first appearance is preserved so callers that fall back to
    "first wins" behave the same as the previous helper.
    """
    has_real_junction = any(
        "turn_direction" in ll.attributes for g in groups for ll in g
    )
    if has_real_junction:
        return []
    seen: List[int] = []
    for group in groups:
        for ll in group:
            rid = mapping.get(ll.id)
            if rid is not None and rid not in seen:
                seen.append(rid)
    return seen


class ConstructedRoadsResult(NamedTuple):
    """Return bundle for ``Road.construct_from_lanelet_map`` (issue #291).

    Attributes:
        roads: Successfully built regular roads.
        lanelet_to_road: ``lanelet_id -> road_id`` for built roads.
        num_groups: Total number of adjacent groups (including failed ones)
            so callers keep the existing offset for connecting-road IDs.
        deferred_predecessor_candidates: ``road_id -> [candidate road IDs]``
            for roads whose predecessor side resolves to >= 2 distinct
            regular roads. The road-level predecessor link is left unset
            for these roads; the divergence/merge synthesis pass owns it.
        deferred_successor_candidates: same, mirror.
        routing_graph: The :class:`RoutingGraph` already built for this map.
            Returned so downstream passes (e.g. divergence synthesis) can
            reuse it instead of paying the construction cost a second time.
    """

    roads: List["Road"]
    lanelet_to_road: Dict[int, int]
    num_groups: int
    deferred_predecessor_candidates: Dict[int, List[int]]
    deferred_successor_candidates: Dict[int, List[int]]
    routing_graph: RoutingGraph


def _evaluate_plan_view_world(
    plan_view: PlanView, at_start: bool
) -> Optional[Tuple[float, float]]:
    """Evaluate the planView's reference-line XY position at its s=0 or
    s=length endpoint.

    This returns the coordinate that the rendered planView actually
    resolves to, which can differ slightly from the raw OSM boundary
    endpoint because of spline-fit approximation.  Supports
    ``<paramPoly3>``, ``<line>``, and ``<arc>`` geometry primitives.
    """
    if plan_view is None or not plan_view.geometries:
        return None

    geom = plan_view.geometries[0] if at_start else plan_view.geometries[-1]
    p = 0.0 if at_start else geom.length

    coeffs: Optional[Tuple[float, float, float, float, float, float, float, float]] = (
        None
    )
    arc_curvature: Optional[float] = None
    if isinstance(geom, ParamPoly3):
        coeffs = (
            geom.aU,
            geom.bU,
            geom.cU,
            geom.dU,
            geom.aV,
            geom.bV,
            geom.cV,
            geom.dV,
        )
    elif isinstance(geom, Arc):
        arc_curvature = geom.curvature

    return evaluate_plan_view_world(geom.x, geom.y, geom.hdg, p, coeffs, arc_curvature)


def _evaluate_elevation_profile(elevation_profile: ElevationProfile, s: float) -> float:
    """Evaluate the piecewise-cubic elevation profile at arc length ``s``."""
    if elevation_profile is None or not elevation_profile.elevations:
        return 0.0
    z = 0.0
    for elev in elevation_profile.elevations:
        if elev.s > s:
            break
        ds = s - elev.s
        z = elev.a + elev.b * ds + elev.c * ds * ds + elev.d * ds * ds * ds
    return z


def _evaluate_planview_endpoint_with_heading(
    plan_view: PlanView, at_start: bool
) -> Optional[Tuple[float, float, float]]:
    """Return ``(x, y, heading)`` at the ``s=0`` or ``s=length`` endpoint.

    Heading is the world-frame tangent angle, computed from the geometry's
    base ``hdg`` and (for ``paramPoly3``) the local UV derivatives evaluated
    at the endpoint parameter, or (for ``arc``) the accumulated curvature.
    Used by lane-aware junction pinning to apply a lateral normal offset
    along the rendered tangent.
    """
    if plan_view is None or not plan_view.geometries:
        return None

    geom = plan_view.geometries[0] if at_start else plan_view.geometries[-1]
    p = 0.0 if at_start else geom.length

    coeffs: Optional[Tuple[float, float, float, float, float, float, float, float]] = (
        None
    )
    arc_curvature: Optional[float] = None
    if isinstance(geom, ParamPoly3):
        coeffs = (
            geom.aU,
            geom.bU,
            geom.cU,
            geom.dU,
            geom.aV,
            geom.bV,
            geom.cV,
            geom.dV,
        )
    elif isinstance(geom, Arc):
        arc_curvature = geom.curvature

    xy = evaluate_plan_view_world(geom.x, geom.y, geom.hdg, p, coeffs, arc_curvature)
    if xy is None:
        return None
    x_w, y_w = xy

    if isinstance(geom, ParamPoly3):
        du = geom.bU + 2.0 * geom.cU * p + 3.0 * geom.dU * p * p
        dv = geom.bV + 2.0 * geom.cV * p + 3.0 * geom.dV * p * p
        cos_h = float(np.cos(geom.hdg))
        sin_h = float(np.sin(geom.hdg))
        dx = du * cos_h - dv * sin_h
        dy = du * sin_h + dv * cos_h
        heading = float(np.arctan2(dy, dx))
    elif isinstance(geom, Arc):
        # For a constant-curvature arc the heading grows linearly with p.
        heading = float(geom.hdg + geom.curvature * p)
    else:
        heading = float(geom.hdg)

    return (float(x_w), float(y_w), heading)


def _evaluate_lane_width(lane: "Lane", s: float) -> Optional[float]:
    """Evaluate a Lane's piecewise-cubic width polynomial at ``s``.

    Returns ``None`` when the lane has no width data — callers must treat
    that as a failure rather than substituting ``0``, otherwise lane-aware
    pinning can silently collapse onto the reference line and mask
    malformed input.
    """
    if not lane.widths:
        return None
    seg = lane.widths[0]
    for w in lane.widths:
        if w.s_offset <= s:
            seg = w
        else:
            break
    ds = s - seg.s_offset
    return float(seg.a + seg.b * ds + seg.c * ds * ds + seg.d * ds * ds * ds)


if TYPE_CHECKING:
    from .signal import Signal
    from .lane import Lane
    from .junction import Junction
    from .objects import CrosswalkObject, StopLineObject
    from .geometry_classifier import ParamPoly3Run


@dataclass
class Road:
    """Road definition."""

    id: int = 0
    name: Optional[str] = None
    length: float = 0.0
    junction: int = -1
    rule: Optional[TrafficRule] = None
    plan_view: Optional[PlanView] = None
    elevation_profile: Optional[ElevationProfile] = None
    lanes: Optional[Lanes] = None
    link: Optional[RoadLink] = None
    signals: Optional[List["Signal"]] = None
    elevation_offset: float = 0.0  # Absolute elevation at road start (s=0)
    road_types: Optional[List[RoadTypeDefinition]] = None
    objects: Optional[List[Union["CrosswalkObject", "StopLineObject"]]] = None
    # World-frame 3D endpoints of the reference line (s=0 and s=length).
    # Populated by ``construct_from_lanelet_groups`` so the junction phase
    # can propagate these as overrides into connecting roads (P0-2).
    reference_start_xyz: Optional[Tuple[float, float, float]] = None
    reference_end_xyz: Optional[Tuple[float, float, float]] = None
    # Source lanelet IDs in left-to-right sorted order, as produced by
    # ``sort_adjacent_groups`` during construction.  The junction phase
    # uses this to map a *specific* predecessor lanelet to its lane index
    # inside this road so connecting-road endpoints can be pinned to the
    # correct rendered lane edge rather than the regular road's reference
    # line — see :meth:`evaluate_lane_anchor_xyz` and #437.
    sorted_lanelet_ids: Optional[List[int]] = None

    def evaluate_lane_anchor_xyz(
        self,
        sorted_index: int,
        at_start: bool,
    ) -> Optional[Tuple[float, float, float]]:
        """Return the rendered ``(x, y, z)`` of the anchor boundary of the
        lanelet at ``sorted_index`` at the road's ``s=0`` or ``s=length``.

        ``sorted_index`` is the 0-based position in
        :attr:`sorted_lanelet_ids` (left-to-right).  The anchor boundary is
        ``leftBound`` for RHT roads and ``rightBound`` for LHT roads — the
        side that joins the reference line for the outermost lanelet.

        For ``sorted_index == 0`` (RHT) or ``sorted_index == n-1`` (LHT)
        this is the reference line itself (lateral offset ``t = 0``) and
        the result equals :attr:`reference_start_xyz` /
        :attr:`reference_end_xyz`.  For other indices the result is offset
        laterally by the cumulative width of the lanes between the anchor
        and the reference line — the lane-aware pin target used when a
        connecting road joins a non-outermost lane of a regular road
        (see #437).

        Returns ``None`` if the road is missing geometry, lanes, or the
        sorted-lanelet index is out of range.
        """
        if (
            self.plan_view is None
            or not self.plan_view.geometries
            or self.lanes is None
            or not self.lanes.lane_sections
            or self.sorted_lanelet_ids is None
        ):
            return None
        n = len(self.sorted_lanelet_ids)
        if not (0 <= sorted_index < n):
            return None

        endpoint = _evaluate_planview_endpoint_with_heading(
            self.plan_view, at_start=at_start
        )
        if endpoint is None:
            return None
        x_ref, y_ref, heading = endpoint

        s = 0.0 if at_start else self.length

        lane_section = self.lanes.lane_sections[0]
        is_lht = self.rule == TrafficRule.LHT
        t = 0.0
        if is_lht:
            # LHT: lanelets at ``sorted_index k`` carry lane id ``n - k``.
            # The anchor (rightBound) of lane ``n - k`` sits at
            # ``t = + sum(widths of lanes 1 .. n - k - 1)`` — the widths of
            # the lanelets to its right (more rightward in sorted order).
            target_lane_id = n - sorted_index
            for lane_id in range(1, target_lane_id):
                lane = lane_section.left_lanes.get(lane_id)
                if lane is None:
                    return None
                w = _evaluate_lane_width(lane, s)
                if w is None:
                    return None
                t += w
        else:
            # RHT: lanelets at ``sorted_index k`` carry lane id ``-(k + 1)``.
            # The anchor (leftBound) of lane ``-(k + 1)`` sits at
            # ``t = - sum(widths of lanes -1 .. -k)`` — the widths of the
            # lanelets to its left in sorted order.
            for j in range(1, sorted_index + 1):
                lane = lane_section.right_lanes.get(-j)
                if lane is None:
                    return None
                w = _evaluate_lane_width(lane, s)
                if w is None:
                    return None
                t -= w

        nx = -float(np.sin(heading))
        ny = float(np.cos(heading))
        x = x_ref + t * nx
        y = y_ref + t * ny
        z = _evaluate_elevation_profile(self.elevation_profile, s)
        return (float(x), float(y), float(z))

    def to_xml(self) -> ET.Element:
        """Convert to XML element."""
        elem = ET.Element("road")
        elem.set("id", str(self.id))
        elem.set("length", str(self.length))
        elem.set("junction", str(self.junction))

        if self.name:
            elem.set("name", self.name)

        if self.rule:
            elem.set("rule", self.rule.value)

        if self.link:
            elem.append(self.link.to_xml())

        # Add road type definitions with speed limits
        if self.road_types:
            for road_type in self.road_types:
                elem.append(road_type.to_xml())

        if self.plan_view:
            elem.append(self.plan_view.to_xml())
        if self.elevation_profile:
            elem.append(self.elevation_profile.to_xml())
        if self.lanes:
            elem.append(self.lanes.to_xml())
        if self.objects:
            objects_elem = ET.SubElement(elem, "objects")
            for obj in self.objects:
                objects_elem.append(obj.to_xml())

        if self.signals:
            signals_elem = ET.SubElement(elem, "signals")

            # Generate signal elements
            for signal in self.signals:
                signals_elem.append(signal.to_xml())

            # Generate signalReference elements (Issue #135)
            # Each signal gets a corresponding signalReference on the reference line
            for signal in self.signals:
                signals_elem.append(signal.to_signal_reference_xml())

        return elem

    def add_predecessor(
        self,
        element_id: int,
        element_type: ElementType = ElementType.ROAD,
        contact_point: Optional[ContactPoint] = None,
    ) -> None:
        """Add a predecessor link to this road.

        Args:
            element_id: ID of the predecessor element
            element_type: Type of the predecessor element (road or junction)
            contact_point: Contact point of the predecessor (start or end)
        """
        if self.link is None:
            self.link = RoadLink()
        self.link.predecessor = Predecessor(
            element_type=element_type,
            element_id=element_id,
            contact_point=contact_point,
        )

    def add_successor(
        self,
        element_id: int,
        element_type: ElementType = ElementType.ROAD,
        contact_point: Optional[ContactPoint] = None,
    ) -> None:
        """Add a successor link to this road.

        Args:
            element_id: ID of the successor element
            element_type: Type of the successor element (road or junction)
            contact_point: Contact point of the successor (start or end)
        """
        if self.link is None:
            self.link = RoadLink()
        self.link.successor = Successor(
            element_type=element_type,
            element_id=element_id,
            contact_point=contact_point,
        )

    def get_lanelet_to_lane_mapping(self) -> Dict[int, int]:
        """Get mapping from lanelet ID to lane ID for all lanes in this road.

        Returns:
            Dictionary mapping lanelet_id -> lane_id
        """
        mapping: Dict[int, int] = {}

        if self.lanes is None:
            return mapping

        for lane_section in self.lanes.lane_sections:
            section_mapping = lane_section.get_lanelet_to_lane_mapping()
            mapping.update(section_mapping)

        return mapping

    def get_elevation_at_s(self, s: float) -> float:
        """Calculate road surface elevation at a given s-coordinate.

        Uses the elevation profile to compute the absolute elevation of the road
        surface at the specified position along the reference line. The elevation
        is calculated using cubic polynomial interpolation from the elevation profile.

        Args:
            s: Position along the road reference line (s-coordinate) in meters.

        Returns:
            Absolute road surface elevation at position s (in meters).
            Returns 0.0 if the road has no elevation profile.

        Example:
            >>> road = Road(...)
            >>> elevation = road.get_elevation_at_s(100.5)
            >>> print(f"Road elevation at s=100.5m: {elevation:.2f}m")
        """
        if not self.elevation_profile or not self.elevation_profile.elevations:
            return 0.0

        road_elevation_at_s = 0.0
        for elevation in self.elevation_profile.elevations:
            if elevation.s <= s:
                # Calculate distance from segment start
                ds = s - elevation.s
                # Evaluate cubic polynomial: elevation = a + b*ds + c*ds^2 + d*ds^3
                road_elevation_at_s = (
                    elevation.a
                    + elevation.b * ds
                    + elevation.c * ds * ds
                    + elevation.d * ds * ds * ds
                )
            else:
                break

        return road_elevation_at_s

    def get_half_width_at_s(self, s: float) -> float:
        """Calculate half of the innermost lane width at a given s-coordinate.

        Evaluates the width of the innermost driving lane (lane 1 for LHT,
        lane -1 for RHT) at position s and returns half of it. This places
        the signal at the lateral center of the innermost lane.

        Args:
            s: Position along the road reference line (s-coordinate) in meters.

        Returns:
            Signed half-width value. Positive for LHT, negative for RHT.
            Returns 0.0 if the road has no lanes.
        """
        if self.lanes is None or not self.lanes.lane_sections:
            return 0.0

        # Find the lane section that contains this s coordinate
        active_section = self.lanes.lane_sections[0]
        for section in self.lanes.lane_sections:
            if section.s_offset <= s:
                active_section = section
            else:
                break

        def _eval_width(widths, s_pos: float) -> float:
            """Evaluate cubic polynomial width at s_pos."""
            result = 0.0
            for w in widths:
                if w.s_offset <= s_pos:
                    ds = s_pos - w.s_offset
                    result = w.a + w.b * ds + w.c * ds * ds + w.d * ds * ds * ds
                else:
                    break
            return result

        # s relative to the lane section start
        s_local = s - active_section.s_offset

        # Use the innermost lane (closest to reference line) width / 2
        if active_section.left_lanes:
            innermost = active_section.left_lanes.get(1)
            if innermost is not None and innermost.widths:
                return _eval_width(innermost.widths, s_local) / 2.0

        if active_section.right_lanes:
            innermost = active_section.right_lanes.get(-1)
            if innermost is not None and innermost.widths:
                return -(_eval_width(innermost.widths, s_local) / 2.0)

        return 0.0

    def set_lane_links(self, context: LaneLinksContext) -> None:
        """Set lane predecessor and successor links based on lanelet connections.

        Args:
            context: LaneLinksContext containing all parameters needed for lane link setup
        """
        if self.lanes is None:
            return

        # Use provided routing graph or create a new one
        if context.routing_graph is None:
            traffic_rules = lanelet2.traffic_rules.create(
                lanelet2.traffic_rules.Locations.Germany,
                lanelet2.traffic_rules.Participants.Vehicle,
            )
            routing_graph = RoutingGraph(
                context.lanelet_map, traffic_rules, [RoutingCostDistance(0.0)]
            )
        else:
            routing_graph = context.routing_graph

        for lane_section in self.lanes.lane_sections:
            # Process left lanes
            for lane in lane_section.left_lanes.values():
                self._set_single_lane_links(
                    lane,
                    context.lanelet_map,
                    routing_graph,
                    context.lanelet_to_road_and_lane,
                    context.road_lane_ids,
                    context.road_id_to_road,
                )

            # Process right lanes
            for lane in lane_section.right_lanes.values():
                self._set_single_lane_links(
                    lane,
                    context.lanelet_map,
                    routing_graph,
                    context.lanelet_to_road_and_lane,
                    context.road_lane_ids,
                    context.road_id_to_road,
                )

    @staticmethod
    def _find_closest_lane(target_lane: int, available_lanes: list[int]) -> int:
        """Find the closest lane ID when exact match doesn't exist.

        This handles cases where a lane references a non-existent lane in the
        target road, typically when the number of lanes changes (e.g., 3 lanes
        merging into 2 lanes).

        Args:
            target_lane: The desired lane ID that doesn't exist
            available_lanes: Sorted list of existing lane IDs in the target road

        Returns:
            Closest existing lane ID

        Example:
            >>> _find_closest_lane(-3, [-1, -2])  # Lane -3 merges into -2
            -2
        """
        if not available_lanes:
            return -1  # Fallback to default lane

        # Find the lane with minimum distance to target_lane
        # For right lanes (negative IDs): -3 is closer to -2 than -1
        # For left lanes (positive IDs): 3 is closer to 2 than 1
        closest = min(available_lanes, key=lambda x: abs(x - target_lane))
        return closest

    def _set_single_lane_links(
        self,
        lane: "Lane",
        lanelet_map: lanelet2.core.LaneletMap,
        routing_graph: RoutingGraph,
        lanelet_to_road_and_lane: Dict[int, Tuple[int, int]],
        road_lane_ids: Optional[Dict[int, Set[int]]] = None,
        road_id_to_road: Optional[Dict[int, "Road"]] = None,
    ) -> None:
        """Set predecessor and successor for a single lane.

        Args:
            lane: The lane to set links for
            lanelet_map: The Lanelet2 map
            routing_graph: Routing graph for connectivity analysis
            lanelet_to_road_and_lane: Global mapping from lanelet_id to (road_id, lane_id)
            road_lane_ids: Optional mapping from road_id to set of existing lane_ids.
                          Used to validate that lane links reference existing lanes.
            road_id_to_road: Optional mapping from road_id to Road objects.
                            Used to check if target roads are connecting roads in junctions.
        """

        if lane.lanelet_id is None:
            return

        # Get the lanelet corresponding to this lane
        try:
            lanelet = lanelet_map.laneletLayer.get(lane.lanelet_id)
        except Exception:
            return

        # Get road link predecessor/successor for consistency check
        road_link_predecessor = self.link.predecessor if self.link else None
        road_link_successor = self.link.successor if self.link else None

        # Issue #124 Part 1 fix: For connecting roads (junction >= 0), allow lane
        # links even without road links
        is_connecting_road = self.junction is not None and self.junction >= 0

        # Find predecessor lanelets
        previous_lanelets = routing_graph.previous(lanelet)
        if previous_lanelets:
            # Take the first predecessor that maps to the road link's predecessor road
            for prev_ll in previous_lanelets:
                if prev_ll.id in lanelet_to_road_and_lane:
                    pred_road_id, pred_lane_id = lanelet_to_road_and_lane[prev_ll.id]
                    # Only set predecessor if it's in a different road
                    # (same road connections would be within lane sections)
                    if pred_road_id != self.id:
                        # Issue #202 fix: Check consistency with road link
                        # Lane links must only exist when corresponding road link exists
                        # This applies to both regular roads and connecting roads
                        if road_link_predecessor is None:
                            # Road has no predecessor link - skip lane link creation
                            # This prevents invalid lane connections that cause CARLA crashes
                            continue

                        # Check if road link predecessor is a junction
                        if road_link_predecessor is not None:
                            if (
                                road_link_predecessor.element_type
                                == ElementType.JUNCTION
                            ):
                                # This road's predecessor is a junction
                                # The lane's predecessor should be a connecting road in that
                                # junction
                                if road_id_to_road is not None:
                                    pred_road = road_id_to_road.get(pred_road_id)
                                    if pred_road is None:
                                        if not is_connecting_road:
                                            continue
                                    # Check if predecessor road is a connecting road in
                                    # the junction
                                    elif (
                                        pred_road.junction
                                        != road_link_predecessor.element_id
                                    ):
                                        if not is_connecting_road:
                                            continue
                                # If no road_id_to_road, we can't validate - skip for safety
                                # (unless this is a connecting road)
                                elif not is_connecting_road:
                                    continue
                            else:
                                # Road link predecessor is a regular road
                                # Lane link must reference the same road
                                # (unless this is a connecting road)
                                if (
                                    pred_road_id != road_link_predecessor.element_id
                                    and not is_connecting_road
                                ):
                                    # Lane predecessor differs from road link - check if it's a branching scenario
                                    # Allow if predecessor is a connecting road (junction branching)
                                    if road_id_to_road is not None:
                                        pred_road = road_id_to_road.get(pred_road_id)
                                        if (
                                            pred_road is not None
                                            and pred_road.junction is not None
                                            and pred_road.junction >= 0
                                        ):
                                            # Predecessor is a connecting road - allow lane branching
                                            pass
                                        else:
                                            # Predecessor is a regular road but doesn't match road link - skip
                                            continue
                                    else:
                                        # Cannot validate - skip for safety
                                        continue

                        # Validate that the lane exists in the predecessor road
                        if road_lane_ids is not None:
                            existing_lanes = road_lane_ids.get(pred_road_id, set())
                            if pred_lane_id not in existing_lanes:
                                # Lane doesn't exist, find closest lane
                                # This handles lane reduction scenarios (e.g., 3 lanes -> 2 lanes)
                                if existing_lanes:
                                    available_lanes = sorted(existing_lanes)
                                    pred_lane_id = Road._find_closest_lane(
                                        pred_lane_id, available_lanes
                                    )
                                else:
                                    # No lanes available, skip this link
                                    continue
                        lane.predecessor = LaneLink(id=pred_lane_id)
                        break

        # Find successor lanelets
        following_lanelets = routing_graph.following(lanelet)
        if following_lanelets:
            # Take the first successor that maps to the road link's successor road
            for next_ll in following_lanelets:
                if next_ll.id in lanelet_to_road_and_lane:
                    succ_road_id, succ_lane_id = lanelet_to_road_and_lane[next_ll.id]
                    # Only set successor if it's in a different road
                    if succ_road_id != self.id:
                        # Issue #202 fix: Check consistency with road link
                        # Lane links must only exist when corresponding road link exists
                        # This applies to both regular roads and connecting roads
                        if road_link_successor is None:
                            # Road has no successor link - skip lane link creation
                            # This prevents invalid lane connections that cause CARLA crashes
                            continue

                        # Check if road link successor is a junction
                        if road_link_successor is not None:
                            if road_link_successor.element_type == ElementType.JUNCTION:
                                # This road's successor is a junction
                                # The lane's successor should be a connecting road in that
                                # junction
                                if road_id_to_road is not None:
                                    succ_road = road_id_to_road.get(succ_road_id)
                                    if succ_road is None:
                                        if not is_connecting_road:
                                            continue
                                    # Check if successor road is a connecting road in
                                    # the junction
                                    elif (
                                        succ_road.junction
                                        != road_link_successor.element_id
                                    ):
                                        if not is_connecting_road:
                                            continue
                                # If no road_id_to_road, we can't validate - skip for safety
                                # (unless this is a connecting road)
                                elif not is_connecting_road:
                                    continue
                            else:
                                # Road link successor is a regular road
                                # Lane link must reference the same road
                                # (unless this is a connecting road)
                                if (
                                    succ_road_id != road_link_successor.element_id
                                    and not is_connecting_road
                                ):
                                    # Lane successor differs from road link - check if it's a branching scenario
                                    # Allow if successor is a connecting road (junction branching)
                                    if road_id_to_road is not None:
                                        succ_road = road_id_to_road.get(succ_road_id)
                                        if (
                                            succ_road is not None
                                            and succ_road.junction is not None
                                            and succ_road.junction >= 0
                                        ):
                                            # Successor is a connecting road - allow lane branching
                                            pass
                                        else:
                                            # Successor is a regular road but doesn't match road link - skip
                                            continue
                                    else:
                                        # Cannot validate - skip for safety
                                        continue

                        # Validate that the lane exists in the successor road
                        if road_lane_ids is not None:
                            existing_lanes = road_lane_ids.get(succ_road_id, set())
                            if succ_lane_id not in existing_lanes:
                                # Lane doesn't exist, find closest lane
                                # This handles lane reduction scenarios (e.g., 3 lanes -> 2 lanes)
                                if existing_lanes:
                                    available_lanes = sorted(existing_lanes)
                                    succ_lane_id = Road._find_closest_lane(
                                        succ_lane_id, available_lanes
                                    )
                                else:
                                    # No lanes available, skip this link
                                    continue
                        lane.successor = LaneLink(id=succ_lane_id)
                        break

    @staticmethod
    def _extract_road_types_from_lanelets(
        lanelet_list: List[lanelet2.core.Lanelet],
    ) -> Optional[List[RoadTypeDefinition]]:
        """Extract road types and speed limits from lanelets.

        Args:
            lanelet_list: List of lanelets to extract information from

        Returns:
            List of RoadTypeDefinition objects, or None if no speed limit found
        """
        if not lanelet_list:
            return None

        # Get the first lanelet to extract attributes
        first_lanelet = lanelet_list[0]

        # Extract speed limit
        speed_limit = None
        if "speed_limit" in first_lanelet.attributes:
            try:
                speed_limit_str = first_lanelet.attributes["speed_limit"]
                speed_limit = float(speed_limit_str)
            except (ValueError, TypeError):
                speed_limit = None

        # Determine road type from location and speed limit
        road_type = RoadType.UNKNOWN
        if "location" in first_lanelet.attributes:
            location = first_lanelet.attributes["location"]
            if location == "urban":
                road_type = RoadType.TOWN
            elif location == "highway":
                road_type = RoadType.MOTORWAY
            elif location == "rural":
                road_type = RoadType.RURAL
            elif location == "private":
                if speed_limit and speed_limit <= 10:
                    road_type = RoadType.LOW_SPEED
                else:
                    road_type = RoadType.TOWN
        else:
            # If no location attribute, infer from speed limit
            if speed_limit:
                if speed_limit <= 10:
                    road_type = RoadType.LOW_SPEED
                elif speed_limit <= 50:
                    road_type = RoadType.TOWN
                elif speed_limit <= 100:
                    road_type = RoadType.RURAL
                else:
                    road_type = RoadType.MOTORWAY

        # Create road type definition
        road_type_speed = None
        if speed_limit is not None:
            road_type_speed = RoadTypeSpeed(max=speed_limit, unit=SpeedUnit.KMH)

        road_type_def = RoadTypeDefinition(s=0.0, type=road_type, speed=road_type_speed)

        return [road_type_def]

    @staticmethod
    def construct_from_lanelet_groups(
        lanelet_map: lanelet2.core.LaneletMap,
        lanelet_group: LaneletInput,
        road_id: int,
        s_offset: float = 0.0,
        traffic_rule: Optional[str] = None,
        parampoly3_config: Optional[ParamPoly3Config] = None,
        arcspiral_config: Optional[ArcSpiralConfig] = None,
        width_config: Optional[WidthEstimationConfig] = None,
        routing_graph: Optional[RoutingGraph] = None,
        start_xyz_override: Optional[Tuple[float, float, float]] = None,
        end_xyz_override: Optional[Tuple[float, float, float]] = None,
    ) -> "Road":
        """Construct a Road from a group of lanelets.

        Args:
            lanelet_map: The lanelet2 map containing the lanelets
            lanelet_group: Group of lanelets to convert to a road
            s_offset: Starting s-coordinate offset for the road
            traffic_rule: Traffic rule for lanes (RHT or LHT)
            parampoly3_config: Configuration for ParamPoly3 segment generation
            arcspiral_config: Configuration for arc/spiral primitive detection
                (issue #466). Default ``None`` preserves the byte-exact
                paramPoly3-only output for backward compatibility.
            width_config: Configuration for width spline sampling
            routing_graph: Optional pre-built routing graph for lane-change detection.
                If None, creates a new one internally.
            start_xyz_override: Optional world-frame (x, y, z) coordinate that
                pins the reference-line start (s=0) exactly.  Used by the
                junction phase to eliminate gaps between connecting roads and
                their linked incoming road.
            end_xyz_override: Optional world-frame (x, y, z) coordinate that
                pins the reference-line end (s=length) exactly.  Used by the
                junction phase to align with the linked outgoing road.

        Returns:
            Road object constructed from the lanelet group

        Raises:
            ValueError: If lanelet_group is empty or contains non-adjacent lanelets
        """
        if not lanelet_group:
            raise ValueError("Lanelet group cannot be empty")

        # Convert input to list for consistent processing
        lanelet_list = to_lanelet_list(lanelet_group)

        reference_line = ReferenceLine.construct_from_lanelet_groups(
            lanelet_map,
            lanelet_list,
            traffic_rule=traffic_rule,
            start_xyz_override=start_xyz_override,
            end_xyz_override=end_xyz_override,
        )
        centerline_2d = reference_line.centerline_2d

        # Create planView geometries from 2D spline. When ``arcspiral_config``
        # is None or disabled this is bit-for-bit identical to the previous
        # ParamPoly3-only path; otherwise the classifier dispatches into
        # <line> / <arc> / <paramPoly3> primitives (issue #466).
        # ParamPoly3 only uses XY coordinates, so 2D spline is appropriate.
        geometries: List[GeometryBase] = _build_planview_geometries(
            centerline_2d,
            parampoly3_config=parampoly3_config,
            arcspiral_config=arcspiral_config,
        )

        # Create plan view with the paramPoly3 geometries
        plan_view = PlanView(geometries=geometries)

        # Calculate total road length from ParamPoly3 geometries (XY projection)
        # IMPORTANT: Use XY-plane length from geometries, not 3D spline length
        # ParamPoly3.from_spline() uses XY coordinates only, ignoring Z
        road_length = sum(geometry.length for geometry in geometries)

        def get_lanes() -> Lanes:
            """Create Lanes object from lanelet group."""
            from .lane_section import LaneSection

            lane_section = LaneSection.construct_from_lanelet_groups(
                lanelet_map,
                lanelet_list,
                s_offset=s_offset,
                traffic_rule=traffic_rule,
                width_config=width_config,
                routing_graph=routing_graph,
                start_xyz_override=start_xyz_override,
                end_xyz_override=end_xyz_override,
            )
            lanes = Lanes(lane_sections=[lane_section])
            return lanes

        # Extract geometry segment boundaries (s-coordinates)
        # This ensures elevation profile segments align with ParamPoly3 segments
        geometry_s_values = [g.s for g in geometries]

        # Get elevation profile from reference line, aligned with geometry boundaries
        elevation_profile = reference_line.get_elevation_profile(geometry_s_values)

        # Extract speed limit and road type from lanelets
        road_types_list = Road._extract_road_types_from_lanelets(lanelet_list)

        # Convert traffic_rule string to TrafficRule enum
        rule_enum = None
        if traffic_rule:
            traffic_rule_normalized = traffic_rule.upper()
            if traffic_rule_normalized == "RHT":
                rule_enum = TrafficRule.RHT
            elif traffic_rule_normalized == "LHT":
                rule_enum = TrafficRule.LHT

        # Create a basic road with the extracted information
        # Note: This is a simplified implementation
        # A complete implementation would also need to:
        # - Create proper lane sections from the lanelets
        # - Set appropriate road ID and other attributes
        # Compute the world-frame 3D endpoints that the rendered paramPoly3
        # planView + elevationProfile will actually land on.  These — not the
        # raw OSM boundary endpoints — are what downstream roads must align
        # to, because a connecting road pinned to the OSM endpoint would
        # still show a gap against the regular road's rendered endpoint if
        # the spline fit shifted it slightly.
        rendered_start_xyz = _evaluate_plan_view_world(plan_view, at_start=True)
        rendered_end_xyz = _evaluate_plan_view_world(plan_view, at_start=False)
        if elevation_profile is not None and rendered_start_xyz is not None:
            z_start = _evaluate_elevation_profile(elevation_profile, 0.0)
            rendered_start_xyz = (
                rendered_start_xyz[0],
                rendered_start_xyz[1],
                z_start,
            )
        if elevation_profile is not None and rendered_end_xyz is not None:
            z_end = _evaluate_elevation_profile(elevation_profile, road_length)
            rendered_end_xyz = (
                rendered_end_xyz[0],
                rendered_end_xyz[1],
                z_end,
            )

        from ..util import sort_adjacent_groups

        try:
            sorted_lls = sort_adjacent_groups(
                lanelet_map, set(lanelet_list), routing_graph
            )
            sorted_lanelet_ids: Optional[List[int]] = [ll.id for ll in sorted_lls]
        except ValueError:
            # Non-adjacent lanelet group — leaves lane-aware pinning
            # disabled for this road; the junction phase falls back to
            # the natural lanelet boundary for any incoming connection.
            sorted_lanelet_ids = None

        road = Road(
            id=road_id,
            name=f"Road_{road_id}",
            length=road_length,
            junction=-1,  # Not in a junction by default
            rule=rule_enum,  # Set traffic rule for the road
            plan_view=plan_view,
            elevation_profile=elevation_profile,
            lanes=get_lanes(),
            elevation_offset=reference_line.elevation_offset,
            road_types=road_types_list,
            reference_start_xyz=rendered_start_xyz,
            reference_end_xyz=rendered_end_xyz,
            sorted_lanelet_ids=sorted_lanelet_ids,
        )

        return road

    @staticmethod
    def construct_from_lanelet_map(
        lanelet_map: lanelet2.core.LaneletMap,
        traffic_rule: Optional[str] = None,
        parampoly3_config: Optional[ParamPoly3Config] = None,
        arcspiral_config: Optional[ArcSpiralConfig] = None,
        width_config: Optional[WidthEstimationConfig] = None,
    ) -> "ConstructedRoadsResult":
        """Construct Roads from a lanelet map.

        Args:
            lanelet_map: The lanelet2 map containing all lanelets
            traffic_rule: Traffic rule for lanes (RHT or LHT)
            parampoly3_config: Configuration for ParamPoly3 segment generation
            arcspiral_config: Configuration for arc/spiral primitive detection
                (issue #466). Default ``None`` preserves byte-exact output.
            width_config: Configuration for width spline sampling

        Returns:
            ConstructedRoadsResult: roads, lanelet-to-road mapping, total
                adjacent-group count, and deferred predecessor/successor
                candidate maps for divergence/merge sites (issue #291).

        Raises:
            ValueError: If lanelet_map is empty or contains no valid lanelets
        """
        if not lanelet_map or not lanelet_map.laneletLayer:
            raise ValueError("Lanelet map is empty or contains no lanelets")

        # Get all lanelets from the map
        all_lanelets = list(lanelet_map.laneletLayer)

        # Filter out lanelets inside junctions
        from ..junction import _filter_lanelets_outside_junction

        # Include non-road subtypes (highway, walkway, road_shoulder) so they can
        # be emitted with the correct OpenDRIVE lane type
        # (see Lane.construct_from_lanelet for the subtype -> LaneType mapping).
        # Different subtypes form separate adjacent groups because the routing
        # graph treats them as different participant classes, so e.g. walkways
        # will not be grouped together with road lanelets here.
        road_lanelets = _filter_lanelets_outside_junction(
            filter_lanelets_by_subtype(
                all_lanelets, ["road", "highway", "walkway", "road_shoulder"]
            )
        )

        if not road_lanelets:
            raise ValueError("No lanelets found outside junctions")

        # Create routing graph once and reuse for all operations
        traffic_rules = lanelet2.traffic_rules.create(
            lanelet2.traffic_rules.Locations.Germany,
            lanelet2.traffic_rules.Participants.Vehicle,
        )
        routing_graph = RoutingGraph(
            lanelet_map, traffic_rules, [RoutingCostDistance(0.0)]
        )

        # Find adjacent groups of lanelets
        from ..util import find_adjacent_groups

        # Pass the road_lanelets set to find_adjacent_groups so it only groups these
        adjacent_groups = find_adjacent_groups(
            lanelet_map, set(road_lanelets), routing_graph
        )

        # Create roads from adjacent groups
        roads = []
        # Store mapping from lanelet ID to road ID and adjacent group for each road
        lanelet_to_road: dict[int, int] = {}
        road_to_group: dict[int, Set[lanelet2.core.Lanelet]] = {}

        print(f"Creating roads from {len(adjacent_groups)} lanelet groups...")
        for road_id, adjacent_group in tqdm(
            enumerate(adjacent_groups),
            total=len(adjacent_groups),
            desc="Building roads",
        ):
            # ll_ids = [ll.id for ll in adjacent_group]
            # target_ids = [112, 114, 115, 116]
            # target_ids = [3013239]
            # target_ids = [3013249, 3001324]
            # target_ids = [3013259]
            # if set(ll_ids).isdisjoint(target_ids):
            #     continue
            try:
                road = Road.construct_from_lanelet_groups(
                    lanelet_map=lanelet_map,  # Use original map for proper connectivity
                    lanelet_group=adjacent_group,
                    road_id=road_id,
                    s_offset=0.0,
                    traffic_rule=traffic_rule,
                    parampoly3_config=parampoly3_config,
                    arcspiral_config=arcspiral_config,
                    width_config=width_config,
                    routing_graph=routing_graph,
                )
                roads.append(road)

                # Store lanelet ID to road ID mapping
                for lanelet in adjacent_group:
                    lanelet_to_road[lanelet.id] = road_id

                # Store road ID to adjacent group mapping
                road_to_group[road_id] = adjacent_group

            except Exception as e:
                # Log warning but continue with other groups
                if "has no len()" in str(e):
                    import traceback

                    full_trace = traceback.format_exc()
                    # Find the line that actually called len()
                    for line in full_trace.split("\n"):
                        if "len(" in line and "autoware_lanelet2_to_opendrive" in line:
                            tqdm.write(
                                f"len() error in group {road_id}: {line.strip()}"
                            )
                            break
                tqdm.write(f"Warning: Failed to create road from group {road_id}: {e}")
                continue

        if not roads:
            raise ValueError("No valid roads could be constructed from the lanelet map")

        # Build road links based on lanelet previous/following relationships
        from ..util import find_connecting_lanelet_groups, ConnectionDirection

        deferred_predecessor_candidates: Dict[int, List[int]] = {}
        deferred_successor_candidates: Dict[int, List[int]] = {}

        print(f"Building road links for {len(roads)} roads...")
        for road in tqdm(roads, desc="Building road links"):
            adjacent_group = road_to_group[road.id]

            # Predecessor
            try:
                preceding_groups = find_connecting_lanelet_groups(
                    lanelet_map,
                    adjacent_group,
                    ConnectionDirection.PREVIOUS,
                    routing_graph,
                )
                pred_candidates = _resolve_candidate_road_ids(
                    preceding_groups, lanelet_to_road
                )
                if len(pred_candidates) == 1:
                    road.add_predecessor(
                        element_id=pred_candidates[0],
                        element_type=ElementType.ROAD,
                        contact_point=ContactPoint.END,
                    )
                elif len(pred_candidates) >= 2:
                    # Defer to divergence/merge synthesis (issue #291).
                    deferred_predecessor_candidates[road.id] = pred_candidates
            except Exception as e:
                tqdm.write(
                    f"Warning: Failed to find predecessors for road {road.id}: {e}"
                )

            # Successor
            try:
                following_groups = find_connecting_lanelet_groups(
                    lanelet_map,
                    adjacent_group,
                    ConnectionDirection.FOLLOWING,
                    routing_graph,
                )
                succ_candidates = _resolve_candidate_road_ids(
                    following_groups, lanelet_to_road
                )
                if len(succ_candidates) == 1:
                    road.add_successor(
                        element_id=succ_candidates[0],
                        element_type=ElementType.ROAD,
                        contact_point=ContactPoint.START,
                    )
                elif len(succ_candidates) >= 2:
                    # Defer to divergence/merge synthesis (issue #291).
                    deferred_successor_candidates[road.id] = succ_candidates
            except Exception as e:
                tqdm.write(
                    f"Warning: Failed to find successors for road {road.id}: {e}"
                )

        # Build lane links based on lanelet previous/following relationships
        # Note: This only sets links between regular roads.
        # For complete lane links including junction roads, use set_all_lane_links()
        # after combining regular roads and connecting roads from junctions.
        Road.set_all_lane_links(lanelet_map, roads, routing_graph)

        return ConstructedRoadsResult(
            roads=roads,
            lanelet_to_road=lanelet_to_road,
            num_groups=len(adjacent_groups),
            deferred_predecessor_candidates=deferred_predecessor_candidates,
            deferred_successor_candidates=deferred_successor_candidates,
            routing_graph=routing_graph,
        )

    @staticmethod
    def construct_connecting_roads_from_junctions(
        lanelet_map: lanelet2.core.LaneletMap,
        junction_groups: List[List[lanelet2.core.Lanelet]],
        starting_road_id: int = 0,
        junction_id_offset: int = 0,
        traffic_rule: Optional[str] = None,
        parampoly3_config: Optional[ParamPoly3Config] = None,
        arcspiral_config: Optional[ArcSpiralConfig] = None,
        width_config: Optional[WidthEstimationConfig] = None,
        regular_roads: Optional[List["Road"]] = None,
        lanelet_to_road_id: Optional[Dict[int, int]] = None,
        routing_graph: Optional[RoutingGraph] = None,
    ) -> Tuple[List["Road"], Dict[int, List[int]], Dict[int, int]]:
        """Construct connecting roads from junction lanelet groups.

        Creates roads from lanelets inside junctions. These roads have their
        junction field set to the appropriate junction ID.

        When ``regular_roads`` and ``lanelet_to_road_id`` are provided, the
        junction phase pins each connecting road's reference-line endpoints
        to the world-frame endpoints of the linked incoming/outgoing regular
        roads (P0-2 junction endpoint fidelity).  Without the overrides the
        connecting road's reference line is fitted from a potentially
        different OSM LineString than its neighbours, leaving gaps of up to
        several metres at the junction boundary.

        The incoming side is always overridden when a unique incoming
        regular road can be identified.  The outgoing side is overridden
        only when exactly one outgoing regular road exists — multi-successor
        connecting roads have a single "end" that cannot be pinned to more
        than one downstream road, so we leave them asymmetric and rely on
        the incoming-side alignment to dominate.

        Args:
            lanelet_map: The lanelet2 map containing all lanelets
            junction_groups: List of junction lanelet groups from find_junction_groups()
            starting_road_id: Starting ID for road numbering (default: 0)
            junction_id_offset: Offset to add to junction IDs to avoid conflicts
                               with road IDs (default: 0). Issue #132 fix.
            traffic_rule: Traffic rule for lanes (RHT or LHT)
            parampoly3_config: Configuration for ParamPoly3 segment generation
            arcspiral_config: Configuration for arc/spiral primitive detection
                (issue #466). Default ``None`` preserves byte-exact output.
            width_config: Configuration for width spline sampling
            regular_roads: Already-built non-junction roads used to source
                world-frame endpoints for connecting-road overrides.
            lanelet_to_road_id: Mapping ``lanelet_id -> road_id`` for regular
                roads.  Used together with ``regular_roads`` to resolve which
                regular road a connecting lanelet enters / leaves.
            routing_graph: Pre-built routing graph (reused when available).

        Returns:
            Tuple of:
            - List of Road objects (connecting roads with junction field set)
            - Dict mapping junction_id -> list of road IDs in that junction
            - Dict mapping lanelet_id -> road_id for all junction lanelets
        """
        from ..util import find_adjacent_groups

        connecting_roads = []
        junction_to_roads: dict[int, List[int]] = {}
        lanelet_to_road: dict[int, int] = {}

        # Build the infrastructure for endpoint overrides.
        regular_road_by_id: Dict[int, "Road"] = {r.id: r for r in (regular_roads or [])}
        ll_to_regular_road: Dict[int, int] = lanelet_to_road_id or {}

        if regular_road_by_id and routing_graph is None:
            traffic_rules = lanelet2.traffic_rules.create(
                lanelet2.traffic_rules.Locations.Germany,
                lanelet2.traffic_rules.Participants.Vehicle,
            )
            routing_graph = RoutingGraph(
                lanelet_map, traffic_rules, [RoutingCostDistance(0.0)]
            )

        is_lht = (traffic_rule or "RHT").upper() == "LHT"

        def _lane_aware_endpoint(
            group: Set[lanelet2.core.Lanelet],
            direction: str,
        ) -> Optional[Tuple[float, float, float]]:
            """Resolve the rendered XYZ at the connecting group's anchor lane.

            Walks the routing graph from the *outermost* lanelet of the
            connecting group (leftmost for RHT, rightmost for LHT) in the
            requested direction and identifies the unique predecessor /
            successor lanelet ``pl`` that lives in a regular road ``R``.
            Returns ``R``'s rendered ``(x, y, z)`` evaluated at the anchor
            boundary of ``pl`` — i.e., the lane edge that ``pl`` shares
            with the lane immediately closer to the road's reference line.

            This is the structurally correct pin target: it equals
            ``R.reference_end_xyz`` only when ``pl`` is itself the outermost
            lanelet of ``R``; otherwise the two are offset laterally by
            one or more lane widths, which is the original wrong-pin case
            previously rejected by ``_MAX_OVERRIDE_LATERAL_M``.

            Returns ``None`` when the predecessor/successor cannot be
            resolved to a single lanelet inside a single known regular
            road (asymmetric / multi-successor / non-regular neighbour).
            """
            if not regular_road_by_id or routing_graph is None:
                return None

            from ..util import sort_adjacent_groups

            try:
                sorted_lls = sort_adjacent_groups(lanelet_map, group)
            except Exception:
                return None
            if not sorted_lls:
                return None

            outermost_ll = sorted_lls[-1] if is_lht else sorted_lls[0]
            if direction == "previous":
                neighbour_lls = list(routing_graph.previous(outermost_ll))
            else:
                neighbour_lls = list(routing_graph.following(outermost_ll))

            # Filter to lanelets that map to known regular roads.
            candidates = [
                n
                for n in neighbour_lls
                if ll_to_regular_road.get(n.id) in regular_road_by_id
            ]
            if len(candidates) != 1:
                # Asymmetric junction or chained connecting roads — leave
                # the endpoint at its natural (un-pinned) lanelet boundary.
                return None
            pl = candidates[0]
            r_id = ll_to_regular_road[pl.id]
            r = regular_road_by_id[r_id]

            if r.sorted_lanelet_ids is None or pl.id not in r.sorted_lanelet_ids:
                return None
            sorted_index = r.sorted_lanelet_ids.index(pl.id)

            # ``previous`` ⇒ pin C's start to R's end; ``following`` ⇒ pin
            # C's end to R's start.
            return r.evaluate_lane_anchor_xyz(
                sorted_index=sorted_index,
                at_start=(direction == "following"),
            )

        current_road_id = starting_road_id

        print(
            f"Creating connecting roads from {len(junction_groups)} junction groups..."
        )

        for junction_index, junction_group in tqdm(
            enumerate(junction_groups),
            total=len(junction_groups),
            desc="Building junction roads",
        ):
            # Issue #132 fix: Apply offset to junction ID
            junction_id = junction_index + junction_id_offset
            # Find adjacent groups within this junction
            adjacent_groups_in_junction = find_adjacent_groups(
                lanelet_map, set(junction_group)
            )

            junction_road_ids = []

            # Create one road per adjacent group within the junction
            for adjacent_group in adjacent_groups_in_junction:
                # Resolve endpoint overrides from linked regular roads using
                # lane-aware pinning: the pin target is the rendered lane
                # edge of the *specific* predecessor/successor lanelet of
                # the connecting group's outermost lanelet, not the regular
                # road's reference line.  See ``_lane_aware_endpoint`` and
                # #437 for the wrong-pin case this fixes.
                start_override = _lane_aware_endpoint(adjacent_group, "previous")
                end_override = _lane_aware_endpoint(adjacent_group, "following")

                try:
                    road = Road.construct_from_lanelet_groups(
                        lanelet_map=lanelet_map,
                        lanelet_group=adjacent_group,
                        road_id=current_road_id,
                        s_offset=0.0,
                        traffic_rule=traffic_rule,
                        parampoly3_config=parampoly3_config,
                        arcspiral_config=arcspiral_config,
                        width_config=width_config,
                        routing_graph=routing_graph,
                        start_xyz_override=start_override,
                        end_xyz_override=end_override,
                    )

                    # Set the junction field to mark this as a connecting road
                    road.junction = junction_id

                    connecting_roads.append(road)
                    junction_road_ids.append(current_road_id)

                    # Store lanelet ID to road ID mapping
                    for lanelet in adjacent_group:
                        lanelet_to_road[lanelet.id] = current_road_id

                    current_road_id += 1

                except Exception as e:
                    tqdm.write(
                        f"Warning: Failed to create connecting road in junction {junction_id}: {e}"
                    )
                    continue

            # Store the mapping from junction ID to its road IDs
            junction_to_roads[junction_id] = junction_road_ids

        print(
            f"Created {len(connecting_roads)} connecting roads across {len(junction_groups)} junctions"
        )

        return connecting_roads, junction_to_roads, lanelet_to_road

    @staticmethod
    def set_all_lane_links(
        lanelet_map: lanelet2.core.LaneletMap,
        roads: List["Road"],
        routing_graph: Optional[RoutingGraph] = None,
    ) -> Dict[int, Tuple[int, int]]:
        """Set lane links for all roads based on lanelet connections.

        This method builds a global mapping from lanelet IDs to (road_id, lane_id)
        and sets predecessor/successor links for all lanes in all roads.

        Args:
            lanelet_map: The Lanelet2 map containing connectivity information
            roads: List of all roads (both regular and connecting roads from junctions)
            routing_graph: Optional pre-built routing graph. If None, creates a new one.

        Returns:
            Mapping from lanelet ID to (road_id, lane_id) for all lanes.

        Example:
            >>> # After creating all roads
            >>> all_roads = regular_roads + connecting_roads
            >>> mapping = Road.set_all_lane_links(lanelet_map, all_roads)
        """
        # Build global mapping from lanelet_id to (road_id, lane_id)
        lanelet_to_road_and_lane: Dict[int, Tuple[int, int]] = {}
        for road in roads:
            lane_mapping = road.get_lanelet_to_lane_mapping()
            for lanelet_id, lane_id in lane_mapping.items():
                lanelet_to_road_and_lane[lanelet_id] = (road.id, lane_id)

        # Build mapping from road_id to set of existing lane_ids
        # This is used to validate that lane links reference existing lanes
        road_lane_ids: Dict[int, Set[int]] = {}
        for road in roads:
            lane_ids: Set[int] = set()
            if road.lanes:
                for lane_section in road.lanes.lane_sections:
                    lane_ids.update(lane_section.left_lanes.keys())
                    lane_ids.update(lane_section.right_lanes.keys())
            road_lane_ids[road.id] = lane_ids

        # Build mapping from road_id to Road object
        # This is used to check if target roads are connecting roads in junctions
        road_id_to_road: Dict[int, "Road"] = {road.id: road for road in roads}

        # Use provided routing graph or create a new one
        if routing_graph is None:
            traffic_rules = lanelet2.traffic_rules.create(
                lanelet2.traffic_rules.Locations.Germany,
                lanelet2.traffic_rules.Participants.Vehicle,
            )
            routing_graph = RoutingGraph(
                lanelet_map, traffic_rules, [RoutingCostDistance(0.0)]
            )

        # Set lane links for each road
        print(f"Building lane links for {len(roads)} roads...")
        for road in tqdm(roads, desc="Building lane links"):
            try:
                context = LaneLinksContext(
                    lanelet_map=lanelet_map,
                    lanelet_to_road_and_lane=lanelet_to_road_and_lane,
                    routing_graph=routing_graph,
                    road_lane_ids=road_lane_ids,
                    road_id_to_road=road_id_to_road,
                )
                road.set_lane_links(context)
            except Exception as e:
                tqdm.write(f"Warning: Failed to set lane links for road {road.id}: {e}")

        return lanelet_to_road_and_lane

    @staticmethod
    def _find_connected_road(
        road_lanelets: List[lanelet2.core.Lanelet],
        get_connected: object,
        own_lanelet_ids: Set[int],
        junction_lanelet_ids: Set[int],
        lanelet_to_road_id: Dict[int, int],
    ) -> Optional[int]:
        """Find a connected road, preferring non-junction roads.

        Searches through connected lanelets (previous or following) for each
        road lanelet.  Roads outside the junction are preferred; another
        connecting road within the same junction is returned as a fallback.

        Args:
            road_lanelets: Lanelet objects belonging to the current road.
            get_connected: Callable that returns connected lanelets for a
                given lanelet (e.g. ``routing_graph.previous``).
            own_lanelet_ids: Lanelet IDs of the current road (skipped to
                avoid self-links).
            junction_lanelet_ids: All lanelet IDs belonging to connecting
                roads in this junction.
            lanelet_to_road_id: Global lanelet-to-road mapping.

        Returns:
            Road ID of the connected road, or ``None`` if not found.
        """
        primary: Optional[int] = None
        fallback: Optional[int] = None
        for ll in road_lanelets:
            for connected_ll in get_connected(ll):
                if connected_ll.id in own_lanelet_ids:
                    continue
                if connected_ll.id not in lanelet_to_road_id:
                    continue
                road_id = lanelet_to_road_id[connected_ll.id]
                if connected_ll.id not in junction_lanelet_ids:
                    primary = road_id
                    break
                elif fallback is None:
                    fallback = road_id
            if primary is not None:
                break
        return primary if primary is not None else fallback

    @staticmethod
    def set_connecting_road_links(
        lanelet_map: lanelet2.core.LaneletMap,
        connecting_roads: List["Road"],
        lanelet_to_road_id: Dict[int, int],
        road_to_lanelet_ids: Dict[int, List[int]],
    ) -> None:
        """Set predecessor/successor links for connecting roads inside junctions.

        For each connecting road, finds the incoming road (predecessor) and
        outgoing road (successor) by analyzing the routing graph connections
        of the lanelets that make up the road.

        Priority: roads outside the junction are preferred. When no outside
        road is found, another connecting road within the same junction is
        used (chained connecting roads).

        Args:
            lanelet_map: The Lanelet2 map containing connectivity information
            connecting_roads: List of roads inside junctions (junction >= 0)
            lanelet_to_road_id: Mapping from lanelet ID to road ID for ALL lanelets
            road_to_lanelet_ids: Mapping from road ID to list of lanelet IDs
        """
        traffic_rules = lanelet2.traffic_rules.create(
            lanelet2.traffic_rules.Locations.Germany,
            lanelet2.traffic_rules.Participants.Vehicle,
        )
        routing_graph = RoutingGraph(
            lanelet_map, traffic_rules, [RoutingCostDistance(0.0)]
        )

        # Get junction lanelet IDs (all lanelets belonging to connecting roads)
        junction_lanelet_ids: set[int] = set()
        for road in connecting_roads:
            if road.id in road_to_lanelet_ids:
                junction_lanelet_ids.update(road_to_lanelet_ids[road.id])

        print(f"Setting road links for {len(connecting_roads)} connecting roads...")
        for road in tqdm(connecting_roads, desc="Building connecting road links"):
            if road.id not in road_to_lanelet_ids:
                continue

            road_lanelet_ids = road_to_lanelet_ids[road.id]
            if not road_lanelet_ids:
                continue

            road_lanelets = [
                lanelet_map.laneletLayer.get(lid)
                for lid in road_lanelet_ids
                if lid in lanelet_map.laneletLayer
            ]
            if not road_lanelets:
                continue

            own_lanelet_ids = set(road_lanelet_ids)

            pred_id = Road._find_connected_road(
                road_lanelets,
                routing_graph.previous,
                own_lanelet_ids,
                junction_lanelet_ids,
                lanelet_to_road_id,
            )
            if pred_id is not None:
                road.add_predecessor(
                    element_id=pred_id,
                    element_type=ElementType.ROAD,
                    contact_point=ContactPoint.END,
                )

            succ_id = Road._find_connected_road(
                road_lanelets,
                routing_graph.following,
                own_lanelet_ids,
                junction_lanelet_ids,
                lanelet_to_road_id,
            )
            if succ_id is not None:
                road.add_successor(
                    element_id=succ_id,
                    element_type=ElementType.ROAD,
                    contact_point=ContactPoint.START,
                )

    @staticmethod
    def set_incoming_road_junction_links(
        roads: List["Road"],
        junctions: List["Junction"],
    ) -> None:
        """Set junction links for incoming roads.

        For roads that connect to junctions (as incoming roads), this method
        sets the appropriate successor/predecessor link to the junction.

        This is required for CARLA compatibility and ensures correct routing
        through junctions. Issue #132 fix: Use connection.contactPoint directly
        instead of checking connecting road links, which may only reference
        one of multiple incoming roads.

        Args:
            roads: List of all roads (both regular and connecting roads)
            junctions: List of all junctions with their connections
        """
        # Build a map of road_id to road for quick lookup
        road_map: Dict[int, "Road"] = {road.id: road for road in roads}

        # For each junction, find incoming roads and set their junction links
        for junction in junctions:
            for connection in junction.connections:
                incoming_road_id = connection.incoming_road

                if incoming_road_id not in road_map:
                    continue

                incoming_road = road_map[incoming_road_id]

                # Issue #132 fix: Use contactPoint to determine which side connects
                # contactPoint indicates which end of the connecting road is used
                # - START: connecting road starts at this junction
                #   → incoming road ends at junction → set successor
                # - END: connecting road ends at this junction
                #   → incoming road starts at junction → set predecessor
                if connection.contact_point == ContactPoint.START:
                    # The end of incoming road connects to junction
                    # Set successor to junction
                    if (
                        incoming_road.link is None
                        or incoming_road.link.successor is None
                    ):
                        incoming_road.add_successor(
                            element_id=junction.id,
                            element_type=ElementType.JUNCTION,
                            contact_point=None,  # Junction links don't have contact point
                        )
                    elif incoming_road.link.successor.element_type == ElementType.ROAD:
                        # A road-to-road successor was set earlier, but this road
                        # is actually incoming to a junction. Junction link takes
                        # priority so that connecting-road lane links work correctly.
                        logger.info(
                            "Road %d: overwriting road successor %d with "
                            "junction %d (junction link takes priority)",
                            incoming_road.id,
                            incoming_road.link.successor.element_id,
                            junction.id,
                        )
                        incoming_road.add_successor(
                            element_id=junction.id,
                            element_type=ElementType.JUNCTION,
                            contact_point=None,
                        )
                elif connection.contact_point == ContactPoint.END:
                    # The start of incoming road connects to junction
                    # Set predecessor to junction
                    if (
                        incoming_road.link is None
                        or incoming_road.link.predecessor is None
                    ):
                        incoming_road.add_predecessor(
                            element_id=junction.id,
                            element_type=ElementType.JUNCTION,
                            contact_point=None,  # Junction links don't have contact point
                        )
                    elif (
                        incoming_road.link.predecessor.element_type == ElementType.ROAD
                    ):
                        # Same as above: junction link takes priority over
                        # road-to-road predecessor.
                        logger.info(
                            "Road %d: overwriting road predecessor %d with "
                            "junction %d (junction link takes priority)",
                            incoming_road.id,
                            incoming_road.link.predecessor.element_id,
                            junction.id,
                        )
                        incoming_road.add_predecessor(
                            element_id=junction.id,
                            element_type=ElementType.JUNCTION,
                            contact_point=None,
                        )
