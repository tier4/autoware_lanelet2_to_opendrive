"""Static map-quality preflight for emitted OpenDRIVE documents.

This module implements, as a standalone static analysis, the map-quality
checks that downstream map-verification tooling documents as its acceptance
criteria. It never modifies converter geometry or topology; it only parses a
serialized OpenDRIVE document, rebuilds lane-level geometry (center line,
left/right borders, lane polygons and lane links) and reports findings.

No external verification tool is executed: the result is a static preflight
against documented requirements, not a verdict from such a tool. Checks whose
exact reference implementation needs a tool-internal representation (see
``MSP_LANE_LENGTH_CONSISTENCY_PROXY``) are provided as proxies only, and the
documented ``min_connection_width`` default has no documented decision logic,
so it is reported as ``DOCUMENTATION_INSUFFICIENT`` instead of being guessed.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

DRIVING_TYPE = "driving"

CLASS_CONVERTER = "converter"
CLASS_SOURCE = "source"
CLASS_CLIP_BOUNDARY = "clip_boundary"
CLASS_PROXY_LIMITATION = "proxy_limitation"
CLASS_DOC_INSUFFICIENT = "documentation_insufficient"
CLASS_UNCLASSIFIED = "unclassified"


@dataclass(frozen=True)
class PreflightConfig:
    """Documented acceptance defaults (all configurable)."""

    sample_step: float = 3.0
    len_consistency_sample_step: float = 0.2
    min_expected_radius: float = 1.5
    max_lateral_gap_or_overlap: float = 0.30
    max_longitudinal_gap_or_overlap: float = 0.50
    min_connection_width: float = 1.50
    max_connection_gap: float = 0.25
    msp_inconsistency_len_factor: float = 0.10
    min_length_for_missing_connection_check: float = 1.0
    # Below this, a geometry's progression / integrated length / derivative
    # norm counts as "no advance at all" (degenerate geometry).
    degenerate_geometry_tolerance: float = 1e-6
    # OpenDRIVE semantics: left lanes travel against the reference (True).
    # Maps that link every road end to a successor start regardless of lane
    # side (this converter's LHT output) travel along +s on all lanes.
    left_lanes_travel_against_s: bool = True

    @property
    def jitter_angle_threshold(self) -> float:
        """Documented: abs(angle_change) must stay below step/radius."""
        return self.sample_step / self.min_expected_radius

    @property
    def logical_connection_heading_threshold(self) -> float:
        return 2.0 * self.max_connection_gap / self.min_expected_radius


@dataclass
class Finding:
    anomaly: str
    subject: str
    road_ids: Tuple[str, ...]
    value: float
    threshold: float
    detail: str
    station: Optional[float] = None
    classification: str = CLASS_UNCLASSIFIED


@dataclass
class PreflightReport:
    findings: List[Finding] = field(default_factory=list)
    jitter_diagnostics: Dict[str, int] = field(default_factory=dict)
    geometry_statistics: Dict[str, object] = field(default_factory=dict)
    notes: List[str] = field(default_factory=list)


# --------------------------------------------------------------------------
# XODR parsing and geometry reconstruction
# --------------------------------------------------------------------------


@dataclass
class _PlanGeometry:
    s: float
    x: float
    y: float
    hdg: float
    length: float
    kind: str
    curvature: float = 0.0
    curvature_end: float = 0.0
    poly: Optional[Tuple[float, ...]] = None
    p_range: str = "arcLength"


def _eval_geometry(
    geometry: _PlanGeometry, ds: float
) -> Tuple[float, float, float, float]:
    """World (x, y, heading, curvature) at distance ``ds`` into the element."""
    ds = min(max(ds, 0.0), geometry.length)
    if geometry.kind == "line":
        return (
            geometry.x + ds * math.cos(geometry.hdg),
            geometry.y + ds * math.sin(geometry.hdg),
            geometry.hdg,
            0.0,
        )
    if geometry.kind == "arc":
        k = geometry.curvature
        if abs(k) < 1e-12:
            return (
                geometry.x + ds * math.cos(geometry.hdg),
                geometry.y + ds * math.sin(geometry.hdg),
                geometry.hdg,
                0.0,
            )
        theta = geometry.hdg + k * ds
        return (
            geometry.x + (math.sin(theta) - math.sin(geometry.hdg)) / k,
            geometry.y - (math.cos(theta) - math.cos(geometry.hdg)) / k,
            theta,
            k,
        )
    if geometry.kind == "spiral":
        # Clothoid via small-step numeric integration.
        steps = max(2, int(math.ceil(ds / 0.05)))
        k0 = geometry.curvature
        k1 = geometry.curvature_end
        rate = (k1 - k0) / geometry.length if geometry.length > 0 else 0.0
        x, y, hdg = geometry.x, geometry.y, geometry.hdg
        step = ds / steps
        for i in range(steps):
            k_here = k0 + rate * (i + 0.5) * step
            hdg_mid = hdg + 0.5 * k_here * step
            x += step * math.cos(hdg_mid)
            y += step * math.sin(hdg_mid)
            hdg += k_here * step
        return x, y, hdg, k0 + rate * ds
    if geometry.kind == "paramPoly3":
        assert geometry.poly is not None
        a_u, b_u, c_u, d_u, a_v, b_v, c_v, d_v = geometry.poly
        p = ds
        if geometry.p_range == "normalized" and geometry.length > 0:
            p = ds / geometry.length
        u = a_u + b_u * p + c_u * p * p + d_u * p**3
        v = a_v + b_v * p + c_v * p * p + d_v * p**3
        du = b_u + 2 * c_u * p + 3 * d_u * p * p
        dv = b_v + 2 * c_v * p + 3 * d_v * p * p
        ddu = 2 * c_u + 6 * d_u * p
        ddv = 2 * c_v + 6 * d_v * p
        speed_sq = du * du + dv * dv
        curvature = (du * ddv - dv * ddu) / speed_sq**1.5 if speed_sq > 1e-18 else 0.0
        ch, sh = math.cos(geometry.hdg), math.sin(geometry.hdg)
        return (
            geometry.x + u * ch - v * sh,
            geometry.y + u * sh + v * ch,
            geometry.hdg + math.atan2(dv, du),
            curvature,
        )
    raise ValueError(f"unsupported planView geometry kind: {geometry.kind}")


def check_degenerate_parampoly3(
    roads: Dict[str, RoadModel],
    config: Optional["PreflightConfig"] = None,
) -> List[Finding]:
    """Report ParamPoly3 elements that do not advance along their length.

    A ``<paramPoly3>`` whose declared length is positive but whose curve
    never moves is degenerate geometry: consumers integrating the curve get
    a zero-length reference for a non-zero station span and reject the map.
    Short stubs are NOT exempt — a valid 1 cm stub is ``u(p)=p, v(p)=0``
    under ``pRange="arcLength"``, which advances exactly 1 cm.

    Flags any of: all coefficients zero; start-to-end progression
    effectively zero; integrated arc length effectively zero against a
    positive declared length; derivative norm effectively zero across the
    whole interval.
    """
    config = config or PreflightConfig()
    tolerance = config.degenerate_geometry_tolerance
    findings: List[Finding] = []
    for road in roads.values():
        for index, geometry in enumerate(road.geometries):
            if geometry.kind != "paramPoly3" or geometry.poly is None:
                continue
            if geometry.length <= tolerance:
                continue
            all_zero = all(abs(value) <= tolerance for value in geometry.poly)
            samples = max(8, int(math.ceil(geometry.length / 0.05)))
            points = [
                _eval_geometry(geometry, geometry.length * step / samples)
                for step in range(samples + 1)
            ]
            integrated = sum(
                math.hypot(
                    points[step + 1][0] - points[step][0],
                    points[step + 1][1] - points[step][1],
                )
                for step in range(samples)
            )
            progression = math.hypot(
                points[-1][0] - points[0][0],
                points[-1][1] - points[0][1],
            )
            derivative_dead = all(
                math.hypot(
                    points[step + 1][0] - points[step][0],
                    points[step + 1][1] - points[step][1],
                )
                <= tolerance
                for step in range(samples)
            )
            reasons = []
            if all_zero:
                reasons.append("all coefficients zero")
            if progression <= tolerance:
                reasons.append(f"start-to-end progression {progression:.2e} m")
            if integrated <= tolerance:
                reasons.append(f"integrated arc length {integrated:.2e} m")
            if derivative_dead:
                reasons.append("derivative norm zero across the interval")
            if not reasons:
                continue
            findings.append(
                Finding(
                    anomaly="DEGENERATE_PARAMPOLY3",
                    subject=f"road {road.road_id} geometry {index}",
                    road_ids=(road.road_id,),
                    value=max(progression, integrated),
                    threshold=tolerance,
                    station=geometry.s,
                    detail=(
                        f"declared length {geometry.length:.4f} m but "
                        + "; ".join(reasons)
                    ),
                    classification=CLASS_CONVERTER,
                )
            )
    return findings


def _poly_at(
    records: Sequence[Tuple[float, Tuple[float, float, float, float]]], s: float
) -> float:
    value = 0.0
    for offset, (a, b, c, d) in records:
        if offset <= s + 1e-9:
            ds = s - offset
            value = a + b * ds + c * ds * ds + d * ds**3
    return value


def _poly_derivative_at(
    records: Sequence[Tuple[float, Tuple[float, float, float, float]]],
    s: float,
) -> float:
    value = 0.0
    for offset, (_a, b, c, d) in records:
        if offset <= s + 1e-9:
            ds = s - offset
            value = b + 2 * c * ds + 3 * d * ds * ds
    return value


@dataclass
class _Lane:
    lane_id: int
    lane_type: str
    widths: List[Tuple[float, Tuple[float, float, float, float]]]
    predecessor: Optional[int]
    successor: Optional[int]


@dataclass
class _Section:
    s: float
    end_s: float
    left: List[_Lane]
    right: List[_Lane]

    def lane(self, lane_id: int) -> Optional[_Lane]:
        for lane in self.left + self.right:
            if lane.lane_id == lane_id:
                return lane
        return None


@dataclass
class _Link:
    element_type: str
    element_id: str
    contact_point: str


@dataclass
class RoadModel:
    road_id: str
    length: float
    junction: str
    geometries: List[_PlanGeometry]
    elevations: List[Tuple[float, Tuple[float, float, float, float]]]
    lane_offsets: List[Tuple[float, Tuple[float, float, float, float]]]
    sections: List[_Section]
    predecessor: Optional[_Link]
    successor: Optional[_Link]

    def reference_at(self, s: float) -> Tuple[float, float, float, float]:
        s = min(max(s, 0.0), self.length)
        chosen = self.geometries[0]
        for geometry in self.geometries:
            if geometry.s <= s + 1e-9:
                chosen = geometry
        return _eval_geometry(chosen, s - chosen.s)

    def elevation_at(self, s: float) -> float:
        return _poly_at(self.elevations, s)

    def section_index_at(self, s: float) -> int:
        index = 0
        for i, section in enumerate(self.sections):
            if section.s <= s + 1e-9:
                index = i
        return index

    def lane_borders_at(
        self,
        s: float,
        section_index: int,
        lane_id: int,
    ) -> Tuple[float, float]:
        """(inner_t, outer_t) lateral offsets of the lane at station ``s``."""
        section = self.sections[section_index]
        t = _poly_at(self.lane_offsets, s)
        side = section.left if lane_id > 0 else section.right
        sign = 1.0 if lane_id > 0 else -1.0
        ordered = sorted(side, key=lambda lane: abs(lane.lane_id))
        inner = t
        for lane in ordered:
            width = _poly_at(lane.widths, s - section.s)
            outer = inner + sign * width
            if lane.lane_id == lane_id:
                return inner, outer
            inner = outer
        raise KeyError(f"lane {lane_id} not in road {self.road_id} section")

    def lane_point(
        self,
        s: float,
        section_index: int,
        lane_id: int,
        lateral: str,
    ) -> Tuple[float, float, float]:
        x, y, hdg, _k = self.reference_at(s)
        inner, outer = self.lane_borders_at(s, section_index, lane_id)
        if lateral == "inner":
            t = inner
        elif lateral == "outer":
            t = outer
        else:
            t = 0.5 * (inner + outer)
        nx, ny = -math.sin(hdg), math.cos(hdg)
        return (x + t * nx, y + t * ny, self.elevation_at(s))


def parse_roads(root: ET.Element) -> Dict[str, RoadModel]:
    roads: Dict[str, RoadModel] = {}
    for road in root.findall("road"):
        geometries: List[_PlanGeometry] = []
        for g in road.findall("planView/geometry"):
            child = list(g)[0]
            kind = child.tag
            geometry = _PlanGeometry(
                s=float(g.get("s", "0")),
                x=float(g.get("x", "0")),
                y=float(g.get("y", "0")),
                hdg=float(g.get("hdg", "0")),
                length=float(g.get("length", "0")),
                kind=kind,
            )
            if kind == "arc":
                geometry.curvature = float(child.get("curvature", "0"))
            elif kind == "spiral":
                geometry.curvature = float(child.get("curvStart", "0"))
                geometry.curvature_end = float(child.get("curvEnd", "0"))
            elif kind == "paramPoly3":
                geometry.poly = tuple(
                    float(child.get(key, "0"))
                    for key in (
                        "aU",
                        "bU",
                        "cU",
                        "dU",
                        "aV",
                        "bV",
                        "cV",
                        "dV",
                    )
                )
                geometry.p_range = (
                    "normalized"
                    if child.get("pRange", "arcLength") == "normalized"
                    else "arcLength"
                )
            geometries.append(geometry)

        def _quad(element: ET.Element) -> Tuple[float, float, float, float]:
            return (
                float(element.get("a", "0")),
                float(element.get("b", "0")),
                float(element.get("c", "0")),
                float(element.get("d", "0")),
            )

        elevations = [
            (float(e.get("s", "0")), _quad(e))
            for e in road.findall("elevationProfile/elevation")
        ]
        lane_offsets = [
            (float(e.get("s", "0")), _quad(e)) for e in road.findall("lanes/laneOffset")
        ]
        length = float(road.get("length", "0"))
        raw_sections = road.findall("lanes/laneSection")
        sections: List[_Section] = []
        for index, section in enumerate(raw_sections):
            s0 = float(section.get("s", "0"))
            s1 = (
                float(raw_sections[index + 1].get("s", "0"))
                if index + 1 < len(raw_sections)
                else length
            )

            def lanes_of(side: str) -> List[_Lane]:
                result = []
                for ln in section.findall(f"{side}/lane"):
                    widths = [
                        (float(w.get("sOffset", "0")), _quad(w))
                        for w in ln.findall("width")
                    ]
                    pred = ln.find("link/predecessor")
                    succ = ln.find("link/successor")
                    result.append(
                        _Lane(
                            lane_id=int(ln.get("id", "0")),
                            lane_type=ln.get("type", "none"),
                            widths=widths,
                            predecessor=(
                                int(pred.get("id")) if pred is not None else None
                            ),
                            successor=(
                                int(succ.get("id")) if succ is not None else None
                            ),
                        )
                    )
                return result

            sections.append(
                _Section(s=s0, end_s=s1, left=lanes_of("left"), right=lanes_of("right"))
            )

        def link_of(tag: str) -> Optional[_Link]:
            el = road.find(f"link/{tag}")
            if el is None:
                return None
            return _Link(
                element_type=el.get("elementType", ""),
                element_id=el.get("elementId", ""),
                contact_point=el.get("contactPoint", ""),
            )

        roads[road.get("id", "")] = RoadModel(
            road_id=road.get("id", ""),
            length=length,
            junction=road.get("junction", "-1"),
            geometries=geometries,
            elevations=elevations,
            lane_offsets=lane_offsets,
            sections=sections,
            predecessor=link_of("predecessor"),
            successor=link_of("successor"),
        )
    return roads


# --------------------------------------------------------------------------
# Lane registry and logical connections
# --------------------------------------------------------------------------

LaneKey = Tuple[str, int, int]  # (road_id, section_index, lane_id)


@dataclass
class LaneRef:
    key: LaneKey
    lane_type: str
    section_start: float
    section_end: float

    @property
    def length(self) -> float:
        return self.section_end - self.section_start


def lane_registry(roads: Dict[str, RoadModel]) -> Dict[LaneKey, LaneRef]:
    registry: Dict[LaneKey, LaneRef] = {}
    for road in roads.values():
        for index, section in enumerate(road.sections):
            for lane in section.left + section.right:
                key = (road.road_id, index, lane.lane_id)
                registry[key] = LaneRef(
                    key=key,
                    lane_type=lane.lane_type,
                    section_start=section.s,
                    section_end=section.end_s,
                )
    return registry


@dataclass
class LaneConnection:
    """Directed longitudinal lane connection at declared contacts."""

    from_key: LaneKey
    from_contact_s: float
    to_key: LaneKey
    to_contact_s: float
    via: str  # "internal" | "road_link" | "junction"


def _lane_travel_endpoints(
    ref: LaneRef,
    config: Optional["PreflightConfig"] = None,
) -> Tuple[float, float]:
    """(travel_start_s, travel_end_s) for the lane's driving direction."""
    _road, _section, lane_id = ref.key
    against = config.left_lanes_travel_against_s if config is not None else True
    if lane_id > 0 and against:
        return ref.section_end, ref.section_start
    return ref.section_start, ref.section_end


def lane_connections(
    root: ET.Element,
    roads: Dict[str, RoadModel],
) -> List[LaneConnection]:
    connections: List[LaneConnection] = []
    for road in roads.values():
        # Internal section-to-section links.
        for index in range(len(road.sections) - 1):
            section = road.sections[index]
            nxt = road.sections[index + 1]
            for lane in section.left + section.right:
                if lane.successor is None or nxt.lane(lane.successor) is None:
                    continue
                connections.append(
                    LaneConnection(
                        from_key=(road.road_id, index, lane.lane_id),
                        from_contact_s=section.end_s,
                        to_key=(road.road_id, index + 1, lane.successor),
                        to_contact_s=nxt.s,
                        via="internal",
                    )
                )
        # Road-level successor links (road to road).
        successor = road.successor
        if (
            successor is not None
            and successor.element_type == "road"
            and successor.element_id in roads
        ):
            target = roads[successor.element_id]
            at_start = successor.contact_point == "start"
            target_section = 0 if at_start else len(target.sections) - 1
            target_s = 0.0 if at_start else target.length
            last = len(road.sections) - 1
            for lane in road.sections[last].left + road.sections[last].right:
                if lane.successor is None:
                    continue
                if target.sections[target_section].lane(lane.successor) is None:
                    continue
                connections.append(
                    LaneConnection(
                        from_key=(road.road_id, last, lane.lane_id),
                        from_contact_s=road.length,
                        to_key=(
                            target.road_id,
                            target_section,
                            lane.successor,
                        ),
                        to_contact_s=target_s,
                        via="road_link",
                    )
                )
    # Junction laneLinks.
    for junction in root.findall("junction"):
        for connection in junction.findall("connection"):
            incoming_id = connection.get("incomingRoad", "")
            connecting_id = connection.get("connectingRoad", "")
            incoming = roads.get(incoming_id)
            connecting = roads.get(connecting_id)
            if incoming is None or connecting is None:
                continue
            contact = connection.get("contactPoint", "start")
            junction_id = junction.get("id", "")
            incoming_at_end = (
                incoming.successor is not None
                and incoming.successor.element_type == "junction"
                and incoming.successor.element_id == junction_id
            )
            incoming_s = incoming.length if incoming_at_end else 0.0
            incoming_section = len(incoming.sections) - 1 if incoming_at_end else 0
            connecting_at_start = contact == "start"
            connecting_s = 0.0 if connecting_at_start else connecting.length
            connecting_section = (
                0 if connecting_at_start else len(connecting.sections) - 1
            )
            for lane_link in connection.findall("laneLink"):
                from_lane = int(lane_link.get("from", "0"))
                to_lane = int(lane_link.get("to", "0"))
                if incoming.sections[incoming_section].lane(from_lane) is None:
                    continue
                if connecting.sections[connecting_section].lane(to_lane) is None:
                    continue
                connections.append(
                    LaneConnection(
                        from_key=(incoming_id, incoming_section, from_lane),
                        from_contact_s=incoming_s,
                        to_key=(connecting_id, connecting_section, to_lane),
                        to_contact_s=connecting_s,
                        via="junction",
                    )
                )
    return connections


# --------------------------------------------------------------------------
# Checks
# --------------------------------------------------------------------------


def _angdiff(a: float, b: float) -> float:
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def _folded_angdiff(a: float, b: float) -> float:
    """Tangent difference tolerant of reversed lane travel orientation."""
    raw = _angdiff(a, b)
    return min(raw, abs(math.pi - raw))


def _lane_stations(ref: LaneRef, step: float) -> np.ndarray:
    count = max(2, int(math.ceil(ref.length / step)) + 1)
    return np.linspace(ref.section_start, ref.section_end, count)


def _driving_lanes(registry: Dict[LaneKey, LaneRef]) -> List[LaneRef]:
    return [ref for ref in registry.values() if ref.lane_type == DRIVING_TYPE]


def check_lane_ref_line_jitter(
    roads: Dict[str, RoadModel],
    registry: Dict[LaneKey, LaneRef],
    config: PreflightConfig,
) -> Tuple[List[Finding], Dict[str, int]]:
    findings: List[Finding] = []
    tiers = {
        "angle_gt_0.1deg": 0,
        "angle_gt_0.5deg": 0,
        "angle_gt_1.0deg": 0,
        "angle_gt_documented": 0,
        "samples": 0,
    }
    threshold = config.jitter_angle_threshold
    for ref in _driving_lanes(registry):
        road_id, section_index, lane_id = ref.key
        road = roads[road_id]
        stations = _lane_stations(ref, config.sample_step)
        if len(stations) < 3:
            continue
        points = np.array(
            [
                road.lane_point(float(s), section_index, lane_id, "center")[:2]
                for s in stations
            ]
        )
        segments = np.diff(points, axis=0)
        headings = np.arctan2(segments[:, 1], segments[:, 0])
        for i in range(len(headings) - 1):
            change = _angdiff(float(headings[i + 1]), float(headings[i]))
            tiers["samples"] += 1
            if change > math.radians(0.1):
                tiers["angle_gt_0.1deg"] += 1
            if change > math.radians(0.5):
                tiers["angle_gt_0.5deg"] += 1
            if change > math.radians(1.0):
                tiers["angle_gt_1.0deg"] += 1
            if change >= threshold:
                tiers["angle_gt_documented"] += 1
                findings.append(
                    Finding(
                        anomaly="LANE_REF_LINE_JITTER",
                        subject=f"road {road_id} lane {lane_id}",
                        road_ids=(road_id,),
                        value=change,
                        threshold=threshold,
                        station=float(stations[i + 1]),
                        detail=(
                            f"centerline angle change {math.degrees(change):.2f} deg"
                            f" at {config.sample_step} m sampling"
                        ),
                    )
                )
    return findings, tiers


def check_lane_connections_geometry(
    roads: Dict[str, RoadModel],
    registry: Dict[LaneKey, LaneRef],
    connections: Sequence[LaneConnection],
    config: PreflightConfig,
) -> List[Finding]:
    """Sections 4 (connection geometry) with the documented priority order."""
    findings: List[Finding] = []
    threshold_angle = config.jitter_angle_threshold
    for connection in connections:
        from_ref = registry[connection.from_key]
        to_ref = registry[connection.to_key]
        if from_ref.lane_type != DRIVING_TYPE or to_ref.lane_type != DRIVING_TYPE:
            continue
        from_road = roads[connection.from_key[0]]
        to_road = roads[connection.to_key[0]]
        p_from = from_road.lane_point(
            connection.from_contact_s,
            connection.from_key[1],
            connection.from_key[2],
            "center",
        )
        p_to = to_road.lane_point(
            connection.to_contact_s,
            connection.to_key[1],
            connection.to_key[2],
            "center",
        )
        gap_xy = math.hypot(p_from[0] - p_to[0], p_from[1] - p_to[1])
        gap_z = abs(p_from[2] - p_to[2])
        gap = max(gap_xy, gap_z)
        subject = (
            f"{connection.from_key[0]}.{connection.from_key[2]}"
            f" -> {connection.to_key[0]}.{connection.to_key[2]} ({connection.via})"
        )
        road_ids = (connection.from_key[0], connection.to_key[0])
        if gap > config.max_longitudinal_gap_or_overlap:
            findings.append(
                Finding(
                    anomaly="LANES_NOT_CONNECTED_GEOMETRICALLY",
                    subject=subject,
                    road_ids=road_ids,
                    value=gap,
                    threshold=config.max_longitudinal_gap_or_overlap,
                    detail=f"center endpoint gap xy={gap_xy:.3f} z={gap_z:.3f} m",
                )
            )
            continue
        if gap > config.max_connection_gap:
            findings.append(
                Finding(
                    anomaly="LANE_CONNECTION_REF_LINE_JITTER",
                    subject=subject,
                    road_ids=road_ids,
                    value=gap,
                    threshold=config.max_connection_gap,
                    detail=(
                        "gap failure: center endpoint gap"
                        f" xy={gap_xy:.3f} z={gap_z:.3f} m"
                    ),
                )
            )
            continue
        # Angle across the connection: 3 m before, at, 3 m after.
        step = config.sample_step
        before_s = connection.from_contact_s + (
            -step if connection.from_contact_s > from_ref.section_start else step
        )
        before_s = min(max(before_s, from_ref.section_start), from_ref.section_end)
        after_s = connection.to_contact_s + (
            step if connection.to_contact_s < to_ref.section_end else -step
        )
        after_s = min(max(after_s, to_ref.section_start), to_ref.section_end)
        p_before = from_road.lane_point(
            before_s, connection.from_key[1], connection.from_key[2], "center"
        )
        p_after = to_road.lane_point(
            after_s, connection.to_key[1], connection.to_key[2], "center"
        )
        heading_in = math.atan2(p_from[1] - p_before[1], p_from[0] - p_before[0])
        heading_out = math.atan2(p_after[1] - p_to[1], p_after[0] - p_to[0])
        change = _folded_angdiff(heading_out, heading_in)
        if change >= threshold_angle:
            findings.append(
                Finding(
                    anomaly="LANE_CONNECTION_REF_LINE_JITTER",
                    subject=subject,
                    road_ids=road_ids,
                    value=change,
                    threshold=threshold_angle,
                    detail=(
                        "angle failure: connection angle change"
                        f" {math.degrees(change):.2f} deg"
                    ),
                )
            )
    return findings


def check_border_connection_jitter(
    roads: Dict[str, RoadModel],
    registry: Dict[LaneKey, LaneRef],
    connections: Sequence[LaneConnection],
    config: PreflightConfig,
) -> List[Finding]:
    findings: List[Finding] = []
    for connection in connections:
        from_ref = registry[connection.from_key]
        to_ref = registry[connection.to_key]
        if from_ref.lane_type != DRIVING_TYPE or to_ref.lane_type != DRIVING_TYPE:
            continue
        from_road = roads[connection.from_key[0]]
        to_road = roads[connection.to_key[0]]
        # Center gap gate: border jitter is only meaningful on connections
        # that are geometrically connected at all.
        worst = 0.0
        worst_detail = ""
        for lateral in ("inner", "outer"):
            p_from = from_road.lane_point(
                connection.from_contact_s,
                connection.from_key[1],
                connection.from_key[2],
                lateral,
            )
            candidates = []
            for to_lateral in ("inner", "outer"):
                p_to = to_road.lane_point(
                    connection.to_contact_s,
                    connection.to_key[1],
                    connection.to_key[2],
                    to_lateral,
                )
                candidates.append(
                    max(
                        math.hypot(p_from[0] - p_to[0], p_from[1] - p_to[1]),
                        abs(p_from[2] - p_to[2]),
                    )
                )
            gap = min(candidates)
            if gap > worst:
                worst = gap
                worst_detail = f"{lateral} border endpoint gap {gap:.3f} m"
        if worst > config.max_connection_gap:
            findings.append(
                Finding(
                    anomaly="LANE_CONNECTION_BORDER_LINE_JITTER",
                    subject=(
                        f"{connection.from_key[0]}.{connection.from_key[2]}"
                        f" -> {connection.to_key[0]}.{connection.to_key[2]}"
                    ),
                    road_ids=(connection.from_key[0], connection.to_key[0]),
                    value=worst,
                    threshold=config.max_connection_gap,
                    detail=worst_detail,
                )
            )
    return findings


def neighbor_border_deviation(
    inner_lane_outer_border: np.ndarray,
    outer_lane_inner_border: np.ndarray,
) -> float:
    """Max separation between two borders that should coincide."""
    if len(inner_lane_outer_border) != len(outer_lane_inner_border):
        raise ValueError("border sample counts differ")
    return float(
        np.max(
            np.linalg.norm(
                np.asarray(inner_lane_outer_border)[:, :2]
                - np.asarray(outer_lane_inner_border)[:, :2],
                axis=1,
            )
        )
    )


def check_neighbor_lanes(
    roads: Dict[str, RoadModel],
    config: PreflightConfig,
) -> List[Finding]:
    findings: List[Finding] = []
    for road in roads.values():
        for index, section in enumerate(road.sections):
            for side in (section.left, section.right):
                driving = sorted(
                    [lane for lane in side if lane.lane_type == DRIVING_TYPE],
                    key=lambda lane: abs(lane.lane_id),
                )
                for inner_lane, outer_lane in zip(driving, driving[1:]):
                    if abs(outer_lane.lane_id) - abs(inner_lane.lane_id) != 1:
                        continue
                    stations = np.arange(
                        section.s,
                        section.end_s + 1e-9,
                        config.sample_step,
                    )
                    if len(stations) < 2:
                        stations = np.array([section.s, section.end_s])
                    shared_a = np.array(
                        [
                            road.lane_point(
                                float(s), index, inner_lane.lane_id, "outer"
                            )
                            for s in stations
                        ]
                    )
                    shared_b = np.array(
                        [
                            road.lane_point(
                                float(s), index, outer_lane.lane_id, "inner"
                            )
                            for s in stations
                        ]
                    )
                    deviation = neighbor_border_deviation(shared_a, shared_b)
                    if deviation > config.max_lateral_gap_or_overlap:
                        findings.append(
                            Finding(
                                anomaly="GAP_BETWEEN_NEIGHBOR_LANES",
                                subject=(
                                    f"road {road.road_id} lanes"
                                    f" {inner_lane.lane_id}/{outer_lane.lane_id}"
                                ),
                                road_ids=(road.road_id,),
                                value=deviation,
                                threshold=config.max_lateral_gap_or_overlap,
                                detail=(
                                    "shared border separation"
                                    f" {deviation:.3f} m (gap or overlap)"
                                ),
                            )
                        )
    return findings


def opposite_penetration(
    left_inner_border_t: np.ndarray,
    right_inner_border_t: np.ndarray,
) -> float:
    """Max penetration of one direction's inner border into the other.

    Both arrays are lateral offsets of the innermost opposing driving lanes'
    inner borders sampled at identical stations. The left border must stay
    at ``t >= right border`` (no crossing); penetration is how far the left
    inner border dips below the right one.
    """
    return float(
        np.max(np.asarray(right_inner_border_t) - np.asarray(left_inner_border_t))
    )


def check_opposite_roads_overlap(
    roads: Dict[str, RoadModel],
    config: PreflightConfig,
) -> List[Finding]:
    findings: List[Finding] = []
    for road in roads.values():
        for index, section in enumerate(road.sections):
            left_driving = sorted(
                [ln for ln in section.left if ln.lane_type == DRIVING_TYPE],
                key=lambda ln: ln.lane_id,
            )
            right_driving = sorted(
                [ln for ln in section.right if ln.lane_type == DRIVING_TYPE],
                key=lambda ln: -ln.lane_id,
            )
            if not left_driving or not right_driving:
                continue
            innermost_left = left_driving[0]
            innermost_right = right_driving[0]
            stations = np.arange(section.s, section.end_s + 1e-9, config.sample_step)
            if len(stations) < 2:
                stations = np.array([section.s, section.end_s])
            left_t = np.array(
                [
                    road.lane_borders_at(float(s), index, innermost_left.lane_id)[0]
                    for s in stations
                ]
            )
            right_t = np.array(
                [
                    road.lane_borders_at(float(s), index, innermost_right.lane_id)[0]
                    for s in stations
                ]
            )
            penetration = opposite_penetration(left_t, right_t)
            if penetration > config.max_lateral_gap_or_overlap:
                findings.append(
                    Finding(
                        anomaly="OVERLAP_BETWEEN_OPPOSITE_ROADS",
                        subject=(
                            f"road {road.road_id} lanes"
                            f" {innermost_left.lane_id}/{innermost_right.lane_id}"
                        ),
                        road_ids=(road.road_id,),
                        value=penetration,
                        threshold=config.max_lateral_gap_or_overlap,
                        detail=(
                            "opposite-direction inner borders cross by"
                            f" {penetration:.3f} m"
                        ),
                    )
                )
    return findings


def check_driving_non_driving_connections(
    registry: Dict[LaneKey, LaneRef],
    connections: Sequence[LaneConnection],
) -> List[Finding]:
    findings: List[Finding] = []
    for connection in connections:
        from_type = registry[connection.from_key].lane_type
        to_type = registry[connection.to_key].lane_type
        if (from_type == DRIVING_TYPE) == (to_type == DRIVING_TYPE):
            continue
        findings.append(
            Finding(
                anomaly="CONNECTION_BETWEEN_DRIVING_AND_NON_DRIVING_LANES",
                subject=(
                    f"{connection.from_key[0]}.{connection.from_key[2]}"
                    f" ({from_type}) -> {connection.to_key[0]}."
                    f"{connection.to_key[2]} ({to_type}) via {connection.via}"
                ),
                road_ids=(connection.from_key[0], connection.to_key[0]),
                value=1.0,
                threshold=0.0,
                detail="longitudinal link between driving and non-driving lane",
            )
        )
    return findings


def check_missing_logical_connections(
    roads: Dict[str, RoadModel],
    registry: Dict[LaneKey, LaneRef],
    connections: Sequence[LaneConnection],
    config: PreflightConfig,
) -> List[Finding]:
    """Section 5: geometric continuations without a logical connection."""
    findings: List[Finding] = []
    graph: Dict[LaneKey, List[LaneKey]] = {}
    for connection in connections:
        graph.setdefault(connection.from_key, []).append(connection.to_key)

    def reachable(from_key: LaneKey, to_key: LaneKey) -> bool:
        # Direct edge, or a directed path through short intermediate
        # connector lanes.
        seen = {from_key}
        frontier = [from_key]
        for _depth in range(4):
            next_frontier = []
            for key in frontier:
                for neighbour in graph.get(key, []):
                    if neighbour == to_key:
                        return True
                    if neighbour in seen:
                        continue
                    seen.add(neighbour)
                    intermediate = registry[neighbour]
                    if (
                        intermediate.length
                        <= config.min_length_for_missing_connection_check
                        or roads[neighbour[0]].junction != "-1"
                    ):
                        next_frontier.append(neighbour)
            frontier = next_frontier
            if not frontier:
                break
        return False

    candidates = [
        ref
        for ref in _driving_lanes(registry)
        if ref.length > config.min_length_for_missing_connection_check
    ]
    ends = []
    for ref in candidates:
        road = roads[ref.key[0]]
        _travel_start, travel_end = _lane_travel_endpoints(ref, config)
        point = road.lane_point(travel_end, ref.key[1], ref.key[2], "center")
        neighbour_s = travel_end + (-0.5 if travel_end > ref.section_start else 0.5)
        neighbour_s = min(max(neighbour_s, ref.section_start), ref.section_end)
        support = road.lane_point(neighbour_s, ref.key[1], ref.key[2], "center")
        heading = math.atan2(point[1] - support[1], point[0] - support[0])
        ends.append((ref, np.asarray(point[:2]), heading))
    starts = []
    for ref in candidates:
        road = roads[ref.key[0]]
        travel_start, _travel_end = _lane_travel_endpoints(ref, config)
        point = road.lane_point(travel_start, ref.key[1], ref.key[2], "center")
        neighbour_s = travel_start + (0.5 if travel_start < ref.section_end else -0.5)
        neighbour_s = min(max(neighbour_s, ref.section_start), ref.section_end)
        support = road.lane_point(neighbour_s, ref.key[1], ref.key[2], "center")
        heading = math.atan2(support[1] - point[1], support[0] - point[0])
        inner, outer = road.lane_borders_at(travel_start, ref.key[1], ref.key[2])
        starts.append((ref, np.asarray(point[:2]), heading, abs(outer - inner)))

    heading_threshold = config.logical_connection_heading_threshold
    for from_ref, end_point, end_heading in ends:
        probe = end_point + config.max_connection_gap * np.array(
            [math.cos(end_heading), math.sin(end_heading)]
        )
        for to_ref, start_point, start_heading, start_width in starts:
            if to_ref.key == from_ref.key:
                continue
            # The probe must land inside the first 0.50 m of lane B.
            along = float(
                np.dot(
                    probe - start_point,
                    [math.cos(start_heading), math.sin(start_heading)],
                )
            )
            lateral = float(
                np.dot(
                    probe - start_point,
                    [-math.sin(start_heading), math.cos(start_heading)],
                )
            )
            if not (-1e-9 <= along <= 0.50):
                continue
            if abs(lateral) > 0.5 * max(start_width, 1e-6):
                continue
            if _angdiff(end_heading, start_heading) >= heading_threshold:
                continue
            if reachable(from_ref.key, to_ref.key):
                continue
            findings.append(
                Finding(
                    anomaly="LANES_NOT_CONNECTED_LOGICALLY",
                    subject=(
                        f"{from_ref.key[0]}.{from_ref.key[2]}"
                        f" -> {to_ref.key[0]}.{to_ref.key[2]}"
                    ),
                    road_ids=(from_ref.key[0], to_ref.key[0]),
                    value=float(np.linalg.norm(end_point - start_point)),
                    threshold=config.max_connection_gap,
                    detail=(
                        "geometric continuation without a logical connection"
                        f" (heading diff"
                        f" {math.degrees(_angdiff(end_heading, start_heading)):.2f}"
                        " deg); diagnostics only, no connection added"
                    ),
                )
            )
    return findings


def check_msp_length_proxy(
    roads: Dict[str, RoadModel],
    registry: Dict[LaneKey, LaneRef],
    config: PreflightConfig,
) -> List[Finding]:
    findings: List[Finding] = []
    for ref in _driving_lanes(registry):
        if ref.length <= config.len_consistency_sample_step:
            continue
        road_id, section_index, lane_id = ref.key
        road = roads[road_id]
        stations = _lane_stations(ref, config.len_consistency_sample_step)
        points = np.array(
            [
                road.lane_point(float(s), section_index, lane_id, "center")[:2]
                for s in stations
            ]
        )
        steps = np.linalg.norm(np.diff(points, axis=0), axis=1)
        cartesian = float(np.sum(steps))
        station_span = float(stations[-1] - stations[0])
        if station_span <= 0.0:
            continue
        relative = abs(cartesian - station_span) / station_span
        if relative <= config.msp_inconsistency_len_factor:
            continue
        # Offset lanes on curved references legitimately progress at
        # (1 - t*kappa) per station metre; when that fully explains the
        # deviation this is a proxy limitation, not a converter defect.
        expected = 0.0
        for i, s in enumerate(stations[:-1]):
            ds = float(stations[i + 1] - s)
            _x, _y, _hdg, kappa = road.reference_at(float(s))
            inner, outer = road.lane_borders_at(float(s), section_index, lane_id)
            t_center = 0.5 * (inner + outer)
            expected += abs(1.0 - t_center * kappa) * ds
        explained = (
            abs(cartesian - expected) / station_span
            <= config.msp_inconsistency_len_factor
        )
        findings.append(
            Finding(
                anomaly="MSP_LANE_LENGTH_CONSISTENCY_PROXY",
                subject=f"road {road_id} lane {lane_id}",
                road_ids=(road_id,),
                value=relative,
                threshold=config.msp_inconsistency_len_factor,
                detail=(
                    f"cartesian {cartesian:.2f} m vs station span"
                    f" {station_span:.2f} m"
                    + (
                        "; fully explained by lateral offset on curvature"
                        if explained
                        else "; NOT explained by lane offset progression"
                    )
                ),
                classification=(
                    CLASS_PROXY_LIMITATION if explained else CLASS_CONVERTER
                ),
            )
        )
    return findings


# --------------------------------------------------------------------------
# Developer-log proxies and geometry statistics
# --------------------------------------------------------------------------


def developer_log_proxies(
    roads: Dict[str, RoadModel],
    registry: Dict[LaneKey, LaneRef],
    connections: Sequence[LaneConnection],
    config: PreflightConfig,
) -> Dict[str, List[Finding]]:
    proxies: Dict[str, List[Finding]] = {
        "MAP_HICCUP_1_PROXY": [],
        "MAP_HICCUP_2_PROXY": [],
        "MAP_HICCUP_3_PROXY": [],
        "MAP_HICCUP_6_PROXY": [],
        "DROPPED_INVALID_CONNECTION_PROXY": [],
        "POSSIBLE_MAP_DISCREPANCY_PROXY": [],
    }
    # 1: separation of logically connected lane endpoints.
    for connection in connections:
        from_road = roads[connection.from_key[0]]
        to_road = roads[connection.to_key[0]]
        p_from = from_road.lane_point(
            connection.from_contact_s,
            connection.from_key[1],
            connection.from_key[2],
            "center",
        )
        p_to = to_road.lane_point(
            connection.to_contact_s,
            connection.to_key[1],
            connection.to_key[2],
            "center",
        )
        gap = math.hypot(p_from[0] - p_to[0], p_from[1] - p_to[1])
        if gap > 0.01:
            proxies["MAP_HICCUP_1_PROXY"].append(
                Finding(
                    anomaly="MAP_HICCUP_1_PROXY",
                    subject=(
                        f"{connection.from_key[0]}.{connection.from_key[2]}"
                        f" -> {connection.to_key[0]}.{connection.to_key[2]}"
                    ),
                    road_ids=(connection.from_key[0], connection.to_key[0]),
                    value=gap,
                    threshold=0.01,
                    detail="connected lane endpoint separation",
                )
            )
    # 2 and 6: planView internal C0 and declared-length mismatch.
    for road in roads.values():
        total = 0.0
        for i, geometry in enumerate(road.geometries):
            total += geometry.length
            if i + 1 >= len(road.geometries):
                continue
            end = _eval_geometry(geometry, geometry.length)
            nxt = road.geometries[i + 1]
            gap = math.hypot(end[0] - nxt.x, end[1] - nxt.y)
            if gap > 0.001:
                proxies["MAP_HICCUP_2_PROXY"].append(
                    Finding(
                        anomaly="MAP_HICCUP_2_PROXY",
                        subject=f"road {road.road_id} geometry {i}->{i + 1}",
                        road_ids=(road.road_id,),
                        value=gap,
                        threshold=0.001,
                        station=nxt.s,
                        detail="internal planView C0 gap",
                    )
                )
        if abs(total - road.length) > 0.01:
            proxies["MAP_HICCUP_6_PROXY"].append(
                Finding(
                    anomaly="MAP_HICCUP_6_PROXY",
                    subject=f"road {road.road_id}",
                    road_ids=(road.road_id,),
                    value=abs(total - road.length),
                    threshold=0.01,
                    detail=(
                        f"declared {road.length:.3f} m vs geometry sum"
                        f" {total:.3f} m"
                    ),
                )
            )
    # 3: lane polyline zigzag / reversal.
    for ref in _driving_lanes(registry):
        road = roads[ref.key[0]]
        stations = _lane_stations(ref, config.len_consistency_sample_step)
        if len(stations) < 3:
            continue
        points = np.array(
            [
                road.lane_point(float(s), ref.key[1], ref.key[2], "center")[:2]
                for s in stations
            ]
        )
        steps = np.diff(points, axis=0)
        dots = np.einsum("ij,ij->i", steps[:-1], steps[1:])
        reversals = int(np.sum(dots < 0.0))
        if reversals:
            proxies["MAP_HICCUP_3_PROXY"].append(
                Finding(
                    anomaly="MAP_HICCUP_3_PROXY",
                    subject=f"road {ref.key[0]} lane {ref.key[2]}",
                    road_ids=(ref.key[0],),
                    value=float(reversals),
                    threshold=0.0,
                    detail="lane center polyline reversal / zigzag steps",
                )
            )
    # Dropped invalid connection proxy: driving/non-driving links.
    proxies["DROPPED_INVALID_CONNECTION_PROXY"] = [
        Finding(
            anomaly="DROPPED_INVALID_CONNECTION_PROXY",
            subject=finding.subject,
            road_ids=finding.road_ids,
            value=finding.value,
            threshold=finding.threshold,
            detail=finding.detail,
        )
        for finding in check_driving_non_driving_connections(registry, connections)
    ]
    # Possible map discrepancy: geometric continuation across linked roads
    # without a laneLink.
    linked_pairs = {
        (connection.from_key, connection.to_key) for connection in connections
    }
    for road in roads.values():
        successor = road.successor
        if (
            successor is None
            or successor.element_type != "road"
            or successor.element_id not in roads
        ):
            continue
        target = roads[successor.element_id]
        at_start = successor.contact_point == "start"
        target_section = 0 if at_start else len(target.sections) - 1
        target_s = 0.0 if at_start else target.length
        last = len(road.sections) - 1
        for lane in road.sections[last].left + road.sections[last].right:
            if lane.lane_type != DRIVING_TYPE:
                continue
            from_key = (road.road_id, last, lane.lane_id)
            if any(pair[0] == from_key for pair in linked_pairs):
                continue
            p_from = road.lane_point(road.length, last, lane.lane_id, "center")
            for candidate in (
                target.sections[target_section].left
                + target.sections[target_section].right
            ):
                if candidate.lane_type != DRIVING_TYPE:
                    continue
                p_to = target.lane_point(
                    target_s, target_section, candidate.lane_id, "center"
                )
                gap = math.hypot(p_from[0] - p_to[0], p_from[1] - p_to[1])
                if gap <= config.max_connection_gap:
                    proxies["POSSIBLE_MAP_DISCREPANCY_PROXY"].append(
                        Finding(
                            anomaly="POSSIBLE_MAP_DISCREPANCY_PROXY",
                            subject=(
                                f"{road.road_id}.{lane.lane_id} ->"
                                f" {target.road_id}.{candidate.lane_id}"
                            ),
                            road_ids=(road.road_id, target.road_id),
                            value=gap,
                            threshold=config.max_connection_gap,
                            detail=(
                                "lane continues geometrically but has no" " laneLink"
                            ),
                        )
                    )
                    break
    return proxies


def geometry_statistics(
    roads: Dict[str, RoadModel],
    *,
    stub_length: float = 0.5,
) -> Dict[str, object]:
    kinds: Dict[str, int] = {"line": 0, "arc": 0, "spiral": 0, "paramPoly3": 0}
    lengths: List[float] = []
    tiny_5cm = 0
    tiny_10cm = 0
    per_road_max = 0
    max_declared_gap = 0.0
    max_pp3_length_error = 0.0
    max_internal_gap = 0.0
    max_heading_jump = 0.0
    cusp_or_reversal = 0
    zero_derivative = 0
    for road in roads.values():
        per_road_max = max(per_road_max, len(road.geometries))
        total = 0.0
        is_stub = road.length <= stub_length
        for i, geometry in enumerate(road.geometries):
            kinds[geometry.kind] = kinds.get(geometry.kind, 0) + 1
            lengths.append(geometry.length)
            total += geometry.length
            if not is_stub:
                if geometry.length <= 0.05:
                    tiny_5cm += 1
                if geometry.length <= 0.10:
                    tiny_10cm += 1
            if geometry.kind == "paramPoly3":
                samples = max(8, int(math.ceil(geometry.length / 0.05)))
                previous = _eval_geometry(geometry, 0.0)
                integrated = 0.0
                previous_heading = previous[2]
                for j in range(1, samples + 1):
                    ds = geometry.length * j / samples
                    current = _eval_geometry(geometry, ds)
                    step = math.hypot(
                        current[0] - previous[0], current[1] - previous[1]
                    )
                    if step < 1e-9:
                        zero_derivative += 1
                    integrated += step
                    if _angdiff(current[2], previous_heading) > math.pi / 2:
                        cusp_or_reversal += 1
                    previous_heading = current[2]
                    previous = current
                max_pp3_length_error = max(
                    max_pp3_length_error, abs(integrated - geometry.length)
                )
            if i + 1 < len(road.geometries):
                end = _eval_geometry(geometry, geometry.length)
                nxt = road.geometries[i + 1]
                max_internal_gap = max(
                    max_internal_gap,
                    math.hypot(end[0] - nxt.x, end[1] - nxt.y),
                )
                start_heading = _eval_geometry(nxt, 0.0)[2]
                max_heading_jump = max(
                    max_heading_jump, _angdiff(end[2], start_heading)
                )
        max_declared_gap = max(max_declared_gap, abs(total - road.length))
    length_array = np.asarray(lengths) if lengths else np.zeros(1)
    return {
        "counts": kinds,
        "length_min": float(np.min(length_array)),
        "length_p50": float(np.percentile(length_array, 50)),
        "length_p95": float(np.percentile(length_array, 95)),
        "length_max": float(np.max(length_array)),
        "non_stub_leq_5cm": tiny_5cm,
        "non_stub_leq_10cm": tiny_10cm,
        "per_road_geometry_max": per_road_max,
        "max_declared_vs_sum_gap": max_declared_gap,
        "max_parampoly3_length_error": max_pp3_length_error,
        "max_internal_c0_gap": max_internal_gap,
        "max_internal_heading_jump_deg": math.degrees(max_heading_jump),
        "cusp_or_reversal_samples": cusp_or_reversal,
        "zero_derivative_samples": zero_derivative,
    }


# --------------------------------------------------------------------------
# Entry point
# --------------------------------------------------------------------------


def run_preflight(
    root: ET.Element,
    config: Optional[PreflightConfig] = None,
) -> PreflightReport:
    config = config or PreflightConfig()
    roads = parse_roads(root)
    registry = lane_registry(roads)
    connections = lane_connections(root, roads)
    report = PreflightReport()
    jitter_findings, tiers = check_lane_ref_line_jitter(roads, registry, config)
    report.findings.extend(jitter_findings)
    report.jitter_diagnostics = tiers
    report.findings.extend(
        check_lane_connections_geometry(roads, registry, connections, config)
    )
    report.findings.extend(
        check_border_connection_jitter(roads, registry, connections, config)
    )
    report.findings.extend(check_neighbor_lanes(roads, config))
    report.findings.extend(check_opposite_roads_overlap(roads, config))
    report.findings.extend(check_driving_non_driving_connections(registry, connections))
    report.findings.extend(
        check_missing_logical_connections(roads, registry, connections, config)
    )
    report.findings.extend(check_msp_length_proxy(roads, registry, config))
    report.findings.extend(check_degenerate_parampoly3(roads, config))
    for proxy_findings in developer_log_proxies(
        roads, registry, connections, config
    ).values():
        report.findings.extend(proxy_findings)
    report.geometry_statistics = geometry_statistics(roads)
    report.notes.append(
        "min_connection_width: DOCUMENTATION_INSUFFICIENT — the reference"
        " material documents the default value but no decision logic;"
        " not guessed."
    )
    report.notes.append(
        "MSP_LANE_LENGTH_CONSISTENCY exact: NOT REPRODUCIBLE without the"
        " reference tool's internal representation (proxy results only)."
    )
    report.notes.append("External map-verification tool runtime check: NOT EXECUTED.")
    return report
