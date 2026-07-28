"""Tests for topology/emission reference geometry separation."""

import math
from types import SimpleNamespace
from unittest.mock import MagicMock

import lanelet2
import numpy as np
import pytest

from autoware_lanelet2_to_opendrive.conversion_config import (
    ConversionConfig,
    EmissionGeometryConfig,
    WidthEstimationConfig,
)
from autoware_lanelet2_to_opendrive.config import DEFAULT_CONFIG
from autoware_lanelet2_to_opendrive.main import _Lanelet2ToOpenDRIVEConverter
from autoware_lanelet2_to_opendrive.opendrive.geometry import (
    Line,
    ParamPoly3,
    PlanView,
    evaluate_plan_view_world,
)
from autoware_lanelet2_to_opendrive.opendrive.enums import (
    ContactPoint,
    TrafficRule,
)
from autoware_lanelet2_to_opendrive.opendrive.lane_elements import LaneLink, LaneWidth
from autoware_lanelet2_to_opendrive.opendrive.road import Road
from autoware_lanelet2_to_opendrive.opendrive.reference_geometry import (
    EmissionReferenceGeometry,
    RoadEmissionContext,
    StationMapping,
    TopologyReferenceGeometry,
    measure_reference_domain_coverage,
    reproject_source_point_to_emission,
    _polygon_cross_section_width,
)
from autoware_lanelet2_to_opendrive.physical_connection import (
    PhysicalConnectionType,
    _constrain_width_endpoint,
    _width_value_and_derivative,
    build_divergence_physical_connection_plans,
    build_junction_incoming_physical_connection_plans,
    build_ordinary_physical_connection_plans,
    endpoint_constraints_by_road,
)
from autoware_lanelet2_to_opendrive.spline import Splines
from autoware_lanelet2_to_opendrive.divergence import (
    DivergenceSide,
    DivergenceSite,
    SanityGateInputs,
    _make_zero_length_connecting_road,
    sanity_gate_passes,
)
from autoware_lanelet2_to_opendrive.util import RoadLaneletMapping


SYNTHETIC_CASES = {
    "road53_long_straight_short_tail": np.array(
        [[0.0, 0.0], [30.0, 0.0], [35.0, 0.6], [37.8, 2.0]]
    ),
    "straight": np.array([[0.0, 0.0], [30.0, 0.0], [60.0, 0.0]]),
    "gentle_curve": np.column_stack(
        [np.linspace(0.0, 60.0, 8), 2.0 * np.sin(np.linspace(0.0, math.pi, 8))]
    ),
    "s_curve": np.column_stack(
        [
            np.linspace(0.0, 80.0, 10),
            4.0 * np.sin(np.linspace(0.0, 2.0 * math.pi, 10)),
        ]
    ),
    "sparse_curve": np.array([[0.0, 0.0], [20.0, 1.0], [35.0, 5.0], [45.0, 12.0]]),
    "duplicate_point": np.array([[0.0, 0.0], [0.0, 0.0], [20.0, 0.0], [40.0, 1.0]]),
    "near_duplicate_point": np.array(
        [[0.0, 0.0], [20.0, 0.0], [20.000001, 0.000001], [40.0, 1.0]]
    ),
    "degenerate_endpoint": np.array([[0.0, 0.0], [0.0, 0.0], [10.0, 1.0], [20.0, 3.0]]),
    "short_valid": np.array([[0.0, 0.0], [0.326, 0.02]]),
    "road50_staggered_endpoint": np.array(
        [[0.0, 0.0], [0.8, 0.15], [12.0, 0.3], [24.0, 0.4], [24.3, 1.2]]
    ),
    "self_near_geometry": np.array(
        [[0.0, 0.0], [20.0, 0.0], [20.0, 0.4], [0.0, 0.4], [0.0, 0.8], [20.0, 0.8]]
    ),
}


def _polyline_length(points: np.ndarray) -> float:
    return float(np.sum(np.linalg.norm(np.diff(points, axis=0), axis=1)))


def _sample_polyline(points: np.ndarray, spacing: float = 0.05) -> np.ndarray:
    length = _polyline_length(points)
    if length <= 0.0:
        return points[:1]
    cumulative = np.concatenate(
        ([0.0], np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1)))
    )
    stations = np.linspace(0.0, length, max(1, math.ceil(length / spacing)) + 1)
    samples = []
    for station in stations:
        idx = int(np.searchsorted(cumulative, station, side="right") - 1)
        idx = max(0, min(idx, len(points) - 2))
        seg_len = cumulative[idx + 1] - cumulative[idx]
        ratio = 0.0 if seg_len <= 0.0 else (station - cumulative[idx]) / seg_len
        samples.append(points[idx] + ratio * (points[idx + 1] - points[idx]))
    return np.asarray(samples)


def _sample_emission(
    geometry: EmissionReferenceGeometry, spacing: float = 0.05
) -> np.ndarray:
    stations = np.linspace(
        0.0,
        geometry.length,
        max(1, math.ceil(geometry.length / spacing)) + 1,
    )
    return np.asarray([geometry.evaluate(float(s)).xy for s in stations])


def _sample_topology(
    geometry: TopologyReferenceGeometry, spacing: float = 0.05
) -> np.ndarray:
    stations = np.linspace(
        0.0,
        geometry.length,
        max(1, math.ceil(geometry.length / spacing)) + 1,
    )
    return np.asarray([geometry.evaluate(float(s)).xy for s in stations])


def _symmetric_hausdorff(a: np.ndarray, b: np.ndarray) -> float:
    distances_a = np.min(np.linalg.norm(a[:, None, :] - b[None, :, :], axis=2), axis=1)
    distances_b = np.min(np.linalg.norm(b[:, None, :] - a[None, :, :], axis=2), axis=1)
    return float(max(np.max(distances_a), np.max(distances_b)))


def _make_lanelet_from_reference(
    reference_points: np.ndarray,
    *,
    width: float = 3.0,
    lanelet_id: int = 100,
) -> tuple[lanelet2.core.LaneletMap, lanelet2.core.Lanelet]:
    """Create one RHT road lanelet whose left bound is the reference line."""
    right_bound_array = []
    for idx, point in enumerate(reference_points):
        if idx == 0:
            tangent = reference_points[1, :2] - point[:2]
        elif idx == len(reference_points) - 1:
            tangent = point[:2] - reference_points[idx - 1, :2]
        else:
            tangent = reference_points[idx + 1, :2] - reference_points[idx - 1, :2]
        norm = np.linalg.norm(tangent)
        if norm <= 0.0:
            normal = np.array([0.0, 1.0])
        else:
            tangent = tangent / norm
            normal = np.array([-tangent[1], tangent[0]])
        right_bound_array.append(
            [point[0] - width * normal[0], point[1] - width * normal[1], point[2]]
        )

    left_points = [
        lanelet2.core.Point3d(lanelet2.core.getId(), x, y, z)
        for x, y, z in reference_points
    ]
    right_points = [
        lanelet2.core.Point3d(lanelet2.core.getId(), x, y, z)
        for x, y, z in right_bound_array
    ]
    left_bound = lanelet2.core.LineString3d(lanelet2.core.getId(), left_points)
    right_bound = lanelet2.core.LineString3d(lanelet2.core.getId(), right_points)
    lanelet = lanelet2.core.Lanelet(lanelet_id, left_bound, right_bound)
    lanelet.attributes["subtype"] = "road"
    lanelet.attributes["one_way"] = "yes"
    lanelet_map = lanelet2.core.LaneletMap()
    lanelet_map.add(lanelet)
    return lanelet_map, lanelet


def _make_lanelet_from_bounds(
    left_points_array: np.ndarray,
    right_points_array: np.ndarray,
    *,
    lanelet_id: int = 100,
    subtype: str = "road",
) -> tuple[lanelet2.core.LaneletMap, lanelet2.core.Lanelet]:
    left_points = [
        lanelet2.core.Point3d(lanelet2.core.getId(), float(x), float(y), float(z))
        for x, y, z in left_points_array
    ]
    right_points = [
        lanelet2.core.Point3d(lanelet2.core.getId(), float(x), float(y), float(z))
        for x, y, z in right_points_array
    ]
    left_bound = lanelet2.core.LineString3d(lanelet2.core.getId(), left_points)
    right_bound = lanelet2.core.LineString3d(lanelet2.core.getId(), right_points)
    lanelet = lanelet2.core.Lanelet(lanelet_id, left_bound, right_bound)
    lanelet.attributes["subtype"] = subtype
    lanelet.attributes["one_way"] = "yes"
    lanelet_map = lanelet2.core.LaneletMap()
    lanelet_map.add(lanelet)
    return lanelet_map, lanelet


def _make_two_lane_merge_fixture() -> (
    tuple[
        lanelet2.core.LaneletMap,
        list[lanelet2.core.Lanelet],
        list[lanelet2.core.Lanelet],
        list[lanelet2.core.Lanelet],
    ]
):
    lanelet_map = lanelet2.core.LaneletMap()

    def points(coords: list[tuple[float, float, float]]) -> list[lanelet2.core.Point3d]:
        return [
            lanelet2.core.Point3d(lanelet2.core.getId(), x, y, z) for x, y, z in coords
        ]

    source_left = lanelet2.core.LineString3d(
        lanelet2.core.getId(), points([(0.0, 0.0, 0.0), (12.0, 0.0, 0.0)])
    )
    source_middle = lanelet2.core.LineString3d(
        lanelet2.core.getId(), points([(0.0, -2.0, 0.0), (12.0, -2.0, 0.0)])
    )
    source_right = lanelet2.core.LineString3d(
        lanelet2.core.getId(), points([(0.0, -4.0, 0.0), (12.0, -4.0, 0.0)])
    )
    source_outer = lanelet2.core.Lanelet(1001, source_left, source_middle)
    source_inner = lanelet2.core.Lanelet(1002, source_middle, source_right)

    cand_a_left = lanelet2.core.LineString3d(
        lanelet2.core.getId(), points([(-10.0, 0.0, 0.0), (0.0, 0.0, 0.0)])
    )
    cand_a_right = lanelet2.core.LineString3d(
        lanelet2.core.getId(), points([(-10.0, -2.0, 0.0), (0.0, -2.0, 0.0)])
    )
    cand_b_left = lanelet2.core.LineString3d(
        lanelet2.core.getId(), points([(-10.0, -2.0, 0.0), (0.0, -2.0, 0.0)])
    )
    cand_b_right = lanelet2.core.LineString3d(
        lanelet2.core.getId(), points([(-10.0, -4.0, 0.0), (0.0, -4.0, 0.0)])
    )
    cand_a = lanelet2.core.Lanelet(2001, cand_a_left, cand_a_right)
    cand_b = lanelet2.core.Lanelet(3001, cand_b_left, cand_b_right)

    for lanelet in (source_outer, source_inner, cand_a, cand_b):
        lanelet.attributes["subtype"] = "road"
        lanelet.attributes["one_way"] = "yes"
        lanelet_map.add(lanelet)

    return lanelet_map, [source_outer, source_inner], [cand_a], [cand_b]


def _sample_road_reference(road: Road, spacing: float = 0.05) -> np.ndarray:
    assert road.plan_view is not None
    samples = []
    for geometry in road.plan_view.geometries:
        param_poly3_coeffs = None
        if isinstance(geometry, ParamPoly3):
            param_poly3_coeffs = (
                geometry.aU,
                geometry.bU,
                geometry.cU,
                geometry.dU,
                geometry.aV,
                geometry.bV,
                geometry.cV,
                geometry.dV,
            )
        count = max(1, math.ceil(geometry.length / spacing))
        for i in range(count + 1):
            p = min(geometry.length, i * geometry.length / count)
            samples.append(
                evaluate_plan_view_world(
                    geometry.x,
                    geometry.y,
                    geometry.hdg,
                    p,
                    param_poly3_coeffs,
                    getattr(geometry, "curvature", None),
                )
            )
    return np.asarray(samples, dtype=float)


def _width_at(lane, s: float) -> float:
    segment = lane.widths[0]
    for width in lane.widths:
        if width.s_offset <= s:
            segment = width
    ds = s - segment.s_offset
    return float(segment.a + segment.b * ds + segment.c * ds * ds + segment.d * ds**3)


def _pose_on_source_reference(
    reference_points: np.ndarray,
    station: float,
) -> tuple[np.ndarray, float]:
    cumulative = np.concatenate(
        (
            [0.0],
            np.cumsum(np.linalg.norm(np.diff(reference_points[:, :2], axis=0), axis=1)),
        )
    )
    station = float(np.clip(station, 0.0, cumulative[-1]))
    idx = int(np.searchsorted(cumulative, station, side="right") - 1)
    idx = max(0, min(idx, len(reference_points) - 2))
    segment = reference_points[idx + 1, :2] - reference_points[idx, :2]
    segment_length = float(np.linalg.norm(segment))
    ratio = (
        0.0 if segment_length <= 0.0 else (station - cumulative[idx]) / segment_length
    )
    point = reference_points[idx, :2] + ratio * segment
    heading = math.atan2(float(segment[1]), float(segment[0]))
    return point, heading


def _source_polygon_width_at(
    reference_points: np.ndarray,
    outer_points: np.ndarray,
    station: float,
    side_sign: float,
) -> float:
    origin, heading = _pose_on_source_reference(reference_points, station)
    side_direction = side_sign * np.array([-math.sin(heading), math.cos(heading)])
    polygon_points = np.vstack((reference_points[:, :2], outer_points[::-1, :2]))
    width = _polygon_cross_section_width(polygon_points, origin, side_direction)
    assert width is not None
    return width


def _assert_emitted_width_matches_source_polygon(
    lane,
    reference_points: np.ndarray,
    outer_points: np.ndarray,
    side_sign: float,
) -> None:
    reference_length = _polyline_length(reference_points[:, :2])
    stations = np.linspace(0.25, reference_length - 0.25, 81)
    errors = []
    for station in stations:
        expected = _source_polygon_width_at(
            reference_points,
            outer_points,
            float(station),
            side_sign,
        )
        errors.append(abs(_width_at(lane, float(station)) - expected))

    assert np.percentile(errors, 95) <= 0.05
    assert max(errors) <= 0.08


def _lane_link_id(link) -> int | None:
    return None if link is None else link.id


def _road_link_tuple(road_link) -> tuple | None:
    if road_link is None:
        return None
    predecessor = road_link.predecessor
    successor = road_link.successor
    return (
        (
            predecessor.element_type.value if predecessor is not None else None,
            predecessor.element_id if predecessor is not None else None,
            predecessor.contact_point.value
            if predecessor is not None and predecessor.contact_point is not None
            else None,
        ),
        (
            successor.element_type.value if successor is not None else None,
            successor.element_id if successor is not None else None,
            successor.contact_point.value
            if successor is not None and successor.contact_point is not None
            else None,
        ),
    )


def _topology_fingerprint(roads: list[Road]) -> tuple:
    lane_ownership = []
    lane_links = []
    road_membership = []
    for road in roads:
        road_lanelets = []
        if road.lanes is None:
            continue
        for lane_section in road.lanes.lane_sections:
            lanes = list(lane_section.left_lanes.values()) + list(
                lane_section.right_lanes.values()
            )
            for lane in lanes:
                if lane.lanelet_id is None:
                    continue
                road_lanelets.append(lane.lanelet_id)
                lane_ownership.append((lane.lanelet_id, road.id, lane.lane_id))
                lane_links.append(
                    (
                        lane.lanelet_id,
                        road.id,
                        lane.lane_id,
                        _lane_link_id(lane.predecessor),
                        _lane_link_id(lane.successor),
                    )
                )
        road_membership.append((tuple(sorted(road_lanelets)), road.id, road.junction))

    road_links = [
        (
            tuple(sorted(road.get_lanelet_to_lane_mapping().keys())),
            road.id,
            road.junction,
            _road_link_tuple(road.link),
        )
        for road in roads
    ]
    synthetic_junctions = [
        (
            road.junction,
            tuple(sorted(road.get_lanelet_to_lane_mapping().keys())),
            road.id,
        )
        for road in roads
        if road.junction is not None and road.junction >= 0
    ]
    return (
        tuple(sorted(lane_ownership)),
        tuple(sorted(lane_links)),
        tuple(sorted(road_membership)),
        tuple(sorted(road_links)),
        tuple(sorted(synthetic_junctions)),
    )


def _make_api(
    points: np.ndarray,
) -> tuple[TopologyReferenceGeometry, EmissionReferenceGeometry, StationMapping]:
    emission = EmissionReferenceGeometry.from_source_boundary(points)
    source_points = emission.source_points
    topology_spline = Splines(source_points)
    topology = TopologyReferenceGeometry(topology_spline)
    mapping = StationMapping.from_lengths(
        topology.length,
        emission.source_length,
        emission.length,
    )
    return topology, emission, mapping


@pytest.mark.parametrize("case_name,points", SYNTHETIC_CASES.items())
def test_emission_reference_geometry_synthetic_suite(
    case_name: str, points: np.ndarray
) -> None:
    topology, emission, mapping = _make_api(points)

    assert emission.validate().valid, case_name
    assert mapping.validate().valid, case_name
    assert emission.length > 0.0
    assert np.all(np.isfinite(emission.source_points))

    # Topology geometry is only wrapped; it is not replaced by emission geometry.
    for s in np.linspace(0.0, topology.length, 7):
        wrapped = topology.evaluate(float(s)).xy
        direct = topology.spline.evaluate(float(s))[:2]
        assert np.allclose(wrapped, direct)

    source_samples = _sample_polyline(emission.source_points)
    emission_samples = _sample_emission(emission)
    assert _symmetric_hausdorff(source_samples, emission_samples) <= 0.005

    topology_stations = np.linspace(0.0, topology.length, 51)
    source_stations = [mapping.topology_to_source(float(s)) for s in topology_stations]
    emission_stations = [mapping.source_to_emission(u) for u in source_stations]
    assert np.all(np.diff(source_stations) >= 0.0)
    assert np.all(np.diff(emission_stations) >= 0.0)

    assert mapping.topology_to_source(0.0) == pytest.approx(0.0, abs=1e-12)
    assert mapping.source_to_emission(0.0) == pytest.approx(0.0, abs=1e-12)
    assert mapping.topology_to_source(topology.length) == pytest.approx(
        emission.source_length, abs=1e-12
    )
    assert mapping.source_to_emission(emission.source_length) == pytest.approx(
        emission.length, abs=1e-12
    )

    for s_topology in topology_stations:
        u_source = mapping.topology_to_source(float(s_topology))
        s_emission = mapping.source_to_emission(u_source)
        u_roundtrip = mapping.emission_to_source(s_emission)
        s_roundtrip = mapping.source_to_topology(u_roundtrip)
        p_source = emission.evaluate(u_source).xy
        p_roundtrip = emission.evaluate(u_roundtrip).xy
        assert np.linalg.norm(p_source - p_roundtrip) <= 1e-9
        assert s_roundtrip == pytest.approx(float(s_topology), abs=1e-9)


def test_road53_synthetic_emission_preserves_source_fidelity() -> None:
    points = SYNTHETIC_CASES["road53_long_straight_short_tail"]
    topology, emission, _mapping = _make_api(points)

    source_samples = _sample_polyline(emission.source_points)
    topology_samples = _sample_topology(topology)
    emission_samples = _sample_emission(emission)

    assert _symmetric_hausdorff(source_samples, topology_samples) > 0.10
    assert _symmetric_hausdorff(source_samples, emission_samples) <= 0.03


def test_station_mapping_rejects_non_monotonic_breakpoints() -> None:
    with pytest.raises(ValueError, match="strictly increasing"):
        StationMapping(
            topology_stations=[0.0, 2.0, 1.0],
            source_stations=[0.0, 1.0, 2.0],
            emission_stations=[0.0, 1.0, 2.0],
        )


def test_local_projection_uses_station_hint_for_self_near_geometry() -> None:
    emission = EmissionReferenceGeometry.from_source_boundary(
        np.array([[0.0, 0.0], [20.0, 0.0], [20.0, 0.4], [0.0, 0.4]])
    )
    ambiguous_point = np.array([10.0, 0.2])

    first_branch = emission.project(
        ambiguous_point,
        preferred_s=10.0,
        search_radius=2.0,
    )
    second_branch = emission.project(
        ambiguous_point,
        preferred_s=30.0,
        search_radius=2.0,
    )

    assert first_branch.s < 15.0
    assert second_branch.s > 25.0
    assert np.allclose(
        emission.point_at_lateral_offset(first_branch.s, first_branch.t),
        ambiguous_point,
    )
    assert np.allclose(
        emission.point_at_lateral_offset(second_branch.s, second_branch.t),
        ambiguous_point,
    )


def test_source_regulatory_geometry_reprojects_to_emission() -> None:
    topology, emission, mapping = _make_api(
        SYNTHETIC_CASES["road53_long_straight_short_tail"]
    )

    signal_point = emission.point_at_lateral_offset(20.0, 2.0)
    signal = reproject_source_point_to_emission(
        signal_point,
        topology_geometry=topology,
        emission_geometry=emission,
        station_mapping=mapping,
    )
    assert np.isfinite(signal.topology.s)
    assert np.isfinite(signal.source_station_hint)
    assert (
        np.linalg.norm(
            emission.point_at_lateral_offset(signal.emission.s, signal.emission.t)
            - signal_point
        )
        <= 1e-9
    )

    stop_line_points = [
        emission.point_at_lateral_offset(25.0, -2.0),
        emission.point_at_lateral_offset(25.0, 2.0),
    ]
    for point in stop_line_points:
        projection = reproject_source_point_to_emission(
            point,
            topology_geometry=topology,
            emission_geometry=emission,
            station_mapping=mapping,
        ).emission
        reconstructed = emission.point_at_lateral_offset(projection.s, projection.t)
        assert np.linalg.norm(reconstructed - point) <= 1e-9

    crosswalk_corners = [
        emission.point_at_lateral_offset(28.0, -1.5),
        emission.point_at_lateral_offset(28.0, 1.5),
        emission.point_at_lateral_offset(31.0, 1.5),
        emission.point_at_lateral_offset(31.0, -1.5),
    ]
    for point in crosswalk_corners:
        projection = reproject_source_point_to_emission(
            point,
            topology_geometry=topology,
            emission_geometry=emission,
            station_mapping=mapping,
        ).emission
        reconstructed = emission.point_at_lateral_offset(projection.s, projection.t)
        assert np.linalg.norm(reconstructed - point) <= 1e-9


def test_road_emission_context_emits_atomic_planview_elevation_and_width() -> None:
    reference_points = np.array(
        [
            [0.0, 0.0, 2.0],
            [30.0, 0.0, 2.4],
            [35.0, 0.6, 2.6],
            [37.8, 2.0, 2.8],
        ],
        dtype=float,
    )
    lanelet_map, lanelet = _make_lanelet_from_reference(reference_points, width=3.2)
    context = RoadEmissionContext.from_lanelet_groups(
        lanelet_map,
        [lanelet],
        traffic_rule="RHT",
    )

    topology_road = Road.construct_from_lanelet_groups(
        lanelet_map,
        [lanelet],
        road_id=53,
        traffic_rule="RHT",
        width_config=WidthEstimationConfig(adaptive_sampling=True),
    )
    road = topology_road.copy_with_emission_context(
        lanelet_map=lanelet_map,
        lanelet_group=[lanelet],
        emission_context=context,
        traffic_rule="RHT",
        width_config=WidthEstimationConfig(adaptive_sampling=True),
    )

    assert road.plan_view is not None
    assert road.length == pytest.approx(context.length)
    assert (
        len(road.plan_view.geometries)
        == len(context.emission_geometry.source_points) - 1
    )
    assert all(geometry.length > 0.0 for geometry in road.plan_view.geometries)

    source_samples = _sample_polyline(context.emission_geometry.source_points)
    emitted_samples = _sample_road_reference(road)
    assert _symmetric_hausdorff(source_samples, emitted_samples) <= 0.03

    assert road.elevation_profile is not None
    assert road.get_elevation_at_s(0.0) == pytest.approx(reference_points[0, 2])
    assert road.get_elevation_at_s(road.length) == pytest.approx(
        reference_points[-1, 2]
    )

    assert road.lanes is not None
    lane = road.lanes.lane_sections[0].right_lanes[-1]
    for station in np.linspace(0.0, road.length, 9):
        assert _width_at(lane, float(station)) == pytest.approx(3.2, abs=5e-2)


def test_emission_width_uses_polygon_cross_sections_for_staggered_outer_start() -> None:
    right_reference = np.array(
        [[0.0, 0.0, 0.0], [15.0, 0.0, 0.0], [30.0, 0.0, 0.0]],
        dtype=float,
    )
    left_outer = np.array(
        [[10.0, 3.0, 0.0], [20.0, 3.0, 0.0], [30.0, 3.0, 0.0]],
        dtype=float,
    )
    lanelet_map, lanelet = _make_lanelet_from_bounds(
        left_outer,
        right_reference,
        lanelet_id=2316511,
        subtype="road_shoulder",
    )
    context = RoadEmissionContext.from_lanelet_groups(
        lanelet_map,
        [lanelet],
        traffic_rule="LHT",
    )

    topology_road = Road.construct_from_lanelet_groups(
        lanelet_map,
        [lanelet],
        road_id=50,
        traffic_rule="LHT",
        width_config=WidthEstimationConfig(adaptive_sampling=True),
    )
    road = topology_road.copy_with_emission_context(
        lanelet_map=lanelet_map,
        lanelet_group=[lanelet],
        emission_context=context,
        traffic_rule="LHT",
        width_config=WidthEstimationConfig(adaptive_sampling=True),
    )

    assert road.lanes is not None
    lane = road.lanes.lane_sections[0].left_lanes[1]
    assert _width_at(lane, 0.0) == pytest.approx(0.0, abs=1e-9)
    assert _width_at(lane, 5.0) == pytest.approx(1.5, abs=1e-9)
    assert _width_at(lane, 10.0) == pytest.approx(3.0, abs=1e-9)
    assert _width_at(lane, 20.0) == pytest.approx(3.0, abs=1e-9)


def test_emission_width_uses_endpoint_cap_for_oblique_nonzero_start() -> None:
    right_reference = np.array(
        [[0.0, 0.0, 0.0], [20.0, 0.0, 0.0]],
        dtype=float,
    )
    left_outer = np.array(
        [[0.05, 3.0, 0.0], [20.05, 3.0, 0.0]],
        dtype=float,
    )
    lanelet_map, lanelet = _make_lanelet_from_bounds(left_outer, right_reference)
    context = RoadEmissionContext.from_lanelet_groups(
        lanelet_map,
        [lanelet],
        traffic_rule="LHT",
    )

    topology_road = Road.construct_from_lanelet_groups(
        lanelet_map,
        [lanelet],
        road_id=52,
        traffic_rule="LHT",
        width_config=WidthEstimationConfig(adaptive_sampling=True),
    )
    road = topology_road.copy_with_emission_context(
        lanelet_map=lanelet_map,
        lanelet_group=[lanelet],
        emission_context=context,
        traffic_rule="LHT",
        width_config=WidthEstimationConfig(adaptive_sampling=True),
    )

    assert road.lanes is not None
    lane = road.lanes.lane_sections[0].left_lanes[1]
    assert _width_at(lane, 0.0) == pytest.approx(3.0, abs=1e-9)
    assert _width_at(lane, road.length) == pytest.approx(3.0, abs=1e-9)
    assert _width_at(lane, 0.025) == pytest.approx(3.0, abs=1e-9)


@pytest.mark.parametrize("reverse", [False, True])
def test_multilane_staggered_cap_does_not_create_post_cap_width_hole(
    reverse: bool,
) -> None:
    """A nearby cap intersection must not hide the actual outer boundary."""

    def line(points: np.ndarray) -> lanelet2.core.LineString3d:
        return lanelet2.core.LineString3d(
            lanelet2.core.getId(),
            [
                lanelet2.core.Point3d(
                    lanelet2.core.getId(),
                    float(x),
                    float(y),
                    float(z),
                )
                for x, y, z in points
            ],
        )

    boundaries = [
        np.array(
            [
                [0.0, 0.0, 0.0],
                [-2.8303, 3.8470, 0.0],
                [-5.5840, 7.5913, 0.0],
                [-7.4718, 10.1508, 0.0],
            ]
        ),
        np.array(
            [
                [-2.6849, -1.8586, 0.0],
                [-5.4609, 1.9137, 0.0],
                [-8.2120, 5.6580, 0.0],
                [-10.0973, 8.2114, 0.0],
            ]
        ),
        np.array(
            [
                [-5.3952, -3.7354, 0.0],
                [-8.2134, 0.0806, 0.0],
                [-10.9452, 3.7599, 0.0],
                [-12.7884, 6.2634, 0.0],
            ]
        ),
    ]
    if reverse:
        boundaries = [points[::-1].copy() for points in boundaries]
    lines = [line(points) for points in boundaries]
    lanelet_map = lanelet2.core.LaneletMap()
    lanelets = [
        lanelet2.core.Lanelet(
            lanelet2.core.getId(),
            lines[index + 1],
            lines[index],
        )
        for index in range(2)
    ]
    for lanelet in lanelets:
        lanelet.attributes["subtype"] = "road"
        lanelet.attributes["one_way"] = "yes"
        lanelet_map.add(lanelet)

    context = RoadEmissionContext.from_lanelet_groups(
        lanelet_map,
        lanelets,
        traffic_rule="LHT",
    )
    topology_road = Road.construct_from_lanelet_groups(
        lanelet_map,
        lanelets,
        road_id=53,
        traffic_rule="LHT",
        width_config=WidthEstimationConfig(adaptive_sampling=True),
    )
    road = topology_road.copy_with_emission_context(
        lanelet_map=lanelet_map,
        lanelet_group=lanelets,
        emission_context=context,
        traffic_rule="LHT",
        width_config=WidthEstimationConfig(adaptive_sampling=True),
    )

    assert road.lanes is not None
    outer_lane = road.lanes.lane_sections[0].left_lanes[2]
    endpoint_station = road.length if reverse else 0.0
    start_width = _width_at(outer_lane, endpoint_station)
    local_widths = [
        _width_at(outer_lane, float(station))
        for station in (
            np.linspace(max(0.0, road.length - 0.25), road.length, 101)
            if reverse
            else np.linspace(0.0, 0.25, 101)
        )
    ]
    assert start_width == pytest.approx(3.295, abs=0.02)
    assert min(local_widths) >= start_width - 0.05


def test_multilane_staggered_end_cap_with_heading_override_has_no_width_hole() -> None:
    """A junction-aligned end heading must retain the full oblique cap."""

    def line(points: np.ndarray) -> lanelet2.core.LineString3d:
        return lanelet2.core.LineString3d(
            lanelet2.core.getId(),
            [
                lanelet2.core.Point3d(
                    lanelet2.core.getId(),
                    float(x),
                    float(y),
                    float(z),
                )
                for x, y, z in points
            ],
        )

    boundaries = [
        np.array([[0.0, 0.0, 0.0], [27.549473, 0.0, 0.0]]),
        np.array(
            [
                [0.758241, 3.911892, 0.0],
                [5.162937, 3.728793, 0.0],
                [11.176772, 3.493391, 0.0],
                [15.157968, 3.410842, 0.0],
                [22.472307, 3.247531, 0.0],
                [26.468288, 3.136040, 0.0],
            ]
        ),
        np.array(
            [
                [1.092061, 7.435056, 0.0],
                [5.239077, 7.188106, 0.0],
                [11.371843, 6.780294, 0.0],
                [15.250496, 6.588049, 0.0],
                [22.475603, 6.260103, 0.0],
                [26.386426, 6.081731, 0.0],
            ]
        ),
        np.array(
            [
                [1.433749, 10.746078, 0.0],
                [5.453039, 10.498830, 0.0],
                [11.391689, 10.087912, 0.0],
                [15.554756, 9.794811, 0.0],
                [22.688332, 9.309365, 0.0],
                [26.742931, 8.929952, 0.0],
            ]
        ),
    ]
    lines = [line(points) for points in boundaries]
    lanelet_map = lanelet2.core.LaneletMap()
    lanelets = [
        lanelet2.core.Lanelet(
            lanelet2.core.getId(),
            lines[index + 1],
            lines[index],
        )
        for index in range(3)
    ]
    for lanelet in lanelets:
        lanelet.attributes["subtype"] = "road"
        lanelet.attributes["one_way"] = "yes"
        lanelet_map.add(lanelet)

    context = RoadEmissionContext.from_lanelet_groups(
        lanelet_map,
        lanelets,
        traffic_rule="LHT",
        end_heading_override=0.0900743,
    )
    topology_road = Road.construct_from_lanelet_groups(
        lanelet_map,
        lanelets,
        road_id=54,
        traffic_rule="LHT",
        width_config=WidthEstimationConfig(adaptive_sampling=True),
    )
    road = topology_road.copy_with_emission_context(
        lanelet_map=lanelet_map,
        lanelet_group=lanelets,
        emission_context=context,
        traffic_rule="LHT",
        width_config=WidthEstimationConfig(adaptive_sampling=True),
    )

    assert road.lanes is not None
    for lane in road.lanes.lane_sections[0].left_lanes.values():
        endpoint_width = _width_at(lane, road.length)
        local_widths = [
            _width_at(lane, float(station))
            for station in np.linspace(max(0.0, road.length - 1.5), road.length, 301)
        ]
        assert min(local_widths) >= 0.5 * endpoint_width


def test_emission_reference_terminal_tangent_ignores_one_sided_micro_kink() -> None:
    tiny_tail = np.array([0.008660254, 0.005, 0.0])
    right_reference = np.array(
        [[0.0, 0.0, 0.0], [20.0, 0.0, 0.0], [20.0, 0.0, 0.0] + tiny_tail],
        dtype=float,
    )
    left_outer = np.array(
        [[0.0, 3.0, 0.0], [20.0, 3.0, 0.0], [20.008660254, 3.0, 0.0]],
        dtype=float,
    )
    lanelet_map, lanelet = _make_lanelet_from_bounds(left_outer, right_reference)

    context = RoadEmissionContext.from_lanelet_groups(
        lanelet_map,
        [lanelet],
        traffic_rule="LHT",
    )

    assert context.length == pytest.approx(20.0, abs=1e-6)
    assert context.evaluate(context.length).heading == pytest.approx(0.0, abs=1e-6)
    assert len(context.to_plan_view().geometries) == 1


def test_emission_reference_aligns_multi_lane_continuation_to_shared_cross_section() -> (
    None
):
    def line(points: np.ndarray) -> lanelet2.core.LineString3d:
        return lanelet2.core.LineString3d(
            lanelet2.core.getId(),
            [
                lanelet2.core.Point3d(
                    lanelet2.core.getId(),
                    float(x),
                    float(y),
                    float(z),
                )
                for x, y, z in points
            ],
        )

    tail_y = math.tan(math.radians(10.0))
    incoming_boundaries = [
        np.array(
            [[0.0, offset, 0.0], [10.0, offset, 0.0], [19.0, offset, 0.0]]
            + [[20.0, offset + tail_y, 0.0]],
            dtype=float,
        )
        for offset in (0.0, 3.0, 6.0)
    ]
    incoming_boundaries[1][-1, 0] += 0.1
    incoming_boundaries[1][-2, 1] += tail_y
    incoming_boundaries[2][-2, 1] += tail_y
    outgoing_boundaries = [
        np.array(
            [
                [20.0, offset + tail_y, 0.0],
                [25.0, offset + tail_y, 0.0],
                [30.0, offset + tail_y, 0.0],
            ],
            dtype=float,
        )
        for offset in (0.0, 3.0, 6.0)
    ]
    outgoing_boundaries[1][0, 0] += 0.1

    incoming_lines = [line(points) for points in incoming_boundaries]
    outgoing_lines = [line(points) for points in outgoing_boundaries]
    lanelet_map = lanelet2.core.LaneletMap()
    incoming_lanelets = [
        lanelet2.core.Lanelet(
            lanelet2.core.getId(),
            incoming_lines[index + 1],
            incoming_lines[index],
        )
        for index in range(2)
    ]
    outgoing_lanelets = [
        lanelet2.core.Lanelet(
            lanelet2.core.getId(),
            outgoing_lines[index + 1],
            outgoing_lines[index],
        )
        for index in range(2)
    ]
    for lanelet in incoming_lanelets + outgoing_lanelets:
        lanelet.attributes["subtype"] = "road"
        lanelet.attributes["one_way"] = "yes"
        lanelet_map.add(lanelet)

    incoming_road = Road.construct_from_lanelet_groups(
        lanelet_map,
        incoming_lanelets,
        road_id=1,
        traffic_rule="LHT",
    )
    outgoing_road = Road.construct_from_lanelet_groups(
        lanelet_map,
        outgoing_lanelets,
        road_id=2,
        traffic_rule="LHT",
    )
    incoming_road.add_successor(
        outgoing_road.id,
        contact_point=ContactPoint.START,
    )
    outgoing_road.add_predecessor(
        incoming_road.id,
        contact_point=ContactPoint.END,
    )
    assert incoming_road.lanes is not None
    assert outgoing_road.lanes is not None
    incoming_by_lanelet = {
        lane.lanelet_id: lane
        for lane in incoming_road.lanes.lane_sections[0].left_lanes.values()
    }
    outgoing_by_lanelet = {
        lane.lanelet_id: lane
        for lane in outgoing_road.lanes.lane_sections[0].left_lanes.values()
    }
    for incoming_lanelet, outgoing_lanelet in zip(
        incoming_lanelets,
        outgoing_lanelets,
    ):
        incoming_lane = incoming_by_lanelet[incoming_lanelet.id]
        outgoing_lane = outgoing_by_lanelet[outgoing_lanelet.id]
        assert incoming_lane.lane_id is not None
        assert outgoing_lane.lane_id is not None
        incoming_lane.successor = LaneLink(id=outgoing_lane.lane_id)
        outgoing_lane.predecessor = LaneLink(id=incoming_lane.lane_id)

    converter = _Lanelet2ToOpenDRIVEConverter(
        lanelet_map,
        ConversionConfig(
            traffic_rule="LHT",
            emission_geometry=EmissionGeometryConfig(enabled=True),
        ),
    )
    plans = build_ordinary_physical_connection_plans(
        [incoming_road, outgoing_road],
        {lanelet.id: lanelet for lanelet in incoming_lanelets + outgoing_lanelets},
    )
    assert len(plans) == 1
    assert (
        plans[0].connection_type
        is PhysicalConnectionType.ORDINARY_MULTI_LANE_CONTINUATION
    )
    assert plans[0].cross_section.reference_xyz[:2] == pytest.approx(
        incoming_boundaries[0][-1, :2]
    )
    assert plans[0].cross_section.heading == pytest.approx(0.0, abs=1e-9)
    assert plans[0].cross_section.lane_widths == pytest.approx((3.0, 3.0))

    outgoing_road.junction = 100
    junction_plans = build_junction_incoming_physical_connection_plans(
        [incoming_road, outgoing_road],
        {lanelet.id: lanelet for lanelet in incoming_lanelets + outgoing_lanelets},
    )
    assert len(junction_plans) == 1
    assert junction_plans[0].connection_type is PhysicalConnectionType.JUNCTION_INCOMING
    junction_constraints = endpoint_constraints_by_road(junction_plans)
    assert set(junction_constraints[incoming_road.id]) == {"end"}
    assert set(junction_constraints[outgoing_road.id]) == {"start"}
    outgoing_road.junction = -1

    branch_road = Road.construct_from_lanelet_groups(
        lanelet_map,
        [outgoing_lanelets[1]],
        road_id=5,
        traffic_rule="LHT",
    )
    branch_road.junction = 100
    branch_road.add_predecessor(
        incoming_road.id,
        contact_point=ContactPoint.END,
    )
    assert branch_road.lanes is not None
    branch_lane = next(iter(branch_road.lanes.lane_sections[0].left_lanes.values()))
    incoming_outer_lane = incoming_by_lanelet[incoming_lanelets[1].id]
    assert incoming_outer_lane.lane_id is not None
    assert branch_lane.lane_id is not None
    incoming_outer_lane.successor = LaneLink(id=branch_lane.lane_id)
    branch_lane.predecessor = LaneLink(id=incoming_outer_lane.lane_id)
    branch_plans = build_junction_incoming_physical_connection_plans(
        [incoming_road, branch_road],
        {lanelet.id: lanelet for lanelet in incoming_lanelets + outgoing_lanelets},
    )
    assert len(branch_plans) == 1
    assert branch_plans[0].cross_section.reference_xyz[:2] == pytest.approx(
        (20.0, 3.0 + tail_y),
        abs=1e-9,
    )
    assert branch_plans[0].cross_section.reference_xyz[:2] != pytest.approx(
        outgoing_boundaries[1][0, :2],
        abs=1e-3,
    )
    outgoing_outer_lane_id = outgoing_by_lanelet[outgoing_lanelets[1].id].lane_id
    assert outgoing_outer_lane_id is not None
    incoming_outer_lane.successor = LaneLink(id=outgoing_outer_lane_id)

    single_incoming = Road.construct_from_lanelet_groups(
        lanelet_map,
        [incoming_lanelets[0]],
        road_id=3,
        traffic_rule="LHT",
    )
    single_outgoing = Road.construct_from_lanelet_groups(
        lanelet_map,
        [outgoing_lanelets[0]],
        road_id=4,
        traffic_rule="LHT",
    )
    single_incoming.add_successor(4, contact_point=ContactPoint.START)
    single_outgoing.add_predecessor(3, contact_point=ContactPoint.END)
    assert single_incoming.lanes is not None
    assert single_outgoing.lanes is not None
    single_incoming_lane = next(
        iter(single_incoming.lanes.lane_sections[0].left_lanes.values())
    )
    single_outgoing_lane = next(
        iter(single_outgoing.lanes.lane_sections[0].left_lanes.values())
    )
    assert single_incoming_lane.lane_id is not None
    assert single_outgoing_lane.lane_id is not None
    single_incoming_lane.successor = LaneLink(id=single_outgoing_lane.lane_id)
    single_outgoing_lane.predecessor = LaneLink(id=single_incoming_lane.lane_id)
    single_plans = build_ordinary_physical_connection_plans(
        [single_incoming, single_outgoing],
        {
            incoming_lanelets[0].id: incoming_lanelets[0],
            outgoing_lanelets[0].id: outgoing_lanelets[0],
        },
    )
    assert len(single_plans) == 1
    assert (
        single_plans[0].connection_type is PhysicalConnectionType.ORDINARY_CONTINUATION
    )

    emitted_incoming, emitted_outgoing = (
        converter._build_emitted_roads_after_topology_freeze(
            [incoming_road, outgoing_road],
            RoadLaneletMapping(
                road_to_lanelets={
                    incoming_road.id: [lanelet.id for lanelet in incoming_lanelets],
                    outgoing_road.id: [lanelet.id for lanelet in outgoing_lanelets],
                },
                lanelet_to_road={
                    **{lanelet.id: incoming_road.id for lanelet in incoming_lanelets},
                    **{lanelet.id: outgoing_road.id for lanelet in outgoing_lanelets},
                },
            ),
            routing_graph=None,
        )
    )
    assert emitted_incoming.emission_context is not None
    assert emitted_outgoing.emission_context is not None
    assert emitted_incoming.lanes is not None
    assert emitted_outgoing.lanes is not None
    incoming_pose = emitted_incoming.emission_context.evaluate(emitted_incoming.length)
    outgoing_pose = emitted_outgoing.emission_context.evaluate(0.0)
    normal = np.array(
        [-math.sin(incoming_pose.heading), math.cos(incoming_pose.heading)]
    )
    incoming_lanes = sorted(
        emitted_incoming.lanes.lane_sections[0].left_lanes.values(),
        key=lambda lane: lane.lane_id or 0,
    )
    outgoing_lanes = sorted(
        emitted_outgoing.lanes.lane_sections[0].left_lanes.values(),
        key=lambda lane: lane.lane_id or 0,
    )
    incoming_offsets = np.cumsum(
        [0.0] + [_width_at(lane, emitted_incoming.length) for lane in incoming_lanes]
    )
    outgoing_offsets = np.cumsum(
        [0.0] + [_width_at(lane, 0.0) for lane in outgoing_lanes]
    )
    incoming_cap = np.asarray(
        [incoming_pose.xy + offset * normal for offset in incoming_offsets]
    )
    outgoing_cap = np.asarray(
        [outgoing_pose.xy + offset * normal for offset in outgoing_offsets]
    )
    assert incoming_cap == pytest.approx(outgoing_cap, abs=1e-9)
    for lane in incoming_lanes:
        terminal_widths = [
            _width_at(lane, station)
            for station in np.linspace(
                emitted_incoming.length - 0.1,
                emitted_incoming.length,
                21,
            )
        ]
        assert min(terminal_widths) > 2.9


def test_physical_connection_preserves_two_point_road_in_continuation_chain() -> None:
    def make_line(points: np.ndarray) -> lanelet2.core.LineString3d:
        return lanelet2.core.LineString3d(
            lanelet2.core.getId(),
            [
                lanelet2.core.Point3d(
                    lanelet2.core.getId(),
                    float(x),
                    float(y),
                    float(z),
                )
                for x, y, z in points
            ],
        )

    right_bounds = [
        np.array([[0.0, 0.0, 0.0], [5.0, 0.0, 0.0], [10.0, 0.0, 0.0]]),
        np.array([[10.0, 0.0, 0.0], [20.0, 1.0, 0.0]]),
        np.array([[20.0, 1.0, 0.0], [25.0, 1.5, 0.0], [30.0, 2.0, 0.0]]),
    ]
    left_bounds = [right + np.array([0.0, 3.0, 0.0]) for right in right_bounds]
    lanelet_map = lanelet2.core.LaneletMap()
    lanelets = []
    roads = []
    for index, (left, right) in enumerate(zip(left_bounds, right_bounds)):
        lanelet = lanelet2.core.Lanelet(
            2000 + index,
            make_line(left),
            make_line(right),
        )
        lanelet.attributes["subtype"] = "road"
        lanelet.attributes["one_way"] = "yes"
        lanelet_map.add(lanelet)
        lanelets.append(lanelet)
        roads.append(
            Road.construct_from_lanelet_groups(
                lanelet_map,
                [lanelet],
                road_id=index + 1,
                traffic_rule="LHT",
            )
        )

    for from_road, to_road in zip(roads[:-1], roads[1:]):
        from_road.add_successor(to_road.id, contact_point=ContactPoint.START)
        to_road.add_predecessor(from_road.id, contact_point=ContactPoint.END)
        assert from_road.lanes is not None
        assert to_road.lanes is not None
        from_lane = next(iter(from_road.lanes.lane_sections[0].left_lanes.values()))
        to_lane = next(iter(to_road.lanes.lane_sections[0].left_lanes.values()))
        assert from_lane.lane_id is not None
        assert to_lane.lane_id is not None
        from_lane.successor = LaneLink(id=to_lane.lane_id)
        to_lane.predecessor = LaneLink(id=from_lane.lane_id)

    plans = build_ordinary_physical_connection_plans(
        roads,
        {lanelet.id: lanelet for lanelet in lanelets},
    )
    assert len(plans) == 2
    middle_heading = math.atan2(1.0, 10.0)
    assert plans[0].from_endpoint.heading == pytest.approx(middle_heading)
    assert plans[0].to_endpoint.heading == pytest.approx(middle_heading)
    assert plans[1].from_endpoint.heading == pytest.approx(middle_heading)
    assert plans[1].to_endpoint.heading == pytest.approx(middle_heading)
    endpoint_protected_plans = build_ordinary_physical_connection_plans(
        roads,
        {lanelet.id: lanelet for lanelet in lanelets},
        protected_road_endpoints={(roads[1].id, True)},
    )
    assert [
        (plan.from_road_id, plan.to_road_id) for plan in endpoint_protected_plans
    ] == [(roads[1].id, roads[2].id)]

    converter = _Lanelet2ToOpenDRIVEConverter(
        lanelet_map,
        ConversionConfig(
            traffic_rule="LHT",
            emission_geometry=EmissionGeometryConfig(enabled=True),
        ),
    )
    emitted = converter._build_emitted_roads_after_topology_freeze(
        roads,
        RoadLaneletMapping(
            road_to_lanelets={
                road.id: [lanelet.id] for road, lanelet in zip(roads, lanelets)
            },
            lanelet_to_road={
                lanelet.id: road.id for road, lanelet in zip(roads, lanelets)
            },
        ),
        routing_graph=None,
    )

    middle_context = emitted[1].emission_context
    assert middle_context is not None
    assert middle_context.evaluate(0.0).xy == pytest.approx(
        right_bounds[1][0, :2],
        abs=1e-9,
    )
    assert middle_context.evaluate(middle_context.length).xy == pytest.approx(
        right_bounds[1][-1, :2],
        abs=1e-9,
    )
    for from_road, to_road in zip(emitted[:-1], emitted[1:]):
        assert from_road.emission_context is not None
        assert to_road.emission_context is not None
        from_pose = from_road.emission_context.evaluate(from_road.length)
        to_pose = to_road.emission_context.evaluate(0.0)
        assert from_pose.xy == pytest.approx(to_pose.xy, abs=1e-9)
        assert from_pose.heading == pytest.approx(to_pose.heading, abs=1e-9)


def test_two_point_emission_constraints_use_one_well_conditioned_cubic() -> None:
    right_reference = np.array(
        [[0.0, 0.0, 0.0], [20.0, 1.0, 0.0]],
        dtype=float,
    )
    left_outer = right_reference + np.array([0.0, 3.0, 0.0])
    lanelet_map, lanelet = _make_lanelet_from_bounds(
        left_outer,
        right_reference,
    )
    context = RoadEmissionContext.from_lanelet_groups(
        lanelet_map,
        [lanelet],
        traffic_rule="LHT",
        start_heading_override=0.0,
        end_heading_override=0.0,
    )

    plan_view = context.to_plan_view()
    assert len(plan_view.geometries) == 1
    geometry = plan_view.geometries[0]
    assert isinstance(geometry, ParamPoly3)
    assert geometry.pRange == "arcLength"
    assert geometry.length > 0.5
    assert context.evaluate(0.0).xy == pytest.approx(
        right_reference[0, :2],
        abs=1e-9,
    )
    assert context.evaluate(context.length).xy == pytest.approx(
        right_reference[-1, :2],
        abs=1e-9,
    )
    assert context.evaluate(0.0).heading == pytest.approx(0.0, abs=1e-9)
    assert context.evaluate(context.length).heading == pytest.approx(0.0, abs=1e-9)

    coefficients = (
        geometry.aU,
        geometry.bU,
        geometry.cU,
        geometry.dU,
        geometry.aV,
        geometry.bV,
        geometry.cV,
        geometry.dV,
    )
    samples = np.asarray(
        [
            evaluate_plan_view_world(
                geometry.x,
                geometry.y,
                geometry.hdg,
                float(station),
                param_poly3_coeffs=coefficients,
            )
            for station in np.linspace(0.0, geometry.length, 1001)
        ]
    )
    integrated_length = _polyline_length(samples)
    assert integrated_length == pytest.approx(geometry.length, abs=2e-4)
    assert samples[0] == pytest.approx(right_reference[0, :2], abs=1e-9)
    assert samples[-1] == pytest.approx(right_reference[-1, :2], abs=1e-9)
    assert np.min(np.linalg.norm(np.diff(samples, axis=0), axis=1)) > 0.0


def test_physical_connection_absorbs_short_terminal_segment_without_micro_cubic() -> (
    None
):
    right_reference = np.array(
        [
            [0.0, 0.0, 0.0],
            [10.0, 0.0, 0.0],
            [20.0, 0.0, 0.0],
            [20.008, 0.004, 0.0],
        ],
        dtype=float,
    )
    left_outer = right_reference + np.array([0.0, 3.0, 0.0])
    lanelet_map, lanelet = _make_lanelet_from_bounds(
        left_outer,
        right_reference,
    )
    context = RoadEmissionContext.from_lanelet_groups(
        lanelet_map,
        [lanelet],
        traffic_rule="LHT",
        end_heading_override=0.0,
    )

    plan_view = context.to_plan_view()
    assert all(geometry.length >= 0.5 for geometry in plan_view.geometries)
    assert isinstance(plan_view.geometries[-1], ParamPoly3)
    assert context.evaluate(context.length).xy == pytest.approx(
        right_reference[-1, :2],
        abs=1e-9,
    )
    assert context.evaluate(context.length).heading == pytest.approx(0.0, abs=1e-9)


def test_width_constraint_skips_non_monotone_terminal_cap_support() -> None:
    lane = SimpleNamespace(
        widths=[
            LaneWidth(0.0, 3.0, 0.0, 0.0, 0.0),
            LaneWidth(9.3, 3.0, 0.1, 0.0, 0.0),
            LaneWidth(9.75, 3.045, 3.0, 0.0, 0.0),
        ]
    )

    changed = _constrain_width_endpoint(
        lane,
        10.0,
        at_start=False,
        width=3.05,
        derivative=0.0,
        transition_length=0.25,
    )

    assert changed
    assert lane.widths[-1].s_offset == pytest.approx(9.3)
    stations = np.linspace(9.3, 10.0, 101)
    values = np.asarray(
        [_width_value_and_derivative(lane, float(station))[0] for station in stations]
    )
    assert np.all(np.diff(values) >= -DEFAULT_CONFIG.geometry.epsilon)
    assert values[0] == pytest.approx(3.0)
    assert values[-1] == pytest.approx(3.05)
    assert _width_value_and_derivative(lane, 10.0)[1] == pytest.approx(0.0)


def test_three_point_emission_preserves_source_supported_terminal_tangents() -> None:
    right_reference = np.array(
        [[0.0, 0.0, 0.0], [10.0, 0.2, 0.0], [20.0, 1.0, 0.0]],
        dtype=float,
    )
    left_outer = right_reference + np.array([0.0, 3.0, 0.0])
    lanelet_map, lanelet = _make_lanelet_from_bounds(
        left_outer,
        right_reference,
    )
    start_heading = math.atan2(0.2, 10.0)
    end_heading = math.atan2(0.8, 10.0)
    context = RoadEmissionContext.from_lanelet_groups(
        lanelet_map,
        [lanelet],
        traffic_rule="LHT",
        start_heading_override=start_heading,
        end_heading_override=end_heading,
    )

    plan_view = context.to_plan_view()
    # The interior joint folds offset lanes, so it is C1-filleted: the two
    # terminal chords stay Lines and the joint becomes a Bezier pair that
    # still passes exactly through the source vertex.
    assert len(plan_view.geometries) == 4
    assert isinstance(plan_view.geometries[0], Line)
    assert isinstance(plan_view.geometries[-1], Line)
    source_points = context.emission_geometry.source_points
    middle_index = int(
        np.argmin(np.linalg.norm(source_points - right_reference[1, :2], axis=1))
    )
    middle_station = context.emission_geometry.emission_stations[middle_index]
    assert context.evaluate(0.0).heading == pytest.approx(start_heading, abs=1e-9)
    assert context.evaluate(context.length).heading == pytest.approx(
        end_heading,
        abs=1e-9,
    )
    assert context.evaluate(float(middle_station)).xy == pytest.approx(
        right_reference[1, :2],
        abs=1e-9,
    )


def test_emission_reference_preserves_continuous_short_terminal_curve() -> None:
    right_reference = np.array(
        [
            [0.0, 0.0, 0.0],
            [10.0, 0.0, 0.0],
            [20.0, 5.0, 0.0],
            [20.008660254, 5.005, 0.0],
        ],
        dtype=float,
    )
    left_outer = right_reference.copy()
    left_outer[:, 1] += 3.0
    lanelet_map, lanelet = _make_lanelet_from_bounds(left_outer, right_reference)

    context = RoadEmissionContext.from_lanelet_groups(
        lanelet_map,
        [lanelet],
        traffic_rule="LHT",
    )

    # The micro-kink at the short terminal tail may be C1-filleted, which
    # changes the arc length by micrometres; a collapsed tail would change
    # it by centimetres.
    assert context.length == pytest.approx(
        _polyline_length(right_reference[:, :2]),
        abs=1e-4,
    )
    assert context.evaluate(context.length).heading == pytest.approx(
        math.radians(30.0),
        abs=1e-5,
    )


def test_emission_reference_preserves_true_endpoint_corner() -> None:
    right_reference = np.array(
        [[0.0, 0.0, 0.0], [20.0, 0.0, 0.0], [20.0, 3.0, 0.0]],
        dtype=float,
    )
    left_outer = np.array(
        [[0.0, 3.0, 0.0], [20.0, 3.0, 0.0], [20.0, 6.0, 0.0]],
        dtype=float,
    )
    lanelet_map, lanelet = _make_lanelet_from_bounds(left_outer, right_reference)

    context = RoadEmissionContext.from_lanelet_groups(
        lanelet_map,
        [lanelet],
        traffic_rule="LHT",
    )

    assert context.length == pytest.approx(23.0, abs=1e-6)
    assert context.evaluate(context.length).heading == pytest.approx(
        math.radians(90.0),
        abs=1e-6,
    )


def test_emission_reference_preserves_micro_kink_supported_by_both_boundaries() -> None:
    tiny_tail = np.array([0.008660254, 0.005, 0.0])
    right_reference = np.array(
        [[0.0, 0.0, 0.0], [20.0, 0.0, 0.0], [20.0, 0.0, 0.0] + tiny_tail],
        dtype=float,
    )
    left_outer = np.array(
        [[0.0, 3.0, 0.0], [20.0, 3.0, 0.0], [20.0, 3.0, 0.0] + tiny_tail],
        dtype=float,
    )
    lanelet_map, lanelet = _make_lanelet_from_bounds(left_outer, right_reference)

    context = RoadEmissionContext.from_lanelet_groups(
        lanelet_map,
        [lanelet],
        traffic_rule="LHT",
    )

    assert context.length == pytest.approx(
        _polyline_length(right_reference[:, :2]),
        abs=1e-6,
    )
    assert context.evaluate(context.length).heading == pytest.approx(
        math.radians(30.0),
        abs=1e-5,
    )


def test_emission_reference_cleans_near_duplicate_endpoint() -> None:
    right_reference = np.array(
        [[0.0, 0.0, 0.0], [20.0, 0.0, 0.0], [20.0000001, 0.0, 0.0]],
        dtype=float,
    )
    left_outer = right_reference.copy()
    left_outer[:, 1] += 3.0
    lanelet_map, lanelet = _make_lanelet_from_bounds(left_outer, right_reference)

    context = RoadEmissionContext.from_lanelet_groups(
        lanelet_map,
        [lanelet],
        traffic_rule="LHT",
    )

    assert context.length == pytest.approx(20.0, abs=1e-6)
    assert context.evaluate(context.length).heading == pytest.approx(0.0, abs=1e-6)


def test_emission_width_uses_polygon_cross_sections_for_outer_end_drop() -> None:
    right_reference = np.array(
        [[0.0, 0.0, 0.0], [15.0, 0.0, 0.0], [30.0, 0.0, 0.0]],
        dtype=float,
    )
    left_outer = np.array(
        [[0.0, 3.0, 0.0], [10.0, 3.0, 0.0], [20.0, 3.0, 0.0]],
        dtype=float,
    )
    lanelet_map, lanelet = _make_lanelet_from_bounds(left_outer, right_reference)
    context = RoadEmissionContext.from_lanelet_groups(
        lanelet_map,
        [lanelet],
        traffic_rule="LHT",
    )

    topology_road = Road.construct_from_lanelet_groups(
        lanelet_map,
        [lanelet],
        road_id=51,
        traffic_rule="LHT",
        width_config=WidthEstimationConfig(adaptive_sampling=True),
    )
    road = topology_road.copy_with_emission_context(
        lanelet_map=lanelet_map,
        lanelet_group=[lanelet],
        emission_context=context,
        traffic_rule="LHT",
        width_config=WidthEstimationConfig(adaptive_sampling=True),
    )

    assert road.lanes is not None
    lane = road.lanes.lane_sections[0].left_lanes[1]
    assert _width_at(lane, 0.0) == pytest.approx(3.0, abs=1e-9)
    assert _width_at(lane, 20.0) == pytest.approx(3.0, abs=1e-9)
    assert _width_at(lane, 25.0) == pytest.approx(1.5, abs=1e-9)
    assert _width_at(lane, 30.0) == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize("traffic_rule", ["LHT", "RHT"])
def test_emission_width_refines_oblique_outer_boundary_on_both_sides(
    traffic_rule: str,
) -> None:
    reference = np.array(
        [
            [0.0, 0.0, 0.0],
            [5.0, 0.0, 0.0],
            [9.0, 5.0, 0.0],
            [14.0, 2.5, 0.0],
            [20.0, 0.0, 0.0],
        ],
        dtype=float,
    )
    outer = np.array(
        [
            [0.0, 3.0, 0.0],
            [3.0, 6.0, 0.0],
            [8.0, 20.0, 0.0],
            [15.0, 6.0, 0.0],
            [20.0, 3.0, 0.0],
        ],
        dtype=float,
    )

    if traffic_rule == "LHT":
        lanelet_map, lanelet = _make_lanelet_from_bounds(outer, reference)
        side_sign = 1.0
        reference_points = reference
        outer_points = outer
        expected_lane_id = 1
    else:
        lanelet_map, lanelet = _make_lanelet_from_bounds(
            -reference * np.array([-1.0, 1.0, -1.0]),
            -outer * np.array([-1.0, 1.0, -1.0]),
        )
        side_sign = -1.0
        reference_points = -reference * np.array([-1.0, 1.0, -1.0])
        outer_points = -outer * np.array([-1.0, 1.0, -1.0])
        expected_lane_id = -1

    context = RoadEmissionContext.from_lanelet_groups(
        lanelet_map,
        [lanelet],
        traffic_rule=traffic_rule,
    )
    topology_road = Road.construct_from_lanelet_groups(
        lanelet_map,
        [lanelet],
        road_id=275,
        traffic_rule=traffic_rule,
        width_config=WidthEstimationConfig(adaptive_sampling=True),
    )
    road = topology_road.copy_with_emission_context(
        lanelet_map=lanelet_map,
        lanelet_group=[lanelet],
        emission_context=context,
        traffic_rule=traffic_rule,
        width_config=WidthEstimationConfig(adaptive_sampling=True),
    )

    assert road.lanes is not None
    lane_section = road.lanes.lane_sections[0]
    lane = (
        lane_section.left_lanes[expected_lane_id]
        if expected_lane_id > 0
        else lane_section.right_lanes[expected_lane_id]
    )
    assert len(lane.widths) > 11
    _assert_emitted_width_matches_source_polygon(
        lane,
        reference_points,
        outer_points,
        side_sign,
    )


def test_emission_width_refinement_preserves_lht_multilane_widths() -> None:
    lanelet_map = lanelet2.core.LaneletMap()

    def linestring(points_array: np.ndarray) -> lanelet2.core.LineString3d:
        points = [
            lanelet2.core.Point3d(lanelet2.core.getId(), float(x), float(y), float(z))
            for x, y, z in points_array
        ]
        return lanelet2.core.LineString3d(lanelet2.core.getId(), points)

    right_reference = np.array([[0.0, 0.0, 0.0], [20.0, 0.0, 0.0]], dtype=float)
    middle = np.array([[0.0, 2.0, 0.0], [20.0, 2.0, 0.0]], dtype=float)
    left_outer = np.array([[0.0, 5.0, 0.0], [20.0, 5.0, 0.0]], dtype=float)

    right_bound = linestring(right_reference)
    middle_bound = linestring(middle)
    left_bound = linestring(left_outer)

    right_lane = lanelet2.core.Lanelet(1001, middle_bound, right_bound)
    left_lane = lanelet2.core.Lanelet(1002, left_bound, middle_bound)
    for lanelet in (right_lane, left_lane):
        lanelet.attributes["subtype"] = "road"
        lanelet.attributes["one_way"] = "yes"
        lanelet_map.add(lanelet)

    lanelets = [left_lane, right_lane]
    context = RoadEmissionContext.from_lanelet_groups(
        lanelet_map,
        lanelets,
        traffic_rule="LHT",
    )
    topology_road = Road.construct_from_lanelet_groups(
        lanelet_map,
        lanelets,
        road_id=20,
        traffic_rule="LHT",
        width_config=WidthEstimationConfig(adaptive_sampling=True),
    )
    road = topology_road.copy_with_emission_context(
        lanelet_map=lanelet_map,
        lanelet_group=lanelets,
        emission_context=context,
        traffic_rule="LHT",
        width_config=WidthEstimationConfig(adaptive_sampling=True),
    )

    assert road.lanes is not None
    lane_section = road.lanes.lane_sections[0]
    inner_lane = lane_section.left_lanes[1]
    outer_lane = lane_section.left_lanes[2]
    for station in np.linspace(0.0, road.length, 7):
        assert _width_at(inner_lane, float(station)) == pytest.approx(2.0)
        assert _width_at(outer_lane, float(station)) == pytest.approx(3.0)


def test_polygon_cross_section_width_uses_near_side_interval() -> None:
    polygon_points = np.array(
        [
            [0.0, 0.0],
            [10.0, 0.0],
            [10.0, 2.0],
            [0.0, 2.0],
            [0.0, 4.0],
            [10.0, 4.0],
            [10.0, 6.0],
            [0.0, 6.0],
        ],
        dtype=float,
    )

    width = _polygon_cross_section_width(
        polygon_points,
        origin=np.array([5.0, 0.0]),
        side_direction=np.array([0.0, 1.0]),
    )

    assert width == pytest.approx(2.0)


def test_road_copy_with_emission_context_preserves_topology_road_and_lane_links() -> (
    None
):
    reference_points = np.array(
        [[0.0, 0.0, 0.0], [20.0, 0.0, 0.2], [25.0, 1.0, 0.4]],
        dtype=float,
    )
    lanelet_map, lanelet = _make_lanelet_from_reference(reference_points, width=3.0)
    road = Road.construct_from_lanelet_groups(
        lanelet_map,
        [lanelet],
        road_id=7,
        traffic_rule="RHT",
    )
    assert road.lanes is not None
    lane = road.lanes.lane_sections[0].right_lanes[-1]
    lane.predecessor = LaneLink(id=-1)
    lane.successor = LaneLink(id=-1)

    context = RoadEmissionContext.from_lanelet_groups(
        lanelet_map,
        [lanelet],
        traffic_rule="RHT",
    )
    original_fingerprint = _topology_fingerprint([road])
    original_plan_view = road.plan_view
    emitted = road.copy_with_emission_context(
        lanelet_map=lanelet_map,
        lanelet_group=[lanelet],
        emission_context=context,
        traffic_rule="RHT",
    )

    assert _topology_fingerprint([road]) == original_fingerprint
    assert road.plan_view is original_plan_view
    assert road.emission_context is None

    assert emitted is not road
    assert emitted.plan_view is not None
    assert emitted.length == pytest.approx(context.length)
    assert emitted.lanes is not None
    updated_lane = emitted.lanes.lane_sections[0].right_lanes[-1]
    assert updated_lane.lanelet_id == lanelet.id
    assert updated_lane.predecessor == LaneLink(id=-1)
    assert updated_lane.successor == LaneLink(id=-1)
    assert emitted.emission_context is context


@pytest.mark.parametrize(
    "case_name",
    [
        "road53_long_straight_short_tail",
        "straight",
        "gentle_curve",
        "s_curve",
        "road50_staggered_endpoint",
        "short_valid",
    ],
)
def test_road_emission_context_integration_synthetic_suite(case_name: str) -> None:
    points_2d = SYNTHETIC_CASES[case_name]
    points_3d = np.column_stack((points_2d, np.linspace(1.0, 1.5, len(points_2d))))
    lanelet_map, lanelet = _make_lanelet_from_reference(points_3d, width=2.8)
    context = RoadEmissionContext.from_lanelet_groups(
        lanelet_map,
        [lanelet],
        traffic_rule="RHT",
    )
    topology_road = Road.construct_from_lanelet_groups(
        lanelet_map,
        [lanelet],
        road_id=1,
        traffic_rule="RHT",
    )
    road = topology_road.copy_with_emission_context(
        lanelet_map=lanelet_map,
        lanelet_group=[lanelet],
        emission_context=context,
        traffic_rule="RHT",
    )

    assert road.plan_view is not None
    assert road.elevation_profile is not None
    assert road.length == pytest.approx(context.length)
    assert road.length > 0.0
    assert all(geometry.length > 0.0 for geometry in road.plan_view.geometries)
    assert all(
        math.isfinite(value)
        for geometry in road.plan_view.geometries
        for value in (geometry.s, geometry.x, geometry.y, geometry.hdg, geometry.length)
    )

    source_samples = _sample_polyline(context.emission_geometry.source_points)
    emitted_samples = _sample_road_reference(road)
    assert _symmetric_hausdorff(source_samples, emitted_samples) <= 0.03


def test_copy_only_emission_keeps_topology_fingerprint_for_any_subset() -> None:
    lanelet_map = lanelet2.core.LaneletMap()
    lanelets = []
    for idx, y_offset in enumerate([0.0, 8.0, 16.0], start=1):
        local_map, lanelet = _make_lanelet_from_reference(
            np.array(
                [
                    [0.0, y_offset, 0.0],
                    [20.0, y_offset, 0.2],
                    [25.0, y_offset + 1.0, 0.4],
                ],
                dtype=float,
            ),
            width=3.0,
            lanelet_id=idx,
        )
        assert len(local_map.laneletLayer) == 1
        lanelet_map.add(lanelet)
        lanelets.append(lanelet)

    roads = [
        Road.construct_from_lanelet_groups(
            lanelet_map,
            [lanelet],
            road_id=idx,
            traffic_rule="RHT",
        )
        for idx, lanelet in enumerate(lanelets)
    ]
    roads[0].add_successor(1)
    roads[1].add_predecessor(0)
    roads[1].add_successor(2)
    roads[2].add_predecessor(1)

    for idx, road in enumerate(roads):
        assert road.lanes is not None
        lane = road.lanes.lane_sections[0].right_lanes[-1]
        lane.predecessor = LaneLink(id=-(idx - 1)) if idx > 0 else None
        lane.successor = LaneLink(id=-(idx + 2)) if idx < len(roads) - 1 else None

    baseline = _topology_fingerprint(roads)
    contexts = [
        RoadEmissionContext.from_lanelet_groups(
            lanelet_map,
            [lanelet],
            traffic_rule="RHT",
        )
        for lanelet in lanelets
    ]

    for selected in [set(), {0}, {0, 2}, {0, 1, 2}]:
        emitted_or_topology = [
            road.copy_with_emission_context(
                lanelet_map=lanelet_map,
                lanelet_group=[lanelets[idx]],
                emission_context=contexts[idx],
                traffic_rule="RHT",
            )
            if idx in selected
            else road
            for idx, road in enumerate(roads)
        ]
        assert _topology_fingerprint(roads) == baseline
        assert _topology_fingerprint(emitted_or_topology) == baseline


def test_production_emission_helper_returns_copies_after_topology_freeze() -> None:
    assert not ConversionConfig().emission_geometry.enabled

    reference_points = np.array(
        [[0.0, 0.0, 0.0], [20.0, 0.0, 0.2], [25.0, 1.0, 0.4]],
        dtype=float,
    )
    lanelet_map, lanelet = _make_lanelet_from_reference(reference_points, width=3.0)
    road = Road.construct_from_lanelet_groups(
        lanelet_map,
        [lanelet],
        road_id=42,
        traffic_rule="RHT",
    )
    unmapped_road = Road.construct_from_lanelet_groups(
        lanelet_map,
        [lanelet],
        road_id=43,
        traffic_rule="RHT",
    )
    topology_roads = [road, unmapped_road]
    baseline = _topology_fingerprint(topology_roads)
    converter = _Lanelet2ToOpenDRIVEConverter(
        lanelet_map,
        ConversionConfig(
            traffic_rule="RHT",
            emission_geometry=EmissionGeometryConfig(enabled=True),
        ),
    )
    mapping = RoadLaneletMapping(
        road_to_lanelets={road.id: [lanelet.id]},
        lanelet_to_road={lanelet.id: road.id},
    )

    emitted_roads = converter._build_emitted_roads_after_topology_freeze(
        topology_roads,
        mapping,
        routing_graph=None,
    )

    assert len(emitted_roads) == 2
    assert emitted_roads[0] is not road
    assert emitted_roads[1] is not unmapped_road
    assert road.emission_context is None
    assert unmapped_road.emission_context is None
    assert emitted_roads[0].emission_context is not None
    assert emitted_roads[1].emission_context is None
    assert _topology_fingerprint(topology_roads) == baseline
    assert _topology_fingerprint(emitted_roads) == baseline


def test_post_freeze_emission_reapplies_connecting_road_endpoint_pin() -> None:
    pred_map, pred_lanelet = _make_lanelet_from_reference(
        np.array([[-12.0, 0.0, 0.0], [0.0, 0.0, 0.0]], dtype=float),
        width=4.0,
        lanelet_id=1101,
    )
    conn_map, conn_lanelet = _make_lanelet_from_reference(
        np.array([[0.0, 1.0, 0.0], [8.0, 1.0, 0.0]], dtype=float),
        width=2.0,
        lanelet_id=1102,
    )
    lanelet_map = lanelet2.core.LaneletMap()
    lanelet_map.add(pred_lanelet)
    lanelet_map.add(conn_lanelet)

    predecessor = Road.construct_from_lanelet_groups(
        pred_map,
        [pred_lanelet],
        road_id=1,
        traffic_rule="RHT",
    )
    connecting = Road.construct_from_lanelet_groups(
        conn_map,
        [conn_lanelet],
        road_id=2,
        traffic_rule="RHT",
    )
    connecting.junction = 100
    connecting.add_predecessor(
        element_id=predecessor.id,
        contact_point=ContactPoint.END,
    )
    assert connecting.lanes is not None
    conn_lane = connecting.lanes.lane_sections[0].right_lanes[-1]
    conn_lane.predecessor = LaneLink(id=-1)

    converter = _Lanelet2ToOpenDRIVEConverter(
        lanelet_map,
        ConversionConfig(
            traffic_rule="RHT",
            emission_geometry=EmissionGeometryConfig(enabled=True),
        ),
    )
    emitted = converter._build_emitted_roads_after_topology_freeze(
        [predecessor, connecting],
        RoadLaneletMapping(
            road_to_lanelets={
                predecessor.id: [pred_lanelet.id],
                connecting.id: [conn_lanelet.id],
            },
            lanelet_to_road={
                pred_lanelet.id: predecessor.id,
                conn_lanelet.id: connecting.id,
            },
        ),
        routing_graph=None,
    )

    emitted_predecessor = emitted[0]
    emitted_connecting = emitted[1]
    assert emitted_predecessor.lanes is not None
    assert emitted_connecting.lanes is not None
    expected_start = emitted_predecessor.evaluate_lane_anchor_xyz(
        sorted_index=0,
        at_start=False,
    )

    assert expected_start is not None
    assert emitted_connecting.reference_start_xyz is not None
    assert (
        np.linalg.norm(
            np.asarray(emitted_connecting.reference_start_xyz[:2])
            - np.asarray(expected_start[:2])
        )
        <= 1e-9
    )

    emitted_lane = emitted_connecting.lanes.lane_sections[0].right_lanes[-1]
    predecessor_width = (
        emitted_predecessor.lanes.lane_sections[0].right_lanes[-1].widths[0].a
    )
    assert emitted_lane.widths[0].a == pytest.approx(predecessor_width)
    assert _topology_fingerprint([predecessor, connecting]) == _topology_fingerprint(
        emitted
    )


def test_post_freeze_synthetic_connector_uses_emitted_lane_endpoint_constraints() -> (
    None
):
    lanelet_map = lanelet2.core.LaneletMap()
    pred_map, pred_lanelet = _make_lanelet_from_reference(
        np.array([[-10.0, 0.0, 0.0], [0.0, 0.0, 0.0]], dtype=float),
        width=4.0,
        lanelet_id=1201,
    )
    succ_map, succ_lanelet = _make_lanelet_from_reference(
        np.array([[0.0, 0.0, 0.0], [10.0, 0.0, 0.0]], dtype=float),
        width=2.0,
        lanelet_id=1202,
    )
    lanelet_map.add(pred_lanelet)
    lanelet_map.add(succ_lanelet)

    predecessor = Road.construct_from_lanelet_groups(
        pred_map,
        [pred_lanelet],
        road_id=10,
        traffic_rule="RHT",
    )
    successor = Road.construct_from_lanelet_groups(
        succ_map,
        [succ_lanelet],
        road_id=11,
        traffic_rule="RHT",
    )
    connector = _make_zero_length_connecting_road(
        road_id=12,
        junction_id=1000,
        incoming_road_id=predecessor.id,
        outgoing_road_id=successor.id,
        incoming_contact=ContactPoint.END,
        outgoing_contact=ContactPoint.START,
        start_xyz=(0.0, 3.0, 0.0),
        end_xyz=(0.0, 3.2, 0.0),
        min_segment_length=0.01,
        traffic_rule=TrafficRule.RHT,
        from_lane=-1,
        to_lane=-1,
        fallback_heading=math.pi / 2.0,
        lane_width=3.5,
    )

    converter = _Lanelet2ToOpenDRIVEConverter(
        lanelet_map,
        ConversionConfig(
            traffic_rule="RHT",
            emission_geometry=EmissionGeometryConfig(enabled=True),
        ),
    )
    roads_by_id = {road.id: road for road in [predecessor, successor, connector]}

    assert converter._align_unmapped_connecting_road(connector, roads_by_id)
    assert converter._apply_lane_width_endpoint_constraints(connector, roads_by_id)

    assert connector.reference_start_xyz is not None
    assert connector.reference_end_xyz is not None
    assert np.linalg.norm(np.asarray(connector.reference_start_xyz[:2])) <= 1e-9
    assert np.linalg.norm(np.asarray(connector.reference_end_xyz[:2])) <= 1e-9
    assert connector.plan_view is not None
    assert connector.plan_view.geometries[0].hdg == pytest.approx(0.0)

    assert connector.lanes is not None
    connector_lane = connector.lanes.lane_sections[0].right_lanes[-1]
    start_width = converter._evaluate_width_or_zero(connector_lane, 0.0)
    end_width = converter._evaluate_width_or_zero(connector_lane, connector.length)
    assert start_width == pytest.approx(3.0)
    assert end_width == pytest.approx(3.0)
    assert connector_lane.widths[0].b == pytest.approx(0.0)


def _make_single_lane_road(
    reference: np.ndarray,
    *,
    road_id: int,
    lanelet_id: int,
    traffic_rule: str,
    width: float = 3.0,
) -> tuple[lanelet2.core.LaneletMap, lanelet2.core.Lanelet, Road]:
    if traffic_rule.upper() == "LHT":
        left = reference.copy()
        left[:, 1] += width
        lanelet_map, lanelet = _make_lanelet_from_bounds(
            left,
            reference,
            lanelet_id=lanelet_id,
        )
    else:
        lanelet_map, lanelet = _make_lanelet_from_reference(
            reference,
            width=width,
            lanelet_id=lanelet_id,
        )
    road = Road.construct_from_lanelet_groups(
        lanelet_map,
        [lanelet],
        road_id=road_id,
        traffic_rule=traffic_rule,
    )
    return lanelet_map, lanelet, road


def _align_source_less_connector(
    predecessor: Road,
    successor: Road,
    *,
    incoming_contact: ContactPoint = ContactPoint.END,
    outgoing_contact: ContactPoint = ContactPoint.START,
    traffic_rule: TrafficRule = TrafficRule.RHT,
    from_lane: int = -1,
    to_lane: int = -1,
) -> Road:
    connector = _make_zero_length_connecting_road(
        road_id=102,
        junction_id=1000,
        incoming_road_id=predecessor.id,
        outgoing_road_id=successor.id,
        incoming_contact=incoming_contact,
        outgoing_contact=outgoing_contact,
        start_xyz=(0.0, 1.0, 0.0),
        end_xyz=(0.1, 1.0, 0.0),
        min_segment_length=0.01,
        traffic_rule=traffic_rule,
        from_lane=from_lane,
        to_lane=to_lane,
        fallback_heading=math.pi / 2.0,
        lane_width=3.0,
    )
    lanelet_map = lanelet2.core.LaneletMap()
    converter = _Lanelet2ToOpenDRIVEConverter(
        lanelet_map,
        ConversionConfig(
            traffic_rule=traffic_rule.value,
            emission_geometry=EmissionGeometryConfig(enabled=True),
        ),
    )
    roads_by_id = {road.id: road for road in [predecessor, successor, connector]}
    assert converter._align_unmapped_connecting_road(connector, roads_by_id)
    return connector


@pytest.mark.parametrize(
    ("traffic_rule", "lane_id"),
    [(TrafficRule.RHT, -1), (TrafficRule.LHT, 1)],
)
def test_source_less_short_backward_connector_uses_linked_physical_heading(
    traffic_rule: TrafficRule,
    lane_id: int,
) -> None:
    _pred_map, _pred_lanelet, predecessor = _make_single_lane_road(
        np.array([[-10.0, 0.0, 0.0], [0.0, 0.0, 0.0]], dtype=float),
        road_id=100,
        lanelet_id=1301,
        traffic_rule=traffic_rule.value,
    )
    _succ_map, _succ_lanelet, successor = _make_single_lane_road(
        np.array([[-0.2, 0.0, 0.0], [10.0, 0.0, 0.0]], dtype=float),
        road_id=101,
        lanelet_id=1302,
        traffic_rule=traffic_rule.value,
    )

    connector = _align_source_less_connector(
        predecessor,
        successor,
        traffic_rule=traffic_rule,
        from_lane=lane_id,
        to_lane=lane_id,
    )

    assert connector.length == pytest.approx(
        DEFAULT_CONFIG.geometry.divergence_min_segment_length
    )
    assert connector.reference_start_xyz == pytest.approx((-0.1, 0.0, 0.0))
    assert connector.reference_end_xyz == pytest.approx((-0.1, 0.0, 0.0))
    assert connector.plan_view is not None
    geometry = connector.plan_view.geometries[0]
    assert isinstance(geometry, ParamPoly3)
    assert geometry.x == pytest.approx(-0.1)
    assert geometry.y == pytest.approx(0.0)
    assert geometry.hdg == pytest.approx(0.0)
    # The collapsed stub keeps its midpoint anchor and linked heading, and
    # advances its declared length from there (u(p)=p under arcLength).
    assert geometry.bU == pytest.approx(1.0)


def test_source_less_short_backward_connector_normalizes_contact_point_heading() -> (
    None
):
    _pred_map, _pred_lanelet, predecessor = _make_single_lane_road(
        np.array([[-10.0, 0.0, 0.0], [0.0, 0.0, 0.0]], dtype=float),
        road_id=100,
        lanelet_id=1401,
        traffic_rule="RHT",
    )
    _succ_map, _succ_lanelet, successor = _make_single_lane_road(
        np.array([[10.0, 0.0, 0.0], [-0.2, 0.0, 0.0]], dtype=float),
        road_id=101,
        lanelet_id=1402,
        traffic_rule="RHT",
    )

    connector = _align_source_less_connector(
        predecessor,
        successor,
        outgoing_contact=ContactPoint.END,
    )

    assert connector.length == pytest.approx(
        DEFAULT_CONFIG.geometry.divergence_min_segment_length
    )
    assert connector.reference_start_xyz == pytest.approx((-0.1, 0.0, 0.0))
    assert connector.reference_end_xyz == pytest.approx((-0.1, 0.0, 0.0))
    assert connector.plan_view is not None
    geometry = connector.plan_view.geometries[0]
    assert isinstance(geometry, ParamPoly3)
    assert geometry.hdg == pytest.approx(0.0)
    assert geometry.bU == pytest.approx(1.0)


def test_source_less_short_forward_connector_keeps_endpoint_line() -> None:
    _pred_map, _pred_lanelet, predecessor = _make_single_lane_road(
        np.array([[-10.0, 0.0, 0.0], [0.0, 0.0, 0.0]], dtype=float),
        road_id=100,
        lanelet_id=1501,
        traffic_rule="RHT",
    )
    _succ_map, _succ_lanelet, successor = _make_single_lane_road(
        np.array([[0.2, 0.0, 0.0], [10.0, 1.0, 0.0]], dtype=float),
        road_id=101,
        lanelet_id=1502,
        traffic_rule="RHT",
    )

    connector = _align_source_less_connector(predecessor, successor)

    assert connector.length == pytest.approx(0.2)
    assert connector.reference_start_xyz == pytest.approx((0.0, 0.0, 0.0))
    assert connector.reference_end_xyz == pytest.approx((0.2, 0.0, 0.0))
    assert connector.plan_view is not None
    geometry = connector.plan_view.geometries[0]
    assert isinstance(geometry, ParamPoly3)
    assert geometry.hdg == pytest.approx(0.0)
    assert geometry.bU == pytest.approx(1.0)


def test_source_less_connector_keeps_line_for_genuinely_opposite_physical_tangents() -> (
    None
):
    _pred_map, _pred_lanelet, predecessor = _make_single_lane_road(
        np.array([[-10.0, 0.0, 0.0], [0.0, 0.0, 0.0]], dtype=float),
        road_id=100,
        lanelet_id=1601,
        traffic_rule="RHT",
    )
    _succ_map, _succ_lanelet, successor = _make_single_lane_road(
        np.array([[0.2, 0.0, 0.0], [-10.0, 0.0, 0.0]], dtype=float),
        road_id=101,
        lanelet_id=1602,
        traffic_rule="RHT",
    )

    connector = _align_source_less_connector(predecessor, successor)

    assert connector.length == pytest.approx(0.2)
    assert connector.reference_start_xyz == pytest.approx((0.0, 0.0, 0.0))
    assert connector.reference_end_xyz == pytest.approx((0.2, 0.0, 0.0))
    assert connector.plan_view is not None
    geometry = connector.plan_view.geometries[0]
    assert isinstance(geometry, ParamPoly3)
    assert geometry.hdg == pytest.approx(0.0)
    assert geometry.bU == pytest.approx(1.0)


def test_production_emission_gate_keeps_standard_stop_line_emission_roads() -> None:
    def _road(road_id: int, length: float, emitted: bool) -> MagicMock:
        road = MagicMock()
        road.id = road_id
        road.length = length
        road.plan_view = PlanView(
            geometries=[Line(s=0.0, x=0.0, y=0.0, hdg=0.0, length=length)]
        )
        road.emission_context = object() if emitted else None
        return road

    class _StopLine:
        id = 9001
        attributes = {"type": "stop_line"}

        def __iter__(self):
            return iter(
                [
                    SimpleNamespace(x=10.0, y=-1.0, z=0.0),
                    SimpleNamespace(x=10.0, y=1.0, z=0.0),
                ]
            )

        def __len__(self):
            return 2

    stop_line = _StopLine()

    lanelet_map = MagicMock()
    lanelet_map.lineStringLayer = [stop_line]
    lanelet_map.laneletLayer = []
    converter = _Lanelet2ToOpenDRIVEConverter(lanelet_map, ConversionConfig())

    topology_road = _road(1, length=10.0, emitted=False)
    emitted_road = _road(1, length=9.0, emitted=True)
    filtered = converter._preserve_topology_roads_for_stop_line_fidelity(
        [topology_road],
        [emitted_road],
        lanelet_to_road_and_lane={},
        routing_graph=None,
    )

    assert len(filtered) == 1
    assert filtered[0] is emitted_road
    assert filtered[0].emission_context is not None


def test_production_emission_gate_preserves_topology_for_carla_stop_line_clamp() -> (
    None
):
    def _road(road_id: int, length: float, emitted: bool) -> MagicMock:
        road = MagicMock()
        road.id = road_id
        road.length = length
        road.plan_view = PlanView(
            geometries=[Line(s=0.0, x=0.0, y=0.0, hdg=0.0, length=length)]
        )
        road.emission_context = object() if emitted else None
        return road

    class _StopLine:
        id = 9002
        attributes = {"type": "stop_line"}

        def __iter__(self):
            return iter(
                [
                    SimpleNamespace(x=10.0, y=-1.0, z=0.0),
                    SimpleNamespace(x=10.0, y=1.0, z=0.0),
                ]
            )

        def __len__(self):
            return 2

    config = ConversionConfig()
    config.stopline.carla_stop_line = True
    lanelet_map = MagicMock()
    lanelet_map.lineStringLayer = [_StopLine()]
    lanelet_map.laneletLayer = []
    converter = _Lanelet2ToOpenDRIVEConverter(lanelet_map, config)

    topology_road = _road(1, length=10.0, emitted=False)
    emitted_road = _road(1, length=9.0, emitted=True)
    filtered = converter._preserve_topology_roads_for_stop_line_fidelity(
        [topology_road],
        [emitted_road],
        lanelet_to_road_and_lane={},
        routing_graph=None,
    )

    assert len(filtered) == 1
    assert filtered[0] is not emitted_road
    assert filtered[0] is not topology_road
    assert filtered[0].id == topology_road.id
    assert filtered[0].emission_context is None


def test_divergence_sanity_gate_uses_frozen_topology_after_emission_copy() -> None:
    lanelet_map, source_group, cand_a_group, cand_b_group = (
        _make_two_lane_merge_fixture()
    )
    source = Road.construct_from_lanelet_groups(
        lanelet_map,
        source_group,
        road_id=10,
        traffic_rule="RHT",
    )
    cand_a = Road.construct_from_lanelet_groups(
        lanelet_map,
        cand_a_group,
        road_id=20,
        traffic_rule="RHT",
    )
    cand_b = Road.construct_from_lanelet_groups(
        lanelet_map,
        cand_b_group,
        road_id=21,
        traffic_rule="RHT",
    )

    def gate_inputs() -> SanityGateInputs:
        source_outer = source.evaluate_lane_anchor_xyz(sorted_index=0, at_start=True)
        source_inner = source.evaluate_lane_anchor_xyz(sorted_index=1, at_start=True)
        cand_a_end = cand_a.evaluate_lane_anchor_xyz(sorted_index=0, at_start=False)
        cand_b_end = cand_b.evaluate_lane_anchor_xyz(sorted_index=0, at_start=False)
        assert source.reference_start_xyz is not None
        assert cand_a.reference_end_xyz is not None
        assert cand_b.reference_end_xyz is not None
        assert source_outer is not None
        assert source_inner is not None
        assert cand_a_end is not None
        assert cand_b_end is not None
        return SanityGateInputs(
            endpoint_road=source.reference_start_xyz,
            endpoints_candidates={
                cand_a.id: cand_a.reference_end_xyz,
                cand_b.id: cand_b.reference_end_xyz,
            },
            lane_pairs=[(-1, cand_a.id, -1), (-2, cand_b.id, -1)],
            all_successor_lanelet_road_ids={cand_a.id, cand_b.id},
            lane_pair_endpoint_distances={
                (-1, cand_a.id, -1): float(
                    np.linalg.norm(np.asarray(source_outer) - np.asarray(cand_a_end))
                ),
                (-2, cand_b.id, -1): float(
                    np.linalg.norm(np.asarray(source_inner) - np.asarray(cand_b_end))
                ),
            },
        )

    site = DivergenceSite(
        road_id=source.id,
        side=DivergenceSide.PREDECESSOR,
        candidate_road_ids=[cand_a.id, cand_b.id],
    )
    before = gate_inputs()
    passed, reason = sanity_gate_passes(site, before, endpoint_tolerance=0.5)
    assert passed, reason

    baseline_fingerprint = _topology_fingerprint([source, cand_a, cand_b])
    context = RoadEmissionContext.from_lanelet_groups(
        lanelet_map,
        source_group,
        traffic_rule="RHT",
    )
    emitted_source = source.copy_with_emission_context(
        lanelet_map=lanelet_map,
        lanelet_group=source_group,
        emission_context=context,
        traffic_rule="RHT",
    )

    assert emitted_source is not source
    assert _topology_fingerprint([source, cand_a, cand_b]) == baseline_fingerprint
    after = gate_inputs()
    assert after.lane_pair_endpoint_distances == before.lane_pair_endpoint_distances
    passed, reason = sanity_gate_passes(site, after, endpoint_tolerance=0.5)
    assert passed, reason


@pytest.mark.parametrize(
    (
        "case_name",
        "reference_points",
        "source_points",
        "expected_min",
        "expected_max",
        "expected_start_overhang",
        "expected_end_overhang",
        "expected_start_gap",
        "expected_end_gap",
    ),
    [
        (
            "no_overhang",
            [[0.0, 0.0], [30.0, 0.0]],
            [[0.0, 3.0], [30.0, 3.0]],
            0.0,
            30.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ),
        (
            "outer_starts_before_reference",
            [[0.0, 0.0], [30.0, 0.0]],
            [[-10.0, 3.0], [30.0, 3.0]],
            -10.0,
            30.0,
            10.0,
            0.0,
            0.0,
            0.0,
        ),
        (
            "outer_starts_after_reference",
            [[0.0, 0.0], [30.0, 0.0]],
            [[10.0, 3.0], [30.0, 3.0]],
            10.0,
            30.0,
            0.0,
            0.0,
            10.0,
            0.0,
        ),
        (
            "outer_ends_before_reference",
            [[0.0, 0.0], [30.0, 0.0]],
            [[0.0, 3.0], [20.0, 3.0]],
            0.0,
            20.0,
            0.0,
            0.0,
            0.0,
            10.0,
        ),
        (
            "outer_ends_after_reference",
            [[0.0, 0.0], [30.0, 0.0]],
            [[0.0, 3.0], [40.0, 3.0]],
            0.0,
            40.0,
            0.0,
            10.0,
            0.0,
            0.0,
        ),
        (
            "both_side_overhang",
            [[0.0, 0.0], [30.0, 0.0]],
            [[-5.0, 3.0], [35.0, 3.0]],
            -5.0,
            35.0,
            5.0,
            5.0,
            0.0,
            0.0,
        ),
        (
            "curved_overhang",
            [[0.0, 0.0], [15.0, 0.0], [30.0, 5.0]],
            [[-5.0, 3.0], [15.0, 3.0], [32.0, 8.0]],
            -5.0,
            33.65743819499343,
            5.0,
            2.846049894151531,
            0.0,
            0.0,
        ),
        (
            "multilane_shoulder_overhang",
            [[0.0, 0.0], [30.0, 0.0]],
            [[-4.0, 6.0], [34.0, 6.0]],
            -4.0,
            34.0,
            4.0,
            4.0,
            0.0,
            0.0,
        ),
        (
            "predecessor_successor_attached",
            [[0.0, 0.0], [30.0, 0.0]],
            [[-8.0, 3.0], [0.0, 3.0], [30.0, 3.0], [38.0, 3.0]],
            -8.0,
            38.0,
            8.0,
            8.0,
            0.0,
            0.0,
        ),
        (
            "taper_to_zero_inside_domain",
            [[0.0, 0.0], [30.0, 0.0]],
            [[10.0, 0.0], [20.0, 3.0], [30.0, 3.0]],
            10.0,
            30.0,
            0.0,
            0.0,
            10.0,
            0.0,
        ),
    ],
)
def test_reference_domain_coverage_detects_overhang_cases(
    case_name: str,
    reference_points: list[list[float]],
    source_points: list[list[float]],
    expected_min: float,
    expected_max: float,
    expected_start_overhang: float,
    expected_end_overhang: float,
    expected_start_gap: float,
    expected_end_gap: float,
) -> None:
    reference = EmissionReferenceGeometry.from_source_boundary(reference_points)
    coverage = measure_reference_domain_coverage(source_points, reference)

    assert coverage.projected_min_station == pytest.approx(
        expected_min, abs=1e-6
    ), case_name
    assert coverage.projected_max_station == pytest.approx(
        expected_max, abs=1e-6
    ), case_name
    assert coverage.start_overhang == pytest.approx(
        expected_start_overhang, abs=1e-6
    ), case_name
    assert coverage.end_overhang == pytest.approx(
        expected_end_overhang, abs=1e-6
    ), case_name
    assert coverage.domain_start_gap == pytest.approx(
        expected_start_gap, abs=1e-6
    ), case_name
    assert coverage.domain_end_gap == pytest.approx(
        expected_end_gap, abs=1e-6
    ), case_name
    assert 0.0 <= coverage.source_coverage_ratio <= 1.0
    assert 0.0 <= coverage.domain_coverage_ratio <= 1.0


def _divergence_partition_fixture():
    """Two branch roads whose ends tile one trunk road start (LHT)."""

    def line(points: np.ndarray) -> lanelet2.core.LineString3d:
        return lanelet2.core.LineString3d(
            lanelet2.core.getId(),
            [
                lanelet2.core.Point3d(
                    lanelet2.core.getId(),
                    float(x),
                    float(y),
                    float(z),
                )
                for x, y, z in points
            ],
        )

    def bounds(x0: float, x1: float, offset: float) -> np.ndarray:
        return np.array(
            [
                [x0, offset, 0.0],
                [(x0 + x1) / 2.0, offset, 0.0],
                [x1, offset, 0.0],
            ],
            dtype=float,
        )

    lanelet_map = lanelet2.core.LaneletMap()
    # Adjacent lanelets must share boundary LineString objects so that road
    # grouping recognizes them as one multi-lane road.
    branch_lines = {
        offset: line(bounds(0.0, 20.0, offset)) for offset in (0.0, 3.0, 6.0, 9.0)
    }
    trunk_lines = {
        offset: line(bounds(20.0, 30.0, offset)) for offset in (0.0, 3.0, 6.0, 9.0)
    }

    def make_lanelet(lines, inner: float, outer: float):
        lanelet = lanelet2.core.Lanelet(
            lanelet2.core.getId(),
            lines[outer],
            lines[inner],
        )
        lanelet.attributes["subtype"] = "road"
        lanelet.attributes["one_way"] = "yes"
        lanelet_map.add(lanelet)
        return lanelet

    branch_a_lanelets = [make_lanelet(branch_lines, 0.0, 3.0)]
    branch_b_lanelets = [
        make_lanelet(branch_lines, 3.0, 6.0),
        make_lanelet(branch_lines, 6.0, 9.0),
    ]
    trunk_lanelets = [
        make_lanelet(trunk_lines, 0.0, 3.0),
        make_lanelet(trunk_lines, 3.0, 6.0),
        make_lanelet(trunk_lines, 6.0, 9.0),
    ]

    branch_a = Road.construct_from_lanelet_groups(
        lanelet_map,
        branch_a_lanelets,
        road_id=1,
        traffic_rule="LHT",
    )
    branch_b = Road.construct_from_lanelet_groups(
        lanelet_map,
        branch_b_lanelets,
        road_id=2,
        traffic_rule="LHT",
    )
    trunk = Road.construct_from_lanelet_groups(
        lanelet_map,
        trunk_lanelets,
        road_id=3,
        traffic_rule="LHT",
    )

    def lane_id_for(road: Road, lanelet_id: int) -> int:
        assert road.lanes is not None
        for lane in road.lanes.lane_sections[0].left_lanes.values():
            if lane.lanelet_id == lanelet_id:
                assert lane.lane_id is not None
                return lane.lane_id
        raise AssertionError(f"no lane for lanelet {lanelet_id}")

    stub_specs = [
        (11, branch_a, branch_a_lanelets[0], trunk_lanelets[0]),
        (12, branch_b, branch_b_lanelets[0], trunk_lanelets[1]),
        (13, branch_b, branch_b_lanelets[1], trunk_lanelets[2]),
    ]
    stubs = []
    for stub_id, branch, branch_lanelet, trunk_lanelet in stub_specs:
        from_lane = lane_id_for(branch, branch_lanelet.id)
        to_lane = lane_id_for(trunk, trunk_lanelet.id)
        anchor = (20.0, float(branch_lanelet.rightBound[-1].y), 0.0)
        stubs.append(
            _make_zero_length_connecting_road(
                road_id=stub_id,
                junction_id=900,
                incoming_road_id=branch.id,
                outgoing_road_id=trunk.id,
                incoming_contact=ContactPoint.END,
                outgoing_contact=ContactPoint.START,
                start_xyz=anchor,
                end_xyz=anchor,
                min_segment_length=(
                    DEFAULT_CONFIG.geometry.divergence_min_segment_length
                ),
                traffic_rule=TrafficRule.LHT,
                from_lane=from_lane,
                to_lane=to_lane,
            )
        )

    lanelet_by_id = {
        lanelet.id: lanelet
        for lanelet in branch_a_lanelets + branch_b_lanelets + trunk_lanelets
    }
    return branch_a, branch_b, trunk, stubs, lanelet_by_id


def test_divergence_partition_plans_share_one_trunk_cross_section() -> None:
    branch_a, branch_b, trunk, stubs, lanelet_by_id = _divergence_partition_fixture()

    plans = build_divergence_physical_connection_plans(
        [branch_a, branch_b, trunk, *stubs],
        lanelet_by_id,
    )

    assert len(plans) == 2
    plans_by_branch = {plan.from_road_id: plan for plan in plans}
    assert set(plans_by_branch) == {branch_a.id, branch_b.id}

    plan_a = plans_by_branch[branch_a.id]
    assert plan_a.to_road_id == trunk.id
    assert plan_a.connection_type is PhysicalConnectionType.MERGE
    assert plan_a.cross_section.lane_widths == pytest.approx((3.0,))
    assert plan_a.from_endpoint.reference_xyz[:2] == pytest.approx((20.0, 0.0))
    assert plan_a.to_endpoint.reference_xyz[:2] == pytest.approx((20.0, 0.0))
    assert plan_a.cross_section.heading == pytest.approx(0.0, abs=1e-9)

    plan_b = plans_by_branch[branch_b.id]
    assert plan_b.cross_section.lane_widths == pytest.approx((3.0, 3.0))
    assert plan_b.from_endpoint.reference_xyz[:2] == pytest.approx((20.0, 3.0))
    assert plan_b.to_endpoint.reference_xyz[:2] == pytest.approx((20.0, 0.0))
    assert [
        (pair.from_lane_id, pair.to_lane_id) for pair in plan_b.lane_correspondences
    ] == [(1, 2), (2, 3)]

    constraints = endpoint_constraints_by_road(plans)
    assert set(constraints[trunk.id]) == {"start"}
    assert set(constraints[branch_a.id]) == {"end"}
    assert set(constraints[branch_b.id]) == {"end"}
    assert constraints[trunk.id]["start"].reference_xyz[:2] == pytest.approx(
        (20.0, 0.0)
    )
    assert constraints[branch_b.id]["end"].reference_xyz[:2] == pytest.approx(
        (20.0, 3.0)
    )


def test_divergence_plans_skip_forks_reusing_a_branch_lane() -> None:
    branch_a, branch_b, trunk, stubs, lanelet_by_id = _divergence_partition_fixture()

    # Reuse branch A's only lane for a second stub: the interface is a true
    # fork, so no partition contract may be planned for either target.
    fork_stub = _make_zero_length_connecting_road(
        road_id=14,
        junction_id=900,
        incoming_road_id=branch_a.id,
        outgoing_road_id=trunk.id,
        incoming_contact=ContactPoint.END,
        outgoing_contact=ContactPoint.START,
        start_xyz=(20.0, 0.0, 0.0),
        end_xyz=(20.0, 0.0, 0.0),
        min_segment_length=DEFAULT_CONFIG.geometry.divergence_min_segment_length,
        traffic_rule=TrafficRule.LHT,
        from_lane=1,
        to_lane=1,
    )

    plans = build_divergence_physical_connection_plans(
        [branch_a, branch_b, trunk, *stubs, fork_stub],
        lanelet_by_id,
    )

    assert plans == []


def _emitted_lane_polygons_for_test(road, num_samples: int = 80):
    """Sample emitted lane surfaces as closed rings from planView + widths."""
    stations = np.linspace(0.0, road.length, num_samples)
    section = road.lanes.lane_sections[0]
    lanes = [section.left_lanes[k] for k in sorted(section.left_lanes)]

    def width_at(lane, station):
        active = lane.widths[0]
        for record in lane.widths:
            if record.s_offset <= station + 1e-9:
                active = record
        ds = station - active.s_offset
        return active.a + active.b * ds + active.c * ds**2 + active.d * ds**3

    rows = []
    for station in stations:
        pose = road.emission_context.evaluate(float(station))
        x, y, heading = pose.x, pose.y, pose.heading
        normal = np.array([-math.sin(heading), math.cos(heading)])
        offsets = [0.0]
        for lane in lanes:
            offsets.append(offsets[-1] + width_at(lane, float(station)))
        rows.append((np.array([x, y]), normal, offsets))

    polygons = []
    for index in range(len(lanes)):
        inner = [pt + offs[index] * n for pt, n, offs in rows]
        outer = [pt + offs[index + 1] * n for pt, n, offs in rows]
        polygons.append(np.asarray(inner + outer[::-1]))
    return polygons


def _point_in_ring(point, ring) -> bool:
    x, y = point
    inside = False
    j = len(ring) - 1
    for i in range(len(ring)):
        xi, yi = ring[i]
        xj, yj = ring[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


def test_true_fork_emission_covers_continuous_source_surface() -> None:
    """A true fork with continuous source must not lose drivable surface.

    Minimal fixture mirroring the Odaiba fork pattern: a two-lane trunk whose
    inner lane feeds both a straight two-lane branch and an angled exit
    branch. The exit shares the trunk cap edge, so the source union is
    continuous; near the apex the branches overlap by construction. The fork
    is intentionally NOT planned (a lane feeds two roads), and the emitted
    surfaces must still cover every interior source point.
    """

    def line(points):
        return lanelet2.core.LineString3d(
            lanelet2.core.getId(),
            [
                lanelet2.core.Point3d(lanelet2.core.getId(), *map(float, p))
                for p in points
            ],
        )

    lanelet_map = lanelet2.core.LaneletMap()

    def add_lanelet(left_line, right_line):
        lanelet = lanelet2.core.Lanelet(lanelet2.core.getId(), left_line, right_line)
        lanelet.attributes["subtype"] = "road"
        lanelet.attributes["one_way"] = "yes"
        lanelet_map.add(lanelet)
        return lanelet

    def straight(offset, x0, x1):
        return [
            [x0, offset, 0.0],
            [(x0 + x1) / 2.0, offset, 0.0],
            [x1, offset, 0.0],
        ]

    trunk_lines = {o: line(straight(o, 0.0, 20.0)) for o in (0.0, 3.0, 6.0)}
    branch_a_lines = {o: line(straight(o, 20.0, 40.0)) for o in (0.0, 3.0, 6.0)}
    # Angled exit branch: shares the trunk lane-1 cap points at x=20 and
    # fans away at roughly 14 degrees.
    exit_right = line([[20.0, 0.0, 0.0], [30.0, -2.5, 0.0], [40.0, -5.0, 0.0]])
    exit_left = line([[20.0, 3.0, 0.0], [30.0, 0.5, 0.0], [40.0, -2.0, 0.0]])

    trunk_lanelets = [
        add_lanelet(trunk_lines[3.0], trunk_lines[0.0]),
        add_lanelet(trunk_lines[6.0], trunk_lines[3.0]),
    ]
    branch_a_lanelets = [
        add_lanelet(branch_a_lines[3.0], branch_a_lines[0.0]),
        add_lanelet(branch_a_lines[6.0], branch_a_lines[3.0]),
    ]
    branch_b_lanelets = [add_lanelet(exit_left, exit_right)]

    emitted = []
    source_groups = [trunk_lanelets, branch_a_lanelets, branch_b_lanelets]
    for road_id, group in enumerate(source_groups, start=1):
        road = Road.construct_from_lanelet_groups(
            lanelet_map,
            group,
            road_id=road_id,
            traffic_rule="LHT",
        )
        context = RoadEmissionContext.from_lanelet_groups(
            lanelet_map,
            group,
            traffic_rule="LHT",
        )
        emitted.append(
            road.copy_with_emission_context(
                lanelet_map=lanelet_map,
                lanelet_group=group,
                emission_context=context,
                traffic_rule="LHT",
                width_config=WidthEstimationConfig(adaptive_sampling=True),
            )
        )

    rings = [ring for road in emitted for ring in _emitted_lane_polygons_for_test(road)]

    def sample_interior(lanelet, u, v):
        left = np.asarray([[p.x, p.y] for p in lanelet.leftBound])
        right = np.asarray([[p.x, p.y] for p in lanelet.rightBound])

        def along(points, fraction):
            seg = np.linalg.norm(np.diff(points, axis=0), axis=1)
            total = float(seg.sum())
            target = fraction * total
            acc = 0.0
            for i, s in enumerate(seg):
                if acc + s >= target:
                    ratio = (target - acc) / s
                    return points[i] + ratio * (points[i + 1] - points[i])
                acc += s
            return points[-1]

        return (1.0 - v) * along(right, u) + v * along(left, u)

    uncovered = []
    for group in source_groups:
        for lanelet in group:
            for u in np.linspace(0.03, 0.97, 25):
                for v in np.linspace(0.1, 0.9, 7):
                    point = sample_interior(lanelet, float(u), float(v))
                    if not any(_point_in_ring(point, ring) for ring in rings):
                        uncovered.append(tuple(np.round(point, 3)))

    assert uncovered == []


def _heading_jump_at(context, station: float) -> float:
    before = context.evaluate(max(0.0, station - 1e-4)).heading
    after = context.evaluate(min(context.length, station + 1e-4)).heading
    return math.degrees(
        abs(math.atan2(math.sin(after - before), math.cos(after - before)))
    )


def test_smooth_curved_source_emits_c1_under_terminal_heading_overrides() -> None:
    """A smooth curve split into segments must stay C1 with overrides.

    The endpoint heading overrides model shared physical-connection tangents
    that differ slightly from the terminal chords (the true curve tangent at
    the endpoints). The blend must honor them without displacing interior
    source points or injecting internal tangent kinks — the historic failure
    mode moved the second-to-last point, bending the neighbouring segment by
    the whole correction.
    """
    radius = 300.0
    stations = np.arange(0.0, 30.1, 5.0)
    angles = stations / radius
    right_reference = np.column_stack(
        [
            radius * np.sin(angles),
            radius * (1.0 - np.cos(angles)),
            np.zeros_like(angles),
        ]
    )
    left_outer = right_reference + np.array([0.0, 3.5, 0.0])
    lanelet_map, lanelet = _make_lanelet_from_bounds(left_outer, right_reference)

    start_tangent = 0.0
    end_tangent = float(angles[-1])
    context = RoadEmissionContext.from_lanelet_groups(
        lanelet_map,
        [lanelet],
        traffic_rule="LHT",
        start_heading_override=start_tangent,
        end_heading_override=end_tangent,
    )

    assert context.evaluate(0.0).heading == pytest.approx(start_tangent, abs=1e-8)
    assert context.evaluate(context.length).heading == pytest.approx(
        end_tangent, abs=1e-8
    )

    # Interior source points are never displaced: every source vertex lies
    # exactly on the emitted geometry (kink fillets pass through vertices).
    chord = np.linalg.norm(np.diff(right_reference[:, :2], axis=0), axis=1)
    source_points = context.emission_geometry.source_points
    emitted_stations = context.emission_geometry.emission_stations
    for index in range(1, len(right_reference) - 1):
        vertex = right_reference[index, :2]
        nearest = int(np.argmin(np.linalg.norm(source_points - vertex, axis=1)))
        pose = context.evaluate(float(emitted_stations[nearest]))
        assert pose.xy == pytest.approx(vertex, abs=1e-9)

    # Every internal joint stays within the source vertex bend budget.
    vertex_bend = math.degrees(chord[0] / radius)
    for station in emitted_stations[1:-1]:
        assert _heading_jump_at(context, float(station)) <= vertex_bend + 0.1


def test_true_source_kink_is_preserved_by_emission() -> None:
    """A genuine sharp source corner must survive emission unchanged."""
    kink = math.radians(15.0)
    first = np.array([[0.0, 0.0], [7.5, 0.0], [15.0, 0.0]])
    direction = np.array([math.cos(kink), math.sin(kink)])
    second = np.array([first[-1] + 7.5 * direction, first[-1] + 15.0 * direction])
    right_reference = np.column_stack([np.vstack([first, second]), np.zeros(5)])
    left_outer = right_reference + np.array([0.0, 3.5, 0.0])
    lanelet_map, lanelet = _make_lanelet_from_bounds(left_outer, right_reference)

    context = RoadEmissionContext.from_lanelet_groups(
        lanelet_map,
        [lanelet],
        traffic_rule="LHT",
        start_heading_override=0.0,
        end_heading_override=float(kink),
    )

    assert _heading_jump_at(context, 15.0) == pytest.approx(
        15.0,
        abs=0.1,
    )


def _oblique_seam_fixture(
    from_left,
    from_right,
    to_left,
    to_right,
    traffic_rule: str = "LHT",
):
    """Two linked single-lanelet roads sharing a terminal cap edge."""

    def line(points):
        return lanelet2.core.LineString3d(
            lanelet2.core.getId(),
            [
                lanelet2.core.Point3d(lanelet2.core.getId(), *map(float, p))
                for p in points
            ],
        )

    lanelet_map = lanelet2.core.LaneletMap()
    lanelets = []
    roads = []
    for index, (left, right) in enumerate(
        ((from_left, from_right), (to_left, to_right))
    ):
        lanelet = lanelet2.core.Lanelet(lanelet2.core.getId(), line(left), line(right))
        lanelet.attributes["subtype"] = "road"
        lanelet.attributes["one_way"] = "yes"
        lanelet_map.add(lanelet)
        lanelets.append(lanelet)
        roads.append(
            Road.construct_from_lanelet_groups(
                lanelet_map,
                [lanelet],
                road_id=index + 1,
                traffic_rule=traffic_rule,
            )
        )
    from_road, to_road = roads
    from_road.add_successor(to_road.id, contact_point=ContactPoint.START)
    to_road.add_predecessor(from_road.id, contact_point=ContactPoint.END)
    sections = []
    for road in roads:
        assert road.lanes is not None
        sections.append(road.lanes.lane_sections[0])
    lanes = [
        next(iter((sec.left_lanes or sec.right_lanes).values())) for sec in sections
    ]
    assert lanes[0].lane_id is not None and lanes[1].lane_id is not None
    lanes[0].successor = LaneLink(id=lanes[1].lane_id)
    lanes[1].predecessor = LaneLink(id=lanes[0].lane_id)
    lanelet_by_id = {lanelet.id: lanelet for lanelet in lanelets}
    return lanelet_map, roads, lanelet_by_id


def _straight(offset, x0, x1, cap_shift=0.0):
    return [
        [x0, offset, 0.0],
        [(x0 + x1) / 2.0, offset, 0.0],
        [x1 + cap_shift, offset, 0.0],
    ]


def test_oblique_cap_seam_keeps_source_travel_tangent() -> None:
    """Case A: straight travel with a 30-deg oblique cap.

    The physical terminal cross-section is oblique, but both roads travel
    straight along x. The seam contract must keep the travel tangent instead
    of rotating both references onto the cap normal.
    """
    cap_shift = 2.0  # left bound cap point sits 2 m further along x
    lanelet_map, roads, lanelet_by_id = _oblique_seam_fixture(
        from_left=_straight(3.5, 0.0, 20.0, cap_shift),
        from_right=_straight(0.0, 0.0, 20.0),
        to_left=[[22.0, 3.5, 0.0], [30.0, 3.5, 0.0], [40.0, 3.5, 0.0]],
        to_right=[[20.0, 0.0, 0.0], [30.0, 0.0, 0.0], [40.0, 0.0, 0.0]],
    )
    plans = build_ordinary_physical_connection_plans(roads, lanelet_by_id)
    assert len(plans) == 1
    assert plans[0].cross_section.heading == pytest.approx(0.0, abs=math.radians(0.2))
    assert plans[0].from_endpoint.heading == pytest.approx(0.0, abs=math.radians(0.2))

    # The emitted from-road reference must stay straight: no zigzag pulled
    # in by the oblique cap.
    context = RoadEmissionContext.from_lanelet_groups(
        lanelet_map,
        [lanelet_by_id[min(lanelet_by_id)]],
        traffic_rule="LHT",
        end_heading_override=plans[0].from_endpoint.heading,
    )
    for station in np.linspace(0.0, context.length, 41):
        assert abs(context.evaluate(float(station)).heading) <= math.radians(0.25)


def test_perpendicular_cap_seam_keeps_cap_normal_heading() -> None:
    """Case B: a perpendicular cap behaves exactly as before."""
    _lanelet_map, roads, lanelet_by_id = _oblique_seam_fixture(
        from_left=_straight(3.5, 0.0, 20.0),
        from_right=_straight(0.0, 0.0, 20.0),
        to_left=_straight(3.5, 20.0, 40.0),
        to_right=_straight(0.0, 20.0, 40.0),
    )
    plans = build_ordinary_physical_connection_plans(roads, lanelet_by_id)
    assert len(plans) == 1
    assert plans[0].cross_section.heading == pytest.approx(0.0, abs=1e-9)
    assert plans[0].cross_section.lane_widths == pytest.approx((3.5,))


def test_true_source_turn_seam_uses_miter_heading() -> None:
    """Case C: a genuine 20-deg source turn shares the miter direction."""
    turn = math.radians(20.0)
    direction = np.array([math.cos(turn), math.sin(turn)])

    def turned(start):
        base = np.asarray(start, dtype=float)
        return [
            list(base) + [0.0],
            list(base + 10.0 * direction) + [0.0],
            list(base + 20.0 * direction) + [0.0],
        ]

    lanelet_map, roads, lanelet_by_id = _oblique_seam_fixture(
        from_left=_straight(3.5, 0.0, 20.0),
        from_right=_straight(0.0, 0.0, 20.0),
        to_left=turned([20.0, 3.5]),
        to_right=turned([20.0, 0.0]),
    )
    plans = build_ordinary_physical_connection_plans(roads, lanelet_by_id)
    assert len(plans) == 1
    assert plans[0].cross_section.heading == pytest.approx(
        turn / 2.0, abs=math.radians(0.5)
    )


def test_multi_lane_oblique_cap_seam_keeps_travel_tangent_and_widths() -> None:
    """Cases D/E: multi-lane oblique cap, both lane orderings."""

    def two_lane_roads(cap_shift):
        def line(points):
            return lanelet2.core.LineString3d(
                lanelet2.core.getId(),
                [
                    lanelet2.core.Point3d(lanelet2.core.getId(), *map(float, p))
                    for p in points
                ],
            )

        lanelet_map = lanelet2.core.LaneletMap()
        from_lines = {
            0.0: line(_straight(0.0, 0.0, 20.0)),
            3.5: line(_straight(3.5, 0.0, 20.0, cap_shift / 2.0)),
            7.0: line(_straight(7.0, 0.0, 20.0, cap_shift)),
        }
        to_lines = {
            0.0: line([[20.0, 0.0, 0.0], [30.0, 0.0, 0.0], [40.0, 0.0, 0.0]]),
            3.5: line(
                [
                    [20.0 + cap_shift / 2.0, 3.5, 0.0],
                    [30.0, 3.5, 0.0],
                    [40.0, 3.5, 0.0],
                ]
            ),
            7.0: line(
                [[20.0 + cap_shift, 7.0, 0.0], [30.0, 7.0, 0.0], [40.0, 7.0, 0.0]]
            ),
        }
        roads = []
        lanelets = []
        for road_id, lines in ((1, from_lines), (2, to_lines)):
            group = []
            for inner, outer in ((0.0, 3.5), (3.5, 7.0)):
                lanelet = lanelet2.core.Lanelet(
                    lanelet2.core.getId(), lines[outer], lines[inner]
                )
                lanelet.attributes["subtype"] = "road"
                lanelet.attributes["one_way"] = "yes"
                lanelet_map.add(lanelet)
                group.append(lanelet)
            lanelets.extend(group)
            roads.append(
                Road.construct_from_lanelet_groups(
                    lanelet_map,
                    group,
                    road_id=road_id,
                    traffic_rule="LHT",
                )
            )
        from_road, to_road = roads
        from_road.add_successor(to_road.id, contact_point=ContactPoint.START)
        to_road.add_predecessor(from_road.id, contact_point=ContactPoint.END)
        for from_lane, to_lane in zip(
            sorted(
                from_road.lanes.lane_sections[0].left_lanes.values(),
                key=lambda lane: lane.lane_id,
            ),
            sorted(
                to_road.lanes.lane_sections[0].left_lanes.values(),
                key=lambda lane: lane.lane_id,
            ),
        ):
            from_lane.successor = LaneLink(id=to_lane.lane_id)
            to_lane.predecessor = LaneLink(id=from_lane.lane_id)
        return roads, {lanelet.id: lanelet for lanelet in lanelets}

    roads, lanelet_by_id = two_lane_roads(cap_shift=2.5)
    plans = build_ordinary_physical_connection_plans(roads, lanelet_by_id)
    assert len(plans) == 1
    assert plans[0].cross_section.heading == pytest.approx(0.0, abs=math.radians(0.2))
    assert plans[0].cross_section.lane_widths == pytest.approx((3.5, 3.5), abs=1e-6)


# --- Junction connector physical geometry (synthetic suite A-E) ---


def test_junction_geometry_a_multi_segment_blend_honors_override_c1() -> None:
    """A: an incoming-seam override too sharp for the terminal blend pair is
    distributed over leading segments without any artificial interior kink."""
    reference = np.array(
        [
            [0.0, 0.0, 0.0],
            [1.6, 0.0, 0.0],
            [2.4, 0.09, 0.0],
            [3.0, 0.18, 0.0],
            [5.0, 0.55, 0.0],
            [9.0, 1.5, 0.0],
        ],
        dtype=float,
    )
    lanelet_map, lanelet = _make_lanelet_from_reference(reference, width=3.5)
    override = -0.09  # ~5.2 deg below the first chord (0.0)
    context = RoadEmissionContext.from_lanelet_groups(
        lanelet_map,
        [lanelet],
        traffic_rule="RHT",
        start_heading_override=override,
    )
    assert context.evaluate(0.0).heading == pytest.approx(override, abs=1e-8)
    for station in context.emission_geometry.emission_stations[1:-1]:
        assert _heading_jump_at(context, float(station)) <= 0.5


def test_junction_geometry_b_absorbed_blend_stays_on_source_corridor() -> None:
    """B: when the per-segment blend folds (short anchored terminal
    segments), terminal points are absorbed into one bounded Bezier that
    stays on the source corridor."""
    reference = np.array(
        [
            [0.0, 0.0, 0.0],
            [4.0, 0.0, 0.0],
            [6.5, 0.0, 0.0],
            [8.0, 0.0, 0.0],
            [8.8, 0.0, 0.0],
            [9.4, 0.0, 0.0],
        ],
        dtype=float,
    )
    lanelet_map, lanelet = _make_lanelet_from_reference(reference, width=3.5)
    override = 0.13
    context = RoadEmissionContext.from_lanelet_groups(
        lanelet_map,
        [lanelet],
        traffic_rule="RHT",
        end_heading_override=override,
    )
    assert context.evaluate(context.length).heading == pytest.approx(
        override,
        abs=1e-8,
    )
    max_cut = DEFAULT_CONFIG.geometry.emission_interior_kink_max_corner_cut
    stations = np.linspace(0.0, context.length, 400)
    samples = np.array([context.evaluate(float(s)).xy for s in stations])
    for point in reference[:, :2]:
        assert float(np.min(np.linalg.norm(samples - point, axis=1))) <= (
            max_cut + 0.01
        )


def test_junction_geometry_c_minimax_compromise_heading() -> None:
    """C: a shared junction cross-section splits an infeasible tangent gap
    across the incoming road and its connectors within each fold budget."""
    from autoware_lanelet2_to_opendrive.physical_connection import (
        _minimax_compromise_heading,
    )

    incoming = (0.0, math.radians(4.0))
    straight_connector = (0.0, math.radians(3.8))
    turn_connector = (math.radians(7.5), math.radians(5.4))
    compromise = _minimax_compromise_heading(
        [incoming, straight_connector, turn_connector],
        0.0,
    )
    assert compromise is not None
    assert math.radians(2.1) <= compromise <= math.radians(3.8)
    # An impossible split (every budget smaller than the gap) is refused.
    assert (
        _minimax_compromise_heading(
            [(0.0, math.radians(1.0)), (math.radians(10.0), math.radians(1.0))],
            0.0,
        )
        is None
    )


def test_junction_geometry_d_tight_turn_boundary_never_folds() -> None:
    """D: a coarsely discretized tight turn emits fold-free offset lanes."""
    radius = 9.0
    angles = np.linspace(0.0, math.pi / 2.0, 10)
    reference = np.column_stack(
        [
            radius * np.sin(angles),
            radius * (1.0 - np.cos(angles)),
            np.zeros_like(angles),
        ]
    )
    lanelet_map, lanelet = _make_lanelet_from_reference(reference, width=3.5)
    context = RoadEmissionContext.from_lanelet_groups(
        lanelet_map,
        [lanelet],
        traffic_rule="LHT",
    )
    stations = np.linspace(0.0, context.length, 800)
    offset = 3.5  # left boundary of the single LHT lane
    previous = None
    for station in stations:
        pose = context.evaluate(float(station))
        normal = np.array(
            [-math.sin(pose.heading), math.cos(pose.heading)],
            dtype=float,
        )
        point = np.asarray(pose.xy, dtype=float) + offset * normal
        if previous is not None:
            step = point - previous
            forward = float(
                step[0] * math.cos(pose.heading) + step[1] * math.sin(pose.heading)
            )
            assert forward > -1e-6
        previous = point


def test_junction_geometry_e_isolated_true_corner_not_filleted() -> None:
    """E: an isolated sharp source corner survives even with a wide road."""
    kink = math.radians(15.0)
    direction = np.array([math.cos(kink), math.sin(kink)])
    first = np.array([[0.0, 0.0], [6.0, 0.0], [12.0, 0.0]])
    second = np.array([first[-1] + 6.0 * direction, first[-1] + 12.0 * direction])
    reference = np.column_stack([np.vstack([first, second]), np.zeros(5)])
    lanelet_map, lanelet = _make_lanelet_from_reference(reference, width=3.5)
    context = RoadEmissionContext.from_lanelet_groups(
        lanelet_map,
        [lanelet],
        traffic_rule="RHT",
    )
    assert _heading_jump_at(context, 12.0) == pytest.approx(15.0, abs=0.1)


# --- Consumer-compatibility regressions (zero-chord stub geometry) ---


def _zero_chord_stub_connector() -> tuple[Road, "_Lanelet2ToOpenDRIVEConverter"]:
    """A junction stub whose linked lane edges coincide (zero chord)."""
    lanelet_map = lanelet2.core.LaneletMap()
    pred_map, pred_lanelet = _make_lanelet_from_reference(
        np.array([[-10.0, 0.0, 0.0], [0.0, 0.0, 0.0]], dtype=float),
        width=4.0,
        lanelet_id=1301,
    )
    succ_map, succ_lanelet = _make_lanelet_from_reference(
        np.array([[0.0, 0.0, 0.0], [10.0, 0.0, 0.0]], dtype=float),
        width=4.0,
        lanelet_id=1302,
    )
    lanelet_map.add(pred_lanelet)
    lanelet_map.add(succ_lanelet)
    predecessor = Road.construct_from_lanelet_groups(
        pred_map, [pred_lanelet], road_id=10, traffic_rule="RHT"
    )
    successor = Road.construct_from_lanelet_groups(
        succ_map, [succ_lanelet], road_id=11, traffic_rule="RHT"
    )
    connector = _make_zero_length_connecting_road(
        road_id=12,
        junction_id=1000,
        incoming_road_id=predecessor.id,
        outgoing_road_id=successor.id,
        incoming_contact=ContactPoint.END,
        outgoing_contact=ContactPoint.START,
        start_xyz=(0.0, 0.0, 0.0),
        end_xyz=(0.0, 0.0, 0.0),
        min_segment_length=DEFAULT_CONFIG.geometry.divergence_min_segment_length,
        traffic_rule=TrafficRule.RHT,
        from_lane=-1,
        to_lane=-1,
        fallback_heading=0.0,
        lane_width=3.5,
    )
    converter = _Lanelet2ToOpenDRIVEConverter(
        lanelet_map,
        ConversionConfig(
            traffic_rule="RHT",
            emission_geometry=EmissionGeometryConfig(enabled=True),
        ),
    )
    roads_by_id = {road.id: road for road in (predecessor, successor, connector)}
    assert converter._align_unmapped_connecting_road(connector, roads_by_id)
    return connector, converter


def test_zero_chord_stub_emits_advancing_parampoly3() -> None:
    """A: the zero-chord stub is a valid straight ParamPoly3, not a point."""
    connector, _converter = _zero_chord_stub_connector()
    assert connector.plan_view is not None
    geometries = connector.plan_view.geometries
    assert len(geometries) == 1
    geometry = geometries[0]
    assert isinstance(geometry, ParamPoly3)
    assert geometry.pRange == "arcLength"
    # u(p) = p, v(p) = 0
    assert geometry.bU == pytest.approx(1.0)
    assert (geometry.aU, geometry.cU, geometry.dU) == pytest.approx((0.0, 0.0, 0.0))
    assert (geometry.aV, geometry.bV, geometry.cV, geometry.dV) == pytest.approx(
        (0.0, 0.0, 0.0, 0.0)
    )
    expected_length = DEFAULT_CONFIG.geometry.divergence_min_segment_length
    assert geometry.length == pytest.approx(expected_length)
    assert connector.length == pytest.approx(expected_length)

    coefficients = (
        geometry.aU,
        geometry.bU,
        geometry.cU,
        geometry.dU,
        geometry.aV,
        geometry.bV,
        geometry.cV,
        geometry.dV,
    )
    samples = np.array(
        [
            evaluate_plan_view_world(
                geometry.x,
                geometry.y,
                geometry.hdg,
                geometry.length * step / 32.0,
                coefficients,
                None,
            )
            for step in range(33)
        ]
    )
    steps = np.linalg.norm(np.diff(samples, axis=0), axis=1)
    # Integrated arc length matches the declared length, and no sample step
    # has a vanishing derivative.
    assert float(np.sum(steps)) == pytest.approx(expected_length, rel=1e-6)
    assert float(np.min(steps)) > 0.0
    # The endpoint advances exactly `length` along the geometry heading.
    direction = np.array([math.cos(geometry.hdg), math.sin(geometry.hdg)])
    displacement = samples[-1] - samples[0]
    assert float(np.dot(displacement, direction)) == pytest.approx(
        expected_length, rel=1e-9
    )
    assert abs(float(np.cross(direction, displacement))) <= 1e-12
