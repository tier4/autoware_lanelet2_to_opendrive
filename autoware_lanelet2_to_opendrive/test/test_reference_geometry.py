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
from autoware_lanelet2_to_opendrive.main import _Lanelet2ToOpenDRIVEConverter
from autoware_lanelet2_to_opendrive.opendrive.geometry import (
    Line,
    PlanView,
    evaluate_plan_view_world,
)
from autoware_lanelet2_to_opendrive.opendrive.lane_elements import LaneLink
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
from autoware_lanelet2_to_opendrive.spline import Splines
from autoware_lanelet2_to_opendrive.divergence import (
    DivergenceSide,
    DivergenceSite,
    SanityGateInputs,
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
        count = max(1, math.ceil(geometry.length / spacing))
        for i in range(count + 1):
            p = min(geometry.length, i * geometry.length / count)
            samples.append(
                evaluate_plan_view_world(
                    geometry.x,
                    geometry.y,
                    geometry.hdg,
                    p,
                    None,
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


def test_production_emission_gate_preserves_topology_when_stop_line_clamps() -> None:
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
