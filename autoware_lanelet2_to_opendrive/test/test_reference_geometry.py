"""Tests for topology/emission reference geometry separation."""

import math

import numpy as np
import pytest

from autoware_lanelet2_to_opendrive.opendrive.reference_geometry import (
    EmissionReferenceGeometry,
    StationMapping,
    TopologyReferenceGeometry,
    reproject_source_point_to_emission,
)
from autoware_lanelet2_to_opendrive.spline import Splines


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
