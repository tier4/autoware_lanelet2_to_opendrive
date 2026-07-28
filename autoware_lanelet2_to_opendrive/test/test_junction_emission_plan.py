"""Regression tests for junction-wide OpenDRIVE emission planning."""

from __future__ import annotations

import math
from types import SimpleNamespace
from typing import cast

import lanelet2
import numpy as np
import pytest

from autoware_lanelet2_to_opendrive.junction_emission_plan import (
    ConnectingRoadGroup,
    CutSection,
    EmittedLaneSegment,
    JunctionEmissionPlan,
    LaneTrace,
    LogicalLane,
    LogicalManeuver,
    SurfaceValidation,
    _CrossSection,
    _best_curve_candidate,
    _build_multi_lane_connector,
    _connector_lane_ids_for_contiguous_maneuvers,
    _emitted_lane_block_cross_section,
    _evaluate_lane_width_derivative,
    _single_lane_surface_is_valid,
    _source_boundaries_for_maneuvers,
    _trim_source_backed_road,
    _transform_width_derivatives_for_curvature,
    apply_planned_topology_links,
    build_emitted_traceability,
    repair_invalid_sibling_connecting_road_surfaces,
    search_junction_cutback,
)
from autoware_lanelet2_to_opendrive.opendrive.enums import (
    ElementType,
    TrafficRule,
)
from autoware_lanelet2_to_opendrive.opendrive.geometry import (
    ParamPoly3,
    evaluate_plan_view_world,
)
from autoware_lanelet2_to_opendrive.opendrive.road import _evaluate_lane_width
from autoware_lanelet2_to_opendrive.opendrive.road import Road
from autoware_lanelet2_to_opendrive.opendrive.reference_geometry import (
    EmissionReferenceGeometry,
    RoadEmissionContext,
    StationMapping,
    TopologyReferenceGeometry,
)


def _parallel_maneuvers(
    count: int,
    *,
    side: int = 1,
) -> tuple[LogicalManeuver, ...]:
    return tuple(
        LogicalManeuver(
            incoming=LogicalLane(
                lanelet_id=100 + index,
                road_id=10,
                lane_id=side * index,
                subtype="road",
            ),
            outgoing=LogicalLane(
                lanelet_id=200 + index,
                road_id=20,
                lane_id=side * index,
                subtype="road",
            ),
        )
        for index in range(1, count + 1)
    )


@pytest.mark.parametrize(
    ("side", "traffic_rule"),
    [(1, TrafficRule.LHT), (-1, TrafficRule.RHT)],
)
def test_multi_lane_connector_preserves_lane_side_links_and_widths(
    side: int,
    traffic_rule: TrafficRule,
) -> None:
    maneuvers = _parallel_maneuvers(2, side=side)
    connector = _build_multi_lane_connector(
        road_id=30,
        junction_id=40,
        incoming_road_id=10,
        outgoing_road_id=20,
        maneuvers=maneuvers,
        connector_lane_ids=(side, 2 * side),
        curve_control_points_xyz=np.array(
            [
                [0.0, 0.0, 0.0],
                [1.3, 0.0, 0.0],
                [2.7, 0.2, 0.0],
                [4.0, 0.2, 0.0],
            ]
        ),
        start_widths=(3.2, 3.4),
        end_widths=(3.6, 3.8),
        start_width_derivatives=(0.08, -0.04),
        end_width_derivatives=(-0.03, 0.02),
        traffic_rule=traffic_rule,
    )

    assert connector.lanes is not None
    section = connector.lanes.lane_sections[0]
    lanes = section.left_lanes if side > 0 else section.right_lanes
    assert set(lanes) == {side, 2 * side}
    for index, lane_id in enumerate((side, 2 * side)):
        lane = lanes[lane_id]
        assert lane.predecessor is not None
        assert lane.successor is not None
        assert lane.predecessor.id == lane_id
        assert lane.successor.id == lane_id
        assert _evaluate_lane_width(lane, 0.0) == pytest.approx((3.2, 3.4)[index])
        assert _evaluate_lane_width(lane, connector.length) == pytest.approx(
            (3.6, 3.8)[index]
        )
        assert _evaluate_lane_width_derivative(lane, 0.0) == pytest.approx(
            (0.08, -0.04)[index]
        )
        assert _evaluate_lane_width_derivative(lane, connector.length) == pytest.approx(
            (-0.03, 0.02)[index]
        )
    assert connector.length > 0.0
    assert connector.plan_view is not None
    assert len(connector.plan_view.geometries) == 1
    geometry = connector.plan_view.geometries[0]
    assert isinstance(geometry, ParamPoly3)
    assert geometry.pRange == "arcLength"
    parameters = np.linspace(0.0, geometry.length, 4001)
    du = (
        geometry.bU
        + 2.0 * geometry.cU * parameters
        + 3.0 * geometry.dU * (parameters**2)
    )
    dv = (
        geometry.bV
        + 2.0 * geometry.cV * parameters
        + 3.0 * geometry.dV * (parameters**2)
    )
    integrated_length = float(np.trapezoid(np.sqrt(du * du + dv * dv), parameters))
    assert integrated_length == pytest.approx(geometry.length, rel=1e-7)
    endpoint = evaluate_plan_view_world(
        geometry.x,
        geometry.y,
        geometry.hdg,
        geometry.length,
        (
            geometry.aU,
            geometry.bU,
            geometry.cU,
            geometry.dU,
            geometry.aV,
            geometry.bV,
            geometry.cV,
            geometry.dV,
        ),
    )
    assert endpoint == pytest.approx((4.0, 0.2))


@pytest.mark.parametrize(
    ("side", "traffic_rule"),
    [(1, TrafficRule.LHT), (-1, TrafficRule.RHT)],
)
def test_contiguous_lane_id_shift_uses_canonical_multi_lane_connector(
    side: int,
    traffic_rule: TrafficRule,
) -> None:
    maneuvers = tuple(
        LogicalManeuver(
            incoming=LogicalLane(
                lanelet_id=100 + index,
                road_id=10,
                lane_id=side * index,
                subtype="road",
            ),
            outgoing=LogicalLane(
                lanelet_id=200 + index,
                road_id=20,
                lane_id=side * (index + 1),
                subtype="road",
            ),
        )
        for index in range(1, 3)
    )
    connector_lane_ids = _connector_lane_ids_for_contiguous_maneuvers(
        [maneuver.incoming.lane_id for maneuver in maneuvers],
        [maneuver.outgoing.lane_id for maneuver in maneuvers],
    )

    assert connector_lane_ids == (side, 2 * side)
    connector = _build_multi_lane_connector(
        road_id=30,
        junction_id=40,
        incoming_road_id=10,
        outgoing_road_id=20,
        maneuvers=maneuvers,
        connector_lane_ids=connector_lane_ids,
        curve_control_points_xyz=np.array(
            [
                [0.0, 0.0, 0.0],
                [1.3, 0.0, 0.0],
                [2.7, 0.2, 0.0],
                [4.0, 0.2, 0.0],
            ]
        ),
        start_widths=(3.2, 3.4),
        end_widths=(3.1, 3.3),
        start_lane_offset=0.0,
        end_lane_offset=side * 2.5,
        traffic_rule=traffic_rule,
    )

    assert connector.lanes is not None
    section = connector.lanes.lane_sections[0]
    assert len(connector.lanes.lane_offsets) == 1
    lane_offset = connector.lanes.lane_offsets[0]
    assert lane_offset["a"] == pytest.approx(0.0)
    offset_length = connector.length
    assert (
        lane_offset["a"]
        + lane_offset["b"] * offset_length
        + lane_offset["c"] * offset_length**2
        + lane_offset["d"] * offset_length**3
    ) == pytest.approx(side * 2.5)
    xml = connector.to_xml()
    assert xml.find("lanes/laneOffset") is not None
    assert xml.find("lanes/laneSection/laneOffset") is None
    lanes = section.left_lanes if side > 0 else section.right_lanes
    inner_predecessor = lanes[side].predecessor
    inner_successor = lanes[side].successor
    outer_predecessor = lanes[2 * side].predecessor
    outer_successor = lanes[2 * side].successor
    assert inner_predecessor is not None
    assert inner_successor is not None
    assert outer_predecessor is not None
    assert outer_successor is not None
    assert inner_predecessor.id == side
    assert inner_successor.id == 2 * side
    assert outer_predecessor.id == 2 * side
    assert outer_successor.id == 3 * side


@pytest.mark.parametrize("side", [1, -1])
def test_shifted_contiguous_lane_block_uses_its_inner_boundary_as_reference(
    side: int,
) -> None:
    boundaries = [
        lanelet2.core.LineString3d(
            lanelet2.core.getId(),
            [
                lanelet2.core.Point3d(lanelet2.core.getId(), 0.0, side * offset, 0.0),
                lanelet2.core.Point3d(lanelet2.core.getId(), 10.0, side * offset, 0.0),
            ],
        )
        for offset in (0.0, 3.0, 6.0, 9.0)
    ]
    lanelets = {}
    for index in range(1, 4):
        inner = boundaries[index - 1]
        outer = boundaries[index]
        lanelet = lanelet2.core.Lanelet(
            200 + index,
            outer if side > 0 else inner,
            inner if side > 0 else outer,
        )
        lanelets[lanelet.id] = lanelet
    maneuvers = tuple(
        LogicalManeuver(
            incoming=LogicalLane(
                lanelet_id=100 + index,
                road_id=10,
                lane_id=side * (index - 1),
                subtype="road",
            ),
            outgoing=LogicalLane(
                lanelet_id=200 + index,
                road_id=20,
                lane_id=side * index,
                subtype="road",
            ),
        )
        for index in (2, 3)
    )

    result = _source_boundaries_for_maneuvers(
        maneuvers,
        lanelets,
        incoming=False,
    )

    assert result is not None
    selected, lane_side = result
    assert lane_side == side
    assert len(selected) == 3
    assert selected[0][:, 1] == pytest.approx([side * 3.0, side * 3.0])
    assert selected[1][:, 1] == pytest.approx([side * 6.0, side * 6.0])
    assert selected[2][:, 1] == pytest.approx([side * 9.0, side * 9.0])


@pytest.mark.parametrize(
    ("side", "traffic_rule"),
    [(1, TrafficRule.LHT), (-1, TrafficRule.RHT)],
)
def test_emitted_shifted_lane_block_uses_rendered_inner_boundary(
    side: int,
    traffic_rule: TrafficRule,
) -> None:
    maneuvers = _parallel_maneuvers(3, side=side)
    road = _build_multi_lane_connector(
        road_id=20,
        junction_id=-1,
        incoming_road_id=10,
        outgoing_road_id=30,
        maneuvers=maneuvers,
        connector_lane_ids=(side, 2 * side, 3 * side),
        curve_control_points_xyz=np.array(
            [
                [0.0, 0.0, 1.0],
                [10.0 / 3.0, 0.0, 1.0],
                [20.0 / 3.0, 0.0, 1.0],
                [10.0, 0.0, 1.0],
            ]
        ),
        start_widths=(2.5, 3.0, 3.5),
        end_widths=(2.5, 3.0, 3.5),
        traffic_rule=traffic_rule,
    )

    cross_section = _emitted_lane_block_cross_section(
        road,
        (2 * side, 3 * side),
        at_start=True,
    )

    assert cross_section.reference_xyz == pytest.approx([0.0, side * 2.5, 1.0])
    assert cross_section.tangent == pytest.approx([1.0, 0.0])
    assert cross_section.widths == pytest.approx([3.0, 3.5])
    assert cross_section.lane_side == side
    assert cross_section.lane_offset == pytest.approx(0.0)
    assert cross_section.lane_offset_derivative == pytest.approx(0.0)
    assert cross_section.source_lane_offset == pytest.approx(side * 2.5)
    assert cross_section.source_lane_offset_derivative == pytest.approx(0.0)


@pytest.mark.parametrize("side", [1, -1])
def test_shifted_lane_offset_curve_has_valid_finite_width_surface(side: int) -> None:
    start = _CrossSection(
        reference_xyz=np.array([0.0, 0.0, 0.0]),
        tangent=np.array([1.0, 0.0]),
        widths=np.array([3.0, 3.5]),
        lane_side=side,
    )
    end = _CrossSection(
        reference_xyz=np.array([4.0, side * 0.2, 0.0]),
        tangent=np.array([1.0, 0.0]),
        widths=np.array([3.0, 3.5]),
        lane_side=side,
    )

    curve = _best_curve_candidate(start, end)

    assert curve is not None
    assert curve.validation.c1
    assert curve.validation.finite_width_valid
    assert not curve.validation.boundary_self_intersection
    assert not curve.validation.boundary_reversal


def test_short_backward_chord_requires_cutback_and_produces_valid_surface() -> None:
    angle = math.radians(20.0)
    incoming_tangent = np.array([1.0, 0.0])
    outgoing_tangent = np.array([math.cos(angle), math.sin(angle)])
    incoming_normal = np.array([0.0, 1.0])
    outgoing_normal = np.array([-math.sin(angle), math.cos(angle)])
    incoming_endpoint = 7.0 * incoming_normal
    outgoing_endpoint = 7.0 * outgoing_normal
    widths = np.array([3.5, 3.5])

    def incoming(distance: float) -> _CrossSection:
        point = incoming_endpoint - distance * incoming_tangent
        return _CrossSection(
            np.array([point[0], point[1], 0.0]),
            incoming_tangent,
            widths,
        )

    def outgoing(distance: float) -> _CrossSection:
        point = outgoing_endpoint + distance * outgoing_tangent
        return _CrossSection(
            np.array([point[0], point[1], 0.0]),
            outgoing_tangent,
            widths,
        )

    initial_displacement = (
        outgoing(0.0).reference_xyz[:2] - incoming(0.0).reference_xyz[:2]
    )
    assert float(np.dot(initial_displacement, incoming_tangent)) < 0.0
    assert float(np.dot(initial_displacement, outgoing_tangent)) < 0.0

    result = search_junction_cutback(incoming, outgoing)

    assert result is not None
    incoming_cut, outgoing_cut, curve, start, end = result
    displacement = end.reference_xyz[:2] - start.reference_xyz[:2]
    assert incoming_cut > 0.0
    assert outgoing_cut > 0.0
    assert float(np.dot(displacement, start.tangent)) > 0.0
    assert float(np.dot(displacement, end.tangent)) > 0.0
    assert np.allclose(curve.points_xyz[0], start.reference_xyz)
    assert np.allclose(curve.points_xyz[-1], end.reference_xyz)
    assert curve.validation.c0
    assert curve.validation.c1
    assert curve.validation.finite_width_valid
    assert not curve.validation.center_self_intersection
    assert not curve.validation.boundary_self_intersection
    assert not curve.validation.boundary_reversal
    assert curve.validation.max_kappa_half_width < 1.0


def test_cutback_skips_terminal_taper_with_discontinuous_lane_center_heading() -> None:
    widths = np.array([3.0, 3.0])

    def incoming(distance: float) -> _CrossSection:
        in_terminal_taper = distance < 0.5
        return _CrossSection(
            reference_xyz=np.array([-distance, 0.0, 0.0]),
            tangent=np.array([1.0, 0.0]),
            widths=widths,
            width_derivatives=np.array(
                [-3.0, 0.0] if in_terminal_taper else [0.0, 0.0]
            ),
            source_curvature=0.1 if in_terminal_taper else 0.0,
        )

    def outgoing(distance: float) -> _CrossSection:
        return _CrossSection(
            reference_xyz=np.array([1.0 + distance, 0.0, 0.0]),
            tangent=np.array([1.0, 0.0]),
            widths=widths,
            width_derivatives=np.zeros(2),
        )

    result = search_junction_cutback(incoming, outgoing)

    assert result is not None
    incoming_cut, outgoing_cut, curve, _start, _end = result
    assert incoming_cut == pytest.approx(0.5)
    assert outgoing_cut == pytest.approx(0.0)
    assert curve.validation.finite_width_valid
    assert curve.validation.c1


def test_source_road_trim_preserves_untrimmed_endpoint_curve() -> None:
    points = np.asarray(
        [
            [0.0, 0.0, 0.0],
            [3.0, 0.0, 0.0],
            [6.0, 0.2, 0.0],
            [9.0, 0.4, 0.0],
            [12.0, 0.8, 0.0],
            [15.0, 1.5, 0.0],
        ]
    )
    end_heading = math.atan2(0.5, 3.0)
    emission = EmissionReferenceGeometry(
        points,
        end_heading=end_heading,
    )
    assert emission._bezier_segments
    context = RoadEmissionContext(
        topology_geometry=cast(
            TopologyReferenceGeometry,
            SimpleNamespace(length=emission.length),
        ),
        emission_geometry=emission,
        station_mapping=StationMapping.from_lengths(
            emission.length,
            emission.source_length,
            emission.length,
        ),
    )
    road = _single_lane_connector(
        road_id=10,
        junction_id=-1,
        incoming_road_id=1,
        outgoing_road_id=2,
        start=(0.0, 0.0),
        end=(15.0, 1.5),
        control_points_xy=((5.0, 0.0), (10.0, 0.5)),
        lanelet_id=100,
    )
    road.emission_context = context
    road.plan_view = context.to_plan_view()
    road.elevation_profile = context.to_elevation_profile()
    road.length = context.length
    original_end_heading = context.evaluate(context.length).heading

    _trim_source_backed_road(road, start_trim=0.5, end_trim=0.0)

    assert road.plan_view is not None
    assert any(
        isinstance(geometry, ParamPoly3) for geometry in road.plan_view.geometries
    )
    assert road.emission_context is not None
    assert road.emission_context.evaluate(road.length).heading == pytest.approx(
        original_end_heading
    )


def test_width_derivative_transform_preserves_offset_boundary_heading() -> None:
    widths = [3.5, 3.0]
    source_derivatives = [0.08, -0.03]
    source_curvature = 0.01
    target_curvature = 0.05
    source_speed = 1.2
    target_speed = 0.8

    transformed = _transform_width_derivatives_for_curvature(
        widths,
        source_derivatives,
        lane_side=1,
        source_curvature=source_curvature,
        target_curvature=target_curvature,
        source_speed=source_speed,
        target_speed=target_speed,
    )

    for offset, source_derivative, target_derivative in zip(
        np.cumsum(widths),
        np.cumsum(source_derivatives),
        np.cumsum(transformed),
    ):
        source_heading = math.atan2(
            source_derivative,
            source_speed * (1.0 - source_curvature * offset),
        )
        target_heading = math.atan2(
            target_derivative,
            target_speed * (1.0 - target_curvature * offset),
        )
        assert target_heading == pytest.approx(source_heading)


def _group_for(
    maneuvers: tuple[LogicalManeuver, ...],
) -> ConnectingRoadGroup:
    return ConnectingRoadGroup(
        incoming_road_id=10,
        outgoing_road_id=20,
        connector_road_id=30,
        maneuvers=maneuvers,
        connector_lane_ids=tuple(maneuver.incoming.lane_id for maneuver in maneuvers),
        replaced_connector_road_ids=(30, 31),
        incoming_cut=CutSection(
            road_id=10,
            station_from_boundary=0.8,
            reference_xyz=(0.0, 0.0, 0.0),
            heading=0.0,
            lane_widths=tuple(3.5 for _ in maneuvers),
        ),
        outgoing_cut=CutSection(
            road_id=20,
            station_from_boundary=1.5,
            reference_xyz=(2.3, 0.0, 0.0),
            heading=0.0,
            lane_widths=tuple(3.5 for _ in maneuvers),
        ),
        curve_points_xyz=np.array([[0.0, 0.0, 0.0], [2.3, 0.0, 0.0]]),
        curve_control_points_xyz=np.array(
            [
                [0.0, 0.0, 0.0],
                [2.3 / 3.0, 0.0, 0.0],
                [2.0 * 2.3 / 3.0, 0.0, 0.0],
                [2.3, 0.0, 0.0],
            ]
        ),
        surface=SurfaceValidation(
            c0=True,
            c1=True,
            finite_width_valid=True,
            center_self_intersection=False,
            boundary_self_intersection=False,
            boundary_reversal=False,
            max_abs_curvature=0.0,
            max_kappa_half_width=0.0,
            reference_length=2.3,
            max_internal_heading_change=0.0,
        ),
    )


def test_multi_lane_continuation_with_branch_preserves_semantic_invariant() -> None:
    straight = _parallel_maneuvers(3)
    branch = LogicalManeuver(
        incoming=straight[0].incoming,
        outgoing=LogicalLane(
            lanelet_id=204,
            road_id=25,
            lane_id=1,
            subtype="road",
        ),
    )
    logical = (*straight, branch)
    incoming = tuple(dict.fromkeys(maneuver.incoming for maneuver in logical))
    outgoing = tuple(dict.fromkeys(maneuver.outgoing for maneuver in logical))
    allowed = {maneuver.lanelet_pair for maneuver in logical}
    forbidden = tuple(
        sorted(
            {
                (incoming_lane.lanelet_id, outgoing_lane.lanelet_id)
                for incoming_lane in incoming
                for outgoing_lane in outgoing
            }
            - allowed
        )
    )
    group = _group_for(straight)
    plan = JunctionEmissionPlan(
        junction_id=50,
        logical_incoming_lanes=incoming,
        logical_outgoing_lanes=outgoing,
        logical_maneuvers=logical,
        forbidden_maneuvers=forbidden,
        connecting_road_groups=(group,),
        lane_traces=(),
    )

    assert len(plan.logical_maneuvers) == 4
    assert len(plan.forbidden_maneuvers) == 8
    assert plan.missing_maneuvers == ()
    assert plan.unintended_maneuvers == ()
    assert len(plan.connecting_road_groups) == 1
    assert len(plan.connecting_road_groups[0].lane_links) == 3


def _single_lane_connector(
    *,
    road_id: int,
    junction_id: int,
    incoming_road_id: int,
    outgoing_road_id: int,
    start: tuple[float, float],
    end: tuple[float, float],
    control_points_xy: tuple[tuple[float, float], tuple[float, float]],
    lanelet_id: int,
) -> Road:
    maneuver = LogicalManeuver(
        incoming=LogicalLane(1, incoming_road_id, 1, "road"),
        outgoing=LogicalLane(lanelet_id, outgoing_road_id, 1, "road"),
    )
    road = _build_multi_lane_connector(
        road_id=road_id,
        junction_id=junction_id,
        incoming_road_id=incoming_road_id,
        outgoing_road_id=outgoing_road_id,
        maneuvers=(maneuver,),
        connector_lane_ids=(1,),
        curve_control_points_xyz=np.asarray(
            [
                (*start, 0.0),
                (*control_points_xy[0], 0.0),
                (*control_points_xy[1], 0.0),
                (*end, 0.0),
            ],
            dtype=float,
        ),
        start_widths=(3.5,),
        end_widths=(3.5,),
        traffic_rule=TrafficRule.LHT,
    )
    assert road.lanes is not None
    road.lanes.lane_sections[0].left_lanes[1].lanelet_id = lanelet_id
    return road


def test_invalid_source_backed_sibling_connector_is_rebuilt_atomically() -> None:
    straight = _parallel_maneuvers(2)
    incoming = _build_multi_lane_connector(
        road_id=10,
        junction_id=-1,
        incoming_road_id=1,
        outgoing_road_id=20,
        maneuvers=straight,
        connector_lane_ids=(1, 2),
        curve_control_points_xyz=np.asarray(
            [[-5.0, 0.0, 0.0], [-3.0, 0.0, 0.0], [-2.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
        ),
        start_widths=(3.5, 3.5),
        end_widths=(3.5, 3.5),
        traffic_rule=TrafficRule.LHT,
    )
    outgoing = _single_lane_connector(
        road_id=25,
        junction_id=-1,
        incoming_road_id=31,
        outgoing_road_id=2,
        start=(8.0, 2.0),
        end=(13.0, 2.0),
        control_points_xy=((10.0, 2.0), (11.0, 2.0)),
        lanelet_id=400,
    )
    sibling = _single_lane_connector(
        road_id=31,
        junction_id=50,
        incoming_road_id=10,
        outgoing_road_id=25,
        start=(0.0, 0.0),
        end=(8.0, 2.0),
        control_points_xy=((0.2, 6.0), (4.0, -6.0)),
        lanelet_id=300,
    )
    assert sibling.lanes is not None
    sibling_lane = sibling.lanes.lane_sections[0].left_lanes[1]
    assert not _single_lane_surface_is_valid(sibling, sibling_lane)

    branch = LogicalManeuver(
        incoming=straight[0].incoming,
        outgoing=LogicalLane(300, 31, 1, "road"),
    )
    plan = JunctionEmissionPlan(
        junction_id=50,
        logical_incoming_lanes=tuple(
            dict.fromkeys(maneuver.incoming for maneuver in (*straight, branch))
        ),
        logical_outgoing_lanes=tuple(
            dict.fromkeys(maneuver.outgoing for maneuver in (*straight, branch))
        ),
        logical_maneuvers=(*straight, branch),
        forbidden_maneuvers=(),
        connecting_road_groups=(_group_for(straight),),
        lane_traces=(),
    )
    roads = [incoming, outgoing, sibling]

    repaired = repair_invalid_sibling_connecting_road_surfaces((plan,), roads)

    assert repaired == (31,)
    replacement = next(road for road in roads if road.id == 31)
    assert replacement.link == sibling.link
    assert replacement.junction == sibling.junction
    assert replacement.lanes is not None
    replacement_lane = replacement.lanes.lane_sections[0].left_lanes[1]
    assert replacement_lane.lanelet_id == 300
    assert replacement_lane.predecessor == sibling_lane.predecessor
    assert replacement_lane.successor == sibling_lane.successor
    assert _single_lane_surface_is_valid(replacement, replacement_lane)
    assert replacement.reference_start_xyz == pytest.approx((0.0, 0.0, 0.0))
    assert replacement.reference_end_xyz == pytest.approx((8.0, 2.0, 0.0))


def test_valid_source_backed_sibling_connector_is_not_changed() -> None:
    straight = _parallel_maneuvers(2)
    incoming = _build_multi_lane_connector(
        road_id=10,
        junction_id=-1,
        incoming_road_id=1,
        outgoing_road_id=20,
        maneuvers=straight,
        connector_lane_ids=(1, 2),
        curve_control_points_xyz=np.asarray(
            [[-5.0, 0.0, 0.0], [-3.0, 0.0, 0.0], [-2.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
        ),
        start_widths=(3.5, 3.5),
        end_widths=(3.5, 3.5),
        traffic_rule=TrafficRule.LHT,
    )
    outgoing = _single_lane_connector(
        road_id=25,
        junction_id=-1,
        incoming_road_id=31,
        outgoing_road_id=2,
        start=(8.0, 2.0),
        end=(13.0, 2.0),
        control_points_xy=((10.0, 2.0), (11.0, 2.0)),
        lanelet_id=400,
    )
    sibling = _single_lane_connector(
        road_id=31,
        junction_id=50,
        incoming_road_id=10,
        outgoing_road_id=25,
        start=(0.0, 0.0),
        end=(8.0, 2.0),
        control_points_xy=((2.7, 0.7), (5.3, 1.3)),
        lanelet_id=300,
    )
    assert sibling.lanes is not None
    sibling_lane = sibling.lanes.lane_sections[0].left_lanes[1]
    assert _single_lane_surface_is_valid(sibling, sibling_lane)
    branch = LogicalManeuver(
        incoming=straight[0].incoming,
        outgoing=LogicalLane(300, 31, 1, "road"),
    )
    plan = JunctionEmissionPlan(
        junction_id=50,
        logical_incoming_lanes=(),
        logical_outgoing_lanes=(),
        logical_maneuvers=(*straight, branch),
        forbidden_maneuvers=(),
        connecting_road_groups=(_group_for(straight),),
        lane_traces=(),
    )
    roads = [incoming, outgoing, sibling]

    repaired = repair_invalid_sibling_connecting_road_surfaces((plan,), roads)

    assert repaired == ()
    assert next(road for road in roads if road.id == 31) is sibling


def test_planned_topology_and_traceability_use_one_multi_lane_connector() -> None:
    maneuvers = _parallel_maneuvers(2)
    incoming = _build_multi_lane_connector(
        road_id=10,
        junction_id=-1,
        incoming_road_id=1,
        outgoing_road_id=20,
        maneuvers=maneuvers,
        connector_lane_ids=(1, 2),
        curve_control_points_xyz=np.array(
            [
                [-4.0, 0.0, 0.0],
                [-8.0 / 3.0, 0.0, 0.0],
                [-4.0 / 3.0, 0.0, 0.0],
                [0.0, 0.0, 0.0],
            ]
        ),
        start_widths=(3.5, 3.5),
        end_widths=(3.5, 3.5),
        traffic_rule=TrafficRule.LHT,
    )
    outgoing = _build_multi_lane_connector(
        road_id=20,
        junction_id=-1,
        incoming_road_id=10,
        outgoing_road_id=2,
        maneuvers=maneuvers,
        connector_lane_ids=(1, 2),
        curve_control_points_xyz=np.array(
            [
                [2.0, 0.0, 0.0],
                [10.0 / 3.0, 0.0, 0.0],
                [14.0 / 3.0, 0.0, 0.0],
                [6.0, 0.0, 0.0],
            ]
        ),
        start_widths=(3.5, 3.5),
        end_widths=(3.5, 3.5),
        traffic_rule=TrafficRule.LHT,
    )
    group = _group_for(maneuvers)
    connector = _build_multi_lane_connector(
        road_id=30,
        junction_id=50,
        incoming_road_id=10,
        outgoing_road_id=20,
        maneuvers=maneuvers,
        connector_lane_ids=(1, 2),
        curve_control_points_xyz=group.curve_control_points_xyz,
        start_widths=(3.5, 3.5),
        end_widths=(3.5, 3.5),
        traffic_rule=TrafficRule.LHT,
    )
    plan = JunctionEmissionPlan(
        junction_id=50,
        logical_incoming_lanes=tuple(maneuver.incoming for maneuver in maneuvers),
        logical_outgoing_lanes=tuple(maneuver.outgoing for maneuver in maneuvers),
        logical_maneuvers=maneuvers,
        forbidden_maneuvers=(),
        connecting_road_groups=(group,),
        lane_traces=tuple(
            LaneTrace(
                lanelet_id=maneuver.incoming.lanelet_id,
                emitted_segments=(
                    EmittedLaneSegment(10, lane_id, "source"),
                    EmittedLaneSegment(30, lane_id, "junction_connector"),
                ),
            )
            for maneuver, lane_id in zip(maneuvers, (1, 2))
        ),
    )

    apply_planned_topology_links(
        (plan,),
        (incoming, outgoing, connector),
    )
    trace = build_emitted_traceability(
        {
            maneuver.incoming.lanelet_id: (10, maneuver.incoming.lane_id)
            for maneuver in maneuvers
        },
        (plan,),
    )

    assert incoming.link is not None
    assert incoming.link.successor is not None
    assert incoming.link.successor.element_type is ElementType.JUNCTION
    assert incoming.link.successor.element_id == 50
    assert outgoing.link is not None
    assert outgoing.link.predecessor is not None
    assert outgoing.link.predecessor.element_type is ElementType.JUNCTION
    assert outgoing.link.predecessor.element_id == 50
    assert incoming.lanes is not None
    assert connector.lanes is not None
    assert outgoing.lanes is not None
    for lane_id in (1, 2):
        incoming_lane = incoming.lanes.lane_sections[0].left_lanes[lane_id]
        assert incoming_lane.successor is not None
        assert incoming_lane.successor.id == lane_id
        connector_lane = connector.lanes.lane_sections[0].left_lanes[lane_id]
        assert connector_lane.predecessor is not None
        assert connector_lane.successor is not None
        assert connector_lane.predecessor.id == lane_id
        assert connector_lane.successor.id == lane_id
        outgoing_lane = outgoing.lanes.lane_sections[0].left_lanes[lane_id]
        assert outgoing_lane.predecessor is not None
        assert outgoing_lane.predecessor.id == lane_id
    assert [
        (segment["road_id"], segment["lane_id"], segment["role"])
        for segment in trace[101]
    ] == [(10, 1, "source"), (30, 1, "junction_connector")]
