"""Tests for divergence/merge synthesis (issue #291)."""

import inspect
import logging
import math
from unittest.mock import MagicMock

import pytest

from autoware_lanelet2_to_opendrive.config import DEFAULT_CONFIG
from autoware_lanelet2_to_opendrive.divergence import (
    DivergenceSide,
    DivergenceSite,
    DivergenceSynthesisResult,
    SanityGateInputs,
    SynthesisOutput,
    _directed_edges_for_site,
    _make_zero_length_connecting_road,
    _synthesise_junction_for_edges,
    apply_divergence_synthesis,
    collect_divergence_sites,
    sanity_gate_passes,
    synthesise_junction_for_site,
)
from autoware_lanelet2_to_opendrive.opendrive.enums import (
    ContactPoint,
    ElementType,
    LaneType,
    TrafficRule,
)
from autoware_lanelet2_to_opendrive.opendrive.geometry import ParamPoly3, PlanView
from autoware_lanelet2_to_opendrive.opendrive.lane import Lane
from autoware_lanelet2_to_opendrive.opendrive.lane_elements import LaneWidth
from autoware_lanelet2_to_opendrive.opendrive.lane_section import LaneSection
from autoware_lanelet2_to_opendrive.opendrive.lane_sections import Lanes
from autoware_lanelet2_to_opendrive.opendrive.road import (
    Road,
    _resolve_candidate_road_ids,
)


def test_geometry_constants_expose_divergence_thresholds():
    """Sanity gate and epsilon-floor live on GeometryConstants."""
    geom = DEFAULT_CONFIG.geometry
    assert geom.divergence_endpoint_tolerance == 0.5
    assert geom.divergence_min_segment_length == 0.01


class _StubLanelet:
    def __init__(self, lanelet_id: int, has_turn_direction: bool = False):
        self.id = lanelet_id
        self.attributes = {"turn_direction": "left"} if has_turn_direction else {}


def test_resolve_candidate_road_ids_returns_all_distinct_regular_road_ids():
    groups = [{_StubLanelet(10), _StubLanelet(11)}, {_StubLanelet(20)}]
    mapping = {10: 1, 11: 1, 20: 2}

    result = _resolve_candidate_road_ids(groups, mapping)

    assert result == [1, 2]


def test_resolve_candidate_road_ids_returns_empty_when_any_group_has_turn_direction():
    groups = [
        {_StubLanelet(10)},
        {_StubLanelet(99, has_turn_direction=True)},
    ]
    mapping = {10: 1, 99: 5}

    # When any group is a real-junction lanelet group, defer to the existing
    # turn_direction junction pipeline by returning an empty list.
    assert _resolve_candidate_road_ids(groups, mapping) == []


def test_resolve_candidate_road_ids_preserves_order_of_first_appearance():
    groups = [{_StubLanelet(20)}, {_StubLanelet(10), _StubLanelet(11)}]
    mapping = {10: 1, 11: 1, 20: 2}

    assert _resolve_candidate_road_ids(groups, mapping) == [2, 1]


def test_construct_from_lanelet_map_returns_deferred_candidate_dicts():
    """The signature must include deferred predecessor/successor candidate maps."""
    sig = inspect.signature(Road.construct_from_lanelet_map)
    return_annotation = str(sig.return_annotation).replace(" ", "")
    assert (
        "ConstructedRoadsResult" in return_annotation
        or "Tuple[List[" in return_annotation
    )
    docstring = Road.construct_from_lanelet_map.__doc__ or ""
    assert "deferred" in docstring.lower()


def test_divergence_site_records_side_and_candidates():
    site = DivergenceSite(
        road_id=185,
        side=DivergenceSide.SUCCESSOR,
        candidate_road_ids=[186, 187, 188],
    )
    assert site.road_id == 185
    assert site.side is DivergenceSide.SUCCESSOR
    assert site.candidate_road_ids == [186, 187, 188]
    assert site.is_divergence is True  # successor side with N>=2 candidates


def test_collect_divergence_sites_emits_one_site_per_deferred_entry():
    sites = collect_divergence_sites(
        deferred_predecessor_candidates={11: [1, 2]},
        deferred_successor_candidates={185: [186, 187, 188]},
    )

    sides = {(s.road_id, s.side, tuple(s.candidate_road_ids)) for s in sites}
    assert sides == {
        (11, DivergenceSide.PREDECESSOR, (1, 2)),
        (185, DivergenceSide.SUCCESSOR, (186, 187, 188)),
    }


def test_collect_divergence_sites_skips_singleton_or_empty_lists():
    sites = collect_divergence_sites(
        deferred_predecessor_candidates={5: [9], 6: []},
        deferred_successor_candidates={},
    )
    assert sites == []


def test_collect_divergence_sites_handles_road_with_both_sides_deferred():
    sites = collect_divergence_sites(
        deferred_predecessor_candidates={42: [1, 2]},
        deferred_successor_candidates={42: [3, 4, 5]},
    )

    sides = {s.side for s in sites}
    assert sides == {DivergenceSide.PREDECESSOR, DivergenceSide.SUCCESSOR}
    assert all(s.road_id == 42 for s in sites)


def _site():
    return DivergenceSite(
        road_id=185,
        side=DivergenceSide.SUCCESSOR,
        candidate_road_ids=[186, 187, 188],
    )


def test_sanity_gate_passes_when_all_three_checks_clear():
    inputs = SanityGateInputs(
        endpoint_road=(0.0, 0.0, 0.0),
        endpoints_candidates={
            186: (0.0, 0.0, 0.0),
            187: (0.1, 0.0, 0.0),
            188: (0.2, 0.0, 0.0),
        },
        lane_pairs=[(-1, 186, -1), (-2, 187, -1), (-3, 188, -1)],
        all_successor_lanelet_road_ids={186, 187, 188},
    )

    ok, reason = sanity_gate_passes(_site(), inputs, endpoint_tolerance=0.5)
    assert ok is True
    assert reason == ""


def test_sanity_gate_fails_on_endpoint_distance():
    inputs = SanityGateInputs(
        endpoint_road=(0.0, 0.0, 0.0),
        endpoints_candidates={
            186: (5.0, 0.0, 0.0),
            187: (0.0, 0.0, 0.0),
            188: (0.0, 0.0, 0.0),
        },
        lane_pairs=[(-1, 186, -1), (-2, 187, -1), (-3, 188, -1)],
        all_successor_lanelet_road_ids={186, 187, 188},
    )

    ok, reason = sanity_gate_passes(_site(), inputs, endpoint_tolerance=0.5)
    assert ok is False
    assert "endpoint" in reason


def test_sanity_gate_fails_on_lane_collision():
    inputs = SanityGateInputs(
        endpoint_road=(0.0, 0.0, 0.0),
        endpoints_candidates={
            186: (0.0, 0.0, 0.0),
            187: (0.0, 0.0, 0.0),
            188: (0.0, 0.0, 0.0),
        },
        lane_pairs=[(-1, 186, -1), (-2, 186, -1)],  # two source lanes -> same target
        all_successor_lanelet_road_ids={186, 187, 188},
    )

    ok, reason = sanity_gate_passes(_site(), inputs, endpoint_tolerance=0.5)
    assert ok is False
    assert "lane" in reason


def test_sanity_gate_fails_on_orphan_successor_lanelet_road():
    inputs = SanityGateInputs(
        endpoint_road=(0.0, 0.0, 0.0),
        endpoints_candidates={
            186: (0.0, 0.0, 0.0),
            187: (0.0, 0.0, 0.0),
            188: (0.0, 0.0, 0.0),
        },
        lane_pairs=[(-1, 186, -1), (-2, 187, -1), (-3, 188, -1)],
        all_successor_lanelet_road_ids={186, 187, 188, 999},  # 999 is orphan
    )

    ok, reason = sanity_gate_passes(_site(), inputs, endpoint_tolerance=0.5)
    assert ok is False
    assert "orphan" in reason or "exhaustive" in reason


def test_sanity_gate_fails_when_candidate_has_no_lane_pairs():
    """Every candidate road must contribute at least one lane pair (#291 review)."""
    inputs = SanityGateInputs(
        endpoint_road=(0.0, 0.0, 0.0),
        endpoints_candidates={
            186: (0.0, 0.0, 0.0),
            187: (0.0, 0.0, 0.0),
            188: (0.0, 0.0, 0.0),
        },
        # Road 188 has no lane pair.
        lane_pairs=[(-1, 186, -1), (-2, 187, -1)],
        all_successor_lanelet_road_ids={186, 187, 188},
    )
    ok, reason = sanity_gate_passes(_site(), inputs, endpoint_tolerance=0.5)
    assert ok is False
    assert "188" in reason or "lane pair" in reason


def test_synthesise_divergence_emits_one_connecting_road_per_lane_pair():
    site = DivergenceSite(
        road_id=185,
        side=DivergenceSide.SUCCESSOR,
        candidate_road_ids=[186, 187, 188],
    )
    inputs = SanityGateInputs(
        endpoint_road=(0.0, 0.0, 0.0),
        endpoints_candidates={
            186: (0.0, 0.0, 0.0),
            187: (0.0, 0.0, 0.0),
            188: (0.0, 0.0, 0.0),
        },
        lane_pairs=[(-1, 186, -1), (-2, 187, -1), (-3, 188, -1)],
        all_successor_lanelet_road_ids={186, 187, 188},
    )

    out: SynthesisOutput = synthesise_junction_for_site(
        site=site,
        inputs=inputs,
        starting_connecting_road_id=200,
        junction_id=2000,
        traffic_rule=TrafficRule.RHT,
        min_segment_length=0.01,
    )

    # 1 junction
    assert out.junction.id == 2000
    assert len(out.junction.connections) == 3

    # 3 connecting roads, contiguous IDs from 200
    assert [r.id for r in out.connecting_roads] == [200, 201, 202]
    for r in out.connecting_roads:
        assert r.junction == 2000
        assert r.length >= 0.01
        assert r.link.predecessor.element_id == 185
        assert r.link.predecessor.element_type == ElementType.ROAD
        assert r.link.predecessor.contact_point == ContactPoint.END
        assert r.link.successor.element_type == ElementType.ROAD

    # Lane links inside connections preserve the source-lane ordering.
    incoming_to_connecting = [
        (
            c.incoming_road,
            c.connecting_road,
            c.lane_links[0].from_lane,
            c.lane_links[0].to_lane,
        )
        for c in out.junction.connections
    ]
    assert (185, 200, -1, -1) in incoming_to_connecting
    assert (185, 201, -2, -1) in incoming_to_connecting
    assert (185, 202, -3, -1) in incoming_to_connecting

    # Source road link patch returned for the caller to apply.
    assert out.deferred_link_patch == ("successor", 185, 2000)


def test_synthesise_merge_sets_predecessor_link_on_each_candidate():
    site = DivergenceSite(
        road_id=42,
        side=DivergenceSide.PREDECESSOR,
        candidate_road_ids=[10, 11],
    )
    inputs = SanityGateInputs(
        endpoint_road=(0.0, 0.0, 0.0),
        endpoints_candidates={10: (0.0, 0.0, 0.0), 11: (0.0, 0.0, 0.0)},
        lane_pairs=[(-1, 10, -1), (-2, 11, -1)],
        all_successor_lanelet_road_ids={10, 11},
    )

    out = synthesise_junction_for_site(
        site=site,
        inputs=inputs,
        starting_connecting_road_id=300,
        junction_id=3000,
        traffic_rule=TrafficRule.RHT,
        min_segment_length=0.01,
    )

    # For merge, incoming roads of the connecting road = candidates,
    # successor = the merged road (42).
    for r in out.connecting_roads:
        assert r.link.successor.element_id == 42
        assert r.link.successor.contact_point == ContactPoint.START
        assert r.link.predecessor.element_id in {10, 11}
        assert r.link.predecessor.contact_point == ContactPoint.END

    assert out.deferred_link_patch == ("predecessor", 42, 3000)


def _build_road_stub(
    road_id: int,
    end_xyz=(0.0, 0.0, 0.0),
    start_xyz=(0.0, 0.0, 0.0),
):
    r = MagicMock(
        spec=[
            "id",
            "reference_start_xyz",
            "reference_end_xyz",
            "link",
            "add_successor",
            "add_predecessor",
            "plan_view",
            "sorted_lanelet_ids",
            "evaluate_lane_anchor_xyz",
        ]
    )
    r.id = road_id
    r.reference_start_xyz = start_xyz
    r.reference_end_xyz = end_xyz
    # Lane-aware anchor metadata is intentionally absent so the driver
    # falls back to the road reference endpoint (the synthesiser tests
    # the per-lane override path through the integration suite, not here).
    r.plan_view = None
    r.sorted_lanelet_ids = None
    r.link = MagicMock()
    r.link.predecessor = None
    r.link.successor = None
    return r


def test_apply_divergence_synthesis_happy_path_emits_objects_and_patches_link(
    monkeypatch,
):
    sites = [
        DivergenceSite(
            road_id=185,
            side=DivergenceSide.SUCCESSOR,
            candidate_road_ids=[186, 187, 188],
        )
    ]
    roads_by_id = {
        185: _build_road_stub(185, end_xyz=(10.0, 0.0, 0.0)),
        186: _build_road_stub(186, start_xyz=(10.0, 0.0, 0.0)),
        187: _build_road_stub(187, start_xyz=(10.0, 0.0, 0.0)),
        188: _build_road_stub(188, start_xyz=(10.0, 0.0, 0.0)),
    }

    def fake_lane_pairs(
        site, _roads_by_id, _lanelet_map, _routing_graph, _lanelet_to_road
    ):
        # Bypass the real routing-graph walk: hand back the lane mapping the
        # spec assumes for Road 185.
        return [(-1, 186, -1), (-2, 187, -1), (-3, 188, -1)], {186, 187, 188}

    monkeypatch.setattr(
        "autoware_lanelet2_to_opendrive.divergence._lane_pairs_for_site",
        fake_lane_pairs,
    )

    result = apply_divergence_synthesis(
        sites=sites,
        roads_by_id=roads_by_id,
        lanelet_map=MagicMock(),
        routing_graph=MagicMock(),
        lanelet_to_road=MagicMock(),
        traffic_rule=TrafficRule.RHT,
        starting_connecting_road_id=200,
        starting_junction_id=2000,
        endpoint_tolerance=0.5,
        min_segment_length=0.01,
    )

    assert isinstance(result, DivergenceSynthesisResult)
    assert len(result.junctions) == 1
    assert len(result.connecting_roads) == 3
    # The source road's deferred successor link was patched.
    assert roads_by_id[185].add_successor.called


def test_apply_divergence_synthesis_falls_back_to_nearest_candidate(monkeypatch):
    """A failed gate links the nearest candidate, never an arbitrary one.

    Candidate 186 comes first but is 99 m away; linking to it would emit a
    connector pointing in an almost arbitrary direction.
    """
    sites = [
        DivergenceSite(
            road_id=185,
            side=DivergenceSide.SUCCESSOR,
            candidate_road_ids=[186, 187, 188],
        )
    ]
    roads_by_id = {
        185: _build_road_stub(185, end_xyz=(0.0, 0.0, 0.0)),
        186: _build_road_stub(186, start_xyz=(99.0, 0.0, 0.0)),  # endpoint mismatch
        187: _build_road_stub(187, start_xyz=(0.0, 0.0, 0.0)),
        188: _build_road_stub(188, start_xyz=(0.0, 0.0, 0.0)),
    }
    monkeypatch.setattr(
        "autoware_lanelet2_to_opendrive.divergence._lane_pairs_for_site",
        lambda *_a, **_kw: (
            [(-1, 186, -1), (-2, 187, -1), (-3, 188, -1)],
            {186, 187, 188},
        ),
    )

    result = apply_divergence_synthesis(
        sites=sites,
        roads_by_id=roads_by_id,
        lanelet_map=MagicMock(),
        routing_graph=MagicMock(),
        lanelet_to_road=MagicMock(),
        traffic_rule=TrafficRule.RHT,
        starting_connecting_road_id=200,
        starting_junction_id=2000,
        endpoint_tolerance=0.5,
        min_segment_length=0.01,
    )

    # No synthetic junction emitted; source road took the nearest candidate as
    # its road-level successor (single-road fallback).
    assert result.junctions == []
    assert result.connecting_roads == []
    roads_by_id[185].add_successor.assert_called_once()
    args, kwargs = roads_by_id[185].add_successor.call_args
    assert kwargs["element_type"] == ElementType.ROAD
    assert kwargs["element_id"] == 187


def _lane_anchor_road_stub(
    road_id: int,
    *,
    start_xyz=(0.0, 0.0, 0.0),
    end_xyz=(0.0, 0.0, 0.0),
    sorted_lanelet_ids=None,
    anchors=None,
):
    """Road stub whose ``evaluate_lane_anchor_xyz`` returns scripted anchors.

    ``anchors`` maps ``(sorted_index, at_start)`` to the rendered lane anchor,
    i.e. the point the synthetic connector actually terminates on.
    """
    road = _build_road_stub(road_id, start_xyz=start_xyz, end_xyz=end_xyz)
    road.sorted_lanelet_ids = sorted_lanelet_ids or [road_id]
    scripted = anchors or {}

    def evaluate_lane_anchor_xyz(sorted_index: int, at_start: bool):
        return scripted[(sorted_index, at_start)]

    road.evaluate_lane_anchor_xyz.side_effect = evaluate_lane_anchor_xyz
    return road


def _connector_edges(connecting_roads):
    """Collect ``(incoming_road, from_lane, outgoing_road, to_lane)`` per connector."""
    edges = set()
    for road in connecting_roads:
        lane_section = road.lanes.lane_sections[0]
        lanes = {**lane_section.left_lanes, **lane_section.right_lanes}
        lane = next(lane for lane in lanes.values() if lane.lane_id != 0)
        edges.add(
            (
                road.link.predecessor.element_id,
                lane.predecessor.id,
                road.link.successor.element_id,
                lane.successor.id,
            )
        )
    return edges


def test_sanity_gate_accepts_split_candidate_on_a_non_reference_lane():
    """A candidate attached to lane -2 is offset by a lane width by design."""
    site = DivergenceSite(
        road_id=50,
        side=DivergenceSide.SUCCESSOR,
        candidate_road_ids=[51, 52],
    )
    # Candidate 52 leaves the source's second lane, so its reference start is
    # one lane width away from the source's reference end.
    reference_only = SanityGateInputs(
        endpoint_road=(0.0, 0.0, 0.0),
        endpoints_candidates={51: (0.0, 0.0, 0.0), 52: (0.0, -3.2, 0.0)},
        lane_pairs=[(-1, 51, -1), (-2, 52, -1)],
        all_successor_lanelet_road_ids={51, 52},
    )
    ok, reason = sanity_gate_passes(site, reference_only, endpoint_tolerance=0.5)
    assert ok is False
    assert "endpoint" in reason

    lane_aware = SanityGateInputs(
        endpoint_road=reference_only.endpoint_road,
        endpoints_candidates=reference_only.endpoints_candidates,
        lane_pairs=reference_only.lane_pairs,
        all_successor_lanelet_road_ids=reference_only.all_successor_lanelet_road_ids,
        # The rendered lane anchors coincide, which is what actually matters.
        lane_pair_endpoint_distances={(-1, 51, -1): 0.0, (-2, 52, -1): 0.01},
    )
    ok, reason = sanity_gate_passes(site, lane_aware, endpoint_tolerance=0.5)
    assert ok is True
    assert reason == ""


def test_sanity_gate_rejects_genuinely_distant_lane_anchor():
    """Lane-aware gating still rejects anchors that are really far apart."""
    site = DivergenceSite(
        road_id=50,
        side=DivergenceSide.SUCCESSOR,
        candidate_road_ids=[51, 52],
    )
    inputs = SanityGateInputs(
        endpoint_road=(0.0, 0.0, 0.0),
        endpoints_candidates={51: (0.0, 0.0, 0.0), 52: (0.0, -3.2, 0.0)},
        lane_pairs=[(-1, 51, -1), (-2, 52, -1)],
        all_successor_lanelet_road_ids={51, 52},
        lane_pair_endpoint_distances={(-1, 51, -1): 0.0, (-2, 52, -1): 4.0},
    )
    ok, reason = sanity_gate_passes(site, inputs, endpoint_tolerance=0.5)
    assert ok is False
    assert "lane-aware endpoint distance" in reason


def test_apply_divergence_synthesis_fork_keeps_every_branch(monkeypatch):
    """A 1->2 fork on a regular road keeps both branches.

    Source lane -1 forks into roads 51 and 52 while lane -2 continues into
    road 53, whose reference start is a lane width away. Gating on reference
    endpoints rejected the whole site and dropped both fork branches.
    """
    sites = [
        DivergenceSite(
            road_id=50,
            side=DivergenceSide.SUCCESSOR,
            candidate_road_ids=[51, 52, 53],
        )
    ]
    roads_by_id = {
        50: _lane_anchor_road_stub(
            50,
            end_xyz=(0.0, 0.0, 0.0),
            sorted_lanelet_ids=[5001, 5002],
            anchors={(0, False): (0.0, 0.0, 0.0), (1, False): (0.0, -3.2, 0.0)},
        ),
        51: _lane_anchor_road_stub(
            51, start_xyz=(0.0, 0.0, 0.0), anchors={(0, True): (0.0, 0.0, 0.0)}
        ),
        52: _lane_anchor_road_stub(
            52, start_xyz=(0.0, 0.0, 0.0), anchors={(0, True): (0.0, 0.0, 0.0)}
        ),
        53: _lane_anchor_road_stub(
            53, start_xyz=(0.0, -3.2, 0.0), anchors={(0, True): (0.0, -3.2, 0.0)}
        ),
    }
    monkeypatch.setattr(
        "autoware_lanelet2_to_opendrive.divergence._lane_pairs_for_site",
        lambda *_a, **_kw: (
            [(-1, 51, -1), (-1, 52, -1), (-2, 53, -1)],
            {51, 52, 53},
        ),
    )

    result = apply_divergence_synthesis(
        sites=sites,
        roads_by_id=roads_by_id,
        lanelet_map=MagicMock(),
        routing_graph=MagicMock(),
        lanelet_to_road=MagicMock(),
        traffic_rule=TrafficRule.RHT,
        starting_connecting_road_id=200,
        starting_junction_id=2000,
        endpoint_tolerance=0.5,
        min_segment_length=0.01,
    )

    assert len(result.junctions) == 1
    assert _connector_edges(result.connecting_roads) == {
        (50, -1, 51, -1),
        (50, -1, 52, -1),
        (50, -2, 53, -1),
    }
    _, kwargs = roads_by_id[50].add_successor.call_args
    assert kwargs["element_type"] == ElementType.JUNCTION
    for candidate_id in (51, 52, 53):
        _, kwargs = roads_by_id[candidate_id].add_predecessor.call_args
        assert kwargs["element_type"] == ElementType.JUNCTION


def test_apply_divergence_synthesis_merge_keeps_every_branch(monkeypatch):
    """A 2->1 merge keeps both branches.

    Road 11 merges into the source's lane -2, whose anchor is one lane width
    away from the source's reference start.
    """
    sites = [
        DivergenceSite(
            road_id=42,
            side=DivergenceSide.PREDECESSOR,
            candidate_road_ids=[10, 11],
        )
    ]
    roads_by_id = {
        42: _lane_anchor_road_stub(
            42,
            start_xyz=(0.0, 0.0, 0.0),
            sorted_lanelet_ids=[4201, 4202],
            anchors={(0, True): (0.0, 0.0, 0.0), (1, True): (3.2, 0.0, 0.0)},
        ),
        10: _lane_anchor_road_stub(
            10, end_xyz=(0.0, 0.0, 0.0), anchors={(0, False): (0.0, 0.0, 0.0)}
        ),
        11: _lane_anchor_road_stub(
            11, end_xyz=(3.2, 0.0, 0.0), anchors={(0, False): (3.2, 0.0, 0.0)}
        ),
    }
    monkeypatch.setattr(
        "autoware_lanelet2_to_opendrive.divergence._lane_pairs_for_site",
        lambda *_a, **_kw: ([(-1, 10, -1), (-2, 11, -1)], {10, 11}),
    )

    result = apply_divergence_synthesis(
        sites=sites,
        roads_by_id=roads_by_id,
        lanelet_map=MagicMock(),
        routing_graph=MagicMock(),
        lanelet_to_road=MagicMock(),
        traffic_rule=TrafficRule.RHT,
        starting_connecting_road_id=200,
        starting_junction_id=2000,
        endpoint_tolerance=0.5,
        min_segment_length=0.01,
    )

    assert len(result.junctions) == 1
    assert _connector_edges(result.connecting_roads) == {
        (10, -1, 42, -1),
        (11, -1, 42, -2),
    }
    _, kwargs = roads_by_id[42].add_predecessor.call_args
    assert kwargs["element_type"] == ElementType.JUNCTION
    assert kwargs["element_id"] == 2000
    roads_by_id[10].add_successor.assert_called_once()
    roads_by_id[11].add_successor.assert_called_once()


def test_apply_divergence_synthesis_unions_merge_and_split_of_one_boundary(
    monkeypatch,
):
    """Both sites of one N->M boundary share a single junction."""
    sites = [
        DivergenceSite(
            road_id=31,
            side=DivergenceSide.PREDECESSOR,
            candidate_road_ids=[28, 29],
        ),
        DivergenceSite(
            road_id=29,
            side=DivergenceSide.SUCCESSOR,
            candidate_road_ids=[31, 32],
        ),
    ]
    roads_by_id = {
        28: _lane_anchor_road_stub(
            28, end_xyz=(0.0, 3.0, 0.0), anchors={(0, False): (0.0, 3.0, 0.0)}
        ),
        29: _lane_anchor_road_stub(
            29, end_xyz=(0.0, 0.0, 0.0), anchors={(0, False): (0.0, 0.0, 0.0)}
        ),
        31: _lane_anchor_road_stub(
            31,
            start_xyz=(0.0, 0.0, 0.0),
            sorted_lanelet_ids=[3101, 3102],
            anchors={(0, True): (0.0, 0.0, 0.0), (1, True): (0.0, 3.0, 0.0)},
        ),
        32: _lane_anchor_road_stub(
            32, start_xyz=(0.0, 0.0, 0.0), anchors={(0, True): (0.0, 0.0, 0.0)}
        ),
    }

    def lane_pairs_for_site(site, *_a, **_kw):
        if site.road_id == 31:
            return [(-1, 29, -1), (-2, 28, -1)], {28, 29}
        if site.road_id == 29:
            return [(-1, 31, -1), (-1, 32, -1)], {31, 32}
        raise AssertionError(f"unexpected site {site}")

    monkeypatch.setattr(
        "autoware_lanelet2_to_opendrive.divergence._lane_pairs_for_site",
        lane_pairs_for_site,
    )

    result = apply_divergence_synthesis(
        sites=sites,
        roads_by_id=roads_by_id,
        lanelet_map=MagicMock(),
        routing_graph=MagicMock(),
        lanelet_to_road=MagicMock(),
        traffic_rule=TrafficRule.RHT,
        starting_connecting_road_id=200,
        starting_junction_id=2000,
        endpoint_tolerance=0.5,
        min_segment_length=0.01,
    )

    assert len(result.junctions) == 1
    junction_id = result.junctions[0].id
    assert len(result.junctions[0].connections) == 3
    assert _connector_edges(result.connecting_roads) == {
        (29, -1, 31, -1),
        (28, -1, 31, -2),
        (29, -1, 32, -1),
    }
    assert all(road.junction == junction_id for road in result.connecting_roads)
    for road_id in (31, 32):
        _, kwargs = roads_by_id[road_id].add_predecessor.call_args
        assert kwargs["element_id"] == junction_id
    for road_id in (28, 29):
        _, kwargs = roads_by_id[road_id].add_successor.call_args
        assert kwargs["element_id"] == junction_id


def test_apply_divergence_synthesis_emits_nothing_when_no_candidate_is_close(
    monkeypatch,
):
    """No candidate within tolerance means no link at all, not an arbitrary one."""
    sites = [
        DivergenceSite(
            road_id=42,
            side=DivergenceSide.PREDECESSOR,
            candidate_road_ids=[10, 11],
        )
    ]
    roads_by_id = {
        42: _lane_anchor_road_stub(
            42, start_xyz=(0.0, 0.0, 0.0), anchors={(0, True): (0.0, 0.0, 0.0)}
        ),
        10: _lane_anchor_road_stub(
            10, end_xyz=(5.0, 0.0, 0.0), anchors={(0, False): (5.0, 0.0, 0.0)}
        ),
        11: _lane_anchor_road_stub(
            11, end_xyz=(9.0, 0.0, 0.0), anchors={(0, False): (9.0, 0.0, 0.0)}
        ),
    }
    monkeypatch.setattr(
        "autoware_lanelet2_to_opendrive.divergence._lane_pairs_for_site",
        lambda *_a, **_kw: ([(-1, 10, -1), (-1, 11, -1)], {10, 11}),
    )

    result = apply_divergence_synthesis(
        sites=sites,
        roads_by_id=roads_by_id,
        lanelet_map=MagicMock(),
        routing_graph=MagicMock(),
        lanelet_to_road=MagicMock(),
        traffic_rule=TrafficRule.RHT,
        starting_connecting_road_id=200,
        starting_junction_id=2000,
        endpoint_tolerance=0.5,
        min_segment_length=0.01,
    )

    assert result.junctions == []
    assert result.connecting_roads == []
    roads_by_id[42].add_predecessor.assert_not_called()


# ---------------------------------------------------------------------------
# Synthetic zero-length connectors must reproduce the cross-section they
# bridge, not a nominal one. The checker that cares is
# ``lane_smoothness.contact_point_no_horizontal_gaps``: at every contact it
# compares *both* lane borders of the two lanes and wants at least two of the
# four pairings inside 1 cm. Getting the reference line right is therefore
# only half of it -- the heading (which orients the lane normal) and the lane
# width (which places the outer border) decide the other half.
# ---------------------------------------------------------------------------


def _stub_between(
    start_xyz, end_xyz, *, min_segment_length, start_heading, end_heading=None
):
    return _make_zero_length_connecting_road(
        road_id=900,
        junction_id=9000,
        incoming_road_id=1,
        outgoing_road_id=2,
        incoming_contact=ContactPoint.END,
        outgoing_contact=ContactPoint.START,
        start_xyz=start_xyz,
        end_xyz=end_xyz,
        min_segment_length=min_segment_length,
        traffic_rule=TrafficRule.RHT,
        from_lane=-1,
        to_lane=-1,
        start_heading=start_heading,
        end_heading=end_heading,
    )


def _end_pose(geometry):
    """``(x, y, heading)`` at p=1 of a normalized paramPoly3."""
    du = geometry.bU + 2 * geometry.cU + 3 * geometry.dU
    dv = geometry.bV + 2 * geometry.cV + 3 * geometry.dV
    u = geometry.bU + geometry.cU + geometry.dU
    v = geometry.bV + geometry.cV + geometry.dV
    cos_h, sin_h = math.cos(geometry.hdg), math.sin(geometry.hdg)
    return (
        geometry.x + u * cos_h - v * sin_h,
        geometry.y + u * sin_h + v * cos_h,
        geometry.hdg + math.atan2(dv, du),
    )


# These four tests replace three that pinned the *previous* rule: the
# connector took its heading from the ``start_xyz -> end_xyz`` vector when
# that vector was at least ``min_segment_length`` long, and from the
# incoming road's tangent below it.  That rule is gone -- a curve meeting
# both tangents has no selection to make -- so the coverage is rewritten
# onto the new contract rather than deleted.


def test_both_tangents_give_a_curve_meeting_each_of_them():
    """The contract D5 rests on: both endpoints matched in position *and*
    tangent, so neither contact's lane borders fan out."""
    stub = _stub_between(
        (0.0, 0.0, 0.0),
        (0.5, 0.0, 0.0),
        min_segment_length=0.01,
        start_heading=0.25,
        end_heading=-0.4,
    )
    geometry = stub.plan_view.geometries[0]
    assert geometry.pRange == "normalized"
    assert geometry.hdg == pytest.approx(0.25)

    x1, y1, h1 = _end_pose(geometry)
    assert (x1, y1) == pytest.approx((0.5, 0.0), abs=1e-12)
    assert h1 == pytest.approx(-0.4)


def test_length_is_the_arc_length_not_the_chord():
    """``@length`` must be what the coefficients describe.

    The road's s-domain is written against it -- elevation and the tapered
    lane widths -- and ``road.geometry.parampoly3.normalized_range``
    compares the two.  A curve is longer than its chord, and quoting the
    chord made the checker evaluate the lane borders short of the road's
    real end: measured as 61 gap findings against 20, i.e. worse than not
    curving at all.
    """
    stub = _stub_between(
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        min_segment_length=0.01,
        start_heading=0.0,
        end_heading=math.radians(60.0),
    )
    geometry = stub.plan_view.geometries[0]
    chord = 1.0
    assert geometry.length > chord
    assert geometry.length == pytest.approx(geometry.arc_length(), rel=1e-12)
    # The road and its geometry must agree, or every profile keyed on the
    # road's s is written against a different curve.
    assert stub.length == pytest.approx(geometry.length)


def test_without_the_far_tangent_the_connector_stays_straight():
    """One tangent is not enough to bend towards anything."""
    stub = _stub_between(
        (0.0, 0.0, 0.0),
        (0.0, 0.004, 0.0),
        min_segment_length=0.01,
        start_heading=0.25,
        end_heading=None,
    )
    geometry = stub.plan_view.geometries[0]
    assert geometry.pRange == "arcLength"
    assert geometry.hdg == pytest.approx(0.25)
    assert geometry.length == pytest.approx(0.01)
    assert (geometry.cV, geometry.dV) == (0.0, 0.0)


def test_with_no_tangent_at_all_it_falls_back_to_the_anchor_vector(caplog):
    """The last resort, and it must say so.

    Reached only when a neighbour has no rendered plan view.  The anchor
    vector is a poor direction -- a median 88.9 deg from the incoming
    road's own tangent over the 296 Odaiba connectors that used to take it
    -- so taking it is worth a warning rather than a silent default.
    """
    with caplog.at_level(logging.WARNING):
        stub = _stub_between(
            (0.0, 0.0, 0.0),
            (0.0, 0.5, 0.0),
            min_segment_length=0.01,
            start_heading=None,
            end_heading=None,
        )
    geometry = stub.plan_view.geometries[0]
    assert geometry.hdg == pytest.approx(math.pi / 2)
    assert geometry.length == pytest.approx(0.5)
    assert any("anchor vector" in r.message for r in caplog.records)


def test_coincident_anchors_raise_rather_than_guess():
    """There is no curve between a point and itself.

    ``ParamPoly3.hermite`` refuses instead of inventing a direction; the
    connector builder is the one that decides what to draw, and it draws a
    straight segment along the start heading.
    """
    with pytest.raises(ValueError, match="coincide"):
        ParamPoly3.hermite(
            start_xy=(1.0, 2.0),
            start_heading=0.3,
            end_xy=(1.0, 2.0),
            end_heading=0.3,
        )

    # ...and the builder does not propagate it.
    stub = _stub_between(
        (1.0, 2.0, 0.0),
        (1.0, 2.0, 0.0),
        min_segment_length=0.01,
        start_heading=0.3,
        end_heading=1.2,
    )
    geometry = stub.plan_view.geometries[0]
    assert geometry.hdg == pytest.approx(0.3)
    assert geometry.length == pytest.approx(0.01)


def _width_road(road_id, *, width, length=10.0, heading=0.0, x=0.0, y=0.0):
    """A one-lane RHT road whose lane has a known constant width."""
    lane = Lane(lane_id=-1, lane_type=LaneType.DRIVING)
    lane.widths.append(LaneWidth(s_offset=0.0, a=width, b=0.0, c=0.0, d=0.0))
    lane_section = LaneSection(s_offset=0.0)
    lane_section.right_lanes[-1] = lane
    return Road(
        id=road_id,
        length=length,
        rule=TrafficRule.RHT,
        lanes=Lanes(lane_sections=[lane_section]),
        plan_view=PlanView(
            geometries=[
                ParamPoly3(
                    s=0.0,
                    x=x,
                    y=y,
                    hdg=heading,
                    length=length,
                    aU=0.0,
                    bU=1.0,
                    cU=0.0,
                    dU=0.0,
                    aV=0.0,
                    bV=0.0,
                    cV=0.0,
                    dV=0.0,
                    pRange="arcLength",
                )
            ]
        ),
        sorted_lanelet_ids=[1],
    )


def test_synthesised_connector_tapers_between_the_widths_it_bridges():
    """The stub reproduces both neighbours' widths, not the nominal 3.5 m.

    A constant nominal width leaves the stub's outer border up to a metre
    from the border of the road on either side, which the horizontal-gap
    checker reads as a gap even when the reference lines meet exactly.
    """
    incoming = _width_road(1, width=3.0)
    outgoing = _width_road(2, width=4.25, x=10.0)
    edges = _directed_edges_for_site(
        site=DivergenceSite(
            road_id=1,
            side=DivergenceSide.SUCCESSOR,
            candidate_road_ids=(2,),
        ),
        inputs=SanityGateInputs(
            endpoint_road=(10.0, 0.0, 0.0),
            endpoints_candidates={2: (10.0, 0.0, 0.0)},
            lane_pairs=[(-1, 2, -1)],
            all_successor_lanelet_road_ids={2},
        ),
        traffic_rule=TrafficRule.RHT,
        fallback_heading=0.0,
        roads_by_id={1: incoming, 2: outgoing},
    )
    assert [(e.start_lane_width, e.end_lane_width) for e in edges] == [(3.0, 4.25)]

    out = _synthesise_junction_for_edges(
        junction_name="divergence_1",
        edges=edges,
        starting_connecting_road_id=300,
        junction_id=3000,
        traffic_rule=TrafficRule.RHT,
        min_segment_length=0.005,
        lane_width=DEFAULT_CONFIG.geometry.divergence_default_lane_width,
    )
    stub = out.connecting_roads[0]
    width = stub.lanes.lane_sections[0].right_lanes[-1].widths[0]
    assert width.a == pytest.approx(3.0)
    assert width.a + width.b * stub.length == pytest.approx(4.25)


def test_merge_connector_takes_the_heading_of_the_road_it_leaves():
    """On a merge the incoming road is the *candidate*, not the site's road.

    The stub's near contact is with the road it leaves, so that road's end
    tangent -- not the merged road's start tangent -- is what keeps the
    stub's lane borders on top of the borders it continues.  The edge now
    carries the far tangent too, so the connector can meet both.
    """
    candidate = _width_road(1, width=3.0, heading=0.75)
    merged = _width_road(2, width=3.0, heading=0.0, x=10.0)
    edges = _directed_edges_for_site(
        site=DivergenceSite(
            road_id=2,
            side=DivergenceSide.PREDECESSOR,
            candidate_road_ids=(1,),
        ),
        inputs=SanityGateInputs(
            endpoint_road=(10.0, 0.0, 0.0),
            endpoints_candidates={1: (10.0, 0.0, 0.0)},
            lane_pairs=[(-1, 1, -1)],
            all_successor_lanelet_road_ids={1},
        ),
        traffic_rule=TrafficRule.RHT,
        fallback_heading=0.0,
        roads_by_id={1: candidate, 2: merged},
    )
    assert len(edges) == 1
    assert edges[0].incoming_road_id == 1
    assert edges[0].start_heading == pytest.approx(0.75)
    # The merged road's own start tangent is the far end of the connector.
    assert edges[0].end_heading == pytest.approx(0.0)
