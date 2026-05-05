"""Tests for divergence/merge synthesis (issue #291)."""

import inspect
from unittest.mock import MagicMock

from autoware_lanelet2_to_opendrive.config import DEFAULT_CONFIG
from autoware_lanelet2_to_opendrive.divergence import (
    DivergenceSide,
    DivergenceSite,
    DivergenceSynthesisResult,
    SanityGateInputs,
    SynthesisOutput,
    apply_divergence_synthesis,
    collect_divergence_sites,
    sanity_gate_passes,
    synthesise_junction_for_site,
)
from autoware_lanelet2_to_opendrive.opendrive.enums import (
    ContactPoint,
    ElementType,
    TrafficRule,
)
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


def test_apply_divergence_synthesis_falls_back_on_gate_failure(monkeypatch):
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

    # No synthetic junction emitted; source road took the first candidate as its
    # road-level successor (single-road fallback).
    assert result.junctions == []
    assert result.connecting_roads == []
    roads_by_id[185].add_successor.assert_called_once()
    args, kwargs = roads_by_id[185].add_successor.call_args
    assert kwargs["element_type"] == ElementType.ROAD
    assert kwargs["element_id"] == 186
