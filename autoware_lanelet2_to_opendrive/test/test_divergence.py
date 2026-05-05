"""Tests for divergence/merge synthesis (issue #291)."""

import inspect

from autoware_lanelet2_to_opendrive.config import DEFAULT_CONFIG
from autoware_lanelet2_to_opendrive.divergence import (
    DivergenceSide,
    DivergenceSite,
    collect_divergence_sites,
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
