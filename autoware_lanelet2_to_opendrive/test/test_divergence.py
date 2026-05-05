"""Tests for divergence/merge synthesis (issue #291)."""

import inspect

from autoware_lanelet2_to_opendrive.config import DEFAULT_CONFIG
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
