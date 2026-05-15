"""Regression tests for routing-graph reuse during conversion.

Profiling showed ``create_routing_graph`` was rebuilt ~2200 times per
conversion because several call paths dropped the already-built map-wide
``RoutingGraph``. These tests pin each path to *reuse* a supplied graph and
guard the end-to-end call count so the redundancy cannot silently return.
"""

from __future__ import annotations

from unittest.mock import patch

import pytest

from autoware_lanelet2_to_opendrive import util as util_mod
from autoware_lanelet2_to_opendrive.opendrive.reference_line import ReferenceLine
from autoware_lanelet2_to_opendrive.util import (
    create_routing_graph,
    filter_lanelets_by_subtype,
    find_adjacent_groups,
)


@pytest.fixture(scope="session")
def routing_graph(lanelet_map):
    """A single map-wide routing graph, built once for the test module."""
    return create_routing_graph(lanelet_map)


@pytest.fixture(scope="session")
def adjacent_groups(lanelet_map, routing_graph):
    """All adjacent road-lanelet groups for the test map."""
    road_lanelets = filter_lanelets_by_subtype(
        list(lanelet_map.laneletLayer), ["road"]
    )
    return find_adjacent_groups(lanelet_map, set(road_lanelets), routing_graph)


@pytest.fixture(scope="session")
def sample_group(adjacent_groups):
    """One adjacent lanelet group suitable for ReferenceLine/Road construction."""
    if not adjacent_groups:
        pytest.skip("No adjacent road groups found in test map")
    return adjacent_groups[0]


def test_reference_line_reuses_supplied_routing_graph(
    lanelet_map, routing_graph, sample_group
):
    """ReferenceLine must not rebuild the graph when one is supplied."""
    with patch.object(
        util_mod, "create_routing_graph", wraps=util_mod.create_routing_graph
    ) as spy:
        ReferenceLine.construct_from_lanelet_groups(
            lanelet_map, sample_group, routing_graph=routing_graph
        )
    assert spy.call_count == 0


def test_road_forwards_routing_graph_to_reference_line(
    lanelet_map, routing_graph, sample_group
):
    """Road.construct_from_lanelet_groups must pass its graph to ReferenceLine."""
    from autoware_lanelet2_to_opendrive.opendrive.road import Road

    original = ReferenceLine.construct_from_lanelet_groups
    with patch.object(
        ReferenceLine, "construct_from_lanelet_groups", wraps=original
    ) as spy:
        Road.construct_from_lanelet_groups(
            lanelet_map=lanelet_map,
            lanelet_group=sample_group,
            road_id=0,
            routing_graph=routing_graph,
        )
    # The first ReferenceLine call is the direct one in Road; later calls come
    # from LaneSection (fixed separately in Task 3).
    first_call = spy.call_args_list[0]
    assert first_call.kwargs.get("routing_graph") is routing_graph


def test_lane_section_forwards_routing_graph_to_reference_line(
    lanelet_map, routing_graph, sample_group
):
    """LaneSection.construct_from_lanelet_groups must pass its graph onward."""
    from autoware_lanelet2_to_opendrive.opendrive.lane_section import LaneSection

    original = ReferenceLine.construct_from_lanelet_groups
    with patch.object(
        ReferenceLine, "construct_from_lanelet_groups", wraps=original
    ) as spy:
        LaneSection.construct_from_lanelet_groups(
            lanelet_map,
            sample_group,
            routing_graph=routing_graph,
        )
    assert spy.call_args_list[0].kwargs.get("routing_graph") is routing_graph


@pytest.mark.slow
def test_construct_from_lanelet_map_reuses_single_routing_graph(
    lanelet_map, monkeypatch
):
    """End-to-end: building all regular roads rebuilds the graph <=10 times.

    Before the fix this exceeded 1500 (one rebuild per ReferenceLine /
    LaneSection construction). ``Road.construct_from_lanelet_map`` builds its
    own graph via the ``RoutingGraph`` constructor directly, so a correct run
    should reach ``create_routing_graph`` only via incidental fallbacks.
    """
    from autoware_lanelet2_to_opendrive.opendrive.road import Road

    real = util_mod.create_routing_graph
    count = 0

    def counting(lanelet_map_arg):
        nonlocal count
        count += 1
        return real(lanelet_map_arg)

    monkeypatch.setattr(util_mod, "create_routing_graph", counting)
    Road.construct_from_lanelet_map(lanelet_map)
    assert count <= 10, f"create_routing_graph called {count} times (expected <=10)"
