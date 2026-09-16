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
    road_lanelets = filter_lanelets_by_subtype(list(lanelet_map.laneletLayer), ["road"])
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
    # from LaneSection (covered by
    # test_lane_section_forwards_routing_graph_to_reference_line).
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


@pytest.fixture(scope="session")
def regular_roads_result(lanelet_map):
    """Regular roads + lanelet->road map, built once for the junction test."""
    from autoware_lanelet2_to_opendrive.opendrive.road import Road

    return Road.construct_from_lanelet_map(lanelet_map)


@pytest.mark.slow
def test_connecting_roads_reuse_single_routing_graph(
    lanelet_map, regular_roads_result, monkeypatch
):
    """Building connecting roads (incl. _lane_aware_endpoint) must not rebuild
    the graph per call. Before the fix this reached the hundreds."""
    from autoware_lanelet2_to_opendrive.junction import (
        _filter_lanelets_inside_junction,
        find_junction_groups,
    )
    from autoware_lanelet2_to_opendrive.opendrive.road import Road

    junction_lanelets = _filter_lanelets_inside_junction(list(lanelet_map.laneletLayer))
    junction_groups = find_junction_groups(junction_lanelets)

    real = util_mod.create_routing_graph
    count = 0

    def counting(lanelet_map_arg):
        nonlocal count
        count += 1
        return real(lanelet_map_arg)

    monkeypatch.setattr(util_mod, "create_routing_graph", counting)
    Road.construct_connecting_roads_from_junctions(
        lanelet_map=lanelet_map,
        junction_groups=junction_groups[:5],
        regular_roads=regular_roads_result.roads,
        lanelet_to_road_id=regular_roads_result.lanelet_to_road,
    )
    assert count <= 30, f"create_routing_graph called {count} times (expected <=30)"


def test_calculate_signal_position_forwards_routing_graph():
    """_calculate_signal_position must forward routing_graph to ReferenceLine."""
    from unittest.mock import Mock

    import numpy as np

    from autoware_lanelet2_to_opendrive.opendrive import SignalsAndControllers
    from autoware_lanelet2_to_opendrive.spline import Splines

    traffic_light = Mock()
    traffic_light.stopLine = None
    traffic_light.id = 1

    point = Mock(x=10.0, y=5.0, z=2.0)
    linestring = Mock()
    linestring.__len__ = Mock(return_value=1)
    linestring.__getitem__ = Mock(return_value=point)
    traffic_light.trafficLights = [linestring]

    spline = Splines(np.array([[0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [20.0, 0.0, 0.0]]))
    lanelet_map = Mock()
    lanelet_map.laneletLayer.get.return_value = Mock()
    mapping = Mock()
    mapping.get_lanelets_for_road.return_value = [1, 2, 3]

    sentinel = object()
    with patch(
        "autoware_lanelet2_to_opendrive.opendrive.reference_line.ReferenceLine"
    ) as mock_rl:
        reference_line = Mock()
        reference_line.centerline_2d = spline
        mock_rl.construct_from_lanelet_groups.return_value = reference_line

        SignalsAndControllers._calculate_signal_position(
            traffic_light=traffic_light,
            light_linestring=linestring,
            road_id=0,
            lanelet_map=lanelet_map,
            road_lanelet_mapping=mapping,
            routing_graph=sentinel,
        )

    assert (
        mock_rl.construct_from_lanelet_groups.call_args.kwargs.get("routing_graph")
        is sentinel
    )


@pytest.fixture(scope="session")
def adjacent_pair(adjacent_groups):
    """A complete adjacent group of exactly two lanelets.

    A two-member group is, by construction, two lanelets joined by a single
    left/right adjacency -- exactly the directly-adjacent pair that
    ``extract_centerline_as_spline_from_two_lanelets`` and
    ``sort_adjacent_groups`` require. An arbitrary 2-element subset of a
    larger group is NOT guaranteed to be two direct neighbours.
    """
    for group in adjacent_groups:
        if len(group) == 2:
            return set(group)
    pytest.skip("test map has no 2-lanelet adjacent group")


def test_extract_centerline_reuses_supplied_routing_graph(
    lanelet_map, routing_graph, adjacent_pair
):
    """extract_centerline_as_spline_from_two_lanelets must reuse a graph."""
    from autoware_lanelet2_to_opendrive.centerline import (
        extract_centerline_as_spline_from_two_lanelets,
    )

    with patch.object(
        util_mod, "create_routing_graph", wraps=util_mod.create_routing_graph
    ) as spy:
        extract_centerline_as_spline_from_two_lanelets(
            lanelet_map, adjacent_pair, routing_graph=routing_graph
        )
    assert spy.call_count == 0


def _signal_construct_mocks():
    """Minimal mocks to drive ``construct_from_lanelet_map`` with one signal."""
    from unittest.mock import MagicMock, Mock

    from autoware_lanelet2_to_opendrive.opendrive.enums import TrafficRule
    from autoware_lanelet2_to_opendrive.util import RoadLaneletMapping

    points = [MagicMock(x=10.0, y=5.0, z=8.0)]
    linestring = MagicMock()
    linestring.id = 1001
    linestring.__len__ = Mock(return_value=len(points))
    linestring.__getitem__ = Mock(side_effect=lambda i: points[i])
    linestring.__iter__ = Mock(side_effect=lambda: iter(points))

    traffic_light = MagicMock()
    traffic_light.id = 9000
    traffic_light.trafficLights = [linestring]
    traffic_light.stopLine = None
    traffic_light.attributes = {}

    lanelet_map = MagicMock()
    lanelet_map.laneletLayer.get.return_value = MagicMock()

    mapping = RoadLaneletMapping(
        road_to_lanelets={0: [100, 101], 1: [200, 201]},
        lanelet_to_road={100: 0, 101: 0, 200: 1, 201: 1},
    )

    roads = []
    for road_id, lane_map in ((0, {100: -1, 101: -2}), (1, {200: -1, 201: -2})):
        road = MagicMock()
        road.id = road_id
        road.rule = TrafficRule.RHT
        road.get_lanelet_to_lane_mapping.return_value = lane_map
        road.get_half_width_at_s.return_value = -4.0
        road.get_elevation_at_s.return_value = 0.0
        roads.append(road)

    return lanelet_map, mapping, roads, {9000: (traffic_light, [100, 200])}


def test_signals_construct_reuses_supplied_routing_graph():
    """SignalsAndControllers must not rebuild the graph when one is supplied.

    The signals phase used to call ``create_routing_graph`` unconditionally,
    paying for a full map-wide graph a second time on every conversion.
    """
    from autoware_lanelet2_to_opendrive.opendrive import SignalsAndControllers

    lanelet_map, mapping, roads, tl_map = _signal_construct_mocks()
    sentinel = object()

    with (
        patch(
            "autoware_lanelet2_to_opendrive.opendrive.signals_and_controllers.filter_regulatory_element_by_type",
            return_value=tl_map,
        ),
        patch(
            "autoware_lanelet2_to_opendrive.opendrive.signals_and_controllers.create_routing_graph"
        ) as build,
        patch(
            "autoware_lanelet2_to_opendrive.opendrive.signals_and_controllers.SignalsAndControllers._calculate_signal_position",
            return_value=(5.0, -3.0),
        ) as position,
    ):
        SignalsAndControllers.construct_from_lanelet_map(
            lanelet_map=lanelet_map,
            road_lanelet_mapping=mapping,
            roads=roads,
            routing_graph=sentinel,
        )

    assert build.call_count == 0
    # ...and the supplied graph is what reaches the per-signal fit.
    assert position.call_args.kwargs.get("routing_graph") is sentinel


def test_signals_construct_builds_graph_when_none_supplied():
    """Backward compatibility: omitting the graph keeps the single build."""
    from autoware_lanelet2_to_opendrive.opendrive import SignalsAndControllers

    lanelet_map, mapping, roads, tl_map = _signal_construct_mocks()

    with (
        patch(
            "autoware_lanelet2_to_opendrive.opendrive.signals_and_controllers.filter_regulatory_element_by_type",
            return_value=tl_map,
        ),
        patch(
            "autoware_lanelet2_to_opendrive.opendrive.signals_and_controllers.create_routing_graph"
        ) as build,
        patch(
            "autoware_lanelet2_to_opendrive.opendrive.signals_and_controllers.SignalsAndControllers._calculate_signal_position",
            return_value=(5.0, -3.0),
        ),
    ):
        SignalsAndControllers.construct_from_lanelet_map(
            lanelet_map=lanelet_map,
            road_lanelet_mapping=mapping,
            roads=roads,
        )

    assert build.call_count == 1


def test_signal_position_cache_fits_each_road_once():
    """A shared cache reuses one reference line per road, results unchanged.

    ``_calculate_signal_position`` refits the road's reference line for every
    (signal, road) pair; the B-spline solve dominates the signals phase on
    maps where several traffic lights share a road.
    """
    from unittest.mock import Mock

    import numpy as np

    from autoware_lanelet2_to_opendrive.opendrive import SignalsAndControllers
    from autoware_lanelet2_to_opendrive.spline import Splines

    traffic_light = Mock()
    traffic_light.stopLine = None
    traffic_light.id = 1

    point = Mock(x=10.0, y=5.0, z=2.0)
    linestring = Mock()
    linestring.__len__ = Mock(return_value=1)
    linestring.__getitem__ = Mock(return_value=point)

    spline = Splines(np.array([[0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [20.0, 0.0, 0.0]]))
    lanelet_map = Mock()
    lanelet_map.laneletLayer.get.return_value = Mock()
    mapping = Mock()
    mapping.get_lanelets_for_road.return_value = [1, 2, 3]

    cache: dict = {}
    with patch(
        "autoware_lanelet2_to_opendrive.opendrive.reference_line.ReferenceLine"
    ) as mock_rl:
        reference_line = Mock()
        reference_line.centerline_2d = spline
        mock_rl.construct_from_lanelet_groups.return_value = reference_line

        def call(road_id):
            return SignalsAndControllers._calculate_signal_position(
                traffic_light=traffic_light,
                light_linestring=linestring,
                road_id=road_id,
                lanelet_map=lanelet_map,
                road_lanelet_mapping=mapping,
                reference_line_cache=cache,
            )

        first = call(7)
        second = call(7)
        assert mock_rl.construct_from_lanelet_groups.call_count == 1
        assert first == second

        # A different road is a cache miss and must be fitted on its own.
        call(8)
        assert mock_rl.construct_from_lanelet_groups.call_count == 2

    assert set(cache) == {7, 8}


def test_signal_position_without_cache_still_fits_every_time():
    """Omitting the cache keeps the previous (uncached) behaviour."""
    from unittest.mock import Mock

    import numpy as np

    from autoware_lanelet2_to_opendrive.opendrive import SignalsAndControllers
    from autoware_lanelet2_to_opendrive.spline import Splines

    traffic_light = Mock()
    traffic_light.stopLine = None
    traffic_light.id = 1

    point = Mock(x=10.0, y=5.0, z=2.0)
    linestring = Mock()
    linestring.__len__ = Mock(return_value=1)
    linestring.__getitem__ = Mock(return_value=point)

    spline = Splines(np.array([[0.0, 0.0, 0.0], [10.0, 0.0, 0.0], [20.0, 0.0, 0.0]]))
    lanelet_map = Mock()
    lanelet_map.laneletLayer.get.return_value = Mock()
    mapping = Mock()
    mapping.get_lanelets_for_road.return_value = [1, 2, 3]

    with patch(
        "autoware_lanelet2_to_opendrive.opendrive.reference_line.ReferenceLine"
    ) as mock_rl:
        reference_line = Mock()
        reference_line.centerline_2d = spline
        mock_rl.construct_from_lanelet_groups.return_value = reference_line

        for _ in range(2):
            SignalsAndControllers._calculate_signal_position(
                traffic_light=traffic_light,
                light_linestring=linestring,
                road_id=7,
                lanelet_map=lanelet_map,
                road_lanelet_mapping=mapping,
            )

    assert mock_rl.construct_from_lanelet_groups.call_count == 2


def test_signal_position_failed_fit_is_not_cached():
    """A road whose fit raises must warn and fall back on every signal."""
    from unittest.mock import Mock

    from autoware_lanelet2_to_opendrive.opendrive import SignalsAndControllers

    traffic_light = Mock()
    traffic_light.stopLine = None
    traffic_light.id = 1

    point = Mock(x=10.0, y=5.0, z=2.0)
    linestring = Mock()
    linestring.__len__ = Mock(return_value=1)
    linestring.__getitem__ = Mock(return_value=point)

    lanelet_map = Mock()
    lanelet_map.laneletLayer.get.return_value = Mock()
    mapping = Mock()
    mapping.get_lanelets_for_road.return_value = [1, 2, 3]

    cache: dict = {}
    with patch(
        "autoware_lanelet2_to_opendrive.opendrive.reference_line.ReferenceLine"
    ) as mock_rl:
        mock_rl.construct_from_lanelet_groups.side_effect = RuntimeError("fit failed")

        results = [
            SignalsAndControllers._calculate_signal_position(
                traffic_light=traffic_light,
                light_linestring=linestring,
                road_id=7,
                lanelet_map=lanelet_map,
                road_lanelet_mapping=mapping,
                reference_line_cache=cache,
            )
            for _ in range(2)
        ]

    assert results == [(0.0, -4.0), (0.0, -4.0)]
    assert mock_rl.construct_from_lanelet_groups.call_count == 2
    assert cache == {}
