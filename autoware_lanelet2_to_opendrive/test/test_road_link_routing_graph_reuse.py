"""Regression tests for routing-graph reuse in the road-link passes.

``_setup_connections`` receives the map-wide ``RoutingGraph`` that
``Road.construct_from_lanelet_map`` already built, but two of the passes it
drives used to rebuild it from scratch: ``set_connecting_road_links``
(unconditionally) and ``set_all_lane_links`` (because the caller never
forwarded its graph). On a 25k-lanelet map each rebuild costs whole seconds.
These tests pin both paths to reuse a supplied graph.
"""

from __future__ import annotations

from unittest.mock import Mock, patch

from autoware_lanelet2_to_opendrive.main import _Lanelet2ToOpenDRIVEConverter
from autoware_lanelet2_to_opendrive.opendrive import road as road_mod
from autoware_lanelet2_to_opendrive.opendrive.road import Road


def test_set_connecting_road_links_reuses_supplied_routing_graph():
    """A supplied graph must suppress the internal ``RoutingGraph`` build."""
    sentinel_graph = Mock()

    with patch.object(road_mod, "RoutingGraph") as graph_ctor:
        Road.set_connecting_road_links(
            lanelet_map=Mock(),
            connecting_roads=[],
            lanelet_to_road_id={},
            road_to_lanelet_ids={},
            routing_graph=sentinel_graph,
        )

    graph_ctor.assert_not_called()


def test_set_connecting_road_links_builds_graph_when_omitted():
    """Omitting the graph keeps the previous, self-sufficient behaviour."""
    with patch.object(road_mod, "RoutingGraph") as graph_ctor:
        Road.set_connecting_road_links(
            lanelet_map=Mock(),
            connecting_roads=[],
            lanelet_to_road_id={},
            road_to_lanelet_ids={},
        )

    assert graph_ctor.call_count == 1


def test_setup_connections_forwards_routing_graph_to_every_pass():
    """``_setup_connections`` must hand its graph to every pass that takes one.

    ``set_incoming_road_junction_links`` works off the road/junction objects
    alone and takes no graph, so it is only patched out here.
    """
    converter = object.__new__(_Lanelet2ToOpenDRIVEConverter)
    converter.lanelet_map = Mock()
    sentinel_graph = Mock()

    with (
        patch.object(Road, "set_connecting_road_links") as connecting,
        patch.object(Road, "set_incoming_road_junction_links"),
        patch.object(Road, "set_outgoing_road_junction_links") as outgoing,
        patch.object(Road, "set_all_lane_links") as lane_links,
    ):
        converter._setup_connections(
            all_roads=[],
            connecting_roads=[],
            road_to_lanelet_ids={},
            lanelet_to_road_id={},
            junctions=[],
            routing_graph=sentinel_graph,
        )

    assert connecting.call_args.kwargs.get("routing_graph") is sentinel_graph
    assert outgoing.call_args.kwargs.get("routing_graph") is sentinel_graph
    # set_all_lane_links takes the graph positionally: (lanelet_map, roads, graph)
    assert lane_links.call_args.args[2] is sentinel_graph
