"""Lane-level maneuver topology of junction connecting roads.

The junction decomposition must represent the Lanelet2 routing graph
exactly: multi-lane connectors whose lanes exit to different roads are
split (fan-out), connections exist only for maneuvers present in the
source routing graph (restricted fan-out), multi-stage connector chains
become one end-to-end connecting road, and parallel same-destination
maneuvers keep the legacy multi-lane connector.
"""

import lanelet2
import pytest

from autoware_lanelet2_to_opendrive.opendrive.junction import Junction
from autoware_lanelet2_to_opendrive.opendrive.road import Road


def _point_factory():
    """Shared Point3d cache: lanelet2 routing connects lanelets only when
    consecutive boundaries share the same point objects."""
    cache = {}

    def point(x, y):
        key = (round(float(x), 6), round(float(y), 6))
        if key not in cache:
            cache[key] = lanelet2.core.Point3d(
                lanelet2.core.getId(), float(x), float(y), 0.0
            )
        return cache[key]

    return point


def _straight_line(point, offset, x0, x1):
    return lanelet2.core.LineString3d(
        lanelet2.core.getId(),
        [
            point(x0, offset),
            point((x0 + x1) / 2.0, offset),
            point(x1, offset),
        ],
    )


def _add_lanelet(lanelet_map, left, right, turn=None):
    lanelet = lanelet2.core.Lanelet(lanelet2.core.getId(), left, right)
    lanelet.attributes["subtype"] = "road"
    lanelet.attributes["one_way"] = "yes"
    if turn is not None:
        lanelet.attributes["turn_direction"] = turn
    lanelet_map.add(lanelet)
    return lanelet


def _build_road(lanelet_map, group, road_id):
    return Road.construct_from_lanelet_groups(
        lanelet_map,
        group,
        road_id=road_id,
        traffic_rule="LHT",
    )


def _construct(lanelet_map, junction_groups, regular_roads, lanelet_to_road_id):
    return Road.construct_connecting_roads_from_junctions(
        lanelet_map=lanelet_map,
        junction_groups=junction_groups,
        starting_road_id=100,
        junction_id_offset=1000,
        traffic_rule="LHT",
        regular_roads=regular_roads,
        lanelet_to_road_id=dict(lanelet_to_road_id),
    )


def _connections(lanelet_map, junction_group, lanelet_to_road, road_ids, roads):
    return Junction.build_connections_from_roads(
        lanelet_map=lanelet_map,
        junction_lanelet_group=junction_group,
        junction_id=1000,
        lanelet_to_road_id=lanelet_to_road,
        connecting_road_ids=road_ids,
        roads=roads,
    )


def test_fan_out_connector_lanes_split_per_outgoing_road() -> None:
    """Fixture A: two parallel junction lanes exit to different roads."""
    lanelet_map = lanelet2.core.LaneletMap()
    point = _point_factory()
    lines_in = {o: _straight_line(point, o, 0.0, 20.0) for o in (0.0, 3.0, 6.0)}
    lines_j = {o: _straight_line(point, o, 20.0, 30.0) for o in (0.0, 3.0, 6.0)}
    incoming = [
        _add_lanelet(lanelet_map, lines_in[3.0], lines_in[0.0]),
        _add_lanelet(lanelet_map, lines_in[6.0], lines_in[3.0]),
    ]
    junction_lls = [
        _add_lanelet(lanelet_map, lines_j[3.0], lines_j[0.0], turn="straight"),
        _add_lanelet(lanelet_map, lines_j[6.0], lines_j[3.0], turn="straight"),
    ]
    out_a = [
        _add_lanelet(
            lanelet_map,
            _straight_line(point, 3.0, 30.0, 50.0),
            _straight_line(point, 0.0, 30.0, 50.0),
        )
    ]
    out_b = [
        _add_lanelet(
            lanelet_map,
            _straight_line(point, 6.0, 30.0, 50.0),
            _straight_line(point, 3.0, 30.0, 50.0),
        )
    ]
    roads = [
        _build_road(lanelet_map, incoming, 1),
        _build_road(lanelet_map, out_a, 2),
        _build_road(lanelet_map, out_b, 3),
    ]
    lanelet_to_road_id = {
        incoming[0].id: 1,
        incoming[1].id: 1,
        out_a[0].id: 2,
        out_b[0].id: 3,
    }
    connecting, _to_roads, ll_to_road, chain_traces = _construct(
        lanelet_map, [junction_lls], roads, lanelet_to_road_id
    )

    assert len(connecting) == 2, "fan-out lanes must not share one connector"
    assert not chain_traces
    merged_map = {**lanelet_to_road_id, **ll_to_road}
    Road.set_connecting_road_links(
        lanelet_map=lanelet_map,
        connecting_roads=connecting,
        lanelet_to_road_id=merged_map,
        road_to_lanelet_ids={
            road.id: [
                lanelet_id for lanelet_id, rid in ll_to_road.items() if rid == road.id
            ]
            for road in connecting
        },
    )
    successors = sorted(road.link.successor.element_id for road in connecting)
    assert successors == [2, 3]

    merged = {**lanelet_to_road_id, **ll_to_road}
    connections = _connections(
        lanelet_map,
        junction_lls,
        merged,
        [road.id for road in connecting],
        roads + connecting,
    )
    hops = {
        (c.incoming_road, link.from_lane, c.connecting_road)
        for c in connections
        for link in c.lane_links
    }
    by_connector = {road.link.successor.element_id: road.id for road in connecting}
    assert (1, 1, by_connector[2]) in hops
    assert (1, 2, by_connector[3]) in hops
    assert len(hops) == 2, f"no extra connections expected: {hops}"


def test_restricted_fan_out_creates_no_unallowed_connections() -> None:
    """Fixture B: nearby connectors reachable only from their own incoming."""
    lanelet_map = lanelet2.core.LaneletMap()
    point = _point_factory()
    incoming_a = [
        _add_lanelet(
            lanelet_map,
            _straight_line(point, 3.0, 0.0, 20.0),
            _straight_line(point, 0.0, 0.0, 20.0),
        )
    ]
    incoming_b = [
        _add_lanelet(
            lanelet_map,
            _straight_line(point, 9.0, 0.0, 20.0),
            _straight_line(point, 6.0, 0.0, 20.0),
        )
    ]
    junction_a = [
        _add_lanelet(
            lanelet_map,
            _straight_line(point, 3.0, 20.0, 30.0),
            _straight_line(point, 0.0, 20.0, 30.0),
            turn="straight",
        )
    ]
    junction_b = [
        _add_lanelet(
            lanelet_map,
            _straight_line(point, 9.0, 20.0, 30.0),
            _straight_line(point, 6.0, 20.0, 30.0),
            turn="straight",
        )
    ]
    out_a = [
        _add_lanelet(
            lanelet_map,
            _straight_line(point, 3.0, 30.0, 50.0),
            _straight_line(point, 0.0, 30.0, 50.0),
        )
    ]
    out_b = [
        _add_lanelet(
            lanelet_map,
            _straight_line(point, 9.0, 30.0, 50.0),
            _straight_line(point, 6.0, 30.0, 50.0),
        )
    ]
    roads = [
        _build_road(lanelet_map, incoming_a, 1),
        _build_road(lanelet_map, incoming_b, 2),
        _build_road(lanelet_map, out_a, 3),
        _build_road(lanelet_map, out_b, 4),
    ]
    lanelet_to_road_id = {
        incoming_a[0].id: 1,
        incoming_b[0].id: 2,
        out_a[0].id: 3,
        out_b[0].id: 4,
    }
    group = junction_a + junction_b
    connecting, _to_roads, ll_to_road, _traces = _construct(
        lanelet_map, [group], roads, lanelet_to_road_id
    )
    merged = {**lanelet_to_road_id, **ll_to_road}
    connections = _connections(
        lanelet_map,
        group,
        merged,
        [road.id for road in connecting],
        roads + connecting,
    )
    incoming_by_connector = {c.connecting_road: c.incoming_road for c in connections}
    connector_a = ll_to_road[junction_a[0].id]
    connector_b = ll_to_road[junction_b[0].id]
    assert incoming_by_connector[connector_a] == 1
    assert incoming_by_connector[connector_b] == 2
    assert len(connections) == 2, "no cross connections may be invented"


def test_connector_chain_becomes_one_end_to_end_road() -> None:
    """Fixture C: incoming -> connector A -> connector B -> outgoing."""
    lanelet_map = lanelet2.core.LaneletMap()
    point = _point_factory()
    incoming = [
        _add_lanelet(
            lanelet_map,
            _straight_line(point, 3.0, 0.0, 20.0),
            _straight_line(point, 0.0, 0.0, 20.0),
        )
    ]
    stage_one = [
        _add_lanelet(
            lanelet_map,
            _straight_line(point, 3.0, 20.0, 30.0),
            _straight_line(point, 0.0, 20.0, 30.0),
            turn="straight",
        )
    ]
    stage_two = [
        _add_lanelet(
            lanelet_map,
            _straight_line(point, 3.0, 30.0, 40.0),
            _straight_line(point, 0.0, 30.0, 40.0),
            turn="straight",
        )
    ]
    outgoing = [
        _add_lanelet(
            lanelet_map,
            _straight_line(point, 3.0, 40.0, 60.0),
            _straight_line(point, 0.0, 40.0, 60.0),
        )
    ]
    roads = [
        _build_road(lanelet_map, incoming, 1),
        _build_road(lanelet_map, outgoing, 2),
    ]
    lanelet_to_road_id = {incoming[0].id: 1, outgoing[0].id: 2}
    group = stage_one + stage_two
    connecting, _to_roads, ll_to_road, chain_traces = _construct(
        lanelet_map, [group], roads, lanelet_to_road_id
    )

    assert len(connecting) == 1, "the chain must merge into one connector"
    chain_road = connecting[0]
    assert chain_road.chain_source_lanelet_ids == [
        stage_one[0].id,
        stage_two[0].id,
    ]
    assert chain_road.length == pytest.approx(20.0, abs=1e-6)
    assert chain_road.link.predecessor.element_id == 1
    assert chain_road.link.successor.element_id == 2
    assert chain_traces == {chain_road.id: [stage_one[0].id, stage_two[0].id]}

    merged = {**lanelet_to_road_id, **ll_to_road}
    connections = _connections(
        lanelet_map,
        group,
        merged,
        [chain_road.id],
        roads + connecting,
    )
    hops = {
        (c.incoming_road, link.from_lane, c.connecting_road, link.to_lane)
        for c in connections
        for link in c.lane_links
    }
    assert hops == {(1, 1, chain_road.id, 1)}


def test_parallel_same_destination_maneuvers_keep_multi_lane_connector() -> None:
    """Fixture D: no fan-out — the legacy multi-lane connector is preserved."""
    lanelet_map = lanelet2.core.LaneletMap()
    point = _point_factory()
    lines_in = {o: _straight_line(point, o, 0.0, 20.0) for o in (0.0, 3.0, 6.0)}
    lines_j = {o: _straight_line(point, o, 20.0, 30.0) for o in (0.0, 3.0, 6.0)}
    lines_out = {o: _straight_line(point, o, 30.0, 50.0) for o in (0.0, 3.0, 6.0)}
    incoming = [
        _add_lanelet(lanelet_map, lines_in[3.0], lines_in[0.0]),
        _add_lanelet(lanelet_map, lines_in[6.0], lines_in[3.0]),
    ]
    junction_lls = [
        _add_lanelet(lanelet_map, lines_j[3.0], lines_j[0.0], turn="straight"),
        _add_lanelet(lanelet_map, lines_j[6.0], lines_j[3.0], turn="straight"),
    ]
    outgoing = [
        _add_lanelet(lanelet_map, lines_out[3.0], lines_out[0.0]),
        _add_lanelet(lanelet_map, lines_out[6.0], lines_out[3.0]),
    ]
    roads = [
        _build_road(lanelet_map, incoming, 1),
        _build_road(lanelet_map, outgoing, 2),
    ]
    lanelet_to_road_id = {
        incoming[0].id: 1,
        incoming[1].id: 1,
        outgoing[0].id: 2,
        outgoing[1].id: 2,
    }
    connecting, _to_roads, _ll_to_road, chain_traces = _construct(
        lanelet_map, [junction_lls], roads, lanelet_to_road_id
    )
    assert len(connecting) == 1, "same-destination lanes stay one connector"
    section = connecting[0].lanes.lane_sections[0]
    assert len(section.left_lanes) + len(section.right_lanes) == 2
    assert not chain_traces
