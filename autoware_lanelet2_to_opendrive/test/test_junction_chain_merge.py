"""In-junction chains become one connecting road per end-to-end maneuver.

A road with ``@junction != -1`` may only link to roads and owns a single
road-level ``<successor>``.  A connecting road whose lane forks to several
other connecting roads of the same junction therefore used to keep only one
branch: the alternatives were reachable neither through the road link (one
slot) nor through a ``<connection>`` (an ``incomingRoad`` that is itself a
connecting road crashes CARLA's parser and was deliberately removed in
090be208).

The representable form is to stop chaining connectors: every end-to-end
longitudinal path through the junction becomes its own connecting road, so
each connector runs from an outside road to an outside road and each fork
branch is a separate ``<connection>`` row of the external incoming road.
"""

import lanelet2
import pytest

from autoware_lanelet2_to_opendrive.opendrive.junction import Junction
from autoware_lanelet2_to_opendrive.opendrive.road import Road


def _point_factory():
    """Shared ``Point3d`` cache.

    lanelet2 routing connects two lanelets only when the consecutive
    boundaries share the *same* point objects, so every fixture coordinate
    has to come from one cache.
    """
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


def _wire_links(lanelet_map, connecting, lanelet_to_road_id, ll_to_road):
    merged = {**lanelet_to_road_id, **ll_to_road}
    road_to_lanelet_ids = {}
    for lanelet_id, road_id in ll_to_road.items():
        road_to_lanelet_ids.setdefault(road_id, []).append(lanelet_id)
    Road.set_connecting_road_links(
        lanelet_map=lanelet_map,
        connecting_roads=connecting,
        lanelet_to_road_id=merged,
        road_to_lanelet_ids=road_to_lanelet_ids,
    )
    return merged


def _connections(lanelet_map, junction_group, lanelet_to_road, road_ids, roads):
    return Junction.build_connections_from_roads(
        lanelet_map=lanelet_map,
        junction_lanelet_group=junction_group,
        junction_id=1000,
        lanelet_to_road_id=lanelet_to_road,
        connecting_road_ids=road_ids,
        roads=roads,
    )


def _chain_fixture(branches: int):
    """incoming -> connector A -> ``branches`` connectors -> one road each.

    ``branches == 1`` is the plain two-stage chain; larger values are the
    in-junction fork that motivates the merge.
    """
    lanelet_map = lanelet2.core.LaneletMap()
    point = _point_factory()
    incoming = [
        _add_lanelet(
            lanelet_map,
            _straight_line(point, 3.0, 0.0, 20.0),
            _straight_line(point, 0.0, 0.0, 20.0),
        )
    ]
    stage_one = _add_lanelet(
        lanelet_map,
        _straight_line(point, 3.0, 20.0, 30.0),
        _straight_line(point, 0.0, 20.0, 30.0),
        turn="straight",
    )
    # Each branch leaves the fork at its own lateral offset so the branches
    # are distinct geometry rather than duplicates of one another.
    stage_two = []
    outgoing = []
    for index in range(branches):
        shift = 6.0 * index
        stage_two.append(
            _add_lanelet(
                lanelet_map,
                lanelet2.core.LineString3d(
                    lanelet2.core.getId(),
                    [
                        point(30.0, 3.0),
                        point(35.0, 3.0 + shift),
                        point(40.0, 3.0 + shift),
                    ],
                ),
                lanelet2.core.LineString3d(
                    lanelet2.core.getId(),
                    [
                        point(30.0, 0.0),
                        point(35.0, 0.0 + shift),
                        point(40.0, 0.0 + shift),
                    ],
                ),
                turn="straight",
            )
        )
        outgoing.append(
            _add_lanelet(
                lanelet_map,
                _straight_line(point, 3.0 + shift, 40.0, 60.0),
                _straight_line(point, 0.0 + shift, 40.0, 60.0),
            )
        )
    regular_roads = [_build_road(lanelet_map, incoming, 1)]
    lanelet_to_road_id = {incoming[0].id: 1}
    for index, out_lanelet in enumerate(outgoing):
        road_id = 2 + index
        regular_roads.append(_build_road(lanelet_map, [out_lanelet], road_id))
        lanelet_to_road_id[out_lanelet.id] = road_id
    junction_group = [stage_one, *stage_two]
    return (
        lanelet_map,
        junction_group,
        regular_roads,
        lanelet_to_road_id,
        incoming[0],
        stage_one,
        stage_two,
        outgoing,
    )


def test_connector_chain_becomes_one_end_to_end_road() -> None:
    """A two-stage chain merges into a single incoming->outgoing connector."""
    (
        lanelet_map,
        junction_group,
        regular_roads,
        lanelet_to_road_id,
        _incoming,
        stage_one,
        stage_two,
        _outgoing,
    ) = _chain_fixture(branches=1)

    connecting, _to_roads, ll_to_road = _construct(
        lanelet_map, [junction_group], regular_roads, lanelet_to_road_id
    )

    assert len(connecting) == 1, "the chain must merge into one connecting road"
    chain_road = connecting[0]
    assert chain_road.chain_source_lanelet_ids == [stage_one.id, stage_two[0].id]
    # Nothing is shared with another path, so the merged lane owns both.
    assert sorted(chain_road.chain_owned_lanelet_ids) == sorted(
        [stage_one.id, stage_two[0].id]
    )
    assert chain_road.length == pytest.approx(20.0, abs=0.5)

    _wire_links(lanelet_map, connecting, lanelet_to_road_id, ll_to_road)
    assert chain_road.link.predecessor.element_id == 1
    assert chain_road.link.successor.element_id == 2


def test_in_junction_fork_keeps_every_branch() -> None:
    """The defect: a connector forking inside the junction lost all but one.

    One connecting road per branch, each linking an outside road to an
    outside road, and one ``<connection>`` row per branch.
    """
    (
        lanelet_map,
        junction_group,
        regular_roads,
        lanelet_to_road_id,
        _incoming,
        stage_one,
        stage_two,
        _outgoing,
    ) = _chain_fixture(branches=3)

    connecting, to_roads, ll_to_road = _construct(
        lanelet_map, [junction_group], regular_roads, lanelet_to_road_id
    )

    assert len(connecting) == 3, "every branch of the fork needs its own road"
    assert {tuple(road.chain_source_lanelet_ids) for road in connecting} == {
        (stage_one.id, branch.id) for branch in stage_two
    }
    # The shared prefix belongs to no single road's lane: a lanelet may name
    # only one (road, lane) pair, and claiming one branch would assert a
    # maneuver the other two do not make.
    for road in connecting:
        assert stage_one.id not in (road.chain_owned_lanelet_ids or [])

    merged = _wire_links(lanelet_map, connecting, lanelet_to_road_id, ll_to_road)

    connecting_ids = {road.id for road in connecting}
    for road in connecting:
        assert road.link.predecessor.element_id == 1
        successor = road.link.successor.element_id
        assert successor in {2, 3, 4}
        assert successor not in connecting_ids, (
            "no connecting road may have another connecting road as its "
            "successor: that is the single-<successor> bottleneck this "
            "merge removes"
        )
    assert {road.link.successor.element_id for road in connecting} == {2, 3, 4}

    connections = _connections(
        lanelet_map,
        junction_group,
        merged,
        to_roads[1000],
        regular_roads + connecting,
    )
    assert {c.incoming_road for c in connections} == {1}
    assert (
        {c.connecting_road for c in connections} == connecting_ids
    ), "each fork branch must be reachable from the external incoming road"
    for connection in connections:
        assert connection.lane_links, "a connection without a laneLink is dead"


def test_junction_without_a_chain_is_unchanged() -> None:
    """A single-stage junction keeps the ordinary lateral decomposition."""
    lanelet_map = lanelet2.core.LaneletMap()
    point = _point_factory()
    incoming = [
        _add_lanelet(
            lanelet_map,
            _straight_line(point, 3.0, 0.0, 20.0),
            _straight_line(point, 0.0, 0.0, 20.0),
        )
    ]
    connector = _add_lanelet(
        lanelet_map,
        _straight_line(point, 3.0, 20.0, 30.0),
        _straight_line(point, 0.0, 20.0, 30.0),
        turn="straight",
    )
    outgoing = [
        _add_lanelet(
            lanelet_map,
            _straight_line(point, 3.0, 30.0, 50.0),
            _straight_line(point, 0.0, 30.0, 50.0),
        )
    ]
    regular_roads = [
        _build_road(lanelet_map, incoming, 1),
        _build_road(lanelet_map, outgoing, 2),
    ]
    lanelet_to_road_id = {incoming[0].id: 1, outgoing[0].id: 2}

    connecting, _to_roads, ll_to_road = _construct(
        lanelet_map, [[connector]], regular_roads, lanelet_to_road_id
    )

    assert len(connecting) == 1
    assert connecting[0].chain_source_lanelet_ids is None
    assert ll_to_road == {connector.id: connecting[0].id}
