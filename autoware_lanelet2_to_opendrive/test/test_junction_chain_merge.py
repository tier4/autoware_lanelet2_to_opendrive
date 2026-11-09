"""In-junction chains become one connecting road per end-to-end maneuver.

A road with ``@junction != -1`` may only link to roads and owns a single
road-level ``<successor>``.  A connecting road whose lane forks to several
other connecting roads of the same junction therefore used to keep only one
branch: the alternatives were reachable neither through the road link (one
slot) nor through a ``<connection>`` (an ``incomingRoad`` that is itself a
connecting road crashes CARLA's parser and was deliberately removed in
090be208).

The representable form is to duplicate the shared stage: every end-to-end
longitudinal path through the junction becomes its own connecting road, so
each connector runs from an outside road to an outside road and each fork
branch is a separate ``<connection>`` row of the external incoming road.
"""

import lanelet2

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


def test_unshared_chain_keeps_its_per_stage_roads() -> None:
    """A chain nothing else branches off is already expressible as a chain.

    Merging it would buy no reachability and would cost the one-lanelet-per-
    lane relation that the lanelet-to-lane mapping and the geometric
    cross-check both rely on, so the legacy decomposition is kept.
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
    ) = _chain_fixture(branches=1)

    connecting, _to_roads, ll_to_road = _construct(
        lanelet_map, [junction_group], regular_roads, lanelet_to_road_id
    )

    assert len(connecting) == 2, "an unshared chain keeps one road per stage"
    assert all(road.chain_source_lanelet_ids is None for road in connecting)
    assert ll_to_road.keys() == {stage_one.id, stage_two[0].id}

    _wire_links(lanelet_map, connecting, lanelet_to_road_id, ll_to_road)
    first = next(r for r in connecting if r.id == ll_to_road[stage_one.id])
    second = next(r for r in connecting if r.id == ll_to_road[stage_two[0].id])
    assert first.link.predecessor.element_id == 1
    assert first.link.successor.element_id == second.id
    assert second.link.successor.element_id == 2


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
    # The shared stage is traversed by all three roads but claimed by exactly
    # one, so it keeps a single entry in the lanelet-to-lane mapping instead
    # of being dropped from it.
    owners = [
        road
        for road in connecting
        if stage_one.id in (road.chain_owned_lanelet_ids or [])
    ]
    assert len(owners) == 1
    assert {
        lanelet_id
        for road in connecting
        for lanelet_id in (road.chain_owned_lanelet_ids or [])
    } == {stage_one.id} | {branch.id for branch in stage_two}

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


def test_stage_that_also_leaves_the_junction_gets_its_own_road() -> None:
    """A stage can both continue inside the junction and exit it.

    ``incoming -> A -> B -> out_far`` and ``A -> out_near``. A connecting
    road links to one outside road at each end, so folding the ``A ->
    out_near`` maneuver into the ``A -> B`` path would drop it exactly the
    way the in-junction fork used to be dropped.
    """
    lanelet_map = lanelet2.core.LaneletMap()
    point = _point_factory()
    incoming = _add_lanelet(
        lanelet_map,
        _straight_line(point, 3.0, 0.0, 20.0),
        _straight_line(point, 0.0, 0.0, 20.0),
    )
    stage_a = _add_lanelet(
        lanelet_map,
        _straight_line(point, 3.0, 20.0, 30.0),
        _straight_line(point, 0.0, 20.0, 30.0),
        turn="straight",
    )
    stage_b = _add_lanelet(
        lanelet_map,
        _straight_line(point, 3.0, 30.0, 40.0),
        _straight_line(point, 0.0, 30.0, 40.0),
        turn="straight",
    )
    out_far = _add_lanelet(
        lanelet_map,
        _straight_line(point, 3.0, 40.0, 60.0),
        _straight_line(point, 0.0, 40.0, 60.0),
    )
    # Leaves the junction at the end of stage A, off to the side.
    out_near = _add_lanelet(
        lanelet_map,
        lanelet2.core.LineString3d(
            lanelet2.core.getId(),
            [point(30.0, 3.0), point(40.0, 13.0), point(50.0, 23.0)],
        ),
        lanelet2.core.LineString3d(
            lanelet2.core.getId(),
            [point(30.0, 0.0), point(40.0, 10.0), point(50.0, 20.0)],
        ),
    )
    regular_roads = [
        _build_road(lanelet_map, [incoming], 1),
        _build_road(lanelet_map, [out_far], 2),
        _build_road(lanelet_map, [out_near], 3),
    ]
    lanelet_to_road_id = {incoming.id: 1, out_far.id: 2, out_near.id: 3}

    connecting, _to_roads, ll_to_road = _construct(
        lanelet_map, [[stage_a, stage_b]], regular_roads, lanelet_to_road_id
    )

    paths = {tuple(road.chain_source_lanelet_ids) for road in connecting}
    assert paths == {(stage_a.id, stage_b.id), (stage_a.id,)}, (
        "the maneuver that leaves the junction at the end of stage A needs "
        "its own connecting road"
    )

    _wire_links(lanelet_map, connecting, lanelet_to_road_id, ll_to_road)
    by_path = {tuple(road.chain_source_lanelet_ids): road for road in connecting}
    assert by_path[(stage_a.id, stage_b.id)].link.successor.element_id == 2
    assert by_path[(stage_a.id,)].link.successor.element_id == 3
    for road in connecting:
        assert road.link.predecessor.element_id == 1
