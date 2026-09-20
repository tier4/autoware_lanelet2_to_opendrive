"""A connector leaving the junction into several roads keeps every exit.

The exit-side mirror of ``test_junction_chain_merge``.  A road with
``@junction != -1`` owns exactly one road-level ``<successor>``, so a
connecting road whose lanes hand over to several roads *outside* the
junction used to name one of them and lose the rest: the other maneuvers
were reachable neither through the road link (one slot) nor through a
``<connection>``, whose ``connectingRoad`` was that same single road.

The representable form is the one the in-junction fork already uses --
duplicate what is shared.  The lateral group is emitted once per outside
road, the copies differ only in their ``<successor>``, and the first of them
claims the group's lanelets because a lanelet may name one ``(road, lane)``
pair.  The junction's ``<connection>`` table lists every copy, so the
external incoming road still reaches all of them.
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


def _line(point, x0, x1, y0, y1):
    """A three-vertex line from ``(x0, y0)`` to ``(x1, y1)``."""
    return lanelet2.core.LineString3d(
        lanelet2.core.getId(),
        [
            point(x0, y0),
            point((x0 + x1) / 2.0, (y0 + y1) / 2.0),
            point(x1, y1),
        ],
    )


def _straight(point, offset, x0, x1):
    return _line(point, x0, x1, offset, offset)


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
    road_to_lanelet_ids: dict[int, list[int]] = {}
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


def _exit_fork_fixture(exits: int):
    """``incoming -> connector -> one regular road per exit``.

    ``exits == 1`` is the ordinary single-exit connector; larger values are
    the exit-side fork.  Every outgoing road starts at the connector's end
    points, which is what makes it a routing successor.
    """
    lanelet_map = lanelet2.core.LaneletMap()
    point = _point_factory()
    incoming = _add_lanelet(
        lanelet_map,
        _straight(point, 3.0, 0.0, 20.0),
        _straight(point, 0.0, 0.0, 20.0),
    )
    connector = _add_lanelet(
        lanelet_map,
        _straight(point, 3.0, 20.0, 30.0),
        _straight(point, 0.0, 20.0, 30.0),
        turn="straight",
    )
    # Each exit leaves at its own lateral offset so the outgoing roads are
    # distinct geometry rather than copies of one another.
    outgoing = [
        _add_lanelet(
            lanelet_map,
            _line(point, 30.0, 50.0, 3.0, 3.0 + 8.0 * index),
            _line(point, 30.0, 50.0, 0.0, 0.0 + 8.0 * index),
        )
        for index in range(exits)
    ]

    regular_roads = [_build_road(lanelet_map, [incoming], 1)]
    lanelet_to_road_id = {incoming.id: 1}
    for index, out_lanelet in enumerate(outgoing):
        road_id = 2 + index
        regular_roads.append(_build_road(lanelet_map, [out_lanelet], road_id))
        lanelet_to_road_id[out_lanelet.id] = road_id

    return lanelet_map, [connector], regular_roads, lanelet_to_road_id, outgoing


def test_single_exit_connector_is_unchanged() -> None:
    """A connector with one outside exit keeps exactly one connecting road."""
    lanelet_map, junction_group, regular_roads, lanelet_to_road_id, _out = (
        _exit_fork_fixture(exits=1)
    )

    connecting, _to_roads, ll_to_road = _construct(
        lanelet_map, [junction_group], regular_roads, lanelet_to_road_id
    )

    assert len(connecting) == 1
    assert connecting[0].exit_fork_successor_road_id is None
    assert connecting[0].exit_fork_duplicate is False
    assert ll_to_road == {junction_group[0].id: connecting[0].id}


def test_exit_side_fork_keeps_every_exit() -> None:
    """The defect: a connector leaving into three roads kept only one.

    One connecting road per outside road, each naming its own
    ``<successor>``, and one ``<connection>`` row per copy so the external
    incoming road reaches all of them.
    """
    lanelet_map, junction_group, regular_roads, lanelet_to_road_id, _out = (
        _exit_fork_fixture(exits=3)
    )
    connector = junction_group[0]

    connecting, to_roads, ll_to_road = _construct(
        lanelet_map, [junction_group], regular_roads, lanelet_to_road_id
    )

    assert len(connecting) == 3, "every exit of the fork needs its own road"
    assert {road.exit_fork_successor_road_id for road in connecting} == {2, 3, 4}
    for road in connecting:
        assert road.exit_fork_lanelet_ids == [connector.id]

    merged = _wire_links(lanelet_map, connecting, lanelet_to_road_id, ll_to_road)

    assert {road.link.successor.element_id for road in connecting} == {2, 3, 4}
    for road in connecting:
        assert road.link.predecessor.element_id == 1

    connections = _connections(
        lanelet_map,
        junction_group,
        merged,
        to_roads[1000],
        regular_roads + connecting,
    )
    assert {c.incoming_road for c in connections} == {1}
    assert {c.connecting_road for c in connections} == {
        road.id for road in connecting
    }, "each exit of the fork must be reachable from the external incoming road"
    for connection in connections:
        assert connection.lane_links, "a connection without a laneLink is dead"


def test_exit_side_fork_claims_its_lanelets_exactly_once() -> None:
    """A lanelet may name one ``(road, lane)`` pair, so one copy claims it.

    The others carry the same lanes but stay out of the lanelet-to-lane
    mapping, which is how the sidecar and the geometric cross-check stay
    one-to-one.
    """
    lanelet_map, junction_group, regular_roads, lanelet_to_road_id, _out = (
        _exit_fork_fixture(exits=2)
    )
    connector = junction_group[0]

    connecting, _to_roads, ll_to_road = _construct(
        lanelet_map, [junction_group], regular_roads, lanelet_to_road_id
    )

    claiming = [road for road in connecting if not road.exit_fork_duplicate]
    duplicates = [road for road in connecting if road.exit_fork_duplicate]
    assert len(claiming) == 1
    assert len(duplicates) == 1
    assert ll_to_road == {connector.id: claiming[0].id}

    mapping = Road.set_all_lane_links(lanelet_map, regular_roads + connecting)
    assert (
        mapping[connector.id][0] == claiming[0].id
    ), "the duplicate must not overwrite the claiming copy's entry"


def test_exit_side_fork_keeps_the_lateral_lane_structure() -> None:
    """Each copy is the whole lateral group, not one road per lane.

    The lanes of a forking group all reach the same outside roads, so the
    fork is a property of the group; splitting it per lane would replace one
    two-lane connecting road with two one-lane ones and change the geometry
    of a case that has nothing wrong with it.
    """
    lanelet_map = lanelet2.core.LaneletMap()
    point = _point_factory()

    # Shared boundaries make the two lanelets of each stage laterally
    # adjacent, so they group into one road.
    in_mid = _straight(point, 3.0, 0.0, 20.0)
    incoming_right = _add_lanelet(lanelet_map, in_mid, _straight(point, 0.0, 0.0, 20.0))
    incoming_left = _add_lanelet(lanelet_map, _straight(point, 6.0, 0.0, 20.0), in_mid)

    conn_mid = _straight(point, 3.0, 20.0, 30.0)
    connector_right = _add_lanelet(
        lanelet_map, conn_mid, _straight(point, 0.0, 20.0, 30.0), turn="straight"
    )
    connector_left = _add_lanelet(
        lanelet_map, _straight(point, 6.0, 20.0, 30.0), conn_mid, turn="straight"
    )

    outgoing_roads = []
    for road_index, rise in enumerate((0.0, 12.0)):
        mid = _line(point, 30.0, 50.0, 3.0, 3.0 + rise)
        right = _add_lanelet(
            lanelet_map, mid, _line(point, 30.0, 50.0, 0.0, 0.0 + rise)
        )
        left = _add_lanelet(lanelet_map, _line(point, 30.0, 50.0, 6.0, 6.0 + rise), mid)
        outgoing_roads.append((2 + road_index, [right, left]))

    regular_roads = [_build_road(lanelet_map, [incoming_right, incoming_left], 1)]
    lanelet_to_road_id = {incoming_right.id: 1, incoming_left.id: 1}
    for road_id, group in outgoing_roads:
        regular_roads.append(_build_road(lanelet_map, group, road_id))
        for lanelet in group:
            lanelet_to_road_id[lanelet.id] = road_id

    junction_group = [connector_right, connector_left]
    connecting, _to_roads, ll_to_road = _construct(
        lanelet_map, [junction_group], regular_roads, lanelet_to_road_id
    )

    assert len(connecting) == 2, "one copy of the whole group per outside road"
    assert {road.exit_fork_successor_road_id for road in connecting} == {2, 3}
    for road in connecting:
        assert road.exit_fork_lanelet_ids == sorted(
            [connector_right.id, connector_left.id]
        )
        lanes = road.get_lanelet_to_lane_mapping()
        assert set(lanes) == {
            connector_right.id,
            connector_left.id,
        }, "a copy must keep both lanes of the group it stands for"
    # Only one copy claims them.
    assert set(ll_to_road) == {connector_right.id, connector_left.id}
    assert len(set(ll_to_road.values())) == 1


def test_chain_path_with_a_forking_exit_is_emitted_once_per_exit() -> None:
    """A merged in-junction path forks on its exit side too.

    ``incoming -> A -> {B, C} -> out``, where B and C each leave the junction
    into two outside roads.  The shared stage A already forces the chain
    merge; each merged path then has to be emitted once per outside road its
    last stage reaches, or the merge reinstates the very bottleneck it exists
    to remove one stage further on.
    """
    lanelet_map = lanelet2.core.LaneletMap()
    point = _point_factory()
    incoming = _add_lanelet(
        lanelet_map,
        _straight(point, 3.0, 0.0, 20.0),
        _straight(point, 0.0, 0.0, 20.0),
    )
    stage_a = _add_lanelet(
        lanelet_map,
        _straight(point, 3.0, 20.0, 30.0),
        _straight(point, 0.0, 20.0, 30.0),
        turn="straight",
    )
    branches = [
        _add_lanelet(
            lanelet_map,
            _line(point, 30.0, 40.0, 3.0, 3.0 + 6.0 * index),
            _line(point, 30.0, 40.0, 0.0, 0.0 + 6.0 * index),
            turn="straight",
        )
        for index in range(2)
    ]
    # Every branch leaves into the same two outside roads.
    regular_roads = [_build_road(lanelet_map, [incoming], 1)]
    lanelet_to_road_id = {incoming.id: 1}
    for index, branch in enumerate(branches):
        base = 3.0 + 6.0 * index
        for road_offset, rise in enumerate((0.0, 10.0)):
            road_id = 2 + 2 * index + road_offset
            out_lanelet = _add_lanelet(
                lanelet_map,
                _line(point, 40.0, 60.0, base, base + rise),
                _line(point, 40.0, 60.0, base - 3.0, base - 3.0 + rise),
            )
            regular_roads.append(_build_road(lanelet_map, [out_lanelet], road_id))
            lanelet_to_road_id[out_lanelet.id] = road_id

    junction_group = [stage_a, *branches]
    connecting, _to_roads, ll_to_road = _construct(
        lanelet_map, [junction_group], regular_roads, lanelet_to_road_id
    )

    paths = [tuple(road.chain_source_lanelet_ids or ()) for road in connecting]
    assert all(paths), "the shared stage forces every connector to be chain-merged"
    # Two branches, two outside roads each: four merged roads, one per
    # (path, outside road) pair.
    assert len(connecting) == 4
    assert sorted(paths) == sorted([(stage_a.id, branch.id) for branch in branches] * 2)
    assert {road.exit_fork_successor_road_id for road in connecting} == {2, 3, 4, 5}

    _wire_links(lanelet_map, connecting, lanelet_to_road_id, ll_to_road)
    by_key = {
        (tuple(road.chain_source_lanelet_ids), road.link.successor.element_id): road
        for road in connecting
    }
    assert len(by_key) == 4, "each (path, exit) pair is its own connecting road"
    connecting_ids = {road.id for road in connecting}
    for road in connecting:
        assert road.link.predecessor.element_id == 1
        assert road.link.successor.element_id not in connecting_ids, (
            "a connecting road must not have another connecting road as its "
            "successor"
        )


def test_duplicate_copies_are_recorded_in_the_mapping_sidecar() -> None:
    """The sidecar names each duplicate and the lanelets it realises.

    The XODR shows two identical connecting roads and no trace of which one
    claimed the source lanelets, so a consumer reading it back -- the
    geometric cross-check in ``analyze`` above all -- has to be told. The
    lanelet ids travel with the road ids because
    ``lanelet_to_road_and_lane`` can name only the claiming copy; without
    them a duplicate's lanes read as synthetic rather than as the same
    maneuver stage.
    """
    from autoware_lanelet2_to_opendrive.road_lanelet_geo_mapping import (
        GeoRoadLaneletMapping,
        _exit_fork_duplicate_roads,
    )

    lanelet_map, junction_group, regular_roads, lanelet_to_road_id, _out = (
        _exit_fork_fixture(exits=2)
    )
    connector = junction_group[0]

    connecting, _to_roads, _ll_to_road = _construct(
        lanelet_map, [junction_group], regular_roads, lanelet_to_road_id
    )

    duplicates = _exit_fork_duplicate_roads(regular_roads + connecting)
    claiming = [road for road in connecting if not road.exit_fork_duplicate]
    assert list(duplicates) == [
        road.id for road in connecting if road.exit_fork_duplicate
    ]
    assert claiming[0].id not in duplicates, "the claiming copy is not a duplicate"
    assert all(ids == [connector.id] for ids in duplicates.values())

    # Survives the JSON round trip the sidecar goes through.
    mapping = GeoRoadLaneletMapping(
        xodr_sha256="x",
        osm_sha256="o",
        exit_fork_duplicate_roads=duplicates,
    )
    assert (
        GeoRoadLaneletMapping.from_dict(mapping.to_dict()).exit_fork_duplicate_roads
        == duplicates
    )


def test_duplicate_copies_get_their_own_lanelet_attribution() -> None:
    """A duplicated stage names every ``(road, lane)`` it belongs to.

    ``lanelet_to_road_and_lane`` can name only the copy that claimed the
    lanelet, which is what made the shared stage unattributable: assigning
    it to either copy traded one missing transition for one extra.  The
    extra pairs go in ``lanelet_additional_road_lanes`` instead, so the
    relation can be read in full while the 1:1 field stays exactly as every
    existing consumer expects it.
    """
    from autoware_lanelet2_to_opendrive.road_lanelet_geo_mapping import (
        GeoRoadLaneletMapping,
        _additional_road_lanes,
    )

    lanelet_map, junction_group, regular_roads, lanelet_to_road_id, _out = (
        _exit_fork_fixture(exits=2)
    )
    connector = junction_group[0]

    connecting, _to_roads, _ll_to_road = _construct(
        lanelet_map, [junction_group], regular_roads, lanelet_to_road_id
    )
    roads = regular_roads + connecting

    # The record as the converter builds it: the duplicates claim nothing.
    primary: dict[int, tuple[int, int]] = {}
    for road in roads:
        if road.exit_fork_duplicate:
            continue
        for lanelet_id, lane_id in road.get_lanelet_to_lane_mapping().items():
            primary[lanelet_id] = (road.id, lane_id)

    extras = _additional_road_lanes(roads, primary)

    claiming = [road for road in connecting if not road.exit_fork_duplicate]
    dup_ids = {road.id for road in connecting if road.exit_fork_duplicate}
    assert dup_ids, "the fixture is supposed to produce at least one duplicate"

    # The shared connector lanelet is the one that gains pairs, and it gains
    # exactly the duplicates -- never the copy the record already names.
    assert set(extras) == {connector.id}
    assert {road_id for road_id, _lane in extras[connector.id]} == dup_ids
    assert primary[connector.id] not in extras[connector.id]
    assert primary[connector.id][0] == claiming[0].id

    # Extras only: the union with the 1:1 field is the complete relation,
    # and the two fields never restate each other.
    complete = {primary[connector.id], *extras[connector.id]}
    assert len(complete) == 1 + len(dup_ids)

    # Survives the JSON round trip the sidecar goes through.
    mapping = GeoRoadLaneletMapping(
        xodr_sha256="x",
        osm_sha256="o",
        lanelet_to_road_and_lane=primary,
        lanelet_additional_road_lanes=extras,
    )
    restored = GeoRoadLaneletMapping.from_dict(mapping.to_dict())
    assert restored.lanelet_additional_road_lanes == extras
    assert restored.lanelet_to_road_and_lane == primary
