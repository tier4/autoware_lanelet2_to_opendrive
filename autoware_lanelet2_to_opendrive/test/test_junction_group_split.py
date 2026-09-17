"""Regression for :func:`split_junction_groups_by_connections`.

Inside a junction, :func:`find_adjacent_groups` bundles laterally adjacent
connecting lanelets into one connecting road even when they come from or
lead to different roads.  Such a road can only carry one road-level
``<predecessor>`` and one ``<successor>``, so the lane links of the other
lanes name lanes of a road the road-level link does not point at; CARLA
resolves them against the linked road and crashes on a missing lane.

The Nishishinjuku fixture is the source of truth: it contains connecting
lanelet pairs whose successors lie in different regular roads, and a
three-lane chained connector whose lanes each come from and lead to a
separate single-lane connector (three roads, so three connecting roads).
"""

from autoware_lanelet2_to_opendrive.junction import (
    _filter_lanelets_inside_junction,
    _filter_lanelets_outside_junction,
    find_junction_groups,
)
from autoware_lanelet2_to_opendrive.util import (
    create_routing_graph,
    filter_lanelets_by_subtype,
    find_adjacent_groups,
    split_groups_by_divergent_connections,
    split_junction_groups_by_connections,
)

# Adjacent connecting lanelets whose successors lie in different regular roads.
FAN_OUT_PAIR = (3012728, 3012795)
# Three-lane chained connector: each lane comes from and continues into a
# different single-lane connector of the same junction.
CHAINED_TRIPLE = (3002147, 3002148, 3002149)
# Adjacent connecting lanelets that share both neighbouring roads and stay
# together, next to one (397) that leads elsewhere.
BUNDLED_PAIR = (398, 399)
BUNDLED_PAIR_NEIGHBOUR = 397


def _regular_road_of(lanelet_map, routing_graph):
    """lanelet id -> index of its regular road group (raw map, no preprocessing)."""
    road_lanelets = _filter_lanelets_outside_junction(
        filter_lanelets_by_subtype(
            lanelet_map.laneletLayer,
            ["road", "highway", "walkway", "road_shoulder"],
        )
    )
    groups = split_groups_by_divergent_connections(
        lanelet_map,
        find_adjacent_groups(lanelet_map, set(road_lanelets), routing_graph),
        routing_graph,
    )
    return {ll.id: gid for gid, group in enumerate(groups) for ll in group}


def _junction_lateral_groups(lanelet_map, routing_graph):
    """[(lateral groups of one junction), ...] before the split."""
    junction_lanelets = _filter_lanelets_inside_junction(list(lanelet_map.laneletLayer))
    return [
        find_adjacent_groups(lanelet_map, set(junction_group), routing_graph)
        for junction_group in find_junction_groups(junction_lanelets)
    ]


def _group_index(groups, lanelet_id):
    for index, group in enumerate(groups):
        if any(ll.id == lanelet_id for ll in group):
            return index
    raise AssertionError(f"lanelet {lanelet_id} not in any group")


def _neighbour_keys(lanelet, refined, road_of, routing_graph):
    """(predecessor keys, successor keys) as the splitter sees them."""
    inside = {ll.id: gid for gid, group in enumerate(refined) for ll in group}

    def key(neighbour):
        if neighbour.id in inside:
            return ("group", inside[neighbour.id])
        if neighbour.id in road_of:
            return ("road", road_of[neighbour.id])
        return ("lanelet", neighbour.id)

    return (
        frozenset(key(n) for n in routing_graph.previous(lanelet)),
        frozenset(key(n) for n in routing_graph.following(lanelet)),
    )


def test_junction_split_is_a_refinement(lanelet_map):
    """The splitter only subdivides lateral groups; it never merges or drops."""
    routing_graph = create_routing_graph(lanelet_map)
    road_of = _regular_road_of(lanelet_map, routing_graph)
    for groups in _junction_lateral_groups(lanelet_map, routing_graph):
        refined = split_junction_groups_by_connections(
            lanelet_map, groups, routing_graph, road_of
        )
        original = [frozenset(ll.id for ll in g) for g in groups]
        for group in refined:
            ids = frozenset(ll.id for ll in group)
            assert ids and any(ids <= parent for parent in original)
        assert sorted(ll.id for g in refined for ll in g) == sorted(
            ll.id for g in groups for ll in g
        )


def test_junction_split_unifies_neighbour_roads(lanelet_map):
    """After the split every group agrees on where its lanes come from and go."""
    routing_graph = create_routing_graph(lanelet_map)
    road_of = _regular_road_of(lanelet_map, routing_graph)
    for groups in _junction_lateral_groups(lanelet_map, routing_graph):
        refined = split_junction_groups_by_connections(
            lanelet_map, groups, routing_graph, road_of
        )
        for group in refined:
            keys = {
                _neighbour_keys(ll, refined, road_of, routing_graph) for ll in group
            }
            assert len(keys) == 1, [ll.id for ll in group]


def test_junction_split_separates_lanes_leading_to_different_roads(lanelet_map):
    """Adjacent connecting lanelets with different successor roads part ways."""
    routing_graph = create_routing_graph(lanelet_map)
    road_of = _regular_road_of(lanelet_map, routing_graph)
    for groups in _junction_lateral_groups(lanelet_map, routing_graph):
        if not any(ll.id == FAN_OUT_PAIR[0] for g in groups for ll in g):
            continue
        assert _group_index(groups, FAN_OUT_PAIR[0]) == _group_index(
            groups, FAN_OUT_PAIR[1]
        )
        refined = split_junction_groups_by_connections(
            lanelet_map, groups, routing_graph, road_of
        )
        assert _group_index(refined, FAN_OUT_PAIR[0]) != _group_index(
            refined, FAN_OUT_PAIR[1]
        )
        return
    raise AssertionError("fan-out pair not found in any junction")


def test_junction_split_separates_chained_lanes_with_different_connectors(
    lanelet_map,
):
    """Chained lanes are keyed by their neighbouring connecting groups.

    Each lane of the triple has its own upstream and downstream single-lane
    connector, so one connecting road could not name them all in its
    road-level links; the lanes become three connecting roads.
    """
    routing_graph = create_routing_graph(lanelet_map)
    road_of = _regular_road_of(lanelet_map, routing_graph)
    for groups in _junction_lateral_groups(lanelet_map, routing_graph):
        if not any(ll.id == CHAINED_TRIPLE[0] for g in groups for ll in g):
            continue
        assert len({_group_index(groups, lid) for lid in CHAINED_TRIPLE}) == 1
        for lid in CHAINED_TRIPLE:
            lanelet = lanelet_map.laneletLayer.get(lid)
            assert len(routing_graph.following(lanelet)) == 1
            assert len(routing_graph.previous(lanelet)) == 1
        refined = split_junction_groups_by_connections(
            lanelet_map, groups, routing_graph, road_of
        )
        assert len({_group_index(refined, lid) for lid in CHAINED_TRIPLE}) == 3
        return
    raise AssertionError("chained triple not found in any junction")


def test_junction_split_keeps_lanes_with_the_same_neighbours_together(lanelet_map):
    """Only the lane that leads elsewhere leaves the group; the rest stay bundled."""
    routing_graph = create_routing_graph(lanelet_map)
    road_of = _regular_road_of(lanelet_map, routing_graph)
    for groups in _junction_lateral_groups(lanelet_map, routing_graph):
        if not any(ll.id == BUNDLED_PAIR[0] for g in groups for ll in g):
            continue
        assert (
            len(
                {
                    _group_index(groups, lid)
                    for lid in (*BUNDLED_PAIR, BUNDLED_PAIR_NEIGHBOUR)
                }
            )
            == 1
        )
        refined = split_junction_groups_by_connections(
            lanelet_map, groups, routing_graph, road_of
        )
        assert _group_index(refined, BUNDLED_PAIR[0]) == _group_index(
            refined, BUNDLED_PAIR[1]
        )
        assert _group_index(refined, BUNDLED_PAIR_NEIGHBOUR) != _group_index(
            refined, BUNDLED_PAIR[0]
        )
        return
    raise AssertionError("bundled pair not found in any junction")


def test_junction_split_only_subdivides_groups_that_need_it(lanelet_map):
    """Groups whose lanes share their neighbours are passed through untouched."""
    routing_graph = create_routing_graph(lanelet_map)
    road_of = _regular_road_of(lanelet_map, routing_graph)
    before = after = multi_before = multi_after = 0
    for groups in _junction_lateral_groups(lanelet_map, routing_graph):
        refined = split_junction_groups_by_connections(
            lanelet_map, groups, routing_graph, road_of
        )
        before += len(groups)
        after += len(refined)
        multi_before += sum(len(g) > 1 for g in groups)
        multi_after += sum(len(g) > 1 for g in refined)
    # 346 lateral groups, 34 multi-lane; 13 split: 11 pairs and the chained
    # triple into single lanes, [397, 398, 399] into [397] + [398, 399]
    assert (before, after) == (346, 360)
    assert (multi_before, multi_after) == (34, 22)


def test_junction_split_without_road_mapping_splits_only_along_connectors(
    lanelet_map,
):
    """Unmapped outside neighbours cannot be told apart, so they never split a group."""
    routing_graph = create_routing_graph(lanelet_map)
    for groups in _junction_lateral_groups(lanelet_map, routing_graph):
        ids = {ll.id for g in groups for ll in g}
        refined = split_junction_groups_by_connections(
            lanelet_map, groups, routing_graph
        )
        if FAN_OUT_PAIR[0] in ids:
            assert _group_index(refined, FAN_OUT_PAIR[0]) == _group_index(
                refined, FAN_OUT_PAIR[1]
            )
        if CHAINED_TRIPLE[0] in ids:
            assert len({_group_index(refined, lid) for lid in CHAINED_TRIPLE}) == 3


def test_junction_split_empty_input(lanelet_map):
    assert split_junction_groups_by_connections(lanelet_map, []) == []
