"""Regression for :func:`split_groups_by_divergent_connections`.

Companion to ``specs/2026-05-22-road-successor-mis-merge-design.md``.

The default :func:`find_adjacent_groups` groups lanelets purely by
lateral adjacency. That over-merges groups whose lanes diverge to
different successor roads, producing a single OpenDRIVE road that can
only carry a road-level link to one of them and silently drops the
remaining lanes' ``<lane><link><successor/></link></lane>`` entries.

These tests assert the two halves of the contract:

* the split-aware grouper produces a strict refinement of the lateral
  grouping (only ever splits, never merges across pre-existing
  boundaries);
* after the refinement every group's lanelets agree on which group
  their routing-graph followings land in — the invariant that prevents
  the silent lane drop downstream.

The Nishishinjuku fixture is the source of truth because it exhibits
all four real-world divergent groups (roads 17 / 79 / 82 of the
generated XODR, per the design doc).
"""

import lanelet2

from autoware_lanelet2_to_opendrive.util import (
    create_routing_graph,
    filter_lanelets_by_subtype,
    find_adjacent_groups,
    split_groups_by_divergent_connections,
)


def _road_lanelet_groups(lanelet_map):
    """Build the same (road, highway, walkway, road_shoulder) grouping the
    converter uses."""
    from autoware_lanelet2_to_opendrive.junction import (
        _filter_lanelets_outside_junction,
    )

    road_lanelets = _filter_lanelets_outside_junction(
        filter_lanelets_by_subtype(
            lanelet_map.laneletLayer,
            ["road", "highway", "walkway", "road_shoulder"],
        )
    )
    routing_graph = create_routing_graph(lanelet_map)
    groups = find_adjacent_groups(lanelet_map, set(road_lanelets), routing_graph)
    return groups, routing_graph


def test_split_groups_is_a_refinement(lanelet_map):
    """The split-aware grouper never moves a lanelet across pre-existing
    lateral boundaries — it only subdivides."""
    groups, routing_graph = _road_lanelet_groups(lanelet_map)
    refined = split_groups_by_divergent_connections(
        lanelet_map, groups, routing_graph
    )

    original_ids = [frozenset(ll.id for ll in g) for g in groups]
    refined_ids = [frozenset(ll.id for ll in g) for g in refined]

    # Lanelet partition is preserved.
    assert {ll for ids in original_ids for ll in ids} == {
        ll for ids in refined_ids for ll in ids
    }, "split must not lose or invent lanelets"

    # Every refined group must be a subset of exactly one original group
    # (refinement, not arbitrary repartitioning).
    for refined_group in refined_ids:
        containing = [orig for orig in original_ids if refined_group <= orig]
        assert len(containing) == 1, (
            f"refined group {sorted(refined_group)[:5]}... must be a subset "
            f"of exactly one original lateral group, got {len(containing)} matches"
        )


def test_split_groups_unifies_successor_signature(lanelet_map):
    """After the split every group's lanelets must agree on the set of
    neighbouring (already-split) groups their followings land in. That is
    the invariant that prevents the road-level successor from silently
    dropping a lane's downstream road."""
    groups, routing_graph = _road_lanelet_groups(lanelet_map)
    refined = split_groups_by_divergent_connections(
        lanelet_map, groups, routing_graph
    )

    ll_to_gid = {
        ll.id: gid for gid, group in enumerate(refined) for ll in group
    }

    for gid, group in enumerate(refined):
        signatures = {
            frozenset(
                ll_to_gid[s.id]
                for s in routing_graph.following(lanelet)
                if s.id in ll_to_gid
            )
            for lanelet in group
        }
        assert len(signatures) == 1, (
            f"refined group {gid} still contains lanelets with divergent "
            f"successor-group signatures: {signatures}"
        )


def test_split_groups_actually_splits_nishishinjuku(lanelet_map):
    """Sanity guard: the Nishishinjuku fixture's well-known divergent
    groups (corresponding to roads 17 / 79 / 82 in the generated XODR)
    must trigger at least one split, otherwise the test above is
    vacuously satisfied by the input grouping."""
    groups, routing_graph = _road_lanelet_groups(lanelet_map)
    refined = split_groups_by_divergent_connections(
        lanelet_map, groups, routing_graph
    )
    assert len(refined) > len(groups), (
        "Nishishinjuku is expected to contain at least one divergent group; "
        "if split count is unchanged the helper is a no-op on real data"
    )


def test_split_groups_empty_input(lanelet_map):
    """Empty input must round-trip to empty output."""
    routing_graph = create_routing_graph(lanelet_map)
    assert (
        split_groups_by_divergent_connections(lanelet_map, [], routing_graph)
        == []
    )


def test_split_groups_singleton_lanelet_groups_unchanged(lanelet_map):
    """A group with a single lanelet has nothing to split — must round-trip
    unchanged (set equality, since the helper rebuilds the set)."""
    # Pick a real lanelet from the map.
    any_lanelet = next(iter(lanelet_map.laneletLayer))
    routing_graph = create_routing_graph(lanelet_map)

    out = split_groups_by_divergent_connections(
        lanelet_map, [{any_lanelet}], routing_graph
    )
    assert len(out) == 1
    assert {ll.id for ll in out[0]} == {any_lanelet.id}


def _is_lanelet(obj) -> bool:
    return isinstance(obj, (lanelet2.core.Lanelet, lanelet2.core.ConstLanelet))


def test_split_groups_preserves_lanelet_types(lanelet_map):
    """Defensive: the output must contain real Lanelet instances, not bare
    ids — the consumer (``Road.construct_from_lanelet_groups``) calls
    ``ll.id`` and other Lanelet methods on each element."""
    groups, routing_graph = _road_lanelet_groups(lanelet_map)
    refined = split_groups_by_divergent_connections(
        lanelet_map, groups, routing_graph
    )
    for group in refined:
        for lanelet in group:
            assert _is_lanelet(lanelet), (
                f"split must return Lanelet instances, got {type(lanelet)}"
            )
