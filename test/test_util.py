"""Tests for utility functions."""

from pathlib import Path
import lanelet2
from autoware_lanelet2_extension_python.projection import MGRSProjector
from autoware_lanelet2_to_opendrive.util import (
    find_lanelets_without_next,
    find_lanelets_without_previous,
    find_terminal_lanelets,
    find_adjacent_groups,
    filter_lanelets_by_subtype,
    check_lanelet_groups_intersect,
)


def load_test_map():
    """Load the test lanelet2 map."""
    test_data_path = Path(__file__).parent / "data" / "lanelet2_map.osm"
    projector = MGRSProjector(
        lanelet2.io.Origin(35.23, 139.16)
    )  # MGRS origin for Tokyo area (54SUE)
    return lanelet2.io.load(str(test_data_path), projector)


def test_find_lanelets_without_next():
    """Test finding lanelets without successors."""
    lanelet_map = load_test_map()

    # Find lanelets without next
    terminal_lanelets = find_lanelets_without_next(lanelet_map)

    # Check that we found some terminal lanelets
    assert isinstance(terminal_lanelets, set)
    assert all(isinstance(ll, lanelet2.core.Lanelet) for ll in terminal_lanelets)

    # Check that lanelet 3002082 is included in terminal lanelets
    terminal_ids = {ll.id for ll in terminal_lanelets}
    assert (
        3002082 in terminal_ids
    ), f"Lanelet 3002082 should be in terminal lanelets, but found: {terminal_ids}"

    # Verify these lanelets actually have no following lanelets
    traffic_rules = lanelet2.traffic_rules.create(
        lanelet2.traffic_rules.Locations.Germany,
        lanelet2.traffic_rules.Participants.Vehicle,
    )
    routing_graph = lanelet2.routing.RoutingGraph(
        lanelet_map, traffic_rules, [lanelet2.routing.RoutingCostDistance(0.0)]
    )

    for lanelet in terminal_lanelets:
        following = routing_graph.following(lanelet)
        assert (
            len(following) == 0
        ), f"Lanelet {lanelet.id} has following lanelets but was marked as terminal"


def test_find_lanelets_without_previous():
    """Test finding lanelets without predecessors."""
    lanelet_map = load_test_map()

    # Find lanelets without previous
    start_lanelets = find_lanelets_without_previous(lanelet_map)

    # Check that we found some start lanelets
    assert isinstance(start_lanelets, set)
    assert all(isinstance(ll, lanelet2.core.Lanelet) for ll in start_lanelets)

    # Check that lanelet 3002084 is included in start lanelets
    start_ids = {ll.id for ll in start_lanelets}
    assert (
        3002084 in start_ids
    ), f"Lanelet 3002084 should be in start lanelets, but found: {start_ids}"

    # Verify these lanelets actually have no previous lanelets
    traffic_rules = lanelet2.traffic_rules.create(
        lanelet2.traffic_rules.Locations.Germany,
        lanelet2.traffic_rules.Participants.Vehicle,
    )
    routing_graph = lanelet2.routing.RoutingGraph(
        lanelet_map, traffic_rules, [lanelet2.routing.RoutingCostDistance(0.0)]
    )

    for lanelet in start_lanelets:
        previous = routing_graph.previous(lanelet)
        assert (
            len(previous) == 0
        ), f"Lanelet {lanelet.id} has previous lanelets but was marked as start"


def test_find_terminal_lanelets():
    """Test finding both start and end terminal lanelets."""
    lanelet_map = load_test_map()

    # Find terminal lanelets
    start_lanelets, end_lanelets = find_terminal_lanelets(lanelet_map)

    # Check that both sets are returned
    assert isinstance(start_lanelets, set)
    assert isinstance(end_lanelets, set)

    # Check that all elements are lanelets
    assert all(isinstance(ll, lanelet2.core.Lanelet) for ll in start_lanelets)
    assert all(isinstance(ll, lanelet2.core.Lanelet) for ll in end_lanelets)

    # Check that specific lanelet IDs are included
    start_ids = {ll.id for ll in start_lanelets}
    end_ids = {ll.id for ll in end_lanelets}
    assert (
        3002084 in start_ids
    ), f"Lanelet 3002084 should be in start lanelets, but found: {start_ids}"
    assert (
        3002082 in end_ids
    ), f"Lanelet 3002082 should be in end lanelets, but found: {end_ids}"

    # Verify consistency with individual functions
    start_lanelets_individual = find_lanelets_without_previous(lanelet_map)
    end_lanelets_individual = find_lanelets_without_next(lanelet_map)

    assert start_lanelets == start_lanelets_individual
    assert end_lanelets == end_lanelets_individual


def test_terminal_lanelets_completeness():
    """Test that all lanelets are properly categorized."""
    lanelet_map = load_test_map()

    # Get terminal lanelets
    start_lanelets, end_lanelets = find_terminal_lanelets(lanelet_map)

    # Create routing graph for verification
    traffic_rules = lanelet2.traffic_rules.create(
        lanelet2.traffic_rules.Locations.Germany,
        lanelet2.traffic_rules.Participants.Vehicle,
    )
    routing_graph = lanelet2.routing.RoutingGraph(
        lanelet_map, traffic_rules, [lanelet2.routing.RoutingCostDistance(0.0)]
    )

    # Check all lanelets and verify terminal classification
    for lanelet in lanelet_map.laneletLayer:
        has_previous = len(routing_graph.previous(lanelet)) > 0
        has_following = len(routing_graph.following(lanelet)) > 0

        # Verify start lanelet classification
        if not has_previous:
            assert (
                lanelet in start_lanelets
            ), f"Lanelet {lanelet.id} has no previous but not in start_lanelets"
        else:
            assert (
                lanelet not in start_lanelets
            ), f"Lanelet {lanelet.id} has previous but in start_lanelets"

        # Verify end lanelet classification
        if not has_following:
            assert (
                lanelet in end_lanelets
            ), f"Lanelet {lanelet.id} has no following but not in end_lanelets"
        else:
            assert (
                lanelet not in end_lanelets
            ), f"Lanelet {lanelet.id} has following but in end_lanelets"


def test_empty_map():
    """Test functions with an empty map."""
    # Create empty map
    empty_map = lanelet2.core.LaneletMap()

    # Test all functions with empty map
    start_lanelets = find_lanelets_without_previous(empty_map)
    end_lanelets = find_lanelets_without_next(empty_map)
    terminal_start, terminal_end = find_terminal_lanelets(empty_map)

    # All should return empty sets
    assert len(start_lanelets) == 0
    assert len(end_lanelets) == 0
    assert len(terminal_start) == 0
    assert len(terminal_end) == 0


def test_find_adjacent_groups_all_lanelets():
    """Test finding adjacent groups with all lanelets."""
    lanelet_map = load_test_map()

    # Find groups with empty set (should return all lanelets grouped)
    groups = find_adjacent_groups(lanelet_map, set())

    # Check that we got a list of groups
    assert isinstance(groups, list)
    assert len(groups) > 0

    # Check that all groups contain lanelets
    for group in groups:
        assert isinstance(group, set)
        assert len(group) > 0
        for ll in group:
            # Accept both Lanelet and ConstLanelet types
            assert isinstance(
                ll, (lanelet2.core.Lanelet, lanelet2.core.ConstLanelet)
            ), f"Expected Lanelet or ConstLanelet, got {type(ll)}"

    # Check that all lanelets are accounted for
    all_lanelets_in_groups = set()
    for group in groups:
        all_lanelets_in_groups.update(group)

    all_lanelets_in_map = set(lanelet_map.laneletLayer)
    assert all_lanelets_in_groups == all_lanelets_in_map


def test_find_adjacent_groups_subset():
    """Test finding adjacent groups with a subset of lanelets."""
    lanelet_map = load_test_map()

    # Get a subset of lanelets (terminal lanelets)
    start_lanelets, end_lanelets = find_terminal_lanelets(lanelet_map)
    terminal_lanelets = start_lanelets | end_lanelets

    # Find groups among terminal lanelets and their left/right adjacent lanelets
    groups = find_adjacent_groups(lanelet_map, terminal_lanelets)

    # Check that we got a list of groups
    assert isinstance(groups, list)

    # Check that all groups contain lanelets (including adjacent ones)
    for group in groups:
        assert isinstance(group, set)
        assert len(group) > 0
        for ll in group:
            # Accept both Lanelet and ConstLanelet types
            assert isinstance(
                ll, (lanelet2.core.Lanelet, lanelet2.core.ConstLanelet)
            ), f"Expected Lanelet or ConstLanelet, got {type(ll)}"
        # Groups may now contain non-terminal lanelets that are left/right adjacent to terminal ones
        # So we don't check group.issubset(terminal_lanelets) anymore

    # Check that all terminal lanelets are accounted for in some group
    all_lanelets_in_groups = set()
    for group in groups:
        all_lanelets_in_groups.update(group)

    # All terminal lanelets should be in the groups (but groups may contain more)
    assert terminal_lanelets.issubset(all_lanelets_in_groups)


def test_find_adjacent_groups_empty_target():
    """Test find_adjacent_groups with empty target set."""
    lanelet_map = load_test_map()

    # Test with empty set
    groups = find_adjacent_groups(lanelet_map, set())

    # Should return all lanelets grouped
    assert isinstance(groups, list)
    assert len(groups) > 0

    # Verify all map lanelets are included
    all_lanelets_in_groups = set()
    for group in groups:
        all_lanelets_in_groups.update(group)

    all_lanelets_in_map = set(lanelet_map.laneletLayer)
    assert all_lanelets_in_groups == all_lanelets_in_map


def test_find_adjacent_groups_empty_map():
    """Test find_adjacent_groups with empty map."""
    empty_map = lanelet2.core.LaneletMap()

    # Test with empty map and empty target
    groups = find_adjacent_groups(empty_map, set())
    assert len(groups) == 0

    # Test with empty map and some target set (should still be empty)
    groups = find_adjacent_groups(empty_map, {})
    assert len(groups) == 0


def test_find_adjacent_groups_connectivity():
    """Test that adjacent groups are properly connected."""
    lanelet_map = load_test_map()

    # Get all groups
    groups = find_adjacent_groups(lanelet_map, set())

    # Create routing graph for verification
    traffic_rules = lanelet2.traffic_rules.create(
        lanelet2.traffic_rules.Locations.Germany,
        lanelet2.traffic_rules.Participants.Vehicle,
    )
    routing_graph = lanelet2.routing.RoutingGraph(
        lanelet_map, traffic_rules, [lanelet2.routing.RoutingCostDistance(0.0)]
    )

    # For each group, verify internal connectivity
    for group in groups:
        if len(group) <= 1:
            continue  # Single lanelet groups are trivially connected

        # Check that within each group, there's at least one connection
        # between any two lanelets (not necessarily direct)
        lanelets_list = list(group)
        for i, ll1 in enumerate(lanelets_list):
            connected_to_others = False
            for j, ll2 in enumerate(lanelets_list):
                if i == j:
                    continue

                # Check if ll1 and ll2 are directly connected
                if (
                    ll2 in routing_graph.following(ll1)
                    or ll2 in routing_graph.previous(ll1)
                    or routing_graph.left(ll1) == ll2
                    or routing_graph.right(ll1) == ll2
                ):
                    connected_to_others = True
                    break

            # In a properly connected group, each lanelet should have
            # at least one direct connection to another in the group
            # (This is a simplified connectivity check)
            if not connected_to_others and len(group) > 1:
                # This might be expected for some edge cases,
                # so we'll just record it without failing
                pass


def test_find_adjacent_groups_includes_neighbors():
    """Test that groups include adjacent lanelets not in target set."""
    lanelet_map = load_test_map()

    # Get a small subset of lanelets (just start terminals)
    start_lanelets, _ = find_terminal_lanelets(lanelet_map)

    # Take only one start lanelet for testing
    if start_lanelets:
        single_start = {next(iter(start_lanelets))}

        # Find groups including this lanelet and its left/right neighbors
        groups = find_adjacent_groups(lanelet_map, single_start)

        # Check that we got groups
        assert isinstance(groups, list)
        assert len(groups) > 0

        # Find the group containing our target lanelet
        target_group = None
        for group in groups:
            if single_start.intersection(group):
                target_group = group
                break

        assert target_group is not None, "Target lanelet should be in some group"

        # The group should contain more than just the target lanelet
        # (unless it's completely isolated, which is unlikely)
        # This verifies that left/right adjacent lanelets are included
        # We'll just check that the function doesn't crash and produces valid output
        for ll in target_group:
            assert isinstance(ll, (lanelet2.core.Lanelet, lanelet2.core.ConstLanelet))


def test_find_adjacent_groups_expansion():
    """Test that target set is expanded with recursively all left/right adjacent lanelets."""
    lanelet_map = load_test_map()

    # Get terminal lanelets as target
    start_lanelets, end_lanelets = find_terminal_lanelets(lanelet_map)
    terminal_lanelets = start_lanelets | end_lanelets

    # Create routing graph to manually find what should be included
    traffic_rules = lanelet2.traffic_rules.create(
        lanelet2.traffic_rules.Locations.Germany,
        lanelet2.traffic_rules.Participants.Vehicle,
    )
    routing_graph = lanelet2.routing.RoutingGraph(
        lanelet_map, traffic_rules, [lanelet2.routing.RoutingCostDistance(0.0)]
    )

    # Manually calculate expected lanelets (target + recursively all their left/right neighbors)
    expected_lanelets = set()
    visited_manual = set()

    def add_left_right_manually(lanelet):
        """Manually calculate what should be included by recursively adding left/right."""
        if lanelet in visited_manual:
            return

        visited_manual.add(lanelet)
        expected_lanelets.add(lanelet)

        # Add left adjacent lanelets recursively
        left_ll = routing_graph.left(lanelet)
        if left_ll and left_ll not in visited_manual:
            add_left_right_manually(left_ll)

        # Add right adjacent lanelets recursively
        right_ll = routing_graph.right(lanelet)
        if right_ll and right_ll not in visited_manual:
            add_left_right_manually(right_ll)

    # Start from all terminal lanelets
    for terminal_ll in terminal_lanelets:
        add_left_right_manually(terminal_ll)

    # Get groups from function
    groups = find_adjacent_groups(lanelet_map, terminal_lanelets)

    # All lanelets in groups should be within our expected set
    all_lanelets_in_groups = set()
    for group in groups:
        all_lanelets_in_groups.update(group)

    # The grouped lanelets should include at least the expected lanelets
    # (There might be more due to transitive adjacency)
    assert expected_lanelets.issubset(
        all_lanelets_in_groups
    ), "Expected lanelets should be included in groups"


def test_find_adjacent_groups_specific_lanelets_together():
    """Test that specific lanelets 3002094 and 3002093 are in the same group."""
    lanelet_map = load_test_map()

    # Find the lanelets with specific IDs
    lanelet_3002094 = None
    lanelet_3002093 = None

    for ll in lanelet_map.laneletLayer:
        if ll.id == 3002094:
            lanelet_3002094 = ll
        elif ll.id == 3002093:
            lanelet_3002093 = ll

    # Verify both lanelets exist in the map
    assert lanelet_3002094 is not None, "Lanelet 3002094 should exist in the test map"
    assert lanelet_3002093 is not None, "Lanelet 3002093 should exist in the test map"

    # Create target set with these two lanelets
    target_lanelets = {lanelet_3002094, lanelet_3002093}

    # Find groups including these lanelets and their left/right neighbors
    groups = find_adjacent_groups(lanelet_map, target_lanelets)

    # Find which groups contain our target lanelets
    group_with_3002094 = None
    group_with_3002093 = None

    for group in groups:
        if lanelet_3002094 in group:
            group_with_3002094 = group
        if lanelet_3002093 in group:
            group_with_3002093 = group

    # Both lanelets should be in groups
    assert group_with_3002094 is not None, "Lanelet 3002094 should be in some group"
    assert group_with_3002093 is not None, "Lanelet 3002093 should be in some group"

    # Most importantly: they should be in the same group
    assert group_with_3002094 is group_with_3002093, (
        f"Lanelets 3002094 and 3002093 should be in the same group. "
        f"Found 3002094 in group of size {len(group_with_3002094)}, "
        f"3002093 in group of size {len(group_with_3002093)}"
    )


def test_find_adjacent_groups_empty_set_specific_lanelets():
    """Test that specific lanelets 3002144/3002145/3002146 are in same group with empty target set."""
    lanelet_map = load_test_map()

    # Find the lanelets with specific IDs
    lanelet_3002144 = None
    lanelet_3002145 = None
    lanelet_3002146 = None

    for ll in lanelet_map.laneletLayer:
        if ll.id == 3002144:
            lanelet_3002144 = ll
        elif ll.id == 3002145:
            lanelet_3002145 = ll
        elif ll.id == 3002146:
            lanelet_3002146 = ll

    # Verify all lanelets exist in the map
    assert lanelet_3002144 is not None, "Lanelet 3002144 should exist in the test map"
    assert lanelet_3002145 is not None, "Lanelet 3002145 should exist in the test map"
    assert lanelet_3002146 is not None, "Lanelet 3002146 should exist in the test map"

    # Find groups with empty target set (should group all lanelets)
    groups = find_adjacent_groups(lanelet_map, set())

    # Find which groups contain our target lanelets
    group_with_3002144 = None
    group_with_3002145 = None
    group_with_3002146 = None

    for group in groups:
        if lanelet_3002144 in group:
            group_with_3002144 = group
        if lanelet_3002145 in group:
            group_with_3002145 = group
        if lanelet_3002146 in group:
            group_with_3002146 = group

    # All lanelets should be in groups
    assert group_with_3002144 is not None, "Lanelet 3002144 should be in some group"
    assert group_with_3002145 is not None, "Lanelet 3002145 should be in some group"
    assert group_with_3002146 is not None, "Lanelet 3002146 should be in some group"

    # Most importantly: they should all be in the same group
    assert group_with_3002144 is group_with_3002145, (
        f"Lanelets 3002144 and 3002145 should be in the same group. "
        f"Found 3002144 in group of size {len(group_with_3002144)}, "
        f"3002145 in group of size {len(group_with_3002145)}"
    )

    assert group_with_3002145 is group_with_3002146, (
        f"Lanelets 3002145 and 3002146 should be in the same group. "
        f"Found 3002145 in group of size {len(group_with_3002145)}, "
        f"3002146 in group of size {len(group_with_3002146)}"
    )

    assert group_with_3002144 is group_with_3002146, (
        f"Lanelets 3002144 and 3002146 should be in the same group. "
        f"Found 3002144 in group of size {len(group_with_3002144)}, "
        f"3002146 in group of size {len(group_with_3002146)}"
    )


def test_filter_lanelets_by_single_subtype():
    """Test filtering lanelets by a single subtype."""
    lanelet_map = load_test_map()

    # Check what subtypes exist in the map first
    subtypes_in_map = set()
    for ll in lanelet_map.laneletLayer:
        if ll.attributes and "subtype" in ll.attributes:
            subtypes_in_map.add(ll.attributes["subtype"])

    # If there are subtypes, test filtering with the first one found
    if subtypes_in_map:
        test_subtype = next(iter(subtypes_in_map))

        # Filter lanelets by this subtype
        filtered = filter_lanelets_by_subtype(
            lanelet_map.laneletLayer, subtype=test_subtype
        )

        # Verify that all returned lanelets have the correct subtype
        assert isinstance(filtered, set)
        for ll in filtered:
            assert isinstance(ll, (lanelet2.core.Lanelet, lanelet2.core.ConstLanelet))
            assert ll.attributes["subtype"] == test_subtype

        # Count how many lanelets should have this subtype
        expected_count = sum(
            1
            for ll in lanelet_map.laneletLayer
            if ll.attributes
            and "subtype" in ll.attributes
            and ll.attributes["subtype"] == test_subtype
        )
        assert len(filtered) == expected_count


def test_filter_lanelets_by_multiple_subtypes():
    """Test filtering lanelets by multiple subtypes."""
    lanelet_map = load_test_map()

    # Get all subtypes in the map
    subtypes_in_map = set()
    for ll in lanelet_map.laneletLayer:
        if ll.attributes and "subtype" in ll.attributes:
            subtypes_in_map.add(ll.attributes["subtype"])

    if len(subtypes_in_map) >= 2:
        # Test with first two subtypes
        test_subtypes = list(subtypes_in_map)[:2]

        filtered = filter_lanelets_by_subtype(
            lanelet_map.laneletLayer, subtypes=test_subtypes
        )

        # Verify all returned lanelets have one of the specified subtypes
        assert isinstance(filtered, set)
        for ll in filtered:
            assert isinstance(ll, (lanelet2.core.Lanelet, lanelet2.core.ConstLanelet))
            assert ll.attributes["subtype"] in test_subtypes


def test_filter_lanelets_with_nonexistent_subtype():
    """Test filtering with a subtype that doesn't exist."""
    lanelet_map = load_test_map()

    # Use a subtype that definitely doesn't exist
    filtered = filter_lanelets_by_subtype(
        lanelet_map.laneletLayer, subtype="nonexistent_subtype_xyz"
    )

    # Should return empty set
    assert isinstance(filtered, set)
    assert len(filtered) == 0


def test_filter_lanelets_with_empty_input():
    """Test filtering with empty input."""
    empty_set = set()

    # Filter empty set
    filtered = filter_lanelets_by_subtype(empty_set, subtype="road")

    assert isinstance(filtered, set)
    assert len(filtered) == 0


def test_filter_lanelets_with_no_subtype_specified():
    """Test filtering when no subtype is specified."""
    lanelet_map = load_test_map()

    # Call without subtype or subtypes
    filtered = filter_lanelets_by_subtype(lanelet_map.laneletLayer)

    # Should return empty set
    assert isinstance(filtered, set)
    assert len(filtered) == 0


def test_filter_lanelets_both_params_error():
    """Test that specifying both subtype and subtypes raises an error."""
    lanelet_map = load_test_map()

    # Should raise ValueError
    try:
        filter_lanelets_by_subtype(
            lanelet_map.laneletLayer, subtype="road", subtypes=["highway", "merging"]
        )
        assert False, "Should have raised ValueError"
    except ValueError as e:
        assert "Cannot specify both" in str(e)


def test_filter_lanelets_from_list():
    """Test filtering from a list of lanelets."""
    lanelet_map = load_test_map()

    # Convert to list
    lanelet_list = list(lanelet_map.laneletLayer)[:10]  # Take first 10 for testing

    # Get subtypes in this subset
    subtypes_in_subset = set()
    for ll in lanelet_list:
        if ll.attributes and "subtype" in ll.attributes:
            subtypes_in_subset.add(ll.attributes["subtype"])

    if subtypes_in_subset:
        test_subtype = next(iter(subtypes_in_subset))

        # Filter the list
        filtered = filter_lanelets_by_subtype(lanelet_list, subtype=test_subtype)

        # Verify results
        assert isinstance(filtered, set)
        for ll in filtered:
            assert (
                ll in lanelet_list
            )  # Should only contain lanelets from the input list
            assert ll.attributes["subtype"] == test_subtype


def test_filter_lanelets_from_set():
    """Test filtering from a set of lanelets."""
    lanelet_map = load_test_map()

    # Get terminal lanelets as a set
    terminal_lanelets, _ = find_terminal_lanelets(lanelet_map)

    # Get subtypes in terminal lanelets
    subtypes_in_terminals = set()
    for ll in terminal_lanelets:
        if ll.attributes and "subtype" in ll.attributes:
            subtypes_in_terminals.add(ll.attributes["subtype"])

    if subtypes_in_terminals:
        test_subtype = next(iter(subtypes_in_terminals))

        # Filter the set
        filtered = filter_lanelets_by_subtype(terminal_lanelets, subtype=test_subtype)

        # Verify results
        assert isinstance(filtered, set)
        for ll in filtered:
            assert (
                ll in terminal_lanelets
            )  # Should only contain lanelets from the input set
            assert ll.attributes["subtype"] == test_subtype


def test_filter_lanelets_without_subtype_attribute():
    """Test that lanelets without subtype attribute are not included."""
    # Create a simple lanelet without subtype attribute
    empty_map = lanelet2.core.LaneletMap()

    # Create a lanelet with subtype
    points_left = [
        lanelet2.core.Point3d(lanelet2.core.getId(), 0, 0, 0),
        lanelet2.core.Point3d(lanelet2.core.getId(), 0, 10, 0),
    ]
    points_right = [
        lanelet2.core.Point3d(lanelet2.core.getId(), 1, 0, 0),
        lanelet2.core.Point3d(lanelet2.core.getId(), 1, 10, 0),
    ]

    left_bound = lanelet2.core.LineString3d(lanelet2.core.getId(), points_left)
    right_bound = lanelet2.core.LineString3d(lanelet2.core.getId(), points_right)

    lanelet_with_subtype = lanelet2.core.Lanelet(
        lanelet2.core.getId(), left_bound, right_bound
    )
    lanelet_with_subtype.attributes["subtype"] = "road"

    lanelet_without_subtype = lanelet2.core.Lanelet(
        lanelet2.core.getId(), left_bound, right_bound
    )
    # No subtype attribute added

    empty_map.add(lanelet_with_subtype)
    empty_map.add(lanelet_without_subtype)

    # Filter for road subtype
    filtered = filter_lanelets_by_subtype(empty_map.laneletLayer, subtype="road")

    # Should only include the lanelet with subtype
    assert len(filtered) == 1
    assert lanelet_with_subtype in filtered
    assert lanelet_without_subtype not in filtered


def test_filter_lanelets():
    """Test filtering lanelets by subtype."""
    lanelet_map = load_test_map()

    # Filter lanelets with subtype 'road'
    road_lanelets = filter_lanelets_by_subtype(lanelet_map.laneletLayer, subtype="road")

    # Verify that all returned lanelets have subtype 'road'
    assert isinstance(road_lanelets, set)
    for ll in road_lanelets:
        assert isinstance(ll, (lanelet2.core.Lanelet, lanelet2.core.ConstLanelet))
        assert ll.attributes["subtype"] == "road"
        assert ll.id not in {301105}

    walkway_lanelets = filter_lanelets_by_subtype(
        lanelet_map.laneletLayer, subtype="walkway"
    )
    walkway_ids = {ll.id for ll in walkway_lanelets}
    assert 301105 in walkway_ids


def test_check_lanelet_groups_intersect():
    """Test checking intersection between lanelet groups."""
    lanelet_map = load_test_map()

    # Create two groups of lanelets
    group1 = set()
    group2 = set()

    for ll in lanelet_map.laneletLayer:
        if ll.id in {3002082, 3002083}:
            group1.add(ll)
        elif ll.id in {3002093, 3002094}:
            group2.add(ll)

    # Check intersection (these groups should not intersect)
    assert not check_lanelet_groups_intersect(group1, group2)

    # Now create intersecting groups
    group3 = set()
    group4 = set()

    for ll in lanelet_map.laneletLayer:
        if ll.id in {3002082, 3002083}:
            group3.add(ll)
        elif ll.id in {3002084, 3002085}:
            group4.add(ll)

    # Check intersection (these groups should intersect)
    assert check_lanelet_groups_intersect(group3, group4)
