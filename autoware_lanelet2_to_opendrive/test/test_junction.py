"""Tests for junction functions."""


def test__filter_lanelets_inside_junction(lanelet_map):
    """Test filtering lanelets inside a junction."""
    lanelets = list(lanelet_map.laneletLayer)

    from autoware_lanelet2_to_opendrive.junction import _filter_lanelets_inside_junction

    junction_lanelets = _filter_lanelets_inside_junction(lanelets)

    junction_ids = {ll.id for ll in junction_lanelets}
    assert 3002084 in junction_ids  # Example junction lanelet ID

    # Check that the filtered lanelets have the 'turn_direction' attribute
    for lanelet in junction_lanelets:
        assert "turn_direction" in lanelet.attributes


def test_find_junction_groups(lanelet_map):
    """Test finding separate junction groups from lanelets."""
    from autoware_lanelet2_to_opendrive.junction import (
        _filter_lanelets_inside_junction,
        find_junction_groups,
    )

    lanelets = list(lanelet_map.laneletLayer)

    # First filter to get only junction lanelets
    junction_lanelets = _filter_lanelets_inside_junction(lanelets)

    # Find groups of junctions
    junction_groups = find_junction_groups(junction_lanelets)

    # Basic assertions
    assert len(junction_groups) > 0  # Should have at least one junction group

    # All original junction lanelets should be in exactly one group
    all_grouped_lanelets = []
    for group in junction_groups:
        all_grouped_lanelets.extend(group)

    assert len(all_grouped_lanelets) == len(junction_lanelets)
    assert set(ll.id for ll in all_grouped_lanelets) == set(
        ll.id for ll in junction_lanelets
    )

    # Each lanelet should appear in exactly one group
    lanelet_ids = [ll.id for ll in all_grouped_lanelets]
    assert len(lanelet_ids) == len(set(lanelet_ids))  # No duplicates

    # Test with empty input
    empty_groups = find_junction_groups([])
    assert empty_groups == []

    # Test with single lanelet
    single_lanelet_groups = find_junction_groups([junction_lanelets[0]])
    assert len(single_lanelet_groups) == 1
    assert len(single_lanelet_groups[0]) == 1

    # Test that lanelets 397 and 403 are in the same group
    lanelet_397 = None
    lanelet_403 = None
    for lanelet in junction_lanelets:
        if lanelet.id == 397:
            lanelet_397 = lanelet
        elif lanelet.id == 403:
            lanelet_403 = lanelet

    # Find which groups contain these lanelets
    group_with_397 = None
    group_with_403 = None
    for group in junction_groups:
        for lanelet in group:
            if lanelet.id == 397:
                group_with_397 = group
            if lanelet.id == 403:
                group_with_403 = group

    # Assert both lanelets exist and are in the same group
    assert lanelet_397 is not None, "Lanelet 397 should exist in junction lanelets"
    assert lanelet_403 is not None, "Lanelet 403 should exist in junction lanelets"
    assert group_with_397 is not None, "Lanelet 397 should be in a junction group"
    assert group_with_403 is not None, "Lanelet 403 should be in a junction group"
    assert (
        group_with_397 is group_with_403
    ), "Lanelets 397 and 403 should be in the same junction group"


def test_construct_from_lanelet_groups(lanelet_map):
    """Test constructing Junction instances from lanelet groups."""
    from autoware_lanelet2_to_opendrive.junction import (
        _filter_lanelets_inside_junction,
        find_junction_groups,
    )
    from autoware_lanelet2_to_opendrive.opendrive.junction import Junction

    lanelets = list(lanelet_map.laneletLayer)

    # Get junction lanelets and group them
    junction_lanelets = _filter_lanelets_inside_junction(lanelets)
    junction_groups = find_junction_groups(junction_lanelets)

    # Test constructing Junctions from groups
    junctions = []
    for i, group in enumerate(junction_groups):
        junction = Junction.construct_from_lanelet_groups(
            junction_id=i, lanelet_group=group
        )
        junctions.append(junction)

    # Verify we have the same number of junctions as groups
    assert len(junctions) == len(junction_groups)

    # Verify each junction has the correct ID
    for i, junction in enumerate(junctions):
        assert junction.id == i
        assert junction.name is not None
        assert junction.name.startswith("junction_")
        assert junction.connections == []  # No connections initially

    # Test with custom name
    if junction_groups:
        custom_junction = Junction.construct_from_lanelet_groups(
            junction_id=100, lanelet_group=junction_groups[0], name="CustomJunction"
        )
        assert custom_junction.id == 100
        assert custom_junction.name == "CustomJunction"
        assert custom_junction.connections == []

    # Test with empty group
    empty_junction = Junction.construct_from_lanelet_groups(
        junction_id=999, lanelet_group=[]
    )
    assert empty_junction.id == 999
    assert empty_junction.name == "junction_999"
    assert empty_junction.connections == []


def test_construct_from_lanelet_map(lanelet_map):
    """Test constructing Junction instances directly from a lanelet map."""
    from autoware_lanelet2_to_opendrive.opendrive.junction import Junction
    from autoware_lanelet2_to_opendrive.junction import (
        _filter_lanelets_inside_junction,
        find_junction_groups,
    )

    # Test with default starting ID
    junctions = Junction.construct_from_lanelet_map(lanelet_map)

    # Verify we get junctions
    assert len(junctions) > 0, "Should find at least one junction in test map"

    # Verify junction IDs start from 0 and are sequential
    for i, junction in enumerate(junctions):
        assert junction.id == i
        assert junction.name is not None
        assert junction.name.startswith("junction_")
        assert junction.connections == []  # No connections initially

    # Verify consistency with manual approach
    lanelets = list(lanelet_map.laneletLayer)
    junction_lanelets = _filter_lanelets_inside_junction(lanelets)
    junction_groups = find_junction_groups(junction_lanelets)
    assert len(junctions) == len(
        junction_groups
    ), "Should have same number of junctions as groups"

    # Test with custom starting ID
    custom_junctions = Junction.construct_from_lanelet_map(
        lanelet_map, starting_junction_id=100
    )
    assert len(custom_junctions) == len(junctions)
    for i, junction in enumerate(custom_junctions):
        assert junction.id == 100 + i
        assert junction.name is not None
        assert junction.connections == []


def _reference_find_junction_groups(lanelets):
    """Original O(M^2) grouping, kept as the equivalence reference.

    This is the pre-spatial-index implementation, transcribed unchanged. It
    calls intersects2d on every group pair on every pass; find_junction_groups
    must return exactly the same nested lists.
    """
    from autoware_lanelet2_to_opendrive.util import check_lanelet_groups_intersect

    if not lanelets:
        return []

    lanelet_list = list(lanelets)
    groups = [[lanelet] for lanelet in lanelet_list]

    changed = True
    while changed:
        changed = False
        new_groups = []
        merged_indices = set()

        for i in range(len(groups)):
            if i in merged_indices:
                continue

            current_group = groups[i]
            merged_group = current_group.copy()

            for j in range(i + 1, len(groups)):
                if j in merged_indices:
                    continue

                if check_lanelet_groups_intersect(set(current_group), set(groups[j])):
                    merged_group.extend(groups[j])
                    merged_indices.add(j)
                    changed = True

            new_groups.append(merged_group)

        groups = new_groups

    return groups


def test_find_junction_groups_matches_reference_implementation(lanelet_map):
    """The spatially indexed grouping must reproduce the original exactly.

    Not just the same membership: the same group order and the same member
    order within each group, so that anything downstream keyed on position is
    unaffected.
    """
    from autoware_lanelet2_to_opendrive.junction import (
        _filter_lanelets_inside_junction,
        find_junction_groups,
    )

    junction_lanelets = _filter_lanelets_inside_junction(list(lanelet_map.laneletLayer))
    assert len(junction_lanelets) > 1

    actual = find_junction_groups(junction_lanelets)
    expected = _reference_find_junction_groups(junction_lanelets)

    actual_ids = [[ll.id for ll in group] for group in actual]
    expected_ids = [[ll.id for ll in group] for group in expected]

    assert actual_ids == expected_ids


def test_find_junction_groups_is_order_stable_for_subsets(lanelet_map):
    """Sliced inputs must also match the reference, including ordering."""
    from autoware_lanelet2_to_opendrive.junction import (
        _filter_lanelets_inside_junction,
        find_junction_groups,
    )

    junction_lanelets = _filter_lanelets_inside_junction(list(lanelet_map.laneletLayer))

    for size in (2, 5, 17, 40):
        subset = junction_lanelets[:size]
        if len(subset) < size:
            continue

        actual = [[ll.id for ll in group] for group in find_junction_groups(subset)]
        expected = [
            [ll.id for ll in group] for group in _reference_find_junction_groups(subset)
        ]
        assert actual == expected, f"mismatch for subset size {size}"
