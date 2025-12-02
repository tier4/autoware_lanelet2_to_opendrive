"""Tests for junction functions."""

from pathlib import Path
from autoware_lanelet2_extension_python.projection import MGRSProjector
import lanelet2


def load_test_map():
    """Load the test lanelet2 map."""
    test_data_path = Path(__file__).parent / "data" / "lanelet2_map.osm"
    projector = MGRSProjector(
        lanelet2.io.Origin(35.23, 139.16)
    )  # MGRS origin for Tokyo area (54SUE)
    return lanelet2.io.load(str(test_data_path), projector)


def test_filter_lanelets_inside_junction():
    """Test filtering lanelets inside a junction."""
    lanelet_map = load_test_map()
    lanelets = list(lanelet_map.laneletLayer)

    from autoware_lanelet2_to_opendrive.junction import filter_lanelets_inside_junction

    junction_lanelets = filter_lanelets_inside_junction(lanelets)

    junction_ids = {ll.id for ll in junction_lanelets}
    assert 3002084 in junction_ids  # Example junction lanelet ID

    # Check that the filtered lanelets have the 'turn_direction' attribute
    for lanelet in junction_lanelets:
        assert "turn_direction" in lanelet.attributes


def test_find_junction_groups():
    """Test finding separate junction groups from lanelets."""
    from autoware_lanelet2_to_opendrive.junction import (
        filter_lanelets_inside_junction,
        find_junction_groups,
    )

    lanelet_map = load_test_map()
    lanelets = list(lanelet_map.laneletLayer)

    # First filter to get only junction lanelets
    junction_lanelets = filter_lanelets_inside_junction(lanelets)

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
