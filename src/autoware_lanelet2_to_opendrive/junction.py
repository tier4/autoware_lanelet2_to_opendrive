"""Junction-related utility functions for lanelet2 to OpenDRIVE conversion."""

from typing import List, Set, Union
import lanelet2
from .util import check_lanelet_groups_intersect


def filter_lanelets_inside_junction(
    lanelets: Union[List[lanelet2.core.Lanelet], Set[lanelet2.core.Lanelet]],
) -> List[lanelet2.core.Lanelet]:
    """Filter lanelets that are inside a junction (intersection).

    Args:
        lanelets: List or set of lanelets to filter

    Returns:
        List of lanelets that have turn_direction attribute (indicating junction lanelets)
    """
    junction_lanelets = []

    for lanelet in lanelets:
        if "turn_direction" in lanelet.attributes:
            junction_lanelets.append(lanelet)

    return junction_lanelets


def find_junction_groups(
    lanelets: Union[List[lanelet2.core.Lanelet], Set[lanelet2.core.Lanelet]],
) -> List[List[lanelet2.core.Lanelet]]:
    """Find groups of lanelets that form separate junctions.

    This function takes a group of lanelets and splits them into separate groups
    where each group represents a junction. Lanelets that intersect with each other
    are placed in the same junction group.

    Args:
        lanelets: List or set of lanelets to group

    Returns:
        List of lanelet groups, where each group represents a separate junction
    """
    if not lanelets:
        return []

    # Convert to list if it's a set
    lanelet_list = list(lanelets)

    # Initialize each lanelet as its own group
    groups = [[lanelet] for lanelet in lanelet_list]

    # Keep merging intersecting groups until no more merges are possible
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

                # Check if the two groups intersect
                if check_lanelet_groups_intersect(set(current_group), set(groups[j])):
                    merged_group.extend(groups[j])
                    merged_indices.add(j)
                    changed = True

            new_groups.append(merged_group)

        groups = new_groups

    return groups
