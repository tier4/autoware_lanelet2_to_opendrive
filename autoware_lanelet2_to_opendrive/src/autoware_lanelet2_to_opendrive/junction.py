"""Junction-related utility functions for lanelet2 to OpenDRIVE conversion."""

from typing import List, Set, Union
import lanelet2
from .util import build_lanelet_intersection_adjacency


def _filter_lanelets_inside_junction(
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


def _filter_lanelets_outside_junction(
    lanelets: Union[List[lanelet2.core.Lanelet], Set[lanelet2.core.Lanelet]],
) -> List[lanelet2.core.Lanelet]:
    """Filter lanelets that are outside a junction (intersection).

    Args:
        lanelets: List or set of lanelets to filter

    Returns:
        List of lanelets that do not have turn_direction attribute (indicating non-junction lanelets)
    """
    junction_lanelets = []

    for lanelet in lanelets:
        if "turn_direction" not in lanelet.attributes:
            junction_lanelets.append(lanelet)

    return junction_lanelets


def find_junction_groups(
    lanelets: Union[List[lanelet2.core.Lanelet], Set[lanelet2.core.Lanelet]],
) -> List[List[lanelet2.core.Lanelet]]:
    """Find groups of lanelets that form separate junctions.

    This function takes a group of lanelets and splits them into separate groups
    where each group represents a junction. Lanelets that intersect with each other
    are placed in the same junction group.

    The pairwise intersections are resolved up front by
    ``build_lanelet_intersection_adjacency``, which uses a bounding-box filter
    so that only plausible pairs reach the geometric test. The merge loop below
    then works on lanelet indices and consults that adjacency instead of
    re-running geometry on every group pair.

    Args:
        lanelets: List or set of lanelets to group

    Returns:
        List of lanelet groups, where each group represents a separate junction
    """
    if not lanelets:
        return []

    # Convert to list if it's a set
    lanelet_list = list(lanelets)

    # index -> indices of the lanelets it intersects
    adjacency = build_lanelet_intersection_adjacency(lanelet_list)

    # Initialize each lanelet as its own group (groups hold indices into lanelet_list)
    groups = [[index] for index in range(len(lanelet_list))]

    # Keep merging intersecting groups until no more merges are possible
    changed = True
    while changed:
        changed = False
        new_groups = []
        merged_indices = set()

        group_of = {}
        for group_index, group in enumerate(groups):
            for member in group:
                group_of[member] = group_index

        for i in range(len(groups)):
            if i in merged_indices:
                continue

            current_group = groups[i]
            merged_group = current_group.copy()

            # Groups holding at least one lanelet that intersects the current
            # group -- exactly the groups the pairwise scan would have matched.
            neighbor_groups = set()
            for member in current_group:
                for neighbor in adjacency[member]:
                    neighbor_groups.add(group_of[neighbor])
            neighbor_groups.discard(i)

            # Visited in ascending order so the merged group keeps the same
            # member order as a left-to-right scan over the group list.
            for j in sorted(neighbor_groups):
                if j < i or j in merged_indices:
                    continue

                merged_group.extend(groups[j])
                merged_indices.add(j)
                changed = True

            new_groups.append(merged_group)

        groups = new_groups

    return [[lanelet_list[index] for index in group] for group in groups]
