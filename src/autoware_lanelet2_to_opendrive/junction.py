"""Junction-related utility functions for lanelet2 to OpenDRIVE conversion."""

from typing import List, Set, Union
import lanelet2


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
