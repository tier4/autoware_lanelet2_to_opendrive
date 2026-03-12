"""Stop line linestring extraction from Lanelet2 regulatory elements."""

from __future__ import annotations

import logging
from typing import Any


from ..coordinate.map_manager import MapManager

logger = logging.getLogger(__name__)


def _attr_get(attrs: Any, key: str) -> str | None:
    """Safely get a value from a lanelet2 AttributeMap.

    The lanelet2 ``AttributeMap`` does not support ``.get()``, so this
    helper uses ``in`` + ``[]`` access instead.
    """
    try:
        if key in attrs:
            return str(attrs[key])
    except Exception:
        pass
    return None


def _collect_stop_lines_from_reg_elems(
    reg_elems: Any,
    seen_ids: set[int],
) -> list[Any]:
    """Extract stop line linestrings from a sequence of regulatory elements.

    Searches three patterns:

    1. **Traffic light RE**: ``reg_elem.stopLine``
    2. **Traffic sign RE**: ``ref_line`` parameter (any linestring in the
       ``ref_line`` role is treated as a stop line regardless of its attributes)
    3. **Road marking RE**: ``refers`` parameter with ``type="stop_line"``
    """
    results: list[Any] = []

    for reg_elem in reg_elems:
        # Pattern 1: Traffic light — stopLine attribute
        if hasattr(reg_elem, "stopLine"):
            try:
                stop_line = reg_elem.stopLine
                if stop_line is not None and stop_line.id not in seen_ids:
                    seen_ids.add(stop_line.id)
                    results.append(stop_line)
            except Exception:
                pass

        attrs = reg_elem.attributes if hasattr(reg_elem, "attributes") else None

        # Pattern 2: Traffic sign — ref_line role (all ref_line entries are stop lines)
        if attrs is not None and _attr_get(attrs, "subtype") == "traffic_sign":
            try:
                params = reg_elem.parameters
                if "ref_line" in params:
                    for rl in params["ref_line"]:
                        if rl.id not in seen_ids:
                            seen_ids.add(rl.id)
                            results.append(rl)
            except Exception:
                pass

        # Pattern 3: Road marking — refers with type="stop_line"
        if attrs is not None and _attr_get(attrs, "subtype") == "road_marking":
            try:
                params = reg_elem.parameters
                if "refers" in params:
                    for refers_ls in params["refers"]:
                        if refers_ls.id not in seen_ids:
                            ls_attrs = (
                                refers_ls.attributes
                                if hasattr(refers_ls, "attributes")
                                else None
                            )
                            if (
                                ls_attrs is not None
                                and _attr_get(ls_attrs, "type") == "stop_line"
                            ):
                                seen_ids.add(refers_ls.id)
                                results.append(refers_ls)
            except Exception:
                pass

    return results


def get_stop_line_linestrings(lanelet_id: int) -> list[Any]:
    """Return stop line linestrings associated with the given lanelet.

    Searches three types of regulatory elements:

    1. **Traffic light RE**: ``reg_elem.stopLine``
    2. **Traffic sign RE**: all linestrings in the ``ref_line`` parameter
    3. **Road marking RE**: ``reg_elem.parameters['refers']`` with ``type="stop_line"``

    Duplicate stop lines (same linestring ID) are excluded.

    Requires :class:`MapManager` to be initialised.

    Args:
        lanelet_id: The Lanelet2 lanelet ID to search for stop lines.

    Returns:
        List of lanelet2 linestring objects for each unique stop line found.
        Empty list if no stop lines are associated with the lanelet.

    Raises:
        ValueError: If the lanelet ID is not found in the map.
    """
    mm = MapManager.get_instance()
    lanelet_map = mm.lanelet_map

    try:
        lanelet = lanelet_map.laneletLayer[lanelet_id]
    except (KeyError, IndexError) as exc:
        raise ValueError(f"Lanelet ID {lanelet_id} not found in map") from exc

    seen_ids: set[int] = set()
    return _collect_stop_lines_from_reg_elems(lanelet.regulatoryElements, seen_ids)


def get_stop_line_linestrings_with_following(
    lanelet_id: int,
) -> list[tuple[int, Any]]:
    """Return stop line linestrings from the lanelet and its immediate successors.

    Searches the given lanelet first, then its ``following`` lanelets in the
    routing graph. Returns as soon as stop lines are found on any lanelet.

    Args:
        lanelet_id: The starting Lanelet2 lanelet ID.

    Returns:
        List of ``(owner_lanelet_id, linestring)`` tuples.
        Empty list if no stop lines are found.

    Raises:
        ValueError: If the lanelet ID is not found in the map.
    """
    import lanelet2.routing
    import lanelet2.traffic_rules

    mm = MapManager.get_instance()
    lanelet_map = mm.lanelet_map

    try:
        lanelet = lanelet_map.laneletLayer[lanelet_id]
    except (KeyError, IndexError) as exc:
        raise ValueError(f"Lanelet ID {lanelet_id} not found in map") from exc

    seen_ids: set[int] = set()

    # First: check the lanelet itself
    results = _collect_stop_lines_from_reg_elems(lanelet.regulatoryElements, seen_ids)
    if results:
        return [(lanelet_id, ls) for ls in results]

    # Second: check following lanelets
    traffic_rules = lanelet2.traffic_rules.create(
        lanelet2.traffic_rules.Locations.Germany,
        lanelet2.traffic_rules.Participants.Vehicle,
    )
    routing_graph = lanelet2.routing.RoutingGraph(lanelet_map, traffic_rules)
    following = routing_graph.following(lanelet)

    all_results: list[tuple[int, Any]] = []
    for fll in following:
        fll_results = _collect_stop_lines_from_reg_elems(
            fll.regulatoryElements, seen_ids
        )
        all_results.extend((fll.id, ls) for ls in fll_results)

    return all_results
