"""Stop line linestring extraction from Lanelet2 regulatory elements."""

from __future__ import annotations

import logging
from typing import Any


from ..coordinate.map_manager import MapManager

logger = logging.getLogger(__name__)


def get_stop_line_linestrings(lanelet_id: int) -> list[Any]:
    """Return stop line linestrings associated with the given lanelet.

    Searches three types of regulatory elements:

    1. **Traffic light RE**: ``reg_elem.stopLine``
    2. **Traffic sign RE**: ``reg_elem.parameters['ref_line']`` with ``type="stop_line"``
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

    stop_line_linestrings: list[Any] = []
    seen_ids: set[int] = set()

    for reg_elem in lanelet.regulatoryElements:
        # Pattern 1: Traffic light — stopLine attribute
        if hasattr(reg_elem, "stopLine"):
            try:
                stop_line = reg_elem.stopLine
                if stop_line is not None and stop_line.id not in seen_ids:
                    seen_ids.add(stop_line.id)
                    stop_line_linestrings.append(stop_line)
            except Exception:
                pass

        # Pattern 2: Traffic sign — ref_line with type="stop_line"
        attrs = reg_elem.attributes if hasattr(reg_elem, "attributes") else None
        if attrs is not None and attrs.get("subtype") == "traffic_sign":
            try:
                params = reg_elem.parameters
                if "ref_line" in params:
                    for rl in params["ref_line"]:
                        if rl.id not in seen_ids:
                            ls_attrs = (
                                rl.attributes if hasattr(rl, "attributes") else None
                            )
                            if (
                                ls_attrs is not None
                                and ls_attrs.get("type") == "stop_line"
                            ):
                                seen_ids.add(rl.id)
                                stop_line_linestrings.append(rl)
            except Exception:
                pass

        # Pattern 3: Road marking — refers with type="stop_line"
        if attrs is not None and attrs.get("subtype") == "road_marking":
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
                                and ls_attrs.get("type") == "stop_line"
                            ):
                                seen_ids.add(refers_ls.id)
                                stop_line_linestrings.append(refers_ls)
            except Exception:
                pass

    return stop_line_linestrings
