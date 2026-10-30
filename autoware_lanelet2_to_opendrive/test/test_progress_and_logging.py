"""Progress and diagnostics must go through ``logging``, not ``tqdm.write``.

``road.py`` and ``road_lanelet_geo_mapping.py`` are the only two modules that
write to the terminal directly. Six of their messages report a **caught
exception** and are the sole trace that the exception happened, so a test
cannot see them, a user cannot filter them, and no handler can route them.
Every other module in the package already uses ``logging``.
"""

from __future__ import annotations

import logging
from typing import Any, Dict

from autoware_lanelet2_to_opendrive.opendrive.road import Road


class _ExplodingRoad:
    """The smallest thing ``set_all_lane_links`` will walk over.

    ``set_lane_links`` raises, which is the case the production code catches
    and reports. Nothing here needs a real Lanelet2 map: the map and the
    routing graph are only put into the context object that
    ``set_lane_links`` is handed, and this one never looks at it.
    """

    def __init__(self, road_id: int) -> None:
        self.id = road_id
        self.lanes = None

    def get_lanelet_to_lane_mapping(self) -> Dict[int, int]:
        return {}

    def set_lane_links(self, context: Any) -> None:
        raise RuntimeError("lane link resolution failed")


def test_lane_link_failure_is_reported_through_logging(
    caplog: Any,
) -> None:
    """A swallowed lane-link exception must reach ``logging``.

    ``set_all_lane_links`` catches every exception per road so that one bad
    road cannot stop the conversion. That is the right behaviour, but the
    report has to be observable; today it goes to stderr via ``tqdm.write``
    and leaves no record a handler can see.
    """
    road = _ExplodingRoad(road_id=4242)

    with caplog.at_level(logging.WARNING):
        mapping = Road.set_all_lane_links(
            lanelet_map=None,
            roads=[road],  # type: ignore[list-item]
            routing_graph=object(),  # not None, so no graph is constructed
        )

    assert mapping == {}, "the walk should still complete and return its mapping"

    warnings = [r for r in caplog.records if r.levelno >= logging.WARNING]
    assert warnings, "the caught exception was not reported to logging at all"

    assert any("4242" in r.getMessage() for r in warnings), (
        "the report does not name the road that failed; "
        f"got {[r.getMessage() for r in warnings]}"
    )
