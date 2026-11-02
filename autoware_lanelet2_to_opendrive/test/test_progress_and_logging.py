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


class _EmptyMap:
    """A lanelet map with no lanelets.

    ``set_all_lane_links`` builds a boundary-adjacency index over the whole
    ``laneletLayer`` before it reaches the per-road loop, so ``None`` no
    longer gets that far. An empty layer makes every map-wide pass a no-op,
    which is what this test wants: it is about the per-road exception, not
    about any map.
    """

    def __init__(self) -> None:
        self.laneletLayer: list = []


class _ExplodingRoad(Road):
    """A real ``Road`` whose lane-link resolution raises.

    ``set_lane_links`` raising is the case the production code catches and
    reports; everything else about this road has to be ordinary. It subclasses
    ``Road`` rather than standing in for it because ``set_all_lane_links``
    reads a growing set of attributes before it reaches the call that raises
    (``exit_fork_duplicate``, ``chain_source_lanelet_ids``, ...), each added
    by a different branch. A hand-rolled fake has to be taught each one, and
    is taught it by a failing test on somebody else's branch; a subclass gets
    ``Road``'s own defaults for free.

    Nothing here needs a real Lanelet2 map: the map and the routing graph are
    only put into the context object ``set_lane_links`` is handed, and this
    one never looks at it.
    """

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
    road = _ExplodingRoad(id=4242)

    with caplog.at_level(logging.WARNING):
        mapping = Road.set_all_lane_links(
            lanelet_map=_EmptyMap(),  # type: ignore[arg-type]
            roads=[road],
            routing_graph=object(),  # not None, so no graph is constructed
        )

    assert mapping == {}, "the walk should still complete and return its mapping"

    warnings = [r for r in caplog.records if r.levelno >= logging.WARNING]
    assert warnings, "the caught exception was not reported to logging at all"

    assert any("4242" in r.getMessage() for r in warnings), (
        "the report does not name the road that failed; "
        f"got {[r.getMessage() for r in warnings]}"
    )
