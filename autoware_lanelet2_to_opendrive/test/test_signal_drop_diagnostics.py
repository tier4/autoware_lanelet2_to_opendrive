"""A conversion that emits no ``<signal>`` must say why.

A 2026-10-23 report covering four maps found one that emitted no ``<signal>``
and no ``<controller>`` at all, and another whose signals all landed on the
same spot. Neither symptom is diagnosable from the log, because every path
that drops a traffic light, and every path that falls back to the default
``(s=0, t=-4)`` position, is silent.

Measured on ``nishishinjuku.osm`` at ``awf-master-sync`` @ ``f59cf2e8``:
``target=default`` emits 508 signals and ``target=carla`` emits 334, so
``exclude_non_junction_signals`` -- the condition the report ranked first --
removes 34% of signals there and only 12 of 282 traffic lights. It cannot
produce the reported total absence on a map whose junction lanelets are
recognised. Which of the five conditions did fire is what the log has to
answer, and these tests pin that it does.

The five whole-light drop paths in ``construct_from_lanelet_map``, all of
which were silent:

1. the map has no ``trafficLights`` regulatory elements at all;
2. the CARLA junction filter excluded the light;
3. none of the light's lanelets belongs to an OpenDRIVE road;
4. every one of the light's linestrings is empty;
5. the matching road holds none of the light's lanelets (partial, per road).
"""

from __future__ import annotations

import logging
from typing import List, Optional
from unittest.mock import MagicMock, Mock, patch

import pytest

from autoware_lanelet2_to_opendrive.opendrive import SignalsAndControllers
from autoware_lanelet2_to_opendrive.opendrive import (
    signals_and_controllers as sac_module,
)
from autoware_lanelet2_to_opendrive.opendrive.enums import TrafficRule
from autoware_lanelet2_to_opendrive.util import RoadLaneletMapping

LOGGER_NAME = sac_module.__name__

_MODULE = "autoware_lanelet2_to_opendrive.opendrive.signals_and_controllers"


def _linestring(ls_id: int, points: List) -> MagicMock:
    """Return a mock linestring with the given id and points."""
    ls = MagicMock()
    ls.id = ls_id
    ls.__len__ = Mock(return_value=len(points))
    ls.__getitem__ = Mock(side_effect=lambda i: points[i])
    ls.__iter__ = Mock(side_effect=lambda: iter(points))
    return ls


def _traffic_light(light_id: int = 9000, *, empty_linestrings: bool = False):
    """Return a mock traffic-light regulatory element."""
    points = [] if empty_linestrings else [MagicMock(x=10.0, y=5.0, z=8.0)]
    light = MagicMock()
    light.id = light_id
    light.trafficLights = [_linestring(1001, points)]
    light.stopLine = None
    light.attributes = {}
    return light


def _road(road_id: int = 0) -> MagicMock:
    road = MagicMock()
    road.id = road_id
    road.rule = TrafficRule.RHT
    road.get_lanelet_to_lane_mapping.return_value = {100: -1}
    road.get_half_width_at_s.return_value = -4.0
    road.get_elevation_at_s.return_value = 0.0
    return road


def _construct(
    *,
    traffic_light_map: Optional[dict] = None,
    mapping: Optional[RoadLaneletMapping] = None,
    exclude_non_junction_signals: bool = False,
    junction_lanelet_ids: Optional[set] = None,
) -> SignalsAndControllers:
    """Run ``construct_from_lanelet_map`` over mocks, with the fit stubbed out."""
    if traffic_light_map is None:
        traffic_light_map = {9000: (_traffic_light(), [100])}
    if mapping is None:
        mapping = RoadLaneletMapping(
            road_to_lanelets={0: [100]}, lanelet_to_road={100: 0}
        )

    lanelet_map = MagicMock()
    lanelet_map.laneletLayer.get.return_value = MagicMock()

    with (
        patch(
            f"{_MODULE}.filter_regulatory_element_by_type",
            return_value=traffic_light_map,
        ),
        patch(f"{_MODULE}.create_routing_graph", return_value=MagicMock()),
        patch(
            f"{_MODULE}.SignalsAndControllers._calculate_signal_position",
            return_value=(5.0, -3.0),
        ),
    ):
        return SignalsAndControllers.construct_from_lanelet_map(
            lanelet_map=lanelet_map,
            road_lanelet_mapping=mapping,
            roads=[_road()],
            exclude_non_junction_signals=exclude_non_junction_signals,
            junction_lanelet_ids=junction_lanelet_ids,
        )


def _warnings(caplog) -> List[str]:
    return [
        rec.getMessage() for rec in caplog.records if rec.levelno >= logging.WARNING
    ]


# ---------------------------------------------------------------------------
# The five drop paths
# ---------------------------------------------------------------------------


@pytest.mark.xfail(strict=True, reason="signal drops are silent; 2026-10-23 report")
def test_a_map_with_no_traffic_light_elements_says_so(caplog):
    """Condition 1: the map carries no trafficLights regulatory element."""
    with caplog.at_level(logging.WARNING, logger=LOGGER_NAME):
        result = _construct(traffic_light_map={})

    assert not result.signals
    messages = _warnings(caplog)
    assert messages, "a map that can emit no signal at all said nothing"
    assert any(
        "no trafficLights regulatory element" in m.lower() for m in messages
    ), f"the reason is not named: {messages}"


@pytest.mark.xfail(strict=True, reason="signal drops are silent; 2026-10-23 report")
def test_a_traffic_light_with_no_mapped_road_is_reported(caplog):
    """Condition 3: none of the light's lanelets belongs to a road."""
    with caplog.at_level(logging.WARNING, logger=LOGGER_NAME):
        result = _construct(
            mapping=RoadLaneletMapping(road_to_lanelets={}, lanelet_to_road={})
        )

    assert not result.signals
    messages = _warnings(caplog)
    assert messages, "every traffic light was dropped and nothing was logged"
    assert any("1" in m for m in messages), f"the count is not named: {messages}"


@pytest.mark.xfail(strict=True, reason="signal drops are silent; 2026-10-23 report")
def test_a_traffic_light_with_only_empty_linestrings_is_reported(caplog):
    """Condition 4: every linestring of the light is empty."""
    light_map = {9000: (_traffic_light(empty_linestrings=True), [100])}
    with caplog.at_level(logging.WARNING, logger=LOGGER_NAME):
        result = _construct(traffic_light_map=light_map)

    assert not result.signals
    assert _warnings(caplog), "a light with no geometry was dropped silently"


@pytest.mark.xfail(strict=True, reason="signal drops are silent; 2026-10-23 report")
def test_the_carla_junction_filter_reports_what_it_removed(caplog):
    """Condition 2: ``exclude_non_junction_signals`` removed the light."""
    with caplog.at_level(logging.WARNING, logger=LOGGER_NAME):
        result = _construct(
            exclude_non_junction_signals=True, junction_lanelet_ids=set()
        )

    assert not result.signals
    messages = _warnings(caplog)
    assert messages, "the CARLA filter removed every signal and said nothing"
    assert any(
        "junction" in m.lower() for m in messages
    ), f"the filter is not named as the reason: {messages}"


@pytest.mark.xfail(strict=True, reason="signal drops are silent; 2026-10-23 report")
def test_the_default_position_fallback_is_reported(caplog):
    """The ``(s=0, t=-4)`` fallback is the reported 'all signals in one spot'."""
    light = _traffic_light()
    with caplog.at_level(logging.WARNING, logger=LOGGER_NAME):
        s, t = SignalsAndControllers._calculate_signal_position(
            traffic_light=light,
            light_linestring=_linestring(1002, []),
            road_id=0,
            lanelet_map=MagicMock(),
            road_lanelet_mapping=RoadLaneletMapping(
                road_to_lanelets={0: [100]}, lanelet_to_road={100: 0}
            ),
        )

    assert (s, t) == (0.0, -4.0)
    messages = _warnings(caplog)
    assert messages, "a signal was placed at the fallback position silently"
    assert any(
        "9000" in m for m in messages
    ), f"the traffic light is not identified: {messages}"


# ---------------------------------------------------------------------------
# Guard: a clean conversion stays quiet
# ---------------------------------------------------------------------------


def test_a_conversion_that_drops_nothing_does_not_warn(caplog):
    """Nothing dropped, nothing warned -- so the warnings stay meaningful."""
    with caplog.at_level(logging.WARNING, logger=LOGGER_NAME):
        result = _construct()

    assert result.signals, "the happy path stopped producing signals"
    assert not _warnings(caplog), f"a clean conversion warned: {_warnings(caplog)}"
