"""Unit tests for traffic light utility functions (no CARLA required)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List
from unittest.mock import MagicMock, patch

import pytest

from autoware_carla_scenario.utils.traffic_light import (
    find_nearest_traffic_light,
    set_group_traffic_light_state,
)


# ---------------------------------------------------------------------------
# Lightweight stubs for CARLA types
# ---------------------------------------------------------------------------


@dataclass
class _FakeLocation:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def distance(self, other: "_FakeLocation") -> float:
        return (
            (self.x - other.x) ** 2 + (self.y - other.y) ** 2 + (self.z - other.z) ** 2
        ) ** 0.5


@dataclass
class _FakeTransform:
    location: _FakeLocation


class _FakeTrafficLight:
    """Minimal traffic light stub."""

    def __init__(
        self,
        x: float,
        y: float,
        z: float = 0.0,
        opendrive_id: str = "",
    ) -> None:
        self._transform = _FakeTransform(location=_FakeLocation(x, y, z))
        self._state: object = None
        self._frozen: bool = False
        self._group: List["_FakeTrafficLight"] = [self]
        self._opendrive_id = opendrive_id
        self.type_id = "traffic.traffic_light"

    def get_transform(self) -> _FakeTransform:
        return self._transform

    def get_group_traffic_lights(self) -> List["_FakeTrafficLight"]:
        return self._group

    def get_opendrive_id(self) -> str:
        return self._opendrive_id

    def set_state(self, state: object) -> None:
        self._state = state

    def freeze(self, frozen: bool) -> None:
        self._frozen = frozen


class _FakeWorld:
    """Minimal CARLA world stub."""

    def __init__(self, actors: List[_FakeTrafficLight]) -> None:
        self._actors = actors

    def get_actors(self) -> List[_FakeTrafficLight]:
        return self._actors


def _make_group(*lights: _FakeTrafficLight) -> List[_FakeTrafficLight]:
    """Link traffic lights into a single group."""
    group = list(lights)
    for tl in group:
        tl._group = group
    return group


# ---------------------------------------------------------------------------
# find_nearest_traffic_light
# ---------------------------------------------------------------------------


class TestFindNearestTrafficLight:
    def test_returns_nearest(self) -> None:
        tl_close = _FakeTrafficLight(1.0, 0.0)
        tl_far = _FakeTrafficLight(100.0, 0.0)
        origin = _FakeLocation(0.0, 0.0, 0.0)

        nearest, dist = find_nearest_traffic_light([tl_far, tl_close], origin)

        assert nearest is tl_close
        assert dist == pytest.approx(1.0)

    def test_all_beyond_max_distance(self) -> None:
        tl = _FakeTrafficLight(200.0, 0.0)
        origin = _FakeLocation(0.0, 0.0, 0.0)

        nearest, dist = find_nearest_traffic_light([tl], origin, max_distance=50.0)

        assert nearest is None
        assert dist == float("inf")

    def test_empty_list(self) -> None:
        origin = _FakeLocation(0.0, 0.0, 0.0)

        nearest, dist = find_nearest_traffic_light([], origin)

        assert nearest is None
        assert dist == float("inf")

    def test_custom_max_distance(self) -> None:
        tl = _FakeTrafficLight(10.0, 0.0)
        origin = _FakeLocation(0.0, 0.0, 0.0)

        nearest, dist = find_nearest_traffic_light([tl], origin, max_distance=5.0)
        assert nearest is None

        nearest, dist = find_nearest_traffic_light([tl], origin, max_distance=15.0)
        assert nearest is tl

    def test_multiple_lights_selects_closest(self) -> None:
        tl_a = _FakeTrafficLight(5.0, 0.0)
        tl_b = _FakeTrafficLight(3.0, 0.0)
        tl_c = _FakeTrafficLight(8.0, 0.0)
        origin = _FakeLocation(0.0, 0.0, 0.0)

        nearest, dist = find_nearest_traffic_light([tl_a, tl_b, tl_c], origin)

        assert nearest is tl_b
        assert dist == pytest.approx(3.0)


# ---------------------------------------------------------------------------
# set_group_traffic_light_state
# ---------------------------------------------------------------------------


class TestSetGroupTrafficLightState:
    _PATCH_TARGET = (
        "autoware_carla_scenario.utils.traffic_light._get_signal_ids_for_controller"
    )

    def test_sets_state_on_all_matching_signals(self) -> None:
        tl1 = _FakeTrafficLight(0.0, 0.0, opendrive_id="sig_1")
        tl2 = _FakeTrafficLight(10.0, 0.0, opendrive_id="sig_2")
        world = _FakeWorld([tl1, tl2])

        state = MagicMock(name="Green")
        with patch(self._PATCH_TARGET, return_value=["sig_1", "sig_2"]):
            set_group_traffic_light_state(world, 100, state)

        assert tl1._state is state
        assert tl2._state is state

    def test_freezes_all_by_default(self) -> None:
        tl1 = _FakeTrafficLight(0.0, 0.0, opendrive_id="sig_1")
        tl2 = _FakeTrafficLight(10.0, 0.0, opendrive_id="sig_2")
        world = _FakeWorld([tl1, tl2])

        with patch(self._PATCH_TARGET, return_value=["sig_1", "sig_2"]):
            set_group_traffic_light_state(world, 100, MagicMock())

        assert tl1._frozen is True
        assert tl2._frozen is True

    def test_freeze_false(self) -> None:
        tl1 = _FakeTrafficLight(0.0, 0.0, opendrive_id="sig_1")
        tl2 = _FakeTrafficLight(10.0, 0.0, opendrive_id="sig_2")
        world = _FakeWorld([tl1, tl2])

        with patch(self._PATCH_TARGET, return_value=["sig_1", "sig_2"]):
            set_group_traffic_light_state(world, 100, MagicMock(), freeze=False)

        assert tl1._frozen is False
        assert tl2._frozen is False

    def test_only_matching_signals_affected(self) -> None:
        tl1 = _FakeTrafficLight(0.0, 0.0, opendrive_id="sig_1")
        tl2 = _FakeTrafficLight(10.0, 0.0, opendrive_id="sig_other")
        world = _FakeWorld([tl1, tl2])
        state = MagicMock(name="Red")

        with patch(self._PATCH_TARGET, return_value=["sig_1"]):
            set_group_traffic_light_state(world, 100, state)

        assert tl1._state is state
        assert tl1._frozen is True
        assert tl2._state is None
        assert tl2._frozen is False

    def test_no_signals_found(self) -> None:
        tl = _FakeTrafficLight(0.0, 0.0, opendrive_id="sig_1")
        world = _FakeWorld([tl])

        with patch(self._PATCH_TARGET, return_value=[]):
            set_group_traffic_light_state(world, 999, MagicMock())

        assert tl._state is None
        assert tl._frozen is False
