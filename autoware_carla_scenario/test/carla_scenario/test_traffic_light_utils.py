"""Unit tests for traffic light utility functions (no CARLA required)."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List

import pytest

from autoware_carla_scenario.utils.traffic_light import (
    find_nearest_traffic_light,
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

    def get_actors(self) -> "_FakeActorList":
        return _FakeActorList(self._actors)


class _FakeActorList:
    """Stub for CARLA ActorList with filter support."""

    def __init__(self, actors: List[_FakeTrafficLight]) -> None:
        self._actors = actors

    def filter(self, pattern: str) -> List[_FakeTrafficLight]:
        """Filter actors by type_id pattern (simplified glob match)."""
        base = pattern.rstrip("*")
        return [a for a in self._actors if a.type_id.startswith(base)]

    def __iter__(self):
        return iter(self._actors)

    def __len__(self):
        return len(self._actors)


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
        world = _FakeWorld([tl_far, tl_close])
        origin = _FakeLocation(0.0, 0.0, 0.0)

        nearest, dist = find_nearest_traffic_light(world, origin)

        assert nearest is tl_close
        assert dist == pytest.approx(1.0)

    def test_all_beyond_max_distance(self) -> None:
        tl = _FakeTrafficLight(200.0, 0.0)
        world = _FakeWorld([tl])
        origin = _FakeLocation(0.0, 0.0, 0.0)

        nearest, dist = find_nearest_traffic_light(world, origin, max_distance=50.0)

        assert nearest is None
        assert dist == float("inf")

    def test_empty_world(self) -> None:
        world = _FakeWorld([])
        origin = _FakeLocation(0.0, 0.0, 0.0)

        nearest, dist = find_nearest_traffic_light(world, origin)

        assert nearest is None
        assert dist == float("inf")

    def test_custom_max_distance(self) -> None:
        tl = _FakeTrafficLight(10.0, 0.0)
        world = _FakeWorld([tl])
        origin = _FakeLocation(0.0, 0.0, 0.0)

        nearest, dist = find_nearest_traffic_light(world, origin, max_distance=5.0)
        assert nearest is None

        nearest, dist = find_nearest_traffic_light(world, origin, max_distance=15.0)
        assert nearest is tl

    def test_multiple_lights_selects_closest(self) -> None:
        tl_a = _FakeTrafficLight(5.0, 0.0)
        tl_b = _FakeTrafficLight(3.0, 0.0)
        tl_c = _FakeTrafficLight(8.0, 0.0)
        world = _FakeWorld([tl_a, tl_b, tl_c])
        origin = _FakeLocation(0.0, 0.0, 0.0)

        nearest, dist = find_nearest_traffic_light(world, origin)

        assert nearest is tl_b
        assert dist == pytest.approx(3.0)

    def test_ignores_non_traffic_light_actors(self) -> None:
        tl = _FakeTrafficLight(1.0, 0.0)
        non_tl = _FakeTrafficLight(0.5, 0.0)
        non_tl.type_id = "vehicle.car"
        world = _FakeWorld([non_tl, tl])
        origin = _FakeLocation(0.0, 0.0, 0.0)

        nearest, dist = find_nearest_traffic_light(world, origin)

        assert nearest is tl
        assert dist == pytest.approx(1.0)
