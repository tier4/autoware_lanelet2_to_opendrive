"""Unit tests for the relative-position action and condition.

These exercise the longitudinal-gap control law and the gap-reached check with
mocked CARLA actors (no simulator required, but the framework import needs
``carla`` present, so they run in CI rather than on a bare host).
"""

from __future__ import annotations

from unittest.mock import MagicMock

import pytest

from autoware_carla_scenario import (
    RelativePositionAction,
    RelativePositionCondition,
)


def _vec(x: float, y: float, z: float = 0.0) -> MagicMock:
    v = MagicMock()
    v.x, v.y, v.z = x, y, z
    return v


def _actor(role: str, *, x: float, y: float, vx: float = 0.0) -> MagicMock:
    """A mock vehicle facing +x at (x, y) with longitudinal speed vx (m/s)."""
    actor = MagicMock()
    actor.attributes = {"role_name": role}
    actor.get_location.return_value = _vec(x, y)
    actor.get_velocity.return_value = _vec(vx, 0.0)
    transform = MagicMock()
    transform.get_forward_vector.return_value = _vec(1.0, 0.0)
    actor.get_transform.return_value = transform
    return actor


def _world(*actors: MagicMock) -> MagicMock:
    world = MagicMock()
    world.get_actors.return_value = list(actors)
    return world


def _client_capturing() -> tuple[MagicMock, list[float]]:
    calls: list[float] = []
    tm = MagicMock()
    tm.set_desired_speed.side_effect = lambda actor, kmh: calls.append(kmh)
    client = MagicMock()
    client.get_trafficmanager.return_value = tm
    return client, calls


class TestRelativePositionAction:
    def test_behind_reference_speeds_up_to_max(self) -> None:
        # ego at 0 (36 km/h), npc 15 m behind, target +20 m ahead.
        ego = _actor("ego", x=0.0, y=0.0, vx=10.0)
        npc = _actor("npc", x=-15.0, y=0.0, vx=10.0)
        client, calls = _client_capturing()
        action = RelativePositionAction(
            "npc", "ego", target_gap=20.0, client=client, gain=2.0, max_speed_kmh=60.0
        )
        action.execute(_world(ego, npc))
        # error = 20 - (-15) = 35 -> 36 + 70 = 106, clamped to 60.
        assert calls == [pytest.approx(60.0)]

    def test_too_far_ahead_slows_down(self) -> None:
        ego = _actor("ego", x=0.0, y=0.0, vx=10.0)
        npc = _actor("npc", x=25.0, y=0.0, vx=10.0)
        client, calls = _client_capturing()
        action = RelativePositionAction(
            "npc", "ego", target_gap=20.0, client=client, gain=2.0, max_speed_kmh=60.0
        )
        action.execute(_world(ego, npc))
        # error = 20 - 25 = -5 -> 36 - 10 = 26.
        assert calls == [pytest.approx(26.0)]

    def test_missing_actor_is_graceful(self) -> None:
        ego = _actor("ego", x=0.0, y=0.0, vx=10.0)
        client, calls = _client_capturing()
        action = RelativePositionAction("npc", "ego", target_gap=20.0, client=client)
        action.execute(_world(ego))  # npc absent
        assert calls == []

    def test_is_continuous_by_default(self) -> None:
        action = RelativePositionAction(
            "npc", "ego", target_gap=5.0, client=MagicMock()
        )
        assert action._once is False  # noqa: SLF001


class TestRelativePositionCondition:
    def test_passes_within_tolerance(self) -> None:
        ego = _actor("ego", x=0.0, y=0.0)
        npc = _actor("npc", x=20.5, y=0.0)
        cond = RelativePositionCondition(
            "npc", "ego", target_gap=20.0, tolerance=2.0, label="goal"
        )
        result = cond.check(_world(ego, npc), 5.0)
        assert result is not None and result.passed

    def test_none_outside_tolerance(self) -> None:
        ego = _actor("ego", x=0.0, y=0.0)
        npc = _actor("npc", x=10.0, y=0.0)
        cond = RelativePositionCondition(
            "npc", "ego", target_gap=20.0, tolerance=2.0, label="goal"
        )
        assert cond.check(_world(ego, npc), 5.0) is None

    def test_none_when_reference_missing(self) -> None:
        npc = _actor("npc", x=20.0, y=0.0)
        cond = RelativePositionCondition(
            "npc", "ego", target_gap=20.0, tolerance=2.0, label="goal"
        )
        assert cond.check(_world(npc), 5.0) is None
