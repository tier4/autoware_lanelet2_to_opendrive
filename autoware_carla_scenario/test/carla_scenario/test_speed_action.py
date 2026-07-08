"""Unit tests for SpeedAction (timed cruise-speed command via TrafficManager).

Uses mocked CARLA actors; the framework import needs ``carla`` present, so
these run in CI rather than on a bare host.
"""

from __future__ import annotations

from unittest.mock import MagicMock

from autoware_carla_scenario import SpeedAction


def _actor(role: str) -> MagicMock:
    actor = MagicMock()
    actor.attributes = {"role_name": role}
    return actor


def _world(*actors: MagicMock) -> MagicMock:
    world = MagicMock()
    world.get_actors.return_value = list(actors)
    return world


def _client_capturing() -> tuple[MagicMock, list[tuple[object, float]]]:
    calls: list[tuple[object, float]] = []
    tm = MagicMock()
    tm.set_desired_speed.side_effect = lambda actor, kmh: calls.append((actor, kmh))
    client = MagicMock()
    client.get_trafficmanager.return_value = tm
    return client, calls


def test_sets_desired_speed_on_execute() -> None:
    npc = _actor("npc")
    client, calls = _client_capturing()
    action = SpeedAction("npc", target_speed_kmh=20.0, client=client)
    action.execute(_world(npc))
    assert calls == [(npc, 20.0)]


def test_missing_actor_is_graceful() -> None:
    client, calls = _client_capturing()
    action = SpeedAction("npc", target_speed_kmh=20.0, client=client)
    action.execute(_world(_actor("ego")))  # npc absent
    assert calls == []


def test_is_one_shot_by_default() -> None:
    action = SpeedAction("npc", target_speed_kmh=20.0, client=MagicMock())
    assert action._once is True  # noqa: SLF001
