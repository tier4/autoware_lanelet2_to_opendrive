"""Unit tests for :class:`AutowareEgoEntity`.

The CARLA world/actor are faked with lightweight stand-ins so no live CARLA
server is required.  These tests verify the attach-instead-of-spawn behaviour,
the no-destroy lifecycle, and the initialization handshake driven through the
:class:`~autoware_carla_scenario.entity.ego.EgoVehicle` lifecycle hooks
(``on_scenario_start`` / ``on_tick`` / ``on_scenario_end``).
"""

from __future__ import annotations

from typing import List

import pytest

from autoware_carla_scenario.autoware_bridge import (
    AutowareBridgeConfig,
    BridgePose,
    FakeAutowareBridge,
    InitState,
)
from autoware_carla_scenario.constants import EGO_ROLE_NAME
from autoware_carla_scenario.entity import AutowareEgoEntity, AutowareEntity

_INITIAL = BridgePose(x=1.0, y=2.0, yaw=0.5)
_GOAL = BridgePose(x=10.0, y=20.0, yaw=1.5)


# ---------------------------------------------------------------------------
# Fake CARLA world / actor
# ---------------------------------------------------------------------------


class _FakeActor:
    def __init__(self, actor_id: int, role_name: str) -> None:
        self.id = actor_id
        self.attributes = {"role_name": role_name}
        self.destroyed = False

    def destroy(self) -> None:
        self.destroyed = True


class _FakeActorList:
    def __init__(self, actors: List[_FakeActor]) -> None:
        self._actors = actors

    def __iter__(self):
        return iter(self._actors)


class _FakeWorld:
    def __init__(self, actors: List[_FakeActor]) -> None:
        self._actors = actors

    def get_actors(self) -> _FakeActorList:
        return _FakeActorList(self._actors)


def _make_entity(bridge=None, **config_kwargs) -> AutowareEgoEntity:
    bridge = bridge if bridge is not None else FakeAutowareBridge()
    config = AutowareBridgeConfig(**config_kwargs) if config_kwargs else None
    return AutowareEgoEntity(
        config, bridge=bridge, initial_pose=_INITIAL, goal_pose=_GOAL
    )


def _drive_to_ready(entity: AutowareEgoEntity, world: _FakeWorld, max_ticks=50) -> None:
    entity.on_scenario_start(world)
    for _ in range(max_ticks):
        entity.on_tick(world, 0.0)
        if entity.is_initialized or entity.termination_requested:
            break


# ---------------------------------------------------------------------------
# Construction
# ---------------------------------------------------------------------------


def test_use_autopilot_is_false() -> None:
    # The runner relies on this to skip TrafficManager autopilot on the ego.
    assert AutowareEgoEntity.use_autopilot is False


def test_constructor_requires_bridge() -> None:
    # bridge is a required keyword argument until GrpcAutowareBridge lands.
    with pytest.raises(TypeError):
        AutowareEgoEntity(initial_pose=_INITIAL, goal_pose=_GOAL)  # type: ignore[call-arg]


def test_autoware_entity_placeholder_still_exists() -> None:
    # The bare placeholder is kept for backwards compatibility (used by run.py).
    assert AutowareEntity is not AutowareEgoEntity
    assert AutowareEntity.use_autopilot is False


# ---------------------------------------------------------------------------
# Attach behaviour
# ---------------------------------------------------------------------------


def test_spawn_attaches_to_existing_ego_actor() -> None:
    ego_actor = _FakeActor(42, str(EGO_ROLE_NAME))
    other = _FakeActor(1, "npc1")
    world = _FakeWorld([other, ego_actor])
    entity = _make_entity()

    attached = entity.spawn(world, config=None)  # type: ignore[arg-type]

    assert attached is ego_actor
    assert entity.actor is ego_actor


def test_spawn_times_out_when_ego_absent() -> None:
    world = _FakeWorld([_FakeActor(1, "npc1")])
    entity = _make_entity(attach_timeout=0.0)

    with pytest.raises(RuntimeError, match="No ego actor"):
        entity.spawn(world, config=None)  # type: ignore[arg-type]


def test_destroy_does_not_destroy_actor() -> None:
    ego_actor = _FakeActor(42, str(EGO_ROLE_NAME))
    world = _FakeWorld([ego_actor])
    entity = _make_entity()
    entity.spawn(world, config=None)  # type: ignore[arg-type]

    entity.destroy()

    # The interface node owns the actor lifecycle; we must not destroy it.
    assert ego_actor.destroyed is False
    assert entity.actor is None


# ---------------------------------------------------------------------------
# Initialization handshake via lifecycle hooks
# ---------------------------------------------------------------------------


def test_lifecycle_drives_handshake_to_engaged() -> None:
    bridge = FakeAutowareBridge()
    ego_actor = _FakeActor(42, str(EGO_ROLE_NAME))
    world = _FakeWorld([ego_actor])
    entity = _make_entity(bridge=bridge)
    entity.spawn(world, config=None)  # type: ignore[arg-type]

    assert entity.init_state is None
    _drive_to_ready(entity, world)

    assert entity.is_initialized
    assert entity.init_state is InitState.RUNNING
    assert not entity.termination_requested
    assert bridge.started is True  # transport/state stream opened before handshake
    assert bridge.initialized_pose == _INITIAL
    assert bridge.route_goal == _GOAL


def test_estimated_pose_exposed_after_localization() -> None:
    bridge = FakeAutowareBridge()
    ego_actor = _FakeActor(42, str(EGO_ROLE_NAME))
    world = _FakeWorld([ego_actor])
    entity = _make_entity(bridge=bridge)
    entity.spawn(world, config=None)  # type: ignore[arg-type]

    assert entity.estimated_pose is None  # before the handshake localizes
    _drive_to_ready(entity, world)

    # Autoware's estimate is exposed for monitoring; ground-truth pose for
    # conditions still comes from entity.actor / CARLA, not from here.
    assert entity.estimated_pose == _INITIAL


def test_on_scenario_start_before_spawn_raises() -> None:
    entity = _make_entity()
    world = _FakeWorld([])

    with pytest.raises(RuntimeError, match="before spawn"):
        entity.on_scenario_start(world)


def test_on_scenario_start_requires_poses() -> None:
    ego_actor = _FakeActor(42, str(EGO_ROLE_NAME))
    world = _FakeWorld([ego_actor])
    entity = AutowareEgoEntity(bridge=FakeAutowareBridge())  # no poses
    entity.spawn(world, config=None)  # type: ignore[arg-type]

    with pytest.raises(ValueError, match="initial_pose and goal_pose"):
        entity.on_scenario_start(world)


def test_on_tick_before_start_is_noop() -> None:
    entity = _make_entity()
    world = _FakeWorld([])
    # No init sequence yet -> must not raise.
    entity.on_tick(world, 0.0)
    assert entity.init_state is None


def test_failed_handshake_requests_termination() -> None:
    # Autoware never reports ready within the step budget.
    bridge = FakeAutowareBridge(ready_after=10_000)
    ego_actor = _FakeActor(42, str(EGO_ROLE_NAME))
    world = _FakeWorld([ego_actor])
    entity = _make_entity(bridge=bridge, step_timeout=3)
    entity.spawn(world, config=None)  # type: ignore[arg-type]

    entity.on_scenario_start(world)
    for _ in range(50):
        entity.on_tick(world, 0.0)
        if entity.termination_requested:
            break

    assert entity.termination_requested
    assert not entity.is_initialized
    assert entity.init_state is InitState.FAILED


def test_on_scenario_start_opens_bridge_stream() -> None:
    bridge = FakeAutowareBridge()
    world = _FakeWorld([_FakeActor(42, str(EGO_ROLE_NAME))])
    entity = _make_entity(bridge=bridge)
    entity.spawn(world, config=None)  # type: ignore[arg-type]

    entity.on_scenario_start(world)

    # The transport/state stream is opened before the handshake begins so the
    # per-tick query methods stay non-blocking.
    assert bridge.started is True


def test_on_scenario_end_closes_bridge() -> None:
    bridge = FakeAutowareBridge()
    world = _FakeWorld([_FakeActor(42, str(EGO_ROLE_NAME))])
    entity = _make_entity(bridge=bridge)

    entity.on_scenario_end(world)

    assert bridge.closed is True
