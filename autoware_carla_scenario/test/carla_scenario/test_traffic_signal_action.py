"""Unit tests for TrafficSignalAction (no CARLA required)."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List
from unittest.mock import MagicMock, patch


from autoware_carla_scenario.actions.traffic_signal import (
    TrafficLightTarget,
    TrafficSignalAction,
)
from autoware_carla_scenario.conditions.always_true import AlwaysTrueCondition
from autoware_carla_scenario.conditions.elapsed_time import ElapsedTimeCondition


# ---------------------------------------------------------------------------
# Lightweight stubs for CARLA types
# ---------------------------------------------------------------------------


@dataclass
class _FakeLocation:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class _FakeTransform:
    location: _FakeLocation = field(default_factory=_FakeLocation)


class _FakeTrafficLight:
    """Minimal traffic light stub."""

    def __init__(self, opendrive_id: str = "") -> None:
        self._state: object = None
        self._frozen: bool = False
        self._opendrive_id = opendrive_id
        self.type_id = "traffic.traffic_light"

    def get_opendrive_id(self) -> str:
        return self._opendrive_id

    def set_state(self, state: object) -> None:
        self._state = state

    def freeze(self, frozen: bool) -> None:
        self._frozen = frozen

    def get_transform(self) -> _FakeTransform:
        return _FakeTransform()


class _FakeActorList:
    """Stub for CARLA ActorList with filter support."""

    def __init__(self, actors: List[_FakeTrafficLight]) -> None:
        self._actors = actors

    def filter(self, pattern: str) -> List[_FakeTrafficLight]:
        base = pattern.rstrip("*")
        return [a for a in self._actors if a.type_id.startswith(base)]

    def __iter__(self):
        return iter(self._actors)

    def __len__(self):
        return len(self._actors)


class _FakeWorld:
    """Minimal CARLA world stub."""

    def __init__(self, actors: List[_FakeTrafficLight]) -> None:
        self._actors = actors

    def get_actors(self) -> _FakeActorList:
        return _FakeActorList(self._actors)


# ---------------------------------------------------------------------------
# TrafficSignalAction — execute with TrafficLightTarget.ALL
# ---------------------------------------------------------------------------


class TestTrafficSignalActionAll:
    def test_sets_all_traffic_lights(self) -> None:
        tl1 = _FakeTrafficLight()
        tl2 = _FakeTrafficLight()
        world = _FakeWorld([tl1, tl2])
        state = MagicMock(name="Green")

        action = TrafficSignalAction(
            state=state,
            lanelet2_traffic_light_ids=TrafficLightTarget.ALL,
            label="set_all_green",
        )
        action.execute(world)

        assert tl1._state is state
        assert tl2._state is state

    def test_freezes_all_by_default(self) -> None:
        tl1 = _FakeTrafficLight()
        tl2 = _FakeTrafficLight()
        world = _FakeWorld([tl1, tl2])

        action = TrafficSignalAction(
            state=MagicMock(),
            lanelet2_traffic_light_ids=TrafficLightTarget.ALL,
            label="freeze_test",
        )
        action.execute(world)

        assert tl1._frozen is True
        assert tl2._frozen is True

    def test_freeze_false(self) -> None:
        tl1 = _FakeTrafficLight()
        tl2 = _FakeTrafficLight()
        world = _FakeWorld([tl1, tl2])

        action = TrafficSignalAction(
            state=MagicMock(),
            lanelet2_traffic_light_ids=TrafficLightTarget.ALL,
            freeze=False,
            label="no_freeze_test",
        )
        action.execute(world)

        assert tl1._frozen is False
        assert tl2._frozen is False


# ---------------------------------------------------------------------------
# TrafficSignalAction — execute with None (NOP)
# ---------------------------------------------------------------------------


class TestTrafficSignalActionNone:
    def test_nop_does_nothing(self) -> None:
        tl = _FakeTrafficLight()
        world = _FakeWorld([tl])

        action = TrafficSignalAction(
            state=MagicMock(),
            lanelet2_traffic_light_ids=None,
            label="nop_test",
        )
        action.execute(world)

        assert tl._state is None
        assert tl._frozen is False


# ---------------------------------------------------------------------------
# TrafficSignalAction — execute with specific Lanelet2 IDs
# ---------------------------------------------------------------------------

_PATCH_CONTROLLER_ID = (
    "autoware_carla_scenario.actions.traffic_signal"
    ".lanelet2_traffic_light_id_to_opendrive_controller_id"
)
_PATCH_SIGNAL_IDS = (
    "autoware_carla_scenario.actions.traffic_signal.get_signal_ids_for_controller"
)


class TestTrafficSignalActionByLanelet2Ids:
    def test_sets_matching_signals_only(self) -> None:
        tl1 = _FakeTrafficLight(opendrive_id="sig_1")
        tl2 = _FakeTrafficLight(opendrive_id="sig_other")
        world = _FakeWorld([tl1, tl2])
        state = MagicMock(name="Red")

        with (
            patch(_PATCH_CONTROLLER_ID, return_value=100),
            patch(_PATCH_SIGNAL_IDS, return_value=["sig_1"]),
        ):
            action = TrafficSignalAction(
                state=state,
                lanelet2_traffic_light_ids=[242],
                label="specific_ids_test",
            )
            action.execute(world)

        assert tl1._state is state
        assert tl1._frozen is True
        assert tl2._state is None
        assert tl2._frozen is False

    def test_sets_all_matching_signals_for_multiple_ids(self) -> None:
        tl1 = _FakeTrafficLight(opendrive_id="sig_1")
        tl2 = _FakeTrafficLight(opendrive_id="sig_2")
        tl3 = _FakeTrafficLight(opendrive_id="sig_other")
        world = _FakeWorld([tl1, tl2, tl3])
        state = MagicMock(name="Green")

        def fake_controller_id(ll2_id: int):
            return {242: 100, 460: 200}.get(ll2_id)

        def fake_signal_ids(controller_id: int):
            return {100: ["sig_1"], 200: ["sig_2"]}.get(controller_id, [])

        with (
            patch(_PATCH_CONTROLLER_ID, side_effect=fake_controller_id),
            patch(_PATCH_SIGNAL_IDS, side_effect=fake_signal_ids),
        ):
            action = TrafficSignalAction(
                state=state,
                lanelet2_traffic_light_ids=[242, 460],
                label="multi_ids_test",
            )
            action.execute(world)

        assert tl1._state is state
        assert tl2._state is state
        assert tl3._state is None

    def test_unknown_lanelet2_id_warns(self) -> None:
        tl = _FakeTrafficLight(opendrive_id="sig_1")
        world = _FakeWorld([tl])

        with (
            patch(_PATCH_CONTROLLER_ID, return_value=None),
            patch(_PATCH_SIGNAL_IDS, return_value=[]),
        ):
            action = TrafficSignalAction(
                state=MagicMock(),
                lanelet2_traffic_light_ids=[999],
                label="unknown_id_test",
            )
            action.execute(world)

        # NOP — nothing should be set
        assert tl._state is None
        assert tl._frozen is False

    def test_freeze_false_for_specific_ids(self) -> None:
        tl = _FakeTrafficLight(opendrive_id="sig_1")
        world = _FakeWorld([tl])
        state = MagicMock(name="Yellow")

        with (
            patch(_PATCH_CONTROLLER_ID, return_value=100),
            patch(_PATCH_SIGNAL_IDS, return_value=["sig_1"]),
        ):
            action = TrafficSignalAction(
                state=state,
                lanelet2_traffic_light_ids=[242],
                freeze=False,
                label="no_freeze_specific_test",
            )
            action.execute(world)

        assert tl._state is state
        assert tl._frozen is False


# ---------------------------------------------------------------------------
# TrafficSignalAction — tick + once behaviour
# ---------------------------------------------------------------------------


class TestTrafficSignalActionTick:
    def test_tick_with_elapsed_time(self) -> None:
        """Verify action fires only after elapsed time passes the condition threshold."""
        tl = _FakeTrafficLight()
        world = _FakeWorld([tl])
        state = MagicMock(name="Green")

        action = TrafficSignalAction(
            state=state,
            lanelet2_traffic_light_ids=TrafficLightTarget.ALL,
            condition=ElapsedTimeCondition(3.0, label="delay"),
            label="tick_test",
        )

        # Before threshold: should not fire
        action.tick(world, elapsed=2.0)
        assert tl._state is None

        # After threshold: should fire
        action.tick(world, elapsed=3.5)
        assert tl._state is state

    def test_once_prevents_re_execution(self) -> None:
        """Verify once=True prevents the action from firing again."""
        tl = _FakeTrafficLight()
        world = _FakeWorld([tl])
        state1 = MagicMock(name="Green")

        action = TrafficSignalAction(
            state=state1,
            lanelet2_traffic_light_ids=TrafficLightTarget.ALL,
            condition=AlwaysTrueCondition(),
            once=True,
            label="once_test",
        )

        action.tick(world, elapsed=0.0)
        assert tl._state is state1
        assert action.done is True

        # Reset the traffic light state to verify no second execution
        tl._state = None
        action.tick(world, elapsed=1.0)
        assert tl._state is None

    def test_once_false_allows_re_execution(self) -> None:
        """Verify once=False allows repeated execution."""
        tl = _FakeTrafficLight()
        world = _FakeWorld([tl])
        state = MagicMock(name="Red")

        action = TrafficSignalAction(
            state=state,
            lanelet2_traffic_light_ids=TrafficLightTarget.ALL,
            condition=AlwaysTrueCondition(),
            once=False,
            label="repeat_test",
        )

        action.tick(world, elapsed=0.0)
        assert tl._state is state

        tl._state = None
        action.tick(world, elapsed=1.0)
        assert tl._state is state
