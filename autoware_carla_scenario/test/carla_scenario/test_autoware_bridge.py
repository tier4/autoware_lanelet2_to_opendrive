"""Unit tests for the Autoware bridge contract and initialization sequence.

These tests are CARLA- and ROS 2-free: they exercise the abstract contract via
:class:`FakeAutowareBridge` and the :class:`AutowareInitSequence` state machine.
"""

from __future__ import annotations

from autoware_carla_scenario.autoware_bridge import (
    AutowareInitSequence,
    BridgePose,
    FakeAutowareBridge,
    InitState,
    LocalizationState,
    OperationMode,
    RoutingState,
)

_INITIAL = BridgePose.from_yaw(x=1.0, y=2.0, z=0.5, yaw=0.5)
_GOAL = BridgePose.from_yaw(x=10.0, y=20.0, z=3.0, yaw=1.5)


def _run_to_completion(seq: AutowareInitSequence, max_steps: int = 10_000) -> int:
    """Step the sequence until terminal; return the number of steps taken."""
    steps = 0
    while not seq.is_done and steps < max_steps:
        seq.step()
        steps += 1
    return steps


# ---------------------------------------------------------------------------
# FakeAutowareBridge
# ---------------------------------------------------------------------------


def test_fake_bridge_progresses_through_stages() -> None:
    bridge = FakeAutowareBridge()

    assert bridge.is_autoware_ready() is True

    assert bridge.get_localization_state() is LocalizationState.UNINITIALIZED
    bridge.initialize_pose(_INITIAL)
    assert bridge.initialized_pose == _INITIAL
    assert bridge.get_localization_state() is LocalizationState.INITIALIZED

    assert bridge.get_routing_state() is RoutingState.UNSET
    bridge.set_route(_GOAL)
    assert bridge.route_goal == _GOAL
    assert bridge.get_routing_state() is RoutingState.SET

    assert bridge.get_operation_mode() is OperationMode.STOP
    bridge.change_to_autonomous()
    assert bridge.get_operation_mode() is OperationMode.AUTONOMOUS


def test_fake_bridge_honours_delays() -> None:
    bridge = FakeAutowareBridge(ready_after=2, localize_after=3)

    assert bridge.is_autoware_ready() is False
    assert bridge.is_autoware_ready() is False
    assert bridge.is_autoware_ready() is True

    bridge.initialize_pose(_INITIAL)
    states = [bridge.get_localization_state() for _ in range(4)]
    assert states[:3] == [LocalizationState.INITIALIZING] * 3
    assert states[3] is LocalizationState.INITIALIZED


def test_fake_bridge_estimated_pose_after_localization() -> None:
    bridge = FakeAutowareBridge()
    assert bridge.get_estimated_pose() is None  # before pose init
    bridge.initialize_pose(_INITIAL)
    bridge.get_localization_state()  # advances to INITIALIZED (localize_after=0)
    assert bridge.get_estimated_pose() == _INITIAL


def test_fake_bridge_close_is_recorded() -> None:
    bridge = FakeAutowareBridge()
    assert bridge.closed is False
    bridge.close()
    assert bridge.closed is True
    assert "close" in bridge.calls


# ---------------------------------------------------------------------------
# AutowareInitSequence
# ---------------------------------------------------------------------------


def test_init_sequence_reaches_running() -> None:
    bridge = FakeAutowareBridge()
    seq = AutowareInitSequence(bridge, _INITIAL, _GOAL)

    assert seq.state is InitState.IDLE
    _run_to_completion(seq)

    assert seq.is_ready
    assert seq.state is InitState.RUNNING
    assert not seq.failed
    assert seq.failure_reason is None

    # Every handshake action was invoked in order.
    for expected in (
        "is_autoware_ready",
        "initialize_pose",
        "get_localization_state",
        "set_route",
        "get_routing_state",
        "change_to_autonomous",
        "get_operation_mode",
    ):
        assert expected in bridge.calls
    assert bridge.calls.index("initialize_pose") < bridge.calls.index("set_route")
    assert bridge.calls.index("set_route") < bridge.calls.index("change_to_autonomous")

    # The correct poses were forwarded.
    assert bridge.initialized_pose == _INITIAL
    assert bridge.route_goal == _GOAL


def test_init_sequence_visits_each_state() -> None:
    bridge = FakeAutowareBridge(
        ready_after=1, localize_after=1, route_after=1, engage_after=1
    )
    seq = AutowareInitSequence(bridge, _INITIAL, _GOAL)

    seen: set[InitState] = set()
    while not seq.is_done:
        seen.add(seq.step())

    for state in (
        InitState.WAIT_READY,
        InitState.WAIT_LOCALIZED,
        InitState.WAIT_ROUTE,
        InitState.WAIT_ENGAGED,
        InitState.RUNNING,
    ):
        assert state in seen


def test_init_sequence_times_out_when_never_ready() -> None:
    # ready_after larger than the timeout budget -> WAIT_READY never completes.
    bridge = FakeAutowareBridge(ready_after=1000)
    seq = AutowareInitSequence(bridge, _INITIAL, _GOAL, step_timeout=5)

    _run_to_completion(seq)

    assert seq.failed
    assert seq.state is InitState.FAILED
    assert not seq.is_ready
    assert seq.failure_reason is not None
    assert "WAIT_READY" in seq.failure_reason


def test_init_sequence_step_is_noop_after_terminal() -> None:
    bridge = FakeAutowareBridge()
    seq = AutowareInitSequence(bridge, _INITIAL, _GOAL)
    _run_to_completion(seq)

    n_calls = len(bridge.calls)
    assert seq.step() is InitState.RUNNING
    # No further bridge interaction once terminal.
    assert len(bridge.calls) == n_calls
