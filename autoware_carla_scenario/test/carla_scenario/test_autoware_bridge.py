"""Unit tests for the minimal Autoware bridge contract.

CARLA- and ROS 2-free: exercises the abstract contract via
:class:`FakeAutowareBridge`.
"""

from __future__ import annotations

from autoware_carla_scenario.autoware_bridge import BridgePose, FakeAutowareBridge

_INITIAL = BridgePose.from_yaw(x=1.0, y=2.0, z=0.5, yaw=0.5)
_GOAL = BridgePose.from_yaw(x=10.0, y=20.0, z=3.0, yaw=1.5)


def test_not_ready_before_configure() -> None:
    bridge = FakeAutowareBridge()
    assert bridge.is_ready() is False


def test_ready_after_configure() -> None:
    bridge = FakeAutowareBridge()
    bridge.configure(_INITIAL, _GOAL)

    assert bridge.configured_initial_pose == _INITIAL
    assert bridge.configured_goal == _GOAL
    assert bridge.is_ready() is True


def test_ready_honours_delay() -> None:
    bridge = FakeAutowareBridge(ready_after=3)
    bridge.configure(_INITIAL, _GOAL)

    readies = [bridge.is_ready() for _ in range(4)]
    assert readies == [False, False, False, True]


def test_close_is_recorded() -> None:
    bridge = FakeAutowareBridge()
    assert bridge.closed is False
    bridge.close()
    assert bridge.closed is True
    assert "close" in bridge.calls


def test_configure_recorded_in_calls() -> None:
    bridge = FakeAutowareBridge()
    bridge.configure(_INITIAL, _GOAL)
    assert "configure" in bridge.calls
