"""Unit tests for the tick-callback batching helpers in scenario_runner."""

from __future__ import annotations

from unittest.mock import MagicMock

from autoware_carla_scenario.command_batch import CommandBatch
from autoware_carla_scenario.scenario_runner import (
    _callback_wants_batch,
    _invoke_tick_callback,
)


class TestCallbackWantsBatch:
    def test_single_arg_callable_does_not_want_batch(self) -> None:
        assert _callback_wants_batch(lambda world: None) is False

    def test_two_arg_callable_wants_batch(self) -> None:
        assert _callback_wants_batch(lambda world, batch: None) is True

    def test_var_positional_wants_batch(self) -> None:
        assert _callback_wants_batch(lambda *args: None) is True

    def test_unreadable_signature_defaults_to_no_batch(self) -> None:
        # ``inspect.signature`` raises on a bare MagicMock, so the legacy
        # single-argument form must be assumed.
        assert _callback_wants_batch(MagicMock()) is False


class TestInvokeTickCallback:
    def test_legacy_callback_invoked_with_world_only(self) -> None:
        received: list = []
        cb = lambda world: received.append(world)  # noqa: E731
        world = object()
        _invoke_tick_callback(cb, world, CommandBatch())
        assert received == [world]

    def test_batch_aware_callback_receives_batch(self) -> None:
        received: list = []
        cb = lambda world, batch: received.append((world, batch))  # noqa: E731
        world = object()
        batch = CommandBatch()
        _invoke_tick_callback(cb, world, batch)
        assert received == [(world, batch)]

    def test_batch_aware_callback_can_emit_commands(self) -> None:
        def cb(world: object, batch: CommandBatch) -> None:
            batch.add("cmd")

        batch = CommandBatch()
        _invoke_tick_callback(cb, object(), batch)
        assert batch.commands == ["cmd"]
