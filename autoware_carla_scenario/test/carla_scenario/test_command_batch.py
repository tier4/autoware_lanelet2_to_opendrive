"""Unit tests for CommandBatch (no CARLA required).

``CommandBatch`` only imports ``carla`` under ``TYPE_CHECKING``, so these
tests exercise it with plain stand-in objects and a mock client.
"""

from __future__ import annotations

from unittest.mock import MagicMock

from autoware_carla_scenario.command_batch import CommandBatch


class _FakeResponse:
    """Stand-in for ``carla.command.Response``."""

    def __init__(self, error: str = "") -> None:
        self.error = error


class TestCommandBatch:
    def test_empty_batch_has_zero_length(self) -> None:
        batch = CommandBatch()
        assert len(batch) == 0

    def test_add_accumulates_commands(self) -> None:
        batch = CommandBatch()
        batch.add("cmd-a")
        batch.add("cmd-b")
        assert len(batch) == 2
        assert batch.commands == ["cmd-a", "cmd-b"]

    def test_commands_property_returns_copy(self) -> None:
        batch = CommandBatch()
        batch.add("cmd-a")
        snapshot = batch.commands
        snapshot.append("mutated")
        # Mutating the returned list must not affect the batch.
        assert batch.commands == ["cmd-a"]

    def test_apply_empty_batch_skips_rpc(self) -> None:
        client = MagicMock()
        batch = CommandBatch()
        result = batch.apply(client)
        assert result == []
        client.apply_batch_sync.assert_not_called()

    def test_apply_dispatches_single_batch_sync_call(self) -> None:
        client = MagicMock()
        client.apply_batch_sync.return_value = [_FakeResponse(), _FakeResponse()]
        batch = CommandBatch()
        batch.add("cmd-a")
        batch.add("cmd-b")

        responses = batch.apply(client)

        # One RPC for the whole frame, not one per command.
        client.apply_batch_sync.assert_called_once_with(["cmd-a", "cmd-b"], False)
        assert len(responses) == 2
        # Batch is emptied after a successful apply.
        assert len(batch) == 0

    def test_apply_forwards_due_tick_cue(self) -> None:
        client = MagicMock()
        client.apply_batch_sync.return_value = []
        batch = CommandBatch()
        batch.add("cmd-a")

        batch.apply(client, due_tick_cue=True)

        client.apply_batch_sync.assert_called_once_with(["cmd-a"], True)

    def test_apply_clears_batch_even_when_rpc_raises(self) -> None:
        client = MagicMock()
        client.apply_batch_sync.side_effect = RuntimeError("boom")
        batch = CommandBatch()
        batch.add("cmd-a")

        try:
            batch.apply(client)
        except RuntimeError:
            pass

        assert len(batch) == 0

    def test_apply_logs_per_command_errors(self, caplog) -> None:
        client = MagicMock()
        client.apply_batch_sync.return_value = [
            _FakeResponse(),
            _FakeResponse(error="actor not found"),
        ]
        batch = CommandBatch()
        batch.add("cmd-a")
        batch.add("cmd-b")

        with caplog.at_level("WARNING"):
            batch.apply(client)

        assert "actor not found" in caplog.text
