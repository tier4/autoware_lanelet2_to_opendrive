"""Unit tests for CarlaServerManager."""

from __future__ import annotations

import os

import pytest

from autoware_carla_scenario import CarlaServerManager


def test_start_raises_without_env_var(monkeypatch: pytest.MonkeyPatch) -> None:
    """CarlaServerManager.start() raises RuntimeError when env var is missing."""
    monkeypatch.delenv(CarlaServerManager.ENV_VAR, raising=False)
    manager = CarlaServerManager()
    with pytest.raises(RuntimeError, match=CarlaServerManager.ENV_VAR):
        manager.start()


def test_is_alive_false_before_start() -> None:
    """is_alive() returns False when no process has been started."""
    manager = CarlaServerManager()
    assert manager.is_alive() is False


def test_stop_is_idempotent() -> None:
    """Calling stop() on an unstarted manager must not raise."""
    manager = CarlaServerManager()
    manager.stop()  # Should not raise


class TestCarlaServerIntegration:
    """Integration tests that require CARLA to be installed."""

    @pytest.fixture(autouse=True)
    def skip_if_no_carla(self) -> None:
        if not os.environ.get(CarlaServerManager.ENV_VAR):
            pytest.skip("CARLA_UE5_EXECUTABLE not set")

    def test_context_manager_start_stop(self) -> None:
        """Server starts and stops cleanly via context manager."""
        with CarlaServerManager() as server:
            assert server.is_alive()
        # After __exit__ the process should be gone
        assert server._process is None

    def test_is_alive_after_stop(self) -> None:
        """is_alive() returns False after stop()."""
        manager = CarlaServerManager()
        manager.start()
        assert manager.is_alive()
        manager.stop()
        assert manager.is_alive() is False
