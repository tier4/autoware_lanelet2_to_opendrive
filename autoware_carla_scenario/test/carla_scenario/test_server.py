"""Unit tests for CarlaServerManager."""

from __future__ import annotations

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
    """Integration tests that verify the session-scoped CARLA server.

    These tests reuse the server already started by the ``carla_queue``
    session fixture.  They do NOT start a new CarlaServerManager because
    doing so would attempt to bind the same port twice.
    """

    @pytest.fixture(autouse=True)
    def skip_if_no_carla(self, carla_queue) -> None:  # noqa: ANN001
        """Depend on the session fixture; skips automatically if CARLA is unavailable."""

    def test_server_is_alive(self, carla_queue) -> None:  # noqa: ANN001
        """The session server must be reachable during the test run."""
        assert carla_queue._server.is_alive()

    def test_server_process_is_running(self, carla_queue) -> None:  # noqa: ANN001
        """The server process is alive (owned or reused)."""
        server = carla_queue._server
        if server._reused:
            # Externally-managed server: no _process, but must be pingable.
            assert server._ping()
        else:
            assert server._process is not None
            assert server._process.poll() is None
