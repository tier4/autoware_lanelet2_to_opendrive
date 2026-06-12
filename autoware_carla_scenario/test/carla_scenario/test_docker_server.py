"""Unit tests for CarlaDockerServerManager."""

from __future__ import annotations

from unittest.mock import MagicMock, patch

import pytest

from autoware_carla_scenario import CarlaDockerServerManager


def test_default_image_from_env_var(monkeypatch: pytest.MonkeyPatch) -> None:
    """The image falls back to CARLA_DOCKER_IMAGE when not given explicitly."""
    monkeypatch.setenv(CarlaDockerServerManager.ENV_VAR_IMAGE, "myorg/carla:custom")
    manager = CarlaDockerServerManager()
    assert manager.image == "myorg/carla:custom"


def test_default_image_when_env_var_missing(monkeypatch: pytest.MonkeyPatch) -> None:
    """The image falls back to carlasim/carla:<tag> when env var is not set."""
    monkeypatch.delenv(CarlaDockerServerManager.ENV_VAR_IMAGE, raising=False)
    manager = CarlaDockerServerManager()
    assert manager.image.startswith("carlasim/carla:")
    assert manager.image == (
        f"{CarlaDockerServerManager.DEFAULT_IMAGE_REPOSITORY}:"
        f"{CarlaDockerServerManager.DEFAULT_IMAGE_TAG}"
    )


def test_explicit_image_overrides_env(monkeypatch: pytest.MonkeyPatch) -> None:
    """Constructor argument wins over the env var."""
    monkeypatch.setenv(CarlaDockerServerManager.ENV_VAR_IMAGE, "from-env")
    manager = CarlaDockerServerManager(image="from-arg")
    assert manager.image == "from-arg"


def test_default_image_tag_override(monkeypatch: pytest.MonkeyPatch) -> None:
    """Monkey-patching DEFAULT_IMAGE_TAG changes the resolved default image."""
    monkeypatch.delenv(CarlaDockerServerManager.ENV_VAR_IMAGE, raising=False)
    monkeypatch.setattr(CarlaDockerServerManager, "DEFAULT_IMAGE_TAG", "0.9.16")
    manager = CarlaDockerServerManager()
    assert manager.image == "carlasim/carla:0.9.16"


def test_is_alive_false_before_start() -> None:
    """is_alive() returns False when no container has been started."""
    manager = CarlaDockerServerManager(reuse_if_running=False)
    assert manager.is_alive() is False


def test_stop_is_idempotent() -> None:
    """Calling stop() on an unstarted manager must not raise."""
    manager = CarlaDockerServerManager()
    manager.stop()  # Should not raise


def test_start_reuses_running_server() -> None:
    """start() skips Docker entirely if the RPC port is already serving."""
    manager = CarlaDockerServerManager(reuse_if_running=True)
    with (
        patch.object(manager, "_ping", return_value=True),
        patch.object(manager, "_get_client") as mock_get_client,
    ):
        manager.start()
    assert manager._reused is True
    assert manager._container is None
    mock_get_client.assert_not_called()


def test_stop_is_noop_when_reused() -> None:
    """stop() must not touch Docker if the server was reused."""
    manager = CarlaDockerServerManager(reuse_if_running=True)
    with patch.object(manager, "_ping", return_value=True):
        manager.start()
    assert manager._reused is True
    # No container, so stop() should just return.
    manager.stop()


def test_start_launches_container_when_not_running() -> None:
    """When no server is reachable, start() runs a new container via the SDK."""
    manager = CarlaDockerServerManager(
        image="carlasim/carla:0.9.15",
        reuse_if_running=False,
        timeout=1.0,
    )

    fake_container = MagicMock()
    fake_client = MagicMock()
    fake_client.containers.run.return_value = fake_container

    with (
        patch.object(manager, "_get_client", return_value=fake_client),
        patch.object(manager, "_ping", return_value=True),
    ):
        manager.start()

    fake_client.containers.run.assert_called_once()
    kwargs = fake_client.containers.run.call_args.kwargs
    assert kwargs["image"] == "carlasim/carla:0.9.15"
    assert kwargs["detach"] is True
    assert kwargs["network_mode"] == "host"
    # GPU passthrough is requested by default.
    assert "device_requests" in kwargs
    assert manager._container is fake_container
    assert manager._reused is False


def test_start_reuses_named_container_when_present() -> None:
    """If container_name exists, start() reuses it instead of creating a new one."""
    manager = CarlaDockerServerManager(
        container_name="carla-server",
        reuse_if_running=False,
        timeout=1.0,
    )

    fake_container = MagicMock()
    fake_container.status = "exited"
    fake_client = MagicMock()
    fake_client.containers.get.return_value = fake_container

    with (
        patch.object(manager, "_get_client", return_value=fake_client),
        patch.object(manager, "_ping", return_value=True),
    ):
        manager.start()

    fake_client.containers.get.assert_called_once_with("carla-server")
    fake_container.start.assert_called_once()
    fake_client.containers.run.assert_not_called()
    assert manager._container is fake_container


def test_stop_removes_container_when_remove_on_stop_true() -> None:
    """stop() stops and removes a manager-owned container by default."""
    manager = CarlaDockerServerManager(reuse_if_running=False, remove_on_stop=True)
    fake_container = MagicMock()
    fake_container.status = "running"
    manager._container = fake_container
    manager._reused = False

    manager.stop()

    fake_container.stop.assert_called_once()
    fake_container.remove.assert_called_once_with(force=True)
    assert manager._container is None


def test_stop_keeps_container_when_remove_on_stop_false() -> None:
    """stop() leaves the container in place when remove_on_stop=False."""
    manager = CarlaDockerServerManager(reuse_if_running=False, remove_on_stop=False)
    fake_container = MagicMock()
    fake_container.status = "running"
    manager._container = fake_container
    manager._reused = False

    manager.stop()

    fake_container.stop.assert_called_once()
    fake_container.remove.assert_not_called()


def test_build_device_requests_all() -> None:
    """gpus='all' translates to count=-1 NVIDIA device request."""
    manager = CarlaDockerServerManager(gpus="all")
    requests = manager._build_device_requests()
    assert requests is not None
    assert requests[0]["driver"] == "nvidia"
    assert requests[0]["count"] == -1


def test_build_device_requests_explicit_ids() -> None:
    """Comma-separated gpus values are forwarded as device_ids."""
    manager = CarlaDockerServerManager(gpus="0,1")
    requests = manager._build_device_requests()
    assert requests is not None
    assert requests[0]["device_ids"] == ["0", "1"]


def test_build_device_requests_disabled() -> None:
    """Falsy gpus disables GPU passthrough entirely."""
    manager = CarlaDockerServerManager(gpus=None)
    assert manager._build_device_requests() is None
    manager_false = CarlaDockerServerManager(gpus=False)
    assert manager_false._build_device_requests() is None
