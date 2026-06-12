"""CARLA server lifecycle management backed by a Docker container.

This module mirrors the public API of :mod:`autoware_carla_scenario.server`
(:class:`CarlaServerManager`) but launches CARLA inside a Docker container via
the official Docker Python SDK instead of running ``CarlaUE5.sh`` directly on
the host.

Typical usage::

    from autoware_carla_scenario import CarlaDockerServerManager

    with CarlaDockerServerManager() as server:
        client = carla.Client(server.host, server.port)
        ...
"""

from __future__ import annotations

import atexit
import os
import time
from typing import Any, Dict, List, Mapping, Optional, Union

import carla
from dotenv import load_dotenv

try:
    import docker
    from docker.errors import APIError, DockerException, NotFound
    from docker.models.containers import Container
except ImportError as exc:  # pragma: no cover - import-time error path
    raise ImportError(
        "The 'docker' Python package is required for CarlaDockerServerManager. "
        "Install it with: uv add docker"
    ) from exc

load_dotenv(override=False)


class CarlaDockerServerManager:
    """Start and stop a CARLA server running inside a Docker container.

    The container image defaults to :attr:`DEFAULT_IMAGE`
    (``carlasim/carla:<version>``).  Three override mechanisms are available,
    listed in increasing priority:

    1. Override :attr:`DEFAULT_IMAGE` (or :attr:`DEFAULT_IMAGE_TAG`) by
       subclassing or monkey-patching::

           CarlaDockerServerManager.DEFAULT_IMAGE_TAG = "0.9.16"

    2. Set the ``CARLA_DOCKER_IMAGE`` environment variable (e.g. in a
       ``.env`` file)::

           CARLA_DOCKER_IMAGE=carlasim/carla:0.9.16

    3. Pass ``image=`` directly to the constructor::

           CarlaDockerServerManager(image="carlasim/carla:0.9.16")

    If *reuse_if_running* is ``True`` (the default) and a CARLA server is
    already reachable on *host*:*port* at the time :meth:`start` is called,
    no new container is launched – the existing server is reused.  This is
    the recommended mode for local development where CARLA may already be
    running.

    Example – managed lifecycle::

        with CarlaDockerServerManager() as server:
            client = carla.Client(server.host, server.port)
            ...

    Example – reuse an already-running server::

        manager = CarlaDockerServerManager(reuse_if_running=True)
        manager.start()   # no-op if CARLA is already up
        ...
        manager.stop()    # no-op if the server was not launched by us
    """

    ENV_VAR_IMAGE: str = "CARLA_DOCKER_IMAGE"
    # Default image repository and version tag.  Override at the class level
    # (or via the env var / constructor argument) to use a different CARLA
    # release.  The image follows the official ``carlasim/carla:<version>``
    # naming on Docker Hub: https://hub.docker.com/r/carlasim/carla/tags
    DEFAULT_IMAGE_REPOSITORY: str = "carlasim/carla"
    DEFAULT_IMAGE_TAG: str = "0.9.15"
    DEFAULT_IMAGE: str = f"{DEFAULT_IMAGE_REPOSITORY}:{DEFAULT_IMAGE_TAG}"

    def __init__(
        self,
        image: Optional[str] = None,
        host: str = "localhost",
        port: int = 2000,
        timeout: float = 120.0,
        container_name: Optional[str] = None,
        command: Optional[Union[str, List[str]]] = None,
        gpus: Optional[Union[str, bool]] = "all",
        network_mode: str = "host",
        ports: Optional[Mapping[str, Any]] = None,
        environment: Optional[Mapping[str, str]] = None,
        volumes: Optional[Mapping[str, Mapping[str, str]]] = None,
        extra_run_kwargs: Optional[Mapping[str, Any]] = None,
        reuse_if_running: bool = True,
        remove_on_stop: bool = True,
        pull_image: bool = False,
    ) -> None:
        """Initialize the Docker-backed server manager.

        Args:
            image: Container image to run, in ``repository:tag`` form
                (e.g. ``"carlasim/carla:0.9.16"``).  When ``None``, falls
                back to the ``CARLA_DOCKER_IMAGE`` env var, then to
                ``{DEFAULT_IMAGE_REPOSITORY}:{DEFAULT_IMAGE_TAG}``
                (default: ``carlasim/carla:0.9.15``).
            host: Hostname used to connect to the CARLA RPC server.  Use
                ``"localhost"`` together with ``network_mode="host"`` or
                with explicit ``ports`` mapping.
            port: TCP port of the CARLA RPC server (default 2000).
            timeout: Seconds to wait for the server to become reachable
                after launching the container.
            container_name: Optional Docker container name.  When provided,
                an existing container with the same name is reused instead
                of creating a new one.
            command: Command to run inside the container.  Typically the
                CARLA launch script and its arguments
                (e.g. ``["/bin/bash", "./CarlaUE4.sh", "-RenderOffScreen"]``).
            gpus: GPU passthrough.  ``"all"`` (default) requests every GPU
                via the NVIDIA device-request mechanism, ``None``/``False``
                disables GPU access, and any other string is forwarded as
                the device ID list.
            network_mode: Docker network mode.  Defaults to ``"host"`` so
                CARLA's RPC port is reachable on ``localhost`` without
                explicit port publishing.
            ports: Optional port mapping passed directly to
                ``client.containers.run`` (used when ``network_mode`` is not
                ``"host"``).
            environment: Optional environment variables for the container.
            volumes: Optional bind-mount specification.
            extra_run_kwargs: Escape hatch for additional keyword arguments
                forwarded verbatim to ``client.containers.run``.
            reuse_if_running: When ``True`` (default), :meth:`start` skips
                launching a new container if a CARLA server is already
                reachable on *host*:*port*.  :meth:`stop` is then a no-op
                so the externally-managed server is left alive.
            remove_on_stop: When ``True`` (default), the container is
                removed after it is stopped.  Set to ``False`` to keep the
                container around for inspection.
            pull_image: When ``True``, ``docker pull`` is invoked before
                running the container.  Defaults to ``False`` because pulls
                can be very slow for CARLA images; rely on a locally cached
                image during development.
        """
        # Resolve image with this priority: explicit arg > env var > class default.
        # The class default is rebuilt from DEFAULT_IMAGE_REPOSITORY and
        # DEFAULT_IMAGE_TAG so that subclasses / monkey-patches of either
        # attribute take effect even though DEFAULT_IMAGE is set at class
        # definition time.
        default_image = f"{self.DEFAULT_IMAGE_REPOSITORY}:{self.DEFAULT_IMAGE_TAG}"
        self.image = image or os.environ.get(self.ENV_VAR_IMAGE) or default_image
        self.host = host
        self.port = port
        self.timeout = timeout
        self.container_name = container_name
        self.command = command
        self.gpus = gpus
        self.network_mode = network_mode
        self.ports = dict(ports) if ports else None
        self.environment = dict(environment) if environment else None
        self.volumes = dict(volumes) if volumes else None
        self.extra_run_kwargs: Dict[str, Any] = (
            dict(extra_run_kwargs) if extra_run_kwargs else {}
        )
        self.reuse_if_running = reuse_if_running
        self.remove_on_stop = remove_on_stop
        self.pull_image = pull_image

        self._client: Optional[docker.DockerClient] = None
        self._container: Optional[Container] = None
        self._reused: bool = False  # True when we connected to an existing server

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Launch (or reuse) the CARLA container.

        Behaviour:
        1. If *reuse_if_running* is ``True`` and the server is already
           reachable, record that we are reusing it and return immediately.
        2. If *container_name* is provided and a container with that name
           already exists and is running, reuse it (and verify reachability).
        3. Otherwise, create a new container with the configured image,
           command, and Docker options, then poll until the server accepts
           connections.

        Raises:
            RuntimeError: If the Docker daemon is not reachable, the image
                cannot be found, or the server does not become reachable
                within *timeout* seconds.
        """
        if self.reuse_if_running and self._ping():
            self._reused = True
            return

        client = self._get_client()

        if self.container_name:
            existing = self._lookup_container(client, self.container_name)
            if existing is not None:
                if existing.status != "running":
                    existing.start()
                self._container = existing
                self._reused = False
                atexit.register(self.stop)
                self._wait_until_ready()
                return

        if self.pull_image:
            try:
                client.images.pull(self.image)
            except (APIError, DockerException) as exc:
                raise RuntimeError(
                    f"Failed to pull CARLA image {self.image!r}: {exc}"
                ) from exc

        run_kwargs: Dict[str, Any] = {
            "image": self.image,
            "detach": True,
            "network_mode": self.network_mode,
            "auto_remove": False,
        }
        if self.container_name:
            run_kwargs["name"] = self.container_name
        if self.command is not None:
            run_kwargs["command"] = self.command
        if self.environment:
            run_kwargs["environment"] = self.environment
        if self.volumes:
            run_kwargs["volumes"] = self.volumes
        if self.ports and self.network_mode != "host":
            run_kwargs["ports"] = self.ports

        device_requests = self._build_device_requests()
        if device_requests is not None:
            run_kwargs["device_requests"] = device_requests

        run_kwargs.update(self.extra_run_kwargs)

        try:
            self._container = client.containers.run(**run_kwargs)
        except (APIError, DockerException) as exc:
            raise RuntimeError(
                f"Failed to start CARLA container from image {self.image!r}: {exc}"
            ) from exc

        self._reused = False
        # Guarantee cleanup even if __exit__ / stop() is never called explicitly
        # (e.g. pytest interrupted by Ctrl-C or an unhandled exception).
        atexit.register(self.stop)
        self._wait_until_ready()

    def stop(self) -> None:
        """Stop (and optionally remove) the CARLA container.

        If the server was *reused* (not launched by this manager), this method
        is a no-op so the externally-managed container is left running.
        """
        if self._reused or self._container is None:
            return
        try:
            try:
                self._container.reload()
            except (APIError, NotFound):
                # Container already gone.
                self._container = None
                atexit.unregister(self.stop)
                return

            if self._container.status == "running":
                try:
                    self._container.stop(timeout=10)
                except (APIError, DockerException):
                    try:
                        self._container.kill()
                    except (APIError, DockerException):
                        pass

            if self.remove_on_stop:
                try:
                    self._container.remove(force=True)
                except (APIError, NotFound):
                    pass
        finally:
            self._container = None
            atexit.unregister(self.stop)

    def is_alive(self) -> bool:
        """Return True if the server is reachable (regardless of who started it)."""
        if not self._reused and self._container is None:
            return False
        if self._container is not None:
            try:
                self._container.reload()
            except (APIError, NotFound):
                return False
            if self._container.status != "running":
                return False
        return self._ping()

    # ------------------------------------------------------------------
    # Context manager support
    # ------------------------------------------------------------------

    def __enter__(self) -> "CarlaDockerServerManager":
        self.start()
        return self

    def __exit__(self, *args: object) -> None:
        self.stop()

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _get_client(self) -> docker.DockerClient:
        if self._client is None:
            try:
                self._client = docker.from_env()
            except DockerException as exc:
                raise RuntimeError(
                    "Failed to connect to the Docker daemon. "
                    "Is Docker running and is the current user allowed to access it?"
                ) from exc
        return self._client

    @staticmethod
    def _lookup_container(
        client: docker.DockerClient, name: str
    ) -> Optional[Container]:
        try:
            return client.containers.get(name)
        except NotFound:
            return None
        except (APIError, DockerException):
            return None

    def _build_device_requests(self) -> Optional[List[Dict[str, Any]]]:
        """Translate the *gpus* option to Docker SDK ``device_requests``.

        Returns ``None`` when GPU access is disabled, otherwise a list with
        a single NVIDIA device request – the SDK equivalent of
        ``docker run --gpus all`` / ``--gpus device=0,1``.
        """
        if not self.gpus:
            return None
        request: Dict[str, Any] = {
            "driver": "nvidia",
            "capabilities": [["gpu"]],
        }
        if self.gpus == "all" or self.gpus is True:
            request["count"] = -1
        else:
            request["device_ids"] = [
                d.strip() for d in str(self.gpus).split(",") if d.strip()
            ]
        return [request]

    def _ping(self) -> bool:
        """Try to connect to the CARLA RPC port; return True on success."""
        try:
            client = carla.Client(self.host, self.port)
            client.set_timeout(2.0)
            client.get_server_version()
            return True
        except Exception:
            return False

    def _wait_until_ready(self) -> None:
        """Poll until the server accepts connections or the timeout expires."""
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            if self._ping():
                return
            time.sleep(1.0)
        raise RuntimeError(
            f"CARLA server in container did not become reachable within "
            f"{self.timeout}s at {self.host}:{self.port}"
        )
