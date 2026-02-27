"""CARLA UE5 server lifecycle management."""

from __future__ import annotations

import os
import signal
import subprocess
import time
from pathlib import Path
from typing import List, Optional


class CarlaServerManager:
    """Start and stop a CARLA UE5 server process.

    The path to the ``CarlaUE5.sh`` executable is read from the
    ``CARLA_UE5_EXECUTABLE`` environment variable.

    Example::

        with CarlaServerManager() as server:
            client = carla.Client(server.host, server.port)
            ...
    """

    ENV_VAR: str = "CARLA_UE5_EXECUTABLE"

    def __init__(
        self,
        host: str = "localhost",
        port: int = 2000,
        timeout: float = 60.0,
        extra_args: Optional[List[str]] = None,
    ) -> None:
        """Initialize the server manager.

        Args:
            host: Hostname to bind the CARLA server to.
            port: TCP port for the CARLA RPC server.
            timeout: Seconds to wait for the server to become reachable.
            extra_args: Additional command-line arguments passed to CarlaUE5.sh.
        """
        self.host = host
        self.port = port
        self.timeout = timeout
        self.extra_args: List[str] = extra_args or []
        self._process: Optional[subprocess.Popen[bytes]] = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Launch the CARLA server and wait until it is reachable.

        Reads the executable path from the ``CARLA_UE5_EXECUTABLE`` env var.

        Raises:
            RuntimeError: If ``CARLA_UE5_EXECUTABLE`` is not set, the
                executable does not exist, or the server does not become
                reachable within *timeout* seconds.
        """
        executable = os.environ.get(self.ENV_VAR)
        if not executable:
            raise RuntimeError(
                f"Environment variable '{self.ENV_VAR}' is not set. "
                "Set it to the path of CarlaUE5.sh before starting the server."
            )

        exe_path = Path(executable)
        if not exe_path.exists():
            raise RuntimeError(f"CARLA executable not found: {exe_path}")

        cmd = [str(exe_path)] + self.extra_args
        # start_new_session=True puts CarlaUE5.sh and all its children
        # (the actual UE5 binary) into a new process group so that
        # stop() can kill the entire group with os.killpg().
        self._process = subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )

        self._wait_until_ready()

    def stop(self) -> None:
        """Terminate the CARLA server process group.

        CarlaUE5.sh launches the real UE5 binary as a child process.
        Sending SIGTERM only to the shell would leave the UE5 binary running
        as an orphan.  Instead, we send SIGTERM/SIGKILL to the *entire
        process group* created by ``start_new_session=True`` in :meth:`start`,
        which guarantees that the UE5 binary and any other children are also
        terminated.
        """
        if self._process is None:
            return
        try:
            pgid = os.getpgid(self._process.pid)
            os.killpg(pgid, signal.SIGTERM)
            try:
                self._process.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                os.killpg(pgid, signal.SIGKILL)
                self._process.wait()
        except ProcessLookupError:
            # Process already exited; nothing to do.
            pass
        self._process = None

    def is_alive(self) -> bool:
        """Return True if the server process is running and reachable."""
        if self._process is None or self._process.poll() is not None:
            return False
        return self._ping()

    # ------------------------------------------------------------------
    # Context manager support
    # ------------------------------------------------------------------

    def __enter__(self) -> "CarlaServerManager":
        self.start()
        return self

    def __exit__(self, *args: object) -> None:
        self.stop()

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _ping(self) -> bool:
        """Try to connect to the CARLA RPC port; return True on success."""
        try:
            import carla

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
            f"CARLA server did not become reachable within {self.timeout}s "
            f"at {self.host}:{self.port}"
        )
