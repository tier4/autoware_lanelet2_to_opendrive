"""CARLA UE5 server lifecycle management."""

from __future__ import annotations

import atexit
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

    If *reuse_if_running* is ``True`` (the default) and a CARLA server is
    already reachable on *host*:*port* at the time :meth:`start` is called,
    no new process is launched – the existing server is reused.  This is the
    recommended mode for local development where CARLA may already be running.

    Example – managed lifecycle::

        with CarlaServerManager() as server:
            client = carla.Client(server.host, server.port)
            ...

    Example – reuse an already-running server::

        manager = CarlaServerManager(reuse_if_running=True)
        manager.start()   # no-op if CARLA is already up
        ...
        manager.stop()    # no-op if the server was not launched by us
    """

    ENV_VAR: str = "CARLA_UE5_EXECUTABLE"

    def __init__(
        self,
        host: str = "localhost",
        port: int = 2000,
        timeout: float = 120.0,
        extra_args: Optional[List[str]] = None,
        reuse_if_running: bool = True,
    ) -> None:
        """Initialize the server manager.

        Args:
            host: Hostname to connect / bind the CARLA server to.
            port: TCP port for the CARLA RPC server.
            timeout: Seconds to wait for the server to become reachable
                after launching it.  Defaults to 120 s (UE5 can be slow
                on first boot).
            extra_args: Additional CLI arguments passed to ``CarlaUE5.sh``.
            reuse_if_running: When ``True`` (default), :meth:`start` skips
                launching a new process if a CARLA server is already
                reachable on *host*:*port*.  :meth:`stop` is also a no-op
                in that case so the externally-managed server is left alive.
        """
        self.host = host
        self.port = port
        self.timeout = timeout
        self.extra_args: List[str] = extra_args or []
        self.reuse_if_running = reuse_if_running
        self._process: Optional[subprocess.Popen[bytes]] = None
        self._reused: bool = False  # True when we connected to an existing server

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Launch (or reuse) the CARLA server.

        Behaviour:
        1. If *reuse_if_running* is ``True`` and the server is already
           reachable, record that we are reusing it and return immediately.
        2. Otherwise, read ``CARLA_UE5_EXECUTABLE``, launch the process, and
           poll until the server accepts connections.

        Raises:
            RuntimeError: If ``CARLA_UE5_EXECUTABLE`` is not set (when a new
                process must be launched), the executable does not exist, or
                the server does not become reachable within *timeout* seconds.
        """
        if self.reuse_if_running and self._ping():
            self._reused = True
            return

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
        self._reused = False
        # Guarantee cleanup even if __exit__ / stop() is never called explicitly
        # (e.g. pytest interrupted by Ctrl-C or an unhandled exception).
        atexit.register(self.stop)
        self._wait_until_ready()

    def stop(self) -> None:
        """Terminate the CARLA server process group.

        If the server was *reused* (not launched by this manager), this method
        is a no-op so the externally-managed process is left running.

        CarlaUE5.sh launches the real UE5 binary as a child process.
        Sending SIGTERM only to the shell would leave the UE5 binary running
        as an orphan.  Instead, we send SIGTERM/SIGKILL to the *entire
        process group* created by ``start_new_session=True`` in :meth:`start`.

        After the direct child (CarlaUE5.sh) exits, the UE5 binary and its
        worker processes may still be shutting down.  We therefore poll the
        process group with signal 0 and send SIGKILL if it has not fully
        disappeared within the grace period.
        """
        if self._reused or self._process is None:
            return
        try:
            pgid = os.getpgid(self._process.pid)
            os.killpg(pgid, signal.SIGTERM)
            try:
                self._process.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                os.killpg(pgid, signal.SIGKILL)
                self._process.wait()

            # Wait for all processes in the group (UE5 binary, worker threads,
            # etc.) to finish.  CarlaUE5.sh may exit before the UE5 binary
            # completes its cleanup, so we poll the group explicitly.
            if not self._wait_for_pgid_exit(pgid, timeout=5.0):
                try:
                    os.killpg(pgid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        except ProcessLookupError:
            # Process already exited; nothing to do.
            pass
        self._process = None
        atexit.unregister(self.stop)

    def is_alive(self) -> bool:
        """Return True if the server is reachable (regardless of who started it)."""
        if not self._reused and (
            self._process is None or self._process.poll() is not None
        ):
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

    def _wait_for_pgid_exit(self, pgid: int, timeout: float) -> bool:
        """Poll until the process group is fully gone or *timeout* expires.

        Uses signal 0 (existence check, no real signal delivered) on the
        group.  Returns ``True`` if the group has disappeared, ``False`` if
        it still exists when the timeout elapses.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                os.killpg(pgid, 0)  # raises ProcessLookupError if gone
                time.sleep(0.1)
            except (ProcessLookupError, PermissionError):
                return True
        return False

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
