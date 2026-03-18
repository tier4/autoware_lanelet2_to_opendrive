"""Scenario execution bridge using subprocess + progress tracking.

Runs ``uv run scenario`` in a subprocess to avoid importing CARLA
directly into the viewer process, providing SIGSEGV isolation.
"""

from __future__ import annotations

import logging
import subprocess
import threading
from pathlib import Path

from .models import RunProgress

logger = logging.getLogger(__name__)

# Global state for the current run.
_lock = threading.Lock()
_progress: RunProgress | None = None
_running = False


def get_progress() -> RunProgress | None:
    """Return the current run progress, or ``None`` if no run is active."""
    with _lock:
        return _progress


def _set_progress(p: RunProgress) -> None:
    with _lock:
        global _progress  # noqa: PLW0603
        _progress = p


def _set_running(state: bool) -> None:
    with _lock:
        global _running  # noqa: PLW0603
        _running = state


def is_running() -> bool:
    """Return ``True`` if a scenario run is currently in progress."""
    with _lock:
        return _running


def start_run(
    overrides_list: list[list[str]],
    base_path: Path | None = None,
    extra_overrides: list[str] | None = None,
    timeout: int = 300,
    sweeper: str = "",
) -> None:
    """Start scenario execution in a background thread.

    Args:
        overrides_list: List of override lists. Each inner list is passed
            as overrides to a single ``uv run scenario`` invocation.
        base_path: Working directory for the subprocess.
        extra_overrides: Additional Hydra overrides appended to every run
            (e.g. ``["server.host=192.168.1.100"]``).
        timeout: Per-scenario subprocess timeout in seconds.
        sweeper: Hydra sweeper name (e.g. ``"lanelet_constraint"``).
            When set, ``--multirun`` and ``hydra/sweeper=<name>`` are
            added to the command.
    """
    if is_running():
        logger.warning("A scenario run is already in progress")
        return

    thread = threading.Thread(
        target=_run_worker,
        args=(overrides_list, base_path, extra_overrides or [], timeout, sweeper),
        daemon=True,
    )
    thread.start()


def _run_worker(
    overrides_list: list[list[str]],
    base_path: Path | None,
    extra_overrides: list[str],
    timeout: int,
    sweeper: str,
) -> None:
    """Background worker that executes scenarios sequentially."""
    _set_running(True)
    total = len(overrides_list)
    cwd = str(base_path) if base_path else None

    try:
        for i, overrides in enumerate(overrides_list):
            scenario_name = _extract_scenario_name(overrides)
            _set_progress(
                RunProgress(
                    current=i + 1,
                    total=total,
                    scenario_name=scenario_name,
                    status="running",
                )
            )

            cmd = ["uv", "run", "scenario"]
            # --multirun + hydra/sweeper must come before overrides.
            # Only added when a sweeper is selected.
            if sweeper:
                cmd.append("--multirun")
            cmd.extend(overrides)
            if sweeper:
                cmd.append(f"hydra/sweeper={sweeper}")
            cmd.extend(extra_overrides)
            logger.info("Running [%d/%d]: %s", i + 1, total, " ".join(cmd))

            try:
                result = subprocess.run(  # noqa: S603
                    cmd,
                    cwd=cwd,
                    capture_output=True,
                    text=True,
                    timeout=timeout,
                )
                status = "passed" if result.returncode == 0 else "failed"
                if result.returncode != 0:
                    logger.warning(
                        "Scenario %s failed (exit code %d): %s",
                        scenario_name,
                        result.returncode,
                        result.stderr[-500:] if result.stderr else "",
                    )
            except subprocess.TimeoutExpired:
                status = "failed"
                logger.error("Scenario %s timed out", scenario_name)
            except OSError as exc:
                status = "failed"
                logger.error("Failed to run scenario %s: %s", scenario_name, exc)

            _set_progress(
                RunProgress(
                    current=i + 1,
                    total=total,
                    scenario_name=scenario_name,
                    status=status,
                )
            )

        # Signal completion
        _set_progress(
            RunProgress(
                current=total,
                total=total,
                scenario_name="",
                status="done",
            )
        )
    finally:
        _set_running(False)


def _extract_scenario_name(overrides: list[str]) -> str:
    """Extract a human-readable scenario name from override arguments."""
    for ov in overrides:
        if ov.startswith("scenario="):
            return ov[len("scenario=") :]
    return "scenario"


def build_command(
    scenario: str,
    extra_overrides: list[str] | None = None,
    sweeper: str = "",
) -> list[str]:
    """Build the ``uv run scenario`` command for preview purposes.

    Returns the command as a list of strings.
    """
    cmd = ["uv", "run", "scenario"]
    if sweeper:
        cmd.append("--multirun")
    cmd.append(f"scenario={scenario}")
    if sweeper:
        cmd.append(f"hydra/sweeper={sweeper}")
    if extra_overrides:
        cmd.extend(extra_overrides)
    return cmd
