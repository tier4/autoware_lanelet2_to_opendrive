"""Scenario execution bridge using subprocess + progress tracking.

Runs ``uv run scenario`` in a subprocess to avoid importing CARLA
directly into the viewer process, providing SIGSEGV isolation.

When a Hydra sweeper is active the subprocess internally runs multiple
jobs.  This module parses the sweeper's log output in real-time to
extract per-job progress (``[1/3] Launching …``).
"""

from __future__ import annotations

import logging
import re
import subprocess
import threading
from pathlib import Path

from .models import RunProgress

logger = logging.getLogger(__name__)

# Regex for sweeper progress lines:
#   "[1/3] Launching (attempt 1/1): ..."
_RE_SWEEP_LAUNCH = re.compile(r"\[(\d+)/(\d+)\] Launching")
# Regex for sweeper "Sweeping N lanelet(s)" line:
_RE_SWEEP_TOTAL = re.compile(r"Sweeping (\d+) lanelet")
# Regex for sweep completion:
#   "Sweep complete: 3/3 succeeded, 0 failed (0 timed out)."
_RE_SWEEP_COMPLETE = re.compile(r"Sweep complete: (\d+)/(\d+) succeeded, (\d+) failed")
# Regex for job result in sweeper log:
#   "[1/3] Job FAILED ..." or successful launch
_RE_JOB_FAILED = re.compile(r"\[(\d+)/(\d+)\] Job (FAILED|CRASHED|TIMED OUT)")

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
            if sweeper:
                cmd.append("--multirun")
            cmd.extend(overrides)
            if sweeper:
                cmd.append(f"hydra/sweeper={sweeper}")
            cmd.extend(extra_overrides)
            logger.info("Running [%d/%d]: %s", i + 1, total, " ".join(cmd))

            if sweeper:
                status = _run_with_progress(cmd, cwd, timeout, scenario_name)
            else:
                status = _run_simple(cmd, cwd, timeout, scenario_name)

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


def _run_simple(
    cmd: list[str], cwd: str | None, timeout: int, scenario_name: str
) -> str:
    """Run a subprocess and return ``"passed"`` or ``"failed"``."""
    try:
        result = subprocess.run(  # noqa: S603
            cmd,
            cwd=cwd,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
        if result.returncode != 0:
            logger.warning(
                "Scenario %s failed (exit code %d): %s",
                scenario_name,
                result.returncode,
                result.stderr[-500:] if result.stderr else "",
            )
        return "passed" if result.returncode == 0 else "failed"
    except subprocess.TimeoutExpired:
        logger.error("Scenario %s timed out", scenario_name)
        return "failed"
    except OSError as exc:
        logger.error("Failed to run scenario %s: %s", scenario_name, exc)
        return "failed"


def _run_with_progress(
    cmd: list[str], cwd: str | None, timeout: int, scenario_name: str
) -> str:
    """Run a sweeper subprocess, parsing stderr for per-job progress.

    The sweeper logs lines like ``[1/3] Launching (attempt 1/1): ...``
    which are parsed in real-time to update the progress bar.
    """
    try:
        proc = subprocess.Popen(  # noqa: S603
            cmd,
            cwd=cwd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
    except OSError as exc:
        logger.error("Failed to start scenario %s: %s", scenario_name, exc)
        return "failed"

    sweep_total = 0
    sweep_current = 0
    stderr_lines: list[str] = []

    def _read_stderr() -> None:
        nonlocal sweep_total, sweep_current
        assert proc.stderr is not None  # noqa: S101
        for line in proc.stderr:
            stderr_lines.append(line)
            line_stripped = line.strip()

            # Parse "Sweeping N lanelet(s)"
            m = _RE_SWEEP_TOTAL.search(line_stripped)
            if m:
                sweep_total = int(m.group(1))
                _set_progress(
                    RunProgress(
                        current=0,
                        total=sweep_total,
                        scenario_name=f"{scenario_name} (sweep)",
                        status="running",
                    )
                )
                continue

            # Parse "[N/M] Launching ..."
            m = _RE_SWEEP_LAUNCH.search(line_stripped)
            if m:
                sweep_current = int(m.group(1))
                sweep_total = max(sweep_total, int(m.group(2)))
                _set_progress(
                    RunProgress(
                        current=sweep_current,
                        total=sweep_total,
                        scenario_name=f"{scenario_name} [{sweep_current}/{sweep_total}]",
                        status="running",
                    )
                )
                continue

            # Parse "[N/M] Job FAILED/CRASHED/TIMED OUT ..."
            m = _RE_JOB_FAILED.search(line_stripped)
            if m:
                _set_progress(
                    RunProgress(
                        current=int(m.group(1)),
                        total=int(m.group(2)),
                        scenario_name=f"{scenario_name} [{m.group(1)}/{m.group(2)}]",
                        status="failed",
                    )
                )
                continue

            # Parse "Sweep complete: ..."
            m = _RE_SWEEP_COMPLETE.search(line_stripped)
            if m:
                succeeded = int(m.group(1))
                total_jobs = int(m.group(2))
                failed = int(m.group(3))
                status = "passed" if failed == 0 else "failed"
                _set_progress(
                    RunProgress(
                        current=total_jobs,
                        total=total_jobs,
                        scenario_name=f"{scenario_name} ({succeeded}/{total_jobs} passed)",
                        status=status,
                    )
                )

    # Read stderr in a separate thread to avoid deadlock.
    reader = threading.Thread(target=_read_stderr, daemon=True)
    reader.start()

    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()
        logger.error("Scenario %s timed out", scenario_name)
        return "failed"

    reader.join(timeout=5)

    if proc.returncode != 0:
        tail = "".join(stderr_lines[-10:])
        logger.warning(
            "Scenario %s failed (exit code %d): %s",
            scenario_name,
            proc.returncode,
            tail[-500:] if tail else "",
        )
        return "failed"

    return "passed"


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
