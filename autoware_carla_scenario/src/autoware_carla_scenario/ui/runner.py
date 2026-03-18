"""Scenario execution bridge using subprocess + progress tracking.

Runs ``uv run scenario`` in a subprocess to avoid importing CARLA
directly into the viewer process, providing SIGSEGV isolation.

Sweep parameter expansion is handled by :mod:`.sweep_resolver` before
this module is called, so each entry in *overrides_list* is a concrete
single-scenario run.
"""

from __future__ import annotations

import logging
import subprocess
import threading
from datetime import datetime
from pathlib import Path
from .models import RunProgress, Status

logger = logging.getLogger(__name__)

RAW_OUTPUT_LOG = "raw_output.log"
"""Filename used to persist subprocess stdout/stderr alongside result JSON."""

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
    group_as_multirun: bool = False,
) -> None:
    """Start scenario execution in a background thread.

    Args:
        overrides_list: List of override lists. Each inner list is passed
            as overrides to a single ``uv run scenario`` invocation.
        base_path: Working directory for the subprocess.
        extra_overrides: Additional Hydra overrides appended to every run.
        timeout: Per-scenario subprocess timeout in seconds.
        group_as_multirun: When ``True``, all jobs share a single
            ``multirun/{date}/{time}/`` directory with numbered
            subdirectories (0, 1, 2, …) instead of each writing to
            its own ``outputs/`` directory.
    """
    if is_running():
        logger.warning("A scenario run is already in progress")
        return

    thread = threading.Thread(
        target=_run_worker,
        args=(
            overrides_list,
            base_path,
            extra_overrides or [],
            timeout,
            group_as_multirun,
        ),
        daemon=True,
    )
    thread.start()


def _run_worker(
    overrides_list: list[list[str]],
    base_path: Path | None,
    extra_overrides: list[str],
    timeout: int,
    group_as_multirun: bool,
) -> None:
    """Background worker that executes scenarios sequentially."""
    _set_running(True)
    total = len(overrides_list)
    cwd = str(base_path) if base_path else None

    # When grouping as multirun, create a shared timestamped directory.
    multirun_dir: str | None = None
    if group_as_multirun and total > 0:
        now = datetime.now()  # noqa: DTZ005
        multirun_dir = f"multirun/{now:%Y-%m-%d}/{now:%H-%M-%S}"
        logger.info("Grouping %d jobs under %s", total, multirun_dir)

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

            cmd = ["uv", "run", "scenario", *overrides, *extra_overrides]
            # Direct each job's output to multirun/{date}/{time}/{index}/
            if multirun_dir:
                cmd.append(f"hydra.run.dir={multirun_dir}/{i}")
            logger.info("Running [%d/%d]: %s", i + 1, total, " ".join(cmd))

            try:
                result = subprocess.run(  # noqa: S603
                    cmd,
                    cwd=cwd,
                    capture_output=True,
                    text=True,
                    timeout=timeout,
                )
                status: Status = "passed" if result.returncode == 0 else "failed"
                if result.returncode != 0:
                    logger.warning(
                        "Scenario %s failed (exit code %d): %s",
                        scenario_name,
                        result.returncode,
                        result.stderr[-500:] if result.stderr else "",
                    )
                # Save raw terminal output to the job directory.
                if multirun_dir:
                    base = Path(cwd) if cwd else Path.cwd()
                    _save_raw_output(result, base / multirun_dir / str(i))
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


def _save_raw_output(
    result: subprocess.CompletedProcess[str],
    job_dir: Path,
) -> None:
    """Save subprocess stdout/stderr to ``raw_output.log`` in *job_dir*."""
    parts = [s for s in (result.stdout, result.stderr) if s]
    if not parts:
        return
    try:
        job_dir.mkdir(parents=True, exist_ok=True)
        (job_dir / RAW_OUTPUT_LOG).write_text("\n\n".join(parts), encoding="utf-8")
    except OSError:
        logger.warning("Failed to save raw output to %s", job_dir)


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
) -> tuple[list[str], str]:
    """Build the ``uv run scenario`` command for preview purposes.

    Returns:
        A 2-tuple of ``(command_tokens, note)``.  *note* is a
        human-readable string (e.g. sweep info) or empty.
    """
    cmd = ["uv", "run", "scenario"]
    cmd.append(f"scenario={scenario}")
    if extra_overrides:
        cmd.extend(extra_overrides)
    note = f"sweep: {sweeper} (resolved to N individual jobs)" if sweeper else ""
    return cmd, note
