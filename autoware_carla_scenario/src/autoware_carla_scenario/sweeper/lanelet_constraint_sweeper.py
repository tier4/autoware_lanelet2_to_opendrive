"""Hydra Sweeper plugin that enumerates lanelets matching declarative constraints.

When activated via ``--multirun hydra/sweeper=lanelet_constraint``, this
sweeper:

1. Loads the Lanelet2 map (lightweight, no CARLA required).
2. Parses constraints from ``sweep.constraints`` and finds matching lanelets.
3. For each match, resolves bindings from ``sweep.bindings`` to compute
   derived parameters (e.g. ``ego.spawn_s``).
4. Constructs per-lanelet Hydra override batches and launches them.
"""

from __future__ import annotations

import json
import logging
import os
import signal
import time
from pathlib import Path
from typing import Any

from hydra.core.plugins import Plugins
from hydra.core.utils import JobReturn, JobStatus
from hydra.plugins.sweeper import Sweeper
from hydra.types import HydraContext
from omegaconf import DictConfig, OmegaConf
from tqdm import tqdm

from .bindings import Binding, parse_binding
from .constraints import (
    Constraint,
    create_routing_graph,
    find_matching_lanelets,
    parse_constraint,
)
from .map_loader import load_lanelet2_map

logger = logging.getLogger(__name__)

# Default hard timeout per job (seconds).  Can be overridden via
# ``sweep.job_timeout_seconds`` in the scenario YAML.
_DEFAULT_JOB_TIMEOUT = 120


def _check_result_json_in_dir(working_dir: str | None) -> bool | None:
    """Read the scenario result JSON from a job's output directory.

    Args:
        working_dir: The Hydra job output directory (from ``JobReturn.working_dir``).

    Returns:
        ``True`` if the scenario passed, ``False`` if it failed or the
        result file is missing (e.g. spawn failure prevented the scenario
        from running), or ``None`` if *working_dir* is not available.
    """
    if not working_dir:
        return None
    output_dir = Path(working_dir)
    if not output_dir.is_dir():
        return None
    result_files = sorted(output_dir.glob("*_result.json"))
    if not result_files:
        # Output directory exists but no result JSON — the scenario did
        # not complete (e.g. spawn failure raised before ScenarioRunner
        # could write the result).
        return False
    try:
        data = json.loads(result_files[0].read_text(encoding="utf-8"))
        return bool(data.get("passed", True))
    except (json.JSONDecodeError, OSError):
        return False


def _get_job_output_dir(ret: JobReturn) -> str | None:
    """Return the Hydra output directory for a completed job.

    ``JobReturn.working_dir`` is the *current working directory* at
    execution time, which equals the project root when
    ``hydra.job.chdir=False`` (the default since Hydra 1.2).
    The actual output directory where :class:`ScenarioRunner` writes
    result JSONs is ``hydra.runtime.output_dir`` inside the job's
    ``hydra_cfg``.
    """
    hydra_cfg = getattr(ret, "hydra_cfg", None)
    if hydra_cfg is not None:
        try:
            return str(OmegaConf.select(hydra_cfg, "hydra.runtime.output_dir"))
        except Exception:
            pass
    # Fallback: working_dir (correct only when hydra.job.chdir=True).
    return getattr(ret, "working_dir", None)


def _launch_job_isolated(
    launcher: Any,
    batch: tuple[str, ...],
    idx: int,
    timeout: int,
) -> tuple[bool, int | None]:
    """Fork a child process to run a single job, isolating SIGSEGV crashes.

    Args:
        launcher: Hydra job launcher instance.
        batch: Override tuple for this job.
        idx: 0-based job index.
        timeout: Hard timeout in seconds (0 = no timeout).

    Returns:
        ``(succeeded, exit_code)`` where *exit_code* is 0 on success,
        a positive int on normal failure, a negative int (``-signal_number``)
        on signal death, or ``None`` on timeout.
    """
    child_pid = os.fork()

    if child_pid == 0:
        # ---- child process ----
        # Create a new process group so the parent can kill the entire tree.
        os.setpgrp()
        # Reset any inherited SIGALRM handler so it doesn't fire in the child.
        signal.signal(signal.SIGALRM, signal.SIG_DFL)
        signal.alarm(0)
        try:
            job_returns = launcher.launch([batch], initial_job_idx=idx)
        except SystemExit as exc:
            os._exit(exc.code if isinstance(exc.code, int) else 1)
        except Exception:
            logger.exception("Job raised an exception in child process")
            os._exit(1)
        # launcher.launch() completes without error even when the scenario
        # *fails* (in --multirun mode _hydra_main does not sys.exit).
        # Hydra catches task-function exceptions internally and stores them
        # in JobReturn with status=FAILED, so we must inspect the status
        # and the result JSON to determine the actual outcome.
        if job_returns:
            ret = job_returns[0]
            # If the task function raised (e.g. spawn failure), Hydra
            # sets status to FAILED without re-raising the exception.
            if ret.status != JobStatus.COMPLETED:
                os._exit(1)
            # Use hydra_cfg.hydra.runtime.output_dir — NOT working_dir,
            # which equals cwd (unchanged) when hydra.job.chdir=False
            # (the default since Hydra 1.2).
            output_dir = _get_job_output_dir(ret)
            if _check_result_json_in_dir(output_dir) is False:
                os._exit(1)
        os._exit(0)

    # ---- parent process ----
    timed_out = False

    def _kill_child(signum: int, frame: Any) -> None:  # noqa: ARG001
        nonlocal timed_out
        timed_out = True
        try:
            os.killpg(child_pid, signal.SIGKILL)
        except OSError:
            pass

    prev_handler = signal.getsignal(signal.SIGALRM)
    try:
        if timeout > 0:
            signal.signal(signal.SIGALRM, _kill_child)
            signal.alarm(timeout)

        _, status = os.waitpid(child_pid, 0)
    finally:
        signal.alarm(0)
        signal.signal(signal.SIGALRM, prev_handler)

    if timed_out:
        return False, None

    if os.WIFSIGNALED(status):
        sig = os.WTERMSIG(status)
        return False, -sig

    exit_code = os.WEXITSTATUS(status) if os.WIFEXITED(status) else 1
    return exit_code == 0, exit_code


class LaneletConstraintSweeper(Sweeper):
    """Sweep over lanelets that satisfy declarative constraints."""

    def __init__(self) -> None:
        super().__init__()
        self.config: DictConfig | None = None
        self.hydra_context: HydraContext | None = None
        self.launcher: Any = None

    def setup(
        self,
        *,
        config: DictConfig,
        hydra_context: HydraContext,
        task_function: Any,
    ) -> None:
        """Store references and instantiate the job launcher."""
        self.config = config
        self.hydra_context = hydra_context
        self.launcher = Plugins.instance().instantiate_launcher(
            hydra_context=hydra_context,
            task_function=task_function,
            config=config,
        )

    def sweep(self, arguments: list[str]) -> Any:
        """Execute the constraint-based sweep.

        Args:
            arguments: Additional CLI overrides passed alongside ``--multirun``.

        Returns:
            The aggregated results from all launched jobs.
        """
        assert self.config is not None
        assert self.launcher is not None

        cfg = self.config

        # -- 1. Resolve map paths ------------------------------------------
        lanelet2_path = OmegaConf.select(cfg, "map.lanelet2_path")
        xodr_path = OmegaConf.select(cfg, "map.xodr_path")
        if lanelet2_path is None or xodr_path is None:
            raise ValueError(
                "LaneletConstraintSweeper requires both map.lanelet2_path and "
                "map.xodr_path to be set in the config."
            )

        # -- 2. Load the Lanelet2 map (lightweight) ------------------------
        lanelet_map = load_lanelet2_map(lanelet2_path, xodr_path)

        # -- 3. Parse constraints ------------------------------------------
        sweep_cfg = OmegaConf.select(cfg, "sweep")
        if sweep_cfg is None:
            raise ValueError(
                "No 'sweep' section found in config. "
                "LaneletConstraintSweeper requires sweep.constraints."
            )
        sweep_dict = OmegaConf.to_container(sweep_cfg, resolve=True)
        assert isinstance(sweep_dict, dict)

        constraints_cfg = sweep_dict.get("constraints", {})
        if not constraints_cfg:
            raise ValueError("sweep.constraints is empty; nothing to sweep.")

        # Constraints are keyed by the target parameter (e.g. ego.spawn_lanelet_id).
        # Each value is a list of constraint dicts.
        all_constraints: list[Constraint] = []
        constraint_target_key: str | None = None
        for target_key, constraint_list in constraints_cfg.items():
            constraint_target_key = target_key
            for c_cfg in constraint_list:
                all_constraints.append(parse_constraint(c_cfg))

        if not all_constraints or constraint_target_key is None:
            raise ValueError("No valid constraints found in sweep.constraints.")

        # -- 4. Build routing graph (once) and find matching lanelets --------
        routing_graph = create_routing_graph(lanelet_map)
        matched_ids = find_matching_lanelets(
            all_constraints, lanelet_map, routing_graph
        )
        if not matched_ids:
            logger.warning("No lanelets match the given constraints. Nothing to sweep.")
            return []

        # -- 5. Parse bindings ---------------------------------------------
        bindings_cfg = sweep_dict.get("bindings", {})
        bindings: list[Binding] = []
        for target_key, b_cfg in bindings_cfg.items():
            bindings.append(parse_binding(target_key, b_cfg))

        # -- 6. Build override batches -------------------------------------
        batches: list[tuple[str, ...]] = []
        for lid in matched_ids:
            overrides: list[str] = [f"{constraint_target_key}={lid}"]
            for binding in bindings:
                try:
                    result = binding.resolve(lid, lanelet_map, routing_graph)
                    overrides.append(f"{binding.target_key}={result.value}")
                    if result.lanelet_id_override is not None:
                        overrides[0] = (
                            f"{constraint_target_key}={result.lanelet_id_override}"
                        )
                except Exception:
                    logger.warning(
                        "Binding %s failed for lanelet %d; skipping this lanelet.",
                        binding.target_key,
                        lid,
                        exc_info=True,
                    )
                    break
            else:
                # Merge CLI arguments.
                overrides.extend(arguments)
                batches.append(tuple(overrides))

        if not batches:
            logger.warning("All lanelets were skipped due to binding failures.")
            return []

        logger.info(
            "Sweeping %d lanelet(s): %s",
            len(batches),
            [b[0] for b in batches],
        )

        # -- 7. Resolve per-job timeout, cooldown, retry count, and resume ---
        job_timeout: int = int(
            sweep_dict.get("job_timeout_seconds", _DEFAULT_JOB_TIMEOUT)
        )
        cooldown: float = float(
            OmegaConf.select(cfg, "server.cooldown_seconds", default=0.0)
        )
        max_retries: int = int(
            OmegaConf.select(cfg, "server.cooldown_max_retries", default=0)
        )
        max_attempts = 1 + max_retries
        resume_from: int = int(
            sweep_dict.get("resume_from", 0) or os.environ.get("SWEEP_RESUME_FROM", "0")
        )

        # -- 8. Apply resume: skip jobs before the resume point (1-indexed) --
        if resume_from > 1:
            skip_count = min(resume_from - 1, len(batches))
            logger.info(
                "Resuming from job %d — skipping %d/%d job(s).",
                resume_from,
                skip_count,
                len(batches),
            )
            batches = batches[skip_count:]
            # Offset indices so logs still show the original 1-based position.
            idx_offset = skip_count
        else:
            idx_offset = 0

        # -- 9. Launch batches one-by-one via fork isolation --
        all_returns: list[Any] = []
        failed_count = 0
        timed_out_count = 0
        first_job = True

        for local_idx, batch in enumerate(batches):
            idx = local_idx + idx_offset

            succeeded = False
            for attempt in range(max_attempts):
                # Cooldown in the parent process (no CARLA state, SIGSEGV-safe).
                # Skip cooldown before the very first job of this run.
                if (not first_job or attempt > 0) and cooldown > 0:
                    _wait_with_progress(
                        cooldown,
                        desc="CARLA cooldown"
                        if attempt == 0
                        else "CARLA cooldown (retry)",
                    )

                logger.info(
                    "[%d/%d] Launching (attempt %d/%d): %s",
                    idx + 1,
                    idx_offset + len(batches),
                    attempt + 1,
                    max_attempts,
                    batch,
                )

                ok, exit_code = _launch_job_isolated(
                    self.launcher, batch, idx, job_timeout
                )
                if ok:
                    succeeded = True
                    break

                # Log based on exit code.
                if exit_code is None:
                    logger.error(
                        "[%d/%d] Job TIMED OUT after %ds (attempt %d/%d) "
                        "for overrides %s",
                        idx + 1,
                        idx_offset + len(batches),
                        job_timeout,
                        attempt + 1,
                        max_attempts,
                        batch,
                    )
                elif exit_code < 0:
                    logger.error(
                        "[%d/%d] Job CRASHED (signal %d) (attempt %d/%d) "
                        "for overrides %s",
                        idx + 1,
                        idx_offset + len(batches),
                        -exit_code,
                        attempt + 1,
                        max_attempts,
                        batch,
                    )
                else:
                    logger.error(
                        "[%d/%d] Job FAILED (exit code %d) (attempt %d/%d) "
                        "for overrides %s",
                        idx + 1,
                        idx_offset + len(batches),
                        exit_code,
                        attempt + 1,
                        max_attempts,
                        batch,
                    )

            first_job = False
            # Result JSON is written to disk by the child; parent only
            # tracks success/failure.
            all_returns.append(None)
            if not succeeded:
                # Distinguish timeouts from other failures for the summary.
                if exit_code is None:
                    timed_out_count += 1
                failed_count += 1
                logger.error(
                    "[%d/%d] Job FAILED after %d attempt(s) for overrides %s "
                    "— continuing.",
                    idx + 1,
                    idx_offset + len(batches),
                    max_attempts,
                    batch,
                )

        executed = len(batches)
        logger.info(
            "Sweep complete: %d/%d succeeded, %d failed (%d timed out).",
            executed - failed_count,
            executed,
            failed_count,
            timed_out_count,
        )
        return all_returns


def _wait_with_progress(seconds: float, *, desc: str = "Waiting") -> None:
    """Sleep for *seconds* with a tqdm progress bar (0.1 s resolution)."""
    steps = max(1, int(seconds * 10))
    interval = seconds / steps
    for _ in tqdm(
        range(steps),
        desc=desc,
        bar_format="{desc}: {bar}| {elapsed}<{remaining}",
    ):
        time.sleep(interval)
