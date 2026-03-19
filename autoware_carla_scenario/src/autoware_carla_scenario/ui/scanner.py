"""Scan multirun/ and outputs/ directories for scenario test results."""

from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import Any

import yaml  # type: ignore[import-untyped]

from .models import ConditionNode, ScenarioResultView, SessionItem, SessionSummary
from .runner import RAW_OUTPUT_LOG

logger = logging.getLogger(__name__)

# Module-level cache: cleared by the Update button.
_cache: dict[str, Any] = {}


def list_scenario_configs() -> list[str]:
    """Return sorted list of available scenario config names.

    Scans ``conf/scenario/`` under the examples package for YAML files
    and returns relative paths without the ``.yaml`` suffix
    (e.g. ``"intersection_passing/left_turn"``).
    """
    try:
        from autoware_carla_scenario.examples.run import (  # noqa: PLC0415
            _CONF_DIR,
        )

        scenario_dir = _CONF_DIR / "scenario"
    except ImportError:
        return []
    if not scenario_dir.is_dir():
        return []
    names: list[str] = []
    for p in sorted(scenario_dir.rglob("*.yaml")):
        rel = p.relative_to(scenario_dir).with_suffix("")
        names.append(str(rel))
    return names


def clear_cache() -> None:
    """Clear all cached scan results."""
    _cache.clear()


def _read_result_json(path: Path) -> dict[str, Any] | None:
    """Read and parse a ``*_result.json`` file, returning ``None`` on error."""
    try:
        return json.loads(path.read_text(encoding="utf-8"))  # type: ignore[no-any-return]
    except (json.JSONDecodeError, OSError):
        logger.warning("Failed to read result JSON: %s", path)
        return None


def _read_overrides(job_dir: Path) -> list[str]:
    """Read ``.hydra/overrides.yaml`` from a job directory."""
    try:
        raw = (job_dir / ".hydra" / "overrides.yaml").read_text(encoding="utf-8")
        data = yaml.safe_load(raw)
        return data if isinstance(data, list) else []
    except (yaml.YAMLError, OSError):
        return []


def _extract_scenario_name(result: dict[str, Any] | None, job_dir: Path) -> str:
    """Extract scenario name from result JSON filename or batch data."""
    if result and "scenario" in result:
        return result["scenario"]
    # Derive from *_result.json filename
    for f in sorted(job_dir.glob("*_result.json")):
        name = f.stem
        if name.endswith("_result"):
            return name[: -len("_result")]
    return "unknown"


def _scan_multirun_session(
    session_dir: Path, date: str, time: str
) -> SessionSummary | None:
    """Scan a single multirun session directory."""
    # Multirun directories contain numbered subdirectories (0, 1, 2, ...)
    job_dirs = sorted(
        (d for d in session_dir.iterdir() if d.is_dir() and d.name.isdigit()),
        key=lambda d: int(d.name),
    )
    if not job_dirs:
        return None

    passed_count = 0
    total_count = len(job_dirs)
    scenario_names: set[str] = set()

    for job_dir in job_dirs:
        result_files = sorted(job_dir.glob("*_result.json"))
        if result_files:
            data = _read_result_json(result_files[0])
            if data and data.get("passed"):
                passed_count += 1
            scenario_names.add(_extract_scenario_name(data, job_dir))
        # Missing result counts as failed (no increment)

    scenario_label = ", ".join(sorted(scenario_names)) if scenario_names else "unknown"

    return SessionSummary(
        date=date,
        time=time,
        session_type="multirun",
        scenario_name=scenario_label,
        passed_count=passed_count,
        total_count=total_count,
    )


def _scan_outputs_session(
    session_dir: Path, date: str, time: str
) -> SessionSummary | None:
    """Scan a single outputs session directory (batch or single)."""
    batch_json = session_dir / "batch_results.json"
    if batch_json.is_file():
        # Batch session
        try:
            data = json.loads(batch_json.read_text(encoding="utf-8"))
        except (json.JSONDecodeError, OSError):
            return None
        if not isinstance(data, list) or not data:
            return None
        passed_count = sum(1 for item in data if item.get("passed"))
        scenario_names = {item.get("scenario", "unknown") for item in data}
        return SessionSummary(
            date=date,
            time=time,
            session_type="batch",
            scenario_name=", ".join(sorted(scenario_names)),
            passed_count=passed_count,
            total_count=len(data),
        )

    # Single run: look for *_result.json
    result_files = sorted(session_dir.glob("*_result.json"))
    if not result_files:
        return None  # Skip directories with no result

    data = _read_result_json(result_files[0])
    passed = bool(data.get("passed")) if data else False
    scenario_name = _extract_scenario_name(data, session_dir)
    return SessionSummary(
        date=date,
        time=time,
        session_type="single",
        scenario_name=scenario_name,
        passed_count=1 if passed else 0,
        total_count=1,
    )


def scan_sessions(base_path: Path) -> list[SessionSummary]:
    """Scan ``multirun/`` and ``outputs/`` under *base_path* for sessions.

    Returns a list of :class:`SessionSummary` sorted by date/time descending.
    """
    cache_key = f"sessions:{base_path}"
    if cache_key in _cache:
        return _cache[cache_key]  # type: ignore[return-value]

    sessions: list[SessionSummary] = []

    # Scan multirun/
    multirun_root = base_path / "multirun"
    if multirun_root.is_dir():
        for date_dir in sorted(multirun_root.iterdir(), reverse=True):
            if not date_dir.is_dir():
                continue
            for time_dir in sorted(date_dir.iterdir(), reverse=True):
                if not time_dir.is_dir():
                    continue
                summary = _scan_multirun_session(time_dir, date_dir.name, time_dir.name)
                if summary:
                    sessions.append(summary)

    # Scan outputs/
    outputs_root = base_path / "outputs"
    if outputs_root.is_dir():
        for date_dir in sorted(outputs_root.iterdir(), reverse=True):
            if not date_dir.is_dir():
                continue
            for time_dir in sorted(date_dir.iterdir(), reverse=True):
                if not time_dir.is_dir():
                    continue
                summary = _scan_outputs_session(time_dir, date_dir.name, time_dir.name)
                if summary:
                    sessions.append(summary)

    # Sort by date+time descending
    sessions.sort(key=lambda s: (s.date, s.time), reverse=True)

    _cache[cache_key] = sessions
    return sessions


def load_session(
    base_path: Path, session_type: str, date: str, time: str
) -> list[SessionItem]:
    """Load all scenario items within a session.

    Returns a list of :class:`SessionItem` ordered by index.
    """
    cache_key = f"session:{session_type}:{date}:{time}"
    if cache_key in _cache:
        return _cache[cache_key]  # type: ignore[return-value]

    items: list[SessionItem] = []

    if session_type == "multirun":
        session_dir = base_path / "multirun" / date / time
        if not session_dir.is_dir():
            return items
        job_dirs = sorted(
            (d for d in session_dir.iterdir() if d.is_dir() and d.name.isdigit()),
            key=lambda d: int(d.name),
        )
        for job_dir in job_dirs:
            index = int(job_dir.name)
            overrides = _read_overrides(job_dir)
            result_files = sorted(job_dir.glob("*_result.json"))
            if result_files:
                data = _read_result_json(result_files[0])
                if data:
                    items.append(
                        SessionItem(
                            index=index,
                            scenario_name=_extract_scenario_name(data, job_dir),
                            passed=data.get("passed"),
                            elapsed_seconds=data.get("elapsed_seconds"),
                            overrides=overrides,
                            message=data.get("message", ""),
                        )
                    )
                else:
                    items.append(
                        SessionItem(
                            index=index,
                            scenario_name="unknown",
                            passed=None,
                            elapsed_seconds=None,
                            overrides=overrides,
                            message="Failed to read result JSON",
                        )
                    )
            else:
                items.append(
                    SessionItem(
                        index=index,
                        scenario_name=_extract_scenario_name(None, job_dir),
                        passed=None,
                        elapsed_seconds=None,
                        overrides=overrides,
                        message="No result file found",
                    )
                )

    elif session_type == "batch":
        session_dir = base_path / "outputs" / date / time
        batch_json = session_dir / "batch_results.json"
        if not batch_json.is_file():
            return items
        try:
            data_list = json.loads(batch_json.read_text(encoding="utf-8"))
        except (json.JSONDecodeError, OSError):
            return items
        if not isinstance(data_list, list):
            return items
        overrides = _read_overrides(session_dir)
        for i, entry in enumerate(data_list):
            items.append(
                SessionItem(
                    index=i,
                    scenario_name=entry.get("scenario", "unknown"),
                    passed=entry.get("passed"),
                    elapsed_seconds=entry.get("elapsed_seconds"),
                    overrides=overrides,
                    message=entry.get("message", ""),
                )
            )

    elif session_type == "single":
        session_dir = base_path / "outputs" / date / time
        if not session_dir.is_dir():
            return items
        overrides = _read_overrides(session_dir)
        result_files = sorted(session_dir.glob("*_result.json"))
        if result_files:
            data = _read_result_json(result_files[0])
            if data:
                items.append(
                    SessionItem(
                        index=0,
                        scenario_name=_extract_scenario_name(data, session_dir),
                        passed=data.get("passed"),
                        elapsed_seconds=data.get("elapsed_seconds"),
                        overrides=overrides,
                        message=data.get("message", ""),
                    )
                )

    _cache[cache_key] = items
    return items


_RAW_LOG_MAX_BYTES = 256 * 1024  # 256 KB cap per log


def _read_raw_log(job_dir: Path) -> str:
    """Read raw terminal log from a job directory.

    Tries ``raw_output.log`` first (subprocess stdout/stderr captured by
    the runner), then falls back to ``run.log`` (Python logging output).
    Returns an empty string when neither file exists.  Large files are
    truncated to the last ``_RAW_LOG_MAX_BYTES`` bytes.
    """
    for name in (RAW_OUTPUT_LOG, "run.log"):
        try:
            log_path = job_dir / name
            size = log_path.stat().st_size
            if size <= _RAW_LOG_MAX_BYTES:
                return log_path.read_text(encoding="utf-8", errors="replace")
            with log_path.open(encoding="utf-8", errors="replace") as f:
                f.seek(size - _RAW_LOG_MAX_BYTES)
                f.readline()  # discard partial first line
                tail = f.read()
            return f"... (truncated, showing last {len(tail)} chars) ...\n{tail}"
        except OSError:
            continue
    return ""


def _build_condition_tree(details: dict[str, Any]) -> list[ConditionNode]:
    """Recursively extract child condition nodes from a details dict."""
    children: list[ConditionNode] = []

    # Composite conditions (AndCondition, OrCondition) have "children"
    if "children" in details and isinstance(details["children"], list):
        for child_detail in details["children"]:
            node = _detail_to_condition_node(child_detail)
            if node:
                children.append(node)

    # Wrapper conditions (StickyCondition, PersistentCondition, NotCondition) have "child"
    if "child" in details and isinstance(details["child"], dict):
        node = _detail_to_condition_node(details["child"])
        if node:
            children.append(node)

    return children


_STRUCTURAL_KEYS = {
    "label",
    "condition_type",
    "satisfied",
    "message",
    "children",
    "child",
    "operator",
    "wrapper",
}


def _detail_to_condition_node(detail: dict[str, Any]) -> ConditionNode | None:
    """Convert a detail dict (from nested condition tree) to a ConditionNode."""
    if not isinstance(detail, dict):
        return None

    label = detail.get("label", "")
    condition_type = detail.get("condition_type", "")
    satisfied = detail.get("satisfied", False)
    message = detail.get("message", "")

    # Build children recursively
    children = _build_condition_tree(detail)

    return ConditionNode(
        label=label,
        satisfied=satisfied,
        message=message,
        condition_type=condition_type,
        role="",
        details={k: v for k, v in detail.items() if k not in _STRUCTURAL_KEYS},
        children=children,
    )


def load_scenario(
    base_path: Path, session_type: str, date: str, time: str, index: int
) -> ScenarioResultView | None:
    """Load the full result detail for a single scenario."""
    cache_key = f"scenario:{session_type}:{date}:{time}:{index}"
    if cache_key in _cache:
        return _cache[cache_key]  # type: ignore[return-value]

    data: dict[str, Any] | None = None
    overrides: list[str] = []
    job_dir: Path | None = None

    if session_type == "multirun":
        job_dir = base_path / "multirun" / date / time / str(index)
        if not job_dir.is_dir():
            return None
        overrides = _read_overrides(job_dir)
        result_files = sorted(job_dir.glob("*_result.json"))
        if result_files:
            data = _read_result_json(result_files[0])

    elif session_type == "batch":
        job_dir = base_path / "outputs" / date / time
        batch_json = job_dir / "batch_results.json"
        overrides = _read_overrides(job_dir)
        if batch_json.is_file():
            try:
                data_list = json.loads(batch_json.read_text(encoding="utf-8"))
                if isinstance(data_list, list) and 0 <= index < len(data_list):
                    data = data_list[index]
            except (json.JSONDecodeError, OSError):
                pass

    elif session_type == "single":
        job_dir = base_path / "outputs" / date / time
        overrides = _read_overrides(job_dir)
        result_files = sorted(job_dir.glob("*_result.json"))
        if result_files and index == 0:
            data = _read_result_json(result_files[0])

    if data is None:
        # No result JSON (e.g. spawn failed), but the job directory exists.
        # Return a partial view with only the log and overrides so the viewer
        # can still display useful information instead of returning a 500 error.
        if job_dir is None:
            return None
        raw_log = _read_raw_log(job_dir)
        if not raw_log and not overrides:
            return None
        result = ScenarioResultView(
            passed=None,
            message="No result data available (scenario may have failed to start)",
            elapsed_seconds=None,
            condition_statuses=[],
            overrides=overrides,
            raw_log=raw_log,
        )
        _cache[cache_key] = result
        return result

    # Build condition tree
    condition_statuses: list[ConditionNode] = []
    for cs in data.get("condition_statuses", []):
        children = _build_condition_tree(cs.get("details", {}))
        condition_statuses.append(
            ConditionNode(
                label=cs.get("label", ""),
                satisfied=cs.get("satisfied", False),
                message=cs.get("message", ""),
                condition_type=cs.get("condition_type", ""),
                role=cs.get("role", ""),
                details={
                    k: v
                    for k, v in cs.get("details", {}).items()
                    if k not in _STRUCTURAL_KEYS
                },
                children=children,
            )
        )

    raw_log = _read_raw_log(job_dir) if job_dir else ""

    result = ScenarioResultView(
        passed=data.get("passed"),
        message=data.get("message", ""),
        elapsed_seconds=data.get("elapsed_seconds"),
        condition_statuses=condition_statuses,
        overrides=overrides,
        raw_log=raw_log,
    )
    _cache[cache_key] = result
    return result
