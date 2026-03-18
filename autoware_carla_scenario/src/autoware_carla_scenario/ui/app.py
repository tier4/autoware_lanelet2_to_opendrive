"""FastAPI application and route definitions for the scenario result viewer."""

from __future__ import annotations

import asyncio
import json
import logging
from pathlib import Path
from typing import Any

from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates

from . import runner, scanner
from .models import RunProgress

logger = logging.getLogger(__name__)

_UI_DIR = Path(__file__).resolve().parent
_TEMPLATES_DIR = _UI_DIR / "templates"
_STATIC_DIR = _UI_DIR / "static"

app = FastAPI(title="Scenario Result Viewer")
app.mount("/static", StaticFiles(directory=str(_STATIC_DIR)), name="static")
templates = Jinja2Templates(directory=str(_TEMPLATES_DIR))

# Base path for multirun/ and outputs/ directories.
# Resolved at startup; can be overridden via environment variable.
BASE_PATH = Path.cwd()


def _base_path() -> Path:
    """Return the base path for log directories."""
    return BASE_PATH


def _is_glob(value: str) -> bool:
    """Return ``True`` if *value* contains glob metacharacters."""
    return any(ch in value for ch in ("*", "?", "["))


# ── Page routes ──────────────────────────────────────────────────────────


@app.get("/", response_class=HTMLResponse)
async def index(request: Request) -> HTMLResponse:
    """Session list page."""
    sessions = scanner.scan_sessions(_base_path())
    return templates.TemplateResponse(
        "index.html",
        {
            "request": request,
            "sessions": sessions,
            "page": "index",
        },
    )


@app.get("/session/{session_type}/{date}/{time}", response_class=HTMLResponse)
async def session_detail(
    request: Request, session_type: str, date: str, time: str
) -> HTMLResponse:
    """Session detail page showing all scenarios in the session."""
    items = scanner.load_session(_base_path(), session_type, date, time)
    passed_count = sum(1 for it in items if it.passed is True)
    total_count = len(items)
    return templates.TemplateResponse(
        "session.html",
        {
            "request": request,
            "items": items,
            "session_type": session_type,
            "date": date,
            "time": time,
            "passed_count": passed_count,
            "total_count": total_count,
            "page": "session",
        },
    )


@app.get(
    "/session/{session_type}/{date}/{time}/{index}",
    response_class=HTMLResponse,
)
async def scenario_detail(
    request: Request, session_type: str, date: str, time: str, index: int
) -> HTMLResponse:
    """Individual scenario detail page with condition tree."""
    result = scanner.load_scenario(_base_path(), session_type, date, time, index)
    # Also load session items for navigation context
    items = scanner.load_session(_base_path(), session_type, date, time)
    scenario_name = ""
    for it in items:
        if it.index == index:
            scenario_name = it.scenario_name
            break
    return templates.TemplateResponse(
        "scenario.html",
        {
            "request": request,
            "result": result,
            "scenario_name": scenario_name,
            "session_type": session_type,
            "date": date,
            "time": time,
            "index": index,
            "page": "scenario",
        },
    )


# ── API routes ───────────────────────────────────────────────────────────


@app.post("/api/refresh", response_class=HTMLResponse)
async def refresh(request: Request) -> HTMLResponse:
    """Clear cache and return updated session table partial."""
    scanner.clear_cache()
    sessions = scanner.scan_sessions(_base_path())
    return templates.TemplateResponse(
        "partials/session_table.html",
        {"request": request, "sessions": sessions},
    )


@app.post("/api/refresh/session/{session_type}/{date}/{time}")
async def refresh_session(
    request: Request, session_type: str, date: str, time: str
) -> HTMLResponse:
    """Clear cache and return updated session detail."""
    scanner.clear_cache()
    items = scanner.load_session(_base_path(), session_type, date, time)
    passed_count = sum(1 for it in items if it.passed is True)
    total_count = len(items)
    return templates.TemplateResponse(
        "partials/session_table.html",
        {
            "request": request,
            "items": items,
            "session_type": session_type,
            "date": date,
            "time": time,
            "passed_count": passed_count,
            "total_count": total_count,
            "page": "session",
        },
    )


@app.get("/api/scenarios")
async def list_scenarios() -> list[str]:
    """Return available scenario config names from the conf directory."""
    return scanner.list_scenario_configs()


@app.post("/api/run/preview")
async def run_preview(request: Request) -> dict[str, str]:
    """Return the command that would be executed, for preview."""
    body: dict[str, Any] = {}
    try:
        body = await request.json()
    except Exception:  # noqa: BLE001
        pass

    scenario: str = body.get("scenario", "*/*")
    extra_overrides: list[str] = body.get("extra_overrides", [])
    sweeper: str = body.get("sweeper", "")

    cmd = runner.build_command(scenario, extra_overrides, sweeper)
    return {"command": " ".join(cmd)}


@app.post("/api/run")
async def run_scenarios(request: Request) -> dict[str, str]:
    """Start scenario execution in background.

    The request body specifies the scenario pattern and options
    explicitly from the Run Options panel.
    """
    body: dict[str, Any] = {}
    try:
        body = await request.json()
    except Exception:  # noqa: BLE001
        pass

    scenario: str = body.get("scenario", "*/*")
    extra_overrides: list[str] = body.get("extra_overrides", [])
    timeout: int = body.get("timeout", 300)
    sweeper: str = body.get("sweeper", "")

    if sweeper:
        # Resolve sweep constraints in-process (lightweight, no CARLA).
        # This expands the scenario + constraints into concrete per-job
        # override lists, which are then run as individual subprocesses.
        from .sweep_resolver import resolve_sweep  # noqa: PLC0415

        # Resolve glob to concrete scenario names first.
        if _is_glob(scenario):
            import fnmatch  # noqa: PLC0415

            all_names = scanner.list_scenario_configs()
            scenario_names = [n for n in all_names if fnmatch.fnmatch(n, scenario)]
        else:
            scenario_names = [scenario]

        if not scenario_names:
            return {"status": "error", "message": f"No scenarios match '{scenario}'"}

        overrides_list: list[list[str]] = []
        for name in scenario_names:
            try:
                batches = resolve_sweep(name, extra_overrides)
                overrides_list.extend(batches)
            except Exception as exc:  # noqa: BLE001
                logger.error("Sweep resolution failed for %s: %s", name, exc)

        if not overrides_list:
            return {
                "status": "error",
                "message": "No jobs generated (no lanelets matched constraints)",
            }

        # Jobs already contain scenario= and sweep overrides;
        # run without sweeper flag (each is a plain single-scenario run).
        runner.start_run(
            overrides_list,
            base_path=_base_path(),
            extra_overrides=[],
            timeout=timeout,
            sweeper="",
        )
    else:
        # No sweeper: pass scenario pattern as-is.
        overrides_list = [[f"scenario={scenario}"]]
        runner.start_run(
            overrides_list,
            base_path=_base_path(),
            extra_overrides=extra_overrides,
            timeout=timeout,
            sweeper="",
        )

    return {"status": "started"}


@app.get("/api/run/progress")
async def run_progress() -> StreamingResponse:
    """SSE endpoint for run progress updates."""

    async def event_stream() -> Any:
        while True:
            progress = runner.get_progress()
            if progress is None:
                # Not running; send idle event and close
                data = json.dumps(
                    RunProgress(
                        current=0, total=0, scenario_name="", status="idle"
                    ).model_dump()
                )
                yield f"data: {data}\n\n"
                break

            data = json.dumps(progress.model_dump())
            yield f"data: {data}\n\n"

            if progress.status == "done":
                break

            await asyncio.sleep(0.5)

    return StreamingResponse(event_stream(), media_type="text/event-stream")
