"""Scenario Result Viewer - Web UI for browsing scenario test results."""

from __future__ import annotations

import os
from pathlib import Path


def main() -> None:
    """Entry point for ``uv run viewer``."""
    import uvicorn  # noqa: PLC0415

    from .app import app  # noqa: PLC0415, F811

    # Allow overriding the base path via environment variable.
    env_base = os.environ.get("VIEWER_BASE_PATH")
    if env_base:
        import autoware_carla_scenario.ui.app as app_module  # noqa: PLC0415

        app_module.BASE_PATH = Path(env_base).resolve()

    host = os.environ.get("VIEWER_HOST", "0.0.0.0")  # noqa: S104
    port = int(os.environ.get("VIEWER_PORT", "9000"))

    uvicorn.run(app, host=host, port=port, log_level="info")
