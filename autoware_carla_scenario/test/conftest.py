"""Top-level test configuration for autoware_carla_scenario.

Generates derived test data (e.g. OpenDRIVE files) on-demand so that
CI and local development work without committing large generated files.
"""

from __future__ import annotations

import os
import subprocess
from pathlib import Path

import pytest
from filelock import FileLock

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
_CONVERTER_TEST_DATA = (
    _PROJECT_ROOT / "autoware_lanelet2_to_opendrive" / "test" / "data"
)
_XODR_PATH = _CONVERTER_TEST_DATA / "nishishinjuku_carla.xodr"
_OSM_PATH = _CONVERTER_TEST_DATA / "nishishinjuku.osm"


@pytest.fixture(scope="session", autouse=True)
def _ensure_nishishinjuku_xodr(tmp_path_factory: pytest.TempPathFactory) -> None:
    """Generate ``nishishinjuku_carla.xodr`` from the source OSM if missing.

    Safe under ``pytest-xdist``: a cross-worker :class:`filelock.FileLock`
    serialises the existence check and the convert subprocess, and the
    output is staged through a temp file then atomically renamed via
    :func:`os.replace`. This closes the TOCTOU race that let one worker
    read a partially-written XODR while another was still mid-write
    (issue #462).
    """
    if _XODR_PATH.exists():
        return

    if not _OSM_PATH.exists():
        pytest.skip(f"Source OSM not found: {_OSM_PATH}")

    # ``getbasetemp().parent`` is the one directory that is shared across
    # every xdist worker for the same pytest invocation, so it is the
    # natural location for an inter-worker lock sentinel.
    lock_path = tmp_path_factory.getbasetemp().parent / "nishishinjuku_carla.xodr.lock"

    with FileLock(str(lock_path)):
        # Re-check inside the critical section: another worker may have
        # produced the file while we were blocked on the lock.
        if _XODR_PATH.exists():
            return

        # Stage to a worker-unique temp path, then atomically rename so
        # other workers never observe a partially-written file.
        tmp_out = _XODR_PATH.with_name(f"{_XODR_PATH.name}.tmp.{os.getpid()}")
        try:
            subprocess.run(
                [
                    "uv",
                    "run",
                    "convert",
                    "map=nishishinjuku",
                    "target=carla",
                    f"input_map_path={_OSM_PATH}",
                    f"output_map_path={tmp_out}",
                ],
                cwd=_PROJECT_ROOT,
                check=True,
            )
            os.replace(tmp_out, _XODR_PATH)
        finally:
            if tmp_out.exists():
                try:
                    tmp_out.unlink()
                except OSError:
                    pass
