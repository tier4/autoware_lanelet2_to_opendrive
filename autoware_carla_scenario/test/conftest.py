"""Top-level test configuration for autoware_carla_scenario.

Generates derived test data (e.g. OpenDRIVE files) on-demand so that
CI and local development work without committing large generated files.

CARLA availability
-------------------
CARLA ships x86_64-only wheels, so it cannot be installed on an arm64-native
interpreter (see the ``dev-nocarla``/``pytest-native`` Docker targets). When
``carla`` is not importable, every test module that imports it -- directly,
or transitively through ``autoware_carla_scenario.coordinate.poses`` /
``.transform`` / ``utils.traffic_light`` / ``server`` -- is excluded from
collection via ``collect_ignore`` below, and ``pytest_report_header``
reports how many modules were skipped.

This list intentionally lives here, at the testpath root, rather than in
``carla_scenario/conftest.py`` (which still carries its own
``_CARLA_AVAILABLE`` guard for the CARLA imports it performs itself -- see
that module's docstring): pytest emits the report header before collection
starts, but a subdirectory conftest is only loaded once collection descends
into that directory, so a header hook defined there never fires (confirmed
empirically for both serial and ``-n`` xdist runs, pytest 9.1.1 /
pytest-xdist 3.8.0). A root-level ``collect_ignore`` also accepts nested
paths (e.g. ``"carla_scenario/test_conditions.py"``), so keeping the ignore
list and the header hook together here keeps them a single source of truth.
"""

from __future__ import annotations

import importlib.util
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

_CARLA_AVAILABLE = importlib.util.find_spec("carla") is not None

if not _CARLA_AVAILABLE:
    # Alphabetical by filename, to make the count easy to eyeball and new
    # entries easy to slot in.
    collect_ignore = [
        "carla_scenario/test_composition_conditions.py",  # via coordinate.poses
        "carla_scenario/test_conditions.py",  # via coordinate.poses
        "carla_scenario/test_coordinate.py",  # via coordinate.transform (direct)
        # kinematics/__init__.py -> .acceleration -> .frames ->
        # coordinate.frames -> coordinate/__init__.py -> .poses -> poses.py:
        # import carla
        "carla_scenario/test_kinematics.py",
        "carla_scenario/test_road_lanelet_mapping.py",  # via coordinate.transform (direct)
        "carla_scenario/test_scenario_base.py",  # imports carla directly
        "carla_scenario/test_scenario_queue.py",  # imports carla directly
        "carla_scenario/test_scenario_registry.py",  # imports carla directly
        "carla_scenario/test_server.py",  # via server.py
        # No module-level `import carla` in this file's own source, but it
        # uses `unittest.mock.patch("autoware_carla_scenario.coordinate...")`
        # / `("autoware_carla_scenario.utils...")` with a dotted *string*
        # target. `mock.patch` resolves that string by actually importing the
        # named module, so patching anything under
        # `autoware_carla_scenario.coordinate` or `.utils` still triggers an
        # `import carla` transitively (coordinate/__init__.py -> .poses ->
        # poses.py, or utils/__init__.py -> .traffic_light ->
        # coordinate.transform). Do not remove this entry just because no
        # `import carla` is visible here -- this is exactly the case the
        # design doc's static analysis missed.
        "carla_scenario/test_stop_line.py",
        "carla_scenario/test_traffic_light_utils.py",  # via utils.traffic_light
        # actions/__init__.py -> attach_camera_sensor.py ->
        # conditions/__init__.py -> .collision -> collision.py: import carla
        "carla_scenario/test_traffic_signal_action.py",
        # Same import chain as test_traffic_signal_action.py above:
        # conditions/__init__.py -> .collision -> collision.py: import carla
        "carla_scenario/test_traffic_signal_condition.py",
    ]


def pytest_report_header(config: pytest.Config) -> str | None:
    """Report how many CARLA-dependent modules were skipped from collection.

    Without this, a full "green" run on the arm64-native target would look
    identical to a full CARLA-inclusive run, silently hiding regressions on
    the CARLA code path.
    """
    if _CARLA_AVAILABLE:
        return None
    return f"carla not installed: {len(collect_ignore)} test modules ignored"


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
