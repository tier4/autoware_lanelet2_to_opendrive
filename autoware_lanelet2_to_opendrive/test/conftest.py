"""Pytest configuration and fixtures."""

import os
import subprocess
from pathlib import Path

import pytest
from filelock import FileLock

# Import autoware extensions before any tests to ensure proper registration.
# Both projection AND regulatory_elements must be imported so that the
# lanelet2 C++ factory recognises custom types (AutowareTrafficLight,
# RoadMarking, DetectionArea, etc.) when loading maps.
from autoware_lanelet2_extension_python.projection import MGRSProjector  # noqa: F401
import autoware_lanelet2_extension_python.regulatory_elements as _ll2_ext_reg  # noqa: F401
import lanelet2  # noqa: F401


@pytest.fixture(scope="session")
def lanelet_map():
    """Load test map once and cache for entire test session.

    This fixture loads the large nishishinjuku.osm file (11MB, 307k lines) once
    per test session and reuses it across all tests. This significantly reduces
    test execution time by avoiding repeated file I/O and parsing.

    Returns:
        lanelet2.core.LaneletMap: The loaded lanelet2 map.
    """
    test_data_path = Path(__file__).parent / "data" / "nishishinjuku.osm"
    projector = MGRSProjector(
        lanelet2.io.Origin(35.23, 139.16)
    )  # MGRS origin for Tokyo area (54SUE)
    return lanelet2.io.load(str(test_data_path), projector)


@pytest.fixture(scope="session")
def nishishinjuku_xodr(tmp_path_factory: pytest.TempPathFactory) -> Path:
    """Convert the Nishishinjuku fixture once per pytest invocation.

    The end-to-end conversion takes about 150 s. Every test that needs the
    converted map requests this fixture instead of running its own
    ``uv run convert``, so the whole invocation pays for it once.

    The output lives under this invocation's temp directory, never a fixed
    cached path, so a regression test always exercises the *current*
    converter rather than an XODR left behind by an earlier checkout.

    Under ``pytest-xdist`` each worker has its own session, so the file is
    written to ``getbasetemp().parent`` -- the one directory shared by every
    worker of the same invocation -- behind a :class:`filelock.FileLock`;
    the first worker converts and the rest reuse its output. Without xdist
    that parent is shared *across* invocations, so the worker's own
    ``getbasetemp()`` is used instead.
    """
    fixture = (Path(__file__).parent / "data" / "nishishinjuku.osm").resolve()
    if not fixture.is_file():
        pytest.skip(f"{fixture} not available; cannot build XODR")

    base = tmp_path_factory.getbasetemp()
    if "PYTEST_XDIST_WORKER" in os.environ:
        base = base.parent
    xodr_path = base / "nishishinjuku_carla.xodr"

    with FileLock(str(base / "nishishinjuku_carla.xodr.lock")):
        if xodr_path.is_file():
            return xodr_path

        # Stage then rename, so no worker ever observes a partial file.
        staged = xodr_path.with_name(f"{xodr_path.name}.tmp.{os.getpid()}")
        cmd = [
            "uv",
            "run",
            "convert",
            "map=nishishinjuku",
            "target=carla",
            f"input_map_path={fixture}",
            f"output_map_path={staged}",
        ]
        try:
            subprocess.run(cmd, check=True)
        except FileNotFoundError as exc:
            pytest.skip(f"converter unavailable: {exc}")

        if not staged.is_file():
            pytest.fail(f"converter exited successfully but {staged} was not produced")
        os.replace(staged, xodr_path)

    return xodr_path
