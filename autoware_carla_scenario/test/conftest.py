"""Top-level test configuration for autoware_carla_scenario.

Generates derived test data (e.g. OpenDRIVE files) on-demand so that
CI and local development work without committing large generated files.
"""

from __future__ import annotations

import subprocess
from pathlib import Path

import pytest

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
_CONVERTER_TEST_DATA = (
    _PROJECT_ROOT / "autoware_lanelet2_to_opendrive" / "test" / "data"
)
_XODR_PATH = _CONVERTER_TEST_DATA / "nishishinjuku_carla.xodr"
_OSM_PATH = _CONVERTER_TEST_DATA / "nishishinjuku.osm"


@pytest.fixture(scope="session", autouse=True)
def _ensure_nishishinjuku_xodr() -> None:
    """Generate nishishinjuku_carla.xodr from the source OSM if missing."""
    if _XODR_PATH.exists():
        return

    if not _OSM_PATH.exists():
        pytest.skip(f"Source OSM not found: {_OSM_PATH}")

    subprocess.run(
        [
            "uv",
            "run",
            "convert",
            "map=nishishinjuku",
            "target=carla",
            f"input_map_path={_OSM_PATH}",
            f"output_map_path={_XODR_PATH}",
        ],
        cwd=_PROJECT_ROOT,
        check=True,
    )
