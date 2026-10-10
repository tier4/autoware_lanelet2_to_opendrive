"""End-to-end tests: a lanelet shorter than min_segment_length keeps its geometry.

``short_lanelet_mini.osm`` chains three single-lane lanelets along s
(10 m -> 0.3 m -> 10 m). The 0.3 m lanelet used to be emitted as
``<road length="0"><planView/>`` because its only ParamPoly3 segment was
dropped for being shorter than ``parampoly3.min_segment_length``; a road
without geometry crashes CARLA's OpenDRIVE parser.
"""

import subprocess
from pathlib import Path

import lxml.etree as ET
import pytest

from autoware_lanelet2_to_opendrive.qc_validate import (
    load_ignore_patterns,
    validate,
)

FIXTURE = (Path(__file__).parent / "data" / "short_lanelet_mini.osm").resolve()


def _run_convert(tmp_path: Path) -> Path:
    """Run the ``convert`` CLI on the fixture and return output path."""
    out = tmp_path / "short_lanelet_mini.xodr"
    # Same minimal map config as walkway_mini / speed_change_mini.
    subprocess.run(
        [
            "uv",
            "run",
            "convert",
            "map=example_mgrs_offset",
            "target=carla",
            f"input_map_path={FIXTURE}",
            f"output_map_path={out}",
        ],
        check=True,
    )
    return out


def test_every_road_has_planview_geometry(tmp_path: Path) -> None:
    """Every <road> must carry at least one <geometry> and a positive length."""
    out = _run_convert(tmp_path)
    root = ET.parse(str(out)).getroot()
    roads = root.findall("road")
    assert len(roads) == 3, f"expected one road per lanelet, got {len(roads)}"
    for road in roads:
        assert road.findall(
            "planView/geometry"
        ), f"road {road.get('id')} has no geometry"
        assert float(road.get("length")) > 0.0, f"road {road.get('id')} has zero length"
    shortest = min(float(road.get("length")) for road in roads)
    assert shortest == pytest.approx(0.3, abs=0.05)


def test_short_lanelet_fixture_passes_qc_validate(tmp_path: Path) -> None:
    """The emitted OpenDRIVE must pass qc-framework with zero ERRORs."""
    out = _run_convert(tmp_path)
    errors = validate(out, load_ignore_patterns())
    assert errors == 0, f"qc-framework reported {errors} ERROR(s)"


def test_short_lanelet_fixture_loads_in_carla(tmp_path: Path) -> None:
    """CARLA's OpenDRIVE parser must accept the 0.3 m road."""
    carla = pytest.importorskip("carla")
    out = _run_convert(tmp_path)
    carla_map = carla.Map("short_lanelet_mini", out.read_text())
    assert carla_map.generate_waypoints(1.0)
