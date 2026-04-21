"""End-to-end tests for P0-1: Lanelet2 walkway → OpenDRIVE sidewalk lane.

Converts the synthetic ``walkway_mini.osm`` fixture and verifies that
at least one ``lane[type="sidewalk"]`` is emitted and that the result
passes the qc-framework validator with zero ERROR-level findings.
"""

import subprocess
from pathlib import Path

import lxml.etree as ET

FIXTURE = (Path(__file__).parent / "data" / "walkway_mini.osm").resolve()


def _run_convert(tmp_path: Path) -> Path:
    """Run the ``convert`` CLI on the walkway fixture and return output path."""
    out = tmp_path / "walkway_mini.xodr"
    # Use the minimal example_mgrs_offset map config (same MGRS offset as
    # nishishinjuku) to avoid preprocessing operations that reference
    # lanelet/point IDs that do not exist in this synthetic fixture.
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


def test_walkway_emits_sidewalk_lane(tmp_path: Path) -> None:
    """A Lanelet2 subtype=walkway lanelet must produce lane[type=sidewalk]."""
    out = _run_convert(tmp_path)
    root = ET.parse(str(out)).getroot()
    sidewalks = root.findall(".//lane[@type='sidewalk']")
    assert (
        len(sidewalks) >= 1
    ), "walkway lanelet should produce at least one lane[type='sidewalk']"


def test_walkway_fixture_passes_qc_validate(tmp_path: Path) -> None:
    """The emitted OpenDRIVE must pass qc-framework with zero ERRORs."""
    from autoware_lanelet2_to_opendrive.qc_validate import (
        load_ignore_patterns,
        validate,
    )

    out = _run_convert(tmp_path)
    errors = validate(out, load_ignore_patterns())
    assert errors == 0, f"qc-framework reported {errors} ERROR(s)"
