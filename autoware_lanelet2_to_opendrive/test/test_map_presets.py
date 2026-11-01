"""The shipped example presets must convert.

``conf/map/example*.yaml`` exist to be copied by someone starting a new map,
so a preset that cannot complete a conversion is worse than no preset: it is
a worked example of something that does not work. Nothing in the suite ran
them, and one of the three does not survive startup.

The fixture is ``walkway_mini.osm`` because it is the smallest map in the
tree and this test is about the configuration, not the geometry. The
``nishishinjuku`` and ``odaiba`` presets are deliberately out of scope --
they name specific lanelet ids and belong to their own maps.
"""

import subprocess
from pathlib import Path

import pytest

CONF_MAP = (
    Path(__file__).parent.parent
    / "src"
    / "autoware_lanelet2_to_opendrive"
    / "conf"
    / "map"
)
FIXTURE = (Path(__file__).parent / "data" / "walkway_mini.osm").resolve()

EXAMPLE_PRESETS = sorted(path.stem for path in CONF_MAP.glob("example*.yaml"))


def test_the_example_presets_are_discovered() -> None:
    """Guard the parametrisation: an empty glob would vacuously pass."""
    assert len(EXAMPLE_PRESETS) >= 3, (
        f"expected at least three example presets under {CONF_MAP},"
        f" found {EXAMPLE_PRESETS}"
    )


@pytest.mark.parametrize("preset", EXAMPLE_PRESETS)
def test_an_example_preset_converts(preset: str, tmp_path: Path) -> None:
    """Each example preset must produce an .xodr from the shared fixture."""
    out = tmp_path / f"{preset}.xodr"
    result = subprocess.run(
        [
            "uv",
            "run",
            "convert",
            f"map={preset}",
            "target=carla",
            f"input_map_path={FIXTURE}",
            f"output_map_path={out}",
        ],
        capture_output=True,
        text=True,
    )
    # Hydra logs to stdout, so stderr alone is empty on a config error -- the
    # useful line is the last one on either stream.
    tail = (result.stdout + result.stderr).splitlines()[-6:]
    assert result.returncode == 0, (
        f"map={preset} exited "
        + str(result.returncode)
        + ". Last output lines:\n"
        + "\n".join(tail)
    )
    assert out.is_file(), f"map={preset} exited 0 but wrote no {out.name}"
