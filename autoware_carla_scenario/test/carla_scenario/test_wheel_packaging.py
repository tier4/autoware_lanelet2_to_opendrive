"""Guard the wheel contents that only show up once the package is built.

An editable install puts ``src/`` on ``sys.path`` wholesale, so a package the
build backend never copies still imports fine during development and only goes
missing inside a wheel -- and therefore inside the container image built by
``.github/actions/pack-scenario-image``.  The ``hydra_plugins`` namespace
package is exactly that blind spot: without it Hydra never picks up a scenario
package's ``conf/`` directory and every ``scenario=<name>/...`` override fails
with "Could not find".

So the test builds the wheel and looks inside it, rather than asserting on how
the build configuration happens to be spelled today.
"""

from __future__ import annotations

import shutil
import subprocess
import zipfile
from pathlib import Path

import pytest

_PACKAGE_ROOT = Path(__file__).resolve().parents[2]
_WORKSPACE_ROOT = _PACKAGE_ROOT.parent
_HYDRA_PLUGINS_DIR = _PACKAGE_ROOT / "src" / "hydra_plugins"


@pytest.fixture(scope="module")
def wheel_contents(tmp_path_factory: pytest.TempPathFactory) -> set[str]:
    """Build the framework wheel once and return the paths it holds."""
    if shutil.which("uv") is None:
        pytest.skip("uv is required to build the wheel")
    out_dir = tmp_path_factory.mktemp("wheelhouse")
    result = subprocess.run(
        [
            "uv",
            "build",
            "--wheel",
            "--package",
            "autoware-carla-scenario",
            "--out-dir",
            str(out_dir),
            str(_WORKSPACE_ROOT),
        ],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, f"uv build failed:\n{result.stderr}"
    (wheel,) = out_dir.glob("*.whl")
    with zipfile.ZipFile(wheel) as archive:
        return set(archive.namelist())


def test_every_hydra_plugin_reaches_the_wheel(wheel_contents: set[str]) -> None:
    """A plugin the backend does not copy breaks config discovery in the image."""
    shipped = {
        name.split("/")[1]
        for name in wheel_contents
        if name.startswith("hydra_plugins/") and name.count("/") > 1
    }
    on_disk = {
        child.name
        for child in _HYDRA_PLUGINS_DIR.iterdir()
        if child.is_dir() and child.name != "__pycache__"
    }
    assert on_disk, "no hydra plugins found -- has the layout changed?"
    assert (
        on_disk <= shipped
    ), f"hydra plugin(s) missing from the wheel: {sorted(on_disk - shipped)}"


def test_framework_package_reaches_the_wheel(wheel_contents: set[str]) -> None:
    """Naming the plugins must not displace the package itself."""
    assert "autoware_carla_scenario/__init__.py" in wheel_contents
    # The scenario CLI is useless without the built-in Hydra configs.
    assert any(name.endswith(".yaml") for name in wheel_contents)


def test_hydra_plugins_is_a_namespace_package() -> None:
    """``hydra_plugins`` is shared with Hydra, so it must stay PEP 420."""
    assert not (_HYDRA_PLUGINS_DIR / "__init__.py").exists()


def test_no_manifest_uses_the_ignored_build_table() -> None:
    """uv reads ``[tool.uv.build-backend]`` and silently ignores other spellings.

    A manifest carrying the ignored ``[tool.uv-build]`` table looks configured
    but is not, so the backend falls back to guessing a single module from the
    distribution name -- which is how ``hydra_plugins`` was dropped in the first
    place.
    """
    offenders = [
        path.relative_to(_WORKSPACE_ROOT)
        for path in _WORKSPACE_ROOT.rglob("pyproject.toml*")
        if ".venv" not in path.parts and "[tool.uv-build]" in path.read_text()
    ]
    assert (
        not offenders
    ), f"these manifests use the table uv ignores: {sorted(map(str, offenders))}"
