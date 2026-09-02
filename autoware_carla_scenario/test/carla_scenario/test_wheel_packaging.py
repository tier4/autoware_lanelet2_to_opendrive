"""Guard the wheel contents that only show up once the package is built.

An editable install puts ``src/`` on ``sys.path`` wholesale, so a package that
the build backend never copies still imports fine during development and only
goes missing inside a wheel -- and therefore inside the container image built
by ``docker/scenario/Dockerfile``.  The ``hydra_plugins`` namespace package is
exactly that kind of blind spot: without it Hydra never picks up a scenario
package's ``conf/`` directory and every ``scenario=<name>/...`` override fails
with "Could not find".

These tests read the build configuration rather than building a wheel, so they
stay fast; ``docker/scenario/smoke-test.py`` checks the built artifact itself.
"""

from __future__ import annotations

import sys

from pathlib import Path

if sys.version_info >= (3, 11):
    import tomllib
else:  # the workspace is pinned to 3.10 by the CARLA client
    import tomli as tomllib

_PACKAGE_ROOT = Path(__file__).resolve().parents[2]
_PYPROJECT = _PACKAGE_ROOT / "pyproject.toml"
_HYDRA_PLUGINS_DIR = _PACKAGE_ROOT / "src" / "hydra_plugins"


def _build_backend_config() -> dict:
    """Return the ``[tool.uv.build-backend]`` table."""
    with _PYPROJECT.open("rb") as handle:
        pyproject = tomllib.load(handle)
    # Note the table name: uv reads `tool.uv.build-backend`, and silently
    # ignores anything else -- a misspelled table looks like it works until you
    # inspect the wheel.
    return pyproject["tool"]["uv"]["build-backend"]


def test_build_backend_declares_namespace_packages() -> None:
    """``namespace`` must be on, or dotted module names are rejected."""
    assert _build_backend_config().get("namespace") is True


def test_every_hydra_plugin_is_shipped() -> None:
    """Each plugin directory must be listed, or it is dropped from the wheel."""
    module_names = _build_backend_config()["module-name"]
    assert isinstance(module_names, list), "module-name must list every module"

    on_disk = {
        f"hydra_plugins.{child.name}"
        for child in _HYDRA_PLUGINS_DIR.iterdir()
        if child.is_dir() and child.name != "__pycache__"
    }
    assert on_disk, "no hydra plugins found -- has the layout changed?"
    assert on_disk <= set(module_names), (
        "hydra plugin(s) missing from [tool.uv.build-backend] module-name: "
        f"{sorted(on_disk - set(module_names))}"
    )


def test_hydra_plugins_is_a_namespace_package() -> None:
    """``hydra_plugins`` is shared with Hydra, so it must stay PEP 420."""
    assert not (_HYDRA_PLUGINS_DIR / "__init__.py").exists()


def test_framework_module_is_shipped() -> None:
    """Listing the plugins must not drop the package itself."""
    assert "autoware_carla_scenario" in _build_backend_config()["module-name"]
