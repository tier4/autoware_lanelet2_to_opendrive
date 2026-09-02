"""Unit tests for the layer-splitting helper shipped with the packing action.

The helper decides which file ends up in which layer of the scenario image, and
the image is only worth pulling incrementally if it gets that exactly right: a
file dropped from every layer breaks the image, and a file duplicated into a
later one costs the download the split was meant to save.  The Dockerfile
reassembles the layers and imports from the result, but that check only runs
inside a build, so the rules themselves are pinned here.
"""

from __future__ import annotations

import os
import subprocess
from pathlib import Path

import pytest

_SCRIPT = (
    Path(__file__).resolve().parents[3]
    / ".github"
    / "actions"
    / "pack-scenario-image"
    / "venv-layer.sh"
)

#: The default `normalize` stamps every exported file with this: 2020-01-01Z.
_EPOCH = 1577836800


def _run(*args: str | Path) -> subprocess.CompletedProcess[str]:
    """Run the helper, failing the test with its output if it errors."""
    return subprocess.run(
        ["sh", str(_SCRIPT), *(str(arg) for arg in args)],
        capture_output=True,
        text=True,
        check=True,
    )


def _install(venv: Path, relative: str, content: str = "x") -> Path:
    """Write a file into the virtualenv the way an install step would."""
    path = venv / "lib" / "python3.10" / "site-packages" / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content)
    return path


def _layer_files(layer: Path) -> set[str]:
    """Every file and symlink in a captured layer, relative to its root."""
    return {
        str(path.relative_to(layer))
        for path in layer.rglob("*")
        if path.is_file() or path.is_symlink()
    }


def _contents(root: Path) -> dict[str, bytes | str]:
    """What every file holds, and where every symlink points."""
    found: dict[str, bytes | str] = {}
    for path in root.rglob("*"):
        key = str(path.relative_to(root))
        if path.is_symlink():
            # Read as a link rather than followed: `bin/python` points at the
            # interpreter outside the tree, which is not this tree's content.
            found[key] = f"-> {os.readlink(path)}"
        elif path.is_file():
            found[key] = path.read_bytes()
    return found


@pytest.fixture
def venv(tmp_path: Path) -> Path:
    """A stand-in virtualenv with the one file `uv venv` would leave behind."""
    root = tmp_path / "venv"
    (root / "bin").mkdir(parents=True)
    (root / "bin" / "python").symlink_to("/usr/local/bin/python3")
    return root


def test_each_step_lands_in_its_own_layer(venv: Path, tmp_path: Path) -> None:
    """The whole point: a step's files, and only its files, are its layer."""
    export = tmp_path / "export"

    _install(venv, "carla/__init__.py")
    _run("capture", venv, export, "carla")
    _install(venv, "numpy/__init__.py")
    _run("capture", venv, export, "deps")
    _install(venv, "autoware_carla_scenario/__init__.py")
    _run("capture", venv, export, "framework")

    site = "lib/python3.10/site-packages"
    assert _layer_files(export / "carla") == {"bin/python", f"{site}/carla/__init__.py"}
    assert _layer_files(export / "deps") == {f"{site}/numpy/__init__.py"}
    assert _layer_files(export / "framework") == {
        f"{site}/autoware_carla_scenario/__init__.py"
    }


def test_reassembling_the_layers_reproduces_the_virtualenv(
    venv: Path, tmp_path: Path
) -> None:
    """Nothing may be lost between the layers -- the runtime image is their sum."""
    export = tmp_path / "export"

    _install(venv, "carla/__init__.py", "client")
    _run("capture", venv, export, "carla")
    _install(venv, "numpy/__init__.py", "dependency")
    _install(venv, "numpy/core.py", "dependency")
    _run("capture", venv, export, "deps")
    _install(venv, "my_scenario/__init__.py", "scenario")
    _run("capture", venv, export, "scenario")

    stacked = tmp_path / "stacked"
    stacked.mkdir()
    for layer in ("carla", "deps", "scenario"):
        subprocess.run(["cp", "-a", f"{export / layer}/.", str(stacked)], check=True)

    assert _contents(stacked) == _contents(venv)


def test_a_rewritten_file_moves_to_the_later_layer(venv: Path, tmp_path: Path) -> None:
    """Otherwise the stack would keep the stale copy from the earlier layer."""
    export = tmp_path / "export"

    _install(venv, "shared.pth", "before")
    _run("capture", venv, export, "carla")
    _install(venv, "shared.pth", "after")
    _run("capture", venv, export, "deps")

    site = "lib/python3.10/site-packages"
    assert (export / "carla" / site / "shared.pth").read_text() == "before"
    # The later layer wins when they are stacked, so it has to carry the new
    # content -- a path diff alone would never notice this file at all.
    assert (export / "deps" / site / "shared.pth").read_text() == "after"


def test_a_new_file_with_an_old_timestamp_is_still_captured(
    venv: Path, tmp_path: Path
) -> None:
    """An installer may copy timestamps out of the wheel it unpacks."""
    export = tmp_path / "export"

    _install(venv, "carla/__init__.py")
    _run("capture", venv, export, "carla")
    vintage = _install(venv, "vintage/__init__.py")
    os.utime(vintage, (100_000, 100_000))
    _run("capture", venv, export, "deps")

    site = "lib/python3.10/site-packages"
    assert _layer_files(export / "deps") == {f"{site}/vintage/__init__.py"}


def test_the_state_directory_is_not_part_of_any_layer(
    venv: Path, tmp_path: Path
) -> None:
    """It lives in the export root, which the runtime stage copies from."""
    export = tmp_path / "export"

    _install(venv, "carla/__init__.py")
    _run("capture", venv, export, "carla")

    assert (export / ".state").is_dir()
    assert not any(".state" in path for path in _layer_files(export / "carla"))


def test_normalize_stamps_every_exported_path(venv: Path, tmp_path: Path) -> None:
    """Equal content has to mean an equal layer, whenever it was installed."""
    export = tmp_path / "export"

    _install(venv, "carla/__init__.py")
    _run("capture", venv, export, "carla")
    _run("normalize", export)

    layer = export / "carla"
    stamped = [layer, *layer.rglob("*")]
    assert len(stamped) > 3
    for path in stamped:
        assert path.lstat().st_mtime == _EPOCH, path


def test_normalize_accepts_an_explicit_epoch(venv: Path, tmp_path: Path) -> None:
    """`LAYER_MTIME` is a build argument, so the value has to be settable."""
    export = tmp_path / "export"

    _install(venv, "carla/__init__.py")
    _run("capture", venv, export, "carla")
    _run("normalize", export, "1000000000")

    site = export / "carla" / "lib" / "python3.10" / "site-packages"
    assert (site / "carla" / "__init__.py").lstat().st_mtime == 1_000_000_000
