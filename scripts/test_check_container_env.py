"""Tests for ``scripts/check_container_env.py``.

The check exists so that a test run never silently uses a ``.venv`` that no
longer matches ``uv.lock``: the venv lives in a named volume that survives
image rebuilds, so it goes stale whenever the lock moves.
"""

import importlib.util
from pathlib import Path

import pytest

# Loaded by path for the same reason as tdd.py: ``scripts/`` is not a package.
_spec = importlib.util.spec_from_file_location(
    "check_container_env", Path(__file__).with_name("check_container_env.py")
)
assert _spec is not None and _spec.loader is not None
check = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(check)

# What uv 0.9.7 prints for a venv that is behind the lock (exit 1).
OUTDATED = """\
Would use project environment at: .venv
Would uninstall 1 package
Would install 1 package
 - pillow==11.0.0
 + pillow==11.3.0
The environment is outdated; run `uv sync` to update the environment
"""


def test_check_command_mirrors_the_image_sync() -> None:
    """The workspace's own packages are left out: the bind-mounted source
    always differs from the copy the image was synced from, so uv would call
    every venv stale. Extraneous packages are left out too: only a missing or
    wrong-version dependency changes what the tests run against."""
    assert check.build_check_command("--dev --extra carla") == [
        "uv",
        "sync",
        "--frozen",
        "--check",
        "--no-install-workspace",
        "--inexact",
        "--dev",
        "--extra",
        "carla",
    ]


def test_a_synchronised_venv_passes() -> None:
    verdict = check.explain(0, "Would make no changes\n", "--dev", "venv-nocarla-cache")
    assert verdict.ok
    assert verdict.message == ""


def test_an_outdated_venv_fails_and_says_what_to_do() -> None:
    verdict = check.explain(1, OUTDATED, "--dev --extra carla", "venv-cache")
    assert not verdict.ok
    # uv's own account of the drift, so the reader sees what is stale
    assert " - pillow==11.0.0" in verdict.message
    assert " + pillow==11.3.0" in verdict.message
    # both ways out: sync in place with the image's arguments, or a new volume
    assert "uv sync --frozen --dev --extra carla" in verdict.message
    assert "L2O_ALLOW_STALE_VENV=1" in verdict.message
    assert "_venv-cache" in verdict.message


def test_a_check_that_cannot_run_does_not_block() -> None:
    """Any other uv failure (no network for a first resolve, say) is reported,
    but is not evidence that the venv is stale, so the run goes ahead."""
    verdict = check.explain(2, "error: something else\n", "--dev", "venv-nocarla-cache")
    assert verdict.ok
    assert "could not check" in verdict.message
    assert "error: something else" in verdict.message


def _uv(returncode: int, output: str):
    def run(cmd, **kwargs):
        return type("Done", (), {"returncode": returncode, "stdout": output})()

    return run


@pytest.fixture
def env(monkeypatch: pytest.MonkeyPatch) -> pytest.MonkeyPatch:
    monkeypatch.setenv("L2O_SYNC_ARGS", "--dev")
    monkeypatch.setenv("L2O_VENV_VOLUME", "venv-nocarla-cache")
    monkeypatch.delenv("L2O_ALLOW_STALE_VENV", raising=False)
    return monkeypatch


def test_main_stops_before_the_command_on_a_stale_venv(env) -> None:
    env.setattr(check.subprocess, "run", _uv(1, OUTDATED))
    env.setattr(check.os, "execvp", lambda *a: pytest.fail("must not exec"))
    assert check.main(["--", "pytest", "-n", "4"]) == 1


def test_main_runs_the_command_on_a_synchronised_venv(env) -> None:
    calls = []
    env.setattr(check.subprocess, "run", _uv(0, ""))
    env.setattr(check.os, "execvp", lambda file, args: calls.append(args))
    check.main(["--", "pytest", "-n", "4"])
    assert calls == [["pytest", "-n", "4"]]


def test_main_without_a_command_only_reports(env) -> None:
    env.setattr(check.subprocess, "run", _uv(1, OUTDATED))
    assert check.main([]) == 1
    env.setattr(check.subprocess, "run", _uv(0, ""))
    assert check.main([]) == 0


def test_the_escape_hatch_skips_the_check(env) -> None:
    calls = []
    env.setenv("L2O_ALLOW_STALE_VENV", "1")
    env.setattr(check.subprocess, "run", lambda *a, **k: pytest.fail("checked"))
    env.setattr(check.os, "execvp", lambda file, args: calls.append(args))
    check.main(["--", "uv", "sync", "--frozen", "--dev"])
    assert calls == [["uv", "sync", "--frozen", "--dev"]]


def test_an_image_without_its_sync_args_is_not_judged(
    env, capsys: pytest.CaptureFixture[str]
) -> None:
    """An image built before the check existed has no L2O_SYNC_ARGS. Checking
    with no extras would call a synchronised CARLA venv stale, so say why the
    check is skipped instead."""
    calls = []
    env.delenv("L2O_SYNC_ARGS")
    env.setattr(check.subprocess, "run", lambda *a, **k: pytest.fail("checked"))
    env.setattr(check.os, "execvp", lambda file, args: calls.append(args))
    check.main(["--", "pytest"])
    assert calls == [["pytest"]]
    assert "rebuild the image" in capsys.readouterr().err
