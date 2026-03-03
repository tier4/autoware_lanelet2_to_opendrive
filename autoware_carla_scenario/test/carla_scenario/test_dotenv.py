"""Unit tests for dotenv loading in server.py."""

from __future__ import annotations

import os
from pathlib import Path

import pytest
from dotenv import load_dotenv


def test_env_var_loaded_from_dotenv(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """CARLA_UE5_EXECUTABLE defined in .env is populated into os.environ."""
    dotenv_file = tmp_path / ".env"
    dotenv_file.write_text("CARLA_UE5_EXECUTABLE=/fake/CarlaUE5.sh\n")

    monkeypatch.delenv("CARLA_UE5_EXECUTABLE", raising=False)

    load_dotenv(str(dotenv_file), override=False)

    assert os.environ.get("CARLA_UE5_EXECUTABLE") == "/fake/CarlaUE5.sh"


def test_shell_export_takes_precedence_over_dotenv(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """A shell-exported value is not overridden by .env (override=False)."""
    dotenv_file = tmp_path / ".env"
    dotenv_file.write_text("CARLA_UE5_EXECUTABLE=/from/dotenv/CarlaUE5.sh\n")

    monkeypatch.setenv("CARLA_UE5_EXECUTABLE", "/from/shell/CarlaUE5.sh")

    load_dotenv(str(dotenv_file), override=False)

    assert os.environ.get("CARLA_UE5_EXECUTABLE") == "/from/shell/CarlaUE5.sh"


def test_map_path_env_var_loaded_from_dotenv(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Map path env vars defined in .env are populated into os.environ."""
    dotenv_file = tmp_path / ".env"
    dotenv_file.write_text("NISHISHINJYUKU_MAP_PATH=/fake/NishishinjyukuMap.xodr\n")

    monkeypatch.delenv("NISHISHINJYUKU_MAP_PATH", raising=False)

    load_dotenv(str(dotenv_file), override=False)

    assert os.environ.get("NISHISHINJYUKU_MAP_PATH") == "/fake/NishishinjyukuMap.xodr"
