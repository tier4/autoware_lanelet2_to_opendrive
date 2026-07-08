"""Unit tests for the scenario-package generator.

These exercise :mod:`autoware_carla_scenario.scaffold` end-to-end.  The
generator has no CARLA dependency (it only renders Jinja2 templates), so the
tests -- including generating a package and byte-compiling its Python files --
run in any environment.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from autoware_carla_scenario.scaffold import (
    create_scenario_package,
    resolve_names,
)


class TestResolveNames:
    @pytest.mark.parametrize(
        ("raw", "expected"),
        [
            (
                "reach_goal_pkg",
                ("reach_goal_pkg", "reach-goal-pkg", "reach_goal", "ReachGoalScenario"),
            ),
            (
                "my-cool-scenario-package",
                (
                    "my_cool_scenario_package",
                    "my-cool-scenario-package",
                    "my_cool_scenario",
                    "MyCoolScenario",  # no "ScenarioScenario" doubling
                ),
            ),
            (
                "ParkingLot",
                ("parking_lot", "parking-lot", "parking_lot", "ParkingLotScenario"),
            ),
        ],
    )
    def test_name_derivation(
        self, raw: str, expected: tuple[str, str, str, str]
    ) -> None:
        names = resolve_names(raw)
        assert (
            names["package_name"],
            names["distribution_name"],
            names["scenario_name"],
            names["scenario_class"],
        ) == expected

    def test_explicit_scenario_name(self) -> None:
        names = resolve_names("my_pkg", scenario_name="overtake")
        assert names["scenario_name"] == "overtake"
        assert names["scenario_class"] == "OvertakeScenario"
        assert names["config_class"] == "OvertakeConfig"

    @pytest.mark.parametrize("bad", ["123", "class", "!!!", ""])
    def test_invalid_names_raise(self, bad: str) -> None:
        with pytest.raises(ValueError):
            resolve_names(bad)


class TestCreateScenarioPackage:
    def test_generates_valid_package(self, tmp_path: Path) -> None:
        result = create_scenario_package("reach_goal_pkg", output_dir=tmp_path)
        root = result.root

        assert root == tmp_path / "reach_goal_pkg"
        assert result.names["scenario_name"] == "reach_goal"
        pkg = root / "src" / "reach_goal_pkg"

        # Expected files exist.
        assert (root / "pyproject.toml").is_file()
        assert (root / "README.md").is_file()
        assert (pkg / "__init__.py").is_file()
        assert (pkg / "reach_goal.py").is_file()
        assert (pkg / "configs.py").is_file()
        yaml_path = pkg / "conf" / "scenario" / "reach_goal" / "default.yaml"
        assert yaml_path.is_file()

        # Python renders to compilable source.
        for py in (pkg / "__init__.py", pkg / "reach_goal.py", pkg / "configs.py"):
            compile(py.read_text(encoding="utf-8"), str(py), "exec")

        # pyproject carries the right distribution name + entry point.
        # (Assert on text rather than parse: tomllib is 3.11+ only, CI is 3.10.)
        pyproject = (root / "pyproject.toml").read_text(encoding="utf-8")
        assert 'name = "reach-goal-pkg"' in pyproject
        assert 'reach_goal_pkg = "reach_goal_pkg:register"' in pyproject
        assert '[project.entry-points."autoware_carla_scenario.scenarios"]' in pyproject

        # YAML binds the scenario name (assert on text; avoids a yaml stub dep).
        yaml_text = yaml_path.read_text(encoding="utf-8")
        assert "name: reach_goal" in yaml_text

    def test_refuses_existing_without_force(self, tmp_path: Path) -> None:
        create_scenario_package("dup_pkg", output_dir=tmp_path)
        with pytest.raises(FileExistsError):
            create_scenario_package("dup_pkg", output_dir=tmp_path)

    def test_force_overwrites(self, tmp_path: Path) -> None:
        create_scenario_package("dup_pkg", output_dir=tmp_path)
        # Should not raise when force=True.
        result = create_scenario_package("dup_pkg", output_dir=tmp_path, force=True)
        assert result.root.is_dir()
