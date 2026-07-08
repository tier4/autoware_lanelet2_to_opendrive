"""Unit tests for the scenario-package extension API.

These tests exercise :mod:`autoware_carla_scenario.registry` directly.  They
deliberately avoid importing CARLA (or any ``BaseScenario`` subclass) so that
they run in any environment: the registry only stores callables and paths, so
plain stand-ins are sufficient.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any
from unittest.mock import MagicMock

import autoware_carla_scenario.registry as registry
from autoware_carla_scenario import (
    get_conf_dirs,
    get_scenario_builder,
    get_scenario_registry,
    register_conf_dir,
    register_scenario,
    register_scenario_builder,
    unregister_conf_dir,
    unregister_scenario,
)


class _StoreKwargs:
    """Stand-in scenario/config class that records its keyword arguments."""

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        self.args = args
        self.kwargs = kwargs


# ---------------------------------------------------------------------------
# Scenario registry
# ---------------------------------------------------------------------------


class TestScenarioRegistry:
    def test_register_scenario_builds_with_config(self) -> None:
        name = "__pkg_test_reach__"
        try:
            register_scenario(name, _StoreKwargs, _StoreKwargs)  # type: ignore[arg-type]
            builder = get_scenario_registry()[name]

            ego = MagicMock()
            spawn_pose = MagicMock()
            ground_projection = MagicMock()
            scenario = builder(
                ego, {"name": name, "foo": 1}, spawn_pose, ground_projection
            )

            assert isinstance(scenario, _StoreKwargs)
            # ego is passed positionally, the rest as keywords.
            assert scenario.args == (ego,)
            assert scenario.kwargs["spawn_pose"] is spawn_pose
            assert scenario.kwargs["ground_projection"] is ground_projection
            # The config was constructed from the scenario dict.
            assert isinstance(scenario.kwargs["config"], _StoreKwargs)
            assert scenario.kwargs["config"].kwargs == {"name": name, "foo": 1}
        finally:
            unregister_scenario(name)

    def test_register_scenario_builder_custom(self) -> None:
        name = "__pkg_test_builder__"
        sentinel = object()
        try:
            register_scenario_builder(name, lambda *_: sentinel)  # type: ignore[arg-type,return-value]
            builder = get_scenario_builder(name)
            assert builder is not None
            assert builder(None, {}, None, None) is sentinel  # type: ignore[arg-type]
        finally:
            unregister_scenario(name)

    def test_get_scenario_registry_returns_copy(self) -> None:
        copy1 = get_scenario_registry()
        copy1["__sentinel__"] = lambda *_: None  # type: ignore[assignment]
        assert "__sentinel__" not in get_scenario_registry()


# ---------------------------------------------------------------------------
# Config directory registry
# ---------------------------------------------------------------------------


class TestConfDirRegistry:
    def test_register_conf_dir_dedupes(self, tmp_path: Path) -> None:
        try:
            register_conf_dir(tmp_path)
            register_conf_dir(tmp_path)  # duplicate must be ignored
            dirs = get_conf_dirs()
            assert dirs.count(tmp_path.resolve()) == 1
            assert tmp_path.resolve() in dirs
        finally:
            unregister_conf_dir(tmp_path)

    def test_unregister_conf_dir(self, tmp_path: Path) -> None:
        register_conf_dir(tmp_path)
        assert tmp_path.resolve() in get_conf_dirs()
        unregister_conf_dir(tmp_path)
        assert tmp_path.resolve() not in get_conf_dirs()
        # Unregistering an unknown dir is a no-op.
        unregister_conf_dir(tmp_path)

    def test_get_conf_dirs_returns_copy(self, tmp_path: Path) -> None:
        dirs = get_conf_dirs()
        dirs.append(tmp_path)
        assert tmp_path not in get_conf_dirs()


# ---------------------------------------------------------------------------
# Entry-point plugin discovery
# ---------------------------------------------------------------------------


class TestLoadScenarioPlugins:
    def test_load_is_idempotent_and_calls_entry_points(self, monkeypatch: Any) -> None:
        calls: list[str] = []

        fake_ep = MagicMock()
        fake_ep.name = "fake"
        fake_ep.value = "fake:register"
        fake_ep.load.return_value = lambda: calls.append("registered")

        def fake_entry_points(*, group: str) -> list[Any]:
            assert group == registry.SCENARIO_ENTRY_POINT_GROUP
            return [fake_ep]

        monkeypatch.setattr("importlib.metadata.entry_points", fake_entry_points)
        # Force a clean load state for this test.
        monkeypatch.setattr(registry, "_plugins_loaded", False)

        registry.load_scenario_plugins()
        registry.load_scenario_plugins()  # second call is a no-op

        assert calls == ["registered"]

    def test_load_survives_a_broken_plugin(self, monkeypatch: Any) -> None:
        bad_ep = MagicMock()
        bad_ep.name = "bad"
        bad_ep.load.side_effect = RuntimeError("boom")

        monkeypatch.setattr(
            "importlib.metadata.entry_points",
            lambda *, group: [bad_ep],
        )
        monkeypatch.setattr(registry, "_plugins_loaded", False)

        # Must not raise even though the plugin's loader blows up.
        registry.load_scenario_plugins()
