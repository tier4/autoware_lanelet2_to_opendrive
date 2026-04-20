"""Unit tests for the pluggable scenario registry (Issue #420)."""

from __future__ import annotations

from typing import Any
from unittest.mock import MagicMock

import carla
import pytest
from omegaconf import DictConfig, OmegaConf

from autoware_carla_scenario import BaseScenario, EgoConfig, SpawnTransform


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


class _DummyConfig:
    """Minimal config class that stores kwargs as attributes."""

    def __init__(self, **kwargs: Any) -> None:
        for k, v in kwargs.items():
            setattr(self, k, v)


class _DummyScenario(BaseScenario):
    """Scenario stub for registry tests."""

    def __init__(self, ego_config: EgoConfig, **kwargs: Any) -> None:
        super().__init__(ego_config)
        self.init_kwargs = kwargs

    def setup(self) -> None:
        pass

    def is_done(self) -> bool:
        return True


def _make_ego() -> EgoConfig:
    return EgoConfig(
        spawn_location=SpawnTransform(
            carla.Transform(carla.Location(x=0.0, y=0.0, z=0.0))
        ),
        vehicle_type="vehicle.mini.cooper",
    )


# ---------------------------------------------------------------------------
# Tests for registry API
# ---------------------------------------------------------------------------


class TestScenarioRegistry:
    """Tests for register_scenario, register_scenario_builder, get_scenario_registry."""

    def test_builtin_scenarios_are_registered(self) -> None:
        """All four built-in scenarios should be present in the registry."""
        from autoware_carla_scenario.examples.run import get_scenario_registry

        registry = get_scenario_registry()
        expected = {
            "intersection_passing",
            "traffic_light_compliance",
            "lane_change",
            "temporary_stop",
        }
        assert expected.issubset(set(registry.keys()))

    def test_get_scenario_registry_returns_copy(self) -> None:
        """Mutating the returned dict must not affect the internal registry."""
        from autoware_carla_scenario.examples.run import get_scenario_registry

        copy1 = get_scenario_registry()
        copy1["__test_sentinel__"] = lambda *_: None  # type: ignore[assignment]
        copy2 = get_scenario_registry()
        assert "__test_sentinel__" not in copy2

    def test_register_scenario(self) -> None:
        """register_scenario() should create a builder in the registry."""
        from autoware_carla_scenario.examples.run import (
            _SCENARIO_REGISTRY,
            register_scenario,
        )

        name = "__test_register_scenario__"
        try:
            register_scenario(name, _DummyScenario, _DummyConfig)
            assert name in _SCENARIO_REGISTRY

            # Verify the builder works.
            ego = _make_ego()
            spawn_pose = MagicMock()
            ground_projection = MagicMock()
            scenario_dict: dict[str, Any] = {"name": name, "foo": "bar"}
            result = _SCENARIO_REGISTRY[name](
                ego, scenario_dict, spawn_pose, ground_projection
            )
            assert isinstance(result, _DummyScenario)
        finally:
            _SCENARIO_REGISTRY.pop(name, None)

    def test_register_scenario_builder(self) -> None:
        """register_scenario_builder() should accept a custom callable."""
        from autoware_carla_scenario.examples.run import (
            _SCENARIO_REGISTRY,
            register_scenario_builder,
        )

        name = "__test_register_builder__"
        sentinel = _DummyScenario(_make_ego())

        def custom_builder(
            ego: EgoConfig,
            scenario_dict: dict[str, Any],
            spawn_pose: Any,
            ground_projection: Any,
        ) -> BaseScenario:
            return sentinel

        try:
            register_scenario_builder(name, custom_builder)
            assert _SCENARIO_REGISTRY[name] is custom_builder
            result = _SCENARIO_REGISTRY[name](_make_ego(), {}, MagicMock(), MagicMock())
            assert result is sentinel
        finally:
            _SCENARIO_REGISTRY.pop(name, None)

    def test_register_overwrites_existing(self) -> None:
        """Re-registering under the same name should overwrite silently."""
        from autoware_carla_scenario.examples.run import (
            _SCENARIO_REGISTRY,
            register_scenario,
        )

        name = "__test_overwrite__"
        try:
            register_scenario(name, _DummyScenario, _DummyConfig)
            old_builder = _SCENARIO_REGISTRY[name]
            register_scenario(name, _DummyScenario, _DummyConfig)
            # Builder should be a new closure.
            assert _SCENARIO_REGISTRY[name] is not old_builder
        finally:
            _SCENARIO_REGISTRY.pop(name, None)


# ---------------------------------------------------------------------------
# Tests for build_scenario with injectable builder
# ---------------------------------------------------------------------------


class TestBuildScenarioInjectable:
    """Tests for the build_scenario_fn parameter (Option B)."""

    def test_build_scenario_fn_overrides_registry(self) -> None:
        """When build_scenario_fn is given, it should be used instead of the registry."""
        from autoware_carla_scenario.examples.run import build_scenario

        ego = _make_ego()
        sentinel = _DummyScenario(ego)

        def custom_fn(cfg: DictConfig) -> tuple[EgoConfig, BaseScenario]:
            return ego, sentinel

        cfg = OmegaConf.create({"scenario": {"name": "nonexistent"}})
        result_ego, result_scenario = build_scenario(cfg, build_scenario_fn=custom_fn)
        assert result_ego is ego
        assert result_scenario is sentinel

    def test_build_scenario_unknown_name_raises(self) -> None:
        """build_scenario should raise ValueError for unregistered names."""
        from autoware_carla_scenario.examples.run import build_scenario

        cfg = OmegaConf.create(
            {
                "scenario": {"name": "__nonexistent__"},
                "entity": {
                    "ground_projection_ray_distance_upper": 10.0,
                    "ground_projection_ray_distance_lower": 1.0,
                    "spawn_retry_max_count": 3,
                    "spawn_retry_t_step": 0.1,
                    "spawn_retry_z_step": 0.1,
                },
                "ego": {
                    "vehicle_type": "vehicle.mini.cooper",
                    "initial_speed_kmh": 0.0,
                    "spawn_lanelet_id": 1,
                    "spawn_s": 0.0,
                },
            }
        )
        with pytest.raises(ValueError, match="Unknown scenario name"):
            build_scenario(cfg)

    def test_build_scenario_unknown_name_lists_registered(self) -> None:
        """The error message should list registered scenario names."""
        from autoware_carla_scenario.examples.run import build_scenario

        cfg = OmegaConf.create(
            {
                "scenario": {"name": "__nonexistent__"},
                "entity": {
                    "ground_projection_ray_distance_upper": 10.0,
                    "ground_projection_ray_distance_lower": 1.0,
                    "spawn_retry_max_count": 3,
                    "spawn_retry_t_step": 0.1,
                    "spawn_retry_z_step": 0.1,
                },
                "ego": {
                    "vehicle_type": "vehicle.mini.cooper",
                    "initial_speed_kmh": 0.0,
                    "spawn_lanelet_id": 1,
                    "spawn_s": 0.0,
                },
            }
        )
        with pytest.raises(ValueError, match="Registered scenarios"):
            build_scenario(cfg)


# ---------------------------------------------------------------------------
# Tests for build_ego_and_spawn helper
# ---------------------------------------------------------------------------


class TestBuildEgoAndSpawn:
    """Tests for the build_ego_and_spawn() helper (Option C)."""

    def test_returns_ego_spawn_and_ground_projection(self) -> None:
        """build_ego_and_spawn should return a 3-tuple from the config."""
        from autoware_carla_scenario import (
            GroundProjectionConfig,
            Lanelet2Pose,
        )
        from autoware_carla_scenario.examples.run import build_ego_and_spawn

        cfg = OmegaConf.create(
            {
                "entity": {
                    "ground_projection_ray_distance_upper": 10.0,
                    "ground_projection_ray_distance_lower": 1.0,
                    "spawn_retry_max_count": 5,
                    "spawn_retry_t_step": 0.2,
                    "spawn_retry_z_step": 0.3,
                },
                "ego": {
                    "vehicle_type": "vehicle.tesla.model3",
                    "initial_speed_kmh": 30.0,
                    "spawn_lanelet_id": 42,
                    "spawn_s": 5.0,
                },
            }
        )
        ego, spawn_pose, ground_projection = build_ego_and_spawn(cfg)

        assert isinstance(ego, EgoConfig)
        assert ego.vehicle_type == "vehicle.tesla.model3"
        assert ego.initial_speed_kmh == 30.0

        assert isinstance(spawn_pose, Lanelet2Pose)
        assert spawn_pose.lanelet_id == 42
        assert spawn_pose.s == 5.0

        assert isinstance(ground_projection, GroundProjectionConfig)
        assert ground_projection.ray_distance_upper == 10.0
        assert ground_projection.ray_distance_lower == 1.0
