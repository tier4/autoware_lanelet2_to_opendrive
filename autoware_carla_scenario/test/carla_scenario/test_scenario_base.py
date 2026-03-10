"""Unit tests for BaseScenario callback registration and ordering."""

from __future__ import annotations

from typing import List, Optional
from unittest.mock import MagicMock

import pytest

from autoware_carla_scenario import (
    BaseCondition,
    BaseScenario,
    EgoConfig,
    ScenarioResult,
    SpawnTransform,
)


# ---------------------------------------------------------------------------
# Minimal concrete scenario for testing
# ---------------------------------------------------------------------------


class _SimpleScenario(BaseScenario):
    """Concrete scenario that records callback invocation order."""

    def __init__(self, ego_config: EgoConfig) -> None:
        super().__init__(ego_config)
        self.setup_called = False
        self.call_log: List[str] = []

    def setup(self, world: object) -> None:
        self.setup_called = True

    def is_done(self) -> bool:
        return False


class _CountingCondition(BaseCondition):
    """Records how many times check() is called before returning a result."""

    def __init__(self, trigger_after: int, passed: bool) -> None:
        self.trigger_after = trigger_after
        self.passed = passed
        self.call_count = 0

    def check(self, world: object, elapsed: float) -> Optional[ScenarioResult]:
        self.call_count += 1
        if self.call_count >= self.trigger_after:
            return ScenarioResult(
                passed=self.passed,
                message="triggered",
                elapsed_seconds=elapsed,
            )
        return None


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_ego_config() -> EgoConfig:
    import carla

    return EgoConfig(
        spawn_location=SpawnTransform(carla.Transform(carla.Location(x=0, y=0, z=0))),
        vehicle_type="vehicle.fuso.mitsubishi",
    )


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


class TestBaseScenario:
    def test_setup_is_called(self) -> None:
        scenario = _SimpleScenario(_make_ego_config())
        world = MagicMock()
        scenario.setup(world)
        assert scenario.setup_called

    def test_register_pre_tick_appends_callback(self) -> None:
        scenario = _SimpleScenario(_make_ego_config())
        cb = MagicMock()
        scenario.register_pre_tick(cb)
        assert cb in scenario._pre_tick_callbacks

    def test_register_post_tick_appends_callback(self) -> None:
        scenario = _SimpleScenario(_make_ego_config())
        cb = MagicMock()
        scenario.register_post_tick(cb)
        assert cb in scenario._post_tick_callbacks

    def test_multiple_pre_tick_callbacks_ordered(self) -> None:
        scenario = _SimpleScenario(_make_ego_config())
        order: List[int] = []
        scenario.register_pre_tick(lambda w: order.append(1))
        scenario.register_pre_tick(lambda w: order.append(2))
        world = MagicMock()
        for cb in scenario._pre_tick_callbacks:
            cb(world)
        assert order == [1, 2]

    def test_register_pass_condition(self) -> None:
        scenario = _SimpleScenario(_make_ego_config())
        cond = _CountingCondition(trigger_after=1, passed=True)
        scenario.register_pass_condition(cond)
        assert cond in scenario._pass_conditions

    def test_register_fail_condition(self) -> None:
        scenario = _SimpleScenario(_make_ego_config())
        cond = _CountingCondition(trigger_after=1, passed=False)
        scenario.register_fail_condition(cond)
        assert cond in scenario._fail_conditions

    def test_ego_config_stored(self) -> None:
        cfg = _make_ego_config()
        scenario = _SimpleScenario(cfg)
        assert scenario.ego_config is cfg

    def test_abstract_class_cannot_be_instantiated(self) -> None:
        with pytest.raises(TypeError):
            BaseScenario(_make_ego_config())  # type: ignore[abstract]

    def test_counting_condition_triggers_after_n_calls(self) -> None:
        cond = _CountingCondition(trigger_after=3, passed=True)
        world = MagicMock()
        assert cond.check(world, 0.0) is None
        assert cond.check(world, 0.5) is None
        result = cond.check(world, 1.0)
        assert result is not None
        assert result.passed is True
