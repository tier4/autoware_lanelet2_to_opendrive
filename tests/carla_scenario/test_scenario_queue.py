"""Unit tests for ScenarioQueue."""

from __future__ import annotations

from unittest.mock import MagicMock

import pytest

from autoware_carla_scenario import (
    BaseScenario,
    CarlaAutowareScenario,
    EgoConfig,
    ScenarioQueue,
    ScenarioResult,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_ego_config() -> EgoConfig:
    import carla

    return EgoConfig(
        transform=carla.Transform(carla.Location(x=0, y=0, z=0)),
        vehicle_type="vehicle.tesla.model3",
    )


class _NullScenario(BaseScenario):
    """Minimal scenario: setup is a no-op, is_done always returns True."""

    def setup(self, world: object) -> None:
        pass

    def is_done(self) -> bool:
        return True


# ---------------------------------------------------------------------------
# ScenarioQueue – unit tests (no CARLA required)
# ---------------------------------------------------------------------------


class TestScenarioQueue:
    def test_empty_queue_run_all_returns_empty_list(self) -> None:
        queue = ScenarioQueue()
        runner = MagicMock(spec=CarlaAutowareScenario)
        results = queue.run_all(runner)
        assert results == []

    def test_add_increases_length(self) -> None:
        queue = ScenarioQueue()
        assert len(queue) == 0
        queue.add(_NullScenario(_make_ego_config()))
        assert len(queue) == 1
        queue.add(_NullScenario(_make_ego_config()))
        assert len(queue) == 2

    def test_run_all_calls_run_scenario_for_each(self) -> None:
        queue = ScenarioQueue()
        s1 = _NullScenario(_make_ego_config())
        s2 = _NullScenario(_make_ego_config())
        queue.add(s1)
        queue.add(s2)

        result_pass = ScenarioResult(passed=True, message="ok", elapsed_seconds=1.0)
        runner = MagicMock(spec=CarlaAutowareScenario)
        runner.run_scenario.return_value = result_pass

        results = queue.run_all(runner)

        assert runner.run_scenario.call_count == 2
        assert results == [result_pass, result_pass]

    def test_run_all_preserves_order(self) -> None:
        queue = ScenarioQueue()
        scenarios = [_NullScenario(_make_ego_config()) for _ in range(3)]
        for s in scenarios:
            queue.add(s)

        expected_results = [
            ScenarioResult(passed=True, message=f"result {i}", elapsed_seconds=float(i))
            for i in range(3)
        ]
        runner = MagicMock(spec=CarlaAutowareScenario)
        runner.run_scenario.side_effect = expected_results

        results = queue.run_all(runner)
        assert results == expected_results

    def test_run_all_passes_correct_scenario_to_runner(self) -> None:
        queue = ScenarioQueue()
        s1 = _NullScenario(_make_ego_config())
        s2 = _NullScenario(_make_ego_config())
        queue.add(s1)
        queue.add(s2)

        runner = MagicMock(spec=CarlaAutowareScenario)
        runner.run_scenario.return_value = ScenarioResult(
            passed=True, message="ok", elapsed_seconds=0.0
        )
        queue.run_all(runner)

        calls = [call.args[0] for call in runner.run_scenario.call_args_list]
        assert calls[0] is s1
        assert calls[1] is s2


# ---------------------------------------------------------------------------
# Integration tests – real CARLA (skipped if CARLA unavailable)
# ---------------------------------------------------------------------------


class TestScenarioQueueIntegration:
    @pytest.fixture(autouse=True)
    def skip_if_no_carla(self, carla_runner: CarlaAutowareScenario) -> None:
        """Require the session-scoped carla_runner fixture."""

    def test_sequential_execution_with_real_runner(
        self, carla_runner: CarlaAutowareScenario
    ) -> None:
        """Two NullScenarios should both complete quickly and pass."""
        queue = ScenarioQueue()
        for _ in range(2):
            queue.add(_NullScenario(_make_ego_config()))

        results = queue.run_all(carla_runner)

        assert len(results) == 2
        for result in results:
            assert isinstance(result, ScenarioResult)
            assert result.passed is True
