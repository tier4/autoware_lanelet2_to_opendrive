"""Unit tests for ScenarioQueue."""

from __future__ import annotations

from unittest.mock import MagicMock, patch

import pytest

from autoware_carla_scenario import (
    BaseScenario,
    EgoConfig,
    ScenarioQueue,
    ScenarioResult,
    ScenarioRunner,
    SpawnTransform,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_ego_config() -> EgoConfig:
    import carla

    return EgoConfig(
        spawn_location=SpawnTransform(carla.Transform(carla.Location(x=0, y=0, z=0))),
        vehicle_type="vehicle.mini.cooper",
    )


class _NullScenario(BaseScenario):
    """Minimal scenario: setup is a no-op, is_done always returns True."""

    def setup(self) -> None:
        pass

    def is_done(self) -> bool:
        return True


def _make_queue_with_mock_runner() -> tuple[ScenarioQueue, MagicMock]:
    """Return a ScenarioQueue whose internal runner is a MagicMock.

    The server is also mocked so no real CARLA connection is attempted.
    """
    with (
        patch(
            "autoware_carla_scenario.scenario_queue.CarlaServerManager"
        ) as mock_server_cls,
        patch(
            "autoware_carla_scenario.scenario_queue.ScenarioRunner"
        ) as mock_runner_cls,
    ):
        mock_server_cls.return_value = MagicMock()
        mock_runner = MagicMock(spec=ScenarioRunner)
        mock_runner_cls.return_value = mock_runner

        queue = ScenarioQueue(map_name="Town10HD_Opt")
        queue.start()  # sets self._runner to mock_runner

    return queue, mock_runner


# ---------------------------------------------------------------------------
# ScenarioQueue – unit tests (no CARLA required)
# ---------------------------------------------------------------------------


class TestScenarioQueueUnit:
    def test_empty_queue_len_is_zero(self) -> None:
        queue = ScenarioQueue()
        assert len(queue) == 0

    def test_add_increases_length(self) -> None:
        queue = ScenarioQueue()
        assert len(queue) == 0
        queue.add(_NullScenario(_make_ego_config()))
        assert len(queue) == 1
        queue.add(_NullScenario(_make_ego_config()))
        assert len(queue) == 2

    def test_run_all_raises_if_not_started(self) -> None:
        queue = ScenarioQueue()
        queue.add(_NullScenario(_make_ego_config()))
        with pytest.raises(RuntimeError, match="started"):
            queue.run_all()

    def test_result_for_raises_before_run_all(self) -> None:
        queue = ScenarioQueue()
        scenario = _NullScenario(_make_ego_config())
        queue.add(scenario)
        with pytest.raises(KeyError):
            queue.result_for(scenario)

    def test_run_all_calls_run_scenario_for_each(self) -> None:
        queue, mock_runner = _make_queue_with_mock_runner()
        s1 = _NullScenario(_make_ego_config())
        s2 = _NullScenario(_make_ego_config())
        queue.add(s1)
        queue.add(s2)

        expected = ScenarioResult(passed=True, message="ok", elapsed_seconds=1.0)
        mock_runner.run_scenario.return_value = expected

        results = queue.run_all()

        assert mock_runner.run_scenario.call_count == 2
        assert results == [expected, expected]

    def test_run_all_preserves_order(self) -> None:
        queue, mock_runner = _make_queue_with_mock_runner()
        scenarios = [_NullScenario(_make_ego_config()) for _ in range(3)]
        for s in scenarios:
            queue.add(s)

        expected_results = [
            ScenarioResult(passed=True, message=f"r{i}", elapsed_seconds=float(i))
            for i in range(3)
        ]
        mock_runner.run_scenario.side_effect = expected_results

        results = queue.run_all()
        assert results == expected_results

    def test_result_for_returns_correct_result(self) -> None:
        queue, mock_runner = _make_queue_with_mock_runner()
        s1 = _NullScenario(_make_ego_config())
        s2 = _NullScenario(_make_ego_config())
        queue.add(s1)
        queue.add(s2)

        r1 = ScenarioResult(passed=True, message="s1", elapsed_seconds=1.0)
        r2 = ScenarioResult(passed=False, message="s2", elapsed_seconds=2.0)
        mock_runner.run_scenario.side_effect = [r1, r2]

        queue.run_all()

        assert queue.result_for(s1) is r1
        assert queue.result_for(s2) is r2

    def test_run_all_passes_correct_scenario_to_runner(self) -> None:
        queue, mock_runner = _make_queue_with_mock_runner()
        s1 = _NullScenario(_make_ego_config())
        s2 = _NullScenario(_make_ego_config())
        queue.add(s1)
        queue.add(s2)
        mock_runner.run_scenario.return_value = ScenarioResult(
            passed=True, message="ok", elapsed_seconds=0.0
        )

        queue.run_all()

        called_scenarios = [
            call.args[0] for call in mock_runner.run_scenario.call_args_list
        ]
        assert called_scenarios[0] is s1
        assert called_scenarios[1] is s2

    def test_owns_server_flag_when_no_server_provided(self) -> None:
        with patch("autoware_carla_scenario.scenario_queue.CarlaServerManager"):
            queue = ScenarioQueue()
        assert queue._owns_server is True

    def test_borrows_server_when_provided(self) -> None:
        mock_server = MagicMock()
        queue = ScenarioQueue(server=mock_server)
        assert queue._owns_server is False
        assert queue._server is mock_server

    def test_stop_does_not_call_server_stop_when_borrowed(self) -> None:
        mock_server = MagicMock()
        with patch("autoware_carla_scenario.scenario_queue.ScenarioRunner"):
            queue = ScenarioQueue(server=mock_server)
            queue.start()
            queue.stop()

        mock_server.stop.assert_not_called()

    def test_context_manager_calls_start_and_stop(self) -> None:
        with (
            patch(
                "autoware_carla_scenario.scenario_queue.CarlaServerManager"
            ) as mock_server_cls,
            patch("autoware_carla_scenario.scenario_queue.ScenarioRunner"),
        ):
            mock_server = MagicMock()
            mock_server_cls.return_value = mock_server

            queue = ScenarioQueue()
            with queue:
                assert queue._runner is not None

            mock_server.start.assert_called_once()
            mock_server.stop.assert_called_once()
            assert queue._runner is None


# ---------------------------------------------------------------------------
# Integration tests – real CARLA (skipped if CARLA unavailable)
# ---------------------------------------------------------------------------


@pytest.mark.integration
class TestScenarioQueueIntegration:
    @pytest.fixture(autouse=True)
    def skip_if_no_carla(self, carla_queue: ScenarioQueue) -> None:
        """Require the session-scoped carla_queue fixture."""

    def test_queue_has_results_after_run(self, carla_queue: ScenarioQueue) -> None:
        """After run_all(), the queue must have non-empty results."""
        assert len(carla_queue._results) > 0

    def test_all_results_are_scenario_result_instances(
        self, carla_queue: ScenarioQueue
    ) -> None:
        for result in carla_queue._results:
            assert isinstance(result, ScenarioResult)
