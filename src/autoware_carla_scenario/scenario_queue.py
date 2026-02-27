"""Sequential scenario queue for running multiple scenarios."""

from __future__ import annotations

from typing import TYPE_CHECKING, List

from .conditions import ScenarioResult
from .scenario_base import BaseScenario

if TYPE_CHECKING:
    from .carla_autoware_scenario import CarlaAutowareScenario


class ScenarioQueue:
    """Collects scenarios and runs them sequentially via a :class:`CarlaAutowareScenario`.

    Example::

        queue = ScenarioQueue()
        queue.add(MyScenario(ego_config))
        queue.add(AnotherScenario(ego_config))
        results = queue.run_all(runner)
    """

    def __init__(self) -> None:
        self._scenarios: List[BaseScenario] = []

    def add(self, scenario: BaseScenario) -> None:
        """Append a scenario to the queue.

        Args:
            scenario: A scenario instance to enqueue.
        """
        self._scenarios.append(scenario)

    def run_all(self, runner: "CarlaAutowareScenario") -> List[ScenarioResult]:
        """Execute every scenario in order and collect results.

        Args:
            runner: The :class:`CarlaAutowareScenario` that executes each scenario.

        Returns:
            A list of :class:`ScenarioResult` objects in the same order as the
            enqueued scenarios.
        """
        results: List[ScenarioResult] = []
        for scenario in self._scenarios:
            result = runner.run_scenario(scenario)
            results.append(result)
        return results

    def __len__(self) -> int:
        return len(self._scenarios)
