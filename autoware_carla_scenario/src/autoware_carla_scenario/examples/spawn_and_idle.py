"""Example scenario: spawn the ego vehicle and idle for a few seconds.

This module is the minimal starting point for writing your own scenario.
Copy it, rename the class, and fill in ``setup`` and ``is_done``.

Typical usage
-------------
Standalone (no pytest)::

    from autoware_carla_scenario import EgoConfig, ScenarioQueue, SpawnPointIndex
    from autoware_carla_scenario.examples.spawn_and_idle import SpawnAndIdleScenario

    ego = EgoConfig(
        spawn_location=SpawnPointIndex(0),
        vehicle_type="vehicle.mini.cooper",
    )
    queue = ScenarioQueue(map_name="Town10HD_Opt")
    queue.add(SpawnAndIdleScenario(ego))

    with queue:
        results = queue.run_all()

    print(results[0])  # ScenarioResult(passed=True, ...)

With pytest — see ``test/carla_scenario/test_examples.py``.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from autoware_carla_scenario import (
    BaseScenario,
    EgoConfig,
)

from .configs import SpawnAndIdleConfig

if TYPE_CHECKING:
    import carla


class SpawnAndIdleScenario(BaseScenario):
    """Spawns the ego vehicle and idles for approximately 2 seconds.

    This is intentionally the simplest possible scenario:

    * ``setup`` does nothing — no extra actors, no callbacks.
    * ``is_done`` counts ticks and returns ``True`` after the configured
      number of ticks (≈ 2 s at the default 20 Hz).
    * The runner automatically registers a
      :class:`~autoware_carla_scenario.TimeoutCondition` as a fail-safe,
      so the scenario will never hang indefinitely.

    Use this as a template when writing your own scenario::

        class MyScenario(BaseScenario):
            def setup(self, world: carla.World) -> None:
                # Spawn NPCs, set weather, register callbacks, …
                ...

            def is_done(self) -> bool:
                # Return True when the scenario has reached its end state.
                ...
    """

    #: Number of simulation ticks before the scenario ends.
    #: At the default 20 Hz fixed timestep this equals roughly 2 seconds.
    DONE_AFTER_TICKS: int = 40

    def __init__(
        self, ego_config: EgoConfig, config: SpawnAndIdleConfig | None = None
    ) -> None:
        super().__init__(ego_config)
        self._config = config or SpawnAndIdleConfig()
        self._ticks: int = 0

    def setup(self, world: "carla.World") -> None:
        """No additional actors needed for this minimal scenario."""

    def is_done(self) -> bool:
        """Return ``True`` after the configured number of ticks."""
        self._ticks += 1
        return self._ticks >= self._config.done_after_ticks
