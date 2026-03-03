"""Example scenario: spawn the ego vehicle and idle for a few seconds.

This module is the minimal starting point for writing your own scenario.
Copy it, rename the class, and fill in ``setup`` and ``is_done``.

Typical usage
-------------
Standalone (no pytest)::

    import carla
    from autoware_carla_scenario import EgoConfig, ScenarioQueue
    from autoware_carla_scenario.examples.spawn_and_idle import SpawnAndIdleScenario

    ego = EgoConfig(
        transform=carla.Transform(carla.Location(x=0.0, y=0.0, z=0.5)),
        vehicle_type="vehicle.tesla.model3",
    )
    queue = ScenarioQueue(map_name="Town01")
    queue.add(SpawnAndIdleScenario(ego))

    with queue:
        results = queue.run_all()

    print(results[0])  # ScenarioResult(passed=True, ...)

With pytest — see ``test/carla_scenario/test_examples.py``.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import TYPE_CHECKING

import carla as _carla

from autoware_carla_scenario import BaseScenario, EgoConfig, ScenarioQueue

if TYPE_CHECKING:
    import carla


class SpawnAndIdleScenario(BaseScenario):
    """Spawns the ego vehicle and idles for approximately 2 seconds.

    This is intentionally the simplest possible scenario:

    * ``setup`` does nothing — no extra actors, no callbacks.
    * ``is_done`` counts ticks and returns ``True`` after
      :attr:`DONE_AFTER_TICKS` ticks (≈ 2 s at the default 20 Hz).
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

    def __init__(self, ego_config: EgoConfig) -> None:
        super().__init__(ego_config)
        self._ticks: int = 0

    def setup(self, world: "carla.World") -> None:
        """No additional actors needed for this minimal scenario."""

    def is_done(self) -> bool:
        """Return ``True`` after :attr:`DONE_AFTER_TICKS` ticks."""
        self._ticks += 1
        return self._ticks >= self.DONE_AFTER_TICKS


def main() -> None:
    """Run SpawnAndIdleScenario as a standalone script.

    Example::

        uv run spawn-and-idle
        uv run spawn-and-idle --map Town02
        uv run spawn-and-idle --xodr /path/to/map.xodr
        uv run spawn-and-idle --x 50.0 --y -20.0 --z 0.5
    """
    parser = argparse.ArgumentParser(
        description="Run the SpawnAndIdle example scenario against a running CARLA server."
    )
    parser.add_argument(
        "--host", default="localhost", help="CARLA server host (default: localhost)"
    )
    parser.add_argument(
        "--port", type=int, default=2000, help="CARLA server port (default: 2000)"
    )

    map_group = parser.add_mutually_exclusive_group()
    map_group.add_argument(
        "--map", default="Town01", help="Built-in CARLA map name (default: Town01)"
    )
    map_group.add_argument(
        "--xodr", type=Path, default=None, help="Path to an OpenDRIVE (.xodr) map file"
    )

    parser.add_argument(
        "--vehicle",
        default="vehicle.tesla.model3",
        help="CARLA blueprint ID for the ego vehicle (default: vehicle.tesla.model3)",
    )
    parser.add_argument(
        "--x", type=float, default=0.0, help="Ego spawn X (default: 0.0)"
    )
    parser.add_argument(
        "--y", type=float, default=0.0, help="Ego spawn Y (default: 0.0)"
    )
    parser.add_argument(
        "--z", type=float, default=0.5, help="Ego spawn Z (default: 0.5)"
    )

    args = parser.parse_args()

    ego = EgoConfig(
        transform=_carla.Transform(_carla.Location(x=args.x, y=args.y, z=args.z)),
        vehicle_type=args.vehicle,
    )

    queue = ScenarioQueue(
        host=args.host,
        port=args.port,
        xodr_path=args.xodr,
        map_name=None if args.xodr else args.map,
    )
    queue.add(SpawnAndIdleScenario(ego))

    with queue:
        results = queue.run_all()

    result = results[0]
    print(result)
    sys.exit(0 if result.passed else 1)


if __name__ == "__main__":
    main()
