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
        vehicle_type="vehicle.fuso.mitsubishi",
    )
    queue = ScenarioQueue(map_name="Town10HD_Opt")
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

from autoware_carla_scenario import (
    BaseScenario,
    EgoConfig,
    ScenarioQueue,
    SpawnPointIndex,
)

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

    Two map-loading modes are supported:

    * ``--map`` only — load a built-in CARLA map by name::

        uv run spawn-and-idle --map Town10HD_Opt

    * ``--xodr`` + ``--map`` — overwrite mode: copy *xodr* to the path given
      by the ``<MAP_NAME_PATH>`` environment variable, then load the built-in
      map (retains full CARLA assets).
      The env-var name is derived from the map name by converting CamelCase to
      ``UPPER_SNAKE_CASE_PATH`` (e.g. ``NishishinjyukuMap`` →
      ``NISHISHINJYUKU_MAP_PATH``)::

        export NISHISHINJYUKU_MAP_PATH=/opt/carla/.../NishishinjyukuMap.xodr
        uv run spawn-and-idle --xodr /path/to/my.xodr --map NishishinjyukuMap
    """
    parser = argparse.ArgumentParser(
        description="Run the SpawnAndIdle example scenario against a running CARLA server.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--host", default="localhost", help="CARLA server host (default: localhost)"
    )
    parser.add_argument(
        "--port", type=int, default=2000, help="CARLA server port (default: 2000)"
    )
    parser.add_argument(
        "--map",
        required=True,
        help="Built-in CARLA map name (e.g. Town10HD_Opt).",
    )
    parser.add_argument(
        "--xodr",
        type=Path,
        default=None,
        help=(
            "Path to an OpenDRIVE (.xodr) file. "
            "Overwrites the built-in map's .xodr before loading."
        ),
    )
    parser.add_argument(
        "--vehicle",
        default="vehicle.fuso.mitsubishi",
        help="CARLA blueprint ID for the ego vehicle (default: vehicle.fuso.mitsubishi)",
    )
    parser.add_argument(
        "--spawn-index",
        type=int,
        default=0,
        help="Index into the map's spawn point list (default: 0)",
    )

    args = parser.parse_args()

    ego = EgoConfig(
        spawn_location=SpawnPointIndex(args.spawn_index),
        vehicle_type=args.vehicle,
    )

    queue = ScenarioQueue(
        host=args.host,
        port=args.port,
        xodr_path=args.xodr,
        map_name=args.map,
    )
    queue.add(SpawnAndIdleScenario(ego))

    with queue:
        results = queue.run_all()

    result = results[0]
    print(result)
    sys.exit(0 if result.passed else 1)


if __name__ == "__main__":
    main()
