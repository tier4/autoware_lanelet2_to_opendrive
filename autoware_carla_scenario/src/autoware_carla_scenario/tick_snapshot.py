"""Immutable per-tick snapshot of simulation state.

Every component in the tick loop (actions, conditions, callbacks) receives the
same :class:`TickSnapshot` instance, guaranteeing a consistent view of time and
world state within a single simulation step.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import carla


@dataclass(frozen=True)
class TickSnapshot:
    """Frozen snapshot of simulation state for one tick.

    Attributes:
        world: The CARLA world instance for the current tick.
        elapsed: Wall-clock seconds elapsed since the tick loop started.
        tick_count: Number of ticks completed since the tick loop started
            (1-based: first tick is 1).
        delta_time: Seconds between this tick and the previous one.  For the
            very first tick this equals *elapsed*.
    """

    world: "carla.World"
    elapsed: float
    tick_count: int
    delta_time: float
