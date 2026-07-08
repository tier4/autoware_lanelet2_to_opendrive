"""Public, reusable configuration dataclasses for scenario packages.

These dataclasses describe the parameters that are **shared** by every
scenario regardless of which package defines it: the CARLA server connection,
the target map, the ego vehicle, per-entity spawn behaviour, NPC vehicles, and
the optional lanelet-constraint sweep section.

They intentionally live at the top level of :mod:`autoware_carla_scenario`
(rather than under ``examples``) so that **external scenario packages** can
import them as a stable public API::

    from autoware_carla_scenario import (
        EgoVehicleConfig,
        MapConfig,
        NpcVehicleConfig,
        ServerConfig,
    )

Scenario-specific dataclasses (e.g. the parameters of a single test) should be
defined by each scenario package itself -- see
:mod:`autoware_carla_scenario.examples.configs` for the built-in examples.

This module has **no heavy dependencies** (no CARLA, no lanelet2), so importing
it is cheap and safe from any environment.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from omegaconf import MISSING

__all__ = [
    "ServerConfig",
    "MapConfig",
    "EntityConfig",
    "EgoVehicleConfig",
    "NpcVehicleConfig",
    "SweepConfig",
]


# ---------------------------------------------------------------------------
# Common / shared configs
# ---------------------------------------------------------------------------


@dataclass
class ServerConfig:
    """CARLA server connection parameters."""

    host: str = "localhost"
    port: int = 2000

    #: Cooldown (seconds) between consecutive scenario runs.  Gives the
    #: CARLA server time to finish cleanup (destroy actors, restore settings)
    #: before the next scenario connects.  0 disables the cooldown.
    cooldown_seconds: float = 3.0

    #: Maximum number of retries when a scenario run fails after cooldown
    #: (e.g. due to CARLA communication errors or initialization failures).
    #: 0 means no retries -- a failure is immediately propagated.
    cooldown_max_retries: int = 0


@dataclass
class MapConfig:
    """Map selection and optional file overrides."""

    #: Built-in CARLA map name (e.g. ``Town10HD_Opt``).  **Required**.
    name: str = MISSING

    #: Optional path to a custom OpenDRIVE file that overwrites the built-in map.
    xodr_path: str | None = None

    #: Optional path to a Lanelet2 (.osm) file for coordinate transforms.
    lanelet2_path: str | None = None


@dataclass
class EntityConfig:
    """Shared entity parameters for ground projection and spawn retry.

    These settings apply to all vehicle entities (ego and NPC).
    """

    #: Search range (m) above the estimated z for the ground projection ray.
    ground_projection_ray_distance_upper: float = 5.0

    #: Search range (m) below the estimated z for the ground projection ray.
    ground_projection_ray_distance_lower: float = 5.0

    #: Maximum number of upward-shift retries when the initial spawn fails.
    #: 0 disables retries.
    spawn_retry_max_count: int = 10

    #: Upward shift (metres) per retry attempt when the initial spawn fails.
    spawn_retry_t_step: float = 0.1

    #: Vertical shift (metres) per retry attempt.
    spawn_retry_z_step: float = 0.5


@dataclass
class EgoVehicleConfig:
    """Ego vehicle parameters."""

    vehicle_type: str = "vehicle.mini.cooper"
    initial_speed_kmh: float = 0.0

    #: Lanelet where the ego is spawned.
    spawn_lanelet_id: int = 242

    #: Longitudinal offset along the lanelet centerline.
    spawn_s: float = 25.0


@dataclass
class NpcVehicleConfig:
    """Configuration for a single NPC vehicle in a scenario."""

    #: Lanelet where the NPC is spawned.
    spawn_lanelet_id: int = MISSING

    #: Longitudinal offset along the lanelet centerline.
    spawn_s: float = 0.0

    #: CARLA vehicle blueprint ID.
    vehicle_type: str = "vehicle.mini.cooper"

    #: Initial speed in km/h applied after warm-up.
    initial_speed_kmh: float = 0.0


# ---------------------------------------------------------------------------
# Sweep config (for lanelet-constraint sweeper)
# ---------------------------------------------------------------------------


@dataclass
class SweepConfig:
    """Optional sweep section for lanelet-constraint-based multirun.

    ``constraints`` maps a target key (e.g. ``ego.spawn_lanelet_id``) to a
    list of constraint dicts.  ``bindings`` maps a target key
    (e.g. ``ego.spawn_s``) to a binding dict that auto-derives the value.
    """

    constraints: dict[str, Any] = field(default_factory=dict)
    bindings: dict[str, Any] = field(default_factory=dict)

    #: Hard timeout (seconds) per job.  If a single scenario run exceeds
    #: this duration (e.g. CARLA hangs), it is forcefully interrupted and
    #: the sweep continues with the next lanelet.  0 disables the timeout.
    job_timeout_seconds: int = 120

    #: 1-indexed job number to resume from.  Jobs before this index are
    #: skipped.  0 (default) means execute all jobs from the beginning.
    resume_from: int = 0
