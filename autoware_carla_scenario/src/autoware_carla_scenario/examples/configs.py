"""Hydra-compatible dataclass definitions for scenario parameters.

Each dataclass corresponds to a YAML config group and provides typed,
documented defaults.  Hydra's ``structured configs`` feature maps the
YAML values directly to these dataclasses via OmegaConf.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from omegaconf import MISSING


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
    #: 0 means no retries — a failure is immediately propagated.
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


# ---------------------------------------------------------------------------
# Scenario-specific configs
# ---------------------------------------------------------------------------


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


@dataclass
class IntersectionPassingConfig:
    """Parameters for the intersection-passing scenario.

    Supports straight-through, left-turn, and right-turn variants via the
    optional ``turn_direction`` field.
    """

    name: str = "intersection_passing"

    #: Ordered lanelets the ego is expected to traverse.
    expected_route_lanelet_ids: list[int] = field(default_factory=lambda: [460, 265])

    #: Fail-safe timeout in seconds.
    timeout_seconds: float = 5.0

    #: Minimum speed (km/h) — scenario fails if ego drops below this.
    #: ``None`` disables the speed check.
    min_speed_kmh: float | None = None

    #: Grace period (seconds) before the minimum-speed check activates.
    #: Gives the ego time to reach cruising speed after spawn.
    speed_check_delay_seconds: float = 0.3

    #: Turn direction at the intersection (``"left"``, ``"right"``, or
    #: ``None`` for straight-through).
    turn_direction: str | None = None

    #: Optional list of NPC vehicles to spawn in the scenario.
    #: Empty by default for backwards compatibility with existing YAML configs.
    npc_vehicles: list[NpcVehicleConfig] = field(default_factory=list)

    def __post_init__(self) -> None:
        """Convert raw dicts from OmegaConf into NpcVehicleConfig instances."""
        self.npc_vehicles = [
            NpcVehicleConfig(**v) if isinstance(v, dict) else v
            for v in self.npc_vehicles
        ]


@dataclass
class TrafficLightComplianceConfig:
    """Parameters for the traffic-light-compliance scenario."""

    name: str = "traffic_light_compliance"

    #: Delay (seconds) before switching traffic lights from red to green.
    light_switch_delay_seconds: float = 3.0

    #: Fail-safe timeout in seconds.
    timeout_seconds: float = 5.0

    #: Speed threshold (km/h) considered "moving".
    moving_speed_kmh: float = 1.0

    #: Margin (seconds) subtracted from the red phase duration when computing
    #: the required standstill time.
    merging_time_seconds: float = 0.5


@dataclass
class LaneChangeConfig:
    """Parameters for the lane-change scenario."""

    name: str = "lane_change"

    #: Direction of the lane change (``"left"`` or ``"right"``).
    direction: str = "left"

    #: Expected outcome: ``"success"`` expects the lane change to complete,
    #: ``"fail"`` expects it to NOT complete (scenario passes if lane change
    #: does not happen within the timeout).
    expect: str = "success"

    #: Fail-safe timeout in seconds.
    timeout_seconds: float = 10.0


@dataclass
class TemporaryStopConfig:
    """Parameters for the temporary-stop scenario."""

    name: str = "temporary_stop"

    #: Arc-length margin (m) around the stop position.
    s_margin: float = 5.0

    #: Maximum speed (m/s) considered as stopped.
    speed_threshold: float = 0.1

    #: Minimum consecutive seconds the entity must remain stopped.
    stop_duration: float = 1.0

    #: Speed (km/h) above which the ego is considered to have restarted.
    restart_speed_kmh: float = 3.0

    #: Fail-safe timeout in seconds.
    timeout_seconds: float = 30.0


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


# ---------------------------------------------------------------------------
# Top-level Hydra config
# ---------------------------------------------------------------------------


@dataclass
class ScenarioRunConfig:
    """Root configuration assembled by Hydra.

    ``defaults`` in ``conf/config.yaml`` populate the ``server``, ``map``,
    ``ego``, and ``scenario`` sub-trees automatically.
    """

    server: ServerConfig = field(default_factory=ServerConfig)
    map: MapConfig = field(default_factory=MapConfig)
    ego: EgoVehicleConfig = field(default_factory=EgoVehicleConfig)
    entity: EntityConfig = field(default_factory=EntityConfig)
    scenario: (
        IntersectionPassingConfig
        | LaneChangeConfig
        | TrafficLightComplianceConfig
        | TemporaryStopConfig
    ) = field(default_factory=IntersectionPassingConfig)
    sweep: SweepConfig = field(default_factory=SweepConfig)
