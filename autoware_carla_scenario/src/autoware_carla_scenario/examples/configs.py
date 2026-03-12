"""Hydra-compatible dataclass definitions for scenario parameters.

Each dataclass corresponds to a YAML config group and provides typed,
documented defaults.  Hydra's ``structured configs`` feature maps the
YAML values directly to these dataclasses via OmegaConf.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from omegaconf import MISSING


# ---------------------------------------------------------------------------
# Common / shared configs
# ---------------------------------------------------------------------------


@dataclass
class ServerConfig:
    """CARLA server connection parameters."""

    host: str = "localhost"
    port: int = 2000


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
    scenario: (
        IntersectionPassingConfig
        | LaneChangeConfig
        | TrafficLightComplianceConfig
        | TemporaryStopConfig
    ) = field(default_factory=IntersectionPassingConfig)
