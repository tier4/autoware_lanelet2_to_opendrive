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


# ---------------------------------------------------------------------------
# Scenario-specific configs
# ---------------------------------------------------------------------------


@dataclass
class LeftTurnConfig:
    """Parameters for the left-turn scenario."""

    name: str = "left_turn"

    #: Lanelet where the ego is spawned (must have a left turn option).
    spawn_lanelet_id: int = 203

    #: Longitudinal offset along the lanelet centerline.
    spawn_s: float = 25.0

    #: Lanelets the ego should traverse after the left turn.
    post_turn_lanelet_ids: list[int] = field(default_factory=lambda: [411, 207])

    #: Fail-safe timeout in seconds.
    timeout_seconds: float = 10.0

    #: Initial ego speed override (km/h).
    initial_speed_kmh: float = 20.0


@dataclass
class IntersectionPassingConfig:
    """Parameters for the intersection-passing scenario."""

    name: str = "intersection_passing"

    #: Lanelet where the ego is spawned.
    spawn_lanelet_id: int = 242

    #: Longitudinal offset along the lanelet centerline.
    spawn_s: float = 25.0

    #: Ordered lanelets the ego is expected to traverse.
    expected_route_lanelet_ids: list[int] = field(default_factory=lambda: [460, 265])

    #: Fail-safe timeout in seconds.
    timeout_seconds: float = 5.0

    #: Minimum speed (km/h) — scenario fails if ego drops below this.
    min_speed_kmh: float = 5.0


@dataclass
class TrafficLightComplianceConfig:
    """Parameters for the traffic-light-compliance scenario."""

    name: str = "traffic_light_compliance"

    #: Lanelet where the ego is spawned.
    spawn_lanelet_id: int = 242

    #: Longitudinal offset along the lanelet centerline.
    spawn_s: float = 15.0

    #: Delay (seconds) before switching traffic lights from red to green.
    light_switch_delay_seconds: float = 3.0

    #: Fail-safe timeout in seconds.
    timeout_seconds: float = 5.0

    #: Speed threshold (km/h) considered "moving".
    moving_speed_kmh: float = 1.0


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
        LeftTurnConfig | IntersectionPassingConfig | TrafficLightComplianceConfig
    ) = field(default_factory=LeftTurnConfig)
