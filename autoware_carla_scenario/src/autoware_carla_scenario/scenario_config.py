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
    "DriverCameraSpec",
    "DriverControlSpec",
    "DriverConfig",
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

    #: Which ego entity drives the vehicle.
    #:
    #: * ``"autopilot"`` -- CARLA's TrafficManager (default).
    #: * ``"autoware"`` -- no driver; the actor is left for an external stack.
    #: * ``"carla_driver"`` -- an external policy over the ``egodriver`` gRPC
    #:   contract, configured by the ``driver`` config group.
    entity: str = "autopilot"


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
# External driver config (for ego.entity == "carla_driver")
# ---------------------------------------------------------------------------


@dataclass
class DriverCameraSpec:
    """One camera streamed to the driver policy.

    ``logical_id`` is the name the policy looks the camera up by, so it must match what
    the policy expects.  Extrinsics use CARLA's convention relative to ``base_link``
    (x forward, y right, z up; angles in degrees).
    """

    logical_id: str = "camera_front_wide_120fov"
    image_width: int = 960
    image_height: int = 604
    fov: float = 120.0
    fps: float = 20.0
    position_x: float = 1.5
    position_y: float = 0.0
    position_z: float = 1.6
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0


@dataclass
class DriverControlSpec:
    """Gains for the controller that tracks the policy's plan.

    Mirrors :class:`~autoware_carla_scenario.driver.control.ControlConfig`; see that
    class for the meaning of each field.  ``max_steer_angle_deg`` is expressed in
    degrees here for readability and converted on the way in.
    """

    lookahead_gain_s: float = 0.9
    min_lookahead_m: float = 4.0
    max_lookahead_m: float = 20.0
    wheelbase_m: float = 2.8
    max_steer_angle_deg: float = 70.0
    max_steer_rate: float = 4.0
    speed_kp: float = 0.6
    speed_ki: float = 0.15
    speed_kd: float = 0.05
    integral_limit: float = 1.0
    stop_speed_mps: float = 0.2
    stop_brake: float = 0.6


@dataclass
class DriverConfig:
    """Connection settings for an external driver policy.

    Only used when ``ego.entity`` is ``"carla_driver"``.  The policy is expected to
    serve ``egodriver.EgodriverService`` at :attr:`address` -- for example
    ``carla-driver-interface serve --policy route_follower --port 50051``.
    """

    #: ``host:port`` of the policy's gRPC server.
    address: str = "localhost:50051"

    #: Per-RPC deadline in seconds.
    timeout_s: float = 60.0

    #: Interval between ``drive`` calls.  Must be a positive integer multiple of the
    #: 0.05 s simulation step.
    policy_timestep_s: float = 0.1

    #: JPEG quality (1-100) for streamed camera frames.
    image_quality: int = 90

    #: How far ahead the submitted route extends, in metres.
    route_horizon_m: float = 80.0

    #: Spacing between route waypoints, in metres.
    route_resolution_m: float = 2.0

    #: Offset from the CARLA actor origin back to the rig origin, in metres.
    #: ``null`` derives it from the vehicle's wheel physics.
    rear_axle_offset_m: float | None = None

    #: Whether to also submit the recorded ground-truth trajectory.
    send_ground_truth: bool = False

    #: Seed handed to the policy in ``start_session``.
    random_seed: int = 0

    #: Cameras streamed to the policy.
    cameras: list[DriverCameraSpec] = field(
        default_factory=lambda: [DriverCameraSpec()]
    )

    #: Trajectory-following gains.
    control: DriverControlSpec = field(default_factory=DriverControlSpec)


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
