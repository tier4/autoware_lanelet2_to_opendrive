"""Configuration and the abstract client for an external ego driver.

The scenario framework plays the *runtime* role of the alpasim contract: it owns the
world, renders the observations, and asks a driver policy what to do next.  The policy
runs elsewhere -- typically as a
`carla_driver_interface <https://github.com/hakuturu583/carla_driver_interface>`_ gRPC
server -- and this module defines the seam between the two.

:class:`BaseEgoDriverClient` is deliberately transport-agnostic so tests can substitute a
fake, mirroring how :class:`~autoware_carla_scenario.sensor.base.CameraSensorBase`
abstracts the simulator behind the camera.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field, fields
from typing import Any, Dict, List, Mapping, Optional, Tuple

import numpy as np
from numpy.typing import NDArray

from ..sensor.carla_camera import CarlaCameraSensorConfig
from .geometry import Pose, Trajectory


__all__ = [
    "BaseEgoDriverClient",
    "DriveOutcome",
    "DriverCameraConfig",
    "DriverClientConfig",
    "EgoObservation",
]

#: Logical camera id used by alpasim's reference rig for the forward-facing camera.
#: Policies look their inputs up by this string, so it must match what the policy expects.
DEFAULT_CAMERA_LOGICAL_ID: str = "camera_front_wide_120fov"


def _checked(
    config_cls: type, mapping: Mapping[str, Any], ignore: Optional[set] = None
) -> dict:
    """Return *mapping* as a dict, rejecting keys *config_cls* does not define.

    A silently dropped key is the failure mode this guards against: a typo in a YAML
    override would otherwise leave the default in place with no indication.

    Raises:
        ValueError: If *mapping* holds an unknown key.
    """
    known = {field.name for field in fields(config_cls)}
    skip = ignore or set()
    unknown = sorted(set(mapping) - known - skip)
    if unknown:
        raise ValueError(
            f"Unknown {config_cls.__name__} key(s): {unknown}. "
            f"Known keys: {sorted(known)}"
        )
    return {key: value for key, value in mapping.items() if key not in skip}


@dataclass(frozen=True)
class DriverCameraConfig:
    """One camera exposed to the driver policy.

    Pairs the alpasim *logical id* a policy addresses the camera by with the CARLA
    camera parameters used to render it.  Defaults match the front-wide camera of
    ``carla_driver_interface``'s default rig.

    Extrinsics are relative to the vehicle's ``base_link``, in CARLA's convention
    (x forward, y right, z up, degrees).
    """

    logical_id: str = DEFAULT_CAMERA_LOGICAL_ID
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

    @classmethod
    def from_mapping(cls, mapping: Mapping[str, Any]) -> "DriverCameraConfig":
        """Return a camera config built from a plain mapping (e.g. a Hydra node).

        Raises:
            ValueError: If *mapping* holds a key this config does not define.
        """
        return cls(**_checked(cls, mapping))

    def to_sensor_config(self) -> CarlaCameraSensorConfig:
        """Return the CARLA sensor configuration for this camera."""
        return CarlaCameraSensorConfig(
            image_width=self.image_width,
            image_height=self.image_height,
            fov=self.fov,
            fps=self.fps,
            position_x=self.position_x,
            position_y=self.position_y,
            position_z=self.position_z,
            roll=self.roll,
            pitch=self.pitch,
            yaw=self.yaw,
        )


@dataclass(frozen=True)
class DriverClientConfig:
    """Connection and cadence settings for an external driver policy."""

    address: str = "localhost:50051"
    """``host:port`` of the driver's gRPC server."""

    timeout_s: float = 60.0
    """Per-RPC deadline in seconds."""

    policy_timestep_s: float = 0.1
    """Interval between ``drive`` calls.

    Must be a positive integer multiple of the simulation step (0.05 s at CARLA's
    20 Hz), matching ``RuntimeConfig.policy_timestep_s`` upstream.  Between policy
    steps the cached plan keeps being tracked, so control is still applied every tick.
    """

    cameras: Tuple[DriverCameraConfig, ...] = (DriverCameraConfig(),)
    """Cameras rendered and streamed to the policy."""

    image_quality: int = 90
    """JPEG quality (1-100) for the streamed camera frames."""

    route_horizon_m: float = 80.0
    """How far ahead the route sent to the policy extends."""

    route_resolution_m: float = 2.0
    """Spacing between route waypoints."""

    rear_axle_offset_m: Optional[float] = None
    """Distance from the CARLA actor origin back to the rig origin.

    alpasim places the rig origin at the rear-axle centre projected to the ground while
    CARLA places the actor origin at the vehicle centre.  ``None`` derives the offset
    from the vehicle's wheel physics; set it explicitly (e.g. ``-1.4``) if the derived
    value looks wrong, which is a known CARLA 0.9.x quirk where wheel positions are
    reported in world coordinates and centimetres.
    """

    send_ground_truth: bool = False
    """Whether to also submit the recorded ground-truth trajectory."""

    send_renderer_data: bool = True
    """Whether to send CARLA ground truth in ``DriveRequest.renderer_data``.

    The alpasim contract carries no traffic lights and no other vehicles, so a
    policy that reasons about either reads them from this extension payload.
    Policies that ignore it are unaffected; policies that read it treat an absent
    payload as "no light applies, no traffic present" **without erroring**, which
    silently disables every rule that depends on the world outside the ego.
    """

    send_actor_ground_truth: bool = True
    """Whether the payload includes the other vehicles' poses and velocities."""

    traffic_light_sight_distance_m: float = 60.0
    """How far down its own lane the ego looks for the light governing it.

    0 falls back to CARLA's ``is_at_traffic_light()``, whose trigger volumes are
    about a metre thick -- a policy learns of a red light on arrival, too late to
    stop from any ordinary speed.
    """

    actor_horizon_m: float = 150.0
    """Radius within which other vehicles are reported to the policy."""

    random_seed: int = 0
    """Seed handed to the policy in ``start_session``."""

    @classmethod
    def from_mapping(cls, mapping: Mapping[str, Any]) -> "DriverClientConfig":
        """Return a client config built from a plain mapping (e.g. a Hydra node).

        The ``cameras`` entry is converted to :class:`DriverCameraConfig` objects and a
        ``control`` entry, if present, is ignored -- the controller is configured
        separately via :meth:`ControlConfig.from_mapping`.

        Raises:
            ValueError: If *mapping* holds a key this config does not define.
        """
        values = _checked(cls, mapping, ignore={"control"})
        cameras = values.pop("cameras", None)
        if cameras is not None:
            values["cameras"] = tuple(
                DriverCameraConfig.from_mapping(camera) for camera in cameras
            )
        return cls(**values)


@dataclass
class EgoObservation:
    """The ego state handed to the driver for one policy step.

    Attributes:
        timestamp_us: Simulation time of this observation, in microseconds.
        pose: Ego rig pose in the local frame (right-handed).
        linear_velocity: ``(3,)`` velocity in the rig frame, m/s.
        angular_velocity: ``(3,)`` angular velocity in the rig frame, rad/s.
        linear_acceleration: ``(3,)`` acceleration in the rig frame, m/s^2.
        speed_mps: Ground speed magnitude, m/s.
    """

    timestamp_us: int
    pose: Pose
    linear_velocity: NDArray[np.float64]
    angular_velocity: NDArray[np.float64]
    linear_acceleration: NDArray[np.float64]
    speed_mps: float


@dataclass
class DriveOutcome:
    """The driver's answer to one ``drive`` call.

    Attributes:
        trajectory: Planned trajectory in the **local** frame, as returned on the wire.
        terminate_session: When ``True`` the policy asked to end the rollout early.
        debug_info: Raw ``unstructured_debug_info`` bytes from the response.  The
            alpasim contract deliberately leaves this unstructured.
        policy_name: Policy name, when the response carried a decodable
            ``CarlaDriveDebugInfo``.
        inference_seconds: How long the policy took, from the same message.
        debug_scalars: Free-form diagnostics the policy chose to surface.
    """

    trajectory: Trajectory
    terminate_session: bool = False
    debug_info: bytes = b""
    policy_name: str = ""
    inference_seconds: float = 0.0
    debug_scalars: Dict[str, float] = field(default_factory=dict)


class BaseEgoDriverClient(ABC):
    """Abstract client for a driver policy that plans on behalf of the ego vehicle.

    Implementations own whatever transport they need.  The lifecycle is:

    1. :meth:`start_session` once, after the ego actor exists.
    2. :meth:`submit_route` once the route is known, then on every policy step
       :meth:`submit_image_observation` / :meth:`submit_egomotion_observation`
       followed by :meth:`drive`.
    3. :meth:`close_session` during teardown.

    Args:
        config: Connection and cadence settings.
    """

    def __init__(self, config: DriverClientConfig) -> None:
        self._config = config

    @property
    def config(self) -> DriverClientConfig:
        """Return the client configuration."""
        return self._config

    @abstractmethod
    def start_session(self, session_uuid: str, scene_id: str) -> None:
        """Open a rollout session with the policy.

        Args:
            session_uuid: Identifier correlating every subsequent call.
            scene_id: Human-readable scene name for the policy's debug output.
        """
        ...

    @abstractmethod
    def submit_route(
        self, timestamp_us: int, waypoints_in_rig: NDArray[np.float64]
    ) -> None:
        """Send the route the ego should follow.

        Args:
            timestamp_us: Simulation time the waypoints are valid at.
            waypoints_in_rig: ``(N, 3)`` array of waypoints in the rig frame.
        """
        ...

    @abstractmethod
    def submit_image_observation(
        self,
        logical_id: str,
        frame_start_us: int,
        frame_end_us: int,
        image_bytes: bytes,
    ) -> None:
        """Send one encoded camera frame.

        Args:
            logical_id: Camera the frame came from.
            frame_start_us: Exposure start time.
            frame_end_us: Exposure end time.
            image_bytes: Encoded image payload (JPEG).
        """
        ...

    @abstractmethod
    def submit_egomotion_observation(self, observation: EgoObservation) -> None:
        """Send the ego's estimated pose and dynamic state."""
        ...

    @abstractmethod
    def drive(
        self,
        time_now_us: int,
        time_query_us: int,
        renderer_data: bytes = b"",
    ) -> DriveOutcome:
        """Ask the policy for a plan.

        Args:
            time_now_us: Planning time; the plan is anchored to the pose at this instant.
            time_query_us: The instant the runtime will next advance to.
            renderer_data: Serialized ``carla_driver.v0.CarlaRendererData`` giving
                the policy CARLA ground truth (traffic light, other vehicles,
                speed limit).  Empty means "no ground truth available".

        Returns:
            The policy's plan and termination flag.
        """
        ...

    @abstractmethod
    def close_session(self) -> None:
        """Close the session and release the transport.  Must be idempotent."""
        ...

    def camera_sensor_configs(self) -> List[Tuple[str, CarlaCameraSensorConfig]]:
        """Return ``(logical_id, sensor_config)`` for each configured camera."""
        return [
            (camera.logical_id, camera.to_sensor_config())
            for camera in self._config.cameras
        ]
