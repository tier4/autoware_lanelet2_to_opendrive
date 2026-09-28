"""Configuration for :class:`~.policy.EgoDriverPolicy`."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

from .control import ControlConfig
from .wire import ImageFormat

__all__ = [
    "SUPPORTED_IMAGE_FORMATS",
    "CameraConfig",
    "EgoDriverPolicyConfig",
    "default_camera_rig",
]

#: The formats alpasim drivers decode. ``sensorsim.proto`` names more, but
#: widening this would break the very drivers the policy targets.
SUPPORTED_IMAGE_FORMATS = (ImageFormat.PNG, ImageFormat.JPEG)


@dataclass(frozen=True)
class CameraConfig:
    """One ``sensor.camera.rgb`` mounted on the ego.

    ``x/y/z`` and the angles are the mount transform *relative to the vehicle
    actor*, in CARLA's own left-handed convention and degrees -- exactly what
    you would pass to ``carla.Transform``. Same fields and defaults as
    ``carla_driver_interface.runtime.CameraConfig``.
    """

    logical_id: str
    width: int = 960
    height: int = 604
    fov_deg: float = 120.0
    x: float = 1.5
    y: float = 0.0
    z: float = 1.6
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0
    roll_deg: float = 0.0


def default_camera_rig() -> tuple[CameraConfig, ...]:
    """The single forward camera ``carla_driver_interface`` mounts by default.

    Not the default here: a scenario without a camera-based driver should not
    pay for rendering one. Pass it as ``cameras`` for image-based drivers.
    """
    return (CameraConfig(logical_id="camera_front_wide_120fov"),)


@dataclass(frozen=True)
class EgoDriverPolicyConfig:
    """How the ego talks to an ``egodriver.EgodriverService`` driver.

    The defaults match ``carla_driver_interface.runtime.RuntimeConfig`` wherever
    the same knob exists, so a driver sees the same inputs under either runtime.
    """

    # -- driver --
    #: ``host:port`` of the driver, e.g. one started with
    #: ``carla-driver-interface serve --port 50051``.
    driver_address: str = "localhost:50051"
    driver_timeout_s: float = 60.0
    #: Reported to the driver as ``DriveSessionRequest.debug_info.scene_id``.
    #: ``None`` uses ``"<map name>:<scenario class name>"``.
    scene_id: Optional[str] = None

    # -- timing --
    #: How often the driver is queried. Must be a whole multiple of the
    #: world's ``fixed_delta_seconds`` (0.05 s under :class:`ScenarioRunner`).
    policy_timestep_s: float = 0.1

    # -- vehicle --
    #: Signed offset from the CARLA actor origin to the rig origin (rear axle
    #: centre) along the body x axis, in metres -- negative for a normal car.
    #: ``None`` derives it from the wheel physics. The value used is logged.
    rear_axle_offset_m: Optional[float] = None

    # -- sensors --
    #: Cameras mounted on the ego and streamed to the driver. Empty by default;
    #: see :func:`default_camera_rig`.
    cameras: tuple[CameraConfig, ...] = ()
    image_format: int = ImageFormat.JPEG
    image_quality: int = 90

    # -- route --
    #: Route to follow, as CARLA world ``(x, y, z)`` points. ``None`` walks the
    #: lane graph forward from the ego, choosing junction exits with the
    #: scenario's random seed.
    route: Optional[tuple[tuple[float, float, float], ...]] = None
    #: Length of the generated route when ``route`` is ``None``.
    route_length_m: float = 500.0
    #: How far ahead of the ego the route is published, in metres.
    route_horizon_m: float = 80.0
    #: Spacing of the published route waypoints, in metres.
    route_resolution_m: float = 2.0
    #: Also send the route as ``submit_recording_ground_truth``. There is no
    #: recording under CARLA, so this is off by default.
    send_ground_truth: bool = False

    # -- ground truth (CarlaRendererData) --
    #: Send ``CarlaRendererData`` in ``DriveRequest.renderer_data``.
    send_renderer_data: bool = True
    #: How far ahead along the lane graph a traffic light is looked for.
    #: 0 falls back to CARLA's trigger-volume test.
    traffic_light_sight_distance_m: float = 60.0
    #: Include other vehicles and pedestrians in the payload.
    send_actor_ground_truth: bool = True
    #: How far from the ego an actor is still reported, in metres.
    actor_horizon_m: float = 150.0

    # -- control --
    control: ControlConfig = field(default_factory=ControlConfig)

    # -- misc --
    #: Added to every timestamp, to place a rollout on an absolute timeline.
    epoch_offset_us: int = 0

    def __post_init__(self) -> None:
        if self.image_format not in SUPPORTED_IMAGE_FORMATS:
            raise ValueError(
                f"image_format {self.image_format!r} is not supported; "
                "alpasim drivers decode PNG and JPEG only"
            )
        if self.policy_timestep_s <= 0.0:
            raise ValueError("policy_timestep_s must be positive")
        if self.route is not None and len(self.route) < 2:
            raise ValueError("route needs at least two points")
        ids = [camera.logical_id for camera in self.cameras]
        if len(set(ids)) != len(ids):
            raise ValueError(f"camera logical ids must be unique, got {ids}")

    @property
    def policy_timestep_us(self) -> int:
        return int(round(self.policy_timestep_s * 1e6))

    def ticks_per_policy_step(self, fixed_delta_s: float) -> int:
        """How many world ticks one policy step spans.

        Raises:
            ValueError: If the policy step is not a whole multiple of the tick.
        """
        if fixed_delta_s <= 0.0:
            raise ValueError(
                "the world must run in synchronous mode with a fixed delta; "
                f"got fixed_delta_seconds={fixed_delta_s!r}"
            )
        ratio = self.policy_timestep_s / fixed_delta_s
        if abs(ratio - round(ratio)) > 1e-6 or round(ratio) < 1:
            raise ValueError(
                f"policy_timestep_s ({self.policy_timestep_s}) must be a positive "
                f"multiple of the world's fixed_delta_seconds ({fixed_delta_s})"
            )
        return int(round(ratio))
