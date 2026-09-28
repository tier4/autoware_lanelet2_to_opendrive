"""``EgoDriverPolicy``: the scenario runner in alpasim's Runtime role.

``carla_driver_interface`` splits a closed loop in two. A *driver* serves
``egodriver.EgodriverService`` and knows nothing about CARLA. A *runtime* owns
the world and queries the driver once per policy step. Its own
``CarlaRuntime`` would connect a second client and load the map again, so a
scenario runner that already owns a world plays the Runtime role itself
(``carla_driver_interface/docs/ARCHITECTURE.md``). This class is that role.

It does not own the clock. :class:`ScenarioRunner` ticks the world; the policy
hooks in on either side of every tick:

* :meth:`post_tick` records the ego state the tick produced.
* :meth:`pre_tick` runs a policy step on every ``ticks_per_policy_step``-th
  tick, in alpasim's ``PolicyEvent`` order: submit observations (images,
  egomotion, route, optional ground truth), then ``drive``, then track the
  returned plan with :class:`TrajectoryFollower` and apply the control.

Egomotion is sent without noise, so the driver's estimated frame is the true
one and no frame correction is needed on the returned plan.
"""

from __future__ import annotations

import contextlib
import logging
import math
import queue
import random
import time
import uuid
from dataclasses import dataclass
from typing import Any, Optional

import grpc
import numpy as np

from .config import CameraConfig, EgoDriverPolicyConfig
from .control import TrajectoryFollower, VehicleCommand
from .conversions import (
    available_camera,
    camera_pose_in_rig,
    carla_transform_to_pose,
    carla_vector_to_local,
    rig_pose_from_actor_transform,
    seconds_to_us,
    vector_local_to_rig,
    waypoint_to_local,
)
from .geometry import Pose, Trajectory, dynamic_state_proto
from .ground_truth import CarlaGroundTruth
from .images import encode_bgra
from .route import RouteProvider
from .wire import (
    AvailableCamera,
    DriveRequest,
    DriveSessionCloseRequest,
    DriveSessionRequest,
    EgodriverServiceStub,
    Empty,
    GroundTruth,
    GroundTruthRequest,
    RolloutCameraImage,
    RolloutEgoTrajectory,
    Route,
    RouteRequest,
    Vec3,
    channel_options,
    describe_api_mismatch,
    pack_renderer_data,
    unpack_debug_info,
)

logger = logging.getLogger(__name__)

__all__ = ["EgoDriverPolicy", "EgoSample"]

#: Route completion at which the policy stops querying the driver and brakes.
_ROUTE_DONE = 0.999


@dataclass(frozen=True)
class EgoSample:
    """One tick's ego state, in the alpasim ``local``/``rig`` convention."""

    timestamp_us: int
    frame_id: int
    pose_local_to_rig: Pose
    linear_velocity_in_rig: np.ndarray
    angular_velocity_in_rig: np.ndarray
    linear_acceleration_in_rig: np.ndarray

    @property
    def speed_mps(self) -> float:
        return float(np.linalg.norm(self.linear_velocity_in_rig))


@dataclass(frozen=True)
class _RawFrame:
    timestamp_us: int
    bgra: bytes
    width: int
    height: int


class EgoDriverPolicy:
    """Drives one ego actor from an external egodriver gRPC service.

    Lifecycle: :meth:`start` once, after the ego has spawned and the world has
    settled; :meth:`pre_tick` / :meth:`post_tick` around every
    ``world.tick()``; :meth:`close` once at the end. :class:`EgoDriverEntity`
    wires these into :class:`ScenarioRunner`.

    Args:
        config: Driver address, timing, sensors and route.
        channel: An existing gRPC channel to the driver. When given, the policy
            uses it and does not close it; otherwise it opens its own to
            ``config.driver_address``.
    """

    def __init__(
        self,
        config: Optional[EgoDriverPolicyConfig] = None,
        channel: Optional[grpc.Channel] = None,
    ) -> None:
        self.config = config or EgoDriverPolicyConfig()
        self._external_channel = channel
        self._channel: Optional[grpc.Channel] = None
        self._stub: Optional[EgodriverServiceStub] = None
        self._session_uuid: Optional[str] = None

        self._carla: Any = None
        self._world: Any = None
        self._ego: Any = None
        self._rear_axle_offset_m = 0.0
        self._ticks_per_step = 1
        self._tick_index = 0

        self._follower = TrajectoryFollower(self.config.control)
        self._route: Optional[RouteProvider] = None
        self._ground_truth: Optional[CarlaGroundTruth] = None
        self._cameras: list[Any] = []
        self._frame_queues: dict[str, queue.Queue[_RawFrame]] = {}

        self._latest: Optional[EgoSample] = None
        self._pending_egomotion: list[EgoSample] = []

        self.steps = 0
        #: ``True`` once the driver set ``DriveResponse.terminate_session``.
        self.terminated_by_driver = False
        self.last_plan: Optional[Trajectory] = None
        self.last_command: Optional[VehicleCommand] = None

    # ------------------------------------------------------------------
    # Status
    # ------------------------------------------------------------------

    @property
    def started(self) -> bool:
        return self._session_uuid is not None

    @property
    def route_completion(self) -> float:
        """Fraction of the route driven so far, in ``[0, 1]``."""
        return self._route.completion if self._route is not None else 0.0

    @property
    def finished(self) -> bool:
        """The driver ended the session or the route is complete."""
        return self.terminated_by_driver or self.route_completion >= _ROUTE_DONE

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(
        self,
        world: Any,
        ego_actor: Any,
        *,
        scene_id: str,
        random_seed: int = 0,
        session_uuid: Optional[str] = None,
    ) -> None:
        """Open the driver session. Call after warm-up, before the tick loop."""
        import carla  # noqa: PLC0415 - keeps the module importable without CARLA

        self._carla = carla
        self._world = world
        self._ego = ego_actor
        self._ticks_per_step = self.config.ticks_per_policy_step(
            world.get_settings().fixed_delta_seconds
        )
        self._rear_axle_offset_m = self._resolve_rear_axle_offset()

        carla_map = world.get_map()
        route = self._build_route(carla_map, random.Random(random_seed))
        self._route = RouteProvider(
            route,
            horizon_m=self.config.route_horizon_m,
            resolution_m=self.config.route_resolution_m,
        )
        if self.config.send_renderer_data:
            self._ground_truth = CarlaGroundTruth(
                world=world,
                ego=ego_actor,
                carla_map=carla_map,
                config=self.config,
                map_name=carla_map.name,
            )
        cameras = self._spawn_cameras()

        self._channel = self._external_channel or grpc.insecure_channel(
            self.config.driver_address, options=channel_options()
        )
        self._stub = EgodriverServiceStub(self._channel)
        self._check_driver_version()

        session_uuid = session_uuid or str(uuid.uuid4())
        self._stub.start_session(
            DriveSessionRequest(
                session_uuid=session_uuid,
                random_seed=random_seed,
                debug_info=DriveSessionRequest.DebugInfo(scene_id=scene_id),
                rollout_spec=DriveSessionRequest.RolloutSpec(
                    vehicle=DriveSessionRequest.RolloutSpec.VehicleDefinition(
                        available_cameras=cameras,
                    ),
                ),
            ),
            timeout=self.config.driver_timeout_s,
        )
        self._session_uuid = session_uuid
        self._record(world.get_snapshot())
        logger.info(
            "ego driver session %s on %s: driver %s, %d camera(s), route %.1f m, "
            "rear axle offset %.3f m, policy every %d tick(s)",
            session_uuid,
            scene_id,
            self.config.driver_address,
            len(cameras),
            self._route.total_length_m,
            self._rear_axle_offset_m,
            self._ticks_per_step,
        )

    def pre_tick(self, world: Any) -> None:
        """Run a policy step if one is due, and apply its control."""
        if not self.started:
            return
        due = self._tick_index % self._ticks_per_step == 0
        self._tick_index += 1
        if not due:
            return
        if self.finished:
            self._apply(VehicleCommand(brake=1.0))
            return
        self._policy_step()

    def post_tick(self, world: Any) -> None:
        """Record the ego state the tick produced."""
        if self.started:
            self._record(world.get_snapshot())

    def close(self) -> None:
        """Close the session and release sensors. Safe to call more than once."""
        for camera in self._cameras:
            with contextlib.suppress(RuntimeError):
                camera.stop()
            with contextlib.suppress(RuntimeError):
                camera.destroy()
        self._cameras.clear()

        if self._stub is not None and self._session_uuid is not None:
            try:
                self._stub.close_session(
                    DriveSessionCloseRequest(session_uuid=self._session_uuid),
                    timeout=self.config.driver_timeout_s,
                )
            except grpc.RpcError as exc:
                logger.warning("driver close_session failed: %s", exc.code().name)
        self._session_uuid = None
        self._stub = None
        if self._channel is not None and self._external_channel is None:
            self._channel.close()
        self._channel = None

    # ------------------------------------------------------------------
    # Policy step
    # ------------------------------------------------------------------

    def _policy_step(self) -> None:
        assert self._stub is not None and self._latest is not None
        sample = self._latest
        self._submit_observations(sample)

        renderer_data = b""
        if self._ground_truth is not None:
            renderer_data = pack_renderer_data(
                self._ground_truth.read(
                    sample.timestamp_us, sample.frame_id, sample.pose_local_to_rig
                )
            )
        started = time.perf_counter()
        response = self._stub.drive(
            DriveRequest(
                session_uuid=self._session_uuid,
                time_now_us=sample.timestamp_us,
                time_query_us=sample.timestamp_us + self.config.policy_timestep_us,
                renderer_data=renderer_data,
            ),
            timeout=self.config.driver_timeout_s,
        )
        latency_s = time.perf_counter() - started
        self.steps += 1

        plan = Trajectory.from_proto(response.trajectory)
        self.last_plan = plan
        self._log_driver_debug(response, latency_s)

        if response.terminate_session:
            logger.info("driver requested termination at t=%d us", sample.timestamp_us)
            self.terminated_by_driver = True
            self._apply(VehicleCommand(brake=1.0))
            return

        command = self._follower.step(
            plan,
            sample.pose_local_to_rig,
            sample.speed_mps,
            self.config.policy_timestep_s,
        )
        self._apply(command)

    def _apply(self, command: VehicleCommand) -> None:
        self.last_command = command
        self._ego.apply_control(
            self._carla.VehicleControl(
                throttle=float(np.clip(command.throttle, 0.0, 1.0)),
                steer=float(np.clip(command.steer, -1.0, 1.0)),
                brake=float(np.clip(command.brake, 0.0, 1.0)),
                hand_brake=command.hand_brake,
                reverse=command.reverse,
            )
        )

    def _check_driver_version(self) -> None:
        """Log the driver's identity; warn if its API version differs. Never fatal."""
        assert self._stub is not None
        try:
            version = self._stub.get_version(
                Empty(), timeout=self.config.driver_timeout_s
            )
        except grpc.RpcError as exc:
            logger.warning(
                "driver get_version failed (%s); continuing", exc.code().name
            )
            return
        logger.info("driver version_id=%r", version.version_id)
        mismatch = describe_api_mismatch(version.grpc_api_version)
        if mismatch:
            logger.warning("%s", mismatch)

    def _log_driver_debug(self, response: Any, latency_s: float) -> None:
        if not logger.isEnabledFor(logging.DEBUG):
            return
        debug = unpack_debug_info(response.debug_info.unstructured_debug_info)
        if debug is None:
            logger.debug("driver step %d: %.1f ms", self.steps, latency_s * 1e3)
            return
        logger.debug(
            "driver %s step %d: %.1f ms (inference %.1f ms), %s",
            debug.policy_name,
            self.steps,
            latency_s * 1e3,
            debug.inference_seconds * 1e3,
            dict(debug.scalars),
        )

    # ------------------------------------------------------------------
    # Observations
    # ------------------------------------------------------------------

    def _submit_observations(self, sample: EgoSample) -> None:
        """Send everything the driver needs before ``drive``.

        The next call is ``drive``, which is the barrier alpasim enforces: all
        observations land before the decision.
        """
        assert self._stub is not None
        timeout = self.config.driver_timeout_s
        for logical_id, frame in self._drain_frames():
            self._stub.submit_image_observation(
                RolloutCameraImage(
                    session_uuid=self._session_uuid,
                    camera_image=RolloutCameraImage.CameraImage(
                        # A CARLA RGB capture is instantaneous: start == end.
                        frame_start_us=frame.timestamp_us,
                        frame_end_us=frame.timestamp_us,
                        image_bytes=encode_bgra(
                            frame.bgra,
                            frame.width,
                            frame.height,
                            self.config.image_format,
                            self.config.image_quality,
                        ),
                        logical_id=logical_id,
                    ),
                ),
                timeout=timeout,
            )

        self._submit_egomotion()

        assert self._route is not None
        # Advances the route's progress marker: exactly once per step.
        waypoints = self._route.waypoints_in_rig(sample.pose_local_to_rig)
        self._stub.submit_route(
            RouteRequest(
                session_uuid=self._session_uuid,
                route=Route(
                    timestamp_us=sample.timestamp_us,
                    waypoints=[
                        Vec3(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                        for p in waypoints
                    ],
                ),
            ),
            timeout=timeout,
        )
        if self.config.send_ground_truth:
            step_us = self.config.policy_timestep_us
            reference = Trajectory(
                [sample.timestamp_us + i * step_us for i in range(len(waypoints))],
                [Pose(p, np.array([0.0, 0.0, 0.0, 1.0])) for p in waypoints],
            )
            self._stub.submit_recording_ground_truth(
                GroundTruthRequest(
                    session_uuid=self._session_uuid,
                    ground_truth=GroundTruth(
                        timestamp_us=sample.timestamp_us,
                        trajectory=reference.to_proto(),
                    ),
                ),
                timeout=timeout,
            )

    def _submit_egomotion(self) -> None:
        """Send every pose since the last step, as alpasim does.

        Drivers that build an ego-history feature need the intermediate ticks,
        not just the newest pose.
        """
        assert self._stub is not None
        if not self._pending_egomotion:
            return
        samples, self._pending_egomotion = self._pending_egomotion, []
        trajectory = Trajectory(
            [s.timestamp_us for s in samples], [s.pose_local_to_rig for s in samples]
        )
        self._stub.submit_egomotion_observation(
            RolloutEgoTrajectory(
                session_uuid=self._session_uuid,
                trajectory=trajectory.to_proto(),
                dynamic_states=[
                    dynamic_state_proto(
                        linear_velocity=s.linear_velocity_in_rig,
                        angular_velocity=s.angular_velocity_in_rig,
                        linear_acceleration=s.linear_acceleration_in_rig,
                    )
                    for s in samples
                ],
            ),
            timeout=self.config.driver_timeout_s,
        )

    def _record(self, snapshot: Any) -> None:
        """Fold one world snapshot into the policy's view of the ego."""
        timestamp_us = seconds_to_us(
            snapshot.timestamp.elapsed_seconds, self.config.epoch_offset_us
        )
        if self._latest is not None and timestamp_us <= self._latest.timestamp_us:
            # A tick that did not advance the clock carries no new information,
            # and the egomotion trajectory must strictly increase in time.
            return
        sample = self._ego_sample(timestamp_us, int(snapshot.frame))
        self._latest = sample
        self._pending_egomotion.append(sample)

    def _ego_sample(self, timestamp_us: int, frame_id: int) -> EgoSample:
        transform = self._ego.get_transform()
        location, rotation = transform.location, transform.rotation
        pose = rig_pose_from_actor_transform(
            (location.x, location.y, location.z),
            (rotation.pitch, rotation.yaw, rotation.roll),
            self._rear_axle_offset_m,
        )
        velocity = self._ego.get_velocity()
        acceleration = self._ego.get_acceleration()
        angular = self._ego.get_angular_velocity()  # degrees/s in CARLA
        return EgoSample(
            timestamp_us=timestamp_us,
            frame_id=frame_id,
            pose_local_to_rig=pose,
            linear_velocity_in_rig=vector_local_to_rig(
                carla_vector_to_local(velocity.x, velocity.y, velocity.z), pose
            ),
            angular_velocity_in_rig=vector_local_to_rig(
                carla_vector_to_local(
                    math.radians(angular.x),
                    math.radians(angular.y),
                    math.radians(angular.z),
                ),
                pose,
            ),
            linear_acceleration_in_rig=vector_local_to_rig(
                carla_vector_to_local(acceleration.x, acceleration.y, acceleration.z),
                pose,
            ),
        )

    # ------------------------------------------------------------------
    # Setup helpers
    # ------------------------------------------------------------------

    def _resolve_rear_axle_offset(self) -> float:
        if self.config.rear_axle_offset_m is not None:
            return self.config.rear_axle_offset_m
        offset = self._derive_rear_axle_offset()
        logger.info(
            "rear axle offset %.3f m derived from wheel physics; set "
            "EgoDriverPolicyConfig.rear_axle_offset_m if the geometry looks wrong",
            offset,
        )
        return offset

    def _derive_rear_axle_offset(self) -> float:
        """Longitudinal distance from the actor origin back to the rear axle.

        CARLA reports ``WheelPhysicsControl.position`` in **world
        centimetres**, so the wheel positions are converted into the actor frame
        explicitly rather than trusted to be relative already.
        """
        try:
            wheels = list(self._ego.get_physics_control().wheels)
        except (AttributeError, RuntimeError):
            wheels = []
        if len(wheels) < 4:
            fallback = -0.5 * float(self._ego.bounding_box.extent.x)
            logger.warning(
                "wheel physics unavailable; using half the bounding box (%.3f m)",
                fallback,
            )
            return fallback

        transform = self._ego.get_transform()
        location, rotation = transform.location, transform.rotation
        world_to_actor = carla_transform_to_pose(
            (location.x, location.y, location.z),
            (rotation.pitch, rotation.yaw, rotation.roll),
        ).inverse()
        rear_x = []
        for wheel in wheels[2:4]:  # CARLA orders wheels FL, FR, RL, RR
            position_cm = wheel.position
            point = carla_vector_to_local(
                position_cm.x / 100.0, position_cm.y / 100.0, position_cm.z / 100.0
            )
            rear_x.append(world_to_actor.transform_points(point)[0][0])
        return float(np.mean(rear_x))

    def _build_route(self, carla_map: Any, rng: random.Random) -> np.ndarray:
        """The route in the ``local`` frame: configured, or walked from the ego."""
        if self.config.route is not None:
            return np.stack([carla_vector_to_local(*p) for p in self.config.route])

        step = max(0.5, self.config.route_resolution_m)
        waypoint = carla_map.get_waypoint(
            self._ego.get_transform().location, project_to_road=True
        )
        if waypoint is None:
            raise RuntimeError("the ego is not on a road; cannot build a route")
        points = [waypoint_to_local(waypoint)]
        travelled = 0.0
        while travelled < self.config.route_length_m:
            options = waypoint.next(step)
            if not options:
                break
            waypoint = options[rng.randrange(len(options))]
            points.append(waypoint_to_local(waypoint))
            travelled += step
        if len(points) < 2:
            raise RuntimeError(
                "could not build a route from the ego position; pass "
                "EgoDriverPolicyConfig.route explicitly"
            )
        return np.stack(points)

    def _spawn_cameras(self) -> list[AvailableCamera]:
        """Attach the configured cameras and describe them for ``start_session``."""
        described = []
        for cam in self.config.cameras:
            blueprint = self._world.get_blueprint_library().find("sensor.camera.rgb")
            blueprint.set_attribute("image_size_x", str(cam.width))
            blueprint.set_attribute("image_size_y", str(cam.height))
            blueprint.set_attribute("fov", str(cam.fov_deg))
            mount = self._carla.Transform(
                self._carla.Location(x=cam.x, y=cam.y, z=cam.z),
                self._carla.Rotation(
                    pitch=cam.pitch_deg, yaw=cam.yaw_deg, roll=cam.roll_deg
                ),
            )
            sensor = self._world.spawn_actor(blueprint, mount, attach_to=self._ego)
            frames: queue.Queue[_RawFrame] = queue.Queue()
            sensor.listen(self._frame_callback(cam, frames))
            self._cameras.append(sensor)
            self._frame_queues[cam.logical_id] = frames
            described.append(
                available_camera(
                    cam.logical_id,
                    cam.width,
                    cam.height,
                    cam.fov_deg,
                    camera_pose_in_rig(
                        cam.x,
                        cam.y,
                        cam.z,
                        cam.pitch_deg,
                        cam.yaw_deg,
                        cam.roll_deg,
                        self._rear_axle_offset_m,
                    ),
                )
            )
        return described

    def _frame_callback(self, cam: CameraConfig, frames: queue.Queue[_RawFrame]) -> Any:
        epoch = self.config.epoch_offset_us

        def callback(image: Any) -> None:
            # Runs on a CARLA client thread: copy out and encode later, on the
            # policy step, so a slow encoder cannot back up the sensor stream.
            frames.put(
                _RawFrame(
                    timestamp_us=seconds_to_us(image.timestamp, epoch),
                    bgra=bytes(image.raw_data),
                    width=cam.width,
                    height=cam.height,
                )
            )

        return callback

    def _drain_frames(self) -> list[tuple[str, _RawFrame]]:
        """The newest frame per camera; anything older is stale and dropped."""
        newest_frames = []
        for logical_id, frames in self._frame_queues.items():
            newest: Optional[_RawFrame] = None
            while True:
                try:
                    newest = frames.get_nowait()
                except queue.Empty:
                    break
            if newest is not None:
                newest_frames.append((logical_id, newest))
        return newest_frames
