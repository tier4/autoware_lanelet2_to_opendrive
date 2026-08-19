"""gRPC client for the alpasim ``egodriver.EgodriverService`` contract.

This is the *runtime* half of the conversation: the scenario framework renders
observations from CARLA and asks a driver policy -- running as a separate gRPC server,
for example ``carla-driver-interface serve`` -- what to do next.

Because the generated stubs come from the vendored alpasim protos, the messages are wire
compatible with an upstream alpasim driver as well; see
``autoware_carla_scenario/proto/README.md``.
"""

from __future__ import annotations

import logging
from typing import Optional

import grpc
import numpy as np
from numpy.typing import NDArray

from ._proto import common_pb2, egodriver_pb2, egodriver_pb2_grpc, sensorsim_pb2
from .base import BaseEgoDriverClient, DriveOutcome, DriverClientConfig, EgoObservation
from .geometry import Trajectory, waypoints_to_proto


logger = logging.getLogger(__name__)

__all__ = ["EgoDriverGrpcClient"]

#: Maximum gRPC message size, matching alpasim's own limit.  Camera frames dominate.
MAX_MESSAGE_BYTES: int = 64 * 1024 * 1024

#: Full service name, used only for log messages and error text.
EGODRIVER_SERVICE_FULL_NAME: str = "egodriver.EgodriverService"


def channel_options() -> list[tuple[str, int]]:
    """Return the gRPC channel options used for driver connections."""
    return [
        ("grpc.max_send_message_length", MAX_MESSAGE_BYTES),
        ("grpc.max_receive_message_length", MAX_MESSAGE_BYTES),
    ]


class EgoDriverGrpcClient(BaseEgoDriverClient):
    """Talks to a driver policy over ``egodriver.EgodriverService``.

    Args:
        config: Connection and cadence settings.
        channel: Pre-built channel to use instead of dialling
            :attr:`DriverClientConfig.address`.  Intended for tests, which run the
            policy in-process.
    """

    def __init__(
        self,
        config: DriverClientConfig,
        channel: Optional[grpc.Channel] = None,
    ) -> None:
        super().__init__(config)
        self._owns_channel = channel is None
        self._channel: Optional[grpc.Channel] = channel
        self._stub: Optional[egodriver_pb2_grpc.EgodriverServiceStub] = None
        self._session_uuid: Optional[str] = None

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------

    @property
    def session_uuid(self) -> Optional[str]:
        """Return the active session id, or ``None`` before :meth:`start_session`."""
        return self._session_uuid

    def _connect(self) -> egodriver_pb2_grpc.EgodriverServiceStub:
        """Return the stub, dialling the configured address on first use."""
        if self._stub is not None:
            return self._stub
        if self._channel is None:
            self._channel = grpc.insecure_channel(
                self._config.address, options=channel_options()
            )
        self._stub = egodriver_pb2_grpc.EgodriverServiceStub(self._channel)
        return self._stub

    def _camera_specs(self) -> list:
        """Return the ``AvailableCamera`` entries describing the configured rig."""
        cameras = []
        for camera in self._config.cameras:
            sensor = camera.to_sensor_config()
            spec = sensorsim_pb2.CameraSpec(
                opencv_pinhole_param=sensorsim_pb2.OpenCVPinholeCameraParam(
                    principal_point_x=sensor.cx,
                    principal_point_y=sensor.cy,
                    focal_length_x=sensor.fx,
                    focal_length_y=sensor.fy,
                ),
                logical_id=camera.logical_id,
                resolution_h=camera.image_height,
                resolution_w=camera.image_width,
            )
            cameras.append(
                sensorsim_pb2.AvailableCamerasReturn.AvailableCamera(
                    intrinsics=spec,
                    logical_id=camera.logical_id,
                )
            )
        return cameras

    # ------------------------------------------------------------------
    # BaseEgoDriverClient interface
    # ------------------------------------------------------------------

    def start_session(self, session_uuid: str, scene_id: str) -> None:
        """Open a rollout session with the policy.

        Also calls ``get_version`` first so that an unreachable or mismatched policy
        fails immediately with a clear message rather than midway through the scenario.

        Raises:
            grpc.RpcError: If the policy cannot be reached.
        """
        stub = self._connect()

        version = stub.get_version(common_pb2.Empty(), timeout=self._config.timeout_s)
        logger.info(
            "Connected to %s at %s (version_id=%r git_hash=%r)",
            EGODRIVER_SERVICE_FULL_NAME,
            self._config.address,
            version.version_id,
            version.git_hash,
        )

        request = egodriver_pb2.DriveSessionRequest(
            session_uuid=session_uuid,
            random_seed=self._config.random_seed,
            debug_info=egodriver_pb2.DriveSessionRequest.DebugInfo(scene_id=scene_id),
            rollout_spec=egodriver_pb2.DriveSessionRequest.RolloutSpec(
                vehicle=egodriver_pb2.DriveSessionRequest.RolloutSpec.VehicleDefinition(
                    available_cameras=self._camera_specs()
                )
            ),
        )
        stub.start_session(request, timeout=self._config.timeout_s)
        self._session_uuid = session_uuid
        logger.info("Driver session started: uuid=%s scene=%s", session_uuid, scene_id)

    def submit_route(
        self, timestamp_us: int, waypoints_in_rig: NDArray[np.float64]
    ) -> None:
        """Send the route the ego should follow, in the rig frame."""
        stub = self._require_session()
        route = egodriver_pb2.Route(
            timestamp_us=timestamp_us,
            waypoints=waypoints_to_proto(np.asarray(waypoints_in_rig).reshape(-1, 3)),
        )
        stub.submit_route(
            egodriver_pb2.RouteRequest(session_uuid=self._session_uuid, route=route),
            timeout=self._config.timeout_s,
        )

    def submit_image_observation(
        self,
        logical_id: str,
        frame_start_us: int,
        frame_end_us: int,
        image_bytes: bytes,
    ) -> None:
        """Send one encoded camera frame."""
        stub = self._require_session()
        stub.submit_image_observation(
            egodriver_pb2.RolloutCameraImage(
                session_uuid=self._session_uuid,
                camera_image=egodriver_pb2.RolloutCameraImage.CameraImage(
                    frame_start_us=frame_start_us,
                    frame_end_us=frame_end_us,
                    image_bytes=image_bytes,
                    logical_id=logical_id,
                ),
            ),
            timeout=self._config.timeout_s,
        )

    def submit_egomotion_observation(self, observation: EgoObservation) -> None:
        """Send the ego's estimated pose and dynamic state for one instant."""
        stub = self._require_session()
        trajectory = Trajectory(
            [observation.timestamp_us], [observation.pose]
        ).to_proto()
        state = common_pb2.DynamicState(
            linear_velocity=_vec3(observation.linear_velocity),
            angular_velocity=_vec3(observation.angular_velocity),
            linear_acceleration=_vec3(observation.linear_acceleration),
        )
        stub.submit_egomotion_observation(
            egodriver_pb2.RolloutEgoTrajectory(
                session_uuid=self._session_uuid,
                trajectory=trajectory,
                dynamic_states=[state],
            ),
            timeout=self._config.timeout_s,
        )

    def drive(self, time_now_us: int, time_query_us: int) -> DriveOutcome:
        """Ask the policy for a plan.

        Returns:
            The plan in the **local** frame, plus the policy's termination flag.
        """
        stub = self._require_session()
        response = stub.drive(
            egodriver_pb2.DriveRequest(
                session_uuid=self._session_uuid,
                time_now_us=time_now_us,
                time_query_us=time_query_us,
            ),
            timeout=self._config.timeout_s,
        )
        return DriveOutcome(
            trajectory=Trajectory.from_proto(response.trajectory),
            terminate_session=bool(response.terminate_session),
            debug_info=bytes(response.debug_info.unstructured_debug_info),
        )

    def close_session(self) -> None:
        """Close the session and the channel.  Safe to call more than once."""
        if self._stub is not None and self._session_uuid is not None:
            try:
                self._stub.close_session(
                    egodriver_pb2.DriveSessionCloseRequest(
                        session_uuid=self._session_uuid
                    ),
                    timeout=self._config.timeout_s,
                )
            except grpc.RpcError:
                logger.warning("close_session RPC failed", exc_info=True)
        self._session_uuid = None
        self._stub = None
        if self._channel is not None and self._owns_channel:
            self._channel.close()
        self._channel = None

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _require_session(self) -> egodriver_pb2_grpc.EgodriverServiceStub:
        """Return the stub, asserting a session is open.

        Raises:
            RuntimeError: If :meth:`start_session` has not been called.
        """
        if self._stub is None or self._session_uuid is None:
            raise RuntimeError("No driver session is open. Call start_session() first.")
        return self._stub


def _vec3(values: NDArray[np.float64]) -> common_pb2.Vec3:
    """Return *values* as a protobuf ``Vec3``."""
    array = np.asarray(values, dtype=np.float64).reshape(-1)
    return common_pb2.Vec3(x=array[0], y=array[1], z=array[2])
