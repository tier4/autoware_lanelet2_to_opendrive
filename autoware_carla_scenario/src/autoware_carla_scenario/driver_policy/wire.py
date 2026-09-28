"""Single import window for the egodriver wire contract.

Everything exchanged with a driver comes from the vendored ``.proto`` files
under ``autoware_carla_scenario/proto`` (see the README there). This module
re-exports the generated messages so call sites read the same way as they do
in ``carla_driver_interface.grpc_api``. It also holds the codec for the CARLA
extension payloads that ride inside the upstream ``bytes`` fields.

**Tolerance policy:** unpacking an extension payload never raises. The fields
are free-form by definition, so a driver from another project may put something
else there. A payload that cannot be parsed means "no CARLA data".
"""

from __future__ import annotations

import logging
from typing import Optional, TypeVar

from google.protobuf.message import DecodeError, Message

from ._proto.alpasim_grpc.v0.common_pb2 import (
    AABB,
    DynamicState,
    Empty,
    Pose,
    PoseAtTime,
    Quat,
    SessionRequestStatus,
    Trajectory,
    Vec3,
    VersionId,
)
from ._proto.alpasim_grpc.v0.egodriver_pb2 import (
    DriveRequest,
    DriveResponse,
    DriveSessionCloseRequest,
    DriveSessionRequest,
    GroundTruth,
    GroundTruthRequest,
    RolloutCameraImage,
    RolloutEgoTrajectory,
    Route,
    RouteRequest,
)
from ._proto.alpasim_grpc.v0.egodriver_pb2_grpc import (
    EgodriverServiceServicer,
    EgodriverServiceStub,
    add_EgodriverServiceServicer_to_server,
)
from ._proto.alpasim_grpc.v0.sensorsim_pb2 import (
    AvailableCamerasReturn,
    CameraSpec,
    ImageFormat,
    OpenCVPinholeCameraParam,
    ShutterType,
)
from ._proto.carla_driver.v0.carla_driver_pb2 import (
    CarlaActorState,
    CarlaDriveDebugInfo,
    CarlaRendererData,
    CarlaWeather,
    TrafficLightState,
)

logger = logging.getLogger(__name__)

#: alpasim commit the vendored ``alpasim_grpc/v0/*.proto`` files come from.
#: ``carla_driver_interface`` pins its ``alpasim-grpc`` dependency to the same one.
ALPASIM_GRPC_REV = "68709245a5dc0f2eda4f8cb2c3aa8cbdfa913043"

#: carla_driver_interface commit the vendored ``carla_driver.proto`` comes from.
CARLA_DRIVER_INTERFACE_REV = "ccc84acc1b2547055885239ec8b85f450d5992ad"

#: What ``alpasim_grpc.API_VERSION_MESSAGE`` evaluates to at
#: :data:`ALPASIM_GRPC_REV`. Upstream derives it from the package version
#: (0.55.0). There is no installed package here to read it from, so the value
#: is written out.
API_VERSION_MESSAGE = VersionId.APIVersion(major=0, minor=55, patch=0)

#: The gRPC service this package speaks. The method paths are
#: ``/egodriver.EgodriverService/<rpc>``.
EGODRIVER_SERVICE_FULL_NAME = "egodriver.EgodriverService"

#: The same limit ``carla_driver_interface`` builds both ends of its channel
#: with. Whole camera frames travel as single unary messages, so the gRPC
#: default of 4 MiB is too small above roughly 1080p.
MAX_MESSAGE_BYTES = 64 * 1024 * 1024

#: Convenience alias: the camera message is deeply nested upstream.
AvailableCamera = AvailableCamerasReturn.AvailableCamera

_M = TypeVar("_M", bound=Message)


def channel_options() -> list[tuple[str, int]]:
    """gRPC options for the driver channel."""
    return [
        ("grpc.max_receive_message_length", MAX_MESSAGE_BYTES),
        ("grpc.max_send_message_length", MAX_MESSAGE_BYTES),
    ]


def describe_api_mismatch(other: VersionId.APIVersion) -> Optional[str]:
    """Compare a driver's API version with ours. Returns a message if they differ."""
    ours = API_VERSION_MESSAGE
    if (other.major, other.minor, other.patch) == (ours.major, ours.minor, ours.patch):
        return None
    return (
        f"alpasim_grpc API version mismatch: driver reports "
        f"{other.major}.{other.minor}.{other.patch}, this policy was built against "
        f"{ours.major}.{ours.minor}.{ours.patch}. "
        "Compatible unless the messages in use actually changed between those releases."
    )


def pack_renderer_data(data: CarlaRendererData) -> bytes:
    """Serialize for ``DriveRequest.renderer_data``."""
    return data.SerializeToString()


def unpack_renderer_data(payload: bytes) -> Optional[CarlaRendererData]:
    """Parse ``DriveRequest.renderer_data``. ``None`` if absent or foreign."""
    return _unpack(payload, CarlaRendererData, "renderer_data")


def unpack_debug_info(payload: bytes) -> Optional[CarlaDriveDebugInfo]:
    """Parse the driver's debug payload. ``None`` if absent or foreign."""
    return _unpack(payload, CarlaDriveDebugInfo, "unstructured_debug_info")


def _unpack(payload: bytes, message_type: type[_M], field: str) -> Optional[_M]:
    if not payload:
        return None
    message = message_type()
    try:
        message.ParseFromString(payload)
    except (DecodeError, UnicodeDecodeError):
        logger.debug("%s is not a %s; ignoring", field, message_type.__name__)
        return None
    return message


__all__ = [
    "AABB",
    "ALPASIM_GRPC_REV",
    "API_VERSION_MESSAGE",
    "AvailableCamera",
    "AvailableCamerasReturn",
    "CARLA_DRIVER_INTERFACE_REV",
    "CameraSpec",
    "CarlaActorState",
    "CarlaDriveDebugInfo",
    "CarlaRendererData",
    "CarlaWeather",
    "DriveRequest",
    "DriveResponse",
    "DriveSessionCloseRequest",
    "DriveSessionRequest",
    "DynamicState",
    "EGODRIVER_SERVICE_FULL_NAME",
    "EgodriverServiceServicer",
    "EgodriverServiceStub",
    "Empty",
    "GroundTruth",
    "GroundTruthRequest",
    "ImageFormat",
    "MAX_MESSAGE_BYTES",
    "OpenCVPinholeCameraParam",
    "Pose",
    "PoseAtTime",
    "Quat",
    "RolloutCameraImage",
    "RolloutEgoTrajectory",
    "Route",
    "RouteRequest",
    "SessionRequestStatus",
    "ShutterType",
    "TrafficLightState",
    "Trajectory",
    "Vec3",
    "VersionId",
    "add_EgodriverServiceServicer_to_server",
    "channel_options",
    "describe_api_mismatch",
    "pack_renderer_data",
    "unpack_debug_info",
    "unpack_renderer_data",
]
