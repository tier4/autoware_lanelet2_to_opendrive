"""ROS 2-compatible message types for Autoware DDS communication.

Each dataclass mirrors its ROS 2 counterpart.  The ``typename`` argument
matches the DDS type name mangling used by ``rmw_cyclonedds`` so that
these readers/writers are wire-compatible with a ROS 2 graph.

Field **order** and **types** must match the CDR serialisation layout of
the corresponding ``.msg`` definition exactly.
"""

from __future__ import annotations

from dataclasses import dataclass

from cyclonedds.idl import IdlStruct
from cyclonedds.idl.types import (
    array,
    float32,
    float64,
    int32,
    sequence,
    uint8,
    uint32,
)

# =====================================================================
# builtin_interfaces
# =====================================================================


@dataclass
class Time(IdlStruct, typename="builtin_interfaces::msg::dds_::Time_"):
    """builtin_interfaces/msg/Time."""

    sec: int32
    nanosec: uint32


@dataclass
class Duration(IdlStruct, typename="builtin_interfaces::msg::dds_::Duration_"):
    """builtin_interfaces/msg/Duration."""

    sec: int32
    nanosec: uint32


# =====================================================================
# std_msgs
# =====================================================================


@dataclass
class Header(IdlStruct, typename="std_msgs::msg::dds_::Header_"):
    """std_msgs/msg/Header."""

    stamp: Time
    frame_id: str


# =====================================================================
# geometry_msgs
# =====================================================================


@dataclass
class Point(IdlStruct, typename="geometry_msgs::msg::dds_::Point_"):
    """geometry_msgs/msg/Point."""

    x: float64
    y: float64
    z: float64


@dataclass
class Quaternion(IdlStruct, typename="geometry_msgs::msg::dds_::Quaternion_"):
    """geometry_msgs/msg/Quaternion."""

    x: float64
    y: float64
    z: float64
    w: float64


@dataclass
class Vector3(IdlStruct, typename="geometry_msgs::msg::dds_::Vector3_"):
    """geometry_msgs/msg/Vector3."""

    x: float64
    y: float64
    z: float64


@dataclass
class Pose(IdlStruct, typename="geometry_msgs::msg::dds_::Pose_"):
    """geometry_msgs/msg/Pose."""

    position: Point
    orientation: Quaternion


@dataclass
class PoseWithCovariance(
    IdlStruct, typename="geometry_msgs::msg::dds_::PoseWithCovariance_"
):
    """geometry_msgs/msg/PoseWithCovariance."""

    pose: Pose
    covariance: array[float64, 36]  # type: ignore[type-arg,valid-type]


@dataclass
class PoseWithCovarianceStamped(
    IdlStruct,
    typename="geometry_msgs::msg::dds_::PoseWithCovarianceStamped_",
):
    """geometry_msgs/msg/PoseWithCovarianceStamped."""

    header: Header
    pose: PoseWithCovariance


@dataclass
class Twist(IdlStruct, typename="geometry_msgs::msg::dds_::Twist_"):
    """geometry_msgs/msg/Twist."""

    linear: Vector3
    angular: Vector3


@dataclass
class TwistStamped(IdlStruct, typename="geometry_msgs::msg::dds_::TwistStamped_"):
    """geometry_msgs/msg/TwistStamped."""

    header: Header
    twist: Twist


# =====================================================================
# autoware_map_msgs
# =====================================================================


@dataclass
class LaneletMapBin(IdlStruct, typename="autoware_map_msgs::msg::dds_::LaneletMapBin_"):
    """autoware_map_msgs/msg/LaneletMapBin."""

    header: Header
    format_version: str
    map_version: str
    data: sequence[uint8]  # type: ignore[type-arg]


# =====================================================================
# autoware_vehicle_msgs
# =====================================================================


@dataclass
class Engage(IdlStruct, typename="autoware_vehicle_msgs::msg::dds_::Engage_"):
    """autoware_vehicle_msgs/msg/Engage."""

    stamp: Time
    engage: bool


@dataclass
class GearCommand(IdlStruct, typename="autoware_vehicle_msgs::msg::dds_::GearCommand_"):
    """autoware_vehicle_msgs/msg/GearCommand."""

    stamp: Time
    command: uint8


@dataclass
class TurnIndicatorsCommand(
    IdlStruct,
    typename="autoware_vehicle_msgs::msg::dds_::TurnIndicatorsCommand_",
):
    """autoware_vehicle_msgs/msg/TurnIndicatorsCommand."""

    stamp: Time
    command: uint8


@dataclass
class HazardLightsCommand(
    IdlStruct,
    typename="autoware_vehicle_msgs::msg::dds_::HazardLightsCommand_",
):
    """autoware_vehicle_msgs/msg/HazardLightsCommand."""

    stamp: Time
    command: uint8


# =====================================================================
# autoware_control_msgs
# =====================================================================


@dataclass
class AckermannLateralCommand(
    IdlStruct,
    typename="autoware_control_msgs::msg::dds_::AckermannLateralCommand_",
):
    """autoware_control_msgs/msg/AckermannLateralCommand."""

    stamp: Time
    steering_tire_angle: float32
    steering_tire_rotation_rate: float32


@dataclass
class LongitudinalCommand(
    IdlStruct,
    typename="autoware_control_msgs::msg::dds_::LongitudinalCommand_",
):
    """autoware_control_msgs/msg/LongitudinalCommand."""

    stamp: Time
    speed: float32
    acceleration: float32
    jerk: float32


@dataclass
class AckermannControlCommand(
    IdlStruct,
    typename="autoware_control_msgs::msg::dds_::AckermannControlCommand_",
):
    """autoware_control_msgs/msg/AckermannControlCommand."""

    stamp: Time
    lateral: AckermannLateralCommand
    longitudinal: LongitudinalCommand


# =====================================================================
# tier4_vehicle_msgs
# =====================================================================


@dataclass
class ActuationCommand(
    IdlStruct, typename="tier4_vehicle_msgs::msg::dds_::ActuationCommand_"
):
    """tier4_vehicle_msgs/msg/ActuationCommand."""

    accel_cmd: float64
    brake_cmd: float64
    steer_cmd: float64


@dataclass
class ActuationCommandStamped(
    IdlStruct,
    typename="tier4_vehicle_msgs::msg::dds_::ActuationCommandStamped_",
):
    """tier4_vehicle_msgs/msg/ActuationCommandStamped."""

    header: Header
    actuation: ActuationCommand


# =====================================================================
# autoware_planning_msgs
# =====================================================================


@dataclass
class TrajectoryPoint(
    IdlStruct,
    typename="autoware_planning_msgs::msg::dds_::TrajectoryPoint_",
):
    """autoware_planning_msgs/msg/TrajectoryPoint."""

    time_from_start: Duration
    pose: Pose
    longitudinal_velocity_mps: float32
    lateral_velocity_mps: float32
    acceleration_mps2: float32
    heading_rate_rps: float32
    front_wheel_angle_rad: float32
    rear_wheel_angle_rad: float32


@dataclass
class Trajectory(IdlStruct, typename="autoware_planning_msgs::msg::dds_::Trajectory_"):
    """autoware_planning_msgs/msg/Trajectory."""

    header: Header
    points: sequence[TrajectoryPoint]  # type: ignore[type-arg]
