"""DDS communication bridge for Autoware integration.

Encapsulates all cyclonedds entities (participants, readers, writers,
listeners) and holds the latest received command state.  Used as a
member of :class:`AutowareEntity` to keep DDS concerns separate from
CARLA actor management.
"""

from __future__ import annotations

import logging
import math
import time
from typing import Any, Callable, Optional

from cyclonedds.core import Listener
from cyclonedds.domain import DomainParticipant
from cyclonedds.pub import DataWriter
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic

from ..dds.msg import (
    Accel,
    AccelWithCovariance,
    AccelWithCovarianceStamped,
    ActuationStatus,
    ActuationStatusStamped,
    Control,
    ControlModeCommandRequest,
    ControlModeCommandResponse,
    Clock,
    ControlModeReport,
    Engage,
    GearCommand,
    GearReport,
    HazardLightsCommand,
    HazardLightsReport,
    Header,
    Odometry,
    Point,
    Pose,
    PoseWithCovariance,
    PoseWithCovarianceStamped,
    Quaternion,
    ServiceHeader,
    SteeringReport,
    TFMessage,
    Time,
    Transform,
    TransformStamped,
    TurnIndicatorsCommand,
    TurnIndicatorsReport,
    Twist,
    TwistWithCovariance,
    Vector3,
    VelocityReport,
)
from ..dds.qos import DEFAULT_QOS

logger = logging.getLogger(__name__)

# -- Pre-computed 6×6 covariance diagonals (avoid per-tick allocation) --


def _diag_covariance(*diag: float) -> list[float]:
    cov = [0.0] * 36
    for i, v in enumerate(diag):
        cov[i * 7] = v
    return cov


_POSE_COVARIANCE: list[float] = _diag_covariance(
    0.0225, 0.0225, 0.0225, 0.000625, 0.000625, 0.000625
)
_TWIST_COVARIANCE: list[float] = _diag_covariance(
    0.001, 0.001, 0.001, 0.001, 0.001, 0.001
)
_ACCEL_COVARIANCE: list[float] = _diag_covariance(
    0.001, 0.001, 0.001, 0.001, 0.001, 0.001
)

# -- Autoware ControlModeCommand constants -----------------------------
_CONTROL_MODE_AUTONOMOUS: int = 1

# ------------------------------------------------------------------
# Topic specification
# ------------------------------------------------------------------


class _TopicSpec:
    """Descriptor for a single DDS input topic."""

    __slots__ = ("name", "dds_name", "msg_type", "qos", "attr")

    def __init__(
        self,
        name: str,
        msg_type: type,
        dds_name: str,
        qos: Any = None,
        *,
        attr: str | None = None,
    ) -> None:
        self.name = name
        self.dds_name = dds_name
        self.msg_type = msg_type
        self.qos = qos
        self.attr = attr


_INPUT_TOPICS: list[_TopicSpec] = [
    _TopicSpec(
        "ackermann_control_command",
        Control,
        dds_name="rt/control/command/control_cmd",
        attr="current_ackermann_cmd",
    ),
    _TopicSpec(
        "engage",
        Engage,
        dds_name="rt/vehicle/engage",
    ),
    _TopicSpec(
        "manual_ackermann_control_command",
        Control,
        dds_name="rt/vehicle/command/manual_control_cmd",
        attr="current_manual_ackermann_cmd",
    ),
    _TopicSpec(
        "gear_command",
        GearCommand,
        dds_name="rt/control/command/gear_cmd",
        attr="current_gear_cmd",
    ),
    _TopicSpec(
        "manual_gear_command",
        GearCommand,
        dds_name="rt/vehicle/command/manual_gear_command",
        attr="current_manual_gear_cmd",
    ),
    _TopicSpec(
        "turn_indicators_command",
        TurnIndicatorsCommand,
        dds_name="rt/control/command/turn_indicators_cmd",
        attr="current_turn_indicators_cmd",
    ),
    _TopicSpec(
        "hazard_lights_command",
        HazardLightsCommand,
        dds_name="rt/control/command/hazard_lights_cmd",
        attr="current_hazard_lights_cmd",
    ),
]

_OUTPUT_TOPICS: list[tuple[str, str, type]] = [
    ("velocity", "rt/vehicle/status/velocity_status", VelocityReport),
    ("odometry", "rt/localization/kinematic_state", Odometry),
    ("acceleration", "rt/localization/acceleration", AccelWithCovarianceStamped),
    (
        "pose",
        "rt/localization/pose_estimator/pose_with_covariance",
        PoseWithCovarianceStamped,
    ),
    ("steering", "rt/vehicle/status/steering_status", SteeringReport),
    ("control_mode_report", "rt/vehicle/status/control_mode", ControlModeReport),
    ("gear_report", "rt/vehicle/status/gear_status", GearReport),
    (
        "turn_indicators_report",
        "rt/vehicle/status/turn_indicators_status",
        TurnIndicatorsReport,
    ),
    (
        "hazard_lights_report",
        "rt/vehicle/status/hazard_lights_status",
        HazardLightsReport,
    ),
    ("actuation_status", "rt/vehicle/status/actuation_status", ActuationStatusStamped),
    ("tf", "rt/tf", TFMessage),
    ("clock", "rt/clock", Clock),
]


# ------------------------------------------------------------------
# AutowareDDSBridge
# ------------------------------------------------------------------


class AutowareDDSBridge:
    """DDS communication layer for Autoware integration.

    Manages all cyclonedds entities and holds the latest received
    command state.  The bridge is owned by :class:`AutowareEntity`.
    """

    def __init__(self, domain_id: int = 0) -> None:
        self._domain_id = domain_id

        # --- Received command state (written by DDS callbacks) ---
        self.is_engaged: bool = False
        self.current_ackermann_cmd: Optional[Control] = None
        self.current_manual_ackermann_cmd: Optional[Control] = None
        self.current_gear_cmd: Optional[GearCommand] = None
        self.current_manual_gear_cmd: Optional[GearCommand] = None
        self.current_turn_indicators_cmd: Optional[TurnIndicatorsCommand] = None
        self.current_hazard_lights_cmd: Optional[HazardLightsCommand] = None
        self.control_mode: int = _CONTROL_MODE_AUTONOMOUS

        # --- DDS entities (created by setup) ---
        self._participant: Optional[DomainParticipant] = None
        self._readers: dict[str, DataReader] = {}
        self._writers: dict[str, DataWriter] = {}
        self._listeners: list[Listener] = []

    # ------------------------------------------------------------------
    # Setup
    # ------------------------------------------------------------------

    def setup(self) -> None:
        """Create DDS participant, readers, writers, and service servers.

        Idempotent – calling twice is a harmless no-op.
        """
        if self._participant is not None:
            logger.warning("DDS bridge already initialised — skipping")
            return

        self._participant = DomainParticipant(domain_id=self._domain_id)

        engage_topic: Optional[Topic] = None

        for spec in _INPUT_TOPICS:
            qos = spec.qos or DEFAULT_QOS
            dds_name = spec.dds_name
            topic: Topic = Topic(self._participant, dds_name, spec.msg_type, qos=qos)
            callback = self._make_event_callback(spec)
            listener = Listener(on_data_available=callback)
            self._listeners.append(listener)
            reader = DataReader(self._participant, topic, qos=qos, listener=listener)

            if spec.name == "engage":
                engage_topic = topic

            self._readers[spec.name] = reader
            logger.debug("Subscribed to %s (%s)", spec.name, dds_name)

        assert engage_topic is not None
        self._writers["engage"] = DataWriter(
            self._participant, engage_topic, qos=DEFAULT_QOS
        )

        self._setup_service(
            "control_mode",
            "rq/control/control_mode_requestRequest",
            "rr/control/control_mode_requestReply",
            ControlModeCommandRequest,
            ControlModeCommandResponse,
            self._on_control_mode_request,
        )

        for key, topic_name, msg_type in _OUTPUT_TOPICS:
            t: Topic = Topic(self._participant, topic_name, msg_type, qos=DEFAULT_QOS)
            self._writers[f"out/{key}"] = DataWriter(
                self._participant, t, qos=DEFAULT_QOS
            )

        logger.info(
            "DDS bridge setup: %d reader(s), %d writer(s) on domain %d",
            len(self._readers),
            len(self._writers),
            self._domain_id,
        )

    def _make_event_callback(self, spec: _TopicSpec) -> Callable[[DataReader], None]:
        if spec.name == "engage":

            def _on_engage(reader: DataReader) -> None:
                samples = reader.take()
                if samples:
                    self.is_engaged = samples[-1].engage
                    logger.debug("engage=%s", self.is_engaged)

            return _on_engage

        assert spec.attr is not None
        attr = spec.attr

        def _on_event(reader: DataReader) -> None:
            samples = reader.take()
            if samples:
                setattr(self, attr, samples[-1])

        return _on_event

    def _setup_service(
        self,
        key: str,
        request_topic_name: str,
        reply_topic_name: str,
        request_type: type,
        response_type: type,
        handler: Callable[[DataReader], None],
    ) -> None:
        assert self._participant is not None
        req_topic: Topic = Topic(
            self._participant, request_topic_name, request_type, qos=DEFAULT_QOS
        )
        rep_topic: Topic = Topic(
            self._participant, reply_topic_name, response_type, qos=DEFAULT_QOS
        )
        listener = Listener(on_data_available=handler)
        self._listeners.append(listener)
        self._readers[f"srv/{key}/request"] = DataReader(
            self._participant, req_topic, qos=DEFAULT_QOS, listener=listener
        )
        self._writers[f"srv/{key}/reply"] = DataWriter(
            self._participant, rep_topic, qos=DEFAULT_QOS
        )

    def _on_control_mode_request(self, reader: DataReader) -> None:
        samples = reader.take()
        for req in samples:
            self.control_mode = req.mode
            logger.info("ControlModeCommand: mode=%d", req.mode)
            writer = self._writers.get("srv/control_mode/reply")
            if writer is not None:
                writer.write(
                    ControlModeCommandResponse(
                        header=ServiceHeader(guid=req.header.guid, seq=req.header.seq),
                        success=True,
                    )
                )

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------

    def publish_engage(self, value: bool) -> None:
        """Publish an Engage message via DDS."""
        writer = self._writers.get("engage")
        if writer is None:
            raise RuntimeError("Call setup() first.")
        writer.write(Engage(stamp=self._now_stamp(), engage=value))
        logger.info("Published engage=%s", value)

    def publish_clock(self, sim_time: Time) -> None:
        """Publish a ``/clock`` message with the given simulation time.

        Args:
            sim_time: The current CARLA simulation timestamp.
        """
        writer = self._writers.get("out/clock")
        if writer is not None:
            writer.write(Clock(clock=sim_time))

    @staticmethod
    def _now_stamp() -> Time:
        now_ns = time.time_ns()
        return Time(sec=now_ns // 1_000_000_000, nanosec=now_ns % 1_000_000_000)

    @staticmethod
    def _euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        return Quaternion(
            x=sr * cp * cy - cr * sp * sy,
            y=cr * sp * cy + sr * cp * sy,
            z=cr * cp * sy - sr * sp * cy,
            w=cr * cp * cy + sr * sp * sy,
        )

    def publish_state(
        self,
        *,
        mgrs_x: float,
        mgrs_y: float,
        mgrs_z: float,
        roll_rad: float,
        pitch_rad: float,
        yaw_rad: float,
        vx_body: float,
        vy_body: float,
        wz: float,
        ax_body: float,
        ay_body: float,
        throttle: float,
        brake: float,
        steer: float,
    ) -> None:
        """Publish all output topics from pre-computed values."""
        stamp = self._now_stamp()
        q = self._euler_to_quaternion(roll_rad, pitch_rad, yaw_rad)
        map_header = Header(stamp=stamp, frame_id="map")
        bl_header = Header(stamp=stamp, frame_id="base_link")
        w = self._writers

        # Odometry
        odom = Odometry(
            header=map_header,
            child_frame_id="base_link",
            pose=PoseWithCovariance(
                pose=Pose(
                    position=Point(x=mgrs_x, y=mgrs_y, z=mgrs_z),
                    orientation=q,
                ),
                covariance=_POSE_COVARIANCE,  # type: ignore[arg-type]
            ),
            twist=TwistWithCovariance(
                twist=Twist(
                    linear=Vector3(x=vx_body, y=vy_body, z=0.0),
                    angular=Vector3(x=0.0, y=0.0, z=wz),
                ),
                covariance=_TWIST_COVARIANCE,  # type: ignore[arg-type]
            ),
        )
        wr = w.get("out/odometry")
        if wr is not None:
            wr.write(odom)

        wr = w.get("out/pose")
        if wr is not None:
            wr.write(PoseWithCovarianceStamped(header=map_header, pose=odom.pose))

        wr = w.get("out/velocity")
        if wr is not None:
            wr.write(
                VelocityReport(
                    header=bl_header,
                    longitudinal_velocity=float(vx_body),
                    lateral_velocity=float(vy_body),
                    heading_rate=float(wz),
                )
            )

        wr = w.get("out/acceleration")
        if wr is not None:
            wr.write(
                AccelWithCovarianceStamped(
                    header=bl_header,
                    accel=AccelWithCovariance(
                        accel=Accel(
                            linear=Vector3(x=ax_body, y=ay_body, z=0.0),
                            angular=Vector3(x=0.0, y=0.0, z=0.0),
                        ),
                        covariance=_ACCEL_COVARIANCE,  # type: ignore[arg-type]
                    ),
                )
            )

        # Steering
        steer_angle = 0.0
        ackermann_cmd = (
            self.current_manual_ackermann_cmd
            if self.current_manual_ackermann_cmd is not None
            else self.current_ackermann_cmd
        )
        if ackermann_cmd is not None:
            steer_angle = float(ackermann_cmd.lateral.steering_tire_angle)

        wr = w.get("out/steering")
        if wr is not None:
            wr.write(SteeringReport(stamp=stamp, steering_tire_angle=steer_angle))

        wr = w.get("out/control_mode_report")
        if wr is not None:
            wr.write(ControlModeReport(stamp=stamp, mode=self.control_mode))

        gear_cmd = (
            self.current_manual_gear_cmd
            if self.current_manual_gear_cmd is not None
            else self.current_gear_cmd
        )
        wr = w.get("out/gear_report")
        if wr is not None:
            wr.write(
                GearReport(stamp=stamp, report=gear_cmd.command if gear_cmd else 0)
            )

        wr = w.get("out/turn_indicators_report")
        if wr is not None and self.current_turn_indicators_cmd is not None:
            wr.write(
                TurnIndicatorsReport(
                    stamp=stamp, report=self.current_turn_indicators_cmd.command
                )
            )

        wr = w.get("out/hazard_lights_report")
        if wr is not None and self.current_hazard_lights_cmd is not None:
            wr.write(
                HazardLightsReport(
                    stamp=stamp, report=self.current_hazard_lights_cmd.command
                )
            )

        wr = w.get("out/actuation_status")
        if wr is not None:
            wr.write(
                ActuationStatusStamped(
                    header=bl_header,
                    status=ActuationStatus(
                        accel_status=throttle,
                        brake_status=brake,
                        steer_status=steer,
                    ),
                )
            )

        wr = w.get("out/tf")
        if wr is not None:
            wr.write(
                TFMessage(
                    transforms=[  # type: ignore[arg-type]
                        TransformStamped(
                            header=map_header,
                            child_frame_id="base_link",
                            transform=Transform(
                                translation=Vector3(x=mgrs_x, y=mgrs_y, z=mgrs_z),
                                rotation=q,
                            ),
                        )
                    ]
                )
            )

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def destroy(self) -> None:
        """Tear down all DDS entities."""
        self._listeners.clear()
        self._writers.clear()
        self._readers.clear()
        self._participant = None
