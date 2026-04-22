"""Autoware ego vehicle controlled via DDS topic I/O.

This entity subscribes to the same input topics as Autoware's
``simple_planning_simulator`` node.

**Per-frame topics** (control commands, trajectory) are synchronised
with a :class:`~cyclonedds.core.WaitSet` — the tick loop blocks in
:meth:`wait_for_frame_data` until at least one new sample arrives.

**Event-driven topics** (engage, gear, indicators, …) are handled by
:class:`~cyclonedds.core.Listener` callbacks that store the latest
sample asynchronously as soon as it is published.
"""

from __future__ import annotations

import logging
import math
import time
from typing import TYPE_CHECKING, Any, Callable, Optional

if TYPE_CHECKING:
    import carla

from cyclonedds.core import (
    GuardCondition,
    Listener,
    ReadCondition,
    SampleState,
    WaitSet,
)
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
from .ego import EgoVehicle

logger = logging.getLogger(__name__)

# -- Autoware GearCommand constants ------------------------------------
_GEAR_REVERSE: int = 20
_GEAR_REVERSE_2: int = 21
_GEAR_PARK: int = 22
_REVERSE_GEARS: frozenset[int] = frozenset({_GEAR_REVERSE, _GEAR_REVERSE_2})

# -- Autoware TurnIndicatorsCommand constants --------------------------
_TURN_LEFT: int = 2
_TURN_RIGHT: int = 3

# -- Autoware HazardLightsCommand constants ----------------------------
_HAZARD_ENABLE: int = 2

# -- Autoware ControlModeCommand constants -----------------------------
_CONTROL_MODE_AUTONOMOUS: int = 1
_CONTROL_MODE_MANUAL: int = 4

# -- tier4_external_api_msgs ResponseStatus constants ------------------
_RESPONSE_SUCCESS: int = 1

#: WaitSet poll interval in nanoseconds (1 second).
_WAIT_POLL_NS: int = 1_000_000_000


# ------------------------------------------------------------------
# Topic specification
# ------------------------------------------------------------------


class _TopicSpec:
    """Descriptor for a single DDS input topic."""

    __slots__ = ("name", "msg_type", "qos", "per_frame", "attr")

    def __init__(
        self,
        name: str,
        msg_type: type,
        qos: Any = None,
        *,
        per_frame: bool = False,
        attr: str | None = None,
    ) -> None:
        self.name = name
        self.msg_type = msg_type
        self.qos = qos
        self.per_frame = per_frame
        self.attr = attr


_INPUT_TOPICS: list[_TopicSpec] = [
    # ---- Per-frame runtime topics (WaitSet) ----
    _TopicSpec(
        "ackermann_control_command",
        Control,
        per_frame=True,
        attr="_current_ackermann_cmd",
    ),
    # ---- Event-driven runtime topics (Listener) ----
    _TopicSpec("engage", Engage),
    _TopicSpec(
        "manual_ackermann_control_command",
        Control,
        attr="_current_manual_ackermann_cmd",
    ),
    _TopicSpec("gear_command", GearCommand, attr="_current_gear_cmd"),
    _TopicSpec("manual_gear_command", GearCommand, attr="_current_manual_gear_cmd"),
    _TopicSpec(
        "turn_indicators_command",
        TurnIndicatorsCommand,
        attr="_current_turn_indicators_cmd",
    ),
    _TopicSpec(
        "hazard_lights_command",
        HazardLightsCommand,
        attr="_current_hazard_lights_cmd",
    ),
]


# ------------------------------------------------------------------
# AutowareEntity
# ------------------------------------------------------------------


class AutowareEntity(EgoVehicle):
    """Ego vehicle controlled by Autoware instead of TrafficManager.

    Lifecycle
    ---------
    1. ``spawn(world, config)`` – create the CARLA actor (inherited).
    2. ``setup_dds()``          – create DDS participant, data readers,
       writers, and service servers.
    3. Per-tick (registered as post-tick callback by ScenarioRunner):
       ``apply_control()`` applies received commands and publishes state.
    4. ``destroy()`` – tear down DDS entities and the CARLA actor.

    Initialisation (``initialpose`` / ``initialtwist``) is handled by
    :class:`~autoware_carla_scenario.scenario_base.BaseScenario` when
    ``psim_compatible_mode=True``.
    """

    use_autopilot: bool = False

    def __init__(
        self,
        domain_id: int = 0,
        topic_prefix: str = "",
    ) -> None:
        """Create an Autoware-controlled ego entity.

        Args:
            domain_id: DDS domain ID (must match ``ROS_DOMAIN_ID``).
            topic_prefix: Optional ROS 2 namespace prefix inserted
                between ``rt/`` and ``input/`` in DDS topic names.
                For example ``"simulation"`` produces
                ``rt/simulation/input/engage``.
        """
        super().__init__()
        self._domain_id = domain_id
        self._topic_prefix = topic_prefix

        # --- Runtime command state ---
        self._is_engaged: bool = False
        self._current_ackermann_cmd: Optional[Control] = None
        self._current_manual_ackermann_cmd: Optional[Control] = None
        self._current_gear_cmd: Optional[GearCommand] = None
        self._current_manual_gear_cmd: Optional[GearCommand] = None
        self._current_turn_indicators_cmd: Optional[TurnIndicatorsCommand] = None
        self._current_hazard_lights_cmd: Optional[HazardLightsCommand] = None
        self._last_light_state: Optional[int] = None
        self._control_mode: int = _CONTROL_MODE_AUTONOMOUS

        # --- DDS entities (created by setup_dds) ---
        self._participant: Optional[DomainParticipant] = None
        self._readers: dict[str, DataReader] = {}
        self._writers: dict[str, DataWriter] = {}
        self._shutdown_guard: Optional[GuardCondition] = None
        self._frame_waitset: Optional[WaitSet] = None
        self._frame_conditions: list[ReadCondition] = []
        self._listeners: list[Listener] = []

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def is_engaged(self) -> bool:
        """Whether Autoware has engaged (``True`` → motion enabled)."""
        return self._is_engaged

    def publish_engage(self, value: bool) -> None:
        """Publish an :class:`Engage` message via DDS.

        The entity's own Listener will receive this message and update
        :attr:`is_engaged` accordingly.

        Args:
            value: Engage state to publish.

        Raises:
            RuntimeError: If :meth:`setup_dds` has not been called yet.
        """
        writer = self._writers.get("engage")
        if writer is None:
            raise RuntimeError("Call setup_dds() before publish_engage().")
        now_ns = time.time_ns()
        stamp = Time(sec=now_ns // 1_000_000_000, nanosec=now_ns % 1_000_000_000)
        writer.write(Engage(stamp=stamp, engage=value))
        logger.info("Published engage=%s", value)

    # ------------------------------------------------------------------
    # DDS setup
    # ------------------------------------------------------------------

    def _resolve_topic_name(self, short_name: str) -> str:
        """Build the full DDS topic name.

        ROS 2 topics are mapped to DDS as ``rt/<ns>/input/<name>``.
        """
        if self._topic_prefix:
            return f"rt/{self._topic_prefix}/input/{short_name}"
        return f"rt/input/{short_name}"

    def _make_event_callback(self, spec: _TopicSpec) -> Callable[[DataReader], None]:
        """Build a Listener callback that stores the latest sample."""
        if spec.name == "engage":

            def _on_engage(reader: DataReader) -> None:
                samples = reader.take()
                if samples:
                    self._is_engaged = samples[-1].engage
                    logger.debug("engage=%s", self._is_engaged)

            return _on_engage

        assert spec.attr is not None
        attr = spec.attr

        def _on_event(reader: DataReader) -> None:
            samples = reader.take()
            if samples:
                setattr(self, attr, samples[-1])

        return _on_event

    def setup_dds(self) -> None:
        """Create the DDS domain participant and data readers.

        * **Per-frame topics** get a :class:`ReadCondition` attached to
          ``_frame_waitset`` so :meth:`wait_for_frame_data` can block.
        * **Event-driven topics** get a :class:`Listener` whose
          ``on_data_available`` callback stores the latest sample.
        * **Initialisation topics** get plain readers (polled explicitly
          in :meth:`wait_for_initialization`).

        Idempotent – calling twice is a harmless no-op.
        """
        if self._participant is not None:
            logger.warning("DDS already initialised — skipping setup_dds()")
            return

        self._participant = DomainParticipant(domain_id=self._domain_id)
        self._shutdown_guard = GuardCondition(self._participant)
        self._frame_waitset = WaitSet(self._participant)

        engage_topic: Optional[Topic] = None

        for spec in _INPUT_TOPICS:
            qos = spec.qos or DEFAULT_QOS
            dds_name = self._resolve_topic_name(spec.name)
            topic: Topic = Topic(self._participant, dds_name, spec.msg_type, qos=qos)

            if spec.per_frame:
                reader = DataReader(self._participant, topic, qos=qos)
                rc = ReadCondition(reader, SampleState.NotRead)
                self._frame_waitset.attach(rc)
                self._frame_conditions.append(rc)
            else:
                callback = self._make_event_callback(spec)
                listener = Listener(on_data_available=callback)
                self._listeners.append(listener)
                reader = DataReader(
                    self._participant, topic, qos=qos, listener=listener
                )

            if spec.name == "engage":
                engage_topic = topic

            self._readers[spec.name] = reader
            logger.debug("Subscribed to %s (%s)", spec.name, dds_name)

        # Reuse the engage Topic from the reader loop for the DataWriter.
        assert engage_topic is not None
        self._writers["engage"] = DataWriter(
            self._participant, engage_topic, qos=DEFAULT_QOS
        )

        # -- Service servers (request reader + response writer) --
        self._setup_service(
            "control_mode",
            "rq/control/control_mode_requestRequest",
            "rr/control/control_mode_requestReply",
            ControlModeCommandRequest,
            ControlModeCommandResponse,
            self._on_control_mode_request,
        )
        # -- Output topic writers --
        _output_topics: list[tuple[str, str, type]] = [
            ("velocity", "rt/vehicle/status/velocity_status", VelocityReport),
            ("odometry", "rt/localization/kinematic_state", Odometry),
            (
                "acceleration",
                "rt/localization/acceleration",
                AccelWithCovarianceStamped,
            ),
            (
                "pose",
                "rt/localization/pose_estimator/pose_with_covariance",
                PoseWithCovarianceStamped,
            ),
            ("steering", "rt/vehicle/status/steering_status", SteeringReport),
            (
                "control_mode_report",
                "rt/vehicle/status/control_mode",
                ControlModeReport,
            ),
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
            (
                "actuation_status",
                "rt/vehicle/status/actuation_status",
                ActuationStatusStamped,
            ),
            ("tf", "rt/tf", TFMessage),
        ]
        for key, topic_name, msg_type in _output_topics:
            t: Topic = Topic(self._participant, topic_name, msg_type, qos=DEFAULT_QOS)
            self._writers[f"out/{key}"] = DataWriter(
                self._participant, t, qos=DEFAULT_QOS
            )

        logger.info(
            "DDS setup complete: %d reader(s), %d writer(s) on domain %d",
            len(self._readers),
            len(self._writers),
            self._domain_id,
        )

    def _setup_service(
        self,
        key: str,
        request_topic_name: str,
        reply_topic_name: str,
        request_type: type,
        response_type: type,
        handler: Callable[[DataReader], None],
    ) -> None:
        """Create a DDS service server (request reader + response writer).

        ROS 2 services map to a pair of DDS topics:
        ``rq/<service>Request`` and ``rr/<service>Reply``.
        A :class:`Listener` on the request reader calls *handler*
        asynchronously when a request arrives.
        """
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
        logger.debug("Service server: %s", key)

    # ------------------------------------------------------------------
    # Service handlers
    # ------------------------------------------------------------------

    def _on_control_mode_request(self, reader: DataReader) -> None:
        """Handle ControlModeCommand service requests."""
        samples = reader.take()
        for req in samples:
            self._control_mode = req.mode
            logger.info("ControlModeCommand: mode=%d", req.mode)
            writer = self._writers.get("srv/control_mode/reply")
            if writer is not None:
                writer.write(
                    ControlModeCommandResponse(
                        header=ServiceHeader(guid=req.header.guid, seq=req.header.seq),
                        success=True,
                    )
                )

    @property
    def control_mode(self) -> int:
        """Current control mode (AUTONOMOUS=1, MANUAL=4, etc.)."""
        return self._control_mode

    # ------------------------------------------------------------------
    # Per-frame runtime – WaitSet gated
    # ------------------------------------------------------------------

    def wait_for_frame_data(self, timeout_ns: int = _WAIT_POLL_NS) -> None:
        """Block until at least one per-frame topic has new data.

        After the WaitSet triggers, all per-frame readers are drained
        and the latest sample for each is stored.  Event-driven topics
        are updated automatically by their Listener callbacks.

        Args:
            timeout_ns: Maximum wait in nanoseconds.

        Raises:
            RuntimeError: If :meth:`setup_dds` has not been called yet.
        """
        if self._frame_waitset is None:
            raise RuntimeError("Call setup_dds() before wait_for_frame_data().")

        self._frame_waitset.wait(timeout=timeout_ns)

        for spec in _INPUT_TOPICS:
            if not spec.per_frame or spec.attr is None:
                continue
            samples = self._readers[spec.name].take()
            if samples:
                setattr(self, spec.attr, samples[-1])

    # ------------------------------------------------------------------
    # Control application (post-tick callback)
    # ------------------------------------------------------------------

    def apply_control(self, world: "carla.World") -> None:
        """Apply the latest DDS commands to the CARLA vehicle.

        Intended to be registered as a ``post_tick`` callback on the
        scenario so it runs once per simulation tick on the main thread.
        All command state is populated asynchronously by DDS Listeners
        and the per-frame WaitSet; this method simply reads the latest
        values and forwards them to the CARLA actor.

        When :attr:`is_engaged` is ``False`` the vehicle is actively
        braked to hold it in place.
        """
        if self._vehicle is None:
            return

        import carla as _carla

        if not self._is_engaged:
            self._vehicle.apply_control(
                _carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=True)
            )
        else:
            gear_cmd = (
                self._current_manual_gear_cmd
                if self._current_manual_gear_cmd is not None
                else self._current_gear_cmd
            )
            is_reverse = gear_cmd is not None and gear_cmd.command in _REVERSE_GEARS
            self._apply_motion_control(_carla, gear_cmd, is_reverse)
            self._apply_lights(_carla, is_reverse)

        self._publish_state()

    def _apply_motion_control(
        self,
        _carla: Any,
        gear_cmd: Optional[GearCommand],
        is_reverse: bool,
    ) -> None:
        """Send ackermann control commands to the CARLA vehicle."""
        assert self._vehicle is not None

        if gear_cmd is not None and gear_cmd.command == _GEAR_PARK:
            self._vehicle.apply_control(_carla.VehicleControl(hand_brake=True))
            return

        ackermann_cmd = (
            self._current_manual_ackermann_cmd
            if self._current_manual_ackermann_cmd is not None
            else self._current_ackermann_cmd
        )
        if ackermann_cmd is None:
            return

        speed = float(ackermann_cmd.longitudinal.velocity)
        if is_reverse:
            speed = -abs(speed)
        self._vehicle.apply_ackermann_control(
            _carla.VehicleAckermannControl(
                steer=float(ackermann_cmd.lateral.steering_tire_angle),
                steer_speed=float(ackermann_cmd.lateral.steering_tire_rotation_rate),
                speed=speed,
                acceleration=float(ackermann_cmd.longitudinal.acceleration),
                jerk=float(ackermann_cmd.longitudinal.jerk),
            )
        )

    def _apply_lights(self, _carla: Any, is_reverse: bool) -> None:
        """Update CARLA vehicle light state only when it has changed."""
        assert self._vehicle is not None

        lights = 0

        if self._current_turn_indicators_cmd is not None:
            cmd = self._current_turn_indicators_cmd.command
            if cmd == _TURN_LEFT:
                lights |= int(_carla.VehicleLightState.LeftBlinker)
            elif cmd == _TURN_RIGHT:
                lights |= int(_carla.VehicleLightState.RightBlinker)

        if (
            self._current_hazard_lights_cmd is not None
            and self._current_hazard_lights_cmd.command == _HAZARD_ENABLE
        ):
            lights |= int(_carla.VehicleLightState.LeftBlinker) | int(
                _carla.VehicleLightState.RightBlinker
            )

        if is_reverse:
            lights |= int(_carla.VehicleLightState.Reverse)

        if lights != self._last_light_state:
            self._vehicle.set_light_state(_carla.VehicleLightState(lights))
            self._last_light_state = lights

    # ------------------------------------------------------------------
    # State publishing (every tick)
    # ------------------------------------------------------------------

    def _now_stamp(self) -> Time:
        now_ns = time.time_ns()
        return Time(sec=now_ns // 1_000_000_000, nanosec=now_ns % 1_000_000_000)

    @staticmethod
    def _euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
        """Convert RPY (radians) to a Quaternion."""
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        return Quaternion(
            x=sr * cp * cy - cr * sp * sy,
            y=cr * sp * cy + sr * cp * sy,
            z=cr * cp * sy - sr * sp * cy,
            w=cr * cp * cy + sr * sp * sy,
        )

    @staticmethod
    def _zero_covariance_36() -> list[float]:
        return [0.0] * 36

    def _publish_state(self) -> None:
        """Read CARLA vehicle state and publish all output topics."""
        assert self._vehicle is not None

        from ..coordinate.map_manager import MapManager

        stamp = self._now_stamp()
        mm = MapManager.get_instance()
        offset_x, offset_y = mm.mgrs_offset
        z_off = mm.z_offset

        # -- Read CARLA state --
        tf = self._vehicle.get_transform()
        vel = self._vehicle.get_velocity()
        accel = self._vehicle.get_acceleration()
        ang_vel = self._vehicle.get_angular_velocity()
        ctrl = self._vehicle.get_control()

        # -- CARLA → MGRS position --
        mgrs_x = tf.location.x + offset_x
        mgrs_y = -(tf.location.y) + offset_y
        mgrs_z = tf.location.z + z_off

        # -- CARLA → MGRS orientation (right-hand) --
        yaw_rad = -math.radians(tf.rotation.yaw)
        pitch_rad = math.radians(tf.rotation.pitch)
        roll_rad = math.radians(tf.rotation.roll)
        q = self._euler_to_quaternion(roll_rad, pitch_rad, yaw_rad)

        # -- Body-frame velocity (project world velocity onto vehicle axes) --
        fwd = tf.get_forward_vector()
        right = tf.get_right_vector()
        vx_body = vel.x * fwd.x + vel.y * fwd.y + vel.z * fwd.z
        vy_body = vel.x * right.x + vel.y * right.y + vel.z * right.z
        # Heading rate: CARLA angular_velocity.z in deg/s → rad/s, flip sign
        wz = -math.radians(ang_vel.z)

        # -- Body-frame acceleration --
        ax_body = accel.x * fwd.x + accel.y * fwd.y + accel.z * fwd.z
        ay_body = accel.x * right.x + accel.y * right.y + accel.z * right.z

        # -- Shared header --
        map_header = Header(stamp=stamp, frame_id="map")
        bl_header = Header(stamp=stamp, frame_id="base_link")

        # -- Build odometry --
        pose_cov = self._zero_covariance_36()
        for i in (0, 7, 14):
            pose_cov[i] = 0.0225  # position covariance
        for i in (21, 28, 35):
            pose_cov[i] = 0.000625  # orientation covariance

        twist_cov = self._zero_covariance_36()
        for i in (0, 7, 14, 21, 28, 35):
            twist_cov[i] = 0.001

        odom = Odometry(
            header=map_header,
            child_frame_id="base_link",
            pose=PoseWithCovariance(
                pose=Pose(
                    position=Point(x=mgrs_x, y=mgrs_y, z=mgrs_z),
                    orientation=q,
                ),
                covariance=pose_cov,  # type: ignore[arg-type]
            ),
            twist=TwistWithCovariance(
                twist=Twist(
                    linear=Vector3(x=vx_body, y=vy_body, z=0.0),
                    angular=Vector3(x=0.0, y=0.0, z=wz),
                ),
                covariance=twist_cov,  # type: ignore[arg-type]
            ),
        )

        w = self._writers

        # 1. Odometry
        writer = w.get("out/odometry")
        if writer is not None:
            writer.write(odom)

        # 2. Pose
        writer = w.get("out/pose")
        if writer is not None:
            writer.write(PoseWithCovarianceStamped(header=map_header, pose=odom.pose))

        # 3. Velocity
        writer = w.get("out/velocity")
        if writer is not None:
            writer.write(
                VelocityReport(
                    header=bl_header,
                    longitudinal_velocity=float(vx_body),
                    lateral_velocity=float(vy_body),
                    heading_rate=float(wz),
                )
            )

        # 4. Acceleration
        accel_cov = self._zero_covariance_36()
        for i in (0, 7, 14, 21, 28, 35):
            accel_cov[i] = 0.001

        writer = w.get("out/acceleration")
        if writer is not None:
            writer.write(
                AccelWithCovarianceStamped(
                    header=bl_header,
                    accel=AccelWithCovariance(
                        accel=Accel(
                            linear=Vector3(x=ax_body, y=ay_body, z=0.0),
                            angular=Vector3(x=0.0, y=0.0, z=0.0),
                        ),
                        covariance=accel_cov,  # type: ignore[arg-type]
                    ),
                )
            )

        # 5. Steering
        steer_angle = 0.0
        ackermann_cmd = (
            self._current_manual_ackermann_cmd
            if self._current_manual_ackermann_cmd is not None
            else self._current_ackermann_cmd
        )
        if ackermann_cmd is not None:
            steer_angle = float(ackermann_cmd.lateral.steering_tire_angle)

        writer = w.get("out/steering")
        if writer is not None:
            writer.write(SteeringReport(stamp=stamp, steering_tire_angle=steer_angle))

        # 6. Control mode report
        writer = w.get("out/control_mode_report")
        if writer is not None:
            writer.write(ControlModeReport(stamp=stamp, mode=self._control_mode))

        # 7. Gear report
        gear_cmd = (
            self._current_manual_gear_cmd
            if self._current_manual_gear_cmd is not None
            else self._current_gear_cmd
        )
        writer = w.get("out/gear_report")
        if writer is not None:
            writer.write(
                GearReport(stamp=stamp, report=gear_cmd.command if gear_cmd else 0)
            )

        # 8. Turn indicators report (skip if never received)
        writer = w.get("out/turn_indicators_report")
        if writer is not None and self._current_turn_indicators_cmd is not None:
            writer.write(
                TurnIndicatorsReport(
                    stamp=stamp, report=self._current_turn_indicators_cmd.command
                )
            )

        # 9. Hazard lights report (skip if never received)
        writer = w.get("out/hazard_lights_report")
        if writer is not None and self._current_hazard_lights_cmd is not None:
            writer.write(
                HazardLightsReport(
                    stamp=stamp, report=self._current_hazard_lights_cmd.command
                )
            )

        # 10. Actuation status
        writer = w.get("out/actuation_status")
        if writer is not None:
            writer.write(
                ActuationStatusStamped(
                    header=bl_header,
                    status=ActuationStatus(
                        accel_status=float(ctrl.throttle),
                        brake_status=float(ctrl.brake),
                        steer_status=float(ctrl.steer),
                    ),
                )
            )

        # 11. TF (map → base_link)
        writer = w.get("out/tf")
        if writer is not None:
            writer.write(
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
    # Shutdown / cleanup
    # ------------------------------------------------------------------

    def request_shutdown(self) -> None:
        """Signal the initialisation wait-loop to exit early."""
        if self._shutdown_guard is not None:
            self._shutdown_guard.set(True)

    def destroy(self) -> None:
        """Tear down DDS entities and destroy the CARLA actor."""
        self._frame_conditions.clear()
        self._listeners.clear()
        self._frame_waitset = None
        self._writers.clear()
        self._readers.clear()
        self._shutdown_guard = None
        self._participant = None
        super().destroy()
