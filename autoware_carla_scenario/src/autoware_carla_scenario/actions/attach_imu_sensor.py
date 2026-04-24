"""Action that attaches a CARLA IMU sensor to an actor.

Spawns a ``sensor.other.imu`` blueprint attached to the target actor and
registers a Python-side ``sensor.listen()`` callback.  When an optional
*dds_participant* is supplied, every measurement is published as a
``sensor_msgs/Imu`` message via CycloneDDS; otherwise data is only logged.
"""

import logging
import time as _time
from dataclasses import dataclass
from typing import TYPE_CHECKING, Optional, Union

from ..conditions import BaseCondition
from ..conditions.base import find_actor_by_role_name
from ..entity_role import EntityRole
from .base import BaseAction, TickTiming

if TYPE_CHECKING:
    import carla
    from cyclonedds.domain import DomainParticipant
    from cyclonedds.pub import DataWriter

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class IMUSensorConfig:
    """Configuration for a CARLA ``sensor.other.imu``.

    See `CARLA sensor reference
    <https://carla.readthedocs.io/en/latest/ref_sensors/#imu-sensor>`_
    for full attribute documentation.

    Extrinsic fields describe the 6-DOF pose of the sensor relative to
    the parent actor's origin.
    """

    sensor_tick: float = 0.0
    """Seconds between sensor captures (0.0 = every simulation step)."""

    noise_accel_stddev_x: float = 0.0
    """Standard deviation of accelerometer noise on the X axis (m/s^2)."""

    noise_accel_stddev_y: float = 0.0
    """Standard deviation of accelerometer noise on the Y axis (m/s^2)."""

    noise_accel_stddev_z: float = 0.0
    """Standard deviation of accelerometer noise on the Z axis (m/s^2)."""

    noise_gyro_bias_x: float = 0.0
    """Bias of gyroscope noise on the X axis (rad/s)."""

    noise_gyro_bias_y: float = 0.0
    """Bias of gyroscope noise on the Y axis (rad/s)."""

    noise_gyro_bias_z: float = 0.0
    """Bias of gyroscope noise on the Z axis (rad/s)."""

    noise_gyro_stddev_x: float = 0.0
    """Standard deviation of gyroscope noise on the X axis (rad/s)."""

    noise_gyro_stddev_y: float = 0.0
    """Standard deviation of gyroscope noise on the Y axis (rad/s)."""

    noise_gyro_stddev_z: float = 0.0
    """Standard deviation of gyroscope noise on the Z axis (rad/s)."""

    noise_seed: int = 0
    """Seed for the noise random number generator (0 = non-deterministic)."""

    # -- Identity ------------------------------------------------------------
    role_name: str = "imu"
    """``role_name`` attribute set on the sensor blueprint.

    CARLA's native ROS 2 bridge uses this value to build the DDS topic
    name (e.g. ``rt/carla/<parent_role_name>/<role_name>``).  Use ROS 2
    launch remapping to map it to the desired topic.
    """

    # -- Extrinsics (parent actor -> IMU) ------------------------------------
    position_x: float = 0.0
    """Forward offset from the parent actor in metres."""

    position_y: float = 0.0
    """Lateral offset from the parent actor in metres."""

    position_z: float = 0.0
    """Vertical offset from the parent actor in metres."""

    roll: float = 0.0
    """Roll angle in degrees."""

    pitch: float = 0.0
    """Pitch angle in degrees."""

    yaw: float = 0.0
    """Yaw angle in degrees."""


class AttachIMUSensorAction(BaseAction):
    """Attach a CARLA ``sensor.other.imu`` to a scenario actor.

    When the trigger condition is met the action spawns the IMU sensor
    blueprint, attaches it to the actor identified by *entity_name*, and
    registers a ``sensor.listen()`` callback.

    If *dds_participant* is provided, each measurement is published as a
    ``sensor_msgs/Imu`` message via CycloneDDS.  Otherwise data is only
    logged (backward-compatible behaviour).

    Args:
        entity_name: ``role_name`` of the actor to attach the sensor to.
        sensor_config: IMU sensor configuration.
        condition: Trigger condition (see :class:`BaseCondition`).
        timing: Tick phase (``PRE_TICK`` or ``POST_TICK``).
        dds_participant: Optional CycloneDDS ``DomainParticipant``.
            When supplied, a ``DataWriter`` is created for the IMU topic
            and measurements are published automatically.
        imu_topic: DDS topic name for the IMU messages.
        frame_id: ``frame_id`` value written into the message header.
        label: Human-readable label for logging.
        once: If ``True`` (default) the action fires at most once.
    """

    def __init__(
        self,
        entity_name: Union[EntityRole, str],
        sensor_config: Optional[IMUSensorConfig] = None,
        condition: Optional[BaseCondition] = None,
        timing: TickTiming = TickTiming.POST_TICK,
        *,
        dds_participant: Optional["DomainParticipant"] = None,
        imu_topic: str = "/sensing/imu/imu_data",
        frame_id: str = "base_link",
        label: str = "attach_imu_sensor",
        once: bool = True,
    ) -> None:
        super().__init__(label=label, condition=condition, timing=timing, once=once)
        self._entity_name = entity_name
        self._config = sensor_config or IMUSensorConfig()
        self._dds_participant = dds_participant
        self._imu_topic = imu_topic
        self._frame_id = frame_id
        self._sensor: Optional[carla.Actor] = None
        self._writer: Optional[DataWriter] = None

    @property
    def sensor_actor(self) -> Optional["carla.Actor"]:
        """Return the spawned CARLA IMU sensor actor, or ``None``."""
        return self._sensor

    def execute(self, world: "carla.World") -> None:
        """Locate the target actor, attach the IMU sensor, and start publishing."""
        actor = find_actor_by_role_name(world, self._entity_name)
        if actor is None:
            logger.warning(
                "%s: actor '%s' not found — skipping",
                type(self).__name__,
                self._entity_name,
            )
            return

        import carla as _carla

        cfg = self._config
        bp_lib = world.get_blueprint_library()
        imu_bp = bp_lib.find("sensor.other.imu")

        imu_bp.set_attribute("sensor_tick", str(cfg.sensor_tick))
        imu_bp.set_attribute("noise_accel_stddev_x", str(cfg.noise_accel_stddev_x))
        imu_bp.set_attribute("noise_accel_stddev_y", str(cfg.noise_accel_stddev_y))
        imu_bp.set_attribute("noise_accel_stddev_z", str(cfg.noise_accel_stddev_z))
        imu_bp.set_attribute("noise_gyro_bias_x", str(cfg.noise_gyro_bias_x))
        imu_bp.set_attribute("noise_gyro_bias_y", str(cfg.noise_gyro_bias_y))
        imu_bp.set_attribute("noise_gyro_bias_z", str(cfg.noise_gyro_bias_z))
        imu_bp.set_attribute("noise_gyro_stddev_x", str(cfg.noise_gyro_stddev_x))
        imu_bp.set_attribute("noise_gyro_stddev_y", str(cfg.noise_gyro_stddev_y))
        imu_bp.set_attribute("noise_gyro_stddev_z", str(cfg.noise_gyro_stddev_z))
        imu_bp.set_attribute("noise_seed", str(cfg.noise_seed))
        imu_bp.set_attribute("role_name", cfg.role_name)

        transform = _carla.Transform(
            _carla.Location(x=cfg.position_x, y=cfg.position_y, z=cfg.position_z),
            _carla.Rotation(roll=cfg.roll, pitch=cfg.pitch, yaw=cfg.yaw),
        )

        self._sensor = world.spawn_actor(imu_bp, transform, attach_to=actor)

        # Set up DDS DataWriter if a participant was supplied
        if self._dds_participant is not None:
            from cyclonedds.pub import DataWriter as _DataWriter
            from cyclonedds.topic import Topic

            from ..dds.msg import Imu
            from ..dds.qos import DEFAULT_QOS

            topic: Topic = Topic(
                self._dds_participant,
                f"rt{self._imu_topic}",
                Imu,
                qos=DEFAULT_QOS,
            )
            self._writer = _DataWriter(self._dds_participant, topic, qos=DEFAULT_QOS)
            logger.info(
                "%s: DDS DataWriter created for topic 'rt%s'",
                type(self).__name__,
                self._imu_topic,
            )

        self._sensor.listen(self._on_imu_data)

        logger.info(
            "%s: IMU sensor attached to '%s' (tick=%.3fs, dds=%s)",
            type(self).__name__,
            self._entity_name,
            cfg.sensor_tick,
            self._writer is not None,
        )

    def _on_imu_data(self, imu_measurement: "carla.IMUMeasurement") -> None:
        """Callback invoked by CARLA for each IMU measurement."""
        try:
            acc = imu_measurement.accelerometer
            gyro = imu_measurement.gyroscope

            if self._writer is not None:
                from ..dds.msg import Header, Imu, Quaternion, Time, Vector3

                now_ns = _time.time_ns()
                stamp = Time(
                    sec=now_ns // 1_000_000_000,
                    nanosec=now_ns % 1_000_000_000,
                )
                msg = Imu(
                    header=Header(stamp=stamp, frame_id=self._frame_id),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                    orientation_covariance=[0.0] * 9,  # type: ignore[arg-type]
                    angular_velocity=Vector3(x=gyro.x, y=gyro.y, z=gyro.z),
                    angular_velocity_covariance=[0.0] * 9,  # type: ignore[arg-type]
                    linear_acceleration=Vector3(x=acc.x, y=acc.y, z=acc.z),
                    linear_acceleration_covariance=[0.0] * 9,  # type: ignore[arg-type]
                )
                self._writer.write(msg)
            else:
                logger.debug(
                    "IMU data (frame=%s): accel=(%.3f,%.3f,%.3f) gyro=(%.3f,%.3f,%.3f)",
                    imu_measurement.frame,
                    acc.x,
                    acc.y,
                    acc.z,
                    gyro.x,
                    gyro.y,
                    gyro.z,
                )
        except Exception:
            logger.exception("IMU callback failed")

    def destroy(self) -> None:
        """Stop listening and destroy the IMU sensor actor."""
        if self._sensor is not None:
            self._sensor.stop()
            self._sensor.destroy()
            self._sensor = None
        self._writer = None
        logger.info("IMU sensor destroyed")
