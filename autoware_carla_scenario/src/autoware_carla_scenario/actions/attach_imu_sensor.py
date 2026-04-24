"""Action that attaches a CARLA IMU sensor to an actor.

Spawns a ``sensor.other.imu`` blueprint attached to the target actor and
registers a Python-side ``sensor.listen()`` callback.  When an optional
*imu_pub* publisher is supplied, every measurement is forwarded to it;
otherwise data is only logged.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import TYPE_CHECKING, Optional, Protocol, Union

from ..conditions import BaseCondition
from ..conditions.base import find_actor_by_role_name
from ..entity_role import EntityRole
from .base import BaseAction, TickTiming

if TYPE_CHECKING:
    import carla

logger = logging.getLogger(__name__)


class ImuPublisher(Protocol):
    """Minimal publish interface expected by :class:`AttachIMUSensorAction`."""

    def publish(
        self,
        *,
        accelerometer: tuple[float, float, float],
        gyroscope: tuple[float, float, float],
    ) -> None: ...


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
    registers a ``sensor.listen()`` callback.  If *imu_pub* is provided,
    each measurement is forwarded to it; errors in the callback are
    caught and logged via ``logger.exception``.

    Args:
        entity_name: ``role_name`` of the actor to attach the sensor to.
        sensor_config: IMU sensor configuration.
        condition: Trigger condition (see :class:`BaseCondition`).
        timing: Tick phase (``PRE_TICK`` or ``POST_TICK``).
        imu_pub: Optional publisher that receives accelerometer/gyroscope
            tuples on every tick.
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
        imu_pub: Optional[ImuPublisher] = None,
        label: str = "attach_imu_sensor",
        once: bool = True,
    ) -> None:
        super().__init__(label=label, condition=condition, timing=timing, once=once)
        self._entity_name = entity_name
        self._config = sensor_config or IMUSensorConfig()
        self._imu_pub = imu_pub
        self._sensor: Optional[carla.Actor] = None

    @property
    def sensor_actor(self) -> Optional["carla.Actor"]:
        """Return the spawned CARLA IMU sensor actor, or ``None``."""
        return self._sensor

    def execute(self, world: "carla.World") -> None:
        """Locate the target actor and attach the IMU sensor."""
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
        self._sensor.listen(self._on_imu_data)

        logger.info(
            "%s: IMU sensor attached to '%s' (tick=%.3fs)",
            type(self).__name__,
            self._entity_name,
            cfg.sensor_tick,
        )

    def _on_imu_data(self, imu_measurement: "carla.IMUMeasurement") -> None:
        """Callback invoked by CARLA for each IMU measurement."""
        try:
            logger.info("IMU callback fired (frame=%s)", imu_measurement.frame)
            acc = imu_measurement.accelerometer
            logger.info("IMU accelerometer accessed: (%s,%s,%s)", acc.x, acc.y, acc.z)
            gyro = imu_measurement.gyroscope
            logger.info("IMU gyroscope accessed: (%s,%s,%s)", gyro.x, gyro.y, gyro.z)
            # NOTE: publish disabled for SIGSEGV debugging
            # if self._imu_pub is not None:
            #     self._imu_pub.publish(
            #         accelerometer=(acc.x, acc.y, acc.z),
            #         gyroscope=(gyro.x, gyro.y, gyro.z),
            #     )
        except Exception:
            logger.exception("IMU callback failed")

    def destroy(self) -> None:
        """Destroy the IMU sensor actor."""
        if self._sensor is not None:
            self._sensor.stop()
            self._sensor.destroy()
            self._sensor = None
            logger.info("IMU sensor destroyed")
