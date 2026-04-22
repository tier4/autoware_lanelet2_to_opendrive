"""Autoware ego vehicle controlled via DDS topic I/O.

DDS communication is delegated to :class:`AutowareDDSBridge`.
This class handles CARLA actor control and coordinate conversion.
"""

from __future__ import annotations

import logging
import math
from typing import TYPE_CHECKING, Any, Optional

if TYPE_CHECKING:
    import carla

from ..dds.msg import Time
from .autoware_dds_bridge import AutowareDDSBridge
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


class AutowareEntity(EgoVehicle):
    """Ego vehicle controlled by Autoware instead of TrafficManager.

    Lifecycle
    ---------
    1. ``spawn(world, config)`` – create the CARLA actor (inherited).
    2. ``setup_dds()``          – initialise the DDS bridge.
    3. Per-tick (registered as post-tick callback by ScenarioRunner):
       ``apply_control()`` applies received commands and publishes state.
    4. ``destroy()`` – tear down DDS and the CARLA actor.

    Initialisation (``initialpose`` / ``initialtwist``) is handled by
    :class:`~autoware_carla_scenario.scenario_base.BaseScenario` when
    ``initialize_with_dds=True``.
    """

    use_autopilot: bool = False

    def __init__(
        self,
        domain_id: int = 0,
        topic_prefix: str = "",
    ) -> None:
        super().__init__()
        self._dds = AutowareDDSBridge(domain_id=domain_id, topic_prefix=topic_prefix)
        self._last_light_state: Optional[int] = None

    # ------------------------------------------------------------------
    # Properties (delegate to bridge)
    # ------------------------------------------------------------------

    @property
    def is_engaged(self) -> bool:
        return self._dds.is_engaged

    @property
    def control_mode(self) -> int:
        return self._dds.control_mode

    # ------------------------------------------------------------------
    # DDS lifecycle
    # ------------------------------------------------------------------

    def setup_dds(self) -> None:
        """Initialise the DDS bridge."""
        self._dds.setup()

    def publish_engage(self, value: bool) -> None:
        """Publish an Engage message via DDS."""
        self._dds.publish_engage(value)

    # ------------------------------------------------------------------
    # Control application (post-tick callback)
    # ------------------------------------------------------------------

    def apply_control(self, world: "carla.World") -> None:
        """Apply received DDS commands to CARLA and publish vehicle state.

        Registered as a ``post_tick`` callback by ScenarioRunner.
        """
        if self._vehicle is None:
            return

        import carla as _carla

        if not self._dds.is_engaged:
            self._vehicle.apply_control(
                _carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=True)
            )
        else:
            gear_cmd = (
                self._dds.current_manual_gear_cmd
                if self._dds.current_manual_gear_cmd is not None
                else self._dds.current_gear_cmd
            )
            is_reverse = gear_cmd is not None and gear_cmd.command in _REVERSE_GEARS
            self._apply_motion_control(_carla, gear_cmd, is_reverse)
            self._apply_lights(_carla, is_reverse)

        self._publish_state()
        self._publish_clock(world)

    def _apply_motion_control(
        self,
        _carla: Any,
        gear_cmd: Any,
        is_reverse: bool,
    ) -> None:
        assert self._vehicle is not None

        if gear_cmd is not None and gear_cmd.command == _GEAR_PARK:
            self._vehicle.apply_control(_carla.VehicleControl(hand_brake=True))
            return

        ackermann_cmd = (
            self._dds.current_manual_ackermann_cmd
            if self._dds.current_manual_ackermann_cmd is not None
            else self._dds.current_ackermann_cmd
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
        assert self._vehicle is not None

        lights = 0

        if self._dds.current_turn_indicators_cmd is not None:
            cmd = self._dds.current_turn_indicators_cmd.command
            if cmd == _TURN_LEFT:
                lights |= int(_carla.VehicleLightState.LeftBlinker)
            elif cmd == _TURN_RIGHT:
                lights |= int(_carla.VehicleLightState.RightBlinker)

        if (
            self._dds.current_hazard_lights_cmd is not None
            and self._dds.current_hazard_lights_cmd.command == _HAZARD_ENABLE
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
    # State publishing
    # ------------------------------------------------------------------

    def _publish_state(self) -> None:
        """Read CARLA vehicle state, convert to MGRS, publish via DDS."""
        assert self._vehicle is not None

        from ..coordinate.map_manager import MapManager

        mm = MapManager.get_instance()
        offset_x, offset_y = mm.mgrs_offset
        z_off = mm.z_offset

        tf = self._vehicle.get_transform()
        vel = self._vehicle.get_velocity()
        accel = self._vehicle.get_acceleration()
        ang_vel = self._vehicle.get_angular_velocity()
        ctrl = self._vehicle.get_control()

        # CARLA → MGRS
        mgrs_x = tf.location.x + offset_x
        mgrs_y = -(tf.location.y) + offset_y
        mgrs_z = tf.location.z + z_off

        yaw_rad = -math.radians(tf.rotation.yaw)
        pitch_rad = math.radians(tf.rotation.pitch)
        roll_rad = math.radians(tf.rotation.roll)

        # Body-frame projections
        fwd = tf.get_forward_vector()
        right = tf.get_right_vector()
        vx_body = vel.x * fwd.x + vel.y * fwd.y + vel.z * fwd.z
        vy_body = vel.x * right.x + vel.y * right.y + vel.z * right.z
        wz = -math.radians(ang_vel.z)
        ax_body = accel.x * fwd.x + accel.y * fwd.y + accel.z * fwd.z
        ay_body = accel.x * right.x + accel.y * right.y + accel.z * right.z

        self._dds.publish_state(
            mgrs_x=mgrs_x,
            mgrs_y=mgrs_y,
            mgrs_z=mgrs_z,
            roll_rad=roll_rad,
            pitch_rad=pitch_rad,
            yaw_rad=yaw_rad,
            vx_body=vx_body,
            vy_body=vy_body,
            wz=wz,
            ax_body=ax_body,
            ay_body=ay_body,
            throttle=float(ctrl.throttle),
            brake=float(ctrl.brake),
            steer=float(ctrl.steer),
        )

    def _publish_clock(self, world: "carla.World") -> None:
        """Publish CARLA simulation time as ``/clock``."""
        ts = world.get_snapshot().timestamp
        self._dds.publish_clock(
            Time(
                sec=int(ts.elapsed_seconds), nanosec=int((ts.elapsed_seconds % 1) * 1e9)
            )
        )

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def destroy(self) -> None:
        """Tear down DDS bridge and CARLA actor."""
        self._dds.destroy()
        super().destroy()
