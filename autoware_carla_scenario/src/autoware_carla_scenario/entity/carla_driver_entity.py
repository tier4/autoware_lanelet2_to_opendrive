"""Ego vehicle driven by an external policy over the ``egodriver`` gRPC contract.

Where :class:`~autoware_carla_scenario.entity.ego.EgoVehicle` hands the vehicle to
CARLA's TrafficManager and :class:`~autoware_carla_scenario.entity.autoware_entity.AutowareEntity`
leaves it standing still, this entity closes the loop against a driver policy served by
`carla_driver_interface <https://github.com/hakuturu583/carla_driver_interface>`_ or any
other implementation of ``egodriver.EgodriverService``.

Each simulation tick the entity applies control; every
:attr:`~autoware_carla_scenario.driver.base.DriverClientConfig.policy_timestep_s` it also
submits fresh observations and asks the policy to re-plan.  Between policy steps the most
recent plan keeps being tracked, which is how the upstream runtime handles a policy that
runs slower than the simulation.
"""

from __future__ import annotations

import logging
import uuid
from typing import TYPE_CHECKING, Dict, List, Optional, Tuple

from ..driver.base import BaseEgoDriverClient, DriverClientConfig, EgoObservation
from ..driver.control import ControlConfig, TrajectoryFollower
from ..driver.egodriver_client import EgoDriverGrpcClient
from ..driver.geometry import Trajectory
from ..driver.renderer import RendererDataBuilder
from ..driver.observation import (
    ego_observation,
    encode_frame_jpeg,
    rear_axle_offset,
    route_waypoints_in_rig,
)
from .ego import EgoVehicle

if TYPE_CHECKING:
    import carla

    from ..sensor.carla_camera import CarlaCameraSensor


logger = logging.getLogger(__name__)

#: Simulation step used to convert ticks to simulation time.  Matches the
#: ``fixed_delta_seconds`` :class:`ScenarioRunner` applies to the world.
_FIXED_DELTA_S: float = 0.05

#: Microseconds per second, for the contract's ``fixed64`` timestamps.
_US_PER_S: int = 1_000_000

#: Log the policy's own diagnostics every N policy steps (~1 s at 10 Hz).
_DEBUG_LOG_INTERVAL: int = 10


class CarlaDriverEntity(EgoVehicle):
    """Ego vehicle controlled by an external driver policy.

    Args:
        config: Driver connection settings.  ``None`` uses the defaults, which dial
            ``localhost:50051``.
        control_config: Gains for the trajectory follower.  ``None`` uses the defaults.
        client: Pre-built driver client, used by tests to substitute a fake or an
            in-process gRPC channel.  ``None`` builds an
            :class:`~autoware_carla_scenario.driver.egodriver_client.EgoDriverGrpcClient`
            from *config*.
    """

    #: The policy drives; TrafficManager must keep its hands off this actor.
    use_autopilot: bool = False

    def __init__(
        self,
        config: Optional[DriverClientConfig] = None,
        control_config: Optional[ControlConfig] = None,
        client: Optional[BaseEgoDriverClient] = None,
    ) -> None:
        super().__init__()
        self._config = config or DriverClientConfig()
        self._client = (
            client if client is not None else EgoDriverGrpcClient(self._config)
        )
        self._follower = TrajectoryFollower(control_config)

        self._cameras: List[Tuple[str, "CarlaCameraSensor"]] = []
        self._plan: Trajectory = Trajectory.empty()
        self._sim_time_us: int = 0
        self._last_policy_time_us: Optional[int] = None
        self._rear_axle_offset_m: float = 0.0
        self._termination_requested: bool = False
        self._session_open: bool = False
        self._drive_count: int = 0
        self._map: Optional["carla.Map"] = None
        self._renderer: Optional[RendererDataBuilder] = None

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def config(self) -> DriverClientConfig:
        """Return the driver connection settings."""
        return self._config

    @property
    def client(self) -> BaseEgoDriverClient:
        """Return the driver client."""
        return self._client

    @property
    def termination_requested(self) -> bool:
        """Whether the policy asked to end the session early."""
        return self._termination_requested

    @property
    def drive_count(self) -> int:
        """Number of ``drive`` calls made so far."""
        return self._drive_count

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def on_scenario_start(self, world: "carla.World") -> None:
        """Attach the cameras, open the driver session, and send the initial route.

        Raises:
            RuntimeError: If the ego actor has not been spawned yet.
        """
        actor = self.actor
        if actor is None:
            raise RuntimeError(
                "CarlaDriverEntity.on_scenario_start called before spawn()"
            )

        self._rear_axle_offset_m = rear_axle_offset(
            actor, self._config.rear_axle_offset_m
        )
        logger.info(
            "Rear-axle offset for the rig origin: %.2f m", self._rear_axle_offset_m
        )

        self._attach_cameras(world, actor)

        # CARLA rebuilds the map object on every ``get_map()`` call, so it is fetched
        # once here and reused for the rolling route walk.
        self._map = world.get_map()

        if self._config.send_renderer_data:
            self._renderer = RendererDataBuilder(world, actor, self._config)

        session_uuid = str(uuid.uuid4())
        self._client.start_session(session_uuid, str(self._map.name))
        self._session_open = True

        self._submit_route(actor, self._ego_observation(actor))

    def on_tick(self, world: "carla.World", elapsed: float) -> None:
        """Advance the driver loop by one simulation tick.

        Observations and a new plan are requested every ``policy_timestep_s``; the
        current plan is tracked and applied on every tick.
        """
        actor = self.actor
        if actor is None or not self._session_open:
            return

        self._sim_time_us += int(round(_FIXED_DELTA_S * _US_PER_S))

        # The actor does not move within a tick, so one observation serves both the
        # policy step and the controller.
        observation = self._ego_observation(actor)

        if self._is_policy_step():
            self._run_policy_step(actor, observation)

        self._apply_control(actor, observation)

    def on_scenario_end(self, world: "carla.World") -> None:
        """Close the driver session and destroy the cameras.

        Teardown failures are logged rather than raised so that one unreachable policy
        cannot abort the rest of the scenario cleanup.
        """
        if self._session_open:
            try:
                self._client.close_session()
            except Exception:  # noqa: BLE001 - teardown must not raise
                logger.warning("Failed to close the driver session", exc_info=True)
            self._session_open = False

        for logical_id, camera in self._cameras:
            try:
                camera.destroy()
            except Exception:  # noqa: BLE001 - teardown must not raise
                logger.warning("Failed to destroy camera %s", logical_id, exc_info=True)
        self._cameras = []
        self._map = None
        self._renderer = None

        logger.info(
            "Driver session finished after %d policy step(s)%s",
            self._drive_count,
            " (terminated by the policy)" if self._termination_requested else "",
        )

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _attach_cameras(self, world: "carla.World", actor: "carla.Actor") -> None:
        """Spawn and attach every configured camera to the ego actor."""
        from ..sensor.carla_camera import CarlaCameraSensor  # noqa: PLC0415

        for logical_id, sensor_config in self._client.camera_sensor_configs():
            camera = CarlaCameraSensor(sensor_config)
            camera.attach(world, actor)
            self._cameras.append((logical_id, camera))
        logger.info("Attached %d driver camera(s) to the ego", len(self._cameras))

    def _ego_observation(self, actor: "carla.Actor") -> EgoObservation:
        """Return the ego's state at the current simulation time."""
        return ego_observation(actor, self._sim_time_us, self._rear_axle_offset_m)

    def _submit_route(self, actor: "carla.Actor", observation: EgoObservation) -> None:
        """Send the road ahead of the ego to the policy.

        The route is a rolling window: it is re-sent on every policy step so the policy
        always sees ``route_horizon_m`` of road ahead, rather than running off the end of
        a route computed once at spawn.
        """
        if self._map is None:
            return
        waypoints = route_waypoints_in_rig(
            self._map,
            actor,
            observation.pose,
            horizon_m=self._config.route_horizon_m,
            resolution_m=self._config.route_resolution_m,
        )
        self._client.submit_route(self._sim_time_us, waypoints)

    def _is_policy_step(self) -> bool:
        """Whether this tick should query the policy."""
        if self._last_policy_time_us is None:
            return True
        interval_us = int(round(self._config.policy_timestep_s * _US_PER_S))
        return self._sim_time_us - self._last_policy_time_us >= interval_us

    def _run_policy_step(
        self, actor: "carla.Actor", observation: EgoObservation
    ) -> None:
        """Submit observations, ask the policy to plan, and cache the result."""
        self._submit_camera_frames()
        self._submit_route(actor, observation)
        self._client.submit_egomotion_observation(observation)

        query_us = self._sim_time_us + int(
            round(self._config.policy_timestep_s * _US_PER_S)
        )
        outcome = self._client.drive(
            self._sim_time_us,
            query_us,
            self._renderer_data(observation),
        )
        self._plan = outcome.trajectory
        self._drive_count += 1
        self._last_policy_time_us = self._sim_time_us

        if outcome.debug_scalars and self._drive_count % _DEBUG_LOG_INTERVAL == 0:
            logger.info(
                "[%s] step %d: %s",
                outcome.policy_name or "driver",
                self._drive_count,
                " ".join(
                    f"{key}={value:.3f}"
                    for key, value in sorted(outcome.debug_scalars.items())
                ),
            )

        if outcome.terminate_session and not self._termination_requested:
            logger.info("Driver policy requested session termination")
            self._termination_requested = True

    def _renderer_data(self, observation: EgoObservation) -> bytes:
        """Return the CARLA ground-truth payload for this policy step.

        Empty when ``send_renderer_data`` is off.  Note that a policy cannot tell
        that apart from "nothing to report": both read as no traffic light and no
        other vehicles, so turning this off quietly weakens any policy that
        reasons about them.
        """
        if self._renderer is None:
            return b""
        return self._renderer.build(self._sim_time_us, observation.pose)

    def _submit_camera_frames(self) -> None:
        """Encode and send the latest frame from each camera."""
        for logical_id, camera in self._cameras:
            image = camera.get_image()
            if image is None:
                logger.debug("No frame available from camera %s", logical_id)
                continue
            encoded = encode_frame_jpeg(image, self._config.image_quality)
            if encoded is None:
                continue
            self._client.submit_image_observation(
                logical_id,
                self._sim_time_us,
                self._sim_time_us,
                encoded,
            )

    def _apply_control(self, actor: "carla.Actor", observation: EgoObservation) -> None:
        """Track the cached plan and push the resulting control to CARLA."""
        command = self._follower.step(
            self._plan,
            observation.pose,
            observation.speed_mps,
            _FIXED_DELTA_S,
        )
        actor.apply_control(command.to_carla_control())

    def debug_state(self) -> Dict[str, float]:
        """Return a snapshot of the loop's state, for logging and tests."""
        return {
            "sim_time_us": float(self._sim_time_us),
            "drive_count": float(self._drive_count),
            "plan_length": float(len(self._plan)),
            "rear_axle_offset_m": self._rear_axle_offset_m,
        }
