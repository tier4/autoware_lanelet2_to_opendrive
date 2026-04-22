"""Base class for CARLA scenarios."""

from __future__ import annotations

import logging
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import TYPE_CHECKING, Callable, List, Optional, Union

if TYPE_CHECKING:
    from .entity.ego import EgoVehicle

import carla

from .actions import BaseAction
from .conditions import BaseCondition, find_actor_by_role_name
from .constants import DEFAULT_TM_PORT, EGO_ROLE_NAME
from .coordinate import (
    GroundProjectionConfig,
    Lanelet2Pose,
    OpenDrivePose,
    snap_to_carla_road,
    to_opendrive,
)
from .entity._spawn import SpawnLocation, SpawnTransform
from .entity.vehicle_entity import VehicleEntity, VehicleEntityConfig

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class SpectatorCameraConfig:
    """Parameters used by :meth:`BaseScenario.follow_with_spectator`.

    Stored so that :class:`ScenarioRunner` can attach an RGB camera sensor
    at the same position for video recording.
    """

    offset_back: float
    offset_up: float
    pitch: float


class EgoConfig(VehicleEntityConfig):
    """Configuration for the ego vehicle.

    Inherits all fields from :class:`VehicleEntityConfig` (``vehicle_type``,
    ``initial_speed_kmh``, etc.).  The ``role_name`` is automatically set to
    :data:`~autoware_carla_scenario.constants.EGO_ROLE_NAME`.
    """

    def __init__(
        self,
        spawn_location: SpawnLocation,
        vehicle_type: str = "vehicle.mini.cooper",
        initial_speed_kmh: float = 0.0,
        spawn_retry_max_count: int = 0,
        spawn_retry_t_step: float = 0.1,
        spawn_retry_z_step: float = 0.5,
    ) -> None:
        super().__init__(
            role_name=EGO_ROLE_NAME,
            spawn_location=spawn_location,
            vehicle_type=vehicle_type,
            initial_speed_kmh=initial_speed_kmh,
            spawn_retry_max_count=spawn_retry_max_count,
            spawn_retry_t_step=spawn_retry_t_step,
            spawn_retry_z_step=spawn_retry_z_step,
        )


class BaseScenario(ABC):
    """Abstract base class for CARLA test scenarios.

    Subclasses must implement :meth:`setup` and :meth:`is_done`.
    The ego vehicle is mandatory and must be provided via *ego_config*.
    """

    #: Default random seed for deterministic simulation.
    #: Override per-instance via ``random_seed`` keyword argument.
    DEFAULT_RANDOM_SEED: int = 0

    #: Number of warm-up ticks after spawn to stabilise physics and
    #: TrafficManager before the main tick loop begins.
    STABILIZE_TICKS: int = 5

    def __init__(
        self,
        ego_config: EgoConfig,
        *,
        spawn_pose: Lanelet2Pose | None = None,
        ground_projection: GroundProjectionConfig | None = None,
        random_seed: int = DEFAULT_RANDOM_SEED,
        ego_type: type[EgoVehicle] | None = None,
        psim_compatible_mode: bool = False,
        domain_id: int = 0,
    ) -> None:
        """Initialize the scenario with an ego vehicle configuration.

        Args:
            ego_config: Spawn configuration for the ego vehicle.
            spawn_pose: Optional Lanelet2 pose for the ego spawn point.
                When provided, :meth:`_setup_ego_spawn` can convert it to a
                CARLA-snapped spawn location.
            ground_projection: Ground-projection settings used when snapping
                poses to the CARLA road surface.  Defaults to
                :class:`GroundProjectionConfig` with default values.
            random_seed: Seed for the CARLA TrafficManager random device.
                Using a fixed seed ensures deterministic NPC behaviour across
                runs.  Defaults to :attr:`DEFAULT_RANDOM_SEED` (``0``).
            ego_type: :class:`EgoVehicle` subclass to instantiate for the ego
                actor.  Pass :class:`AutowareEntity` to disable TrafficManager
                autopilot on the ego vehicle.  ``None`` (default) uses
                :class:`EgoVehicle`.
            psim_compatible_mode: When ``True``, wait for ``initialpose``
                and ``initialtwist`` topics via DDS before spawning the ego
                vehicle as an :class:`AutowareEntity`.  The scenario tick
                loop does not start until the entity is spawned.
                Defaults to ``False`` for backward compatibility.
            domain_id: DDS domain ID used when *psim_compatible_mode* is
                ``True``.  Must match ``ROS_DOMAIN_ID``.
        """
        from .entity.autoware_entity import AutowareEntity  # noqa: PLC0415
        from .entity.ego import EgoVehicle as _EgoVehicle  # noqa: PLC0415

        self.psim_compatible_mode = psim_compatible_mode
        self._domain_id = domain_id

        self.ego_config = ego_config
        self.ego_type: type[EgoVehicle] = (
            AutowareEntity if psim_compatible_mode else (ego_type or _EgoVehicle)
        )
        self._spawn_pose = spawn_pose
        self._ground_projection = ground_projection or GroundProjectionConfig()
        self.random_seed = random_seed
        self._client: Optional["carla.Client"] = None
        self._tm_port: int = DEFAULT_TM_PORT
        self._entities: List[VehicleEntity] = []
        self._pre_tick_callbacks: List[Callable[["carla.World"], None]] = []
        self._post_tick_callbacks: List[Callable[["carla.World"], None]] = []
        self._pre_tick_actions: List[BaseAction] = []
        self._post_tick_actions: List[BaseAction] = []
        self._pass_conditions: List[BaseCondition] = []
        self._fail_conditions: List[BaseCondition] = []
        self._spectator_camera_config: Optional[SpectatorCameraConfig] = None

    # ------------------------------------------------------------------
    # Client injection
    # ------------------------------------------------------------------

    def set_client(
        self, client: "carla.Client", tm_port: int = DEFAULT_TM_PORT
    ) -> None:
        """Inject the CARLA client used by this scenario.

        Called by :class:`ScenarioRunner` before :meth:`setup`.

        Args:
            client: The CARLA client instance.
            tm_port: CARLA TrafficManager port.
        """
        self._client = client
        self._tm_port = tm_port

    @property
    def client(self) -> "carla.Client":
        """Return the injected CARLA client.

        Raises:
            RuntimeError: If :meth:`set_client` has not been called yet.
        """
        if self._client is None:
            raise RuntimeError(
                "CARLA client not set. "
                "Call set_client() before accessing the client property."
            )
        return self._client

    @property
    def tm_port(self) -> int:
        """Return the CARLA TrafficManager port."""
        return self._tm_port

    @property
    def world(self) -> "carla.World":
        """Return the current CARLA world from the injected client.

        Raises:
            RuntimeError: If :meth:`set_client` has not been called yet.
        """
        return self.client.get_world()

    # ------------------------------------------------------------------
    # Abstract interface – must be implemented by subclasses
    # ------------------------------------------------------------------

    def _setup_ego_spawn(self) -> OpenDrivePose:
        """Convert the Lanelet2 spawn pose to CARLA and update ego_config.

        Converts :attr:`_spawn_pose` through OpenDRIVE to a CARLA world
        position, snaps it to the road surface, updates :attr:`ego_config`
        with the resulting spawn location, and registers spectator-follow
        and position-logging callbacks.

        Returns:
            The intermediate :class:`OpenDrivePose` (useful for deriving
            target lane IDs, route conditions, etc.).

        Raises:
            ValueError: If :attr:`_spawn_pose` is ``None``.
        """
        if self._spawn_pose is None:
            msg = f"spawn_pose is required for {type(self).__name__}"
            raise ValueError(msg)
        ll2_pose = self._spawn_pose
        od_pose = to_opendrive(ll2_pose)
        world = self.world
        snapped = snap_to_carla_road(
            od_pose, world, ground_projection=self._ground_projection
        )

        logger.info(
            "Lanelet %d -> OpenDRIVE road='%s' lane=%d s=%.1f -> "
            "CARLA (%.1f, %.1f, %.3f) yaw=%.1f",
            ll2_pose.lanelet_id,
            od_pose.road_id,
            od_pose.lane_id,
            od_pose.s,
            snapped.x,
            snapped.y,
            snapped.z,
            snapped.yaw,
        )

        self.ego_config.spawn_location = SpawnTransform(snapped.to_carla_transform())
        self.ego_config.od_pose = od_pose
        self.ego_config.ground_projection = self._ground_projection

        ego_actor = lambda: find_actor_by_role_name(world, EGO_ROLE_NAME)  # noqa: E731
        self.follow_with_spectator(ego_actor)
        self.log_actor_position(ego_actor, label="ego")

        return od_pose

    @abstractmethod
    def setup(self) -> None:
        """Set up the scenario actors and environment.

        The CARLA world is available via :attr:`world` (which calls
        ``self.client.get_world()``).  :meth:`set_client` is guaranteed
        to have been called before this method.
        """
        ...

    def wait_for_autoware_init(self, timeout_sec: float = 30.0) -> None:
        """Block until ``initialpose`` and ``initialtwist`` arrive via DDS.

        Only used when :attr:`psim_compatible_mode` is ``True``.  Converts
        the received MGRS pose to a CARLA spawn transform and updates
        :attr:`ego_config` so the ego vehicle is spawned at the correct
        location.

        Args:
            timeout_sec: Maximum seconds to wait.

        Raises:
            TimeoutError: If required data does not arrive in time.
        """
        import math  # noqa: PLC0415
        import time as _time  # noqa: PLC0415

        from cyclonedds.core import ReadCondition, SampleState, WaitSet  # noqa: PLC0415
        from cyclonedds.domain import DomainParticipant  # noqa: PLC0415
        from cyclonedds.sub import DataReader  # noqa: PLC0415
        from cyclonedds.topic import Topic  # noqa: PLC0415

        from .coordinate.map_manager import MapManager  # noqa: PLC0415
        from .dds.msg import PoseWithCovarianceStamped, TwistStamped  # noqa: PLC0415
        from .dds.qos import DEFAULT_QOS  # noqa: PLC0415

        participant = DomainParticipant(domain_id=self._domain_id)

        pose_topic: Topic = Topic(
            participant, "rt/initialpose3d", PoseWithCovarianceStamped, qos=DEFAULT_QOS
        )
        twist_topic: Topic = Topic(
            participant, "rt/initialtwist3d", TwistStamped, qos=DEFAULT_QOS
        )
        pose_reader = DataReader(participant, pose_topic, qos=DEFAULT_QOS)
        twist_reader = DataReader(participant, twist_topic, qos=DEFAULT_QOS)

        waitset = WaitSet(participant)
        waitset.attach(ReadCondition(pose_reader, SampleState.NotRead))
        waitset.attach(ReadCondition(twist_reader, SampleState.NotRead))

        logger.info("psim_compatible_mode: waiting for initialpose and initialtwist …")

        received_pose: PoseWithCovarianceStamped | None = None
        received_twist: TwistStamped | None = None
        deadline = _time.monotonic() + timeout_sec
        poll_ns = 1_000_000_000

        while received_pose is None or received_twist is None:
            remaining = deadline - _time.monotonic()
            if remaining <= 0:
                missing = []
                if received_pose is None:
                    missing.append("initialpose")
                if received_twist is None:
                    missing.append("initialtwist")
                raise TimeoutError(
                    f"psim_compatible_mode init timed out after {timeout_sec:.1f}s. "
                    f"Missing: {', '.join(missing)}"
                )

            waitset.wait(timeout=min(int(remaining * 1e9), poll_ns))

            if received_pose is None:
                samples = pose_reader.take()
                if samples:
                    received_pose = samples[-1]
                    logger.info("Received initialpose")

            if received_pose is not None and received_twist is None:
                samples = twist_reader.take()
                if samples:
                    received_twist = samples[-1]
                    logger.info("Received initialtwist")

        # -- Convert MGRS pose → CARLA transform --
        mm = MapManager.get_instance()
        offset_x, offset_y = mm.mgrs_offset

        p = received_pose.pose.pose
        carla_x = p.position.x - offset_x
        carla_y = -(p.position.y - offset_y)
        carla_z = p.position.z - mm.z_offset

        q = p.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        carla_yaw = -math.degrees(math.atan2(siny_cosp, cosy_cosp))

        spawn_transform = carla.Transform(
            carla.Location(x=carla_x, y=carla_y, z=carla_z),
            carla.Rotation(yaw=carla_yaw),
        )
        self.ego_config.spawn_location = SpawnTransform(spawn_transform)

        # -- Apply initial speed from twist --
        tw = received_twist.twist
        speed_ms = (tw.linear.x**2 + tw.linear.y**2 + tw.linear.z**2) ** 0.5
        self.ego_config.initial_speed_kmh = speed_ms * 3.6

        logger.info(
            "psim_compatible_mode: spawn at CARLA(%.1f, %.1f, %.1f) yaw=%.1f° "
            "speed=%.1f km/h",
            carla_x,
            carla_y,
            carla_z,
            carla_yaw,
            self.ego_config.initial_speed_kmh,
        )

        # Clean up temporary DDS entities
        del twist_reader, pose_reader, waitset, participant

    @abstractmethod
    def is_done(self) -> bool:
        """Return True when the scenario should stop ticking.

        This is separate from pass/fail conditions and can be used to
        signal completion for scenarios that have a natural end point.
        """
        ...

    # ------------------------------------------------------------------
    # Callback registration helpers
    # ------------------------------------------------------------------

    def _register_tick(
        self,
        cb: Union[BaseAction, Callable[["carla.World"], None]],
        actions: List[BaseAction],
        callbacks: List[Callable[["carla.World"], None]],
    ) -> None:
        """Dispatch *cb* into the appropriate list.

        :class:`BaseAction` instances go into *actions* so the tick loop can
        pass *elapsed*.  Plain callables go into *callbacks*.
        """
        if isinstance(cb, BaseAction):
            actions.append(cb)
        else:
            callbacks.append(cb)

    def register_pre_tick(
        self, cb: Union[BaseAction, Callable[["carla.World"], None]]
    ) -> None:
        """Register a callback or action to run *before* each world tick.

        Args:
            cb: A :class:`BaseAction` or a plain callable receiving the world.
        """
        self._register_tick(cb, self._pre_tick_actions, self._pre_tick_callbacks)

    def register_post_tick(
        self, cb: Union[BaseAction, Callable[["carla.World"], None]]
    ) -> None:
        """Register a callback or action to run *after* each world tick.

        Args:
            cb: A :class:`BaseAction` or a plain callable receiving the world.
        """
        self._register_tick(cb, self._post_tick_actions, self._post_tick_callbacks)

    def register_entity(self, entity: VehicleEntity) -> None:
        """Register a spawned NPC vehicle entity.

        Registered entities will have their initial speed applied after the
        warm-up phase completes via :meth:`set_initial_speed`.

        Args:
            entity: A :class:`VehicleEntity` that has been spawned in
                :meth:`setup`.
        """
        self._entities.append(entity)

    def register_pass_condition(self, condition: BaseCondition) -> None:
        """Register a condition that marks the scenario as *passed*.

        Args:
            condition: Condition to add to the pass-condition list.
        """
        self._pass_conditions.append(condition)

    def register_fail_condition(self, condition: BaseCondition) -> None:
        """Register a condition that marks the scenario as *failed*.

        Args:
            condition: Condition to add to the fail-condition list.
        """
        self._fail_conditions.append(condition)

    # ------------------------------------------------------------------
    # Convenience helpers for common post-tick patterns
    # ------------------------------------------------------------------

    def follow_with_spectator(
        self,
        actor_getter: Callable[[], Optional["carla.Actor"]],
        *,
        offset_back: float = 8.0,
        offset_up: float = 5.0,
        pitch: float = -15.0,
    ) -> None:
        """Register a post-tick callback that makes the spectator follow an actor.

        The spectator is placed *offset_back* metres behind the actor's
        forward vector and *offset_up* metres above it.

        Args:
            actor_getter: Callable returning the actor to follow, or ``None``
                if the actor is not (yet) available.
            offset_back: Distance behind the actor (metres).
            offset_up: Height above the actor (metres).
            pitch: Camera pitch angle (degrees, negative = look down).
        """
        self._spectator_camera_config = SpectatorCameraConfig(
            offset_back=offset_back, offset_up=offset_up, pitch=pitch
        )

        def _follow(world: "carla.World") -> None:
            actor = actor_getter()
            if actor is None:
                return
            try:
                tf = actor.get_transform()
            except RuntimeError:
                return
            fwd = tf.get_forward_vector()
            loc = carla.Location(
                x=tf.location.x - offset_back * fwd.x,
                y=tf.location.y - offset_back * fwd.y,
                z=tf.location.z + offset_up,
            )
            rot = carla.Rotation(yaw=tf.rotation.yaw, pitch=pitch)
            world.get_spectator().set_transform(carla.Transform(loc, rot))

        self.register_post_tick(_follow)

    def log_actor_position(
        self,
        actor_getter: Callable[[], Optional["carla.Actor"]],
        *,
        label: str = "actor",
        interval_ticks: int = 20,
    ) -> None:
        """Register periodic position / speed logging for an actor.

        Logs the actor's CARLA world position and speed every
        *interval_ticks* simulation ticks (default ≈ 1 s at 20 Hz).

        Args:
            actor_getter: Callable returning the actor, or ``None``.
            label: Label prefix for the log line.
            interval_ticks: Logging interval in ticks.
        """
        state = {"tick": 0}

        def _log(world: "carla.World") -> None:
            state["tick"] += 1
            if state["tick"] % interval_ticks != 0:
                return
            actor = actor_getter()
            if actor is None:
                return
            try:
                loc = actor.get_location()
                vel = actor.get_velocity()
            except RuntimeError:
                logger.warning("[%s] actor no longer valid", label)
                return
            speed = (vel.x**2 + vel.y**2 + vel.z**2) ** 0.5
            logger.info(
                "[%s] speed=%.1f m/s | pos=(%.1f, %.1f, %.1f)",
                label,
                speed,
                loc.x,
                loc.y,
                loc.z,
            )

        self.register_post_tick(_log)

    # ------------------------------------------------------------------
    # Initial speed (called by ScenarioRunner after warm-up)
    # ------------------------------------------------------------------

    def set_initial_speed(self, ego_actor: "carla.Actor") -> None:
        """Apply initial speed to all registered entities and the ego vehicle.

        This method is called by :class:`ScenarioRunner` **after** the warm-up
        ticks complete so that vehicles remain stationary during stabilisation.

        Args:
            ego_actor: The ego vehicle CARLA actor.
        """

        def _apply(actor: "carla.Actor", speed_kmh: float) -> None:
            if speed_kmh <= 0.0:
                return
            speed_ms = speed_kmh / 3.6
            fwd = actor.get_transform().get_forward_vector()
            actor.set_target_velocity(
                carla.Vector3D(x=fwd.x * speed_ms, y=fwd.y * speed_ms, z=0.0)
            )

        for entity in self._entities:
            if entity.actor is not None:
                _apply(entity.actor, entity.initial_speed_kmh)

        if ego_actor is not None:
            _apply(ego_actor, self.ego_config.initial_speed_kmh)
