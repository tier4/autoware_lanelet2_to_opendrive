"""Base class for CARLA scenarios."""

from __future__ import annotations

import logging
from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Callable, List, Optional, Union

from .actions import BaseAction
from .conditions import BaseCondition
from .constants import DEFAULT_TM_PORT, EGO_ROLE_NAME
from .entity._spawn import SpawnLocation
from .entity.vehicle_entity import VehicleEntity, VehicleEntityConfig
from .tick_snapshot import TickSnapshot

if TYPE_CHECKING:
    import carla

logger = logging.getLogger(__name__)


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
    ) -> None:
        super().__init__(
            role_name=EGO_ROLE_NAME,
            spawn_location=spawn_location,
            vehicle_type=vehicle_type,
            initial_speed_kmh=initial_speed_kmh,
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
        self, ego_config: EgoConfig, *, random_seed: int = DEFAULT_RANDOM_SEED
    ) -> None:
        """Initialize the scenario with an ego vehicle configuration.

        Args:
            ego_config: Spawn configuration for the ego vehicle.
            random_seed: Seed for the CARLA TrafficManager random device.
                Using a fixed seed ensures deterministic NPC behaviour across
                runs.  Defaults to :attr:`DEFAULT_RANDOM_SEED` (``0``).
        """
        self.ego_config = ego_config
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

    @abstractmethod
    def setup(self) -> None:
        """Set up the scenario actors and environment.

        The CARLA world is available via :attr:`world` (which calls
        ``self.client.get_world()``).  :meth:`set_client` is guaranteed
        to have been called before this method.
        """
        ...

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
    # Tick-phase execution
    # ------------------------------------------------------------------

    def _run_tick_phase(
        self,
        snapshot: TickSnapshot,
        actions: List[BaseAction],
        callbacks: List[Callable[["carla.World"], None]],
    ) -> None:
        """Execute one tick phase: run actions, then callbacks, then prune.

        Completed one-shot actions (``once=True`` and ``done``) are removed
        from the *actions* list after execution to avoid unnecessary
        iteration on subsequent ticks.

        Args:
            snapshot: Immutable snapshot of the current tick state.
            actions: The mutable list of :class:`BaseAction` for this phase.
            callbacks: The list of plain callbacks for this phase.
        """
        for action in actions:
            action.tick(snapshot)

        for cb in callbacks:
            cb(snapshot.world)

        # Prune completed one-shot actions so they are not iterated again.
        actions[:] = [a for a in actions if not (a._once and a._done)]

    def run_pre_tick(self, snapshot: TickSnapshot) -> None:
        """Execute all pre-tick actions and callbacks.

        Called by :class:`ScenarioRunner` before ``world.tick()``.

        Args:
            snapshot: Immutable snapshot of the current tick state.
        """
        self._run_tick_phase(snapshot, self._pre_tick_actions, self._pre_tick_callbacks)

    def run_post_tick(self, snapshot: TickSnapshot) -> None:
        """Execute all post-tick actions and callbacks.

        Called by :class:`ScenarioRunner` after ``world.tick()``.

        Args:
            snapshot: Immutable snapshot of the current tick state.
        """
        self._run_tick_phase(
            snapshot, self._post_tick_actions, self._post_tick_callbacks
        )

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
        import carla as _carla  # noqa: PLC0415

        def _follow(world: "carla.World") -> None:
            actor = actor_getter()
            if actor is None:
                return
            try:
                tf = actor.get_transform()
            except RuntimeError:
                return
            fwd = tf.get_forward_vector()
            loc = _carla.Location(
                x=tf.location.x - offset_back * fwd.x,
                y=tf.location.y - offset_back * fwd.y,
                z=tf.location.z + offset_up,
            )
            rot = _carla.Rotation(yaw=tf.rotation.yaw, pitch=pitch)
            world.get_spectator().set_transform(_carla.Transform(loc, rot))

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
        import carla as _carla  # noqa: PLC0415

        def _apply(actor: "carla.Actor", speed_kmh: float) -> None:
            if speed_kmh <= 0.0:
                return
            speed_ms = speed_kmh / 3.6
            fwd = actor.get_transform().get_forward_vector()
            actor.set_target_velocity(
                _carla.Vector3D(x=fwd.x * speed_ms, y=fwd.y * speed_ms, z=0.0)
            )

        for entity in self._entities:
            if entity.actor is not None:
                _apply(entity.actor, entity.initial_speed_kmh)

        if ego_actor is not None:
            _apply(ego_actor, self.ego_config.initial_speed_kmh)
