"""Base class for CARLA scenarios."""

from __future__ import annotations

import logging
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import TYPE_CHECKING, Callable, List, Optional

from .conditions import BaseCondition
from .entity._spawn import SpawnLocation

if TYPE_CHECKING:
    import carla

logger = logging.getLogger(__name__)


@dataclass
class EgoConfig:
    """Configuration for the ego vehicle.

    *spawn_location* determines where the vehicle is placed — either a
    :class:`SpawnTransform` (explicit pose) or a :class:`SpawnPointIndex`
    (index into the map's spawn-point list).
    """

    spawn_location: SpawnLocation
    vehicle_type: str = "vehicle.mini.cooper"


class BaseScenario(ABC):
    """Abstract base class for CARLA test scenarios.

    Subclasses must implement :meth:`setup` and :meth:`is_done`.
    The ego vehicle is mandatory and must be provided via *ego_config*.
    """

    #: Default random seed for deterministic simulation.
    #: Override per-instance via ``random_seed`` keyword argument.
    DEFAULT_RANDOM_SEED: int = 0

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
        self._pre_tick_callbacks: List[Callable[["carla.World"], None]] = []
        self._post_tick_callbacks: List[Callable[["carla.World"], None]] = []
        self._pass_conditions: List[BaseCondition] = []
        self._fail_conditions: List[BaseCondition] = []

    # ------------------------------------------------------------------
    # Abstract interface – must be implemented by subclasses
    # ------------------------------------------------------------------

    @abstractmethod
    def setup(self, world: "carla.World") -> None:
        """Set up the scenario actors and environment.

        Args:
            world: The CARLA world instance.
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

    def register_pre_tick(self, cb: Callable[["carla.World"], None]) -> None:
        """Register a callback to run *before* each world tick.

        Args:
            cb: Callable that receives the CARLA world as its argument.
        """
        self._pre_tick_callbacks.append(cb)

    def register_post_tick(self, cb: Callable[["carla.World"], None]) -> None:
        """Register a callback to run *after* each world tick.

        Args:
            cb: Callable that receives the CARLA world as its argument.
        """
        self._post_tick_callbacks.append(cb)

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

    def enable_autopilot_after(
        self,
        actor_getter: Callable[[], Optional["carla.Actor"]],
        *,
        delay_ticks: int = 5,
    ) -> None:
        """Register a post-tick callback that enables autopilot after a delay.

        This gives CARLA time to stabilise after map load / actor spawn
        before the TrafficManager builds its InMemoryMap (which blocks).

        Args:
            actor_getter: Callable returning the vehicle actor, or ``None``
                if the actor is not (yet) available.
            delay_ticks: Number of simulation ticks to wait.
        """
        state = {"count": 0, "enabled": False}

        def _enable(world: "carla.World") -> None:
            if state["enabled"]:
                return
            state["count"] += 1
            if state["count"] >= delay_ticks:
                actor = actor_getter()
                if actor is not None:
                    actor.set_autopilot(True)
                    state["enabled"] = True
                    logger.info("Autopilot enabled after %d ticks", state["count"])

        self.register_post_tick(_enable)

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
