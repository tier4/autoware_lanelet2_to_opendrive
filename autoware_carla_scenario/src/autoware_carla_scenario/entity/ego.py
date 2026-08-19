"""Ego vehicle spawning."""

from __future__ import annotations

from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    import carla

    from ..scenario_base import EgoConfig

from ..constants import EGO_ROLE_NAME
from ._spawn import spawn_vehicle_actor


class EgoVehicle:
    """Manages the ego vehicle actor.

    Beyond spawning and destroying the actor, this class defines the lifecycle hooks
    :class:`ScenarioRunner` calls on the ego: :meth:`on_scenario_start` after warm-up,
    :meth:`on_tick` on every simulation tick, and :meth:`on_scenario_end` during
    teardown.  They are no-ops here so that TrafficManager-driven egos cost nothing;
    subclasses that drive the vehicle themselves (see
    :class:`~autoware_carla_scenario.entity.carla_driver_entity.CarlaDriverEntity`)
    override them.
    """

    #: When ``True`` (default), :class:`ScenarioRunner` enables
    #: TrafficManager autopilot on this actor after warm-up.
    use_autopilot: bool = True

    def __init__(self) -> None:
        self._vehicle: Optional["carla.Actor"] = None

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def actor(self) -> Optional["carla.Actor"]:
        """Return the spawned CARLA actor, or ``None`` before :meth:`spawn`."""
        return self._vehicle

    @property
    def termination_requested(self) -> bool:
        """Whether this entity has asked to end the scenario early.

        :class:`ScenarioRunner` checks this after the pass and fail conditions, so a
        condition that fires on the same tick still decides the outcome.
        """
        return False

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def spawn(self, world: "carla.World", config: EgoConfig) -> "carla.Actor":
        """Spawn the ego vehicle.

        Args:
            world: The CARLA world instance.
            config: Ego vehicle spawn configuration.

        Returns:
            The spawned vehicle actor.

        Raises:
            ValueError: If the vehicle blueprint is not found or spawn index
                is out of range.
            RuntimeError: If the vehicle could not be spawned at the
                requested location.
        """
        self._vehicle = spawn_vehicle_actor(
            world,
            config.vehicle_type,
            str(EGO_ROLE_NAME),
            config.spawn_location,
            od_pose=config.od_pose,
            spawn_retry_max_count=config.spawn_retry_max_count,
            spawn_retry_t_step=config.spawn_retry_t_step,
            spawn_retry_z_step=config.spawn_retry_z_step,
            ground_projection=config.ground_projection,
        )
        return self._vehicle

    # ------------------------------------------------------------------
    # Lifecycle hooks (no-ops by default)
    # ------------------------------------------------------------------

    def on_scenario_start(self, world: "carla.World") -> None:
        """Called once after the warm-up ticks and initial speeds are applied.

        Args:
            world: The CARLA world instance.
        """

    def on_tick(self, world: "carla.World", elapsed: float) -> None:
        """Called on every simulation tick, right after ``world.tick()``.

        Args:
            world: The CARLA world instance.
            elapsed: Wall-clock seconds since the tick loop started.
        """

    def on_scenario_end(self, world: "carla.World") -> None:
        """Called during teardown, before the actor is destroyed.

        Args:
            world: The CARLA world instance.
        """

    def destroy(self) -> None:
        """Destroy the vehicle actor."""
        if self._vehicle is not None:
            self._vehicle.destroy()
            self._vehicle = None
