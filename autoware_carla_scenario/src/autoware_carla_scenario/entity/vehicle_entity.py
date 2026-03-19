"""NPC vehicle entity for CARLA scenarios."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Optional, Union

if TYPE_CHECKING:
    import carla

from ..entity_role import EntityRole
from ._spawn import SpawnLocation, spawn_vehicle_actor

#: Module-level flag set by :class:`~autoware_carla_scenario.scenario_runner.ScenarioRunner`
#: after warm-up ticks complete.  When ``True``, :meth:`VehicleEntity.spawn`
#: raises :class:`RuntimeError` to prevent spawning NPCs too late.
_warmup_done: bool = False


@dataclass
class VehicleEntityConfig:
    """Configuration for spawning a vehicle entity (ego or NPC).

    *spawn_location* determines where the vehicle is placed — either an
    explicit :class:`SpawnTransform` or a :class:`SpawnPointIndex`.
    """

    role_name: Union[EntityRole, str]
    spawn_location: SpawnLocation
    vehicle_type: str = "vehicle.mini.cooper"
    initial_speed_kmh: float = 0.0
    spawn_retry_max_count: int = 0
    spawn_retry_z_step: float = 0.1


class VehicleEntity:
    """Manages the lifecycle of an NPC vehicle actor in CARLA.

    This class handles spawning, destroying, and providing access to an NPC
    vehicle actor.  It mirrors the :class:`EgoVehicle` API but is intended
    for non-ego vehicles that participate in a scenario.
    """

    def __init__(self, config: VehicleEntityConfig) -> None:
        self._config = config
        self._vehicle: Optional["carla.Actor"] = None

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def role_name(self) -> Union[EntityRole, str]:
        """Return the role name that identifies this entity."""
        return self._config.role_name

    @property
    def vehicle_type(self) -> str:
        """Return the CARLA blueprint ID for this vehicle."""
        return self._config.vehicle_type

    @property
    def initial_speed_kmh(self) -> float:
        """Return the configured initial speed in km/h."""
        return self._config.initial_speed_kmh

    @property
    def actor(self) -> Optional["carla.Actor"]:
        """Return the underlying CARLA actor, or ``None`` if not spawned."""
        return self._vehicle

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def spawn(self, world: "carla.World") -> "carla.Actor":
        """Spawn the NPC vehicle in the CARLA world.

        Must be called during :meth:`BaseScenario.setup`, **before** the
        warm-up ticks run.  Spawning after warm-up raises :class:`RuntimeError`.

        Args:
            world: The CARLA world instance.

        Returns:
            The spawned vehicle actor.

        Raises:
            RuntimeError: If called after the warm-up phase has completed.
            ValueError: If the vehicle blueprint is not available or the
                spawn index is out of range.
            RuntimeError: If the vehicle could not be spawned at the
                requested location.
        """
        if _warmup_done:
            raise RuntimeError(
                "CARLA NPCs require ~5 ticks to stabilise after spawning. "
                "Spawn operations must be performed in setup() before "
                "the warm-up phase."
            )

        self._vehicle = spawn_vehicle_actor(
            world,
            self._config.vehicle_type,
            str(self._config.role_name),
            self._config.spawn_location,
            spawn_retry_max_count=self._config.spawn_retry_max_count,
            spawn_retry_z_step=self._config.spawn_retry_z_step,
        )
        return self._vehicle

    def destroy(self) -> None:
        """Destroy the vehicle actor and release resources."""
        if self._vehicle is not None:
            self._vehicle.destroy()
            self._vehicle = None
