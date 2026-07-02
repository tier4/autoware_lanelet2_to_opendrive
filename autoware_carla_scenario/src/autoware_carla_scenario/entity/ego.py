"""Ego vehicle spawning."""

from __future__ import annotations

import time
from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    import carla

    from ..scenario_base import EgoConfig

from ..conditions.base import find_actor_by_role_name
from ..constants import EGO_ROLE_NAME
from ._spawn import spawn_vehicle_actor


class EgoVehicle:
    """Manages the ego vehicle actor."""

    #: When ``True`` (default), :class:`ScenarioRunner` enables
    #: TrafficManager autopilot on this actor after warm-up.
    use_autopilot: bool = True

    def __init__(self) -> None:
        self._vehicle: Optional["carla.Actor"] = None
        self._externally_managed: bool = False

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

    def attach(
        self,
        world: "carla.World",
        *,
        role_name: str = str(EGO_ROLE_NAME),
        timeout_seconds: float = 30.0,
        poll_interval_seconds: float = 0.5,
    ) -> "carla.Actor":
        """Bind to an ego actor that has been spawned externally.

        Polls the world until an actor with ``role_name`` becomes available
        and binds it to this wrapper. The actor will not be destroyed by
        :meth:`destroy` because it is owned by the external spawner.

        Args:
            world: The CARLA world instance.
            role_name: ``role_name`` attribute of the actor to attach to.
            timeout_seconds: Maximum time to wait for the actor to appear.
            poll_interval_seconds: Time between lookup attempts.

        Returns:
            The existing ego vehicle actor.

        Raises:
            RuntimeError: If no matching actor appears within the timeout.
        """
        deadline = time.monotonic() + timeout_seconds
        actor: Optional["carla.Actor"] = None
        while time.monotonic() < deadline:
            actor = find_actor_by_role_name(world, role_name)
            if actor is not None:
                break
            time.sleep(poll_interval_seconds)
        if actor is None:
            raise RuntimeError(
                f"No CARLA actor with role_name='{role_name}' found within "
                f"{timeout_seconds:.1f}s (expected to be spawned externally, "
                "e.g. by autoware_carla_interface)"
            )
        self._vehicle = actor
        self._externally_managed = True
        return actor

    def destroy(self) -> None:
        """Destroy the vehicle actor unless it is externally managed."""
        if self._vehicle is not None and not self._externally_managed:
            self._vehicle.destroy()
        self._vehicle = None
