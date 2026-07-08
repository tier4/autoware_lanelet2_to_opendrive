"""Speed action: set an actor's target cruise speed via TrafficManager."""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Optional, Union

from ..conditions import BaseCondition
from ..conditions.base import find_actor_by_role_name
from ..constants import DEFAULT_TM_PORT
from ..entity_role import EntityRole
from .base import BaseAction, TickTiming

if TYPE_CHECKING:
    import carla

logger = logging.getLogger(__name__)


class SpeedAction(BaseAction):
    """Set an actor's desired cruise speed via TrafficManager.

    When the associated condition is satisfied, this action locates the target
    vehicle by its ``role_name`` and calls
    ``TrafficManager.set_desired_speed(actor, target_speed_kmh)``. TrafficManager
    then holds that speed, so this is a one-shot command (``once=True``) — use it
    to model a timed speed change (e.g. "slow to 20 km/h after 20 s"), gating it
    on the appropriate :class:`BaseCondition`.

    Args:
        entity_name: ``role_name`` of the vehicle actor to control.
        target_speed_kmh: The desired cruise speed in km/h.
        client: A ``carla.Client`` used to obtain the TrafficManager.
        condition: Trigger condition (see :class:`BaseCondition`).
        timing: Tick phase (``PRE_TICK`` or ``POST_TICK``).
        label: Human-readable identifier.
        once: If ``True`` (default) the action fires at most once.
        tm_port: TrafficManager port.
    """

    def __init__(
        self,
        entity_name: Union[EntityRole, str],
        target_speed_kmh: float,
        client: "carla.Client",
        condition: Optional[BaseCondition] = None,
        timing: TickTiming = TickTiming.PRE_TICK,
        *,
        label: str = "speed",
        once: bool = True,
        tm_port: int = DEFAULT_TM_PORT,
    ) -> None:
        super().__init__(label=label, condition=condition, timing=timing, once=once)
        self._entity_name = entity_name
        self._target_speed_kmh = target_speed_kmh
        self._client = client
        self._tm_port = tm_port

    def execute(self, world: "carla.World") -> None:
        """Command the target cruise speed via TrafficManager."""
        actor = find_actor_by_role_name(world, self._entity_name)
        if actor is None:
            logger.warning("SpeedAction: actor '%s' not found", self._entity_name)
            return

        tm = self._client.get_trafficmanager(self._tm_port)
        tm.set_desired_speed(actor, self._target_speed_kmh)
        logger.info(
            "SpeedAction: set desired speed %.1f km/h for '%s'",
            self._target_speed_kmh,
            self._entity_name,
        )
