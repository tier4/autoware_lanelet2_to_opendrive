"""Lane-change action: force a lane change via TrafficManager."""

from __future__ import annotations

import enum
import logging
from typing import TYPE_CHECKING, Union

from typing import Optional as _Optional

from ..conditions import BaseCondition
from ..conditions.base import find_actor_by_role_name
from ..constants import DEFAULT_TM_PORT
from ..entity_role import EntityRole
from .base import BaseAction, TickTiming

if TYPE_CHECKING:
    import carla

logger = logging.getLogger(__name__)


class LaneChangeDirection(enum.Enum):
    """Direction of a lane change."""

    LEFT = "left"
    RIGHT = "right"

    def to_carla_bool(self) -> bool:
        """Convert to the boolean expected by ``TrafficManager.force_lane_change``.

        CARLA convention: ``True`` → right, ``False`` → left.
        """
        return self is LaneChangeDirection.RIGHT


class LaneChangeAction(BaseAction):
    """Force a lane change via TrafficManager.

    When the associated condition is satisfied, this action:

    1. Locates the target vehicle by its ``role_name``
    2. Calls ``TrafficManager.force_lane_change(actor, direction)`` to
       command an immediate lane change

    Args:
        entity_name: ``role_name`` of the vehicle actor to control.
        direction: :class:`LaneChangeDirection` — ``LEFT`` or ``RIGHT``.
        client: A ``carla.Client`` used to obtain the TrafficManager.
        condition: Trigger condition (see :class:`BaseCondition`).
        timing: Tick phase (``PRE_TICK`` or ``POST_TICK``).
        once: If ``True`` (default) the action fires at most once.
    """

    def __init__(
        self,
        entity_name: Union[EntityRole, str],
        direction: LaneChangeDirection,
        client: "carla.Client",
        condition: _Optional[BaseCondition] = None,
        timing: TickTiming = TickTiming.PRE_TICK,
        *,
        label: str = "lane_change",
        once: bool = True,
        tm_port: int = DEFAULT_TM_PORT,
    ) -> None:
        super().__init__(label=label, condition=condition, timing=timing, once=once)
        self._entity_name = entity_name
        self._direction = direction
        self._client = client
        self._tm_port = tm_port

    # ------------------------------------------------------------------
    # BaseAction interface
    # ------------------------------------------------------------------

    def execute(self, world: "carla.World") -> None:
        """Command a lane change via TrafficManager."""
        actor = find_actor_by_role_name(world, self._entity_name)
        if actor is None:
            logger.warning("LaneChangeAction: actor '%s' not found", self._entity_name)
            return

        tm = self._client.get_trafficmanager(self._tm_port)
        tm.force_lane_change(actor, self._direction.to_carla_bool())
        logger.info(
            "LaneChangeAction: forced %s lane change for '%s'",
            self._direction.value,
            self._entity_name,
        )
