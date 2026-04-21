"""Action that publishes an Engage message via DDS."""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Optional

from ..conditions import BaseCondition
from .base import BaseAction, TickTiming

if TYPE_CHECKING:
    import carla

    from ..entity import AutowareEntity

logger = logging.getLogger(__name__)


class EngageAction(BaseAction):
    """Publish an :class:`~..dds.msg.Engage` message via DDS.

    When the trigger condition is met, the action calls
    :meth:`AutowareEntity.publish_engage` which writes to the DDS
    engage topic.  The entity's own Listener receives the message
    and updates :attr:`~AutowareEntity.is_engaged`.

    Args:
        entity: The :class:`AutowareEntity` to publish through.
        value: Engage state to publish (default ``True``).
        condition: Trigger condition (see :class:`BaseCondition`).
        timing: Tick phase (``PRE_TICK`` or ``POST_TICK``).
        label: Human-readable label for logging.
        once: If ``True`` (default) the action fires at most once.
    """

    def __init__(
        self,
        entity: AutowareEntity,
        value: bool = True,
        condition: Optional[BaseCondition] = None,
        timing: TickTiming = TickTiming.POST_TICK,
        *,
        label: str = "engage",
        once: bool = True,
    ) -> None:
        super().__init__(label=label, condition=condition, timing=timing, once=once)
        self._entity = entity
        self._value = value

    def execute(self, world: "carla.World") -> None:
        """Publish the engage message via DDS."""
        self._entity.publish_engage(self._value)
