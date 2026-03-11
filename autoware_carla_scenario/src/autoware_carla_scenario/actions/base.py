"""Base class for conditional actions executed during scenario tick loops."""

from __future__ import annotations

import enum
import logging
from abc import ABC, abstractmethod
from typing import TYPE_CHECKING

from ..conditions import BaseCondition

if TYPE_CHECKING:
    import carla

logger = logging.getLogger(__name__)


class TickTiming(enum.Enum):
    """When the action is evaluated within the tick loop."""

    PRE_TICK = "pre_tick"
    POST_TICK = "post_tick"


class BaseAction(ABC):
    """Abstract base for actions that fire when a condition is met.

    Subclasses must implement :meth:`execute`.  The action is registered on a
    :class:`~autoware_carla_scenario.BaseScenario` via
    :meth:`~autoware_carla_scenario.BaseScenario.register_pre_tick` or
    :meth:`~autoware_carla_scenario.BaseScenario.register_post_tick` depending
    on *timing*.

    Args:
        condition: A :class:`BaseCondition` whose :meth:`check` is called each
            tick.  When ``check`` returns a non-``None`` result the action's
            :meth:`execute` is invoked.
        timing: Whether to run on the pre-tick or post-tick phase.
        once: If ``True`` (default), the action fires at most once.  After
            ``execute`` has been called the condition is no longer evaluated.
    """

    def __init__(
        self,
        condition: BaseCondition,
        timing: TickTiming = TickTiming.POST_TICK,
        *,
        once: bool = True,
    ) -> None:
        self._condition = condition
        self._timing = timing
        self._once = once
        self._done = False
        self._elapsed: float = 0.0

    @property
    def timing(self) -> TickTiming:
        """The tick phase this action is bound to."""
        return self._timing

    @property
    def done(self) -> bool:
        """Whether this action has already fired (relevant when *once=True*)."""
        return self._done

    @abstractmethod
    def execute(self, world: "carla.World") -> None:
        """Perform the action.

        Called when the condition is satisfied.

        Args:
            world: The CARLA world instance.
        """
        ...

    def tick(self, world: "carla.World") -> None:
        """Evaluate the condition and, if met, run :meth:`execute`.

        This method is designed to be registered directly as a tick callback
        via :meth:`BaseScenario.register_pre_tick` or
        :meth:`BaseScenario.register_post_tick`.

        Args:
            world: The CARLA world instance.
        """
        if self._once and self._done:
            return

        result = self._condition.check(world, self._elapsed)
        if result is not None:
            logger.info(
                "%s triggered: %s",
                type(self).__name__,
                result.message,
            )
            self.execute(world)
            self._done = True

    def set_elapsed(self, elapsed: float) -> None:
        """Update the elapsed time used for condition checks.

        The scenario runner or the caller is responsible for calling this
        before each tick so that time-based conditions work correctly.
        """
        self._elapsed = elapsed
