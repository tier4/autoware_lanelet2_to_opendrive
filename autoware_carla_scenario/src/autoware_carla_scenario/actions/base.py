"""Base class for conditional actions executed during scenario tick loops."""

from __future__ import annotations

import enum
import logging
from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Optional

from ..conditions import BaseCondition
from ..conditions.always_true import AlwaysTrueCondition
from ..tick_snapshot import TickSnapshot

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
            :meth:`execute` is invoked.  Defaults to
            :class:`AlwaysTrueCondition` (fires unconditionally).
        timing: Whether to run on the pre-tick or post-tick phase.
        once: If ``True`` (default), the action fires at most once.  After
            ``execute`` has been called the condition is no longer evaluated.
    """

    def __init__(
        self,
        label: str,
        condition: Optional[BaseCondition] = None,
        timing: TickTiming = TickTiming.POST_TICK,
        *,
        once: bool = True,
    ) -> None:
        if not label:
            raise ValueError(
                f"{type(self).__name__}: label must not be empty. "
                "Provide a non-empty string to identify this action."
            )
        self.label = label
        self._condition = condition if condition is not None else AlwaysTrueCondition()
        self._timing = timing
        self._once = once
        self._done = False

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

    def tick(self, snapshot: TickSnapshot) -> None:
        """Evaluate the condition and, if met, run :meth:`execute`.

        Called by the scenario runner's tick loop with an immutable
        snapshot so that time-based conditions work correctly.

        Args:
            snapshot: Immutable snapshot of the current tick state,
                containing the CARLA world, elapsed time, tick count, and
                delta time.
        """
        if self._once and self._done:
            return

        result = self._condition.check(snapshot)
        if result is not None:
            logger.info(
                "%s triggered: %s",
                type(self).__name__,
                result.message,
            )
            self.execute(snapshot.world)
            self._done = True
