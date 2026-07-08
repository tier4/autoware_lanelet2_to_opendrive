"""Base class for conditional actions executed during scenario tick loops."""

from __future__ import annotations

import enum
import logging
from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Optional

from ..conditions import BaseCondition
from ..conditions.always_true import AlwaysTrueCondition

if TYPE_CHECKING:
    import carla

    from ..command_batch import CommandBatch

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
        #: The frame's command batch, set by :meth:`tick` so that
        #: :meth:`execute` may emit batched commands via :meth:`emit`.
        self._active_batch: Optional["CommandBatch"] = None

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

        Subclasses whose effect maps to a ``carla.command.*`` should append it
        via :meth:`emit` so it is dispatched together with the rest of the
        frame's events in a single ``apply_batch_sync`` call.  Effects with no
        command equivalent (TrafficManager routing, traffic-light state) may
        issue their RPC directly.

        Args:
            world: The CARLA world instance.
        """
        ...

    def emit(self, command: object) -> None:
        """Queue a ``carla.command.*`` into the current frame's batch.

        Only valid while :meth:`execute` is running (i.e. from within
        :meth:`tick`).  The scenario runner flushes the batch once per tick
        phase, so all events emitted in the same frame share a single
        ``apply_batch_sync`` round-trip.

        If no batch is active (e.g. :meth:`execute` is called directly in a
        unit test) the command is dropped with a warning.
        """
        if self._active_batch is not None:
            self._active_batch.add(command)
        else:
            logger.warning(
                "%s.emit called without an active CommandBatch; command dropped",
                type(self).__name__,
            )

    def tick(
        self,
        world: "carla.World",
        elapsed: float,
        batch: Optional["CommandBatch"] = None,
    ) -> None:
        """Evaluate the condition and, if met, run :meth:`execute`.

        Called by the scenario runner's tick loop with the current elapsed
        time so that time-based conditions work correctly.

        Args:
            world: The CARLA world instance.
            elapsed: Seconds elapsed since the tick loop started.
            batch: The frame's :class:`~autoware_carla_scenario.command_batch.CommandBatch`.
                When provided, commands emitted via :meth:`emit` are collected
                into it and applied by the runner in a single batch.
        """
        if self._once and self._done:
            return

        result = self._condition.check(world, elapsed)
        if result is not None:
            logger.info(
                "%s triggered: %s",
                type(self).__name__,
                result.message,
            )
            self._active_batch = batch
            try:
                self.execute(world)
            finally:
                self._active_batch = None
            self._done = True
