"""Per-frame CARLA command batching.

In synchronous mode every individual ``actor.set_transform(...)`` /
``actor.set_target_velocity(...)`` style call is a blocking RPC round-trip to
the CARLA server.  When several such writes happen within a single simulation
frame the round-trips add up and slow the tick loop down.

:class:`CommandBatch` collects the ``carla.command.*`` objects produced by the
tick handlers (actions and callbacks) during one frame and dispatches them in a
single :meth:`carla.Client.apply_batch_sync` call, collapsing *N* round-trips
into one.

Only actor-level operations that have a ``carla.command.*`` equivalent
(transforms, target velocities, controls, light states, spawn/destroy, …) can
be batched this way.  TrafficManager calls (``force_lane_change``, ``set_path``)
and traffic-light state changes (``set_state`` / ``freeze``) have no command
form and are still issued directly by their respective actions.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Any, Iterable, List

if TYPE_CHECKING:
    import carla

logger = logging.getLogger(__name__)


class CommandBatch:
    """Accumulate ``carla.command.*`` objects for a single frame.

    Tick handlers append commands via :meth:`add` (or :meth:`extend`) during a
    frame; the scenario runner flushes them once per tick phase via
    :meth:`apply`, issuing a single ``apply_batch_sync`` RPC.

    A batch is cheap to create and a no-op to flush when empty, so the runner
    can build a fresh batch every phase without worrying about overhead.
    """

    def __init__(self) -> None:
        # ``carla.command.*`` objects are heterogeneous and share no typed
        # base class in the CARLA stubs, hence ``Any``.
        self._commands: List[Any] = []

    def add(self, command: Any) -> None:
        """Queue a single ``carla.command.*`` for this frame."""
        self._commands.append(command)

    def extend(self, commands: Iterable[Any]) -> None:
        """Queue multiple ``carla.command.*`` objects for this frame."""
        self._commands.extend(commands)

    def clear(self) -> None:
        """Discard all queued commands without applying them."""
        self._commands.clear()

    @property
    def commands(self) -> List[Any]:
        """A copy of the currently queued commands."""
        return list(self._commands)

    def __len__(self) -> int:
        return len(self._commands)

    def __bool__(self) -> bool:
        return bool(self._commands)

    def apply(self, client: "carla.Client", *, due_tick_cue: bool = False) -> List[Any]:
        """Flush the batch through ``client.apply_batch_sync`` and clear it.

        Args:
            client: The CARLA client used to dispatch the batch.
            due_tick_cue: Forwarded to ``apply_batch_sync``.  Defaults to
                ``False`` because the scenario runner drives ``world.tick()``
                explicitly and must not let the batch advance the simulation.

        Returns:
            The list of ``carla.command.Response`` objects returned by
            ``apply_batch_sync`` (empty when the batch held no commands).  The
            batch is always cleared, even if the RPC raises.
        """
        if not self._commands:
            return []

        # Detach the queued commands before dispatching so the batch is empty
        # for the next frame even if the RPC raises, and so the list handed to
        # ``apply_batch_sync`` is not mutated out from under it afterwards.
        commands = self._commands
        self._commands = []
        count = len(commands)

        responses = client.apply_batch_sync(commands, due_tick_cue)

        errors = [r for r in responses if getattr(r, "error", "")]
        for r in errors:
            logger.warning("apply_batch_sync command error: %s", r.error)
        logger.debug("Applied batch of %d command(s), %d error(s)", count, len(errors))
        return responses
