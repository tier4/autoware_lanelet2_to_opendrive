"""State machine driving the Autoware initialization handshake.

The handshake must be interleaved with world ticks: Autoware only makes
progress while simulation time (``/clock``) advances, which happens when the
scenario framework ticks the CARLA world.  :class:`AutowareInitSequence`
therefore exposes a non-blocking :meth:`step` that performs at most one action
per call and is meant to be invoked once per tick (from
:meth:`AutowareEgoEntity.on_tick`) until :meth:`is_done`.

Progression::

    IDLE -> WAIT_READY -> INIT_POSE -> WAIT_LOCALIZED -> SET_ROUTE
         -> WAIT_ROUTE -> ENGAGE -> WAIT_ENGAGED -> RUNNING

Any ``WAIT_*`` state that does not progress within ``step_timeout`` steps
transitions to ``FAILED`` with a human-readable :attr:`failure_reason`.
"""

from __future__ import annotations

import logging
from enum import Enum, auto
from typing import Optional

from .base import (
    AutowareBridge,
    BridgePose,
    LocalizationState,
    OperationMode,
    RoutingState,
)

logger = logging.getLogger(__name__)


class InitState(Enum):
    """Stages of the Autoware initialization handshake."""

    IDLE = auto()
    WAIT_READY = auto()
    INIT_POSE = auto()
    WAIT_LOCALIZED = auto()
    SET_ROUTE = auto()
    WAIT_ROUTE = auto()
    ENGAGE = auto()
    WAIT_ENGAGED = auto()
    RUNNING = auto()
    FAILED = auto()

    @property
    def is_terminal(self) -> bool:
        """Whether no further :meth:`step` will change this state."""
        return self in (InitState.RUNNING, InitState.FAILED)


class AutowareInitSequence:
    """Drives an :class:`AutowareBridge` through the initialization handshake.

    Args:
        bridge: The bridge used to talk to the interface node.
        initial_pose: Map-frame pose used to initialize localization.
        goal_pose: Map-frame goal pose used to plan the route.
        step_timeout: Maximum number of :meth:`step` calls a single
            ``WAIT_*`` state may spend before the sequence fails.  With the
            framework ticking at 20 Hz, ``600`` corresponds to ~30 s.
    """

    def __init__(
        self,
        bridge: AutowareBridge,
        initial_pose: BridgePose,
        goal_pose: BridgePose,
        *,
        step_timeout: int = 600,
    ) -> None:
        self._bridge = bridge
        self._initial_pose = initial_pose
        self._goal_pose = goal_pose
        self._step_timeout = step_timeout
        self._state = InitState.IDLE
        self._state_steps = 0
        self._failure_reason: Optional[str] = None

    # -- Introspection -------------------------------------------------

    @property
    def state(self) -> InitState:
        """Current state of the handshake."""
        return self._state

    @property
    def is_done(self) -> bool:
        """``True`` once the sequence reached a terminal state."""
        return self._state.is_terminal

    @property
    def is_ready(self) -> bool:
        """``True`` once Autoware is engaged and the scenario may proceed."""
        return self._state is InitState.RUNNING

    @property
    def failed(self) -> bool:
        """``True`` if the handshake failed (e.g. timed out)."""
        return self._state is InitState.FAILED

    @property
    def failure_reason(self) -> Optional[str]:
        """Human-readable failure description, or ``None`` if not failed."""
        return self._failure_reason

    # -- Driving -------------------------------------------------------

    def step(self) -> InitState:
        """Advance the handshake by at most one action.

        Call once per world tick until :attr:`is_done`.

        Returns:
            The state after this step.
        """
        if self._state.is_terminal:
            return self._state

        # Only WAIT_* states dwell across ticks; command states transition
        # immediately and reset _state_steps, so a plain threshold check
        # cannot trip on them.
        if self._state_steps >= self._step_timeout:
            return self._fail(
                f"timed out after {self._step_timeout} steps in {self._state.name}"
            )

        previous = self._state
        self._dispatch()
        if self._state is previous:
            self._state_steps += 1
        else:
            self._state_steps = 0
        return self._state

    # -- Internal ------------------------------------------------------

    def _dispatch(self) -> None:
        """Run the handler for the current state."""
        if self._state is InitState.IDLE:
            self._transition(InitState.WAIT_READY)
        elif self._state is InitState.WAIT_READY:
            if self._bridge.is_autoware_ready():
                self._transition(InitState.INIT_POSE)
        elif self._state is InitState.INIT_POSE:
            self._bridge.initialize_pose(self._initial_pose)
            self._transition(InitState.WAIT_LOCALIZED)
        elif self._state is InitState.WAIT_LOCALIZED:
            if self._bridge.get_localization_state() is LocalizationState.INITIALIZED:
                self._transition(InitState.SET_ROUTE)
        elif self._state is InitState.SET_ROUTE:
            self._bridge.set_route(self._goal_pose)
            self._transition(InitState.WAIT_ROUTE)
        elif self._state is InitState.WAIT_ROUTE:
            if self._bridge.get_routing_state() is RoutingState.SET:
                self._transition(InitState.ENGAGE)
        elif self._state is InitState.ENGAGE:
            self._bridge.change_to_autonomous()
            self._transition(InitState.WAIT_ENGAGED)
        elif self._state is InitState.WAIT_ENGAGED:
            if self._bridge.get_operation_mode() is OperationMode.AUTONOMOUS:
                self._transition(InitState.RUNNING)

    def _transition(self, new_state: InitState) -> None:
        logger.info("AutowareInitSequence: %s -> %s", self._state.name, new_state.name)
        self._state = new_state

    def _fail(self, reason: str) -> InitState:
        logger.warning("AutowareInitSequence failed: %s", reason)
        self._failure_reason = reason
        self._state = InitState.FAILED
        return self._state
