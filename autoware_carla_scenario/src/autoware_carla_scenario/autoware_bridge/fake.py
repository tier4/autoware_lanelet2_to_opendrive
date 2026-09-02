"""In-memory fake :class:`AutowareBridge` for unit tests.

:class:`FakeAutowareBridge` records calls and reports readiness after a
configurable number of polls, so an :class:`AutowareEgoEntity` can be driven to
"ready" without a live Autoware stack.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

from .base import AutowareBridge, BridgePose


@dataclass
class FakeAutowareBridge(AutowareBridge):
    """Deterministic fake bridge for tests.

    Args:
        ready_after: Number of :meth:`is_ready` polls (after :meth:`configure`)
            before it returns ``True``.  ``0`` means ready on the first poll.
    """

    ready_after: int = 0

    #: Ordered log of method names invoked, for assertions.
    calls: List[str] = field(default_factory=list)
    #: The initial pose passed to :meth:`configure`, if any.
    configured_initial_pose: Optional[BridgePose] = None
    #: The goal passed to :meth:`configure`, if any.
    configured_goal: Optional[BridgePose] = None
    #: Whether :meth:`close` has been called.
    closed: bool = False

    _ready_polls: int = 0

    def configure(self, initial_pose: BridgePose, goal: BridgePose) -> None:
        self.calls.append("configure")
        self.configured_initial_pose = initial_pose
        self.configured_goal = goal

    def is_ready(self) -> bool:
        self.calls.append("is_ready")
        if self.configured_initial_pose is None:
            return False
        ready = self._ready_polls >= self.ready_after
        self._ready_polls += 1
        return ready

    def close(self) -> None:
        self.calls.append("close")
        self.closed = True
