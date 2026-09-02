"""In-memory fake :class:`AutowareBridge` for unit tests.

:class:`FakeAutowareBridge` records every call and advances its internal state
deterministically so that
:class:`~autoware_carla_scenario.autoware_bridge.init_sequence.AutowareInitSequence`
can be driven to completion without a live Autoware stack.

State progression (each stage completes after a configurable number of polls):

* ``is_autoware_ready`` returns ``True`` after ``ready_after`` calls.
* ``get_localization_state`` reaches ``INITIALIZED`` ``localize_after`` polls
  after :meth:`initialize_pose` is called.
* ``get_routing_state`` reaches ``SET`` ``route_after`` polls after
  :meth:`set_route` is called.
* ``get_operation_mode`` reaches ``AUTONOMOUS`` ``engage_after`` polls after
  :meth:`change_to_autonomous` is called.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

from .base import (
    AutowareBridge,
    BridgePose,
    LocalizationState,
    OperationMode,
    RoutingState,
    VehicleStatus,
)


@dataclass
class FakeAutowareBridge(AutowareBridge):
    """Deterministic fake bridge for tests.

    Args:
        ready_after: Number of :meth:`is_autoware_ready` calls before it
            returns ``True``.
        localize_after: Polls of :meth:`get_localization_state` (after
            :meth:`initialize_pose`) before it reports ``INITIALIZED``.
        route_after: Polls of :meth:`get_routing_state` (after
            :meth:`set_route`) before it reports ``SET``.
        engage_after: Polls of :meth:`get_operation_mode` (after
            :meth:`change_to_autonomous`) before it reports ``AUTONOMOUS``.
    """

    ready_after: int = 0
    localize_after: int = 0
    route_after: int = 0
    engage_after: int = 0

    #: Ordered log of method names invoked, for assertions.
    calls: List[str] = field(default_factory=list)
    #: The pose passed to :meth:`initialize_pose`, if any.
    initialized_pose: Optional[BridgePose] = None
    #: The goal passed to :meth:`set_route`, if any.
    route_goal: Optional[BridgePose] = None
    #: Whether :meth:`start` has been called.
    started: bool = False
    #: Whether :meth:`close` has been called.
    closed: bool = False

    # Internal counters.  Whether pose init / route were requested is derived
    # from ``initialized_pose`` / ``route_goal`` being set, so no separate flags.
    _ready_calls: int = 0
    _localize_polls: int = 0
    _route_polls: int = 0
    _engage_polls: int = 0
    _engage_requested: bool = False

    # -- Readiness -----------------------------------------------------

    def is_autoware_ready(self) -> bool:
        self.calls.append("is_autoware_ready")
        ready = self._ready_calls >= self.ready_after
        self._ready_calls += 1
        return ready

    # -- Pose initialization ------------------------------------------

    def initialize_pose(self, pose: BridgePose) -> None:
        self.calls.append("initialize_pose")
        self.initialized_pose = pose
        self._localize_polls = 0

    def get_localization_state(self) -> LocalizationState:
        self.calls.append("get_localization_state")
        if self.initialized_pose is None:
            return LocalizationState.UNINITIALIZED
        if self._localize_polls >= self.localize_after:
            return LocalizationState.INITIALIZED
        self._localize_polls += 1
        return LocalizationState.INITIALIZING

    # -- Routing -------------------------------------------------------

    def set_route(self, goal: BridgePose) -> None:
        self.calls.append("set_route")
        self.route_goal = goal
        self._route_polls = 0

    def get_routing_state(self) -> RoutingState:
        self.calls.append("get_routing_state")
        if self.route_goal is None:
            return RoutingState.UNSET
        if self._route_polls >= self.route_after:
            return RoutingState.SET
        self._route_polls += 1
        return RoutingState.UNSET

    # -- Engage --------------------------------------------------------

    def change_to_autonomous(self) -> None:
        self.calls.append("change_to_autonomous")
        self._engage_requested = True
        self._engage_polls = 0

    def get_operation_mode(self) -> OperationMode:
        self.calls.append("get_operation_mode")
        if not self._engage_requested:
            return OperationMode.STOP
        if self._engage_polls >= self.engage_after:
            return OperationMode.AUTONOMOUS
        self._engage_polls += 1
        return OperationMode.STOP

    # -- Status --------------------------------------------------------

    def get_vehicle_status(self) -> VehicleStatus:
        self.calls.append("get_vehicle_status")
        engaged = self._engage_requested and self._engage_polls >= self.engage_after
        return VehicleStatus(
            longitudinal_velocity=0.0,
            steering_tire_angle=0.0,
            operation_mode=(
                OperationMode.AUTONOMOUS if engaged else OperationMode.STOP
            ),
            is_autonomous_available=self.route_goal is not None,
        )

    def get_estimated_pose(self) -> Optional[BridgePose]:
        self.calls.append("get_estimated_pose")
        # Once localized, Autoware's estimate mirrors the initialized pose.
        if (
            self.initialized_pose is not None
            and self._localize_polls >= self.localize_after
        ):
            return self.initialized_pose
        return None

    # -- Lifecycle -----------------------------------------------------

    def start(self) -> None:
        self.calls.append("start")
        self.started = True

    def close(self) -> None:
        self.calls.append("close")
        self.closed = True
