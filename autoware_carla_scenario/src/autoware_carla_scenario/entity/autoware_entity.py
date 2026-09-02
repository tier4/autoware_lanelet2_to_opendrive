"""Autoware ego vehicles.

Two entities live here:

* :class:`AutowareEntity` - a bare placeholder that spawns the ego, opts out of
  TrafficManager, and leaves the actor standing still for an external stack to
  control out of band.  Kept for backwards compatibility.

* :class:`AutowareEgoEntity` - the closed-loop entity.  Autoware (via the
  ``autoware_carla_interface`` ROS 2 node) reads CARLA sensors and applies
  control to the ego **directly**, so unlike
  :class:`~autoware_carla_scenario.entity.carla_driver_entity.CarlaDriverEntity`
  the framework is *not* in the control loop.  Instead this entity **attaches**
  to the ego actor spawned by the interface node and drives Autoware's startup
  handshake (pose init -> routing -> engage) over an
  :class:`~autoware_carla_scenario.autoware_bridge.base.AutowareBridge`.  This is
  a separate, Autoware-specific contract from the alpasim ``egodriver`` one.

Tick ownership: the scenario framework remains the tick master; the interface
node runs as a non-ticking, asynchronous I/O bridge (``sync_mode:=false``).

Role name: the framework identifies the ego by
:data:`~autoware_carla_scenario.constants.EGO_ROLE_NAME` (``"Ego"``).  Launch
the interface node with ``ego_vehicle_role_name:=Ego`` so the spawned actor
matches.
"""

from __future__ import annotations

import logging
import time
from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    import carla

    from ..autoware_bridge.base import AutowareBridge, BridgePose
    from ..autoware_bridge.init_sequence import InitState
    from ..scenario_base import EgoConfig

from ..autoware_bridge.base import AutowareBridgeConfig
from ..autoware_bridge.init_sequence import AutowareInitSequence
from ..conditions.base import find_actor_by_role_name
from ..constants import EGO_ROLE_NAME
from .ego import EgoVehicle

logger = logging.getLogger(__name__)

#: Polling interval while waiting for the interface node to spawn the ego actor.
_ATTACH_POLL_INTERVAL_S: float = 0.5


class AutowareEntity(EgoVehicle):
    """Ego vehicle controlled by Autoware instead of TrafficManager.

    After spawning, the :class:`ScenarioRunner` reads
    :attr:`EgoVehicle.use_autopilot` and skips ``set_autopilot(True)``
    for this actor, leaving it free for external (Autoware) control.

    The lifecycle hooks inherited from :class:`EgoVehicle` stay no-ops, so the
    vehicle stands still unless something outside the scenario drives it.  For a
    closed loop that drives Autoware's startup handshake, use
    :class:`AutowareEgoEntity`.
    """

    use_autopilot: bool = False


class AutowareEgoEntity(EgoVehicle):
    """Ego vehicle spawned by ``autoware_carla_interface`` and driven by Autoware.

    The :class:`ScenarioRunner` reads :attr:`EgoVehicle.use_autopilot` (``False``
    here) and skips ``set_autopilot(True)`` for this actor, leaving it under
    Autoware's control.

    .. warning::

       Not yet wired into :class:`ScenarioRunner` - this class is the foundation
       only.  Two runner-side gaps must be closed before an Autoware scenario can
       run end to end (tracked separately):

       * ``ScenarioRunner._destroy_all_dynamic_actors`` destroys every vehicle
         before ``ego.spawn()``, so it would remove the interface-spawned ego and
         :meth:`spawn` would only expire at ``attach_timeout``.  The runner must
         exempt the interface-owned actor (and its sensors) or create the ego
         after cleanup.
       * The runner evaluates pass/fail conditions from the first tick without
         waiting for :attr:`is_initialized`, so a condition already true near the
         initial pose could record a result before routing/engage completes.  The
         runner must gate scenario timing and condition evaluation on
         :attr:`is_initialized`.

       This entity already exposes the hooks (:attr:`is_initialized`,
       :attr:`termination_requested`) the runner needs; the wiring itself is a
       follow-up.

    Pose feedback to the scenario.  There are two paths, and the important one
    does **not** involve the bridge:

    * *Ground truth* - pass/fail conditions read the ego's pose/velocity from the
      CARLA actor directly.  Because :meth:`spawn` attaches this entity's
      :attr:`~EgoVehicle.actor` by ``role_name``, every existing condition
      (``EntityInAreaCondition``, ``EntityLanePositionCondition``,
      ``WaypointCondition``, ``CollisionCondition`` ...) works against the Autoware
      ego exactly as it does for a TrafficManager or driver ego, via
      ``find_actor_by_role_name(world, EGO_ROLE_NAME).get_transform()``.
    * *Autoware's estimate* - :attr:`estimated_pose` exposes Autoware's
      localization belief relayed over the bridge's ``StreamState``, for checking
      localization accuracy against ground truth.  It is monitoring only.

    The ``bridge`` is a required keyword argument: the live
    ``GrpcAutowareBridge`` is a follow-up (see
    ``proto/autoware_bridge/v0/autoware_bridge.proto``), so callers pass a bridge
    explicitly today (e.g. ``FakeAutowareBridge`` in tests).

    Args:
        config: Bridge connection settings.  ``None`` uses
            :class:`~autoware_carla_scenario.autoware_bridge.base.AutowareBridgeConfig`
            defaults.
        bridge: The bridge to the interface node.
        initial_pose: Map-frame pose used to initialize localization.  Required
            before :meth:`on_scenario_start`.
        goal_pose: Map-frame goal pose used to plan the route.  Required before
            :meth:`on_scenario_start`.
    """

    #: Autoware drives; TrafficManager must keep its hands off this actor.
    use_autopilot: bool = False

    def __init__(
        self,
        config: Optional["AutowareBridgeConfig"] = None,
        *,
        bridge: "AutowareBridge",
        initial_pose: Optional["BridgePose"] = None,
        goal_pose: Optional["BridgePose"] = None,
    ) -> None:
        super().__init__()
        self._config = config or AutowareBridgeConfig()
        self._bridge = bridge
        self._initial_pose = initial_pose
        self._goal_pose = goal_pose
        self._init_sequence: Optional["AutowareInitSequence"] = None
        self._termination_requested: bool = False

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def config(self) -> "AutowareBridgeConfig":
        """Return the bridge connection settings."""
        return self._config

    @property
    def bridge(self) -> "AutowareBridge":
        """Return the bridge used to communicate with the interface node."""
        return self._bridge

    @property
    def termination_requested(self) -> bool:
        """Whether the initialization handshake failed and the run should end."""
        return self._termination_requested

    @property
    def is_initialized(self) -> bool:
        """``True`` once Autoware is engaged and the scenario may proceed."""
        return self._init_sequence is not None and self._init_sequence.is_ready

    @property
    def init_state(self) -> Optional["InitState"]:
        """Current handshake state, or ``None`` before :meth:`on_scenario_start`."""
        return self._init_sequence.state if self._init_sequence is not None else None

    @property
    def estimated_pose(self) -> Optional["BridgePose"]:
        """Autoware's latest estimated map-frame pose, or ``None``.

        Monitoring only (localization accuracy vs. ground truth).  Scenario
        conditions read ground-truth pose from :attr:`~EgoVehicle.actor`, not
        from here.
        """
        return self._bridge.get_estimated_pose()

    # ------------------------------------------------------------------
    # Actor lifecycle (attach, not spawn)
    # ------------------------------------------------------------------

    def spawn(self, world: "carla.World", config: "EgoConfig") -> "carla.Actor":
        """Attach to the ego actor already spawned by the interface node.

        This does **not** create a new actor.  It polls the world for an actor
        whose ``role_name`` matches :data:`EGO_ROLE_NAME` until one appears or
        :attr:`AutowareBridgeConfig.attach_timeout` elapses.

        Args:
            world: The CARLA world instance.
            config: Ego configuration (accepted for API compatibility with
                :class:`EgoVehicle`; the spawn location is owned by the interface
                node and is not used here).

        Returns:
            The attached ego vehicle actor.

        Raises:
            RuntimeError: If no matching ego actor appears within the timeout.
        """
        del config  # Spawn is owned by the interface node; config is unused.

        deadline = time.monotonic() + self._config.attach_timeout
        while True:
            actor = find_actor_by_role_name(world, EGO_ROLE_NAME)
            if actor is not None:
                self._vehicle = actor
                logger.info(
                    "Attached to Autoware ego actor: id=%d role_name=%s",
                    actor.id,
                    EGO_ROLE_NAME,
                )
                return actor
            if time.monotonic() >= deadline:
                raise RuntimeError(
                    f"No ego actor with role_name={str(EGO_ROLE_NAME)!r} appeared "
                    f"within {self._config.attach_timeout:.1f}s. Ensure "
                    "autoware_carla_interface is running and launched with "
                    "ego_vehicle_role_name:=Ego."
                )
            time.sleep(_ATTACH_POLL_INTERVAL_S)

    def destroy(self) -> None:
        """Detach from the ego actor without destroying it.

        The interface node owns the ego actor's lifecycle, so this only clears
        the local reference; it never calls ``actor.destroy()``.
        """
        self._vehicle = None

    # ------------------------------------------------------------------
    # Lifecycle hooks (driven by ScenarioRunner)
    # ------------------------------------------------------------------

    def on_scenario_start(self, world: "carla.World") -> None:
        """Create the initialization handshake state machine.

        The handshake itself is advanced across ticks by :meth:`on_tick`, since
        Autoware only initializes while simulation time advances.

        Raises:
            RuntimeError: If the ego actor has not been attached yet.
            ValueError: If the initial pose or goal pose is missing.
        """
        del world
        if self.actor is None:
            raise RuntimeError(
                "AutowareEgoEntity.on_scenario_start called before spawn()/attach"
            )
        if self._initial_pose is None or self._goal_pose is None:
            raise ValueError(
                "AutowareEgoEntity requires both initial_pose and goal_pose "
                "before the initialization handshake can start."
            )
        # Open the transport and start mirroring Autoware state (a background
        # server-streaming subscription in the gRPC impl) so that the per-tick
        # query methods stay non-blocking.
        self._bridge.start()
        self._init_sequence = AutowareInitSequence(
            self._bridge,
            self._initial_pose,
            self._goal_pose,
            step_timeout=self._config.step_timeout,
        )

    def on_tick(self, world: "carla.World", elapsed: float) -> None:
        """Advance the initialization handshake by one step until it completes.

        Once Autoware is engaged the handshake is terminal and this becomes a
        no-op.  If the handshake fails, the entity requests early termination.
        """
        del world, elapsed
        seq = self._init_sequence
        if seq is None or seq.is_done:
            return
        seq.step()
        if seq.failed:
            logger.warning(
                "Autoware initialization failed: %s - requesting termination",
                seq.failure_reason,
            )
            self._termination_requested = True

    def on_scenario_end(self, world: "carla.World") -> None:
        """Close the bridge transport.  Never raises."""
        del world
        try:
            self._bridge.close()
        except Exception:  # noqa: BLE001 - teardown must not raise
            logger.warning("AutowareEgoEntity: bridge.close() failed", exc_info=True)
