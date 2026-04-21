"""Autoware ego vehicle controlled via DDS topic I/O.

This entity subscribes to the same input topics as Autoware's
``simple_planning_simulator`` node.  A :class:`cyclonedds.core.WaitSet`
gates the simulation: the CARLA tick loop must **not** advance until
:meth:`AutowareEntity.wait_for_initialization` returns, ensuring that
``initialpose`` and ``initialtwist`` have been received.
"""

from __future__ import annotations

import logging
import time
from typing import Any, Optional

from cyclonedds.core import (
    GuardCondition,
    ReadCondition,
    SampleState,
    WaitSet,
)
from cyclonedds.domain import DomainParticipant
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic

from ..dds.msg import (
    AckermannControlCommand,
    ActuationCommandStamped,
    Engage,
    GearCommand,
    HazardLightsCommand,
    LaneletMapBin,
    PoseWithCovarianceStamped,
    Trajectory,
    TurnIndicatorsCommand,
    TwistStamped,
)
from ..dds.qos import DEFAULT_QOS, TRANSIENT_LOCAL_QOS
from .ego import EgoVehicle

logger = logging.getLogger(__name__)

#: WaitSet poll interval in nanoseconds (1 second).
_WAIT_POLL_NS: int = 1_000_000_000


# ------------------------------------------------------------------
# Topic specification helper
# ------------------------------------------------------------------


class _TopicSpec:
    """Lightweight descriptor for a single DDS input topic."""

    __slots__ = ("name", "msg_type", "qos", "required_for_init")

    def __init__(
        self,
        name: str,
        msg_type: type,
        qos: Any = None,
        *,
        required_for_init: bool = False,
    ) -> None:
        self.name = name
        self.msg_type = msg_type
        self.qos = qos  # ``None`` → resolved to DEFAULT_QOS in setup_dds()
        self.required_for_init = required_for_init


#: All input topic specifications.
_INPUT_TOPICS: list[_TopicSpec] = [
    # ---- Initialisation topics ----
    _TopicSpec(
        "initialpose",
        PoseWithCovarianceStamped,
        required_for_init=True,
    ),
    _TopicSpec(
        "initialtwist",
        TwistStamped,
        required_for_init=True,
    ),
    _TopicSpec(
        "vector_map",
        LaneletMapBin,
        qos=TRANSIENT_LOCAL_QOS,
    ),
    # ---- Runtime topics ----
    _TopicSpec("engage", Engage),
    _TopicSpec("ackermann_control_command", AckermannControlCommand),
    _TopicSpec("actuation_command", ActuationCommandStamped),
    _TopicSpec("manual_ackermann_control_command", AckermannControlCommand),
    _TopicSpec("gear_command", GearCommand),
    _TopicSpec("manual_gear_command", GearCommand),
    _TopicSpec("turn_indicators_command", TurnIndicatorsCommand),
    _TopicSpec("hazard_lights_command", HazardLightsCommand),
    _TopicSpec("trajectory", Trajectory),
]

#: Runtime topic key → instance attribute name.
_RT_ATTRS: list[tuple[str, str | None]] = [
    ("engage", None),  # special: extract .engage bool
    ("ackermann_control_command", "_current_ackermann_cmd"),
    ("actuation_command", "_current_actuation_cmd"),
    ("manual_ackermann_control_command", "_current_manual_ackermann_cmd"),
    ("gear_command", "_current_gear_cmd"),
    ("manual_gear_command", "_current_manual_gear_cmd"),
    ("turn_indicators_command", "_current_turn_indicators_cmd"),
    ("hazard_lights_command", "_current_hazard_lights_cmd"),
    ("trajectory", "_current_trajectory"),
]


# ------------------------------------------------------------------
# AutowareEntity
# ------------------------------------------------------------------


class AutowareEntity(EgoVehicle):
    """Ego vehicle controlled by Autoware instead of TrafficManager.

    Lifecycle
    ---------
    1. ``spawn(world, config)`` – create the CARLA actor (inherited).
    2. ``setup_dds()``          – create DDS participant & data readers.
    3. ``wait_for_initialization(timeout_sec)`` – block until
       ``initialpose`` **and** ``initialtwist`` arrive on the DDS bus.
    4. Per-tick: call ``poll_runtime_topics()`` to ingest the latest
       control commands published by Autoware.
    5. ``destroy()`` – tear down DDS entities and the CARLA actor.

    ``vector_map`` is also subscribed (transient-local QoS) but is
    **not** required for initialisation to succeed.
    """

    use_autopilot: bool = False

    def __init__(
        self,
        domain_id: int = 0,
        topic_prefix: str = "",
    ) -> None:
        """Create an Autoware-controlled ego entity.

        Args:
            domain_id: DDS domain ID (must match ``ROS_DOMAIN_ID``).
            topic_prefix: Optional ROS 2 namespace prefix inserted
                between ``rt/`` and ``input/`` in DDS topic names.
                For example ``"simulation"`` produces
                ``rt/simulation/input/initialpose``.
        """
        super().__init__()
        self._domain_id = domain_id
        self._topic_prefix = topic_prefix

        # --- Initialisation state ---
        self._initial_pose: Optional[PoseWithCovarianceStamped] = None
        self._initial_twist: Optional[TwistStamped] = None
        self._vector_map: Optional[LaneletMapBin] = None

        # --- Runtime command state (updated each tick) ---
        self._simulate_motion: bool = False
        self._current_ackermann_cmd: Optional[AckermannControlCommand] = None
        self._current_actuation_cmd: Optional[ActuationCommandStamped] = None
        self._current_manual_ackermann_cmd: Optional[AckermannControlCommand] = None
        self._current_gear_cmd: Optional[GearCommand] = None
        self._current_manual_gear_cmd: Optional[GearCommand] = None
        self._current_turn_indicators_cmd: Optional[TurnIndicatorsCommand] = None
        self._current_hazard_lights_cmd: Optional[HazardLightsCommand] = None
        self._current_trajectory: Optional[Trajectory] = None

        # --- DDS entities (created by setup_dds) ---
        self._participant: Optional[DomainParticipant] = None
        self._readers: dict[str, DataReader] = {}
        self._shutdown_guard: Optional[GuardCondition] = None

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def is_initialized(self) -> bool:
        """Whether all required initialisation data has been received."""
        return self._initial_pose is not None and self._initial_twist is not None

    @property
    def initial_pose(self) -> Optional[PoseWithCovarianceStamped]:
        """The received ``initialpose`` message, or ``None``."""
        return self._initial_pose

    @property
    def initial_twist(self) -> Optional[TwistStamped]:
        """The received ``initialtwist`` message, or ``None``."""
        return self._initial_twist

    @property
    def vector_map(self) -> Optional[LaneletMapBin]:
        """The received ``vector_map`` message, or ``None``."""
        return self._vector_map

    @property
    def simulate_motion(self) -> bool:
        """Current engage flag (``False`` → vehicle state frozen)."""
        return self._simulate_motion

    # ------------------------------------------------------------------
    # DDS setup
    # ------------------------------------------------------------------

    def _resolve_topic_name(self, short_name: str) -> str:
        """Build the full DDS topic name.

        ROS 2 topics are mapped to DDS as ``rt/<ns>/input/<name>``.
        """
        if self._topic_prefix:
            return f"rt/{self._topic_prefix}/input/{short_name}"
        return f"rt/input/{short_name}"

    def setup_dds(self) -> None:
        """Create the DDS domain participant and data readers.

        Idempotent – calling twice is a harmless no-op.
        """
        if self._participant is not None:
            logger.warning("DDS already initialised — skipping setup_dds()")
            return

        self._participant = DomainParticipant(domain_id=self._domain_id)
        self._shutdown_guard = GuardCondition(self._participant)

        for spec in _INPUT_TOPICS:
            qos = spec.qos or DEFAULT_QOS
            dds_name = self._resolve_topic_name(spec.name)
            topic: Topic = Topic(self._participant, dds_name, spec.msg_type, qos=qos)
            reader = DataReader(self._participant, topic, qos=qos)
            self._readers[spec.name] = reader
            logger.debug("Subscribed to %s (%s)", spec.name, dds_name)

        logger.info(
            "DDS setup complete: %d reader(s) on domain %d",
            len(self._readers),
            self._domain_id,
        )

    # ------------------------------------------------------------------
    # Initialisation – WaitSet gated
    # ------------------------------------------------------------------

    def wait_for_initialization(self, timeout_sec: float = 30.0) -> None:
        """Block until all required initialisation data has arrived.

        A :class:`~cyclonedds.core.WaitSet` monitors ``initialpose``,
        ``initialtwist``, and (optionally) ``vector_map``.  The method
        returns as soon as both **required** topics have been received.

        Args:
            timeout_sec: Maximum seconds to wait.

        Raises:
            RuntimeError: If :meth:`setup_dds` has not been called yet.
            TimeoutError: If required data does not arrive in time.
        """
        if self._participant is None:
            raise RuntimeError("Call setup_dds() before wait_for_initialization().")

        waitset = WaitSet(self._participant)

        # Attach read-conditions for init-related readers.
        for spec in _INPUT_TOPICS:
            if spec.required_for_init or spec.name == "vector_map":
                rc = ReadCondition(self._readers[spec.name], SampleState.NotRead)
                waitset.attach(rc)

        # Allow early exit via request_shutdown().
        if self._shutdown_guard is not None:
            waitset.attach(self._shutdown_guard)

        required = [s.name for s in _INPUT_TOPICS if s.required_for_init]
        logger.info(
            "Waiting for initialisation data (required: %s) …",
            ", ".join(required),
        )

        deadline = time.monotonic() + timeout_sec

        while not self.is_initialized:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                missing = self._missing_init_topics()
                raise TimeoutError(
                    f"Initialisation timed out after {timeout_sec:.1f}s. "
                    f"Missing required topic(s): {', '.join(missing)}"
                )

            poll_ns = min(int(remaining * 1e9), _WAIT_POLL_NS)
            waitset.wait(timeout=poll_ns)

            self._poll_init_data()

        logger.info("Initialisation complete.")

    # ------------------------------------------------------------------
    # Initialisation helpers
    # ------------------------------------------------------------------

    def _poll_init_data(self) -> None:
        """Read (and consume) samples from initialisation-related readers.

        All init readers are drained via ``take()`` on every call so that
        stale unread samples do not keep the WaitSet in a triggered
        state.

        ``initialtwist`` is intentionally discarded when ``initialpose``
        has not yet been received – this matches the behaviour of the
        original C++ ``simple_planning_simulator`` callback.
        """
        # --- initialpose (required) ---
        pose_samples = self._readers["initialpose"].take()
        if pose_samples and self._initial_pose is None:
            self._initial_pose = pose_samples[-1]
            logger.info("Received initialpose")

        # --- initialtwist (required, but only after initialpose) ---
        twist_samples = self._readers["initialtwist"].take()
        if twist_samples:
            if self._initial_pose is not None and self._initial_twist is None:
                self._initial_twist = twist_samples[-1]
                logger.info("Received initialtwist")
            else:
                logger.debug("Discarding initialtwist (initialpose not yet received)")

        # --- vector_map (optional) ---
        map_samples = self._readers["vector_map"].take()
        if map_samples and self._vector_map is None:
            self._vector_map = map_samples[-1]
            logger.info("Received vector_map")

    def _missing_init_topics(self) -> list[str]:
        """Return names of required topics that have not yet arrived."""
        missing: list[str] = []
        if self._initial_pose is None:
            missing.append("initialpose")
        if self._initial_twist is None:
            missing.append("initialtwist")
        return missing

    # ------------------------------------------------------------------
    # Runtime polling (call once per simulation tick)
    # ------------------------------------------------------------------

    def poll_runtime_topics(self) -> None:
        """Ingest the latest sample from every runtime input reader.

        Should be called **once per CARLA tick** so that the most recent
        control commands are available for the vehicle model update.
        """
        for key, attr in _RT_ATTRS:
            reader = self._readers.get(key)
            if reader is None:
                continue
            samples = reader.take()
            if not samples:
                continue
            sample = samples[-1]
            if attr is None:
                # ``engage`` topic: extract the boolean flag.
                self._simulate_motion = sample.engage
                logger.debug("engage=%s", self._simulate_motion)
            else:
                setattr(self, attr, sample)

    # ------------------------------------------------------------------
    # Shutdown helpers
    # ------------------------------------------------------------------

    def request_shutdown(self) -> None:
        """Signal the initialisation wait-loop to exit early."""
        if self._shutdown_guard is not None:
            self._shutdown_guard.set(True)

    def destroy(self) -> None:
        """Tear down DDS entities and destroy the CARLA actor."""
        self._readers.clear()
        self._shutdown_guard = None
        self._participant = None
        super().destroy()
