"""Autoware ego vehicle controlled via DDS topic I/O.

This entity subscribes to the same input topics as Autoware's
``simple_planning_simulator`` node.

**Per-frame topics** (control commands, trajectory) are synchronised
with a :class:`~cyclonedds.core.WaitSet` — the tick loop blocks in
:meth:`wait_for_frame_data` until at least one new sample arrives.

**Event-driven topics** (engage, gear, indicators, …) are handled by
:class:`~cyclonedds.core.Listener` callbacks that store the latest
sample asynchronously as soon as it is published.
"""

from __future__ import annotations

import logging
import time
from typing import Any, Callable, Optional

from cyclonedds.core import (
    GuardCondition,
    Listener,
    ReadCondition,
    SampleState,
    WaitSet,
)
from cyclonedds.domain import DomainParticipant
from cyclonedds.pub import DataWriter
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic

from ..dds.msg import (
    ActuationCommandStamped,
    Control,
    Engage,
    GearCommand,
    Time,
    HazardLightsCommand,
    LaneletMapBin,
    PoseWithCovarianceStamped,
    TurnIndicatorsCommand,
    TwistStamped,
)
from ..dds.qos import DEFAULT_QOS, TRANSIENT_LOCAL_QOS
from .ego import EgoVehicle

logger = logging.getLogger(__name__)

#: WaitSet poll interval in nanoseconds (1 second).
_WAIT_POLL_NS: int = 1_000_000_000


# ------------------------------------------------------------------
# Topic specification
# ------------------------------------------------------------------


class _TopicSpec:
    """Descriptor for a single DDS input topic."""

    __slots__ = ("name", "msg_type", "qos", "required_for_init", "per_frame", "attr")

    def __init__(
        self,
        name: str,
        msg_type: type,
        qos: Any = None,
        *,
        required_for_init: bool = False,
        per_frame: bool = False,
        attr: str | None = None,
    ) -> None:
        self.name = name
        self.msg_type = msg_type
        self.qos = qos
        self.required_for_init = required_for_init
        self.per_frame = per_frame
        self.attr = attr


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
    # ---- Per-frame runtime topics (WaitSet) ----
    _TopicSpec(
        "ackermann_control_command",
        Control,
        per_frame=True,
        attr="_current_ackermann_cmd",
    ),
    _TopicSpec(
        "actuation_command",
        ActuationCommandStamped,
        per_frame=True,
        attr="_current_actuation_cmd",
    ),
    # ---- Event-driven runtime topics (Listener) ----
    _TopicSpec("engage", Engage),
    _TopicSpec(
        "manual_ackermann_control_command",
        Control,
        attr="_current_manual_ackermann_cmd",
    ),
    _TopicSpec("gear_command", GearCommand, attr="_current_gear_cmd"),
    _TopicSpec("manual_gear_command", GearCommand, attr="_current_manual_gear_cmd"),
    _TopicSpec(
        "turn_indicators_command",
        TurnIndicatorsCommand,
        attr="_current_turn_indicators_cmd",
    ),
    _TopicSpec(
        "hazard_lights_command",
        HazardLightsCommand,
        attr="_current_hazard_lights_cmd",
    ),
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
       ``initialpose`` **and** ``initialtwist`` arrive.
    4. Per-tick: call ``wait_for_frame_data()`` to block until a
       per-frame control command arrives, then read the latest samples.
       Event-driven topics are updated automatically via Listeners.
    5. ``destroy()`` – tear down DDS entities and the CARLA actor.
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

        # --- Runtime command state ---
        self._is_engaged: bool = False
        self._current_ackermann_cmd: Optional[Control] = None
        self._current_actuation_cmd: Optional[ActuationCommandStamped] = None
        self._current_manual_ackermann_cmd: Optional[Control] = None
        self._current_gear_cmd: Optional[GearCommand] = None
        self._current_manual_gear_cmd: Optional[GearCommand] = None
        self._current_turn_indicators_cmd: Optional[TurnIndicatorsCommand] = None
        self._current_hazard_lights_cmd: Optional[HazardLightsCommand] = None

        # --- DDS entities (created by setup_dds) ---
        self._participant: Optional[DomainParticipant] = None
        self._readers: dict[str, DataReader] = {}
        self._writers: dict[str, DataWriter] = {}
        self._shutdown_guard: Optional[GuardCondition] = None
        self._frame_waitset: Optional[WaitSet] = None
        self._frame_conditions: list[ReadCondition] = []
        self._listeners: list[Listener] = []

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def is_initialized(self) -> bool:
        """Whether all required initialisation data has been received."""
        return self._initial_pose is not None and self._initial_twist is not None

    @property
    def initial_pose(self) -> Optional[PoseWithCovarianceStamped]:
        return self._initial_pose

    @property
    def initial_twist(self) -> Optional[TwistStamped]:
        return self._initial_twist

    @property
    def vector_map(self) -> Optional[LaneletMapBin]:
        return self._vector_map

    @property
    def is_engaged(self) -> bool:
        """Whether Autoware has engaged (``True`` → motion enabled)."""
        return self._is_engaged

    def publish_engage(self, value: bool) -> None:
        """Publish an :class:`Engage` message via DDS.

        The entity's own Listener will receive this message and update
        :attr:`is_engaged` accordingly.

        Args:
            value: Engage state to publish.

        Raises:
            RuntimeError: If :meth:`setup_dds` has not been called yet.
        """
        writer = self._writers.get("engage")
        if writer is None:
            raise RuntimeError("Call setup_dds() before publish_engage().")
        now_ns = time.time_ns()
        stamp = Time(sec=now_ns // 1_000_000_000, nanosec=now_ns % 1_000_000_000)
        writer.write(Engage(stamp=stamp, engage=value))
        logger.info("Published engage=%s", value)

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

    def _make_event_callback(self, spec: _TopicSpec) -> Callable[[DataReader], None]:
        """Build a Listener callback that stores the latest sample."""
        if spec.name == "engage":

            def _on_engage(reader: DataReader) -> None:
                samples = reader.take()
                if samples:
                    self._is_engaged = samples[-1].engage
                    logger.debug("engage=%s", self._is_engaged)

            return _on_engage

        assert spec.attr is not None
        attr = spec.attr

        def _on_event(reader: DataReader) -> None:
            samples = reader.take()
            if samples:
                setattr(self, attr, samples[-1])

        return _on_event

    def setup_dds(self) -> None:
        """Create the DDS domain participant and data readers.

        * **Per-frame topics** get a :class:`ReadCondition` attached to
          ``_frame_waitset`` so :meth:`wait_for_frame_data` can block.
        * **Event-driven topics** get a :class:`Listener` whose
          ``on_data_available`` callback stores the latest sample.
        * **Initialisation topics** get plain readers (polled explicitly
          in :meth:`wait_for_initialization`).

        Idempotent – calling twice is a harmless no-op.
        """
        if self._participant is not None:
            logger.warning("DDS already initialised — skipping setup_dds()")
            return

        self._participant = DomainParticipant(domain_id=self._domain_id)
        self._shutdown_guard = GuardCondition(self._participant)
        self._frame_waitset = WaitSet(self._participant)

        engage_topic: Optional[Topic] = None

        for spec in _INPUT_TOPICS:
            qos = spec.qos or DEFAULT_QOS
            dds_name = self._resolve_topic_name(spec.name)
            topic: Topic = Topic(self._participant, dds_name, spec.msg_type, qos=qos)

            if spec.per_frame:
                reader = DataReader(self._participant, topic, qos=qos)
                rc = ReadCondition(reader, SampleState.NotRead)
                self._frame_waitset.attach(rc)
                self._frame_conditions.append(rc)
            elif self._is_event_topic(spec):
                callback = self._make_event_callback(spec)
                listener = Listener(on_data_available=callback)
                self._listeners.append(listener)
                reader = DataReader(
                    self._participant, topic, qos=qos, listener=listener
                )
            else:
                reader = DataReader(self._participant, topic, qos=qos)

            if spec.name == "engage":
                engage_topic = topic

            self._readers[spec.name] = reader
            logger.debug("Subscribed to %s (%s)", spec.name, dds_name)

        # Reuse the engage Topic from the reader loop for the DataWriter.
        assert engage_topic is not None
        self._writers["engage"] = DataWriter(
            self._participant, engage_topic, qos=DEFAULT_QOS
        )

        logger.info(
            "DDS setup complete: %d reader(s), %d writer(s) on domain %d",
            len(self._readers),
            len(self._writers),
            self._domain_id,
        )

    @staticmethod
    def _is_event_topic(spec: _TopicSpec) -> bool:
        """True for runtime topics that are NOT per-frame and NOT init."""
        return (
            not spec.required_for_init
            and not spec.per_frame
            and spec.name != "vector_map"
        )

    # ------------------------------------------------------------------
    # Initialisation – WaitSet gated
    # ------------------------------------------------------------------

    def wait_for_initialization(self, timeout_sec: float = 30.0) -> None:
        """Block until all required initialisation data has arrived.

        A temporary :class:`~cyclonedds.core.WaitSet` monitors
        ``initialpose``, ``initialtwist``, and (optionally)
        ``vector_map``.  Returns as soon as both **required** topics
        have been received.

        Args:
            timeout_sec: Maximum seconds to wait.

        Raises:
            RuntimeError: If :meth:`setup_dds` has not been called yet.
            TimeoutError: If required data does not arrive in time.
        """
        if self._participant is None:
            raise RuntimeError("Call setup_dds() before wait_for_initialization().")

        waitset = WaitSet(self._participant)

        for spec in _INPUT_TOPICS:
            if spec.required_for_init or spec.name == "vector_map":
                rc = ReadCondition(self._readers[spec.name], SampleState.NotRead)
                waitset.attach(rc)

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

    def _poll_init_data(self) -> None:
        """Read (and consume) samples from initialisation-related readers.

        All init readers are drained via ``take()`` on every call so that
        stale unread samples do not keep the WaitSet in a triggered
        state.

        ``initialtwist`` is intentionally discarded when ``initialpose``
        has not yet been received – this matches the behaviour of the
        original C++ ``simple_planning_simulator`` callback.
        """
        pose_samples = self._readers["initialpose"].take()
        if pose_samples and self._initial_pose is None:
            self._initial_pose = pose_samples[-1]
            logger.info("Received initialpose")

        twist_samples = self._readers["initialtwist"].take()
        if twist_samples:
            if self._initial_pose is not None and self._initial_twist is None:
                self._initial_twist = twist_samples[-1]
                logger.info("Received initialtwist")
            else:
                logger.debug("Discarding initialtwist (initialpose not yet received)")

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
    # Per-frame runtime – WaitSet gated
    # ------------------------------------------------------------------

    def wait_for_frame_data(self, timeout_ns: int = _WAIT_POLL_NS) -> None:
        """Block until at least one per-frame topic has new data.

        After the WaitSet triggers, all per-frame readers are drained
        and the latest sample for each is stored.  Event-driven topics
        are updated automatically by their Listener callbacks.

        Args:
            timeout_ns: Maximum wait in nanoseconds.

        Raises:
            RuntimeError: If :meth:`setup_dds` has not been called yet.
        """
        if self._frame_waitset is None:
            raise RuntimeError("Call setup_dds() before wait_for_frame_data().")

        self._frame_waitset.wait(timeout=timeout_ns)

        for spec in _INPUT_TOPICS:
            if not spec.per_frame or spec.attr is None:
                continue
            samples = self._readers[spec.name].take()
            if samples:
                setattr(self, spec.attr, samples[-1])

    # ------------------------------------------------------------------
    # Shutdown / cleanup
    # ------------------------------------------------------------------

    def request_shutdown(self) -> None:
        """Signal the initialisation wait-loop to exit early."""
        if self._shutdown_guard is not None:
            self._shutdown_guard.set(True)

    def destroy(self) -> None:
        """Tear down DDS entities and destroy the CARLA actor."""
        self._frame_conditions.clear()
        self._listeners.clear()
        self._frame_waitset = None
        self._writers.clear()
        self._readers.clear()
        self._shutdown_guard = None
        self._participant = None
        super().destroy()
