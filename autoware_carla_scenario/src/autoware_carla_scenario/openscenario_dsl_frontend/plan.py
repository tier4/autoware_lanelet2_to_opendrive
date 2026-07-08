"""Semantic scenario plan produced from the syntax IR.

Where :mod:`.ast_model` records *what the DSL said*, :class:`ScenarioPlan`
records *what to build*: a resolved ego actor, NPC actors, and a list of typed
:class:`Spec` entries describing the actions and pass/fail conditions the
generated scenario should register.

The plan is a pure-data intermediate representation shared by the translator
(which produces it) and the code generator (which renders it to Python).  It
contains no :mod:`autoware_carla_scenario` runtime objects, so it can be built
and tested without CARLA installed.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Optional

#: Default CARLA blueprint used when the DSL does not specify a vehicle type.
DEFAULT_VEHICLE_TYPE = "vehicle.mini.cooper"

#: Grace period (seconds) before a minimum-speed fail condition activates,
#: mirroring the built-in example scenarios.
DEFAULT_SPEED_CHECK_DELAY_SECONDS = 3.0

#: Upper bound on generated ``one_of`` variants before truncation.
MAX_VARIANTS = 64


class CondKind(str, Enum):
    """A framework condition a compiled event condition maps to."""

    SPEED = "speed"  # SpeedCondition
    LANE = "lane"  # EntityLanePositionCondition (equality)
    ELAPSED = "elapsed"  # ElapsedTimeCondition


@dataclass(frozen=True)
class ConditionSpec:
    """A compiled event condition (from a ``wait`` / ``until`` clause).

    Attributes:
        kind: The :class:`CondKind`.
        actor: The actor the condition observes (``None`` for time-based).
        op: The comparison operator for :attr:`CondKind.SPEED` (``"<"`` …).
        value: The threshold — m/s for speed, seconds for elapsed.
        lanelet_id: The target lanelet for :attr:`CondKind.LANE`.
        label: A stable label for the generated framework condition.
    """

    kind: CondKind
    actor: Optional[str] = None
    op: Optional[str] = None
    value: Optional[float] = None
    lanelet_id: Optional[int] = None
    label: str = "event_condition"


class GateKind(str, Enum):
    """How an action is triggered / how a serial step signals completion."""

    IMMEDIATE = "immediate"  # fire on the first tick (no wait)
    ACTION_DONE = "action_done"  # wait until a referenced action has fired
    LANE_REACHED = "lane_reached"  # wait until an actor reaches a lanelet
    STANDSTILL = "standstill"  # wait until an actor has stood still
    CONDITION = "condition"  # wait until a compiled event condition holds
    ALL_OF = "all_of"  # wait until every sub-gate is satisfied


@dataclass(frozen=True)
class Gate:
    """A trigger condition for an action (also used as a completion signal).

    A :class:`Gate` is a pure descriptor; :mod:`.codegen` renders it into a
    concrete :mod:`autoware_carla_scenario` condition expression.
    """

    kind: GateKind
    action_label: Optional[str] = None
    actor: Optional[str] = None
    lanelet_id: Optional[int] = None
    duration: Optional[float] = None
    condition: Optional[ConditionSpec] = None
    members: tuple["Gate", ...] = ()

    @staticmethod
    def immediate() -> "Gate":
        return Gate(kind=GateKind.IMMEDIATE)

    @staticmethod
    def action_done(action_label: str) -> "Gate":
        return Gate(kind=GateKind.ACTION_DONE, action_label=action_label)

    @staticmethod
    def lane_reached(actor: str, lanelet_id: int) -> "Gate":
        return Gate(kind=GateKind.LANE_REACHED, actor=actor, lanelet_id=lanelet_id)

    @staticmethod
    def standstill(actor: str, duration: float) -> "Gate":
        return Gate(kind=GateKind.STANDSTILL, actor=actor, duration=duration)

    @staticmethod
    def from_condition(condition: ConditionSpec) -> "Gate":
        return Gate(kind=GateKind.CONDITION, condition=condition)

    @staticmethod
    def all_of(members: "list[Gate]") -> "Gate":
        return Gate(kind=GateKind.ALL_OF, members=tuple(members))

    @property
    def is_immediate(self) -> bool:
        return self.kind is GateKind.IMMEDIATE


class SpecKind(str, Enum):
    """The kind of framework object a :class:`Spec` maps to."""

    TURN = "turn"
    LANE_CHANGE = "lane_change"
    RELATIVE_POSITION = "relative_position"
    TRAFFIC_SIGNAL = "traffic_signal"
    REACH_LANE = "reach_lane"
    STANDSTILL = "standstill"
    MIN_SPEED = "min_speed"
    COLLISION = "collision"
    TIMEOUT = "timeout"


class SpecRole(str, Enum):
    """Whether a :class:`Spec` is an action or a pass/fail condition."""

    ACTION = "action"
    PASS = "pass"
    FAIL = "fail"


#: Which :class:`SpecRole` each :class:`SpecKind` plays.
SPEC_ROLES: dict[SpecKind, SpecRole] = {
    SpecKind.TURN: SpecRole.ACTION,
    SpecKind.LANE_CHANGE: SpecRole.ACTION,
    SpecKind.RELATIVE_POSITION: SpecRole.ACTION,
    SpecKind.TRAFFIC_SIGNAL: SpecRole.ACTION,
    SpecKind.REACH_LANE: SpecRole.PASS,
    SpecKind.STANDSTILL: SpecRole.PASS,
    SpecKind.MIN_SPEED: SpecRole.FAIL,
    SpecKind.COLLISION: SpecRole.FAIL,
    SpecKind.TIMEOUT: SpecRole.FAIL,
}


@dataclass(frozen=True)
class RelativeSpawn:
    """A spawn position expressed relative to another actor.

    Attributes:
        reference: The name of the actor this spawn is relative to.
        side: ``"left"`` / ``"right"`` for a lateral neighbour, or ``None``.
        longitudinal: Signed metres along the reference actor's lane —
            positive is ahead of, negative is behind, the reference.
        anchor: The DSL time anchor (``"start"`` / ``"end"``); only
            ``"start"`` placements are resolved to a concrete spawn.
    """

    reference: str
    side: Optional[str] = None
    longitudinal: float = 0.0
    anchor: str = "start"


@dataclass
class ActorPlan:
    """A resolved actor (the ego or an NPC).

    Attributes:
        name: The DSL actor name (e.g. ``"ego"``, ``"npc"``).
        is_ego: Whether this actor is the ego vehicle.
        index: 1-based NPC index (used for the CARLA ``role_name``); ``0`` for
            the ego.
        vehicle_type: CARLA vehicle blueprint id.
        spawn_lanelet_id: Lanelet2 id of the spawn point, or ``None`` when
            unspecified.
        spawn_s: Longitudinal offset along the spawn lanelet.
        initial_speed_kmh: Initial speed in km/h.
        spawn_lane_index: A map-relative lane index (``lane(1, at: start)``)
            when the spawn was given by lane number rather than a lanelet id.
        relative_spawn: A placement relative to another actor, or ``None``.
    """

    name: str
    is_ego: bool
    index: int = 0
    vehicle_type: str = DEFAULT_VEHICLE_TYPE
    spawn_lanelet_id: Optional[int] = None
    spawn_s: float = 0.0
    initial_speed_kmh: float = 0.0
    spawn_lane_index: Optional[int] = None
    relative_spawn: Optional[RelativeSpawn] = None


@dataclass
class Spec:
    """A single action or condition to register on the generated scenario.

    Attributes:
        kind: The :class:`SpecKind`.
        actor: The DSL name of the actor the spec applies to.
        label: A stable, human-readable label for the framework object.
        params: Kind-specific parameters (e.g. ``{"direction": "left"}``).
        gate: Trigger condition for an action spec (ignored for pass/fail
            specs). Defaults to :meth:`Gate.immediate`.
        ordered: For a pass-condition spec, whether it belongs to a purely
            ``serial`` context and must therefore be satisfied in sequence
            relative to its ordered siblings.
    """

    kind: SpecKind
    actor: str
    label: str
    params: dict[str, Any] = field(default_factory=dict)
    gate: Gate = field(default_factory=Gate.immediate)
    ordered: bool = False

    @property
    def role(self) -> SpecRole:
        return SPEC_ROLES[self.kind]


@dataclass
class BehaviorResult:
    """What a single behaviour invocation contributes to the plan.

    Attributes:
        actions: Action specs to register (each receives a trigger gate).
        passes: Pass-condition specs.
        fails: Fail-condition specs.
        completion: How this behaviour signals completion for serial
            sequencing, or ``None`` when it does not advance the sequence
            (config-only or continuous-monitor behaviours).
    """

    actions: list[Spec] = field(default_factory=list)
    passes: list[Spec] = field(default_factory=list)
    fails: list[Spec] = field(default_factory=list)
    completion: Optional[Gate] = None


@dataclass
class ScenarioPlan:
    """The full semantic plan for one transpiled scenario.

    Attributes:
        name: The DSL scenario name.
        ego: The resolved ego actor.
        npcs: Resolved NPC actors, in declaration order.
        specs: Actions and conditions, in the order they were translated.
        variant_index: 0-based index when a ``one_of`` produced several
            variants; ``0`` for a single-variant scenario.
        variant_count: Total number of variants produced from the scenario.
    """

    name: str
    ego: ActorPlan
    npcs: list[ActorPlan] = field(default_factory=list)
    specs: list[Spec] = field(default_factory=list)
    variant_index: int = 0
    variant_count: int = 1
    #: The map named by a ``set_map(...)`` directive, surfaced as advisory
    #: metadata (the map is still chosen at run time), or ``None``.
    map_hint: Optional[str] = None
    #: Human-readable notes about approximated / unsupported constructs,
    #: rendered into the generated scenario and its README for transparency.
    notes: list[str] = field(default_factory=list)

    def actor(self, name: str) -> ActorPlan:
        """Return the actor named *name* (ego or an NPC).

        Raises:
            KeyError: If no actor with that name exists.
        """
        if name == self.ego.name:
            return self.ego
        for npc in self.npcs:
            if npc.name == name:
                return npc
        raise KeyError(name)

    def specs_for(self, role: SpecRole) -> list[Spec]:
        """Return all specs playing *role*, preserving order."""
        return [s for s in self.specs if s.role is role]
