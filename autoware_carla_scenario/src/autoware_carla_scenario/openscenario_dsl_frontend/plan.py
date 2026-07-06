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


class SpecKind(str, Enum):
    """The kind of framework object a :class:`Spec` maps to."""

    TURN = "turn"
    LANE_CHANGE = "lane_change"
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
    SpecKind.TRAFFIC_SIGNAL: SpecRole.ACTION,
    SpecKind.REACH_LANE: SpecRole.PASS,
    SpecKind.STANDSTILL: SpecRole.PASS,
    SpecKind.MIN_SPEED: SpecRole.FAIL,
    SpecKind.COLLISION: SpecRole.FAIL,
    SpecKind.TIMEOUT: SpecRole.FAIL,
}


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
    """

    name: str
    is_ego: bool
    index: int = 0
    vehicle_type: str = DEFAULT_VEHICLE_TYPE
    spawn_lanelet_id: Optional[int] = None
    spawn_s: float = 0.0
    initial_speed_kmh: float = 0.0


@dataclass
class Spec:
    """A single action or condition to register on the generated scenario.

    Attributes:
        kind: The :class:`SpecKind`.
        actor: The DSL name of the actor the spec applies to.
        label: A stable, human-readable label for the framework object.
        params: Kind-specific parameters (e.g. ``{"direction": "left"}``).
    """

    kind: SpecKind
    actor: str
    label: str
    params: dict[str, Any] = field(default_factory=dict)

    @property
    def role(self) -> SpecRole:
        return SPEC_ROLES[self.kind]


@dataclass
class ScenarioPlan:
    """The full semantic plan for one transpiled scenario.

    Attributes:
        name: The DSL scenario name.
        ego: The resolved ego actor.
        npcs: Resolved NPC actors, in declaration order.
        specs: Actions and conditions, in the order they were translated.
    """

    name: str
    ego: ActorPlan
    npcs: list[ActorPlan] = field(default_factory=list)
    specs: list[Spec] = field(default_factory=list)

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
