"""Constraint engine for filtering lanelets by declarative conditions.

Constraints are parsed from the ``sweep.constraints`` section of a scenario
YAML and evaluated against every lanelet in the map to produce a list of
matching lanelet IDs.

Composite constraints (``and``, ``or``, ``not``) allow arbitrary boolean
combinations::

    # YAML example — stop line that comes from a traffic light
    - type: and
      constraints:
        - type: has_stop_line
        - type: has_traffic_light_stop_line

    # Junction lanelet (has turn_direction tag)
    - type: is_junction

    # Traffic light on a junction — required for CARLA
    - type: and
      constraints:
        - type: has_traffic_light_stop_line
        - type: is_junction

    # Negation
    - type: not
      constraint:
        type: has_traffic_light_stop_line

Dead-end filtering::

    # Exclude dead-end lanelets (no following lanelet in routing graph)
    - type: no_deadend

Simple value constraints::

    # Match a specific lanelet by ID
    - type: equals
      value: 3002141

    # Match any lanelet (useful for debugging)
    - type: equals
      value: any

Set-level relational constraints (``previous_of``, ``following_of``) evaluate
inner constraints across all lanelets first, then resolve to the
previous/following neighbours via the routing graph::

    # Lanelets immediately before those with a traffic light stop line
    - type: previous_of
      constraints:
        - type: has_traffic_light_stop_line

    # Lanelets immediately after those with a stop line
    - type: following_of
      constraints:
        - type: has_stop_line
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Any, Protocol

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Constraint protocol & leaf implementations
# ---------------------------------------------------------------------------


class Constraint(Protocol):
    """Interface that all lanelet constraints must satisfy."""

    def evaluate(self, lanelet: Any) -> bool:
        """Return ``True`` if *lanelet* satisfies this constraint."""
        ...


@dataclass(frozen=True)
class HasStopLineConstraint:
    """Matches lanelets that own at least one stop-line regulatory element.

    Detection reuses the same three patterns as
    :func:`~autoware_carla_scenario.utils.stop_line._collect_stop_lines_from_reg_elems`:

    1. Traffic light RE with a ``stopLine`` attribute.
    2. Traffic sign RE with a ``ref_line`` parameter.
    3. Road marking RE with a ``refers`` parameter containing ``type="stop_line"``.
    """

    type: str = "has_stop_line"

    def evaluate(self, lanelet: Any) -> bool:
        """Return ``True`` if *lanelet* has at least one stop line."""
        from ..utils.stop_line import _collect_stop_lines_from_reg_elems

        seen_ids: set[int] = set()
        results = _collect_stop_lines_from_reg_elems(
            lanelet.regulatoryElements, seen_ids
        )
        return len(results) > 0


@dataclass(frozen=True)
class EqualsConstraint:
    """Matches a lanelet whose ID equals the given value.

    A general-purpose equality check on the lanelet ID::

        - type: equals
          value: 3002141

    Use ``"any"`` to match all lanelets (useful for debugging)::

        - type: equals
          value: any
    """

    value: int | str = 0

    def evaluate(self, lanelet: Any) -> bool:
        if self.value == "any":
            return True
        return lanelet.id == self.value


@dataclass(frozen=True)
class InSetConstraint:
    """Matches lanelets whose ID is contained in a given set of values.

    Useful for filtering by a pre-defined list of lanelet IDs (e.g. IDs
    loaded from a map configuration parameter)::

        - type: in_set
          values: [100, 200, 300]

    Combine with ``not`` to *exclude* specific IDs::

        - type: not
          constraint:
            type: in_set
            values: ${map.no_3d_model_lanelet_ids}
    """

    values: tuple[int, ...] = field(default_factory=tuple)
    _lookup: frozenset[int] = field(init=False, repr=False, compare=False)

    def __post_init__(self) -> None:
        object.__setattr__(self, "_lookup", frozenset(self.values))

    def evaluate(self, lanelet: Any) -> bool:
        return lanelet.id in self._lookup


@dataclass(frozen=True)
class HasTrafficLightStopLineConstraint:
    """Matches lanelets whose stop line originates from a traffic-light RE.

    Only Pattern 1 of ``_collect_stop_lines_from_reg_elems`` is checked:
    the regulatory element must have a ``stopLine`` attribute (i.e. it is a
    traffic-light regulatory element, not a traffic sign or road marking).
    """

    type: str = "has_traffic_light_stop_line"

    def evaluate(self, lanelet: Any) -> bool:
        """Return ``True`` if *lanelet* has a traffic-light stop line."""
        for reg_elem in lanelet.regulatoryElements:
            if hasattr(reg_elem, "stopLine"):
                try:
                    stop_line = reg_elem.stopLine
                    if stop_line is not None:
                        return True
                except Exception:
                    pass
        return False


@dataclass(frozen=True)
class IsJunctionConstraint:
    """Matches lanelets that are inside a junction (intersection).

    A lanelet is considered a junction lanelet when it has the
    ``turn_direction`` attribute in its tags.  This is the standard
    convention used in Lanelet2 maps for Autoware.

    .. note::

        In CARLA, traffic lights must be associated with a junction.
        If a lanelet with a traffic-light stop line is **not** inside a
        junction (i.e. lacks ``turn_direction``), the traffic light
        cannot be mapped to a CARLA junction and the scenario will fail.
        Use this constraint together with ``has_traffic_light_stop_line``
        to filter for valid lanelets::

            - type: and
              constraints:
                - type: has_traffic_light_stop_line
                - type: is_junction
    """

    type: str = "is_junction"

    def evaluate(self, lanelet: Any) -> bool:
        """Return ``True`` if *lanelet* has a ``turn_direction`` attribute."""
        return "turn_direction" in lanelet.attributes


@dataclass(frozen=True)
class NoDeadEndConstraint:
    """Matches lanelets that are **not** dead ends.

    A lanelet is considered a dead end when it has no following lanelets
    in the routing graph — i.e. the road terminates with no continuation.

    This is a pre-computed constraint: during binding, the routing graph
    is queried once to build a set of dead-end lanelet IDs.  The
    per-lanelet ``evaluate()`` call then performs a simple set-membership
    check.

    Usage::

        # Filter out dead-end lanelets
        - type: no_deadend

        # Combine with other constraints
        - type: and
          constraints:
            - type: no_deadend
            - type: has_traffic_light_stop_line
    """

    type: str = "no_deadend"

    def find_dead_end_ids(
        self, lanelet_map: Any, routing_graph: Any | None = None
    ) -> set[int]:
        """Return IDs of lanelets that are dead ends (no following lanelets)."""
        if routing_graph is None:
            routing_graph = create_routing_graph(lanelet_map)
        dead_ends: set[int] = set()
        for lanelet in lanelet_map.laneletLayer:
            if not routing_graph.following(lanelet):
                dead_ends.add(lanelet.id)
        logger.info(
            "no_deadend: %d dead-end lanelet(s) identified: %s",
            len(dead_ends),
            sorted(dead_ends),
        )
        return dead_ends

    def evaluate(self, lanelet: Any) -> bool:
        """Return ``True`` if *lanelet* is **not** a dead end."""
        return lanelet.id not in self._cached_ids  # type: ignore[attr-defined]


# ---------------------------------------------------------------------------
# Composite constraints (and / or / not)
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class AndConstraint:
    """Logical AND — all child constraints must be satisfied."""

    constraints: tuple[Constraint, ...] = field(default_factory=tuple)

    def evaluate(self, lanelet: Any) -> bool:
        return all(c.evaluate(lanelet) for c in self.constraints)


@dataclass(frozen=True)
class OrConstraint:
    """Logical OR — at least one child constraint must be satisfied."""

    constraints: tuple[Constraint, ...] = field(default_factory=tuple)

    def evaluate(self, lanelet: Any) -> bool:
        return any(c.evaluate(lanelet) for c in self.constraints)


@dataclass(frozen=True)
class NotConstraint:
    """Logical NOT — the child constraint must *not* be satisfied."""

    constraint: Constraint

    def evaluate(self, lanelet: Any) -> bool:
        return not self.constraint.evaluate(lanelet)


# ---------------------------------------------------------------------------
# Set-level relational constraints (previous_of / following_of)
# ---------------------------------------------------------------------------


def create_routing_graph(lanelet_map: Any) -> Any:
    """Build a routing graph for *lanelet_map* using default traffic rules."""
    import lanelet2.routing
    import lanelet2.traffic_rules

    traffic_rules = lanelet2.traffic_rules.create(
        lanelet2.traffic_rules.Locations.Germany,
        lanelet2.traffic_rules.Participants.Vehicle,
    )
    return lanelet2.routing.RoutingGraph(lanelet_map, traffic_rules)


@dataclass(frozen=True)
class PreviousOfConstraint:
    """Matches lanelets that are *previous* neighbours of the inner result set.

    Inner constraints are evaluated against the full map first.  Then the
    routing graph is queried for the **previous** lanelets (depth=1) of
    every match.  The resulting IDs are cached so that the per-lanelet
    ``evaluate()`` call is a simple set-membership check.
    """

    constraints: tuple[Constraint, ...] = field(default_factory=tuple)

    def find_matching_ids(
        self, lanelet_map: Any, routing_graph: Any | None = None
    ) -> set[int]:
        """Return IDs of lanelets previous to the inner-constraint matches."""
        inner_matched = find_matching_lanelets(list(self.constraints), lanelet_map)
        if routing_graph is None:
            routing_graph = create_routing_graph(lanelet_map)
        result: set[int] = set()
        for lid in inner_matched:
            lanelet = lanelet_map.laneletLayer[lid]
            for prev_ll in routing_graph.previous(lanelet):
                result.add(prev_ll.id)
        return result

    def evaluate(self, lanelet: Any) -> bool:
        """Check membership in pre-computed previous set."""
        return lanelet.id in self._cached_ids  # type: ignore[attr-defined]


@dataclass(frozen=True)
class FollowingOfConstraint:
    """Matches lanelets that are *following* neighbours of the inner result set.

    Behaves identically to :class:`PreviousOfConstraint` but queries
    ``routing_graph.following()`` instead.
    """

    constraints: tuple[Constraint, ...] = field(default_factory=tuple)

    def find_matching_ids(
        self, lanelet_map: Any, routing_graph: Any | None = None
    ) -> set[int]:
        """Return IDs of lanelets following the inner-constraint matches."""
        inner_matched = find_matching_lanelets(list(self.constraints), lanelet_map)
        if routing_graph is None:
            routing_graph = create_routing_graph(lanelet_map)
        result: set[int] = set()
        for lid in inner_matched:
            lanelet = lanelet_map.laneletLayer[lid]
            for fol_ll in routing_graph.following(lanelet):
                result.add(fol_ll.id)
        return result

    def evaluate(self, lanelet: Any) -> bool:
        """Check membership in pre-computed following set."""
        return lanelet.id in self._cached_ids  # type: ignore[attr-defined]


def _bind_set_level_constraints(
    constraint: Any, lanelet_map: Any, routing_graph: Any | None = None
) -> None:
    """Recursively pre-compute cached IDs for set-level constraints.

    Walks the constraint tree and, for every
    :class:`PreviousOfConstraint` / :class:`FollowingOfConstraint`, calls
    ``find_matching_ids`` and stores the result as ``_cached_ids`` using
    ``object.__setattr__`` (the dataclasses are frozen).
    """
    if isinstance(constraint, (PreviousOfConstraint, FollowingOfConstraint)):
        ids = constraint.find_matching_ids(lanelet_map, routing_graph)
        object.__setattr__(constraint, "_cached_ids", ids)
    if isinstance(constraint, NoDeadEndConstraint):
        ids = constraint.find_dead_end_ids(lanelet_map, routing_graph)
        object.__setattr__(constraint, "_cached_ids", ids)
    if hasattr(constraint, "constraints"):
        for child in constraint.constraints:
            _bind_set_level_constraints(child, lanelet_map, routing_graph)
    if hasattr(constraint, "constraint"):
        _bind_set_level_constraints(constraint.constraint, lanelet_map, routing_graph)


# ---------------------------------------------------------------------------
# Parsing & matching helpers
# ---------------------------------------------------------------------------

# Leaf constraints keyed by their ``type`` string.
_LEAF_REGISTRY: dict[str, type] = {
    "has_stop_line": HasStopLineConstraint,
    "has_traffic_light_stop_line": HasTrafficLightStopLineConstraint,
    "is_junction": IsJunctionConstraint,
    "equals": EqualsConstraint,
    "in_set": InSetConstraint,
    "no_deadend": NoDeadEndConstraint,
}


def parse_constraint(cfg: dict[str, Any]) -> Constraint:
    """Recursively instantiate a :class:`Constraint` tree from a YAML mapping.

    Supported composite types:

    * ``and`` — requires a ``constraints`` list of child configs.
    * ``or``  — requires a ``constraints`` list of child configs.
    * ``not`` — requires a single ``constraint`` child config.
    * ``previous_of`` — requires a ``constraints`` list; resolves to
      lanelets immediately *before* the inner matches via routing graph.
    * ``following_of`` — requires a ``constraints`` list; resolves to
      lanelets immediately *after* the inner matches via routing graph.

    All other ``type`` values are looked up in the leaf registry.

    Args:
        cfg: A dict with at least a ``type`` key.

    Returns:
        The corresponding (possibly composite) constraint instance.

    Raises:
        ValueError: If the ``type`` value is not recognised.
    """
    constraint_type = cfg.get("type")
    if constraint_type is None:
        raise ValueError(f"Constraint config is missing 'type': {cfg}")

    # --- composite: and ---
    if constraint_type == "and":
        children_cfg = cfg.get("constraints")
        if not children_cfg:
            raise ValueError("'and' constraint requires a 'constraints' list.")
        children = tuple(parse_constraint(c) for c in children_cfg)
        return AndConstraint(constraints=children)

    # --- composite: or ---
    if constraint_type == "or":
        children_cfg = cfg.get("constraints")
        if not children_cfg:
            raise ValueError("'or' constraint requires a 'constraints' list.")
        children = tuple(parse_constraint(c) for c in children_cfg)
        return OrConstraint(constraints=children)

    # --- composite: not ---
    if constraint_type == "not":
        child_cfg = cfg.get("constraint")
        if not child_cfg:
            raise ValueError("'not' constraint requires a 'constraint' mapping.")
        return NotConstraint(constraint=parse_constraint(child_cfg))

    # --- set-level: previous_of / following_of ---
    if constraint_type in ("previous_of", "following_of"):
        children_cfg = cfg.get("constraints")
        if not children_cfg:
            raise ValueError(
                f"'{constraint_type}' constraint requires a 'constraints' list."
            )
        children = tuple(parse_constraint(c) for c in children_cfg)
        if constraint_type == "previous_of":
            return PreviousOfConstraint(constraints=children)
        return FollowingOfConstraint(constraints=children)

    # --- leaf ---
    cls = _LEAF_REGISTRY.get(constraint_type)
    if cls is None:
        raise ValueError(
            f"Unknown constraint type: {constraint_type!r}. "
            f"Available leaves: {list(_LEAF_REGISTRY)}, "
            f"composites: ['and', 'or', 'not', 'previous_of', 'following_of']"
        )
    kwargs = {k: v for k, v in cfg.items() if k != "type"}
    # Convert list values to tuples for frozen dataclasses.
    for k, v in kwargs.items():
        if isinstance(v, list):
            kwargs[k] = tuple(v)
    return cls(**kwargs)


def find_matching_lanelets(
    constraints: list[Constraint],
    lanelet_map: Any,
    routing_graph: Any | None = None,
) -> list[int]:
    """Return IDs of lanelets satisfying *all* constraints.

    Args:
        constraints: List of constraint objects to evaluate.
        lanelet_map: A ``lanelet2.core.LaneletMap`` instance.
        routing_graph: Optional pre-built routing graph.  When *None* a new
            graph is created internally only if set-level constraints exist.

    Returns:
        Sorted list of lanelet IDs that pass every constraint.
    """
    # Pre-compute cached IDs for set-level constraints (previous_of, following_of,
    # no_deadend) so that their evaluate() calls become simple lookups.
    for c in constraints:
        _bind_set_level_constraints(c, lanelet_map, routing_graph)

    matched: list[int] = []
    for lanelet in lanelet_map.laneletLayer:
        if all(c.evaluate(lanelet) for c in constraints):
            matched.append(lanelet.id)
    matched.sort()
    logger.info("Constraint matching found %d lanelet(s): %s", len(matched), matched)
    return matched
