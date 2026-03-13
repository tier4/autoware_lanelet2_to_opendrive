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
class LaneletIdsConstraint:
    """Matches only lanelets whose ID is in the given list.

    Useful for re-running specific lanelets that failed during a previous
    sweep::

        - type: lanelet_ids
          ids: [31, 300, 242]
    """

    ids: tuple[int, ...] = field(default_factory=tuple)

    def evaluate(self, lanelet: Any) -> bool:
        return lanelet.id in self.ids


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
# Parsing & matching helpers
# ---------------------------------------------------------------------------

# Leaf constraints keyed by their ``type`` string.
_LEAF_REGISTRY: dict[str, type] = {
    "has_stop_line": HasStopLineConstraint,
    "has_traffic_light_stop_line": HasTrafficLightStopLineConstraint,
    "is_junction": IsJunctionConstraint,
    "lanelet_ids": LaneletIdsConstraint,
}


def parse_constraint(cfg: dict[str, Any]) -> Constraint:
    """Recursively instantiate a :class:`Constraint` tree from a YAML mapping.

    Supported composite types:

    * ``and`` — requires a ``constraints`` list of child configs.
    * ``or``  — requires a ``constraints`` list of child configs.
    * ``not`` — requires a single ``constraint`` child config.

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

    # --- leaf ---
    cls = _LEAF_REGISTRY.get(constraint_type)
    if cls is None:
        raise ValueError(
            f"Unknown constraint type: {constraint_type!r}. "
            f"Available leaves: {list(_LEAF_REGISTRY)}, "
            f"composites: ['and', 'or', 'not']"
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
) -> list[int]:
    """Return IDs of lanelets satisfying *all* constraints.

    Args:
        constraints: List of constraint objects to evaluate.
        lanelet_map: A ``lanelet2.core.LaneletMap`` instance.

    Returns:
        Sorted list of lanelet IDs that pass every constraint.
    """
    matched: list[int] = []
    for lanelet in lanelet_map.laneletLayer:
        if all(c.evaluate(lanelet) for c in constraints):
            matched.append(lanelet.id)
    matched.sort()
    logger.info("Constraint matching found %d lanelet(s): %s", len(matched), matched)
    return matched
