"""Base classes and helpers for scenario pass/fail conditions."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Optional, Union

from ..entity_role import EntityRole

if TYPE_CHECKING:
    import carla


def find_actor_in_list(
    actors: "list[carla.Actor]", role_name: Union[EntityRole, str]
) -> Optional["carla.Actor"]:
    """Find a single actor by its ``role_name`` in a pre-fetched actor list.

    Args:
        actors: Pre-fetched actor list (e.g. from ``world.get_actors()``).
        role_name: The ``role_name`` attribute value to search for.
            Accepts both :class:`EntityRole` and plain ``str``.

    Returns:
        The matching actor, or ``None`` if no actor with that role name exists.
    """
    name = str(role_name)
    return next(
        (a for a in actors if a.attributes.get("role_name") == name),
        None,
    )


def find_actor_by_role_name(
    world: "carla.World", role_name: Union[EntityRole, str]
) -> Optional["carla.Actor"]:
    """Find a single actor by its ``role_name`` attribute.

    Args:
        world: The CARLA world instance.
        role_name: The ``role_name`` attribute value to search for.
            Accepts both :class:`EntityRole` and plain ``str``.

    Returns:
        The matching actor, or ``None`` if no actor with that role name exists.
    """
    return find_actor_in_list(world.get_actors(), role_name)


@dataclass
class ConditionStatus:
    """Snapshot of a single condition's state when the scenario ended."""

    label: str
    satisfied: bool
    message: str


@dataclass
class ScenarioResult:
    """Result of a scenario execution."""

    passed: bool
    message: str
    elapsed_seconds: float
    condition_statuses: list[ConditionStatus] = field(default_factory=list)


class BaseCondition(ABC):
    """Abstract base class for scenario pass/fail conditions."""

    def __init__(self, label: str) -> None:
        if not label:
            raise ValueError(
                f"{type(self).__name__}: label must not be empty. "
                "Provide a non-empty string to identify this condition."
            )
        self.label = label

    @abstractmethod
    def check(self, world: "carla.World", elapsed: float) -> Optional[ScenarioResult]:
        """Check the condition.

        Args:
            world: The CARLA world instance.
            elapsed: Elapsed time in seconds since the scenario started.

        Returns:
            A ScenarioResult if the condition is met, None otherwise.
        """
        ...
