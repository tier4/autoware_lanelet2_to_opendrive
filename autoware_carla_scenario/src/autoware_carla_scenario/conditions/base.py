"""Base classes and helpers for scenario pass/fail conditions."""

from __future__ import annotations

import json
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Optional, Union

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
    """Snapshot of a single condition's state when the scenario ended.

    Attributes:
        label: Human-readable identifier (e.g. ``"pass[0](elapsed_time_30s)"``).
        satisfied: Whether the condition was satisfied at snapshot time.
        message: Human-readable description of the state.
        condition_type: Class name of the condition (e.g. ``"TimeoutCondition"``).
        role: Whether this is a ``"pass"`` or ``"fail"`` condition.
        details: Structured key/value pairs specific to the condition type.
    """

    label: str
    satisfied: bool
    message: str
    condition_type: str = ""
    role: str = ""
    details: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        """Return a JSON-serialisable nested dict."""
        return {
            "label": self.label,
            "satisfied": self.satisfied,
            "message": self.message,
            "condition_type": self.condition_type,
            "role": self.role,
            "details": self.details,
        }


@dataclass
class ScenarioResult:
    """Result of a scenario execution."""

    passed: bool
    message: str
    elapsed_seconds: float
    condition_statuses: list[ConditionStatus] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        """Return a JSON-serialisable nested dict of the full result."""
        return {
            "passed": self.passed,
            "message": self.message,
            "elapsed_seconds": self.elapsed_seconds,
            "condition_statuses": [cs.to_dict() for cs in self.condition_statuses],
        }

    def to_json(self, **kwargs: Any) -> str:
        """Serialise to a JSON string.

        Args:
            **kwargs: Forwarded to :func:`json.dumps`
                (e.g. ``indent=2``, ``ensure_ascii=False``).
        """
        kwargs.setdefault("ensure_ascii", False)
        return json.dumps(self.to_dict(), **kwargs)


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

    def get_details(self) -> dict[str, Any]:
        """Return structured details about this condition's configuration.

        Subclasses should override this to expose condition-specific
        parameters (thresholds, targets, etc.) for structured JSON logging.
        """
        return {}

    def to_summary_dict(self) -> dict[str, Any]:
        """Return a summary dict identifying this condition and its details.

        Combines ``condition_type``, ``label``, and :meth:`get_details`
        into a single dict suitable for nesting inside parent conditions.
        """
        return {
            "condition_type": type(self).__name__,
            "label": self.label,
            **self.get_details(),
        }
