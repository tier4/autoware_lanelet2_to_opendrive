"""Condition for checking Autoware-specific vehicle state.

Checks properties that exist on the :class:`AutowareEntity` Python object
(engage status, control mode, etc.) rather than on the CARLA actor.
"""

from __future__ import annotations

from enum import Enum, auto
from typing import TYPE_CHECKING, Any, Optional, Union

from .base import BaseCondition, ScenarioResult

if TYPE_CHECKING:
    import carla

    from ..entity.autoware_entity import AutowareEntity


class AutowareStateField(Enum):
    """Autoware state fields available for condition checks.

    Attributes:
        ENGAGED: Whether the vehicle is engaged for autonomous control (``bool``).
        CONTROL_MODE: Current control mode as an integer.
            ``1`` = autonomous, etc.
    """

    ENGAGED = auto()
    CONTROL_MODE = auto()


class AutowareStateCondition(BaseCondition):
    """Condition that checks Autoware-specific vehicle state.

    Unlike CARLA-actor-based conditions, this condition queries the
    :class:`AutowareEntity` Python object directly for DDS-layer state
    such as engage status and control mode.

    Args:
        entity: The :class:`AutowareEntity` instance to inspect.
        field: Which state field to check.
        expected: The expected value.  For :attr:`AutowareStateField.ENGAGED`
            this should be ``bool``; for :attr:`AutowareStateField.CONTROL_MODE``
            this should be ``int``.
        label: Human-readable identifier for this condition.

    Example::

        from autoware_carla_scenario.conditions import (
            AutowareStateCondition,
            AutowareStateField,
            StickyCondition,
        )

        engaged_cond = StickyCondition(
            AutowareStateCondition(
                entity=autoware_entity,
                field=AutowareStateField.ENGAGED,
                expected=True,
                label="ego_engaged",
            )
        )
        self.register_pass_condition(engaged_cond)
    """

    def __init__(
        self,
        entity: AutowareEntity,
        field: AutowareStateField,
        expected: Union[bool, int],
        *,
        label: str,
    ) -> None:
        super().__init__(label=label)
        self._entity = entity
        self._field = field
        self._expected = expected

    def get_details(self) -> dict[str, Any]:
        return {
            "field": self._field.name,
            "expected": self._expected,
        }

    def _get_actual(self) -> Union[bool, int]:
        """Return the current value of the monitored state field."""
        if self._field is AutowareStateField.ENGAGED:
            return self._entity.is_engaged
        if self._field is AutowareStateField.CONTROL_MODE:
            return self._entity.control_mode
        msg = f"Unknown AutowareStateField: {self._field}"
        raise ValueError(msg)

    def check(
        self,
        world: "carla.World",  # noqa: ARG002
        elapsed: float,
    ) -> Optional[ScenarioResult]:
        """Return a result when the state field matches the expected value.

        Args:
            world: The CARLA world instance (unused, required by interface).
            elapsed: Elapsed time in seconds since the scenario started.

        Returns:
            :class:`ScenarioResult` with ``passed=True`` when the field
            equals the expected value, ``None`` otherwise.
        """
        actual = self._get_actual()
        if actual == self._expected:
            return ScenarioResult(
                passed=True,
                message=(
                    f"Autoware {self._field.name.lower()} is {actual!r}"
                    f" (expected {self._expected!r})"
                ),
                elapsed_seconds=elapsed,
            )
        return None
