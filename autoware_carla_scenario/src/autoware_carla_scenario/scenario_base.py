"""Base class for CARLA scenarios."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import TYPE_CHECKING, Callable, List

from .conditions import BaseCondition
from .entity._spawn import SpawnLocation

if TYPE_CHECKING:
    import carla


@dataclass
class EgoConfig:
    """Configuration for the ego vehicle.

    *spawn_location* determines where the vehicle is placed — either a
    :class:`SpawnTransform` (explicit pose) or a :class:`SpawnPointIndex`
    (index into the map's spawn-point list).
    """

    spawn_location: SpawnLocation
    vehicle_type: str = "vehicle.fuso.mitsubishi"


class BaseScenario(ABC):
    """Abstract base class for CARLA test scenarios.

    Subclasses must implement :meth:`setup` and :meth:`is_done`.
    The ego vehicle is mandatory and must be provided via *ego_config*.
    """

    def __init__(self, ego_config: EgoConfig) -> None:
        """Initialize the scenario with an ego vehicle configuration.

        Args:
            ego_config: Spawn configuration for the ego vehicle.
        """
        self.ego_config = ego_config
        self._pre_tick_callbacks: List[Callable[["carla.World"], None]] = []
        self._post_tick_callbacks: List[Callable[["carla.World"], None]] = []
        self._pass_conditions: List[BaseCondition] = []
        self._fail_conditions: List[BaseCondition] = []

    # ------------------------------------------------------------------
    # Abstract interface – must be implemented by subclasses
    # ------------------------------------------------------------------

    @abstractmethod
    def setup(self, world: "carla.World") -> None:
        """Set up the scenario actors and environment.

        Args:
            world: The CARLA world instance.
        """
        ...

    @abstractmethod
    def is_done(self) -> bool:
        """Return True when the scenario should stop ticking.

        This is separate from pass/fail conditions and can be used to
        signal completion for scenarios that have a natural end point.
        """
        ...

    # ------------------------------------------------------------------
    # Callback registration helpers
    # ------------------------------------------------------------------

    def register_pre_tick(self, cb: Callable[["carla.World"], None]) -> None:
        """Register a callback to run *before* each world tick.

        Args:
            cb: Callable that receives the CARLA world as its argument.
        """
        self._pre_tick_callbacks.append(cb)

    def register_post_tick(self, cb: Callable[["carla.World"], None]) -> None:
        """Register a callback to run *after* each world tick.

        Args:
            cb: Callable that receives the CARLA world as its argument.
        """
        self._post_tick_callbacks.append(cb)

    def register_pass_condition(self, condition: BaseCondition) -> None:
        """Register a condition that marks the scenario as *passed*.

        Args:
            condition: Condition to add to the pass-condition list.
        """
        self._pass_conditions.append(condition)

    def register_fail_condition(self, condition: BaseCondition) -> None:
        """Register a condition that marks the scenario as *failed*.

        Args:
            condition: Condition to add to the fail-condition list.
        """
        self._fail_conditions.append(condition)
