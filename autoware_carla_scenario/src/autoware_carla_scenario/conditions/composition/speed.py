"""Speed condition for scenario evaluation."""

from __future__ import annotations

from enum import Enum, auto
from typing import TYPE_CHECKING, Any, Optional, Union

from ...entity_role import EntityRole
from ...kinematics import Vector3
from ...tick_snapshot import TickSnapshot
from ..base import ScenarioResult, find_actor_in_list
from ..comparison import ComparisonRule, ScalarComparisonRule
from .base import CompositionCondition

if TYPE_CHECKING:
    import carla

_NEAR_ZERO_THRESHOLD = 1e-12
"""Magnitude below which a forward vector is considered degenerate."""


class SpeedDirection(Enum):
    """Which speed component to evaluate.

    Attributes:
        LONGITUDINAL: Speed component along the reference direction.
        LATERAL: Speed component perpendicular to the reference direction.
        MAGNITUDE: Scalar speed (Euclidean magnitude of the velocity vector).
    """

    LONGITUDINAL = auto()
    LATERAL = auto()
    MAGNITUDE = auto()


class SpeedCoordinateSystem(Enum):
    """Coordinate system for velocity decomposition.

    Attributes:
        WORLD: Decompose velocity in the CARLA world frame.
            Longitudinal corresponds to the world x-axis (East),
            lateral corresponds to the world y-axis (South).
        ENTITY: Decompose velocity relative to a reference entity's orientation.
            Longitudinal is along the reference entity's forward direction,
            lateral is perpendicular (positive = left of the reference entity).
            Requires ``reference_entity_name`` to be set.
    """

    WORLD = auto()
    ENTITY = auto()


class SpeedCondition(CompositionCondition):
    """Pass condition that triggers when an entity's speed meets a comparison.

    Evaluates the specified speed component of an entity against a threshold
    using the given comparison rule.  The component can be the scalar
    magnitude, or the longitudinal / lateral projection in a chosen
    coordinate system.

    Args:
        entity_name: The ``role_name`` attribute of the actor to evaluate.
        value: Threshold speed (m/s) to compare against.
        rule: Comparison operator.
        direction: Which speed component to evaluate.
            Defaults to :attr:`SpeedDirection.MAGNITUDE`.
        coordinate_system: Coordinate system for longitudinal / lateral
            decomposition.  Ignored when *direction* is
            :attr:`SpeedDirection.MAGNITUDE`.
            Defaults to :attr:`SpeedCoordinateSystem.WORLD`.
        reference_entity_name: ``role_name`` of the entity whose orientation
            defines the reference frame.  Required when *coordinate_system*
            is :attr:`SpeedCoordinateSystem.ENTITY`.
        tolerance: Tolerance for :attr:`ComparisonRule.EQUAL_TO`.
            Defaults to ``1e-6``.
    """

    def __init__(
        self,
        entity_name: Union[EntityRole, str],
        value: float,
        rule: ComparisonRule,
        direction: SpeedDirection = SpeedDirection.MAGNITUDE,
        coordinate_system: SpeedCoordinateSystem = SpeedCoordinateSystem.WORLD,
        reference_entity_name: Union[EntityRole, str, None] = None,
        tolerance: float = 1e-6,
        *,
        label: str,
    ) -> None:
        if tolerance < 0:
            raise ValueError("tolerance must be non-negative")
        if (
            coordinate_system == SpeedCoordinateSystem.ENTITY
            and reference_entity_name is None
        ):
            raise ValueError(
                "reference_entity_name is required " "when coordinate_system is ENTITY"
            )
        super().__init__(entity_name=entity_name, label=label)
        self._comparison = ScalarComparisonRule(
            field="speed", rule=rule, value=value, tolerance=tolerance
        )
        self._direction = direction
        self._coordinate_system = coordinate_system
        self._reference_entity_name = reference_entity_name

    def get_details(self) -> dict[str, Any]:
        details = super().get_details()
        details.update(
            {
                "value": self._comparison.value,
                "rule": self._comparison.rule.name,
                "direction": self._direction.name,
                "coordinate_system": self._coordinate_system.name,
            }
        )
        if self._reference_entity_name is not None:
            details["reference_entity_name"] = str(self._reference_entity_name)
        return details

    def _extract_speed_component(
        self,
        entity: carla.Actor,
        actors: list[carla.Actor],
    ) -> Optional[float]:
        """Extract the relevant speed component from *entity*.

        Args:
            entity: The target actor whose velocity is measured.
            actors: Pre-fetched actor list from the world (avoids a second
                ``world.get_actors()`` call when looking up the reference
                entity).

        Returns:
            The speed value (m/s), or ``None`` if the reference entity
            required for :attr:`SpeedCoordinateSystem.ENTITY` cannot be
            found or has a degenerate forward vector.
        """
        velocity: carla.Vector3D = entity.get_velocity()
        vel = Vector3.from_carla_vector3d(velocity)

        if self._direction == SpeedDirection.MAGNITUDE:
            return vel.magnitude()

        if self._coordinate_system == SpeedCoordinateSystem.WORLD:
            if self._direction == SpeedDirection.LONGITUDINAL:
                return vel.x
            # LATERAL
            return vel.y

        # --- ENTITY coordinate system ---
        assert self._reference_entity_name is not None
        ref_entity = find_actor_in_list(actors, self._reference_entity_name)
        if ref_entity is None:
            return None

        ref_fwd_carla: carla.Vector3D = ref_entity.get_transform().get_forward_vector()
        fwd = Vector3(ref_fwd_carla.x, ref_fwd_carla.y, 0.0)
        fwd_mag = fwd.magnitude()
        if fwd_mag < _NEAR_ZERO_THRESHOLD:
            return None
        fwd_unit = fwd / fwd_mag

        if self._direction == SpeedDirection.LONGITUDINAL:
            return vel.dot(fwd_unit)

        # LATERAL: left direction in CARLA's left-handed frame.
        # Rotating forward (fx, fy) by 90° gives left = (fy, -fx).
        left_unit = Vector3(fwd_unit.y, -fwd_unit.x, 0.0)
        return vel.dot(left_unit)

    def _check(self, snapshot: TickSnapshot) -> Optional[ScenarioResult]:
        """Return a pass result if the entity's speed satisfies the rule.

        Args:
            snapshot: Immutable snapshot of the current tick state.

        Returns:
            :class:`ScenarioResult` with ``passed=True`` if the speed
            condition is met, ``None`` otherwise.
        """
        assert self._entity_name is not None
        actors: list[carla.Actor] = snapshot.world.get_actors()
        entity = find_actor_in_list(actors, self._entity_name)
        if entity is None:
            return None

        speed_component = self._extract_speed_component(entity, actors)
        if speed_component is None:
            return None

        if self._comparison.satisfied(speed_component):
            rule_text = self._comparison.rule.name.lower().replace("_", " ")
            return ScenarioResult(
                passed=True,
                message=(
                    f"Entity '{self._entity_name}' speed"
                    f" {self._direction.name.lower()}"
                    f" ({speed_component:.2f} m/s)"
                    f" {rule_text} {self._comparison.value:.2f} m/s"
                ),
                elapsed_seconds=snapshot.elapsed,
            )

        return None
