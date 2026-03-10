"""Speed condition for scenario evaluation."""

from __future__ import annotations

from enum import Enum, auto
from typing import TYPE_CHECKING, Optional

from ..kinematics import Vector3
from .base import BaseCondition, ScenarioResult

if TYPE_CHECKING:
    import carla

_NEAR_ZERO_THRESHOLD = 1e-12
"""Magnitude below which a forward vector is considered degenerate."""


def _find_actor_in_list(
    actors: list[carla.Actor],
    role_name: str,
) -> Optional[carla.Actor]:
    """Find an actor by ``role_name`` in a pre-fetched actor list."""
    for actor in actors:
        if actor.attributes.get("role_name") == role_name:
            return actor
    return None


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


class ComparisonRule(Enum):
    """Comparison operator for condition evaluation.

    Attributes:
        GREATER_THAN: Value must be strictly greater than the threshold.
        LESS_THAN: Value must be strictly less than the threshold.
        EQUAL_TO: Value must be approximately equal (within tolerance).
        GREATER_THAN_OR_EQUAL: Value must be greater than or equal.
        LESS_THAN_OR_EQUAL: Value must be less than or equal.
    """

    GREATER_THAN = auto()
    LESS_THAN = auto()
    EQUAL_TO = auto()
    GREATER_THAN_OR_EQUAL = auto()
    LESS_THAN_OR_EQUAL = auto()


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


class SpeedCondition(BaseCondition):
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
        entity_name: str,
        value: float,
        rule: ComparisonRule,
        direction: SpeedDirection = SpeedDirection.MAGNITUDE,
        coordinate_system: SpeedCoordinateSystem = SpeedCoordinateSystem.WORLD,
        reference_entity_name: Optional[str] = None,
        tolerance: float = 1e-6,
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
        self._entity_name = entity_name
        self._value = value
        self._rule = rule
        self._direction = direction
        self._coordinate_system = coordinate_system
        self._reference_entity_name = reference_entity_name
        self._tolerance = tolerance

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
        ref_entity = _find_actor_in_list(actors, self._reference_entity_name)
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

    def _compare(self, actual: float) -> bool:
        """Return ``True`` if *actual* satisfies the configured rule."""
        if self._rule == ComparisonRule.GREATER_THAN:
            return actual > self._value
        if self._rule == ComparisonRule.LESS_THAN:
            return actual < self._value
        if self._rule == ComparisonRule.EQUAL_TO:
            return abs(actual - self._value) <= self._tolerance
        if self._rule == ComparisonRule.GREATER_THAN_OR_EQUAL:
            return actual >= self._value
        if self._rule == ComparisonRule.LESS_THAN_OR_EQUAL:
            return actual <= self._value
        raise ValueError(f"Unknown comparison rule: {self._rule}")

    def check(self, world: carla.World, elapsed: float) -> Optional[ScenarioResult]:
        """Return a pass result if the entity's speed satisfies the rule.

        Args:
            world: The CARLA world instance.
            elapsed: Elapsed time in seconds since the scenario started.

        Returns:
            :class:`ScenarioResult` with ``passed=True`` if the speed
            condition is met, ``None`` otherwise.
        """
        actors: list[carla.Actor] = world.get_actors()
        entity = _find_actor_in_list(actors, self._entity_name)
        if entity is None:
            return None

        speed_component = self._extract_speed_component(entity, actors)
        if speed_component is None:
            return None

        if self._compare(speed_component):
            rule_text = self._rule.name.lower().replace("_", " ")
            return ScenarioResult(
                passed=True,
                message=(
                    f"Entity '{self._entity_name}' speed"
                    f" {self._direction.name.lower()}"
                    f" ({speed_component:.2f} m/s)"
                    f" {rule_text} {self._value:.2f} m/s"
                ),
                elapsed_seconds=elapsed,
            )

        return None
