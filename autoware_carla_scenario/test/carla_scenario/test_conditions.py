"""Unit tests for scenario pass/fail conditions."""

from __future__ import annotations

from typing import Optional
from unittest.mock import MagicMock

import pytest

from autoware_carla_scenario import (
    BaseCondition,
    EntityInAreaCondition,
    ScenarioResult,
    TimeoutCondition,
)
from autoware_carla_scenario.conditions.entity_in_area import _point_in_polygon_2d
from autoware_carla_scenario.coordinate.poses import CarlaWorldPose


class AlwaysPassCondition(BaseCondition):
    """Test helper: always returns a passing result."""

    def check(self, world: object, elapsed: float) -> Optional[ScenarioResult]:
        return ScenarioResult(
            passed=True, message="Always passes", elapsed_seconds=elapsed
        )


class AlwaysNoneCondition(BaseCondition):
    """Test helper: never triggers."""

    def check(self, world: object, elapsed: float) -> Optional[ScenarioResult]:
        return None


# ---------------------------------------------------------------------------
# TimeoutCondition – unit tests (no CARLA required)
# ---------------------------------------------------------------------------


class TestTimeoutCondition:
    def test_returns_none_before_timeout(self) -> None:
        condition = TimeoutCondition(timeout_seconds=10.0)
        world = MagicMock()
        result = condition.check(world, elapsed=5.0)
        assert result is None

    def test_returns_failure_at_timeout(self) -> None:
        condition = TimeoutCondition(timeout_seconds=10.0)
        world = MagicMock()
        result = condition.check(world, elapsed=10.0)
        assert result is not None
        assert result.passed is False
        assert result.elapsed_seconds == pytest.approx(10.0)

    def test_returns_failure_beyond_timeout(self) -> None:
        condition = TimeoutCondition(timeout_seconds=5.0)
        world = MagicMock()
        result = condition.check(world, elapsed=999.9)
        assert result is not None
        assert result.passed is False

    def test_default_timeout_is_60_seconds(self) -> None:
        condition = TimeoutCondition()
        assert condition.timeout_seconds == pytest.approx(60.0)

    def test_message_contains_timeout_info(self) -> None:
        condition = TimeoutCondition(timeout_seconds=30.0)
        world = MagicMock()
        result = condition.check(world, elapsed=30.0)
        assert result is not None
        assert "30" in result.message


# ---------------------------------------------------------------------------
# BaseCondition – abstract interface
# ---------------------------------------------------------------------------


class TestBaseCondition:
    def test_concrete_subclass_always_pass(self) -> None:
        cond = AlwaysPassCondition()
        result = cond.check(MagicMock(), elapsed=1.0)
        assert result is not None
        assert result.passed is True

    def test_concrete_subclass_always_none(self) -> None:
        cond = AlwaysNoneCondition()
        result = cond.check(MagicMock(), elapsed=1.0)
        assert result is None


# ---------------------------------------------------------------------------
# Integration tests – real CARLA world (skipped if CARLA unavailable)
# ---------------------------------------------------------------------------


class TestTimeoutConditionIntegration:
    """Uses a real CARLA world to test TimeoutCondition."""

    @pytest.fixture(autouse=True)
    def skip_if_no_carla(self, carla_queue) -> None:  # noqa: ANN001
        """Require the session-scoped carla_queue fixture."""

    def test_timeout_triggers_on_real_world(self, carla_queue) -> None:  # noqa: ANN001
        runner = carla_queue._runner
        world = runner._world or runner._client.get_world()
        condition = TimeoutCondition(timeout_seconds=0.0)
        result = condition.check(world, elapsed=1.0)
        assert result is not None
        assert result.passed is False


# ---------------------------------------------------------------------------
# _point_in_polygon_2d – geometry helper
# ---------------------------------------------------------------------------


def _square(
    cx: float = 0.0, cy: float = 0.0, half: float = 1.0
) -> list[CarlaWorldPose]:
    """Return four CarlaWorldPose vertices forming a square centred at (cx, cy)."""
    return [
        CarlaWorldPose(x=cx - half, y=cy - half, z=0.0),
        CarlaWorldPose(x=cx + half, y=cy - half, z=0.0),
        CarlaWorldPose(x=cx + half, y=cy + half, z=0.0),
        CarlaWorldPose(x=cx - half, y=cy + half, z=0.0),
    ]


class TestPointInPolygon2d:
    def test_centre_is_inside(self) -> None:
        assert _point_in_polygon_2d(0.0, 0.0, _square()) is True

    def test_outside_is_false(self) -> None:
        assert _point_in_polygon_2d(5.0, 5.0, _square()) is False

    def test_corner_vicinity_outside(self) -> None:
        assert _point_in_polygon_2d(1.5, 1.5, _square()) is False

    def test_near_edge_inside(self) -> None:
        assert _point_in_polygon_2d(0.9, 0.0, _square()) is True

    def test_triangle(self) -> None:
        triangle = [
            CarlaWorldPose(x=0.0, y=0.0, z=0.0),
            CarlaWorldPose(x=4.0, y=0.0, z=0.0),
            CarlaWorldPose(x=2.0, y=4.0, z=0.0),
        ]
        assert _point_in_polygon_2d(2.0, 1.0, triangle) is True
        assert _point_in_polygon_2d(3.5, 3.5, triangle) is False

    def test_boundary_included_by_default(self) -> None:
        # A point exactly on the bottom edge (y == -1.0) of the unit square.
        assert _point_in_polygon_2d(0.0, -1.0, _square()) is True

    def test_boundary_excluded_when_flag_false(self) -> None:
        assert (
            _point_in_polygon_2d(0.0, -1.0, _square(), include_boundary=False) is False
        )

    def test_interior_still_true_when_boundary_excluded(self) -> None:
        assert _point_in_polygon_2d(0.0, 0.0, _square(), include_boundary=False) is True


# ---------------------------------------------------------------------------
# EntityInAreaCondition – unit tests (no CARLA required)
# ---------------------------------------------------------------------------


def _make_world_with_actor(
    role_name: str, x: float, y: float, z: float = 0.0
) -> MagicMock:
    """Return a MagicMock CARLA world that contains a single actor at (x, y, z)."""
    location = MagicMock()
    location.x = x
    location.y = y
    location.z = z

    actor = MagicMock()
    actor.attributes = {"role_name": role_name}
    actor.get_location.return_value = location

    world = MagicMock()
    world.get_actors.return_value = [actor]
    return world


class TestEntityInAreaCondition:
    def _square_polygon(
        self, cx: float = 0.0, cy: float = 0.0, half: float = 5.0
    ) -> list[CarlaWorldPose]:
        return _square(cx, cy, half)

    def test_entity_inside_returns_pass(self) -> None:
        polygon = self._square_polygon()
        condition = EntityInAreaCondition("npc1", polygon)
        world = _make_world_with_actor("npc1", x=0.0, y=0.0)
        result = condition.check(world, elapsed=1.0)
        assert result is not None
        assert result.passed is True
        assert "npc1" in result.message

    def test_entity_outside_returns_none(self) -> None:
        polygon = self._square_polygon()
        condition = EntityInAreaCondition("npc1", polygon)
        world = _make_world_with_actor("npc1", x=10.0, y=10.0)
        result = condition.check(world, elapsed=1.0)
        assert result is None

    def test_entity_not_found_returns_none(self) -> None:
        polygon = self._square_polygon()
        condition = EntityInAreaCondition("npc1", polygon)
        world = _make_world_with_actor("other_actor", x=0.0, y=0.0)
        result = condition.check(world, elapsed=1.0)
        assert result is None

    def test_result_elapsed_seconds(self) -> None:
        polygon = self._square_polygon()
        condition = EntityInAreaCondition("npc1", polygon)
        world = _make_world_with_actor("npc1", x=0.0, y=0.0)
        result = condition.check(world, elapsed=3.5)
        assert result is not None
        assert result.elapsed_seconds == pytest.approx(3.5)

    def test_polygon_too_few_vertices_raises(self) -> None:
        with pytest.raises(ValueError, match="at least 3"):
            EntityInAreaCondition(
                "npc1", [CarlaWorldPose(0, 0, 0), CarlaWorldPose(1, 0, 0)]
            )

    def test_carla_world_pose_polygon_passthrough(self) -> None:
        """CarlaWorldPose vertices must be used directly without conversion."""
        polygon = [
            CarlaWorldPose(x=-2.0, y=-2.0, z=0.0),
            CarlaWorldPose(x=2.0, y=-2.0, z=0.0),
            CarlaWorldPose(x=0.0, y=2.0, z=0.0),
        ]
        condition = EntityInAreaCondition("npc1", polygon)
        world = _make_world_with_actor("npc1", x=0.0, y=0.0)
        result = condition.check(world, elapsed=2.0)
        assert result is not None
        assert result.passed is True

    def test_boundary_included_by_default(self) -> None:
        # Entity on the bottom edge of the square (y == -5.0).
        polygon = self._square_polygon()
        condition = EntityInAreaCondition("npc1", polygon)
        world = _make_world_with_actor("npc1", x=0.0, y=-5.0)
        assert condition.check(world, elapsed=1.0) is not None

    def test_boundary_excluded_when_flag_false(self) -> None:
        polygon = self._square_polygon()
        condition = EntityInAreaCondition("npc1", polygon, include_boundary=False)
        world = _make_world_with_actor("npc1", x=0.0, y=-5.0)
        assert condition.check(world, elapsed=1.0) is None

    def test_polygon_resolved_on_every_check(self) -> None:
        """_resolve_polygon is called each time check() is invoked."""
        polygon = [
            CarlaWorldPose(x=-1.0, y=-1.0, z=0.0),
            CarlaWorldPose(x=1.0, y=-1.0, z=0.0),
            CarlaWorldPose(x=0.0, y=1.0, z=0.0),
        ]
        condition = EntityInAreaCondition("npc1", polygon)

        world_in = _make_world_with_actor("npc1", x=0.0, y=0.0)
        assert condition.check(world_in, elapsed=1.0) is not None

        world_out = _make_world_with_actor("npc1", x=5.0, y=5.0)
        assert condition.check(world_out, elapsed=2.0) is None
