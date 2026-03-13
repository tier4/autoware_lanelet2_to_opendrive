"""Unit tests for scenario pass/fail conditions."""

from __future__ import annotations

import unittest.mock
from typing import Optional
from unittest.mock import MagicMock

import pytest

from autoware_carla_scenario import (
    AndCondition,
    BaseCondition,
    ComparisonRule,
    ElapsedTimeCondition,
    EntityInAreaCondition,
    EntityLanePositionCondition,
    OrCondition,
    ScalarComparisonRule,
    ScenarioResult,
    SpeedCondition,
    SpeedCoordinateSystem,
    SpeedDirection,
    StandstillCondition,
    StickyCondition,
    TimeoutCondition,
)
from autoware_carla_scenario.conditions.composition.entity_in_area import (
    _point_in_polygon_2d,
)
from autoware_carla_scenario.coordinate.poses import CarlaWorldPose, OpenDrivePose


class AlwaysPassCondition(BaseCondition):
    """Test helper: always returns a passing result."""

    def __init__(self) -> None:
        super().__init__(label="always_pass")

    def check(self, world: object, elapsed: float) -> Optional[ScenarioResult]:
        return ScenarioResult(
            passed=True, message="Always passes", elapsed_seconds=elapsed
        )


class AlwaysNoneCondition(BaseCondition):
    """Test helper: never triggers."""

    def __init__(self) -> None:
        super().__init__(label="always_none")

    def check(self, world: object, elapsed: float) -> Optional[ScenarioResult]:
        return None


# ---------------------------------------------------------------------------
# TimeoutCondition – unit tests (no CARLA required)
# ---------------------------------------------------------------------------


class TestTimeoutCondition:
    def test_returns_none_before_timeout(self) -> None:
        condition = TimeoutCondition(timeout_seconds=10.0, label="test_timeout")
        world = MagicMock()
        result = condition.check(world, elapsed=5.0)
        assert result is None

    def test_returns_failure_at_timeout(self) -> None:
        condition = TimeoutCondition(timeout_seconds=10.0, label="test_timeout")
        world = MagicMock()
        result = condition.check(world, elapsed=10.0)
        assert result is not None
        assert result.passed is False
        assert result.elapsed_seconds == pytest.approx(10.0)

    def test_returns_failure_beyond_timeout(self) -> None:
        condition = TimeoutCondition(timeout_seconds=5.0, label="test_timeout")
        world = MagicMock()
        result = condition.check(world, elapsed=999.9)
        assert result is not None
        assert result.passed is False

    def test_default_timeout_is_60_seconds(self) -> None:
        condition = TimeoutCondition(label="test_timeout")
        assert condition.timeout_seconds == pytest.approx(60.0)

    def test_message_contains_timeout_info(self) -> None:
        condition = TimeoutCondition(timeout_seconds=30.0, label="test_timeout")
        world = MagicMock()
        result = condition.check(world, elapsed=30.0)
        assert result is not None
        assert "30" in result.message


# ---------------------------------------------------------------------------
# ElapsedTimeCondition – unit tests (no CARLA required)
# ---------------------------------------------------------------------------


class TestElapsedTimeCondition:
    def test_returns_none_before_duration(self) -> None:
        condition = ElapsedTimeCondition(duration_seconds=10.0, label="test_elapsed")
        world = MagicMock()
        result = condition.check(world, elapsed=5.0)
        assert result is None

    def test_returns_pass_at_duration(self) -> None:
        condition = ElapsedTimeCondition(duration_seconds=10.0, label="test_elapsed")
        world = MagicMock()
        result = condition.check(world, elapsed=10.0)
        assert result is not None
        assert result.passed is True
        assert result.elapsed_seconds == pytest.approx(10.0)

    def test_returns_pass_beyond_duration(self) -> None:
        condition = ElapsedTimeCondition(duration_seconds=5.0, label="test_elapsed")
        world = MagicMock()
        result = condition.check(world, elapsed=999.9)
        assert result is not None
        assert result.passed is True

    def test_message_contains_duration_info(self) -> None:
        condition = ElapsedTimeCondition(duration_seconds=30.0, label="test_elapsed")
        world = MagicMock()
        result = condition.check(world, elapsed=30.0)
        assert result is not None
        assert "30.00" in result.message

    def test_zero_duration_raises(self) -> None:
        with pytest.raises(ValueError, match="duration_seconds must be positive"):
            ElapsedTimeCondition(duration_seconds=0.0, label="test_elapsed")

    def test_negative_duration_raises(self) -> None:
        with pytest.raises(ValueError, match="duration_seconds must be positive"):
            ElapsedTimeCondition(duration_seconds=-5.0, label="test_elapsed")


class TestElapsedTimeConditionWithRule:
    """ElapsedTimeCondition with explicit ComparisonRule."""

    def test_greater_than(self) -> None:
        cond = ElapsedTimeCondition(
            duration_seconds=10.0,
            rule=ComparisonRule.GREATER_THAN,
            label="test_elapsed",
        )
        world = MagicMock()
        assert cond.check(world, elapsed=10.0) is None  # not strictly greater
        result = cond.check(world, elapsed=10.1)
        assert result is not None
        assert result.passed is True

    def test_less_than(self) -> None:
        cond = ElapsedTimeCondition(
            duration_seconds=10.0, rule=ComparisonRule.LESS_THAN, label="test_elapsed"
        )
        world = MagicMock()
        result = cond.check(world, elapsed=5.0)
        assert result is not None
        assert result.passed is True
        assert cond.check(world, elapsed=10.0) is None

    def test_equal_to_within_tolerance(self) -> None:
        cond = ElapsedTimeCondition(
            duration_seconds=10.0,
            rule=ComparisonRule.EQUAL_TO,
            tolerance=0.01,
            label="test_elapsed",
        )
        world = MagicMock()
        result = cond.check(world, elapsed=10.005)
        assert result is not None
        assert result.passed is True

    def test_equal_to_outside_tolerance(self) -> None:
        cond = ElapsedTimeCondition(
            duration_seconds=10.0,
            rule=ComparisonRule.EQUAL_TO,
            tolerance=0.001,
            label="test_elapsed",
        )
        world = MagicMock()
        assert cond.check(world, elapsed=10.1) is None

    def test_less_than_or_equal(self) -> None:
        cond = ElapsedTimeCondition(
            duration_seconds=10.0,
            rule=ComparisonRule.LESS_THAN_OR_EQUAL,
            label="test_elapsed",
        )
        world = MagicMock()
        result = cond.check(world, elapsed=10.0)
        assert result is not None
        assert result.passed is True
        assert cond.check(world, elapsed=10.1) is None

    def test_default_rule_is_greater_than_or_equal(self) -> None:
        """Backward compatibility: default behaves as >= (original semantics)."""
        cond = ElapsedTimeCondition(duration_seconds=10.0, label="test_elapsed")
        world = MagicMock()
        result = cond.check(world, elapsed=10.0)
        assert result is not None
        assert result.passed is True

    def test_negative_tolerance_raises(self) -> None:
        with pytest.raises(ValueError, match="tolerance must be non-negative"):
            ElapsedTimeCondition(
                duration_seconds=10.0,
                rule=ComparisonRule.EQUAL_TO,
                tolerance=-1.0,
                label="test_elapsed",
            )

    def test_message_contains_rule_text(self) -> None:
        cond = ElapsedTimeCondition(
            duration_seconds=5.0, rule=ComparisonRule.LESS_THAN, label="test_elapsed"
        )
        world = MagicMock()
        result = cond.check(world, elapsed=3.0)
        assert result is not None
        assert "less than" in result.message


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


@pytest.mark.integration
class TestTimeoutConditionIntegration:
    """Uses a real CARLA world to test TimeoutCondition."""

    @pytest.fixture(autouse=True)
    def skip_if_no_carla(self, carla_queue) -> None:  # noqa: ANN001
        """Require the session-scoped carla_queue fixture."""

    def test_timeout_triggers_on_real_world(self, carla_queue) -> None:  # noqa: ANN001
        runner = carla_queue._runner
        world = runner._world or runner._client.get_world()
        condition = TimeoutCondition(timeout_seconds=0.0, label="test_timeout")
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
    role_name: str,
    x: float,
    y: float,
    z: float = 0.0,
    *,
    vx: float | None = None,
    vy: float | None = None,
    vz: float | None = None,
) -> MagicMock:
    """Return a MagicMock CARLA world that contains a single actor.

    Always stubs ``get_location`` with *(x, y, z)*.  When *vx*/*vy*/*vz* are
    provided, ``get_velocity`` is also stubbed.
    """
    location = MagicMock()
    location.x = x
    location.y = y
    location.z = z

    actor = MagicMock()
    actor.attributes = {"role_name": role_name}
    actor.get_location.return_value = location

    if vx is not None:
        velocity = MagicMock()
        velocity.x = vx
        velocity.y = vy if vy is not None else 0.0
        velocity.z = vz if vz is not None else 0.0
        actor.get_velocity.return_value = velocity

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
        condition = EntityInAreaCondition("npc1", polygon, label="test_entity_in_area")
        world = _make_world_with_actor("npc1", x=0.0, y=0.0)
        result = condition.check(world, elapsed=1.0)
        assert result is not None
        assert result.passed is True
        assert "npc1" in result.message

    def test_entity_outside_returns_none(self) -> None:
        polygon = self._square_polygon()
        condition = EntityInAreaCondition("npc1", polygon, label="test_entity_in_area")
        world = _make_world_with_actor("npc1", x=10.0, y=10.0)
        result = condition.check(world, elapsed=1.0)
        assert result is None

    def test_entity_not_found_returns_none(self) -> None:
        polygon = self._square_polygon()
        condition = EntityInAreaCondition("npc1", polygon, label="test_entity_in_area")
        world = _make_world_with_actor("other_actor", x=0.0, y=0.0)
        result = condition.check(world, elapsed=1.0)
        assert result is None

    def test_result_elapsed_seconds(self) -> None:
        polygon = self._square_polygon()
        condition = EntityInAreaCondition("npc1", polygon, label="test_entity_in_area")
        world = _make_world_with_actor("npc1", x=0.0, y=0.0)
        result = condition.check(world, elapsed=3.5)
        assert result is not None
        assert result.elapsed_seconds == pytest.approx(3.5)

    def test_polygon_too_few_vertices_raises(self) -> None:
        with pytest.raises(ValueError, match="at least 3"):
            EntityInAreaCondition(
                "npc1",
                [CarlaWorldPose(0, 0, 0), CarlaWorldPose(1, 0, 0)],
                label="test_entity_in_area",
            )

    def test_carla_world_pose_polygon_passthrough(self) -> None:
        """CarlaWorldPose vertices must be used directly without conversion."""
        polygon = [
            CarlaWorldPose(x=-2.0, y=-2.0, z=0.0),
            CarlaWorldPose(x=2.0, y=-2.0, z=0.0),
            CarlaWorldPose(x=0.0, y=2.0, z=0.0),
        ]
        condition = EntityInAreaCondition("npc1", polygon, label="test_entity_in_area")
        world = _make_world_with_actor("npc1", x=0.0, y=0.0)
        result = condition.check(world, elapsed=2.0)
        assert result is not None
        assert result.passed is True

    def test_boundary_included_by_default(self) -> None:
        # Entity on the bottom edge of the square (y == -5.0).
        polygon = self._square_polygon()
        condition = EntityInAreaCondition("npc1", polygon, label="test_entity_in_area")
        world = _make_world_with_actor("npc1", x=0.0, y=-5.0)
        assert condition.check(world, elapsed=1.0) is not None

    def test_boundary_excluded_when_flag_false(self) -> None:
        polygon = self._square_polygon()
        condition = EntityInAreaCondition(
            "npc1", polygon, include_boundary=False, label="test_entity_in_area"
        )
        world = _make_world_with_actor("npc1", x=0.0, y=-5.0)
        assert condition.check(world, elapsed=1.0) is None

    def test_polygon_resolved_on_every_check(self) -> None:
        """_resolve_polygon is called each time check() is invoked."""
        polygon = [
            CarlaWorldPose(x=-1.0, y=-1.0, z=0.0),
            CarlaWorldPose(x=1.0, y=-1.0, z=0.0),
            CarlaWorldPose(x=0.0, y=1.0, z=0.0),
        ]
        condition = EntityInAreaCondition("npc1", polygon, label="test_entity_in_area")

        world_in = _make_world_with_actor("npc1", x=0.0, y=0.0)
        assert condition.check(world_in, elapsed=1.0) is not None

        world_out = _make_world_with_actor("npc1", x=5.0, y=5.0)
        assert condition.check(world_out, elapsed=2.0) is None


# ---------------------------------------------------------------------------
# EntityLanePositionCondition – unit tests (no CARLA required)
# ---------------------------------------------------------------------------


class TestEntityLanePositionCondition:
    def test_entity_on_matching_road_lane_returns_pass(self) -> None:
        """Condition triggers when entity is on the specified road and lane."""
        condition = EntityLanePositionCondition(
            "npc1", road_id="1", lane_id=-1, label="test_lane_pos"
        )
        world = _make_world_with_actor("npc1", x=100.0, y=200.0)

        fake_od_pose = OpenDrivePose(road_id="1", lane_id=-1, s=10.0, t=-1.5)
        with unittest.mock.patch(
            "autoware_carla_scenario.conditions.composition.entity_lane_position.to_opendrive",
            return_value=fake_od_pose,
        ):
            result = condition.check(world, elapsed=5.0)

        assert result is not None
        assert result.passed is True
        assert "npc1" in result.message
        assert "'1'" in result.message
        assert "-1" in result.message
        assert result.elapsed_seconds == pytest.approx(5.0)

    def test_entity_on_different_road_returns_none(self) -> None:
        """Condition does not trigger when entity is on a different road."""
        condition = EntityLanePositionCondition(
            "npc1", road_id="1", lane_id=-1, label="test_lane_pos"
        )
        world = _make_world_with_actor("npc1", x=100.0, y=200.0)

        fake_od_pose = OpenDrivePose(road_id="2", lane_id=-1, s=10.0, t=-1.5)
        with unittest.mock.patch(
            "autoware_carla_scenario.conditions.composition.entity_lane_position.to_opendrive",
            return_value=fake_od_pose,
        ):
            result = condition.check(world, elapsed=1.0)

        assert result is None

    def test_entity_on_different_lane_returns_none(self) -> None:
        """Condition does not trigger when entity is on a different lane."""
        condition = EntityLanePositionCondition(
            "npc1", road_id="1", lane_id=-1, label="test_lane_pos"
        )
        world = _make_world_with_actor("npc1", x=100.0, y=200.0)

        fake_od_pose = OpenDrivePose(road_id="1", lane_id=-2, s=10.0, t=-3.0)
        with unittest.mock.patch(
            "autoware_carla_scenario.conditions.composition.entity_lane_position.to_opendrive",
            return_value=fake_od_pose,
        ):
            result = condition.check(world, elapsed=1.0)

        assert result is None

    def test_entity_not_found_returns_none(self) -> None:
        """Condition returns None when the entity does not exist in the world."""
        condition = EntityLanePositionCondition(
            "npc1", road_id="1", lane_id=-1, label="test_lane_pos"
        )
        world = _make_world_with_actor("other_actor", x=100.0, y=200.0)

        result = condition.check(world, elapsed=1.0)
        assert result is None

    def test_result_elapsed_seconds(self) -> None:
        """Elapsed time is correctly recorded in the result."""
        condition = EntityLanePositionCondition(
            "npc1", road_id="5", lane_id=1, label="test_lane_pos"
        )
        world = _make_world_with_actor("npc1", x=0.0, y=0.0)

        fake_od_pose = OpenDrivePose(road_id="5", lane_id=1, s=0.0, t=1.0)
        with unittest.mock.patch(
            "autoware_carla_scenario.conditions.composition.entity_lane_position.to_opendrive",
            return_value=fake_od_pose,
        ):
            result = condition.check(world, elapsed=42.5)

        assert result is not None
        assert result.elapsed_seconds == pytest.approx(42.5)

    def test_lane_id_none_matches_any_lane_on_same_road(self) -> None:
        """When lane_id is None, any lane on the matching road triggers pass."""
        condition = EntityLanePositionCondition(
            "npc1", road_id="1", label="test_lane_pos"
        )
        world = _make_world_with_actor("npc1", x=100.0, y=200.0)

        fake_od_pose = OpenDrivePose(road_id="1", lane_id=-3, s=5.0, t=-2.0)
        with unittest.mock.patch(
            "autoware_carla_scenario.conditions.composition.entity_lane_position.to_opendrive",
            return_value=fake_od_pose,
        ):
            result = condition.check(world, elapsed=3.0)

        assert result is not None
        assert result.passed is True
        assert "npc1" in result.message
        assert "'1'" in result.message
        assert "any lane" in result.message
        assert result.elapsed_seconds == pytest.approx(3.0)

    def test_lane_id_none_different_road_returns_none(self) -> None:
        """When lane_id is None, a different road still returns None."""
        condition = EntityLanePositionCondition(
            "npc1", road_id="1", label="test_lane_pos"
        )
        world = _make_world_with_actor("npc1", x=100.0, y=200.0)

        fake_od_pose = OpenDrivePose(road_id="2", lane_id=-1, s=5.0, t=-2.0)
        with unittest.mock.patch(
            "autoware_carla_scenario.conditions.composition.entity_lane_position.to_opendrive",
            return_value=fake_od_pose,
        ):
            result = condition.check(world, elapsed=1.0)

        assert result is None

    # -- ScalarComparisonRule s/t tests --

    def test_s_rule_satisfied(self) -> None:
        """s > 30 with actual s=50 → pass."""
        rules = [ScalarComparisonRule("s", ComparisonRule.GREATER_THAN, 30.0)]
        condition = EntityLanePositionCondition(
            "npc1", road_id="1", lane_id=-1, rules=rules, label="test_lane_pos"
        )
        world = _make_world_with_actor("npc1", x=100.0, y=200.0)

        fake_od_pose = OpenDrivePose(road_id="1", lane_id=-1, s=50.0, t=-1.5)
        with unittest.mock.patch(
            "autoware_carla_scenario.conditions.composition.entity_lane_position.to_opendrive",
            return_value=fake_od_pose,
        ):
            result = condition.check(world, elapsed=5.0)

        assert result is not None
        assert result.passed is True

    def test_s_rule_not_satisfied(self) -> None:
        """s > 30 with actual s=10 → None."""
        rules = [ScalarComparisonRule("s", ComparisonRule.GREATER_THAN, 30.0)]
        condition = EntityLanePositionCondition(
            "npc1", road_id="1", lane_id=-1, rules=rules, label="test_lane_pos"
        )
        world = _make_world_with_actor("npc1", x=100.0, y=200.0)

        fake_od_pose = OpenDrivePose(road_id="1", lane_id=-1, s=10.0, t=-1.5)
        with unittest.mock.patch(
            "autoware_carla_scenario.conditions.composition.entity_lane_position.to_opendrive",
            return_value=fake_od_pose,
        ):
            result = condition.check(world, elapsed=5.0)

        assert result is None

    def test_t_rule_satisfied(self) -> None:
        """t < 0 with actual t=-1.5 → pass."""
        rules = [ScalarComparisonRule("t", ComparisonRule.LESS_THAN, 0.0)]
        condition = EntityLanePositionCondition(
            "npc1", road_id="1", lane_id=-1, rules=rules, label="test_lane_pos"
        )
        world = _make_world_with_actor("npc1", x=100.0, y=200.0)

        fake_od_pose = OpenDrivePose(road_id="1", lane_id=-1, s=10.0, t=-1.5)
        with unittest.mock.patch(
            "autoware_carla_scenario.conditions.composition.entity_lane_position.to_opendrive",
            return_value=fake_od_pose,
        ):
            result = condition.check(world, elapsed=5.0)

        assert result is not None
        assert result.passed is True

    def test_t_rule_not_satisfied(self) -> None:
        """t < 0 with actual t=1.5 → None."""
        rules = [ScalarComparisonRule("t", ComparisonRule.LESS_THAN, 0.0)]
        condition = EntityLanePositionCondition(
            "npc1", road_id="1", lane_id=-1, rules=rules, label="test_lane_pos"
        )
        world = _make_world_with_actor("npc1", x=100.0, y=200.0)

        fake_od_pose = OpenDrivePose(road_id="1", lane_id=-1, s=10.0, t=1.5)
        with unittest.mock.patch(
            "autoware_carla_scenario.conditions.composition.entity_lane_position.to_opendrive",
            return_value=fake_od_pose,
        ):
            result = condition.check(world, elapsed=5.0)

        assert result is None

    def test_multiple_rules_all_satisfied(self) -> None:
        """Both s and t rules pass → pass."""
        rules = [
            ScalarComparisonRule("s", ComparisonRule.GREATER_THAN, 30.0),
            ScalarComparisonRule("t", ComparisonRule.LESS_THAN, 0.0),
        ]
        condition = EntityLanePositionCondition(
            "npc1", road_id="1", lane_id=-1, rules=rules, label="test_lane_pos"
        )
        world = _make_world_with_actor("npc1", x=100.0, y=200.0)

        fake_od_pose = OpenDrivePose(road_id="1", lane_id=-1, s=50.0, t=-1.5)
        with unittest.mock.patch(
            "autoware_carla_scenario.conditions.composition.entity_lane_position.to_opendrive",
            return_value=fake_od_pose,
        ):
            result = condition.check(world, elapsed=5.0)

        assert result is not None
        assert result.passed is True

    def test_multiple_rules_one_fails(self) -> None:
        """s passes but t fails → None."""
        rules = [
            ScalarComparisonRule("s", ComparisonRule.GREATER_THAN, 30.0),
            ScalarComparisonRule("t", ComparisonRule.LESS_THAN, 0.0),
        ]
        condition = EntityLanePositionCondition(
            "npc1", road_id="1", lane_id=-1, rules=rules, label="test_lane_pos"
        )
        world = _make_world_with_actor("npc1", x=100.0, y=200.0)

        fake_od_pose = OpenDrivePose(road_id="1", lane_id=-1, s=50.0, t=1.5)
        with unittest.mock.patch(
            "autoware_carla_scenario.conditions.composition.entity_lane_position.to_opendrive",
            return_value=fake_od_pose,
        ):
            result = condition.check(world, elapsed=5.0)

        assert result is None

    def test_invalid_field_raises(self) -> None:
        """A rule with field='z' raises ValueError."""
        rules = [ScalarComparisonRule("z", ComparisonRule.GREATER_THAN, 0.0)]
        with pytest.raises(ValueError, match="must be 's' or 't'"):
            EntityLanePositionCondition(
                "npc1", road_id="1", rules=rules, label="test_lane_pos"
            )

    def test_lane_id_none_without_rules_does_not_call_project(self) -> None:
        """When lane_id is None and no rules, project_onto_road is not called."""
        condition = EntityLanePositionCondition(
            "npc1", road_id="1", label="test_lane_pos"
        )
        world = _make_world_with_actor("npc1", x=100.0, y=200.0)

        fake_od_pose = OpenDrivePose(road_id="1", lane_id=-1, s=5.0, t=-2.0)
        with unittest.mock.patch(
            "autoware_carla_scenario.conditions.composition.entity_lane_position.to_opendrive",
            return_value=fake_od_pose,
        ), unittest.mock.patch(
            "autoware_carla_scenario.conditions.composition.entity_lane_position.project_onto_road",
        ) as mock_project:
            result = condition.check(world, elapsed=3.0)

        mock_project.assert_not_called()
        assert result is not None
        assert result.passed is True

    def test_lane_id_none_with_rules_uses_project_for_st(self) -> None:
        """When lane_id is None with rules, project_onto_road provides s/t."""
        rules = [ScalarComparisonRule("s", ComparisonRule.GREATER_THAN, 10.0)]
        condition = EntityLanePositionCondition(
            "npc1", road_id="1", rules=rules, label="test_lane_pos"
        )
        world = _make_world_with_actor("npc1", x=100.0, y=200.0)

        # to_opendrive confirms road_id match
        nearest_pose = OpenDrivePose(road_id="1", lane_id=-1, s=48.0, t=-1.0)
        # project_onto_road provides accurate s/t for rule evaluation
        projected_pose = OpenDrivePose(road_id="1", lane_id=-1, s=50.0, t=-2.0)
        with unittest.mock.patch(
            "autoware_carla_scenario.conditions.composition.entity_lane_position.to_opendrive",
            return_value=nearest_pose,
        ), unittest.mock.patch(
            "autoware_carla_scenario.conditions.composition.entity_lane_position.project_onto_road",
            return_value=projected_pose,
        ) as mock_project:
            result = condition.check(world, elapsed=3.0)

        mock_project.assert_called_once()
        assert result is not None
        assert result.passed is True

    def test_rules_none_preserves_existing_behavior(self) -> None:
        """No rules = existing behavior unchanged."""
        condition = EntityLanePositionCondition(
            "npc1", road_id="1", lane_id=-1, label="test_lane_pos"
        )
        world = _make_world_with_actor("npc1", x=100.0, y=200.0)

        fake_od_pose = OpenDrivePose(road_id="1", lane_id=-1, s=10.0, t=-1.5)
        with unittest.mock.patch(
            "autoware_carla_scenario.conditions.composition.entity_lane_position.to_opendrive",
            return_value=fake_od_pose,
        ):
            result = condition.check(world, elapsed=5.0)

        assert result is not None
        assert result.passed is True
        assert "npc1" in result.message
        assert "'1'" in result.message


# ---------------------------------------------------------------------------
# Composite helpers
# ---------------------------------------------------------------------------


class AlwaysFailCondition(BaseCondition):
    """Test helper: always returns a failing result."""

    def __init__(self) -> None:
        super().__init__(label="always_fail")

    def check(self, world: object, elapsed: float) -> Optional[ScenarioResult]:
        return ScenarioResult(
            passed=False, message="Always fails", elapsed_seconds=elapsed
        )


# ---------------------------------------------------------------------------
# AndCondition – unit tests
# ---------------------------------------------------------------------------


class TestAndCondition:
    def test_all_pass_returns_pass(self) -> None:
        cond = AndCondition([AlwaysPassCondition(), AlwaysPassCondition()])
        result = cond.check(MagicMock(), elapsed=1.0)
        assert result is not None
        assert result.passed is True
        assert "AND" in result.message

    def test_one_none_returns_none(self) -> None:
        cond = AndCondition([AlwaysPassCondition(), AlwaysNoneCondition()])
        assert cond.check(MagicMock(), elapsed=1.0) is None

    def test_all_none_returns_none(self) -> None:
        cond = AndCondition([AlwaysNoneCondition(), AlwaysNoneCondition()])
        assert cond.check(MagicMock(), elapsed=1.0) is None

    def test_one_fail_returns_fail_immediately(self) -> None:
        cond = AndCondition([AlwaysFailCondition(), AlwaysPassCondition()])
        result = cond.check(MagicMock(), elapsed=1.0)
        assert result is not None
        assert result.passed is False

    def test_fail_short_circuits(self) -> None:
        """Fail result stops evaluation — second condition is never checked."""
        spy = MagicMock(spec=BaseCondition)
        spy.label = "spy"
        cond = AndCondition([AlwaysFailCondition(), spy])
        cond.check(MagicMock(), elapsed=1.0)
        spy.check.assert_not_called()

    def test_none_short_circuits(self) -> None:
        """None result stops evaluation — second condition is never checked."""
        spy = MagicMock(spec=BaseCondition)
        spy.label = "spy"
        cond = AndCondition([AlwaysNoneCondition(), spy])
        cond.check(MagicMock(), elapsed=1.0)
        spy.check.assert_not_called()

    def test_three_conditions_all_pass(self) -> None:
        cond = AndCondition(
            [AlwaysPassCondition(), AlwaysPassCondition(), AlwaysPassCondition()]
        )
        result = cond.check(MagicMock(), elapsed=2.0)
        assert result is not None
        assert result.passed is True
        assert result.elapsed_seconds == pytest.approx(2.0)

    def test_message_combines_children(self) -> None:
        cond = AndCondition([AlwaysPassCondition(), AlwaysPassCondition()])
        result = cond.check(MagicMock(), elapsed=1.0)
        assert result is not None
        assert result.message.count("Always passes") == 2

    def test_fewer_than_two_raises(self) -> None:
        with pytest.raises(ValueError, match="at least 2"):
            AndCondition([AlwaysPassCondition()])
        with pytest.raises(ValueError, match="at least 2"):
            AndCondition([])


# ---------------------------------------------------------------------------
# OrCondition – unit tests
# ---------------------------------------------------------------------------


class TestOrCondition:
    def test_first_pass_returns_pass(self) -> None:
        cond = OrCondition([AlwaysPassCondition(), AlwaysNoneCondition()])
        result = cond.check(MagicMock(), elapsed=1.0)
        assert result is not None
        assert result.passed is True

    def test_second_pass_returns_pass(self) -> None:
        cond = OrCondition([AlwaysNoneCondition(), AlwaysPassCondition()])
        result = cond.check(MagicMock(), elapsed=1.0)
        assert result is not None
        assert result.passed is True

    def test_all_none_returns_none(self) -> None:
        cond = OrCondition([AlwaysNoneCondition(), AlwaysNoneCondition()])
        assert cond.check(MagicMock(), elapsed=1.0) is None

    def test_fail_result_is_returned(self) -> None:
        cond = OrCondition([AlwaysFailCondition(), AlwaysNoneCondition()])
        result = cond.check(MagicMock(), elapsed=1.0)
        assert result is not None
        assert result.passed is False

    def test_first_match_short_circuits(self) -> None:
        """First non-None result stops evaluation — second is never checked."""
        spy = MagicMock(spec=BaseCondition)
        spy.label = "spy"
        cond = OrCondition([AlwaysPassCondition(), spy])
        cond.check(MagicMock(), elapsed=1.0)
        spy.check.assert_not_called()

    def test_three_conditions(self) -> None:
        cond = OrCondition(
            [AlwaysNoneCondition(), AlwaysNoneCondition(), AlwaysPassCondition()]
        )
        result = cond.check(MagicMock(), elapsed=3.0)
        assert result is not None
        assert result.passed is True

    def test_fewer_than_two_raises(self) -> None:
        with pytest.raises(ValueError, match="at least 2"):
            OrCondition([AlwaysPassCondition()])
        with pytest.raises(ValueError, match="at least 2"):
            OrCondition([])

    def test_nested_and_in_or(self) -> None:
        """OrCondition can contain AndCondition for complex logic."""
        inner_and = AndCondition([AlwaysPassCondition(), AlwaysPassCondition()])
        cond = OrCondition([AlwaysNoneCondition(), inner_and])
        result = cond.check(MagicMock(), elapsed=1.0)
        assert result is not None
        assert result.passed is True


# ---------------------------------------------------------------------------
# StandstillCondition – unit tests (no CARLA required)
# ---------------------------------------------------------------------------


class TestStandstillCondition:
    def test_returns_none_before_duration(self) -> None:
        condition = StandstillCondition("ego", duration=3.0, label="test_standstill")
        world = _make_world_with_actor("ego", 0.0, 0.0, vx=0.0, vy=0.0)
        assert condition.check(world, elapsed=0.0) is None
        assert condition.check(world, elapsed=2.9) is None

    def test_returns_pass_after_duration(self) -> None:
        condition = StandstillCondition("ego", duration=3.0, label="test_standstill")
        world = _make_world_with_actor("ego", 0.0, 0.0, vx=0.0, vy=0.0)
        condition.check(world, elapsed=0.0)
        result = condition.check(world, elapsed=3.0)
        assert result is not None
        assert result.passed is True
        assert "ego" in result.message

    def test_timer_resets_when_moving(self) -> None:
        condition = StandstillCondition("ego", duration=2.0, label="test_standstill")
        world_stop = _make_world_with_actor("ego", 0.0, 0.0, vx=0.0, vy=0.0)
        world_move = _make_world_with_actor("ego", 0.0, 0.0, vx=5.0, vy=0.0)

        # Stand still from t=0 to t=1
        condition.check(world_stop, elapsed=0.0)
        assert condition.check(world_stop, elapsed=1.0) is None

        # Start moving at t=1.5 → timer resets
        assert condition.check(world_move, elapsed=1.5) is None

        # Stand still again from t=2
        condition.check(world_stop, elapsed=2.0)
        assert condition.check(world_stop, elapsed=3.0) is None  # only 1s
        result = condition.check(world_stop, elapsed=4.0)  # 2s standstill
        assert result is not None
        assert result.passed is True

    def test_speed_below_threshold_counts(self) -> None:
        condition = StandstillCondition(
            "ego", duration=1.0, speed_threshold=0.5, label="test_standstill"
        )
        # Speed = sqrt(0.3^2 + 0.3^2) ≈ 0.42 < 0.5
        world = _make_world_with_actor("ego", 0.0, 0.0, vx=0.3, vy=0.3)
        condition.check(world, elapsed=0.0)
        result = condition.check(world, elapsed=1.0)
        assert result is not None
        assert result.passed is True

    def test_speed_above_threshold_no_trigger(self) -> None:
        condition = StandstillCondition(
            "ego", duration=1.0, speed_threshold=0.1, label="test_standstill"
        )
        world = _make_world_with_actor("ego", 0.0, 0.0, vx=1.0, vy=0.0)
        condition.check(world, elapsed=0.0)
        assert condition.check(world, elapsed=5.0) is None

    def test_entity_not_found_returns_none(self) -> None:
        condition = StandstillCondition("ego", duration=1.0, label="test_standstill")
        world = _make_world_with_actor("other", 0.0, 0.0, vx=0.0, vy=0.0)
        assert condition.check(world, elapsed=0.0) is None

    def test_invalid_duration_raises(self) -> None:
        with pytest.raises(ValueError, match="duration must be positive"):
            StandstillCondition("ego", duration=0.0, label="test_standstill")
        with pytest.raises(ValueError, match="duration must be positive"):
            StandstillCondition("ego", duration=-1.0, label="test_standstill")

    def test_invalid_speed_threshold_raises(self) -> None:
        with pytest.raises(ValueError, match="speed_threshold must be non-negative"):
            StandstillCondition(
                "ego", duration=1.0, speed_threshold=-0.1, label="test_standstill"
            )

    def test_elapsed_seconds_in_result(self) -> None:
        condition = StandstillCondition("ego", duration=1.0, label="test_standstill")
        world = _make_world_with_actor("ego", 0.0, 0.0, vx=0.0, vy=0.0)
        condition.check(world, elapsed=10.0)
        result = condition.check(world, elapsed=11.0)
        assert result is not None
        assert result.elapsed_seconds == pytest.approx(11.0)


# ---------------------------------------------------------------------------
# StickyCondition – unit tests
# ---------------------------------------------------------------------------


class TestStickyCondition:
    def test_passes_through_none(self) -> None:
        """When inner returns None, StickyCondition also returns None."""
        cond = StickyCondition(AlwaysNoneCondition())
        assert cond.check(MagicMock(), elapsed=1.0) is None

    def test_passes_through_fail(self) -> None:
        """When inner returns a failure, StickyCondition forwards it without latching."""
        cond = StickyCondition(AlwaysFailCondition())
        result = cond.check(MagicMock(), elapsed=1.0)
        assert result is not None
        assert result.passed is False

    def test_does_not_latch_fail(self) -> None:
        """A failure result is not latched — next check still delegates."""

        class FailThenNone(BaseCondition):
            def __init__(self) -> None:
                super().__init__(label="fail_then_none")
                self._called = False

            def check(self, world: object, elapsed: float) -> Optional[ScenarioResult]:
                if not self._called:
                    self._called = True
                    return ScenarioResult(
                        passed=False, message="fail", elapsed_seconds=elapsed
                    )
                return None

        cond = StickyCondition(FailThenNone())
        r1 = cond.check(MagicMock(), elapsed=1.0)
        assert r1 is not None and r1.passed is False
        # Second call should return None (not the latched failure)
        assert cond.check(MagicMock(), elapsed=2.0) is None

    def test_latches_pass_result(self) -> None:
        """Once inner passes, StickyCondition returns the same result forever."""

        class PassOnce(BaseCondition):
            def __init__(self) -> None:
                super().__init__(label="pass_once")
                self._called = False

            def check(self, world: object, elapsed: float) -> Optional[ScenarioResult]:
                if not self._called:
                    self._called = True
                    return ScenarioResult(
                        passed=True, message="first pass", elapsed_seconds=elapsed
                    )
                return None  # Would return None after first call

        cond = StickyCondition(PassOnce())
        r1 = cond.check(MagicMock(), elapsed=1.0)
        assert r1 is not None and r1.passed is True
        assert r1.message == "first pass"

        # Inner would return None now, but sticky keeps the latched result
        r2 = cond.check(MagicMock(), elapsed=5.0)
        assert r2 is not None and r2.passed is True
        assert r2.message == "first pass"
        assert r2.elapsed_seconds == pytest.approx(1.0)

    def test_inner_not_called_after_latch(self) -> None:
        """Once latched, the inner condition is never called again."""
        spy = MagicMock(spec=BaseCondition)
        spy.label = "spy"
        spy.check.return_value = ScenarioResult(
            passed=True, message="ok", elapsed_seconds=0.0
        )
        cond = StickyCondition(spy)
        cond.check(MagicMock(), elapsed=0.0)  # triggers latch
        spy.check.reset_mock()

        cond.check(MagicMock(), elapsed=1.0)  # should not call inner
        spy.check.assert_not_called()

    def test_with_and_condition(self) -> None:
        """StickyCondition works correctly inside AndCondition for route checks."""
        cond = AndCondition(
            [
                StickyCondition(AlwaysPassCondition()),
                StickyCondition(AlwaysPassCondition()),
            ]
        )
        result = cond.check(MagicMock(), elapsed=1.0)
        assert result is not None
        assert result.passed is True

    def test_and_with_mixed_sticky_timing(self) -> None:
        """AndCondition passes when all sticky children have triggered at least once."""

        class CountedCondition(BaseCondition):
            """Pass on the Nth check, None otherwise."""

            def __init__(self, trigger_at: int) -> None:
                super().__init__(label=f"counted_{trigger_at}")
                self._count = 0
                self._trigger_at = trigger_at

            def check(self, world: object, elapsed: float) -> Optional[ScenarioResult]:
                self._count += 1
                if self._count >= self._trigger_at:
                    return ScenarioResult(
                        passed=True,
                        message=f"triggered at call {self._count}",
                        elapsed_seconds=elapsed,
                    )
                return None

        # First condition triggers immediately, second triggers on 3rd check
        cond = AndCondition(
            [
                StickyCondition(CountedCondition(trigger_at=1)),
                StickyCondition(CountedCondition(trigger_at=3)),
            ]
        )
        world = MagicMock()

        assert cond.check(world, elapsed=1.0) is None  # 2nd not ready
        assert cond.check(world, elapsed=2.0) is None  # 2nd not ready
        result = cond.check(world, elapsed=3.0)  # both now triggered
        assert result is not None
        assert result.passed is True


# ---------------------------------------------------------------------------
# SpeedCondition – unit tests (no CARLA required)
# ---------------------------------------------------------------------------


def _make_world_with_entity_frame(
    target_name: str,
    target_vx: float,
    target_vy: float,
    ref_name: str,
    ref_fwd_x: float,
    ref_fwd_y: float,
) -> MagicMock:
    """Return a mock world with a target actor (velocity) and reference actor (transform)."""
    target_vel = MagicMock()
    target_vel.x, target_vel.y, target_vel.z = target_vx, target_vy, 0.0
    target = MagicMock()
    target.attributes = {"role_name": target_name}
    target.get_velocity.return_value = target_vel

    ref_fwd = MagicMock()
    ref_fwd.x, ref_fwd.y, ref_fwd.z = ref_fwd_x, ref_fwd_y, 0.0
    ref_transform = MagicMock()
    ref_transform.get_forward_vector.return_value = ref_fwd
    ref = MagicMock()
    ref.attributes = {"role_name": ref_name}
    ref.get_transform.return_value = ref_transform

    world = MagicMock()
    world.get_actors.return_value = [target, ref]
    return world


class TestSpeedConditionMagnitude:
    """SpeedCondition with SpeedDirection.MAGNITUDE."""

    def test_speed_above_threshold_greater_than(self) -> None:
        cond = SpeedCondition(
            "ego", value=5.0, rule=ComparisonRule.GREATER_THAN, label="test_speed"
        )
        # speed = sqrt(10^2 + 0^2) = 10.0
        world = _make_world_with_actor("ego", 0.0, 0.0, vx=10.0, vy=0.0)
        result = cond.check(world, elapsed=1.0)
        assert result is not None
        assert result.passed is True

    def test_speed_below_threshold_greater_than(self) -> None:
        cond = SpeedCondition(
            "ego", value=15.0, rule=ComparisonRule.GREATER_THAN, label="test_speed"
        )
        world = _make_world_with_actor("ego", 0.0, 0.0, vx=10.0, vy=0.0)
        assert cond.check(world, elapsed=1.0) is None

    def test_speed_equal_threshold_greater_than(self) -> None:
        cond = SpeedCondition(
            "ego", value=10.0, rule=ComparisonRule.GREATER_THAN, label="test_speed"
        )
        world = _make_world_with_actor("ego", 0.0, 0.0, vx=10.0, vy=0.0)
        assert cond.check(world, elapsed=1.0) is None

    def test_less_than(self) -> None:
        cond = SpeedCondition(
            "ego", value=15.0, rule=ComparisonRule.LESS_THAN, label="test_speed"
        )
        world = _make_world_with_actor("ego", 0.0, 0.0, vx=10.0, vy=0.0)
        result = cond.check(world, elapsed=2.0)
        assert result is not None
        assert result.passed is True

    def test_less_than_not_met(self) -> None:
        cond = SpeedCondition(
            "ego", value=5.0, rule=ComparisonRule.LESS_THAN, label="test_speed"
        )
        world = _make_world_with_actor("ego", 0.0, 0.0, vx=10.0, vy=0.0)
        assert cond.check(world, elapsed=1.0) is None

    def test_equal_to_within_tolerance(self) -> None:
        cond = SpeedCondition(
            "ego",
            value=10.0,
            rule=ComparisonRule.EQUAL_TO,
            tolerance=0.01,
            label="test_speed",
        )
        world = _make_world_with_actor("ego", 0.0, 0.0, vx=10.005, vy=0.0)
        result = cond.check(world, elapsed=1.0)
        assert result is not None
        assert result.passed is True

    def test_equal_to_outside_tolerance(self) -> None:
        cond = SpeedCondition(
            "ego",
            value=10.0,
            rule=ComparisonRule.EQUAL_TO,
            tolerance=0.001,
            label="test_speed",
        )
        world = _make_world_with_actor("ego", 0.0, 0.0, vx=10.1, vy=0.0)
        assert cond.check(world, elapsed=1.0) is None

    def test_greater_than_or_equal_at_boundary(self) -> None:
        cond = SpeedCondition(
            "ego",
            value=10.0,
            rule=ComparisonRule.GREATER_THAN_OR_EQUAL,
            label="test_speed",
        )
        world = _make_world_with_actor("ego", 0.0, 0.0, vx=10.0, vy=0.0)
        result = cond.check(world, elapsed=1.0)
        assert result is not None
        assert result.passed is True

    def test_less_than_or_equal_at_boundary(self) -> None:
        cond = SpeedCondition(
            "ego",
            value=10.0,
            rule=ComparisonRule.LESS_THAN_OR_EQUAL,
            label="test_speed",
        )
        world = _make_world_with_actor("ego", 0.0, 0.0, vx=10.0, vy=0.0)
        result = cond.check(world, elapsed=1.0)
        assert result is not None
        assert result.passed is True

    def test_entity_not_found(self) -> None:
        cond = SpeedCondition(
            "missing", value=0.0, rule=ComparisonRule.GREATER_THAN, label="test_speed"
        )
        world = _make_world_with_actor("ego", 0.0, 0.0, vx=10.0, vy=0.0)
        assert cond.check(world, elapsed=1.0) is None

    def test_message_contains_entity_name(self) -> None:
        cond = SpeedCondition(
            "npc1", value=5.0, rule=ComparisonRule.GREATER_THAN, label="test_speed"
        )
        world = _make_world_with_actor("npc1", 0.0, 0.0, vx=10.0, vy=0.0)
        result = cond.check(world, elapsed=1.0)
        assert result is not None
        assert "npc1" in result.message
        assert "magnitude" in result.message

    def test_elapsed_seconds_recorded(self) -> None:
        cond = SpeedCondition(
            "ego", value=0.0, rule=ComparisonRule.GREATER_THAN, label="test_speed"
        )
        world = _make_world_with_actor("ego", 0.0, 0.0, vx=5.0, vy=0.0)
        result = cond.check(world, elapsed=42.5)
        assert result is not None
        assert result.elapsed_seconds == pytest.approx(42.5)

    def test_3d_magnitude(self) -> None:
        """Magnitude uses all three velocity components."""
        cond = SpeedCondition(
            "ego", value=5.0, rule=ComparisonRule.GREATER_THAN, label="test_speed"
        )
        # speed = sqrt(3^2 + 4^2 + 0^2) = 5.0 — not greater than 5.0
        world = _make_world_with_actor("ego", 0.0, 0.0, vx=3.0, vy=4.0, vz=0.0)
        assert cond.check(world, elapsed=1.0) is None


class TestSpeedConditionWorld:
    """SpeedCondition with SpeedCoordinateSystem.WORLD."""

    def test_longitudinal_returns_x_component(self) -> None:
        cond = SpeedCondition(
            "ego",
            value=5.0,
            rule=ComparisonRule.GREATER_THAN,
            direction=SpeedDirection.LONGITUDINAL,
            coordinate_system=SpeedCoordinateSystem.WORLD,
            label="test_speed",
        )
        world = _make_world_with_actor("ego", 0.0, 0.0, vx=10.0, vy=3.0)
        result = cond.check(world, elapsed=1.0)
        assert result is not None
        assert result.passed is True
        assert "longitudinal" in result.message

    def test_lateral_returns_y_component(self) -> None:
        cond = SpeedCondition(
            "ego",
            value=5.0,
            rule=ComparisonRule.GREATER_THAN,
            direction=SpeedDirection.LATERAL,
            coordinate_system=SpeedCoordinateSystem.WORLD,
            label="test_speed",
        )
        # vy = 8.0 > 5.0
        world = _make_world_with_actor("ego", 0.0, 0.0, vx=1.0, vy=8.0)
        result = cond.check(world, elapsed=1.0)
        assert result is not None
        assert result.passed is True
        assert "lateral" in result.message

    def test_longitudinal_negative_velocity(self) -> None:
        """Negative x-velocity is less than zero."""
        cond = SpeedCondition(
            "ego",
            value=0.0,
            rule=ComparisonRule.LESS_THAN,
            direction=SpeedDirection.LONGITUDINAL,
            coordinate_system=SpeedCoordinateSystem.WORLD,
            label="test_speed",
        )
        world = _make_world_with_actor("ego", 0.0, 0.0, vx=-3.0, vy=0.0)
        result = cond.check(world, elapsed=1.0)
        assert result is not None
        assert result.passed is True


class TestSpeedConditionEntity:
    """SpeedCondition with SpeedCoordinateSystem.ENTITY."""

    def test_entity_frame_requires_reference_name(self) -> None:
        with pytest.raises(ValueError, match="reference_entity_name is required"):
            SpeedCondition(
                "ego",
                value=5.0,
                rule=ComparisonRule.GREATER_THAN,
                direction=SpeedDirection.LONGITUDINAL,
                coordinate_system=SpeedCoordinateSystem.ENTITY,
                label="test_speed",
            )

    def test_reference_entity_not_found(self) -> None:
        cond = SpeedCondition(
            "npc",
            value=0.0,
            rule=ComparisonRule.GREATER_THAN,
            direction=SpeedDirection.LONGITUDINAL,
            coordinate_system=SpeedCoordinateSystem.ENTITY,
            reference_entity_name="missing_ref",
            label="test_speed",
        )
        # World only has "npc", no "missing_ref"
        world = _make_world_with_actor("npc", 0.0, 0.0, vx=10.0, vy=0.0)
        assert cond.check(world, elapsed=1.0) is None

    def test_longitudinal_entity_facing_east_moving_east(self) -> None:
        """Entity facing East, target moving East → longitudinal = vx."""
        cond = SpeedCondition(
            "npc",
            value=5.0,
            rule=ComparisonRule.GREATER_THAN,
            direction=SpeedDirection.LONGITUDINAL,
            coordinate_system=SpeedCoordinateSystem.ENTITY,
            reference_entity_name="ref",
            label="test_speed",
        )
        # ref faces East (fwd=(1,0)), npc moves East at 10 m/s
        world = _make_world_with_entity_frame("npc", 10.0, 0.0, "ref", 1.0, 0.0)
        result = cond.check(world, elapsed=1.0)
        assert result is not None
        assert result.passed is True

    def test_lateral_entity_facing_east_moving_north(self) -> None:
        """Entity facing East, target moving North → lateral positive (left).

        In CARLA: North = -y.  Left of East = North.
        velocity = (0, -5), left_unit = (0, -1) → lateral = 5.
        """
        cond = SpeedCondition(
            "npc",
            value=3.0,
            rule=ComparisonRule.GREATER_THAN,
            direction=SpeedDirection.LATERAL,
            coordinate_system=SpeedCoordinateSystem.ENTITY,
            reference_entity_name="ref",
            label="test_speed",
        )
        # ref faces East (fwd=(1,0)), npc moves North (vy=-5 in CARLA)
        world = _make_world_with_entity_frame("npc", 0.0, -5.0, "ref", 1.0, 0.0)
        result = cond.check(world, elapsed=1.0)
        assert result is not None
        assert result.passed is True

    def test_lateral_entity_facing_east_moving_south(self) -> None:
        """Entity facing East, target moving South → lateral negative (right).

        In CARLA: South = +y.  Right of East = South.
        velocity = (0, 5), left_unit = (0, -1) → lateral = -5.
        """
        cond = SpeedCondition(
            "npc",
            value=0.0,
            rule=ComparisonRule.LESS_THAN,
            direction=SpeedDirection.LATERAL,
            coordinate_system=SpeedCoordinateSystem.ENTITY,
            reference_entity_name="ref",
            label="test_speed",
        )
        world = _make_world_with_entity_frame("npc", 0.0, 5.0, "ref", 1.0, 0.0)
        result = cond.check(world, elapsed=1.0)
        assert result is not None
        assert result.passed is True

    def test_longitudinal_entity_facing_north_moving_east(self) -> None:
        """Ref facing North (fwd=(0,-1)), target moving East (vx=10).

        Longitudinal = dot((10,0), (0,-1)) = 0.
        """
        cond = SpeedCondition(
            "npc",
            value=0.0,
            rule=ComparisonRule.EQUAL_TO,
            direction=SpeedDirection.LONGITUDINAL,
            coordinate_system=SpeedCoordinateSystem.ENTITY,
            reference_entity_name="ref",
            tolerance=0.01,
            label="test_speed",
        )
        world = _make_world_with_entity_frame("npc", 10.0, 0.0, "ref", 0.0, -1.0)
        result = cond.check(world, elapsed=1.0)
        assert result is not None
        assert result.passed is True

    def test_lateral_entity_facing_north_moving_east(self) -> None:
        """Ref facing North (fwd=(0,-1)), target moving East (vx=10).

        Left of North = West = (-1, 0).
        left_unit = (fwd.y, -fwd.x) = (-1, 0).
        Lateral = dot((10,0), (-1,0)) = -10 (moving right).
        """
        cond = SpeedCondition(
            "npc",
            value=-5.0,
            rule=ComparisonRule.LESS_THAN,
            direction=SpeedDirection.LATERAL,
            coordinate_system=SpeedCoordinateSystem.ENTITY,
            reference_entity_name="ref",
            label="test_speed",
        )
        world = _make_world_with_entity_frame("npc", 10.0, 0.0, "ref", 0.0, -1.0)
        result = cond.check(world, elapsed=1.0)
        assert result is not None
        assert result.passed is True

    def test_magnitude_ignores_coordinate_system(self) -> None:
        """MAGNITUDE direction uses scalar speed regardless of coordinate system."""
        cond = SpeedCondition(
            "npc",
            value=5.0,
            rule=ComparisonRule.GREATER_THAN,
            direction=SpeedDirection.MAGNITUDE,
            coordinate_system=SpeedCoordinateSystem.ENTITY,
            reference_entity_name="ref",
            label="test_speed",
        )
        # speed = sqrt(3^2 + 4^2) = 5.0 — not > 5.0
        world = _make_world_with_entity_frame("npc", 3.0, 4.0, "ref", 1.0, 0.0)
        assert cond.check(world, elapsed=1.0) is None


class TestSpeedConditionValidation:
    """Constructor validation for SpeedCondition."""

    def test_negative_tolerance_raises(self) -> None:
        with pytest.raises(ValueError, match="tolerance must be non-negative"):
            SpeedCondition(
                "ego",
                value=5.0,
                rule=ComparisonRule.GREATER_THAN,
                tolerance=-1.0,
                label="test_speed",
            )

    def test_entity_frame_without_reference_raises(self) -> None:
        with pytest.raises(ValueError, match="reference_entity_name is required"):
            SpeedCondition(
                "ego",
                value=5.0,
                rule=ComparisonRule.GREATER_THAN,
                direction=SpeedDirection.LATERAL,
                coordinate_system=SpeedCoordinateSystem.ENTITY,
                label="test_speed",
            )

    def test_world_frame_without_reference_is_valid(self) -> None:
        """WORLD coordinate system does not require reference_entity_name."""
        cond = SpeedCondition(
            "ego",
            value=5.0,
            rule=ComparisonRule.GREATER_THAN,
            direction=SpeedDirection.LONGITUDINAL,
            coordinate_system=SpeedCoordinateSystem.WORLD,
            label="test_speed",
        )
        assert cond is not None
