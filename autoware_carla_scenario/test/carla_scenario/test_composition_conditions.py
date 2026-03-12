"""Unit tests for composition conditions (PersistentCondition, StandstillCondition, TemporaryStopCondition)."""

from __future__ import annotations

from typing import Optional
from unittest.mock import MagicMock, patch

import pytest

from autoware_carla_scenario import (
    AndCondition,
    BaseCondition,
    OrCondition,
    PersistentCondition,
    ScenarioResult,
    StandstillCondition,
    TemporaryStopCondition,
)
from autoware_carla_scenario.coordinate.poses import (
    CarlaWorldPose,
    Lanelet2Pose,
    OpenDrivePose,
)


# ---------------------------------------------------------------------------
# Test helper conditions
# ---------------------------------------------------------------------------


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


class AlwaysFailCondition(BaseCondition):
    """Test helper: always returns a failing result."""

    def check(self, world: object, elapsed: float) -> Optional[ScenarioResult]:
        return ScenarioResult(
            passed=False, message="Always fails", elapsed_seconds=elapsed
        )


class ToggleCondition(BaseCondition):
    """Test helper: alternates between pass and None based on a sequence."""

    def __init__(self, results: list[Optional[bool]]) -> None:
        self._results = results
        self._index = 0

    def check(self, world: object, elapsed: float) -> Optional[ScenarioResult]:
        if self._index >= len(self._results):
            return None
        val = self._results[self._index]
        self._index += 1
        if val is None:
            return None
        return ScenarioResult(
            passed=val, message=f"Toggle: {val}", elapsed_seconds=elapsed
        )


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
    """Return a MagicMock CARLA world that contains a single actor."""
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


# ---------------------------------------------------------------------------
# PersistentCondition – unit tests
# ---------------------------------------------------------------------------


class TestPersistentCondition:
    def test_returns_none_before_duration(self) -> None:
        cond = PersistentCondition(AlwaysPassCondition(), duration=3.0)
        world = MagicMock()
        assert cond.check(world, elapsed=0.0) is None
        assert cond.check(world, elapsed=2.9) is None

    def test_returns_pass_at_duration(self) -> None:
        cond = PersistentCondition(AlwaysPassCondition(), duration=3.0)
        world = MagicMock()
        cond.check(world, elapsed=0.0)
        result = cond.check(world, elapsed=3.0)
        assert result is not None
        assert result.passed is True
        assert result.elapsed_seconds == pytest.approx(3.0)

    def test_returns_pass_beyond_duration(self) -> None:
        cond = PersistentCondition(AlwaysPassCondition(), duration=2.0)
        world = MagicMock()
        cond.check(world, elapsed=0.0)
        result = cond.check(world, elapsed=5.0)
        assert result is not None
        assert result.passed is True

    def test_timer_resets_on_none(self) -> None:
        """Timer resets when child returns None."""
        # Pass, Pass, None, Pass, Pass → needs restart
        toggle = ToggleCondition([True, True, None, True, True])
        cond = PersistentCondition(toggle, duration=1.5)
        world = MagicMock()

        cond.check(world, elapsed=0.0)  # True → start timer
        assert cond.check(world, elapsed=1.0) is None  # True, 1.0s < 1.5s
        cond.check(world, elapsed=1.2)  # None → reset
        cond.check(world, elapsed=2.0)  # True → restart timer
        assert cond.check(world, elapsed=3.0) is None  # True, but only 1.0s

    def test_timer_resets_on_fail(self) -> None:
        """Timer resets when child returns passed=False."""
        toggle = ToggleCondition([True, False, True, True])
        cond = PersistentCondition(toggle, duration=1.0)
        world = MagicMock()

        cond.check(world, elapsed=0.0)  # True → start
        cond.check(world, elapsed=0.5)  # False → reset
        cond.check(world, elapsed=1.0)  # True → restart
        assert cond.check(world, elapsed=1.5) is None  # True, only 0.5s since restart

    def test_invalid_duration_raises(self) -> None:
        with pytest.raises(ValueError, match="duration must be positive"):
            PersistentCondition(AlwaysPassCondition(), duration=0.0)
        with pytest.raises(ValueError, match="duration must be positive"):
            PersistentCondition(AlwaysPassCondition(), duration=-1.0)

    def test_child_always_none(self) -> None:
        """Always returns None when child never triggers."""
        cond = PersistentCondition(AlwaysNoneCondition(), duration=1.0)
        world = MagicMock()
        assert cond.check(world, elapsed=0.0) is None
        assert cond.check(world, elapsed=100.0) is None

    def test_child_always_fails(self) -> None:
        """Always returns None when child always fails."""
        cond = PersistentCondition(AlwaysFailCondition(), duration=1.0)
        world = MagicMock()
        assert cond.check(world, elapsed=0.0) is None
        assert cond.check(world, elapsed=100.0) is None

    def test_message_contains_duration_info(self) -> None:
        cond = PersistentCondition(AlwaysPassCondition(), duration=2.0)
        world = MagicMock()
        cond.check(world, elapsed=0.0)
        result = cond.check(world, elapsed=2.0)
        assert result is not None
        assert "2.00s" in result.message

    def test_wraps_and_condition(self) -> None:
        """PersistentCondition can wrap a composite condition."""
        inner = AndCondition([AlwaysPassCondition(), AlwaysPassCondition()])
        cond = PersistentCondition(inner, duration=1.0)
        world = MagicMock()
        cond.check(world, elapsed=0.0)
        result = cond.check(world, elapsed=1.0)
        assert result is not None
        assert result.passed is True


# ---------------------------------------------------------------------------
# StandstillCondition (composition version) – unit tests
# ---------------------------------------------------------------------------


class TestStandstillConditionComposition:
    """Tests for the composition-based StandstillCondition.

    These match the original StandstillCondition test cases to ensure
    API compatibility.
    """

    def test_returns_none_before_duration(self) -> None:
        condition = StandstillCondition("ego", duration=3.0)
        world = _make_world_with_actor("ego", 0.0, 0.0, vx=0.0, vy=0.0)
        assert condition.check(world, elapsed=0.0) is None
        assert condition.check(world, elapsed=2.9) is None

    def test_returns_pass_after_duration(self) -> None:
        condition = StandstillCondition("ego", duration=3.0)
        world = _make_world_with_actor("ego", 0.0, 0.0, vx=0.0, vy=0.0)
        condition.check(world, elapsed=0.0)
        result = condition.check(world, elapsed=3.0)
        assert result is not None
        assert result.passed is True

    def test_timer_resets_when_moving(self) -> None:
        condition = StandstillCondition("ego", duration=2.0)
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
        condition = StandstillCondition("ego", duration=1.0, speed_threshold=0.5)
        # Speed = sqrt(0.3^2 + 0.3^2) ~ 0.42 < 0.5
        world = _make_world_with_actor("ego", 0.0, 0.0, vx=0.3, vy=0.3)
        condition.check(world, elapsed=0.0)
        result = condition.check(world, elapsed=1.0)
        assert result is not None
        assert result.passed is True

    def test_speed_above_threshold_no_trigger(self) -> None:
        condition = StandstillCondition("ego", duration=1.0, speed_threshold=0.1)
        world = _make_world_with_actor("ego", 0.0, 0.0, vx=1.0, vy=0.0)
        condition.check(world, elapsed=0.0)
        assert condition.check(world, elapsed=5.0) is None

    def test_entity_not_found_returns_none(self) -> None:
        condition = StandstillCondition("ego", duration=1.0)
        world = _make_world_with_actor("other", 0.0, 0.0, vx=0.0, vy=0.0)
        assert condition.check(world, elapsed=0.0) is None

    def test_invalid_duration_raises(self) -> None:
        with pytest.raises(ValueError, match="duration must be positive"):
            StandstillCondition("ego", duration=0.0)
        with pytest.raises(ValueError, match="duration must be positive"):
            StandstillCondition("ego", duration=-1.0)

    def test_invalid_speed_threshold_raises(self) -> None:
        with pytest.raises(ValueError, match="speed_threshold must be non-negative"):
            StandstillCondition("ego", duration=1.0, speed_threshold=-0.1)

    def test_elapsed_seconds_in_result(self) -> None:
        condition = StandstillCondition("ego", duration=1.0)
        world = _make_world_with_actor("ego", 0.0, 0.0, vx=0.0, vy=0.0)
        condition.check(world, elapsed=10.0)
        result = condition.check(world, elapsed=11.0)
        assert result is not None
        assert result.elapsed_seconds == pytest.approx(11.0)


# ---------------------------------------------------------------------------
# TemporaryStopCondition – unit tests
# ---------------------------------------------------------------------------


class TestTemporaryStopCondition:
    """Tests for TemporaryStopCondition.

    Uses mocked coordinate transforms and entity lookups.
    """

    def test_validation_empty_positions(self) -> None:
        with pytest.raises(ValueError, match="stop_positions must not be empty"):
            TemporaryStopCondition("ego", stop_positions=[])

    def test_validation_s_margin(self) -> None:
        od = OpenDrivePose(road_id="1", lane_id=-1, s=50.0)
        with pytest.raises(ValueError, match="s_margin must be positive"):
            TemporaryStopCondition("ego", stop_positions=[od], s_margin=0.0)
        with pytest.raises(ValueError, match="s_margin must be positive"):
            TemporaryStopCondition("ego", stop_positions=[od], s_margin=-1.0)

    def test_validation_speed_threshold(self) -> None:
        od = OpenDrivePose(road_id="1", lane_id=-1, s=50.0)
        with pytest.raises(ValueError, match="speed_threshold must be non-negative"):
            TemporaryStopCondition("ego", stop_positions=[od], speed_threshold=-0.1)

    def test_validation_stop_duration(self) -> None:
        od = OpenDrivePose(road_id="1", lane_id=-1, s=50.0)
        with pytest.raises(ValueError, match="stop_duration must be positive"):
            TemporaryStopCondition("ego", stop_positions=[od], stop_duration=0.0)

    def test_single_opendrive_pose(self) -> None:
        """Single OpenDrivePose creates a PersistentCondition (not OrCondition)."""
        od = OpenDrivePose(road_id="1", lane_id=-1, s=50.0)
        cond = TemporaryStopCondition("ego", stop_positions=[od])
        assert isinstance(cond._child, PersistentCondition)

    def test_multiple_opendrive_poses(self) -> None:
        """Multiple poses creates an OrCondition wrapping PersistentConditions."""
        od1 = OpenDrivePose(road_id="1", lane_id=-1, s=50.0)
        od2 = OpenDrivePose(road_id="2", lane_id=-1, s=100.0)
        cond = TemporaryStopCondition("ego", stop_positions=[od1, od2])
        assert isinstance(cond._child, OrCondition)

    @patch("autoware_carla_scenario.conditions.composition.temporary_stop.to_opendrive")
    def test_lanelet2_pose_converted(self, mock_to_od: MagicMock) -> None:
        """Lanelet2Pose is converted via to_opendrive()."""
        mock_to_od.return_value = OpenDrivePose(road_id="5", lane_id=-1, s=30.0)
        ll2 = Lanelet2Pose(lanelet_id=100, s=10.0, t=0.0)
        cond = TemporaryStopCondition("ego", stop_positions=[ll2])
        mock_to_od.assert_called_once_with(ll2)
        assert isinstance(cond._child, PersistentCondition)

    @patch("autoware_carla_scenario.conditions.composition.temporary_stop.to_opendrive")
    def test_carla_world_pose_converted(self, mock_to_od: MagicMock) -> None:
        """CarlaWorldPose is converted via to_opendrive()."""
        mock_to_od.return_value = OpenDrivePose(road_id="7", lane_id=-1, s=40.0)
        cwp = CarlaWorldPose(x=10.0, y=20.0, z=0.0)
        cond = TemporaryStopCondition("ego", stop_positions=[cwp])
        mock_to_od.assert_called_once_with(cwp)
        assert isinstance(cond._child, PersistentCondition)

    def test_opendrive_pose_not_converted(self) -> None:
        """OpenDrivePose is used directly without calling to_opendrive."""
        od = OpenDrivePose(road_id="1", lane_id=-1, s=50.0)
        with patch(
            "autoware_carla_scenario.conditions.composition.temporary_stop.to_opendrive"
        ) as mock_to_od:
            TemporaryStopCondition("ego", stop_positions=[od])
            mock_to_od.assert_not_called()
