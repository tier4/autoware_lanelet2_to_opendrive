"""Unit tests for scenario pass/fail conditions."""

from __future__ import annotations

from typing import Optional
from unittest.mock import MagicMock

import pytest

from autoware_carla_scenario import (
    BaseCondition,
    ScenarioResult,
    TimeoutCondition,
)


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
