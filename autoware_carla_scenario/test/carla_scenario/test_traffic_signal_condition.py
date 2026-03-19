"""Unit tests for TrafficSignalCondition (no CARLA required)."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List
from unittest.mock import MagicMock, patch

import pytest

from autoware_carla_scenario.conditions.traffic_signal import (
    TrafficSignalCondition,
)


# ---------------------------------------------------------------------------
# Lightweight stubs for CARLA types
# ---------------------------------------------------------------------------


@dataclass
class _FakeLocation:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class _FakeTransform:
    location: _FakeLocation = field(default_factory=_FakeLocation)


class _FakeTrafficLight:
    """Minimal traffic light stub with get_state() support."""

    def __init__(
        self,
        opendrive_id: str = "",
        state: object = None,
    ) -> None:
        self._state = state
        self._opendrive_id = opendrive_id
        self.type_id = "traffic.traffic_light"

    def get_opendrive_id(self) -> str:
        return self._opendrive_id

    def get_state(self) -> object:
        return self._state

    def get_transform(self) -> _FakeTransform:
        return _FakeTransform()


class _FakeActorList:
    """Stub for CARLA ActorList with filter support."""

    def __init__(self, actors: List[_FakeTrafficLight]) -> None:
        self._actors = actors

    def filter(self, pattern: str) -> List[_FakeTrafficLight]:
        base = pattern.rstrip("*")
        return [a for a in self._actors if a.type_id.startswith(base)]

    def __iter__(self):
        return iter(self._actors)

    def __len__(self):
        return len(self._actors)


class _FakeWorld:
    """Minimal CARLA world stub."""

    def __init__(self, actors: List[_FakeTrafficLight]) -> None:
        self._actors = actors

    def get_actors(self) -> _FakeActorList:
        return _FakeActorList(self._actors)


# ---------------------------------------------------------------------------
# Patch targets
# ---------------------------------------------------------------------------

_PATCH_CONTROLLER_ID = (
    "autoware_carla_scenario.conditions.traffic_signal"
    ".lanelet2_traffic_light_id_to_opendrive_controller_id"
)
_PATCH_SIGNAL_IDS = (
    "autoware_carla_scenario.conditions.traffic_signal" ".get_signal_ids_for_controller"
)


# ---------------------------------------------------------------------------
# Tests: state match / mismatch
# ---------------------------------------------------------------------------


class TestTrafficSignalConditionStateMatch:
    """Tests for basic state matching."""

    def test_state_match_returns_passed(self) -> None:
        expected = MagicMock(name="Green")
        tl = _FakeTrafficLight(opendrive_id="sig_1", state=expected)
        world = _FakeWorld([tl])

        with (
            patch(_PATCH_CONTROLLER_ID, return_value=100),
            patch(_PATCH_SIGNAL_IDS, return_value=["sig_1"]),
        ):
            cond = TrafficSignalCondition(
                lanelet2_regulatory_element_id=242,
                expected_state=expected,
                label="green_check",
            )
            result = cond.check(world, elapsed=1.0)

        assert result is not None
        assert result.passed is True

    def test_state_mismatch_returns_failed(self) -> None:
        expected = MagicMock(name="Green")
        actual = MagicMock(name="Red")
        tl = _FakeTrafficLight(opendrive_id="sig_1", state=actual)
        world = _FakeWorld([tl])

        with (
            patch(_PATCH_CONTROLLER_ID, return_value=100),
            patch(_PATCH_SIGNAL_IDS, return_value=["sig_1"]),
        ):
            cond = TrafficSignalCondition(
                lanelet2_regulatory_element_id=242,
                expected_state=expected,
                label="mismatch_check",
            )
            result = cond.check(world, elapsed=1.0)

        assert result is not None
        assert result.passed is False


# ---------------------------------------------------------------------------
# Tests: no matching actors
# ---------------------------------------------------------------------------


class TestTrafficSignalConditionNoActors:
    """Tests for missing actor scenarios."""

    def test_no_matching_actors_returns_failed(self) -> None:
        expected = MagicMock(name="Green")
        tl = _FakeTrafficLight(opendrive_id="sig_other", state=expected)
        world = _FakeWorld([tl])

        with (
            patch(_PATCH_CONTROLLER_ID, return_value=100),
            patch(_PATCH_SIGNAL_IDS, return_value=["sig_1"]),
        ):
            cond = TrafficSignalCondition(
                lanelet2_regulatory_element_id=242,
                expected_state=expected,
                label="no_actors_check",
            )
            result = cond.check(world, elapsed=1.0)

        assert result is not None
        assert result.passed is False


# ---------------------------------------------------------------------------
# Tests: controller ID resolution failure
# ---------------------------------------------------------------------------


class TestTrafficSignalConditionResolutionFailure:
    """Tests for signal ID resolution failures."""

    def test_controller_not_found_returns_none(self) -> None:
        """Resolution failure returns None (retry on next tick)."""
        expected = MagicMock(name="Green")
        world = _FakeWorld([])

        with (
            patch(_PATCH_CONTROLLER_ID, return_value=None),
        ):
            cond = TrafficSignalCondition(
                lanelet2_regulatory_element_id=999,
                expected_state=expected,
                label="resolution_fail",
            )
            result = cond.check(world, elapsed=1.0)

        assert result is None

    def test_empty_signal_ids_returns_none(self) -> None:
        """Controller found but no signal IDs returns None."""
        expected = MagicMock(name="Green")
        world = _FakeWorld([])

        with (
            patch(_PATCH_CONTROLLER_ID, return_value=100),
            patch(_PATCH_SIGNAL_IDS, return_value=[]),
        ):
            cond = TrafficSignalCondition(
                lanelet2_regulatory_element_id=242,
                expected_state=expected,
                label="empty_signals",
            )
            result = cond.check(world, elapsed=1.0)

        assert result is None


# ---------------------------------------------------------------------------
# Tests: multiple actors
# ---------------------------------------------------------------------------


class TestTrafficSignalConditionMultipleActors:
    """Tests for multiple traffic light actor scenarios."""

    def test_all_actors_match_returns_passed(self) -> None:
        expected = MagicMock(name="Green")
        tl1 = _FakeTrafficLight(opendrive_id="sig_1", state=expected)
        tl2 = _FakeTrafficLight(opendrive_id="sig_2", state=expected)
        world = _FakeWorld([tl1, tl2])

        with (
            patch(_PATCH_CONTROLLER_ID, return_value=100),
            patch(_PATCH_SIGNAL_IDS, return_value=["sig_1", "sig_2"]),
        ):
            cond = TrafficSignalCondition(
                lanelet2_regulatory_element_id=242,
                expected_state=expected,
                label="all_match",
            )
            result = cond.check(world, elapsed=1.0)

        assert result is not None
        assert result.passed is True

    def test_partial_mismatch_returns_failed(self) -> None:
        expected = MagicMock(name="Green")
        actual_wrong = MagicMock(name="Red")
        tl1 = _FakeTrafficLight(opendrive_id="sig_1", state=expected)
        tl2 = _FakeTrafficLight(opendrive_id="sig_2", state=actual_wrong)
        world = _FakeWorld([tl1, tl2])

        with (
            patch(_PATCH_CONTROLLER_ID, return_value=100),
            patch(_PATCH_SIGNAL_IDS, return_value=["sig_1", "sig_2"]),
        ):
            cond = TrafficSignalCondition(
                lanelet2_regulatory_element_id=242,
                expected_state=expected,
                label="partial_mismatch",
            )
            result = cond.check(world, elapsed=1.0)

        assert result is not None
        assert result.passed is False


# ---------------------------------------------------------------------------
# Tests: signal ID caching
# ---------------------------------------------------------------------------


class TestTrafficSignalConditionCaching:
    """Tests for signal ID resolution caching behaviour."""

    def test_signal_ids_are_cached_after_first_resolution(self) -> None:
        expected = MagicMock(name="Green")
        tl = _FakeTrafficLight(opendrive_id="sig_1", state=expected)
        world = _FakeWorld([tl])

        mock_ctrl = MagicMock(return_value=100)
        mock_sig = MagicMock(return_value=["sig_1"])

        with (
            patch(_PATCH_CONTROLLER_ID, mock_ctrl),
            patch(_PATCH_SIGNAL_IDS, mock_sig),
        ):
            cond = TrafficSignalCondition(
                lanelet2_regulatory_element_id=242,
                expected_state=expected,
                label="cache_test",
            )
            # First call resolves
            cond.check(world, elapsed=1.0)
            # Second call should use cache
            cond.check(world, elapsed=2.0)

        mock_ctrl.assert_called_once_with(242)
        mock_sig.assert_called_once_with(100)

    def test_failed_resolution_is_not_cached(self) -> None:
        """Resolution failure should not be cached — allow retry."""
        expected = MagicMock(name="Green")
        tl = _FakeTrafficLight(opendrive_id="sig_1", state=expected)
        world = _FakeWorld([tl])

        mock_ctrl = MagicMock(side_effect=[None, 100])
        mock_sig = MagicMock(return_value=["sig_1"])

        with (
            patch(_PATCH_CONTROLLER_ID, mock_ctrl),
            patch(_PATCH_SIGNAL_IDS, mock_sig),
        ):
            cond = TrafficSignalCondition(
                lanelet2_regulatory_element_id=242,
                expected_state=expected,
                label="retry_test",
            )
            # First call fails resolution
            result1 = cond.check(world, elapsed=1.0)
            assert result1 is None

            # Second call succeeds
            result2 = cond.check(world, elapsed=2.0)
            assert result2 is not None
            assert result2.passed is True

        assert mock_ctrl.call_count == 2


# ---------------------------------------------------------------------------
# Tests: get_details()
# ---------------------------------------------------------------------------


class TestTrafficSignalConditionGetDetails:
    """Tests for structured detail reporting."""

    def test_get_details_before_resolution(self) -> None:
        expected = MagicMock(name="Green")
        cond = TrafficSignalCondition(
            lanelet2_regulatory_element_id=242,
            expected_state=expected,
            label="details_test",
        )
        details = cond.get_details()

        assert details["lanelet2_regulatory_element_id"] == 242
        assert details["expected_state"] == str(expected)
        assert details["cached_signal_ids"] is None

    def test_get_details_after_resolution(self) -> None:
        expected = MagicMock(name="Green")
        tl = _FakeTrafficLight(opendrive_id="sig_1", state=expected)
        world = _FakeWorld([tl])

        with (
            patch(_PATCH_CONTROLLER_ID, return_value=100),
            patch(_PATCH_SIGNAL_IDS, return_value=["sig_1"]),
        ):
            cond = TrafficSignalCondition(
                lanelet2_regulatory_element_id=242,
                expected_state=expected,
                label="details_resolved",
            )
            cond.check(world, elapsed=1.0)

        details = cond.get_details()
        assert details["cached_signal_ids"] == ["sig_1"]


# ---------------------------------------------------------------------------
# Tests: empty label raises ValueError
# ---------------------------------------------------------------------------


class TestTrafficSignalConditionValidation:
    """Tests for constructor validation."""

    def test_empty_label_raises_value_error(self) -> None:
        with pytest.raises(ValueError, match="label must not be empty"):
            TrafficSignalCondition(
                lanelet2_regulatory_element_id=242,
                expected_state=MagicMock(),
                label="",
            )
