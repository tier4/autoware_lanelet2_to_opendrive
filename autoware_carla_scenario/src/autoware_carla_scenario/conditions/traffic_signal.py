"""Traffic signal condition: verify traffic light state by Lanelet2 regulatory element ID."""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Any, Optional

from ..utils.traffic_light import (
    get_signal_ids_for_controller,
    lanelet2_traffic_light_id_to_opendrive_controller_id,
)
from .base import BaseCondition, ScenarioResult

if TYPE_CHECKING:
    import carla

logger = logging.getLogger(__name__)


class TrafficSignalCondition(BaseCondition):
    """Check whether a traffic light matches an expected state.

    Resolves a Lanelet2 regulatory element ID to OpenDRIVE signal IDs on the
    first ``check()`` call and caches the result.  Subsequent calls reuse the
    cached signal IDs to find matching CARLA traffic light actors and compare
    their current state against the expected state.

    Args:
        lanelet2_regulatory_element_id: Lanelet2 regulatory element ID of
            the traffic light to monitor.
        expected_state: The :class:`carla.TrafficLightState` the traffic
            light is expected to be in (e.g. ``carla.TrafficLightState.Green``).
        label: Human-readable identifier for this condition.
    """

    def __init__(
        self,
        lanelet2_regulatory_element_id: int,
        expected_state: "carla.TrafficLightState",
        *,
        label: str,
    ) -> None:
        super().__init__(label=label)
        self._lanelet2_id = lanelet2_regulatory_element_id
        self._expected_state = expected_state
        self._cached_signal_ids: Optional[set[str]] = None

    # ------------------------------------------------------------------
    # Signal ID resolution (lazy + cached)
    # ------------------------------------------------------------------

    def _resolve_signal_ids(self) -> Optional[set[str]]:
        """Resolve the Lanelet2 ID to a set of OpenDRIVE signal IDs.

        Returns:
            A set of signal ID strings, or ``None`` if resolution fails.
        """
        controller_id = lanelet2_traffic_light_id_to_opendrive_controller_id(
            self._lanelet2_id,
        )
        if controller_id is None:
            logger.warning(
                "TrafficSignalCondition [%s]: no OpenDRIVE controller found "
                "for Lanelet2 regulatory element ID %d",
                self.label,
                self._lanelet2_id,
            )
            return None

        signal_ids = get_signal_ids_for_controller(controller_id)
        if not signal_ids:
            logger.warning(
                "TrafficSignalCondition [%s]: controller %d has no signal IDs",
                self.label,
                controller_id,
            )
            return None

        return set(signal_ids)

    # ------------------------------------------------------------------
    # BaseCondition interface
    # ------------------------------------------------------------------

    def check(
        self,
        world: "carla.World",
        elapsed: float,
    ) -> Optional[ScenarioResult]:
        """Check whether matching traffic lights are in the expected state.

        Returns:
            ``ScenarioResult(passed=True)`` if all matching actors have the
            expected state, ``ScenarioResult(passed=False)`` if any differ or
            no matching actors are found, or ``None`` if signal ID resolution
            fails (will retry next tick).
        """
        # Lazy resolution with caching
        if self._cached_signal_ids is None:
            resolved = self._resolve_signal_ids()
            if resolved is None:
                return None
            self._cached_signal_ids = resolved

        # Single-pass: find matching actors and collect mismatches
        match_count = 0
        mismatches: list[tuple[str, object]] = []
        for actor in world.get_actors().filter("traffic.traffic_light*"):
            if actor.get_opendrive_id() in self._cached_signal_ids:
                match_count += 1
                state = actor.get_state()
                if state != self._expected_state:
                    mismatches.append((actor.get_opendrive_id(), state))

        if match_count == 0:
            return ScenarioResult(
                passed=False,
                message=(
                    f"TrafficSignalCondition [{self.label}]: "
                    f"no matching actors found for signal IDs "
                    f"{sorted(self._cached_signal_ids)}"
                ),
                elapsed_seconds=elapsed,
            )

        if mismatches:
            mismatch_details = ", ".join(
                f"{sig_id}={state}" for sig_id, state in mismatches
            )
            return ScenarioResult(
                passed=False,
                message=(
                    f"TrafficSignalCondition [{self.label}]: "
                    f"state mismatch — expected {self._expected_state}, "
                    f"got {mismatch_details}"
                ),
                elapsed_seconds=elapsed,
            )

        return ScenarioResult(
            passed=True,
            message=(
                f"TrafficSignalCondition [{self.label}]: "
                f"all {match_count} actor(s) in expected state "
                f"{self._expected_state}"
            ),
            elapsed_seconds=elapsed,
        )

    def get_details(self) -> dict[str, Any]:
        """Return structured details about this condition's configuration."""
        return {
            "lanelet2_regulatory_element_id": self._lanelet2_id,
            "expected_state": str(self._expected_state),
            "cached_signal_ids": (
                sorted(self._cached_signal_ids)
                if self._cached_signal_ids is not None
                else None
            ),
        }
