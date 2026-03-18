"""Traffic signal action: control traffic lights via Lanelet2 regulatory element IDs."""

from __future__ import annotations

import enum
import logging
from typing import TYPE_CHECKING, Optional, Sequence, Union

from ..conditions import BaseCondition
from ..utils.traffic_light import (
    get_signal_ids_for_controller,
    lanelet2_traffic_light_id_to_opendrive_controller_id,
)
from .base import BaseAction, TickTiming

if TYPE_CHECKING:
    import carla

logger = logging.getLogger(__name__)


class TrafficLightTarget(enum.Enum):
    """Special target modes for traffic light selection."""

    ALL = "all"


class TrafficSignalAction(BaseAction):
    """Control traffic light states using Lanelet2 regulatory element IDs.

    This action follows the :class:`BaseAction` pattern, enabling composable
    and condition-aware traffic light control.  For example, a signal change
    can be delayed using :class:`ElapsedTimeCondition` instead of hand-rolled
    closures with manual time tracking.

    Three target modes are supported:

    - ``None`` (default): NOP -- does nothing (safety default).
    - :attr:`TrafficLightTarget.ALL`: Sets all ``traffic.traffic_light*`` actors.
    - A sequence of Lanelet2 regulatory element IDs: Sets only the lights
      matching those IDs.

    Args:
        state: The desired :class:`carla.TrafficLightState`
            (e.g. ``carla.TrafficLightState.Green``).
        lanelet2_traffic_light_ids: Target specification -- see above.
        condition: Trigger condition (see :class:`BaseCondition`).
        timing: Tick phase (``PRE_TICK`` or ``POST_TICK``).
        label: Human-readable label for logging.
        once: If ``True`` (default) the action fires at most once.
        freeze: If ``True`` (default), freeze every light so that the
            CARLA traffic manager does not override the state.
    """

    def __init__(
        self,
        state: "carla.TrafficLightState",
        lanelet2_traffic_light_ids: Union[
            Sequence[int], TrafficLightTarget, None
        ] = None,
        condition: Optional[BaseCondition] = None,
        timing: TickTiming = TickTiming.PRE_TICK,
        *,
        label: str = "traffic_signal",
        once: bool = True,
        freeze: bool = True,
    ) -> None:
        super().__init__(label=label, condition=condition, timing=timing, once=once)
        self._state = state
        self._target = lanelet2_traffic_light_ids
        self._freeze = freeze

    # ------------------------------------------------------------------
    # BaseAction interface
    # ------------------------------------------------------------------

    def execute(self, world: "carla.World") -> None:
        """Set the traffic light state according to the target specification."""
        if self._target is None:
            logger.debug("TrafficSignalAction: target is None -- NOP")
            return

        if self._target is TrafficLightTarget.ALL:
            self._set_all(world)
        else:
            self._set_by_lanelet2_ids(world, self._target)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _set_all(self, world: "carla.World") -> None:
        """Set every traffic light actor in the world."""
        count = 0
        for actor in world.get_actors().filter("traffic.traffic_light*"):
            actor.set_state(self._state)
            actor.freeze(self._freeze)
            count += 1
        logger.info(
            "TrafficSignalAction [%s]: set %d traffic lights to %s (freeze=%s)",
            self.label,
            count,
            self._state,
            self._freeze,
        )

    def _set_by_lanelet2_ids(
        self,
        world: "carla.World",
        lanelet2_ids: Sequence[int],
    ) -> None:
        """Set traffic lights matching the given Lanelet2 regulatory element IDs."""
        # Collect all OpenDRIVE signal IDs for the requested Lanelet2 IDs
        all_signal_ids: set[str] = set()
        for ll2_id in lanelet2_ids:
            controller_id = lanelet2_traffic_light_id_to_opendrive_controller_id(ll2_id)
            if controller_id is None:
                logger.warning(
                    "TrafficSignalAction [%s]: no OpenDRIVE controller found "
                    "for Lanelet2 traffic light ID %d",
                    self.label,
                    ll2_id,
                )
                continue
            signal_ids = get_signal_ids_for_controller(controller_id)
            all_signal_ids.update(signal_ids)

        if not all_signal_ids:
            logger.warning(
                "TrafficSignalAction [%s]: no matching signals found for "
                "Lanelet2 IDs %s",
                self.label,
                list(lanelet2_ids),
            )
            return

        count = 0
        for actor in world.get_actors().filter("traffic.traffic_light*"):
            if actor.get_opendrive_id() in all_signal_ids:
                actor.set_state(self._state)
                actor.freeze(self._freeze)
                count += 1

        logger.info(
            "TrafficSignalAction [%s]: set %d traffic lights to %s "
            "for Lanelet2 IDs %s (freeze=%s)",
            self.label,
            count,
            self._state,
            list(lanelet2_ids),
            self._freeze,
        )
