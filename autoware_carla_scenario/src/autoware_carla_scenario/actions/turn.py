"""Turn action: analyse OpenDRIVE junctions ahead and set a turn route via TrafficManager."""

from __future__ import annotations

import enum
import logging
from typing import TYPE_CHECKING, List, Optional, Union

from typing import Optional as _Optional

from ..conditions import BaseCondition
from ..conditions.base import find_actor_by_role_name
from ..constants import DEFAULT_TM_PORT
from ..entity_role import EntityRole
from .base import BaseAction, TickTiming

if TYPE_CHECKING:
    import carla

logger = logging.getLogger(__name__)


class TurnDirection(enum.Enum):
    """Direction of a turn at a junction."""

    LEFT = "left"
    RIGHT = "right"


class TurnAction(BaseAction):
    """Analyse OpenDRIVE junctions ahead and set a turn route via TrafficManager.

    When the associated condition is satisfied, this action:

    1. Locates the target vehicle by its ``role_name``
    2. Walks forward along CARLA waypoints (which reflect the underlying
       OpenDRIVE road network) to find the next junction
    3. Enumerates all possible paths through the junction
    4. Selects the path whose heading change best matches *direction*
       (approximately −90° for left, +90° for right in CARLA's yaw convention)
    5. Calls ``TrafficManager.set_path`` to apply the route

    Args:
        entity_name: ``role_name`` of the vehicle actor to control.
        direction: :class:`TurnDirection` — ``LEFT`` or ``RIGHT``.
        condition: Trigger condition (see :class:`BaseCondition`).
        client: A ``carla.Client`` used to obtain the TrafficManager.
        timing: Tick phase (``PRE_TICK`` or ``POST_TICK``).
        once: If ``True`` (default) the action fires at most once.
        search_distance: Maximum distance (m) to look ahead for a junction.
        waypoint_step: Sampling distance (m) between waypoints.
        post_junction_distance: How far (m) past the junction exit to extend
            the route for a stable heading measurement.
    """

    _LEFT_TARGET_DEG: float = -90.0
    _RIGHT_TARGET_DEG: float = 90.0

    def __init__(
        self,
        entity_name: Union[EntityRole, str],
        direction: TurnDirection,
        client: "carla.Client",
        condition: _Optional[BaseCondition] = None,
        timing: TickTiming = TickTiming.PRE_TICK,
        *,
        label: str = "turn_signal",
        once: bool = True,
        search_distance: float = 200.0,
        waypoint_step: float = 2.0,
        post_junction_distance: float = 20.0,
        tm_port: int = DEFAULT_TM_PORT,
    ) -> None:
        super().__init__(label=label, condition=condition, timing=timing, once=once)
        self._entity_name = entity_name
        self._direction = direction
        self._client = client
        self._search_distance = search_distance
        self._waypoint_step = waypoint_step
        self._post_junction_distance = post_junction_distance
        self._tm_port = tm_port

    # ------------------------------------------------------------------
    # BaseAction interface
    # ------------------------------------------------------------------

    def execute(self, world: "carla.World") -> None:
        """Find next junction, compute turn route, and apply via TrafficManager."""
        actor = find_actor_by_role_name(world, self._entity_name)
        if actor is None:
            logger.warning("TurnAction: actor '%s' not found", self._entity_name)
            return

        current_wp = world.get_map().get_waypoint(actor.get_location())
        path = self._compute_turn_route(current_wp)
        if not path:
            logger.warning(
                "TurnAction: no %s turn route found for '%s'",
                self._direction.value,
                self._entity_name,
            )
            return

        tm = self._client.get_trafficmanager(self._tm_port)
        tm.set_path(actor, path)
        logger.info(
            "TurnAction: set %s turn route (%d points) for '%s'",
            self._direction.value,
            len(path),
            self._entity_name,
        )

    # ------------------------------------------------------------------
    # Route computation
    # ------------------------------------------------------------------

    def _compute_turn_route(
        self, current_wp: "carla.Waypoint"
    ) -> List["carla.Location"]:
        """Build a waypoint path through the next junction in the desired direction."""
        pre_junction_wp, junction_entries = self._walk_to_junction(current_wp)
        if pre_junction_wp is None or not junction_entries:
            return []

        branches: List[List["carla.Waypoint"]] = []
        for entry_wp in junction_entries:
            branch = self._trace_through_junction(entry_wp)
            if branch:
                branches.append(branch)

        if not branches:
            return []

        best = self._pick_branch(pre_junction_wp, branches)
        if best is None:
            return []

        return [wp.transform.location for wp in best]

    # ------------------------------------------------------------------
    # Junction discovery
    # ------------------------------------------------------------------

    def _walk_to_junction(
        self, start_wp: "carla.Waypoint"
    ) -> tuple[Optional["carla.Waypoint"], List["carla.Waypoint"]]:
        """Walk forward from *start_wp* until the next OpenDRIVE junction.

        If *start_wp* is already inside a junction it is first skipped so that
        the *next* junction ahead is found.

        Returns:
            ``(pre_junction_waypoint, junction_entry_waypoints)`` where the
            entries are the first waypoints on connecting roads inside the
            junction, or ``(None, [])`` when no junction is found within
            *search_distance*.
        """
        wp = start_wp
        distance = 0.0

        # Skip past current junction (if any)
        while wp.is_junction and distance < self._search_distance:
            nxt = wp.next(self._waypoint_step)
            if not nxt:
                return None, []
            wp = nxt[0]
            distance += self._waypoint_step

        # Walk forward to the next junction boundary
        while distance < self._search_distance:
            next_wps = wp.next(self._waypoint_step)
            if not next_wps:
                return None, []

            entries = [w for w in next_wps if w.is_junction]
            if entries:
                return wp, entries

            wp = next_wps[0]
            distance += self._waypoint_step

        return None, []

    # ------------------------------------------------------------------
    # Branch tracing
    # ------------------------------------------------------------------

    def _trace_through_junction(
        self, entry_wp: "carla.Waypoint"
    ) -> List["carla.Waypoint"]:
        """Follow waypoints from *entry_wp* through the junction and a bit beyond.

        The extra post-junction distance provides a stable exit heading for
        direction comparison.
        """
        path: List["carla.Waypoint"] = [entry_wp]
        wp = entry_wp

        # Walk through junction connecting road
        safety_limit = 500
        while wp.is_junction and safety_limit > 0:
            nxt = wp.next(self._waypoint_step)
            if not nxt:
                break
            wp = nxt[0]
            path.append(wp)
            safety_limit -= 1

        # Continue past junction exit for a reliable heading measurement
        post = 0.0
        while post < self._post_junction_distance:
            nxt = wp.next(self._waypoint_step)
            if not nxt:
                break
            wp = nxt[0]
            path.append(wp)
            post += self._waypoint_step

        return path

    # ------------------------------------------------------------------
    # Direction selection
    # ------------------------------------------------------------------

    def _pick_branch(
        self,
        pre_junction_wp: "carla.Waypoint",
        branches: List[List["carla.Waypoint"]],
    ) -> Optional[List["carla.Waypoint"]]:
        """Select the branch whose exit heading change is closest to the target.

        CARLA yaw convention (left-hand, clockwise-positive when viewed from
        above):

        - Left turn  ≈ −90° heading change
        - Right turn ≈ +90° heading change
        """
        entry_yaw = pre_junction_wp.transform.rotation.yaw
        target = (
            self._LEFT_TARGET_DEG
            if self._direction == TurnDirection.LEFT
            else self._RIGHT_TARGET_DEG
        )

        best: Optional[List["carla.Waypoint"]] = None
        best_score = float("inf")

        for branch in branches:
            if not branch:
                continue
            exit_yaw = branch[-1].transform.rotation.yaw
            diff = (exit_yaw - entry_yaw + 180.0) % 360.0 - 180.0
            score = abs(diff - target)
            logger.debug(
                "TurnAction: branch exit_yaw=%.1f, diff=%.1f, score=%.1f",
                exit_yaw,
                diff,
                score,
            )
            if score < best_score:
                best_score = score
                best = branch

        if best is not None:
            exit_yaw = best[-1].transform.rotation.yaw
            diff = (exit_yaw - entry_yaw + 180.0) % 360.0 - 180.0
            logger.info(
                "TurnAction: selected branch with heading change %.1f deg "
                "(target: %.1f deg %s)",
                diff,
                target,
                self._direction.value,
            )

        return best
