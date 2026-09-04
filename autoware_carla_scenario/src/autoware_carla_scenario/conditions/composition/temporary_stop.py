"""Temporary stop condition for detecting standstill at specified positions.

When the margin around a stop position extends beyond an OpenDRIVE road
boundary, additional :class:`EntityLanePositionCondition` instances are
created for the predecessor/successor roads and combined via
:class:`OrCondition`.
"""

from __future__ import annotations

import logging
from typing import TYPE_CHECKING, Optional, Sequence, Union

import numpy as np

from ...coordinate.map_manager import MapManager
from ...coordinate.poses import AnyPose, OpenDrivePose
from ...coordinate.transform import to_opendrive
from ..and_condition import AndCondition
from ..base import BaseCondition, ScenarioResult
from ..comparison import ComparisonRule, ScalarComparisonRule
from ...entity_role import EntityRole
from ..or_condition import OrCondition
from ..persistent import PersistentCondition
from .base import CompositionCondition
from .entity_lane_position import EntityLanePositionCondition
from .speed import SpeedCondition

if TYPE_CHECKING:
    import carla

logger = logging.getLogger(__name__)


class TemporaryStopCondition(CompositionCondition):
    """Pass when entity temporarily stops at any of the given positions.

    For each stop position, constructs a composite condition:
    ``PersistentCondition(AndCondition(position_cond, SpeedCondition))``.

    When the margin around a stop position spans across OpenDRIVE road
    boundaries, multiple :class:`EntityLanePositionCondition` instances
    (one per road segment) are wrapped in an :class:`OrCondition`.

    When multiple stop positions are given, they are combined with
    :class:`OrCondition` — stopping at any one position is sufficient.

    An :class:`EntityExistenceCondition` guard ensures the entity is present
    before the inner conditions are evaluated.

    Args:
        entity_name: The ``role_name`` attribute of the actor to track.
        stop_positions: One or more poses where a stop is expected.
            Each pose is converted to :class:`OpenDrivePose` for road/s matching.
        s_margin: Arc-length margin (m) around each stop position.
            The entity must be within ``[s - s_margin, s + s_margin]``.
        speed_threshold: Maximum speed (m/s) considered as stopped.
        stop_duration: Minimum consecutive seconds the entity must remain
            stopped at the position.

    Raises:
        ValueError: If *stop_positions* is empty, *s_margin* is not positive,
            *speed_threshold* is negative, or *stop_duration* is not positive.
    """

    def __init__(
        self,
        entity_name: Union[EntityRole, str],
        stop_positions: Sequence[AnyPose],
        s_margin: float = 5.0,
        speed_threshold: float = 0.1,
        stop_duration: float = 1.0,
        *,
        label: str,
    ) -> None:
        if not stop_positions:
            raise ValueError("stop_positions must not be empty")
        if s_margin <= 0:
            raise ValueError("s_margin must be positive")
        if speed_threshold < 0:
            raise ValueError("speed_threshold must be non-negative")
        if stop_duration <= 0:
            raise ValueError("stop_duration must be positive")

        persistent_conditions: list[PersistentCondition] = []
        for pose in stop_positions:
            od_pose = self._to_od(pose)

            # Build position conditions spanning multiple roads if needed
            position_conds = self._build_position_conditions(
                entity_name, od_pose, s_margin, label=label
            )
            if len(position_conds) == 1:
                position_cond: BaseCondition = position_conds[0]
            else:
                position_cond = OrCondition(position_conds)

            speed_cond = SpeedCondition(
                entity_name=entity_name,
                value=speed_threshold,
                rule=ComparisonRule.LESS_THAN_OR_EQUAL,
                label=f"{label}_speed",
            )
            and_cond = AndCondition([position_cond, speed_cond])
            persistent = PersistentCondition(and_cond, duration=stop_duration)
            persistent_conditions.append(persistent)

        if len(persistent_conditions) == 1:
            child: BaseCondition = persistent_conditions[0]
        else:
            child = OrCondition(persistent_conditions)

        super().__init__(child=child, entity_name=entity_name, label=label)

    # ------------------------------------------------------------------
    # Margin / road-boundary helpers
    # ------------------------------------------------------------------

    @classmethod
    def _build_position_conditions(
        cls,
        entity_name: Union[EntityRole, str],
        od_pose: OpenDrivePose,
        s_margin: float,
        *,
        label: str,
    ) -> list[EntityLanePositionCondition]:
        """Build EntityLanePositionConditions, splitting across roads when needed.

        If the margin range ``[s - s_margin, s + s_margin]`` stays within the
        road, a single condition is returned.  When the range overflows past
        ``s < 0`` (road start) or ``s > road_length`` (road end), additional
        conditions for predecessor / successor roads are appended.
        """
        road_length = cls._get_road_length(od_pose.road_id)
        s_min = od_pose.s - s_margin
        s_max = od_pose.s + s_margin

        conditions: list[EntityLanePositionCondition] = []

        # --- Main road (clamped to valid range) ---
        clamped_min = max(0.0, s_min)
        clamped_max = min(road_length, s_max)
        conditions.append(
            cls._make_lane_condition(
                entity_name, od_pose.road_id, clamped_min, clamped_max, label=label
            )
        )
        logger.info(
            "Position condition: road='%s' s=[%.1f, %.1f] (road_length=%.1f)",
            od_pose.road_id,
            clamped_min,
            clamped_max,
            road_length,
        )

        # --- Overflow before road start (s - margin < 0) ---
        if s_min < 0:
            overflow = abs(s_min)
            for pred_id, contact in cls._find_linked_roads(
                od_pose.road_id, "predecessor"
            ):
                pred_length = cls._get_road_length(pred_id)
                if contact == "end":
                    lo = max(0.0, pred_length - overflow)
                    hi = pred_length
                else:
                    lo = 0.0
                    hi = min(overflow, pred_length)
                conditions.append(
                    cls._make_lane_condition(entity_name, pred_id, lo, hi, label=label)
                )
                logger.info(
                    "  + predecessor road='%s' s=[%.1f, %.1f] (contact=%s, length=%.1f)",
                    pred_id,
                    lo,
                    hi,
                    contact,
                    pred_length,
                )

        # --- Overflow beyond road end (s + margin > road_length) ---
        if s_max > road_length:
            overflow = s_max - road_length
            for succ_id, contact in cls._find_linked_roads(
                od_pose.road_id, "successor"
            ):
                succ_length = cls._get_road_length(succ_id)
                if contact == "start":
                    lo = 0.0
                    hi = min(overflow, succ_length)
                else:
                    lo = max(0.0, succ_length - overflow)
                    hi = succ_length
                conditions.append(
                    cls._make_lane_condition(entity_name, succ_id, lo, hi, label=label)
                )
                logger.info(
                    "  + successor road='%s' s=[%.1f, %.1f] (contact=%s, length=%.1f)",
                    succ_id,
                    lo,
                    hi,
                    contact,
                    succ_length,
                )

        return conditions

    @staticmethod
    def _make_lane_condition(
        entity_name: Union[EntityRole, str],
        road_id: str,
        s_lo: float,
        s_hi: float,
        *,
        label: str,
    ) -> EntityLanePositionCondition:
        """Create an EntityLanePositionCondition for a road segment."""
        return EntityLanePositionCondition(
            entity_name=entity_name,
            road_id=road_id,
            rules=[
                ScalarComparisonRule(
                    field="s",
                    rule=ComparisonRule.GREATER_THAN_OR_EQUAL,
                    value=s_lo,
                ),
                ScalarComparisonRule(
                    field="s",
                    rule=ComparisonRule.LESS_THAN_OR_EQUAL,
                    value=s_hi,
                ),
            ],
            label=label,
        )

    @staticmethod
    def _get_road_length(road_id: str) -> float:
        """Compute the total arc length of an OpenDRIVE road's reference line."""
        mm = MapManager.get_instance()
        road = mm.road_network.road_ids_to_object[road_id]
        ref_line: np.ndarray = road.reference_line
        if len(ref_line) < 2:
            return 0.0
        deltas = np.diff(ref_line, axis=0)
        return float(np.sum(np.linalg.norm(deltas, axis=1)))

    @staticmethod
    def _find_linked_roads(road_id: str, direction: str) -> list[tuple[str, str]]:
        """Find predecessor or successor roads with their contact points.

        Parses the ``<link>`` XML element of the given road to extract
        ``<predecessor>`` or ``<successor>`` entries whose ``elementType``
        is ``"road"`` (junctions are excluded).

        Args:
            road_id: The OpenDRIVE road ID to query.
            direction: ``"predecessor"`` or ``"successor"``.

        Returns:
            List of ``(linked_road_id, contact_point)`` tuples.
            ``contact_point`` is ``"start"`` or ``"end"``.
        """
        mm = MapManager.get_instance()
        road = mm.road_network.road_ids_to_object[road_id]
        results: list[tuple[str, str]] = []
        try:
            link_xml = road.road_xml.find("link")
            if link_xml is None:
                return results
            default_contact = "end" if direction == "predecessor" else "start"
            for elem_xml in link_xml.findall(direction):
                elem_type = elem_xml.attrib.get("elementType", "")
                if elem_type != "road":
                    continue
                elem_id = elem_xml.attrib.get("elementId", "")
                if elem_id not in mm.road_network.road_ids_to_object:
                    continue
                contact = elem_xml.attrib.get("contactPoint", default_contact)
                results.append((elem_id, contact))
        except Exception:
            logger.debug(
                "Failed to find %s roads for road %s",
                direction,
                road_id,
                exc_info=True,
            )
        return results

    @staticmethod
    def _to_od(pose: AnyPose) -> OpenDrivePose:
        """Convert any pose to OpenDrivePose."""
        if isinstance(pose, OpenDrivePose):
            return pose
        return to_opendrive(pose)

    def _check(self, world: "carla.World", elapsed: float) -> Optional[ScenarioResult]:
        """Return a pass result once the child condition fires.

        The entity is guaranteed to exist by the
        :class:`EntityExistenceCondition` guard, and the child
        (``OrCondition`` / ``PersistentCondition``) has already passed
        when this method is called.

        Args:
            world: The CARLA world instance.
            elapsed: Elapsed time in seconds since the scenario started.

        Returns:
            :class:`ScenarioResult` with ``passed=True``.
        """
        return ScenarioResult(
            passed=True,
            message=(
                f"Entity '{self._entity_name}' has temporarily stopped"
                f" at a target position at {elapsed:.2f}s"
            ),
            elapsed_seconds=elapsed,
        )
