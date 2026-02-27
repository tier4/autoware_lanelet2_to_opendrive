"""Validation utilities for OpenDRIVE data consistency.

This module provides validation functions to ensure OpenDRIVE files comply with the
specification and prevent issues in downstream tools like CARLA.
"""

from collections import Counter
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, List

if TYPE_CHECKING:
    pass

from .road import Road


@dataclass
class LaneConnectionError:
    """Represents an invalid lane connection error."""

    road_id: int
    lane_id: int
    connection_type: str  # "predecessor" or "successor"
    message: str


@dataclass
class ValidationResult:
    """Result of OpenDRIVE validation."""

    is_valid: bool
    errors: List[LaneConnectionError]

    @property
    def error_count(self) -> int:
        """Return the number of validation errors."""
        return len(self.errors)

    def get_error_summary(self) -> str:
        """Return a human-readable summary of validation errors."""
        if self.is_valid:
            return "No validation errors found."

        summary_lines = [
            f"Found {self.error_count} validation errors:",
            "",
        ]

        for error in self.errors:
            summary_lines.append(
                f"  Road {error.road_id} Lane {error.lane_id} "
                f"({error.connection_type}): {error.message}"
            )

        return "\n".join(summary_lines)


def validate_lane_road_link_consistency(
    roads: List[Road],
) -> ValidationResult:
    """Validate consistency between lane-level and road-level connections.

    This function checks that lane predecessor/successor links only exist when the
    corresponding road-level predecessor/successor exists. This prevents invalid
    OpenDRIVE files that cause crashes in tools like CARLA.

    Issue #202: Invalid lane connections without corresponding road connections
    cause segmentation faults in OpenDRIVE parsers.

    Args:
        roads: List of Road objects to validate

    Returns:
        ValidationResult containing validation status and any errors found

    Example:
        >>> from autoware_lanelet2_to_opendrive.opendrive.opendrive import OpenDRIVE
        >>> from autoware_lanelet2_to_opendrive.opendrive.validation import (
        ...     validate_lane_road_link_consistency
        ... )
        >>> opendrive = OpenDRIVE(roads=[...])
        >>> result = validate_lane_road_link_consistency(opendrive.roads)
        >>> if not result.is_valid:
        ...     print(result.get_error_summary())
    """
    errors: List[LaneConnectionError] = []

    for road in roads:
        # Check if road has link connections
        has_road_predecessor = (
            road.link is not None and road.link.predecessor is not None
        )
        has_road_successor = road.link is not None and road.link.successor is not None

        # Check lanes in all lane sections
        if road.lanes is None:
            continue

        for lane_section in road.lanes.lane_sections:
            # Check all lanes (left, center, right)
            all_lanes: List[Any] = []

            if lane_section.left_lanes:
                all_lanes.extend(lane_section.left_lanes.values())

            # Center lane is a single ReferenceLine, not a dict
            # Typically center lane (ID=0) doesn't have predecessor/successor
            # so we skip it for validation

            if lane_section.right_lanes:
                all_lanes.extend(lane_section.right_lanes.values())

            for lane in all_lanes:
                # Check lane predecessor
                if lane.predecessor is not None:
                    if not has_road_predecessor:
                        errors.append(
                            LaneConnectionError(
                                road_id=road.id,
                                lane_id=lane.lane_id,
                                connection_type="predecessor",
                                message=(
                                    "Lane has predecessor but road does not have "
                                    "predecessor link"
                                ),
                            )
                        )

                # Check lane successor
                if lane.successor is not None:
                    if not has_road_successor:
                        errors.append(
                            LaneConnectionError(
                                road_id=road.id,
                                lane_id=lane.lane_id,
                                connection_type="successor",
                                message=(
                                    "Lane has successor but road does not have "
                                    "successor link"
                                ),
                            )
                        )

    return ValidationResult(is_valid=(len(errors) == 0), errors=errors)


def validate_no_duplicate_road_ids(roads: List[Road]) -> ValidationResult:
    """Validate that no two roads share the same ID.

    Duplicate road IDs can cause silent data corruption when building lookup
    dictionaries (last writer wins) and lead to invalid lane links or dangling
    road references in the output OpenDRIVE file.

    Args:
        roads: List of Road objects to validate

    Returns:
        ValidationResult containing validation status and any errors found

    Example:
        >>> result = validate_no_duplicate_road_ids(all_roads)
        >>> if not result.is_valid:
        ...     print(result.get_error_summary())
    """
    errors: List[LaneConnectionError] = []

    id_counts = Counter(road.id for road in roads)
    for road_id, count in id_counts.items():
        if count > 1:
            errors.append(
                LaneConnectionError(
                    road_id=road_id,
                    lane_id=0,
                    connection_type="duplicate",
                    message=f"Road ID {road_id} appears {count} times in the road list",
                )
            )

    return ValidationResult(is_valid=(len(errors) == 0), errors=errors)
