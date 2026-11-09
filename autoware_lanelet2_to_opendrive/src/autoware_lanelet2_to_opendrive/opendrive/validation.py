"""Validation utilities for OpenDRIVE data consistency.

This module provides validation functions to ensure OpenDRIVE files comply with the
specification and prevent issues in downstream tools like CARLA.
"""

from collections import Counter
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, List, Optional

if TYPE_CHECKING:
    pass

from ..config import DEFAULT_CONFIG

# ``_lane_width_samples``/``_width_stats`` are the width-sampling primitives
# committed for ``xodr_digest.py`` (see that module's docstring for why a
# per-segment, own-start/mid/end sample is the correct way to evaluate a
# ``<width>`` cubic polynomial run). They are reused here unchanged rather
# than reimplemented, per the instruction not to duplicate the sampling
# logic: both ``xodr_digest.py`` (post-hoc, reading a written ``.xodr``
# file) and this module (pre-write, reading in-memory ``Lane`` objects) must
# agree on what "this lane's width" means, and a second implementation
# would be a second place for that definition to drift.
from ..xodr_digest import _lane_width_samples, _width_stats
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


@dataclass
class ImplausibleLaneWidth:
    """One non-center lane whose sampled effective width is implausibly large.

    See ``check_lane_width_plausibility`` and
    ``config.LaneWidthValidationConstants`` for what this does and does not
    catch, and how ``threshold_m`` was chosen.
    """

    road_id: int
    lane_id: int
    width_min_m: float
    width_max_m: float
    threshold_m: float

    def __str__(self) -> str:
        return (
            f"Road {self.road_id} Lane {self.lane_id}: sampled width reaches "
            f"{self.width_max_m:.2f}m (min {self.width_min_m:.2f}m), "
            f"above the {self.threshold_m:.1f}m plausibility threshold"
        )


def check_lane_width_plausibility(
    roads: List[Road],
    threshold_m: Optional[float] = None,
) -> List[ImplausibleLaneWidth]:
    """Flag non-center lanes whose sampled width exceeds ``threshold_m``.

    Every ``<width>`` element the converter would emit is legal OpenDRIVE on
    its own — nothing in ``validate_lane_road_link_consistency``,
    ``validate_no_duplicate_road_ids``, or the ``asam-qc-opendrive``
    spec-compliance bundle (``qc_validate.py``) inspects a lane's width at
    all, so a corrupted input coordinate that inflates a lane to 20+ metres
    passes every existing check silently. This function closes that specific
    gap with a plain upper bound, calibrated against the one real map fixture
    in this repository — see ``config.LaneWidthValidationConstants`` for the
    calibration and its caveats.

    **What this does not do:** it is a heuristic threshold tuned on a single
    map, not a physical limit, so a legitimately wide lane on a different map
    could trigger a false report — this is why callers are expected to warn
    rather than abort on the result (see ``main.py``'s Step 6.9 call site).
    It also inherits every limitation of ``_lane_width_samples``/
    ``_width_stats``: in particular, a shape change confined between two
    sample points that leaves the sampled min, max, and mean all unchanged
    is invisible to it, no matter how the threshold is set.

    Args:
        roads: Road objects to check, in their pre-serialization Python form
            (i.e. before ``OpenDRIVE.to_xml()``/``save_opendrive_to_file``).
            Each non-center lane's own ``<width>`` elements are rendered via
            ``Lane.to_xml()`` (cheap: just that lane's small subtree, not the
            whole road) so the shared sampler can be reused unmodified.
        threshold_m: Width (metres) above which a lane is reported. Defaults
            to ``DEFAULT_CONFIG.lane_width_validation.implausible_width_threshold_m``.

    Returns:
        One ``ImplausibleLaneWidth`` per (road, lane) whose sampled
        ``width_max_m`` exceeds ``threshold_m``, in the order the roads and
        their lane sections were given. Empty if none exceed it.
    """
    if threshold_m is None:
        threshold_m = DEFAULT_CONFIG.lane_width_validation.implausible_width_threshold_m

    violations: List[ImplausibleLaneWidth] = []
    for road in roads:
        if road.lanes is None:
            continue
        lane_sections = road.lanes.lane_sections
        for index, lane_section in enumerate(lane_sections):
            section_end = (
                lane_sections[index + 1].s_offset
                if index + 1 < len(lane_sections)
                else road.length
            )
            section_span = section_end - lane_section.s_offset

            # Center lanes (id 0) never carry a <width> element (see
            # xodr_digest._process_road's identical exclusion) - only the
            # left/right dicts hold width-bearing lanes.
            non_center_lanes = list(lane_section.left_lanes.values()) + list(
                lane_section.right_lanes.values()
            )
            for lane in non_center_lanes:
                _segment_count, samples = _lane_width_samples(
                    lane.to_xml(), section_span
                )
                stats = _width_stats(samples)
                if stats is None:
                    continue
                if stats["width_max_m"] > threshold_m:
                    violations.append(
                        ImplausibleLaneWidth(
                            road_id=road.id,
                            lane_id=lane.lane_id,
                            width_min_m=stats["width_min_m"],
                            width_max_m=stats["width_max_m"],
                            threshold_m=threshold_m,
                        )
                    )
    return violations


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
