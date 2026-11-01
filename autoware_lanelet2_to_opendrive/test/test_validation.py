"""Tests for OpenDRIVE validation functionality."""

from pathlib import Path

from autoware_lanelet2_to_opendrive.config import DEFAULT_CONFIG
from autoware_lanelet2_to_opendrive.opendrive.enums import ContactPoint, ElementType
from autoware_lanelet2_to_opendrive.opendrive.lane import Lane
from autoware_lanelet2_to_opendrive.opendrive.lane_elements import LaneLink, LaneWidth
from autoware_lanelet2_to_opendrive.opendrive.lane_section import LaneSection
from autoware_lanelet2_to_opendrive.opendrive.lane_sections import Lanes
from autoware_lanelet2_to_opendrive.opendrive.opendrive_dataclass import LaneType
from autoware_lanelet2_to_opendrive.opendrive.road import Road
from autoware_lanelet2_to_opendrive.opendrive.road_links import (
    Predecessor,
    RoadLink,
    Successor,
)
from autoware_lanelet2_to_opendrive.opendrive.validation import (
    check_lane_width_plausibility,
    validate_lane_road_link_consistency,
    validate_no_duplicate_road_ids,
)


def create_test_lane(
    lane_id: int,
    has_predecessor: bool = False,
    has_successor: bool = False,
) -> Lane:
    """Create a test lane with optional predecessor/successor."""
    lane = Lane(
        lane_id=lane_id,
        lane_type=LaneType.DRIVING,
        level=False,
    )

    if has_predecessor:
        lane.predecessor = LaneLink(id=lane_id)

    if has_successor:
        lane.successor = LaneLink(id=lane_id)

    return lane


def create_test_road(
    road_id: int,
    has_road_predecessor: bool = False,
    has_road_successor: bool = False,
    junction: int = -1,
) -> Road:
    """Create a test road with optional road-level predecessor/successor."""
    # Create road link if needed
    road_link = None
    if has_road_predecessor or has_road_successor:
        predecessor = None
        successor = None

        if has_road_predecessor:
            predecessor = Predecessor(
                element_type=ElementType.ROAD,
                element_id=road_id - 1,
                contact_point=ContactPoint.END,
            )

        if has_road_successor:
            successor = Successor(
                element_type=ElementType.ROAD,
                element_id=road_id + 1,
                contact_point=ContactPoint.START,
            )

        road_link = RoadLink(predecessor=predecessor, successor=successor)

    # Create lane section with test lanes
    lane_section = LaneSection(s_offset=0.0)

    # Create lanes sections
    lanes = Lanes(lane_sections=[lane_section])

    # Create road
    road = Road(
        id=road_id,
        name=f"test_road_{road_id}",
        length=100.0,
        junction=junction,
        link=road_link,
        lanes=lanes,
    )

    return road


class TestLaneRoadLinkConsistency:
    """Test suite for lane-road link consistency validation."""

    def test_valid_road_with_no_connections(self):
        """Test that a road with no lane or road connections is valid."""
        road = create_test_road(
            road_id=1, has_road_predecessor=False, has_road_successor=False
        )

        # Add lane without connections
        lane = create_test_lane(lane_id=-1, has_predecessor=False, has_successor=False)
        road.lanes.lane_sections[0].right_lanes = {-1: lane}

        result = validate_lane_road_link_consistency([road])

        assert result.is_valid
        assert result.error_count == 0

    def test_valid_road_with_consistent_connections(self):
        """Test that a road with consistent lane and road connections is valid."""
        road = create_test_road(
            road_id=1, has_road_predecessor=True, has_road_successor=True
        )

        # Add lane with connections (matches road connections)
        lane = create_test_lane(lane_id=-1, has_predecessor=True, has_successor=True)
        road.lanes.lane_sections[0].right_lanes = {-1: lane}

        result = validate_lane_road_link_consistency([road])

        assert result.is_valid
        assert result.error_count == 0

    def test_invalid_lane_predecessor_without_road_predecessor(self):
        """Test that lane predecessor without road predecessor is invalid."""
        road = create_test_road(
            road_id=1, has_road_predecessor=False, has_road_successor=True
        )

        # Add lane with predecessor but road has no predecessor
        lane = create_test_lane(lane_id=-1, has_predecessor=True, has_successor=True)
        road.lanes.lane_sections[0].right_lanes = {-1: lane}

        result = validate_lane_road_link_consistency([road])

        assert not result.is_valid
        assert result.error_count == 1
        assert result.errors[0].road_id == 1
        assert result.errors[0].lane_id == -1
        assert result.errors[0].connection_type == "predecessor"
        assert "Lane has predecessor but road does not" in result.errors[0].message

    def test_invalid_lane_successor_without_road_successor(self):
        """Test that lane successor without road successor is invalid."""
        road = create_test_road(
            road_id=1, has_road_predecessor=True, has_road_successor=False
        )

        # Add lane with successor but road has no successor
        lane = create_test_lane(lane_id=-1, has_predecessor=True, has_successor=True)
        road.lanes.lane_sections[0].right_lanes = {-1: lane}

        result = validate_lane_road_link_consistency([road])

        assert not result.is_valid
        assert result.error_count == 1
        assert result.errors[0].road_id == 1
        assert result.errors[0].lane_id == -1
        assert result.errors[0].connection_type == "successor"
        assert "Lane has successor but road does not" in result.errors[0].message

    def test_invalid_multiple_lanes_with_invalid_connections(self):
        """Test that multiple lanes with invalid connections are detected."""
        road = create_test_road(
            road_id=1, has_road_predecessor=False, has_road_successor=False
        )

        # Add multiple lanes with invalid connections
        lane1 = create_test_lane(lane_id=-1, has_predecessor=True, has_successor=True)
        lane2 = create_test_lane(lane_id=-2, has_predecessor=True, has_successor=False)

        road.lanes.lane_sections[0].right_lanes = {-1: lane1, -2: lane2}

        result = validate_lane_road_link_consistency([road])

        assert not result.is_valid
        assert result.error_count == 3  # lane1 pred, lane1 succ, lane2 pred

    def test_connecting_road_requires_road_links_for_lane_links(self):
        """Test that connecting roads also require road links before lane links.

        Previous behavior allowed connecting roads to have lane links without road links,
        but this violated OpenDRIVE specification and caused issues. Connecting roads
        must also have road-level links before lane-level links can be created.
        """
        # Create connecting road (junction member)
        road = create_test_road(
            road_id=1,
            has_road_predecessor=False,
            has_road_successor=False,
            junction=100,  # Member of junction 100
        )

        # Add lane with connections (should NOT be allowed even for connecting roads)
        lane = create_test_lane(lane_id=-1, has_predecessor=True, has_successor=True)
        road.lanes.lane_sections[0].right_lanes = {-1: lane}

        result = validate_lane_road_link_consistency([road])

        # Connecting roads must also have consistent road/lane links
        assert not result.is_valid
        assert result.error_count == 2  # predecessor and successor both invalid

    def test_validation_result_error_summary(self):
        """Test that ValidationResult provides useful error summary."""
        road = create_test_road(
            road_id=1, has_road_predecessor=False, has_road_successor=False
        )

        lane = create_test_lane(lane_id=-1, has_predecessor=True, has_successor=False)
        road.lanes.lane_sections[0].right_lanes = {-1: lane}

        result = validate_lane_road_link_consistency([road])

        summary = result.get_error_summary()

        assert "Found 1 validation errors" in summary
        assert "Road 1 Lane -1" in summary
        assert "predecessor" in summary

    def test_empty_roads_list(self):
        """Test validation with empty roads list."""
        result = validate_lane_road_link_consistency([])

        assert result.is_valid
        assert result.error_count == 0

    def test_road_without_lanes(self):
        """Test validation with road that has no lanes."""
        road = Road(
            id=1,
            name="test_road_1",
            length=100.0,
            junction=-1,
            link=None,
            lanes=None,
        )

        result = validate_lane_road_link_consistency([road])

        assert result.is_valid
        assert result.error_count == 0


class TestNoDuplicateRoadIds:
    """Test suite for duplicate road ID validation.

    ``validate_no_duplicate_road_ids`` only requires ``road.id``, so these tests
    use bare ``Road(id=...)`` instances rather than the fully-populated
    ``create_test_road`` helper above.
    """

    def test_all_unique_ids_valid(self):
        """Test that roads with all-unique IDs pass validation."""
        roads = [Road(id=1), Road(id=2), Road(id=3)]

        result = validate_no_duplicate_road_ids(roads)

        assert result.is_valid
        assert result.errors == []

    def test_single_duplicate_id_invalid(self):
        """Test that a duplicated ID is reported along with its occurrence count."""
        roads = [Road(id=1), Road(id=2), Road(id=1)]

        result = validate_no_duplicate_road_ids(roads)

        assert not result.is_valid
        assert result.error_count == 1
        assert result.errors[0].road_id == 1
        assert result.errors[0].connection_type == "duplicate"
        assert "Road ID 1 appears 2 times" in result.errors[0].message

    def test_multiple_duplicate_ids_reported_independently(self):
        """Test that distinct duplicated IDs are each reported with their own counts."""
        roads = [
            Road(id=1),
            Road(id=2),
            Road(id=1),
            Road(id=3),
            Road(id=2),
        ]

        result = validate_no_duplicate_road_ids(roads)

        assert not result.is_valid
        assert result.error_count == 2

        errors_by_id = {error.road_id: error for error in result.errors}
        assert set(errors_by_id) == {1, 2}
        assert "appears 2 times" in errors_by_id[1].message
        assert "appears 2 times" in errors_by_id[2].message

    def test_id_repeated_three_times_counts_correctly(self):
        """Test that the occurrence count distinguishes a 3x duplicate from a 2x one."""
        roads = [
            Road(id=1),
            Road(id=1),
            Road(id=1),
            Road(id=2),
            Road(id=2),
        ]

        result = validate_no_duplicate_road_ids(roads)

        assert not result.is_valid
        assert result.error_count == 2

        errors_by_id = {error.road_id: error for error in result.errors}
        assert "Road ID 1 appears 3 times" in errors_by_id[1].message
        assert "Road ID 2 appears 2 times" in errors_by_id[2].message

    def test_empty_roads_list(self):
        """Test validation with an empty roads list."""
        result = validate_no_duplicate_road_ids([])

        assert result.is_valid
        assert result.errors == []

    def test_validation_result_error_summary(self):
        """Test that ValidationResult provides a useful error summary for duplicates."""
        roads = [Road(id=5), Road(id=5)]

        result = validate_no_duplicate_road_ids(roads)

        summary = result.get_error_summary()

        assert "Found 1 validation errors" in summary
        assert "Road 5 Lane 0 (duplicate)" in summary
        assert "Road ID 5 appears 2 times in the road list" in summary

    def test_duplicates_are_reported_not_raised(self):
        """Duplicate IDs are surfaced via ValidationResult, never raised."""
        roads = [Road(id=1), Road(id=1), Road(id=1)]

        # Should complete normally and return a result, never raise.
        result = validate_no_duplicate_road_ids(roads)

        assert not result.is_valid
        assert result.error_count == 1

    def test_the_caller_reports_rather_than_aborting(self):
        """Pin ``main.py``'s report-only handling of a failed result.

        The validator returning a result rather than raising says nothing
        about what the caller does with it -- and the caller is where
        "report-only" either holds or does not. An earlier version of this
        class claimed to pin that contract from
        ``test_duplicates_are_reported_not_raised``, which only calls the
        validator: changing ``main.py``'s ``print`` to a ``raise`` failed no
        test in the repository.

        Read from the source rather than by running a conversion, so the
        check costs nothing and a switch to fail-fast shows up here as an
        intentional change.
        """
        import ast

        main_py = (
            Path(__file__).resolve().parents[1]
            / "src"
            / "autoware_lanelet2_to_opendrive"
            / "main.py"
        )
        tree = ast.parse(main_py.read_text(encoding="utf-8"))

        branches = [
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.If) and "dup_result" in ast.dump(node.test)
        ]
        assert branches, (
            "no branch on a duplicate-road-id result found in main.py; the "
            "call site moved and this test needs to follow it"
        )

        for branch in branches:
            for statement in branch.body:
                for inner in ast.walk(statement):
                    assert not isinstance(inner, ast.Raise), (
                        f"main.py:{inner.lineno} raises on a duplicate road "
                        "id. That is a reasonable thing to want, but it is a "
                        "behaviour change: update this test deliberately."
                    )
                    if isinstance(inner, ast.Call):
                        name = (
                            inner.func.attr
                            if isinstance(inner.func, ast.Attribute)
                            else getattr(inner.func, "id", "")
                        )
                        assert name != "exit", (
                            f"main.py:{inner.lineno} exits on a duplicate "
                            "road id; see above."
                        )
class TestLaneWidthPlausibility:
    """Test suite for the output-side lane-width plausibility check.

    See ``check_lane_width_plausibility``'s docstring for what this catches
    (a plain upper bound on sampled width, calibrated on a single map) and
    what it does not (shape changes that leave sampled min/max/mean
    unchanged; legitimately wide lanes on a different map).
    """

    def test_healthy_width_does_not_fire(self):
        """A constant, in-distribution lane width produces no violations."""
        road = create_test_road(road_id=1)
        lane = create_test_lane(lane_id=-1)
        lane.widths.append(LaneWidth(s_offset=0.0, a=3.5))
        road.lanes.lane_sections[0].right_lanes = {-1: lane}

        violations = check_lane_width_plausibility([road])

        assert violations == []

    def test_width_above_threshold_is_detected(self):
        """A lane sampled above the default threshold is reported once."""
        road = create_test_road(road_id=7)
        lane = create_test_lane(lane_id=-2)
        lane.widths.append(LaneWidth(s_offset=0.0, a=22.0))
        road.lanes.lane_sections[0].right_lanes = {-2: lane}

        violations = check_lane_width_plausibility([road])

        assert len(violations) == 1
        violation = violations[0]
        assert violation.road_id == 7
        assert violation.lane_id == -2
        assert violation.width_max_m == 22.0
        assert violation.width_min_m == 22.0
        assert (
            violation.threshold_m
            == DEFAULT_CONFIG.lane_width_validation.implausible_width_threshold_m
        )
        assert "22.00m" in str(violation)

    def test_custom_threshold_is_respected(self):
        """An explicit ``threshold_m`` overrides the config default."""
        road = create_test_road(road_id=1)
        lane = create_test_lane(lane_id=-1)
        lane.widths.append(LaneWidth(s_offset=0.0, a=5.0))
        road.lanes.lane_sections[0].right_lanes = {-1: lane}

        assert check_lane_width_plausibility([road], threshold_m=10.0) == []
        violations = check_lane_width_plausibility([road], threshold_m=4.0)
        assert len(violations) == 1
        assert violations[0].threshold_m == 4.0

    def test_road_without_lanes_is_skipped(self):
        """A road with ``lanes=None`` is skipped rather than raising."""
        road = Road(id=1, name="test_road_1", length=100.0, junction=-1, lanes=None)

        assert check_lane_width_plausibility([road]) == []

    def test_multiple_lane_sections_use_correct_span(self):
        """A lane in a non-final section is sampled against its own span.

        Regression guard for the ``section_span`` computation: it must use
        the *next* section's ``s_offset`` (not ``road.length``) for every
        section but the last.
        """
        road = create_test_road(road_id=1)
        road.length = 100.0
        first_section = road.lanes.lane_sections[0]
        second_section = LaneSection(s_offset=40.0)
        road.lanes.lane_sections.append(second_section)

        first_lane = create_test_lane(lane_id=-1)
        first_lane.widths.append(LaneWidth(s_offset=0.0, a=3.5))
        first_section.right_lanes = {-1: first_lane}

        second_lane = create_test_lane(lane_id=-1)
        second_lane.widths.append(LaneWidth(s_offset=0.0, a=20.0))
        second_section.right_lanes = {-1: second_lane}

        violations = check_lane_width_plausibility([road])

        assert len(violations) == 1
        assert violations[0].width_max_m == 20.0
