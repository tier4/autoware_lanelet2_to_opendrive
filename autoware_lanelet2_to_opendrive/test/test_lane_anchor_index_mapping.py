"""A lane id and a sorted-lanelet index must name the same lanelet both ways.

``Road.evaluate_lane_anchor_pose`` takes a **sorted-lanelet index** and works
out which OpenDRIVE lane sits there, by counting outward from the reference
line. Callers that hold a lane id and want its anchor need the inverse, and
`divergence._lane_anchor_xyz` computed it as

    sorted_index = n - lane_id        # LHT
    sorted_index = -lane_id - 1       # RHT

which is not the inverse. It is the *reduction* of the inverse for the case
the comment beside the forward map spells out: "with the reference line at
the outermost lanelet ... the outer side is empty". That stopped being
general when the reference line became the outermost **vehicle** lanelet, so
an attached sidewalk or shoulder can now sort outside it -- 18.8% of roads on
a real map. On those the shortcut names a different lanelet, the anchor comes
back offset by the lanes in between, and the divergence sanity gate rejects
the site for an endpoint separation it created itself.

Measured on a 6,672-lanelet map before the fix: all **178** rejected sites
report a lane-pair separation of p50 2.975 m (min 1.716, max 3.804) against a
p50 driving-lane width of 3.27 m -- one lane, never two, never zero. Those
178 rejections are the junctions behind
``road.lane.link.lanes_across_lane_sections`` (513) and
``road.linkage.is_junction_needed`` (140), 653 of the output's 1,267 ASAM
errors, plus the 20 successions the routing graph cannot traverse.

The property asserted here is the round trip, which needs no convention: for
every index the road has, mapping it to a lane id and back must return the
index it started from.
"""

from __future__ import annotations

import pytest

from autoware_lanelet2_to_opendrive.opendrive.enums import LaneType, TrafficRule
from autoware_lanelet2_to_opendrive.opendrive.lane import Lane
from autoware_lanelet2_to_opendrive.opendrive.lane_elements import LaneWidth
from autoware_lanelet2_to_opendrive.opendrive.lane_section import LaneSection
from autoware_lanelet2_to_opendrive.opendrive.lane_sections import Lanes
from autoware_lanelet2_to_opendrive.opendrive.road import Road


def _road(*, rule: TrafficRule, lanelets: int, reference_index: int | None) -> Road:
    """A road with ``lanelets`` sorted lanelets and a given reference index.

    The lane section carries one lane per lanelet on each side, so the
    cumulative-width walk in ``evaluate_lane_anchor_pose`` can always find
    the lanes it needs; only the index arithmetic is under test here.
    """
    section = LaneSection(s_offset=0.0)
    for lane_id in range(1, lanelets + 1):
        for container, identifier in (
            (section.left_lanes, lane_id),
            (section.right_lanes, -lane_id),
        ):
            lane = Lane(lane_id=identifier, lane_type=LaneType.DRIVING)
            lane.widths.append(LaneWidth(s_offset=0.0, a=3.2, b=0.0, c=0.0, d=0.0))
            container[identifier] = lane

    return Road(
        id=1,
        length=10.0,
        rule=rule,
        lanes=Lanes(lane_sections=[section]),
        sorted_lanelet_ids=list(range(100, 100 + lanelets)),
        reference_lanelet_index=reference_index,
    )


@pytest.mark.parametrize("rule", [TrafficRule.LHT, TrafficRule.RHT])
@pytest.mark.parametrize(
    "lanelets, reference_index",
    [
        # The old assumption: the reference line is the outermost lanelet.
        # Here the shortcut and the inverse agree, so these are the guard
        # rails that the fix does not move what already worked.
        (1, None),
        (3, None),
        # And the shapes that broke it: something sorts outside the
        # reference line, so lane ids run in both directions.
        (2, 0),
        (3, 1),
        (4, 2),
        (5, 0),
    ],
)
def test_a_lane_id_and_its_sorted_index_map_to_each_other(
    rule, lanelets, reference_index
):
    road = _road(rule=rule, lanelets=lanelets, reference_index=reference_index)

    for sorted_index in range(lanelets):
        lane_id = road.lane_id_for_sorted_index(sorted_index)
        assert lane_id is not None, sorted_index
        assert road.sorted_index_for_lane_id(lane_id) == sorted_index, (
            f"{rule.value}, {lanelets} lanelets, reference at "
            f"{reference_index}: index {sorted_index} maps to lane {lane_id}, "
            f"which maps back to {road.sorted_index_for_lane_id(lane_id)}"
        )


@pytest.mark.parametrize("rule", [TrafficRule.LHT, TrafficRule.RHT])
def test_the_lane_beside_the_reference_line_anchors_on_it(rule):
    """The one absolute fact in the arithmetic, so the round trip cannot
    be satisfied by two mistakes that cancel.

    At the reference index the anchor *is* the reference line, so the lane
    the reference index maps to must be the innermost one of its side --
    ``+1`` under LHT, ``-1`` under RHT -- and the inverse must agree.
    """
    road = _road(rule=rule, lanelets=4, reference_index=2)
    innermost = 1 if rule == TrafficRule.LHT else -1

    assert road.lane_id_for_sorted_index(2) == innermost
    assert road.sorted_index_for_lane_id(innermost) == 2


def test_a_lane_the_road_does_not_have_maps_to_nothing():
    """Out of range is ``None``, not a clamped index.

    The caller falls back to the road's reference endpoint on ``None``;
    a clamped index would silently anchor on the wrong lanelet, which is the
    class of defect this pair of methods exists to remove.
    """
    road = _road(rule=TrafficRule.LHT, lanelets=2, reference_index=0)

    assert road.sorted_index_for_lane_id(0) is None
    assert road.sorted_index_for_lane_id(9) is None
    assert road.sorted_index_for_lane_id(-9) is None
    assert road.lane_id_for_sorted_index(2) is None
    assert road.lane_id_for_sorted_index(-1) is None
