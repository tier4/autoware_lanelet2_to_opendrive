"""The reference line must follow the carriageway, not an attached non-vehicle lane.

``attach_boundary_adjacent_lanelets`` puts a ``road_shoulder`` /
``pedestrian_lane`` / ``bicycle_lane`` into the road it borders, which is what
gives those lanes their geometry and links. The lanelet it attaches can land
*outside* the reference side of the cross-section, and
``ReferenceLine.construct_from_lanelet_groups`` takes the LHT reference
boundary from ``sort_adjacent_groups(...)[-1]`` without looking at the
subtype. The attached lane then carries the reference line, and where it
begins or ends along a route the reference line steps sideways by its width
while the vehicle ``road -> road`` link stays correct.

Measured on the customer map (20,057 roads): all 644
``road.geometry.contact_point`` errors are joined by a ``road -> road`` edge of
the vehicle routing graph, 640 have a non-``road`` subtype carrying the
reference line on at least one side, and 596 have the reference lanelet's
subtype change across the contact (offset p50 0.467 m, max 11.81 m). Taking
the reference boundary from the outermost *vehicle* lanelet instead makes 530
of them coincide at the checker's 1e-6 threshold.

The attached lanelet then lies beyond the reference line and becomes a lane on
its other side, which ``LaneSection`` already emits -- RHT fills
``right_lanes`` today.
"""

import lanelet2
import pytest

from autoware_lanelet2_to_opendrive.opendrive.lane_section import LaneSection

# The footway is 2 m wide, so a reference line still on its outer edge lands
# at y = -2 and one on the carriageway edge at y = 0. A 0.1 m tolerance
# separates them by a factor of twenty while leaving room for the spline fit.
POSITION_TOLERANCE_M = 0.1


def _straight(y: float, **attributes: str) -> lanelet2.core.LineString3d:
    """A 30 m linestring along +x at the given lateral offset."""
    points = [
        lanelet2.core.Point3d(lanelet2.core.getId(), x, y, 0.0)
        for x in (0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0)
    ]
    linestring = lanelet2.core.LineString3d(lanelet2.core.getId(), points)
    for key, value in attributes.items():
        linestring.attributes[key] = value
    return linestring


def _lanelet(
    left: lanelet2.core.LineString3d,
    right: lanelet2.core.LineString3d,
    subtype: str,
) -> lanelet2.core.Lanelet:
    lanelet = lanelet2.core.Lanelet(lanelet2.core.getId(), left, right)
    lanelet.attributes["type"] = "lanelet"
    lanelet.attributes["subtype"] = subtype
    lanelet.attributes["location"] = "urban"
    lanelet.attributes["one_way"] = "yes"
    lanelet.attributes["speed_limit"] = "60"
    return lanelet


@pytest.fixture
def carriageway_with_a_footway_on_the_reference_side():
    """Two carriageway lanelets with a footway beyond the LHT reference side.

    Left to right: ``road`` (y 7..3.5), ``road`` (y 3.5..0),
    ``pedestrian_lane`` (y 0..-2). The two carriageway lanelets reach each
    other through the routing graph, which needs ``lane_change`` on the
    boundary they share; the footway reaches the carriageway only through
    ``build_boundary_lateral_adjacency``, exactly as on a real map.
    """
    kerb = {"type": "line_thin", "subtype": "solid"}
    y7 = _straight(7.0, **kerb)
    y35 = _straight(3.5, type="line_thin", subtype="dashed", lane_change="yes")
    y0 = _straight(0.0, **kerb)
    ym2 = _straight(-2.0, type="road_border")

    outer_lane = _lanelet(y7, y35, "road")
    inner_lane = _lanelet(y35, y0, "road")
    footway = _lanelet(y0, ym2, "pedestrian_lane")

    lanelet_map = lanelet2.core.LaneletMap()
    for lanelet in (outer_lane, inner_lane, footway):
        lanelet_map.add(lanelet)
    return lanelet_map, outer_lane, inner_lane, footway


def test_the_reference_line_sits_on_the_carriageway_edge(
    carriageway_with_a_footway_on_the_reference_side,
):
    """An attached footway must not pull the reference line onto its outer edge."""
    lanelet_map, outer_lane, inner_lane, footway = (
        carriageway_with_a_footway_on_the_reference_side
    )

    lane_section = LaneSection.construct_from_lanelet_groups(
        lanelet_map,
        [outer_lane, inner_lane, footway],
        s_offset=0.0,
        traffic_rule="LHT",
    )

    start = lane_section.center_lane.centerline_2d.evaluate(0.0)
    assert start[1] == pytest.approx(0.0, abs=POSITION_TOLERANCE_M), (
        "the LHT reference line must be the outermost VEHICLE lanelet's right"
        f" boundary (y = 0), not the footway's outer edge (y = -2); got y={start[1]}"
    )


def test_a_lanelet_beyond_the_reference_line_becomes_a_lane_on_its_other_side(
    carriageway_with_a_footway_on_the_reference_side,
):
    """The footway keeps its geometry, as lane -1 on the far side."""
    lanelet_map, outer_lane, inner_lane, footway = (
        carriageway_with_a_footway_on_the_reference_side
    )

    lane_section = LaneSection.construct_from_lanelet_groups(
        lanelet_map,
        [outer_lane, inner_lane, footway],
        s_offset=0.0,
        traffic_rule="LHT",
    )

    assert {
        lane_id: lane.lanelet_id for lane_id, lane in lane_section.left_lanes.items()
    } == {1: inner_lane.id, 2: outer_lane.id}
    assert {
        lane_id: lane.lanelet_id for lane_id, lane in lane_section.right_lanes.items()
    } == {-1: footway.id}


def test_an_all_carriageway_group_keeps_every_lane_on_one_side(
    carriageway_with_a_footway_on_the_reference_side,
):
    """Regression guard: with no attached lanelet nothing moves."""
    lanelet_map, outer_lane, inner_lane, _footway = (
        carriageway_with_a_footway_on_the_reference_side
    )

    lane_section = LaneSection.construct_from_lanelet_groups(
        lanelet_map,
        [outer_lane, inner_lane],
        s_offset=0.0,
        traffic_rule="LHT",
    )

    assert sorted(lane_section.left_lanes) == [1, 2]
    assert lane_section.right_lanes == {}
    start = lane_section.center_lane.centerline_2d.evaluate(0.0)
    assert start[1] == pytest.approx(0.0, abs=POSITION_TOLERANCE_M)
