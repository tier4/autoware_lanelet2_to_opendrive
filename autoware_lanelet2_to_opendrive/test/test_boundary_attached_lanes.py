"""Non-vehicle lanelets become lanes of the road they share a boundary with.

``road_shoulder`` / ``pedestrian_lane`` / ``bicycle_lane`` lanelets are not
vertices of the ``Participants.Vehicle`` routing graph that drives lateral
grouping, so ``find_adjacent_groups`` can never put them in the same group
as the driving road they belong to.  Before the shared-boundary attachment
pass each of them became a standalone, link-less, one-lane OpenDRIVE road
geometrically overlapping that driving road.

These tests pin the new behaviour end-to-end (one road, extra lane, correct
lane type and order) and unit-test the deterministic rules of
``attach_boundary_adjacent_lanelets``.
"""

from __future__ import annotations

import contextlib
import io
from typing import List, Optional, Set, Tuple

import lanelet2
import pytest

from autoware_lanelet2_to_opendrive.conversion_config import (
    ConversionConfig,
    OriginSpec,
    ParamPoly3Config,
)
from autoware_lanelet2_to_opendrive.main import convert_lanelet2_to_opendrive
from autoware_lanelet2_to_opendrive.util import (
    BOUNDARY_ATTACHED_LANELET_SUBTYPES,
    attach_boundary_adjacent_lanelets,
    build_boundary_lateral_adjacency,
    sort_adjacent_groups,
)


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------


def _straight_bound(y: float, x0: float = 0.0) -> lanelet2.core.LineString3d:
    """A 10 m straight linestring at lateral offset ``y``."""
    points = [
        lanelet2.core.Point3d(lanelet2.core.getId(), x0 + 0.0, y, 0.0),
        lanelet2.core.Point3d(lanelet2.core.getId(), x0 + 5.0, y, 0.0),
        lanelet2.core.Point3d(lanelet2.core.getId(), x0 + 10.0, y, 0.0),
    ]
    return lanelet2.core.LineString3d(lanelet2.core.getId(), points)


def _lateral_stack(
    subtypes: List[str],
    widths: Optional[List[float]] = None,
    x0: float = 0.0,
) -> Tuple[lanelet2.core.LaneletMap, List[lanelet2.core.Lanelet]]:
    """Build a left-to-right stack of lanelets sharing their boundaries.

    ``subtypes[0]`` is the leftmost lanelet.  Neighbours share one
    ``LineString3d`` instance, which is the adjacency signal the Lanelet2
    map specification guarantees for non-vehicle lanes.
    """
    widths = widths or [3.5] * len(subtypes)
    y = 0.0
    bounds = [_straight_bound(y, x0)]
    for width in widths:
        y -= width
        bounds.append(_straight_bound(y, x0))

    lanelet_map = lanelet2.core.LaneletMap()
    lanelets = []
    for index, subtype in enumerate(subtypes):
        lanelet = lanelet2.core.Lanelet(
            lanelet2.core.getId(), bounds[index], bounds[index + 1]
        )
        lanelet.attributes["subtype"] = subtype
        lanelet_map.add(lanelet)
        lanelets.append(lanelet)
    return lanelet_map, lanelets


def _convert(lanelet_map: lanelet2.core.LaneletMap):
    """Run the normal converter path and return the OpenDRIVE XML root."""
    config = ConversionConfig(
        origin=OriginSpec(mgrs_code="54SUE"),
        parampoly3=ParamPoly3Config(enabled=False),
    )
    with (
        contextlib.redirect_stdout(io.StringIO()),
        contextlib.redirect_stderr(io.StringIO()),
    ):
        opendrive, *_ = convert_lanelet2_to_opendrive(lanelet_map, config)
    return opendrive.to_xml()


def _lane_types_per_road(root) -> List[List[str]]:
    """``[[lane type, ...], ...]``, one inner list per emitted road."""
    return [
        [
            lane.get("type")
            for lane in road.iter("lane")
            if lane.get("id") not in (None, "0")
        ]
        for road in root.findall("road")
    ]


# --------------------------------------------------------------------------
# end-to-end: one road, not two
# --------------------------------------------------------------------------


def test_shoulder_sharing_right_boundary_becomes_a_lane_of_the_road() -> None:
    """The headline case: road + shoulder -> ONE road with a shoulder lane.

    Fails without the shared-boundary attachment pass: the routing graph
    has no vertex for the shoulder, so it used to be emitted as a second,
    link-less road overlapping the first.
    """
    lanelet_map, _ = _lateral_stack(["road", "road_shoulder"], widths=[3.5, 2.0])

    root = _convert(lanelet_map)

    roads = root.findall("road")
    assert len(roads) == 1, (
        "a road lanelet and the shoulder sharing its right boundary must "
        f"produce one road, got {len(roads)}"
    )
    lane_types = _lane_types_per_road(root)[0]
    assert sorted(lane_types) == ["driving", "shoulder"]


@pytest.mark.parametrize(
    "subtype,expected_lane_type",
    [
        ("road_shoulder", "shoulder"),
        ("pedestrian_lane", "sidewalk"),
        ("bicycle_lane", "biking"),
    ],
)
def test_every_boundary_attached_subtype_joins_the_road(
    subtype: str, expected_lane_type: str
) -> None:
    """All three attachable subtypes fold into the adjacent driving road."""
    assert subtype in BOUNDARY_ATTACHED_LANELET_SUBTYPES
    lanelet_map, _ = _lateral_stack(["road", subtype], widths=[3.5, 2.0])

    root = _convert(lanelet_map)

    assert len(root.findall("road")) == 1
    assert sorted(_lane_types_per_road(root)[0]) == sorted(
        ["driving", expected_lane_type]
    )


def test_attached_lane_sits_outside_the_driving_lane() -> None:
    """Lane order: the shoulder is outside the outermost driving lane.

    With RHT the reference line is the leftmost lanelet's left boundary and
    lane ids run -1, -2, ... rightwards, so the shoulder must carry the
    largest ``|id|``.  Cumulative width from the reference line therefore
    stays monotonic.
    """
    lanelet_map, _ = _lateral_stack(
        ["road", "road", "road_shoulder"], widths=[3.5, 3.5, 2.0]
    )

    root = _convert(lanelet_map)

    (road,) = root.findall("road")
    lanes = {
        int(lane.get("id")): lane
        for lane in road.iter("lane")
        if lane.get("id") not in (None, "0")
    }
    assert sorted(lanes) == [-3, -2, -1]
    assert lanes[-1].get("type") == "driving"
    assert lanes[-2].get("type") == "driving"
    assert lanes[-3].get("type") == "shoulder"

    # Cumulative offset from the reference line must grow with |lane id|.
    cumulative = 0.0
    for lane_id in (-1, -2, -3):
        width = float(lanes[lane_id].find("width").get("a"))
        assert width > 0.0
        cumulative += width
    assert cumulative == pytest.approx(3.5 + 3.5 + 2.0, abs=0.2)


def test_chain_of_non_vehicle_lanelets_attaches_through_the_chain() -> None:
    """``road | bicycle_lane | pedestrian_lane`` becomes one three-lane road.

    The pedestrian lane never touches the road directly; it reaches the
    road's group through the bicycle lane.
    """
    lanelet_map, _ = _lateral_stack(
        ["road", "bicycle_lane", "pedestrian_lane"], widths=[3.5, 2.0, 2.0]
    )

    root = _convert(lanelet_map)

    assert len(root.findall("road")) == 1
    assert sorted(_lane_types_per_road(root)[0]) == ["biking", "driving", "sidewalk"]


def test_walkway_is_not_attached_and_keeps_its_own_road() -> None:
    """``walkway`` is explicitly out of scope and must not change.

    Measured on both reference maps, 0% of walkway lanelets share a
    boundary linestring with a road lanelet; the one synthetic case where
    they do must still emit two roads so the existing walkway handling is
    untouched.
    """
    lanelet_map, _ = _lateral_stack(["road", "walkway"], widths=[3.5, 2.0])

    root = _convert(lanelet_map)

    assert len(root.findall("road")) == 2, "walkway must keep its own road"
    assert sorted(sorted(t) for t in _lane_types_per_road(root)) == [
        ["driving"],
        ["sidewalk"],
    ]


def test_detached_shoulder_still_becomes_its_own_road() -> None:
    """A shoulder that no road borders is kept, not dropped."""
    lanelet_map, _ = _lateral_stack(["road_shoulder"], widths=[2.0])

    root = _convert(lanelet_map)

    assert len(root.findall("road")) == 1
    assert _lane_types_per_road(root)[0] == ["shoulder"]


# --------------------------------------------------------------------------
# unit: the adjacency relation
# --------------------------------------------------------------------------


def test_boundary_adjacency_reports_side_and_ignores_road_to_road() -> None:
    """Only pairs involving an attachable subtype produce an edge."""
    lanelet_map, (left_road, right_road, shoulder) = _lateral_stack(
        ["road", "road", "road_shoulder"]
    )
    del lanelet_map

    left_of, right_of = build_boundary_lateral_adjacency(
        [left_road, right_road, shoulder]
    )

    # road <-> road stays with the routing graph, whose lane-change
    # semantics this relation does not model.
    assert left_road.id not in right_of
    assert right_road.id not in left_of

    assert right_of[right_road.id] == {shoulder.id}
    assert left_of[shoulder.id] == {right_road.id}


def test_antiparallel_shared_boundary_is_not_lateral_adjacency() -> None:
    """A linestring shared in the opposite orientation is not adjacency.

    Two lanelets referencing the same linestring inverted lie on the SAME
    side of it (e.g. opposing carriageways sharing their centre line);
    merging them into one road would be wrong.
    """
    road_left = _straight_bound(0.0)
    shared = _straight_bound(-3.5)
    outer = _straight_bound(-5.5)
    road = lanelet2.core.Lanelet(lanelet2.core.getId(), road_left, shared)
    road.attributes["subtype"] = "road"
    shoulder = lanelet2.core.Lanelet(lanelet2.core.getId(), shared.invert(), outer)
    shoulder.attributes["subtype"] = "road_shoulder"

    left_of, right_of = build_boundary_lateral_adjacency([road, shoulder])

    assert left_of == {}
    assert right_of == {}


def test_sort_adjacent_groups_orders_an_attached_lanelet() -> None:
    """Left-to-right ordering works for a group the routing graph cannot sort."""
    lanelet_map, (road, shoulder) = _lateral_stack(["road", "road_shoulder"])

    ordered = sort_adjacent_groups(lanelet_map, {road, shoulder})

    assert [lanelet.id for lanelet in ordered] == [road.id, shoulder.id]


# --------------------------------------------------------------------------
# unit: the deterministic attachment rules
# --------------------------------------------------------------------------


def test_lanelet_bordering_two_groups_joins_one_and_never_merges_them() -> None:
    """Rule 1 + 3: one group only, tie broken on the smallest lanelet id.

    A shoulder squeezed between two carriageways that the vehicle routing
    graph deliberately keeps apart must not fuse them into a single road.
    """
    _, (left_road, shoulder, right_road) = _lateral_stack(
        ["road", "road_shoulder", "road"]
    )
    group_a: Set[lanelet2.core.Lanelet] = {left_road}
    group_b: Set[lanelet2.core.Lanelet] = {right_road}

    result = attach_boundary_adjacent_lanelets([group_a, group_b], {shoulder})

    assert len(result.groups) == 2, "groups must never be merged"
    assert not result.unattached
    assert result.ambiguous_ids == [shoulder.id]
    winner = min(left_road.id, right_road.id)
    joined = [g for g in result.groups if shoulder in g]
    assert len(joined) == 1
    assert min(lanelet.id for lanelet in joined[0] if lanelet is not shoulder) == winner


def test_nearest_group_wins_over_a_longer_chain() -> None:
    """Rule 2: hop distance decides before the id tie-break.

    ``roadA | pedestrian | bicycle | roadB`` — the bicycle lane touches
    ``roadB`` directly, so it joins ``roadB`` even when ``roadA`` owns the
    smaller id and would win a tie.
    """
    _, (road_a, pedestrian, bicycle, road_b) = _lateral_stack(
        ["road", "pedestrian_lane", "bicycle_lane", "road"]
    )
    assert road_a.id < road_b.id  # ids are allocated left to right

    result = attach_boundary_adjacent_lanelets(
        [{road_a}, {road_b}], {pedestrian, bicycle}
    )

    group_of = {
        lanelet.id: index
        for index, group in enumerate(result.groups)
        for lanelet in group
    }
    assert group_of[pedestrian.id] == group_of[road_a.id]
    assert group_of[bicycle.id] == group_of[road_b.id]
    assert result.ambiguous_ids == []


def test_attachment_is_independent_of_input_iteration_order() -> None:
    """The result is reproducible whatever order the sets iterate in."""
    _, (left_road, shoulder, right_road) = _lateral_stack(
        ["road", "road_shoulder", "road"]
    )

    def run(groups):
        result = attach_boundary_adjacent_lanelets(
            [set(group) for group in groups], {shoulder}
        )
        return [
            sorted(lanelet.id for lanelet in group)
            for group in result.groups
            if shoulder in group
        ]

    assert run([{left_road}, {right_road}]) == run([{right_road}, {left_road}])


def test_unattachable_lanelet_is_returned_not_dropped() -> None:
    """Rule 4: nothing is silently discarded."""
    _, (road, lone_shoulder) = _lateral_stack(["road", "road_shoulder"], x0=0.0)
    _, (far_shoulder,) = _lateral_stack(["road_shoulder"], x0=500.0)
    del lone_shoulder

    result = attach_boundary_adjacent_lanelets([{road}], {far_shoulder})

    assert result.unattached == {far_shoulder}
    assert result.groups == [{road}]
