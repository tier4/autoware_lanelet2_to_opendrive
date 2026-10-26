"""Tests for the direct-junction laneLink completion pass.

The pass exists because OpenDRIVE allows exactly one road-level
``<successor>`` per road and a lane link may not name a different road: a
lanelet whose routing followers *mix* a ``turn_direction`` junction
connector with a plain regular-road lanelet can only keep both branches if
the plain-road branch also becomes a ``<connection>`` of the junction that
owns the successor slot.
"""

from typing import Dict, List, Optional, Set, Tuple, cast

from autoware_lanelet2_to_opendrive.direct_junction_completion import (
    complete_direct_junction_lanelinks,
)
from autoware_lanelet2_to_opendrive.divergence import (
    _make_zero_length_connecting_road,
)
from autoware_lanelet2_to_opendrive.opendrive.enums import ContactPoint, TrafficRule
from autoware_lanelet2_to_opendrive.opendrive.junction import Connection, Junction
from autoware_lanelet2_to_opendrive.opendrive.junction import (
    LaneLink as JunctionLaneLink,
)
from autoware_lanelet2_to_opendrive.opendrive.road import Road

XYZ = Tuple[float, float, float]


class _RoadStub:
    """Minimal stand-in for a lanelet-backed ``Road``.

    Only the attributes the completion pass reads are provided.  Lane-anchor
    metadata is deliberately absent so the pass falls back to the road's
    reference endpoints, exactly as it does for a road whose spline could
    not be evaluated.
    """

    def __init__(
        self,
        road_id: int,
        lanelet_to_lane: Dict[int, int],
        *,
        junction: int = -1,
        start_xyz: XYZ = (0.0, 0.0, 0.0),
        end_xyz: XYZ = (0.0, 0.0, 0.0),
    ) -> None:
        self.id = road_id
        self.junction = junction
        self.lanes = None
        self.link = None
        self.plan_view = None
        self.reference_start_xyz = start_xyz
        self.reference_end_xyz = end_xyz
        self._lanelet_to_lane = dict(lanelet_to_lane)

    def get_lanelet_to_lane_mapping(self) -> Dict[int, int]:
        return dict(self._lanelet_to_lane)


class _LaneletStub:
    def __init__(self, lanelet_id: int) -> None:
        self.id = lanelet_id


class _LaneletLayerStub:
    def __init__(self, lanelet_ids: List[int]) -> None:
        self._lanelets = {lid: _LaneletStub(lid) for lid in lanelet_ids}

    def exists(self, lanelet_id: int) -> bool:
        return lanelet_id in self._lanelets

    def get(self, lanelet_id: int) -> _LaneletStub:
        return self._lanelets[lanelet_id]


class _LaneletMapStub:
    def __init__(self, lanelet_ids: List[int]) -> None:
        self.laneletLayer = _LaneletLayerStub(lanelet_ids)


class _RoutingGraphStub:
    def __init__(self, following: Dict[int, List[int]]) -> None:
        self._following = following

    def following(self, lanelet: _LaneletStub) -> List[_LaneletStub]:
        return [_LaneletStub(lid) for lid in self._following.get(lanelet.id, [])]


def _synthetic_connector(
    road_id: int,
    junction_id: int,
    incoming_road_id: int,
    outgoing_road_id: int,
    from_lane: int,
    to_lane: int,
) -> Road:
    """A connector exactly as the divergence/completion passes emit it."""
    return _make_zero_length_connecting_road(
        road_id=road_id,
        junction_id=junction_id,
        incoming_road_id=incoming_road_id,
        outgoing_road_id=outgoing_road_id,
        incoming_contact=ContactPoint.END,
        outgoing_contact=ContactPoint.START,
        start_xyz=(0.0, 0.0, 0.0),
        end_xyz=(0.0, 0.0, 0.0),
        min_segment_length=0.01,
        traffic_rule=TrafficRule.RHT,
        from_lane=from_lane,
        to_lane=to_lane,
    )


def _emitted_edges(roads: List[Road]) -> Set[Tuple[int, int, int, int]]:
    """``(incoming road, from lane, outgoing road, to lane)`` per connector."""
    edges: Set[Tuple[int, int, int, int]] = set()
    for road in roads:
        assert road.lanes is not None and road.link is not None
        predecessor, successor = road.link.predecessor, road.link.successor
        assert predecessor is not None and successor is not None
        lane_section = road.lanes.lane_sections[0]
        lanes = {**lane_section.left_lanes, **lane_section.right_lanes}
        lane = next(lane for lane in lanes.values() if lane.lane_id != 0)
        assert lane.predecessor is not None and lane.successor is not None
        edges.add(
            (
                predecessor.element_id,
                lane.predecessor.id,
                successor.element_id,
                lane.successor.id,
            )
        )
    return edges


def _connection_for(
    junction: Junction, connecting_road_id: int
) -> Optional[Connection]:
    for connection in junction.connections:
        if connection.connecting_road == connecting_road_id:
            return connection
    return None


def test_mixed_fork_keeps_the_plain_road_branch() -> None:
    """A fork into a junction connector *and* onto a plain road keeps both.

    Lanelet 100 (road 10, lane -1) is followed by connector lanelet 200
    (road 20, inside junction 1000) and by plain lanelet 300 (road 30).
    Junction 1000 owns road 10's road-level ``<successor>``, so the branch
    onto road 30 survives only as a second ``<connection>`` of junction
    1000. Gating coverage on the incoming lane alone treated road 10 lane
    -1 as "already covered" by the connector and dropped that branch.
    """
    road_10 = _RoadStub(10, {100: -1}, end_xyz=(10.0, 0.0, 0.0))
    road_20 = _RoadStub(20, {200: -1}, junction=1000, start_xyz=(10.0, 0.0, 0.0))
    road_30 = _RoadStub(30, {300: -1}, start_xyz=(10.0, 0.0, 0.0))

    junction = Junction(
        id=1000,
        name="j",
        connections=[
            Connection(
                id=0,
                incoming_road=10,
                connecting_road=20,
                contact_point=ContactPoint.START,
                lane_links=[JunctionLaneLink(from_lane=-1, to_lane=-1)],
            )
        ],
    )

    new_roads, next_id = complete_direct_junction_lanelinks(
        lanelet_map=_LaneletMapStub([100, 200, 300]),
        routing_graph=_RoutingGraphStub({100: [200, 300]}),
        all_roads=cast(List[Road], [road_10, road_20, road_30]),
        junctions=[junction],
        lanelet_to_road_id={100: 10, 200: 20, 300: 30},
        starting_road_id=900,
        traffic_rule=TrafficRule.RHT,
        min_segment_length=0.01,
    )

    # The turn_direction branch keeps its own connector; only the plain-road
    # branch is materialised.
    assert _emitted_edges(new_roads) == {(10, -1, 30, -1)}
    assert next_id == 901
    assert new_roads[0].junction == 1000

    added = _connection_for(junction, 900)
    assert added is not None
    assert added.incoming_road == 10
    assert [(ll.from_lane, ll.to_lane) for ll in added.lane_links] == [(-1, -1)]


def test_every_extra_branch_of_a_fork_gets_its_own_connector() -> None:
    """One connector per directed lane edge, never just the first follower."""
    road_10 = _RoadStub(10, {100: -1}, end_xyz=(10.0, 0.0, 0.0))
    road_20 = _RoadStub(20, {200: -1}, junction=1000, start_xyz=(10.0, 0.0, 0.0))
    road_30 = _RoadStub(30, {300: -1}, start_xyz=(10.0, 0.0, 0.0))
    road_31 = _RoadStub(31, {310: -2}, start_xyz=(10.0, 0.0, 0.0))

    junction = Junction(
        id=1000,
        name="j",
        connections=[
            Connection(
                id=0,
                incoming_road=10,
                connecting_road=20,
                contact_point=ContactPoint.START,
                lane_links=[JunctionLaneLink(from_lane=-1, to_lane=-1)],
            )
        ],
    )

    new_roads, _next_id = complete_direct_junction_lanelinks(
        lanelet_map=_LaneletMapStub([100, 200, 300, 310]),
        routing_graph=_RoutingGraphStub({100: [200, 300, 310]}),
        all_roads=cast(List[Road], [road_10, road_20, road_30, road_31]),
        junctions=[junction],
        lanelet_to_road_id={100: 10, 200: 20, 300: 30, 310: 31},
        starting_road_id=900,
        traffic_rule=TrafficRule.RHT,
        min_segment_length=0.01,
    )

    assert _emitted_edges(new_roads) == {(10, -1, 30, -1), (10, -1, 31, -2)}


def test_edge_already_carried_by_another_junction_is_not_duplicated() -> None:
    """Coverage spans every junction, not just the one being completed.

    Road 10 is listed as an incoming road of both junction 1000 and
    synthetic junction 11000, and 11000's connector already realises
    10.-1 -> 30.-1. Only the junction named by road 10's road-level
    ``<successor>`` is reachable, so a second copy of that edge inside
    junction 1000 would be dead weight.
    """
    road_10 = _RoadStub(10, {100: -1}, end_xyz=(10.0, 0.0, 0.0))
    road_20 = _RoadStub(20, {200: -1}, junction=1000, start_xyz=(10.0, 0.0, 0.0))
    road_30 = _RoadStub(30, {300: -1}, start_xyz=(10.0, 0.0, 0.0))
    connector = _synthetic_connector(
        road_id=800,
        junction_id=11000,
        incoming_road_id=10,
        outgoing_road_id=30,
        from_lane=-1,
        to_lane=-1,
    )

    real_junction = Junction(
        id=1000,
        name="real",
        connections=[
            Connection(
                id=0,
                incoming_road=10,
                connecting_road=20,
                contact_point=ContactPoint.START,
                lane_links=[JunctionLaneLink(from_lane=-1, to_lane=-1)],
            )
        ],
    )
    synthetic_junction = Junction(
        id=11000,
        name="divergence_10",
        connections=[
            Connection(
                id=0,
                incoming_road=10,
                connecting_road=800,
                contact_point=ContactPoint.START,
                lane_links=[JunctionLaneLink(from_lane=-1, to_lane=-1)],
            )
        ],
    )

    new_roads, next_id = complete_direct_junction_lanelinks(
        lanelet_map=_LaneletMapStub([100, 200, 300]),
        routing_graph=_RoutingGraphStub({100: [300]}),
        all_roads=cast(List[Road], [road_10, road_20, road_30, connector]),
        junctions=[real_junction, synthetic_junction],
        lanelet_to_road_id={100: 10, 200: 20, 300: 30},
        starting_road_id=900,
        traffic_rule=TrafficRule.RHT,
        min_segment_length=0.01,
    )

    assert new_roads == []
    assert next_id == 900
    assert len(real_junction.connections) == 1
    assert len(synthetic_junction.connections) == 1
