"""Test for lane link self-reference bug."""

from pathlib import Path
import lanelet2
from lanelet2.routing import RoutingGraph, RoutingCostDistance
from autoware_lanelet2_extension_python.projection import MGRSProjector
from autoware_lanelet2_to_opendrive.opendrive.road import Road
from autoware_lanelet2_to_opendrive.opendrive.enums import TrafficRule
from autoware_lanelet2_to_opendrive.util import create_routing_graph


def load_test_map():
    """Load the test lanelet2 map."""
    test_data_path = Path(__file__).parent / "data" / "lanelet2_map.osm"
    projector = MGRSProjector(lanelet2.io.Origin(35.23, 139.16))
    return lanelet2.io.load(str(test_data_path), projector)


def test_no_self_referencing_lane_links_lht():
    """Test that LHT roads don't have self-referencing lane links."""
    lanelet_map = load_test_map()

    # Build all roads with LHT
    roads = Road.construct_from_lanelet_map(lanelet_map, traffic_rule=TrafficRule.LHT)

    # Set lane links
    traffic_rules = lanelet2.traffic_rules.create(
        lanelet2.traffic_rules.Locations.Germany,
        lanelet2.traffic_rules.Participants.Vehicle,
    )
    routing_graph = RoutingGraph(lanelet_map, traffic_rules, [RoutingCostDistance(0.0)])
    Road.set_all_lane_links(roads, lanelet_map, routing_graph=routing_graph)

    # Check for self-references
    self_references = []
    for road in roads:
        if road.lanes and road.lanes.lane_sections:
            for section in road.lanes.lane_sections:
                for lane_id, lane in section.left_lanes.items():
                    # Check predecessor
                    if lane.predecessor and lane.predecessor.id == lane_id:
                        self_references.append(
                            f"Road {road.id}, Lane {lane_id}: "
                            f"predecessor self-reference"
                        )
                    # Check successor
                    if lane.successor and lane.successor.id == lane_id:
                        self_references.append(
                            f"Road {road.id}, Lane {lane_id}: "
                            f"successor self-reference"
                        )

                for lane_id, lane in section.right_lanes.items():
                    # Check predecessor
                    if lane.predecessor and lane.predecessor.id == lane_id:
                        self_references.append(
                            f"Road {road.id}, Lane {lane_id}: "
                            f"predecessor self-reference"
                        )
                    # Check successor
                    if lane.successor and lane.successor.id == lane_id:
                        self_references.append(
                            f"Road {road.id}, Lane {lane_id}: "
                            f"successor self-reference"
                        )

    # Report self-references
    if self_references:
        print("\n❌ Self-references found:")
        for ref in self_references[:10]:  # Show first 10
            print(f"  {ref}")
        if len(self_references) > 10:
            print(f"  ... and {len(self_references) - 10} more")

    assert len(self_references) == 0, (
        f"Found {len(self_references)} self-referencing lane links. "
        f"First: {self_references[0] if self_references else 'None'}"
    )


def test_no_self_referencing_lane_links_rht():
    """Test that RHT roads don't have self-referencing lane links."""
    lanelet_map = load_test_map()

    # Build all roads with RHT
    roads = Road.construct_from_lanelet_map(lanelet_map, traffic_rule=TrafficRule.RHT)

    # Set lane links
    traffic_rules = lanelet2.traffic_rules.create(
        lanelet2.traffic_rules.Locations.Germany,
        lanelet2.traffic_rules.Participants.Vehicle,
    )
    routing_graph = RoutingGraph(lanelet_map, traffic_rules, [RoutingCostDistance(0.0)])
    Road.set_all_lane_links(roads, lanelet_map, routing_graph=routing_graph)

    # Check for self-references
    self_references = []
    for road in roads:
        if road.lanes and road.lanes.lane_sections:
            for section in road.lanes.lane_sections:
                for lane_id, lane in section.left_lanes.items():
                    # Check predecessor
                    if lane.predecessor and lane.predecessor.id == lane_id:
                        self_references.append(
                            f"Road {road.id}, Lane {lane_id}: "
                            f"predecessor self-reference"
                        )
                    # Check successor
                    if lane.successor and lane.successor.id == lane_id:
                        self_references.append(
                            f"Road {road.id}, Lane {lane_id}: "
                            f"successor self-reference"
                        )

                for lane_id, lane in section.right_lanes.items():
                    # Check predecessor
                    if lane.predecessor and lane.predecessor.id == lane_id:
                        self_references.append(
                            f"Road {road.id}, Lane {lane_id}: "
                            f"predecessor self-reference"
                        )
                    # Check successor
                    if lane.successor and lane.successor.id == lane_id:
                        self_references.append(
                            f"Road {road.id}, Lane {lane_id}: "
                            f"successor self-reference"
                        )

    # Report self-references
    if self_references:
        print("\n❌ Self-references found:")
        for ref in self_references[:10]:  # Show first 10
            print(f"  {ref}")
        if len(self_references) > 10:
            print(f"  ... and {len(self_references) - 10} more")

    assert len(self_references) == 0, (
        f"Found {len(self_references)} self-referencing lane links. "
        f"First: {self_references[0] if self_references else 'None'}"
    )


def test_debug_road_0_lane_links():
    """Debug Road 0 lane links in detail."""
    lanelet_map = load_test_map()

    # Build all roads with LHT
    roads = Road.construct_from_lanelet_map(lanelet_map, traffic_rule=TrafficRule.LHT)

    # Get Road 0
    road_0 = roads[0]
    assert road_0.id == 0

    # Get lanelet mapping for Road 0
    lanelet_to_lane = road_0.get_lanelet_to_lane_mapping()
    print("\nRoad 0 lanelet-to-lane mapping:")
    for lanelet_id, lane_id in lanelet_to_lane.items():
        print(f"  Lanelet {lanelet_id} → Lane {lane_id}")

    # Set lane links
    routing_graph = create_routing_graph(lanelet_map, TrafficRule.LHT.value)

    # Build global mapping
    lanelet_to_road_and_lane = {}
    for road in roads:
        mapping = road.get_lanelet_to_lane_mapping()
        for lanelet_id, lane_id in mapping.items():
            lanelet_to_road_and_lane[lanelet_id] = (road.id, lane_id)
            print(
                f"Global mapping: Lanelet {lanelet_id} → "
                f"Road {road.id}, Lane {lane_id}"
            )
            if road.id <= 1:  # Only print first 2 roads
                pass
            else:
                break
        if road.id > 1:
            break

    # Check Lane 1 in Road 0
    if road_0.lanes and road_0.lanes.lane_sections:
        section = road_0.lanes.lane_sections[0]
        if 1 in section.left_lanes:
            lane_1 = section.left_lanes[1]
            print("\nRoad 0, Lane 1:")
            print(f"  lane.lane_id: {lane_1.lane_id}")
            print(f"  lane.lanelet_id: {lane_1.lanelet_id}")

            # Get lanelet
            if lane_1.lanelet_id:
                lanelet = lanelet_map.laneletLayer.get(lane_1.lanelet_id)
                print(f"  Lanelet ID: {lanelet.id}")

                # Check routing
                prev_lanelets = routing_graph.previous(lanelet)
                print(f"  Previous lanelets: {[ll.id for ll in prev_lanelets]}")

                for prev_ll in prev_lanelets:
                    if prev_ll.id in lanelet_to_road_and_lane:
                        pred_road_id, pred_lane_id = lanelet_to_road_and_lane[
                            prev_ll.id
                        ]
                        print(
                            f"    Lanelet {prev_ll.id} → "
                            f"Road {pred_road_id}, Lane {pred_lane_id}"
                        )

                        # Check if this would be a self-reference
                        if pred_road_id == road_0.id and pred_lane_id == lane_1.lane_id:
                            print(
                                f"    ❌ SELF-REFERENCE DETECTED: "
                                f"Road {road_0.id}, Lane {lane_1.lane_id} → "
                                f"Road {pred_road_id}, Lane {pred_lane_id}"
                            )

    # Now set links
    Road.set_all_lane_links(roads, lanelet_map, routing_graph=routing_graph)

    # Check Road 0, Lane 1 after setting links
    if road_0.lanes and road_0.lanes.lane_sections:
        section = road_0.lanes.lane_sections[0]
        if 1 in section.left_lanes:
            lane_1 = section.left_lanes[1]
            print("\nAfter set_all_lane_links:")
            print(
                f"  Predecessor: {lane_1.predecessor.id if lane_1.predecessor else 'None'}"
            )
            print(f"  Successor: {lane_1.successor.id if lane_1.successor else 'None'}")
