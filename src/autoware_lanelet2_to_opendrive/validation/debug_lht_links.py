#!/usr/bin/env python3
"""Debug script to understand LHT lane link generation."""

from pathlib import Path
import lanelet2
from autoware_lanelet2_extension_python.projection import MGRSProjector
from autoware_lanelet2_to_opendrive.opendrive.road import Road
from autoware_lanelet2_to_opendrive.opendrive.enums import TrafficRule


def debug_road_0_lane_links():
    """Debug Road 0 lane link generation."""
    # Load test map
    test_data_path = Path(__file__).parent.parent / "test" / "data" / "lanelet2_map.osm"
    projector = MGRSProjector(lanelet2.io.Origin(35.23, 139.16))
    lanelet_map = lanelet2.io.load(str(test_data_path), projector)

    # Get lanelets for Road 0 (from the XO DR analysis, we know Road 0 has Lane 1)
    # We need to find which lanelet corresponds to Road 0

    # Build all roads with LHT
    roads = Road.construct_from_lanelet_map(lanelet_map, traffic_rule=TrafficRule.LHT)

    print(f"Total roads constructed: {len(roads)}")

    # Examine Road 0
    road_0 = roads[0]
    print(f"\n{'=' * 80}")
    print("ROAD 0 DEBUG")
    print(f"{'=' * 80}")
    print(f"Road ID: {road_0.id}")
    print(f"Rule: {road_0.rule}")
    print(f"Length: {road_0.length}")

    # Get lanelet mapping
    lanelet_to_lane = road_0.get_lanelet_to_lane_mapping()
    print("\nLanelet to Lane mapping:")
    for lanelet_id, lane_id in lanelet_to_lane.items():
        print(f"  Lanelet {lanelet_id} -> Lane {lane_id}")

    # Get lane sections
    if road_0.lanes and road_0.lanes.lane_sections:
        print(f"\nLane sections: {len(road_0.lanes.lane_sections)}")

        for i, section in enumerate(road_0.lanes.lane_sections):
            print(f"\n  Section {i} (s={section.s}):")
            print(f"    Left lanes: {len(section.left_lanes)}")
            print(f"    Right lanes: {len(section.right_lanes)}")

            # Check left lanes
            for lane_id, lane in section.left_lanes.items():
                print(f"\n    Lane {lane_id}:")
                print(f"      Lanelet ID: {lane.lanelet_id}")
                print(f"      Type: {lane.lane_type}")

                if lane.predecessor:
                    print(f"      Predecessor: {lane.predecessor.id}")
                else:
                    print("      Predecessor: None")

                if lane.successor:
                    print(f"      Successor: {lane.successor.id}")
                else:
                    print("      Successor: None")

                # Get the lanelet and check routing
                if lane.lanelet_id:
                    try:
                        lanelet = lanelet_map.laneletLayer.get(lane.lanelet_id)
                        print(f"      Lanelet found: {lanelet.id}")

                        # Create routing graph
                        from autoware_lanelet2_to_opendrive.routing_graph import (
                            create_routing_graph,
                        )

                        routing_graph = create_routing_graph(
                            lanelet_map, TrafficRule.LHT.value
                        )

                        # Check predecessors
                        prev_lanelets = routing_graph.previous(lanelet)
                        print(
                            f"      Routing predecessors: {[ll.id for ll in prev_lanelets]}"
                        )

                        # Check successors
                        succ_lanelets = routing_graph.following(lanelet)
                        print(
                            f"      Routing successors: {[ll.id for ll in succ_lanelets]}"
                        )

                    except Exception as e:
                        print(f"      Error accessing lanelet: {e}")

    # Now check links before and after set_lane_links
    print(f"\n{'=' * 80}")
    print("BEFORE set_all_lane_links")
    print(f"{'=' * 80}")

    if road_0.lanes and road_0.lanes.lane_sections:
        for section in road_0.lanes.lane_sections:
            for lane_id, lane in section.left_lanes.items():
                pred_str = lane.predecessor.id if lane.predecessor else "None"
                succ_str = lane.successor.id if lane.successor else "None"
                print(f"Lane {lane_id}: pred={pred_str}, succ={succ_str}")

    # Set all lane links
    from autoware_lanelet2_to_opendrive.routing_graph import create_routing_graph

    routing_graph = create_routing_graph(lanelet_map, TrafficRule.LHT.value)

    print("\nCalling set_all_lane_links...")
    Road.set_all_lane_links(roads, lanelet_map, routing_graph=routing_graph)

    print(f"\n{'=' * 80}")
    print("AFTER set_all_lane_links")
    print(f"{'=' * 80}")

    if road_0.lanes and road_0.lanes.lane_sections:
        for section in road_0.lanes.lane_sections:
            for lane_id, lane in section.left_lanes.items():
                pred_str = lane.predecessor.id if lane.predecessor else "None"
                succ_str = lane.successor.id if lane.successor else "None"
                print(f"Lane {lane_id}: pred={pred_str}, succ={succ_str}")

                # Check if this is a self-reference
                if lane.predecessor and lane.predecessor.id == lane_id:
                    print(f"  ❌ ERROR: Lane {lane_id} has self-reference predecessor!")

                if lane.successor and lane.successor.id == lane_id:
                    print(f"  ❌ ERROR: Lane {lane_id} has self-reference successor!")


if __name__ == "__main__":
    debug_road_0_lane_links()
