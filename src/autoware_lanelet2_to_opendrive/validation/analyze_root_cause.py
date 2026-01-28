#!/usr/bin/env python3
"""Analyze root cause of lane link self-reference bug."""

import argparse
from pathlib import Path
from typing import Dict, Set
import lanelet2
from lanelet2.routing import RoutingGraph, RoutingCostDistance
from autoware_lanelet2_extension_python.projection import MGRSProjector


def analyze_lanelet_connections(
    lanelet_map: lanelet2.core.LaneletMap,
    routing_graph: RoutingGraph,
    sample_size: int = 5,
):
    """Analyze lanelet connections to find self-references."""
    print("\n" + "=" * 80)
    print("ANALYZING LANELET CONNECTIONS")
    print("=" * 80 + "\n")

    self_references = []
    analyzed = 0

    for lanelet in lanelet_map.laneletLayer:
        if analyzed >= sample_size:
            break

        # Get predecessors
        previous = routing_graph.previous(lanelet)
        following = routing_graph.following(lanelet)

        # Check for self-references
        is_self_ref_prev = any(ll.id == lanelet.id for ll in previous)
        is_self_ref_follow = any(ll.id == lanelet.id for ll in following)

        if is_self_ref_prev or is_self_ref_follow:
            self_references.append(lanelet.id)
            print("⚠️  SELF-REFERENCE FOUND:")
            print(f"   Lanelet ID: {lanelet.id}")
            print(f"   Previous: {[ll.id for ll in previous]}")
            print(f"   Following: {[ll.id for ll in following]}")
            if is_self_ref_prev:
                print("   ❌ Contains self in previous!")
            if is_self_ref_follow:
                print("   ❌ Contains self in following!")
            print()
        else:
            # Sample some normal cases for comparison
            if analyzed < 3:
                print("✓  Normal connection:")
                print(f"   Lanelet ID: {lanelet.id}")
                print(f"   Previous: {[ll.id for ll in previous]}")
                print(f"   Following: {[ll.id for ll in following]}")
                print()

        analyzed += 1

    print(f"\nSummary: Found {len(self_references)} self-referencing lanelets")
    if self_references:
        print(f"Self-referencing lanelet IDs: {self_references}")

    return self_references


def analyze_lanelet_to_lane_mapping(
    lanelet_map: lanelet2.core.LaneletMap,
    traffic_rule: str = "LHT",
):
    """Analyze how lanelets are mapped to lanes."""
    from autoware_lanelet2_to_opendrive.opendrive.road import Road
    from autoware_lanelet2_to_opendrive.opendrive.enums import TrafficRule

    print("\n" + "=" * 80)
    print("ANALYZING LANELET-TO-LANE MAPPING")
    print("=" * 80 + "\n")

    # Build roads
    traffic_rule_enum = TrafficRule[traffic_rule.upper()]
    roads = Road.construct_from_lanelet_map(lanelet_map, traffic_rule=traffic_rule_enum)

    print(f"Constructed {len(roads)} roads\n")

    # Build global mapping
    lanelet_to_road_and_lane: Dict[int, tuple[int, int]] = {}
    road_to_lanelets: Dict[int, Set[int]] = {}

    for road in roads[:10]:  # Analyze first 10 roads
        mapping = road.get_lanelet_to_lane_mapping()
        road_to_lanelets[road.id] = set()

        for lanelet_id, lane_id in mapping.items():
            lanelet_to_road_and_lane[lanelet_id] = (road.id, lane_id)
            road_to_lanelets[road.id].add(lanelet_id)

            # Print mapping for first few roads
            if road.id < 3:
                print(f"Road {road.id}, Lane {lane_id}: " f"Lanelet {lanelet_id}")

    print(f"\nTotal lanelets mapped: {len(lanelet_to_road_and_lane)}")

    # Check for duplicate lanelet IDs
    lanelet_counts: Dict[int, int] = {}
    for road in roads:
        mapping = road.get_lanelet_to_lane_mapping()
        for lanelet_id in mapping.keys():
            lanelet_counts[lanelet_id] = lanelet_counts.get(lanelet_id, 0) + 1

    duplicates = {lid: count for lid, count in lanelet_counts.items() if count > 1}
    if duplicates:
        print("\n⚠️  DUPLICATE LANELET MAPPINGS FOUND:")
        for lanelet_id, count in list(duplicates.items())[:5]:
            print(f"   Lanelet {lanelet_id} mapped {count} times")
    else:
        print("\n✓  No duplicate lanelet mappings")

    return lanelet_to_road_and_lane, roads


def analyze_specific_case(
    lanelet_map: lanelet2.core.LaneletMap,
    routing_graph: RoutingGraph,
    roads: list,
    lanelet_to_road_and_lane: Dict[int, tuple[int, int]],
):
    """Analyze a specific case where self-reference occurs."""
    print("\n" + "=" * 80)
    print("ANALYZING SPECIFIC CASE: Road 0")
    print("=" * 80 + "\n")

    # Get Road 0
    road_0 = None
    for road in roads:
        if road.id == 0:
            road_0 = road
            break

    if road_0 is None:
        print("❌ Road 0 not found")
        return

    print("Road 0 details:")
    print(f"  Rule: {road_0.rule}")
    print(f"  Length: {road_0.length}")

    # Get lanelet mapping for Road 0
    mapping = road_0.get_lanelet_to_lane_mapping()
    print("\nRoad 0 lanelet-to-lane mapping:")
    for lanelet_id, lane_id in mapping.items():
        print(f"  Lanelet {lanelet_id} → Lane {lane_id}")

        # Get the lanelet
        try:
            lanelet = lanelet_map.laneletLayer.get(lanelet_id)

            # Check routing
            previous = routing_graph.previous(lanelet)
            following = routing_graph.following(lanelet)

            print(f"    Routing for Lanelet {lanelet_id}:")
            print(f"      Previous lanelets: {[ll.id for ll in previous]}")
            print(f"      Following lanelets: {[ll.id for ll in following]}")

            # Check if previous/following are in the mapping
            for prev_ll in previous:
                if prev_ll.id in lanelet_to_road_and_lane:
                    pred_road, pred_lane = lanelet_to_road_and_lane[prev_ll.id]
                    print(
                        f"      Previous Lanelet {prev_ll.id} → "
                        f"Road {pred_road}, Lane {pred_lane}"
                    )

                    # CHECK: Is this a self-reference?
                    if pred_road == road_0.id and pred_lane == lane_id:
                        print(
                            f"        ❌ SELF-REFERENCE: "
                            f"Road {road_0.id}, Lane {lane_id} → "
                            f"Road {pred_road}, Lane {pred_lane}"
                        )
                        print(
                            f"        Root cause: Lanelet {lanelet_id} has "
                            f"Lanelet {prev_ll.id} as previous, and both map "
                            f"to the same lane!"
                        )

            for next_ll in following:
                if next_ll.id in lanelet_to_road_and_lane:
                    succ_road, succ_lane = lanelet_to_road_and_lane[next_ll.id]
                    print(
                        f"      Following Lanelet {next_ll.id} → "
                        f"Road {succ_road}, Lane {succ_lane}"
                    )

                    # CHECK: Is this a self-reference?
                    if succ_road == road_0.id and succ_lane == lane_id:
                        print(
                            f"        ❌ SELF-REFERENCE: "
                            f"Road {road_0.id}, Lane {lane_id} → "
                            f"Road {succ_road}, Lane {succ_lane}"
                        )
                        print(
                            f"        Root cause: Lanelet {lanelet_id} has "
                            f"Lanelet {next_ll.id} as following, and both map "
                            f"to the same lane!"
                        )

        except Exception as e:
            print(f"    Error: {e}")

        print()


def analyze_lane_section_structure(roads: list):
    """Analyze lane section structure."""
    print("\n" + "=" * 80)
    print("ANALYZING LANE SECTION STRUCTURE")
    print("=" * 80 + "\n")

    for road in roads[:5]:  # First 5 roads
        if road.lanes and road.lanes.lane_sections:
            print(f"Road {road.id}:")
            print(f"  Lane sections: {len(road.lanes.lane_sections)}")

            for i, section in enumerate(road.lanes.lane_sections):
                print(f"\n  Section {i} (s={section.s}):")
                print(f"    Left lanes: {len(section.left_lanes)}")
                print(f"    Right lanes: {len(section.right_lanes)}")

                # Check if multiple lane sections share lanelets
                if len(road.lanes.lane_sections) > 1:
                    print("    ⚠️  Multiple lane sections in one road!")
                    print("       This might cause mapping issues")

                # Show lane IDs and lanelet IDs
                for lane_id, lane in list(section.left_lanes.items())[:2]:
                    print(f"    Lane {lane_id}: lanelet_id={lane.lanelet_id}")

                for lane_id, lane in list(section.right_lanes.items())[:2]:
                    print(f"    Lane {lane_id}: lanelet_id={lane.lanelet_id}")

            print()


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Analyze root cause of lane link self-reference bug"
    )
    parser.add_argument(
        "--map",
        type=str,
        default="test/data/lanelet2_map.osm",
        help="Path to Lanelet2 map file",
    )
    parser.add_argument(
        "--traffic-rule",
        type=str,
        default="LHT",
        choices=["LHT", "RHT"],
        help="Traffic rule to use",
    )
    parser.add_argument(
        "--sample-size",
        type=int,
        default=10,
        help="Number of lanelets to analyze",
    )

    args = parser.parse_args()

    # Load map
    map_path = Path(args.map)
    if not map_path.exists():
        print(f"Error: Map file not found: {map_path}")
        return

    print(f"Loading map: {map_path}")
    projector = MGRSProjector(lanelet2.io.Origin(35.23, 139.16))
    lanelet_map = lanelet2.io.load(str(map_path), projector)

    # Create routing graph
    print("Creating routing graph...")
    traffic_rules = lanelet2.traffic_rules.create(
        lanelet2.traffic_rules.Locations.Germany,
        lanelet2.traffic_rules.Participants.Vehicle,
    )
    routing_graph = RoutingGraph(lanelet_map, traffic_rules, [RoutingCostDistance(0.0)])

    # Analysis 1: Check lanelet connections
    analyze_lanelet_connections(lanelet_map, routing_graph, args.sample_size)

    # Analysis 2: Check lanelet-to-lane mapping
    lanelet_to_road_and_lane, roads = analyze_lanelet_to_lane_mapping(
        lanelet_map, args.traffic_rule
    )

    # Analysis 3: Analyze specific case
    analyze_specific_case(lanelet_map, routing_graph, roads, lanelet_to_road_and_lane)

    # Analysis 4: Check lane section structure
    analyze_lane_section_structure(roads)

    print("\n" + "=" * 80)
    print("ANALYSIS COMPLETE")
    print("=" * 80)
    print("\nCheck the output above for:")
    print("  1. Self-referencing lanelets in routing graph")
    print("  2. Incorrect lanelet-to-lane mappings")
    print("  3. Multiple lane sections sharing lanelets")
    print("  4. Circular lanelet connections in input map")


if __name__ == "__main__":
    main()
