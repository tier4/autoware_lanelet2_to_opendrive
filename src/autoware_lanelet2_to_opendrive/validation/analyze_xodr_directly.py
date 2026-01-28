#!/usr/bin/env python3
"""Analyze OpenDRIVE file directly to identify root cause of self-references."""

import argparse
import xml.etree.ElementTree as ET
from typing import List, Tuple


def extract_self_references(xodr_path: str) -> List[Tuple[str, str, str, str]]:
    """Find all self-referencing lanes in the OpenDRIVE file."""
    tree = ET.parse(xodr_path)
    root = tree.getroot()

    self_refs = []

    for road in root.findall(".//road"):
        road_id = road.get("id")
        if road_id is None:
            continue

        for lane_section in road.findall(".//laneSection"):
            for side in ["left", "center", "right"]:
                for lane in lane_section.findall(f".//{side}/lane"):
                    lane_id = lane.get("id")
                    if lane_id is None:
                        continue
                    link = lane.find("link")

                    if link is None:
                        continue

                    # Check predecessor
                    pred = link.find("predecessor")
                    if pred is not None:
                        pred_id = pred.get("id")
                        if pred_id is not None and pred_id == lane_id:
                            self_refs.append((road_id, lane_id, "predecessor", pred_id))

                    # Check successor
                    succ = link.find("successor")
                    if succ is not None:
                        succ_id = succ.get("id")
                        if succ_id is not None and succ_id == lane_id:
                            self_refs.append((road_id, lane_id, "successor", succ_id))

    return self_refs


def analyze_road_connections(xodr_path: str, road_id: str):
    """Analyze a specific road's connections in detail."""
    tree = ET.parse(xodr_path)
    root = tree.getroot()

    print(f"\n{'=' * 80}")
    print(f"DETAILED ANALYSIS: Road {road_id}")
    print(f"{'=' * 80}\n")

    road = root.find(f".//road[@id='{road_id}']")
    if road is None:
        print(f"❌ Road {road_id} not found")
        return

    # Basic info
    print(f"Road {road_id}:")
    print(f"  Name: {road.get('name', 'N/A')}")
    print(f"  Junction: {road.get('junction', '-1')}")
    print(f"  Rule: {road.get('rule', 'N/A')}")
    print(f"  Length: {road.get('length', 'N/A')}\n")

    # Road links
    link = road.find("link")
    pred_road_id = None
    succ_road_id = None

    if link is not None:
        print("Road Links:")
        pred = link.find("predecessor")
        if pred is not None:
            pred_road_id = pred.get("elementId")
            print(
                f"  Predecessor: {pred.get('elementType')} {pred_road_id} "
                f"(contact: {pred.get('contactPoint', 'N/A')})"
            )

        succ = link.find("successor")
        if succ is not None:
            succ_road_id = succ.get("elementId")
            print(
                f"  Successor: {succ.get('elementType')} {succ_road_id} "
                f"(contact: {succ.get('contactPoint', 'N/A')})"
            )
        print()

    # Lanes
    for lane_section in road.findall(".//laneSection"):
        s = lane_section.get("s", "0.0")
        print(f"Lane Section (s={s}):")

        for side in ["left", "center", "right"]:
            lanes = lane_section.findall(f".//{side}/lane")
            if lanes:
                print(f"\n  {side.upper()} lanes:")

            for lane in lanes:
                lane_id = lane.get("id")
                lane_type = lane.get("type")
                print(f"    Lane {lane_id} (type={lane_type}):")

                link = lane.find("link")
                if link is not None:
                    pred = link.find("predecessor")
                    succ = link.find("successor")

                    if pred is not None:
                        pred_lane_id = pred.get("id")
                        print(f"      Predecessor: {pred_lane_id}")

                        # Analyze if this is a self-reference
                        if pred_lane_id == lane_id:
                            print("        ❌ SELF-REFERENCE!")
                            print(
                                f"           Lane {lane_id} → predecessor {pred_lane_id}"
                            )
                            print("           This lane links to itself as predecessor")

                            # Check road link
                            if pred_road_id:
                                print(
                                    f"           Road link says predecessor is Road {pred_road_id}"
                                )
                                print(
                                    "           But lane link points to same lane ID!"
                                )

                    if succ is not None:
                        succ_lane_id = succ.get("id")
                        print(f"      Successor: {succ_lane_id}")

                        # Analyze if this is a self-reference
                        if succ_lane_id == lane_id:
                            print("        ❌ SELF-REFERENCE!")
                            print(
                                f"           Lane {lane_id} → successor {succ_lane_id}"
                            )
                            print("           This lane links to itself as successor")

                            # Check road link
                            if succ_road_id:
                                print(
                                    f"           Road link says successor is Road {succ_road_id}"
                                )
                                print(
                                    "           But lane link points to same lane ID!"
                                )

                    if pred is None and succ is None:
                        print("      No lane links")
                else:
                    print("      No link element")

        print()


def find_connected_roads(xodr_path: str, road_id: str):
    """Find roads connected to the given road."""
    tree = ET.parse(xodr_path)
    root = tree.getroot()

    print(f"\n{'=' * 80}")
    print(f"CONNECTED ROADS FOR Road {road_id}")
    print(f"{'=' * 80}\n")

    road = root.find(f".//road[@id='{road_id}']")
    if road is None:
        return

    # Get connected roads
    link = road.find("link")
    connected = []

    if link is not None:
        pred = link.find("predecessor")
        if pred is not None and pred.get("elementType") == "road":
            connected.append(("predecessor", pred.get("elementId")))

        succ = link.find("successor")
        if succ is not None and succ.get("elementType") == "road":
            connected.append(("successor", succ.get("elementId")))

    # Analyze each connected road
    for direction, connected_road_id in connected:
        print(f"\n{direction.upper()}: Road {connected_road_id}")

        connected_road = root.find(f".//road[@id='{connected_road_id}']")
        if connected_road is None:
            print("  ❌ Road not found")
            continue

        print(f"  Name: {connected_road.get('name', 'N/A')}")
        print(f"  Rule: {connected_road.get('rule', 'N/A')}")

        # Get lanes
        for lane_section in connected_road.findall(".//laneSection"):
            for side in ["left", "right"]:
                for lane in lane_section.findall(f".//{side}/lane"):
                    lane_id = lane.get("id")
                    print(f"  Lane {lane_id} ({side})")


def analyze_hypothesis(xodr_path: str):
    """Test hypothesis about why self-references occur."""
    print(f"\n{'=' * 80}")
    print("HYPOTHESIS TESTING")
    print(f"{'=' * 80}\n")

    print("Hypothesis 1: Lane IDs are being set to lanelet IDs")
    print("  - If true: Lane IDs should be very large numbers (lanelet IDs)")
    print("  - If false: Lane IDs should be small (1, 2, 3, -1, -2, -3)\n")

    tree = ET.parse(xodr_path)
    root = tree.getroot()

    lane_ids = []
    for road in root.findall(".//road")[:10]:  # Sample first 10 roads
        for lane in road.findall(".//lane"):
            lane_id = lane.get("id")
            if lane_id and lane_id != "0":  # Exclude center lane
                try:
                    lane_ids.append(int(lane_id))
                except ValueError:
                    pass

    if lane_ids:
        print(f"Sample lane IDs: {lane_ids[:20]}")
        print(f"Min lane ID: {min(lane_ids)}")
        print(f"Max lane ID: {max(lane_ids)}")

        if max(lane_ids) > 100:
            print("\n✓ Hypothesis 1: LIKELY TRUE")
            print("  Lane IDs are large, suggesting they might be lanelet IDs")
        else:
            print("\n❌ Hypothesis 1: FALSE")
            print("  Lane IDs are small, as expected for OpenDRIVE")

    print("\n" + "-" * 80 + "\n")

    print("Hypothesis 2: Same lanelet mapped to multiple lanes in same road")
    print("  - If true: Could cause confusion in lane link generation\n")

    # This would require looking at the actual code, not just the XODR
    print("  → This requires code inspection (see analyze_root_cause.py)")


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Analyze OpenDRIVE file for self-reference root cause"
    )
    parser.add_argument(
        "xodr_file",
        type=str,
        help="Path to OpenDRIVE file",
    )
    parser.add_argument(
        "--road",
        type=str,
        help="Analyze specific road in detail",
    )

    args = parser.parse_args()

    print(f"Analyzing: {args.xodr_file}\n")

    # Find all self-references
    print("=" * 80)
    print("FINDING SELF-REFERENCES")
    print("=" * 80)

    self_refs = extract_self_references(args.xodr_file)

    if self_refs:
        print(f"\n❌ Found {len(self_refs)} self-references:\n")
        for road_id, lane_id, link_type, ref_id in self_refs[:10]:
            print(f"  Road {road_id}, Lane {lane_id}: {link_type} → {ref_id}")

        if len(self_refs) > 10:
            print(f"  ... and {len(self_refs) - 10} more\n")

        # Analyze first self-reference in detail
        first_road = self_refs[0][0]
        if args.road:
            analyze_road_connections(args.xodr_file, args.road)
            find_connected_roads(args.xodr_file, args.road)
        else:
            analyze_road_connections(args.xodr_file, first_road)
            find_connected_roads(args.xodr_file, first_road)

    else:
        print("\n✓ No self-references found!\n")

    # Test hypotheses
    analyze_hypothesis(args.xodr_file)

    print("\n" + "=" * 80)
    print("ANALYSIS COMPLETE")
    print("=" * 80)


if __name__ == "__main__":
    main()
