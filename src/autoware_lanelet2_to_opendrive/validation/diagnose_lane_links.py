#!/usr/bin/env python3
"""Diagnostic script to analyze lane link issues in detail."""

import argparse
import sys
from typing import List, Tuple
import xml.etree.ElementTree as ET


def analyze_specific_road(xodr_path: str, road_id: str):
    """Analyze a specific road in detail."""
    tree = ET.parse(xodr_path)
    root = tree.getroot()

    road = root.find(f".//road[@id='{road_id}']")
    if road is None:
        print(f"Road {road_id} not found!")
        return

    print(f"\n{'=' * 80}")
    print(f"ROAD {road_id} ANALYSIS")
    print(f"{'=' * 80}\n")

    # Basic info
    print(f"Name: {road.get('name', 'N/A')}")
    print(f"Junction: {road.get('junction', '-1')}")
    print(f"Length: {road.get('length', 'N/A')}")
    print(f"Rule: {road.get('rule', 'N/A (defaults to RHT)')}\n")

    # Road links
    link = road.find("link")
    if link is not None:
        print("Road Links:")
        pred = link.find("predecessor")
        if pred is not None:
            print(
                f"  Predecessor: {pred.get('elementType')} {pred.get('elementId')} "
                f"(contact: {pred.get('contactPoint', 'N/A')})"
            )
        succ = link.find("successor")
        if succ is not None:
            print(
                f"  Successor: {succ.get('elementType')} {succ.get('elementId')} "
                f"(contact: {succ.get('contactPoint', 'N/A')})"
            )
        print()

    # Lane sections
    lane_sections = road.findall(".//laneSection")
    print(f"Lane Sections: {len(lane_sections)}\n")

    for i, section in enumerate(lane_sections):
        s = section.get("s", "0.0")
        print(f"  Section {i} (s={s}):")

        # Count lanes by side
        left_lanes = section.findall(".//left/lane")
        center_lanes = section.findall(".//center/lane")
        right_lanes = section.findall(".//right/lane")

        print(f"    Left lanes: {len(left_lanes)}")
        print(f"    Center lanes: {len(center_lanes)}")
        print(f"    Right lanes: {len(right_lanes)}\n")

        # Analyze each lane
        for side, side_name in [
            (left_lanes, "LEFT"),
            (center_lanes, "CENTER"),
            (right_lanes, "RIGHT"),
        ]:
            for lane in side:
                lane_id = lane.get("id")
                lane_type = lane.get("type")
                print(f"    Lane {lane_id} ({side_name}, type={lane_type}):")

                link = lane.find("link")
                if link is not None:
                    pred = link.find("predecessor")
                    succ = link.find("successor")

                    if pred is not None:
                        print(f"      Predecessor: {pred.get('id')}")
                    if succ is not None:
                        print(f"      Successor: {succ.get('id')}")

                    if pred is None and succ is None:
                        print("      No lane links")
                else:
                    print("      No link element")

                print()


def find_broken_links(xodr_path: str) -> List[Tuple[str, str, str, str, str]]:
    """Find lane links that reference non-existent lanes."""
    tree = ET.parse(xodr_path)
    root = tree.getroot()

    # Build map of available lanes per road
    road_lanes = {}
    for road in root.findall(".//road"):
        road_id = road.get("id")
        lanes = set()

        for lane_section in road.findall(".//laneSection"):
            for side in ["left", "center", "right"]:
                for lane in lane_section.findall(f".//{side}/lane"):
                    lane_id = lane.get("id")
                    if lane_id is not None:
                        lanes.add(int(lane_id))

        road_lanes[road_id] = lanes

    # Find broken links
    broken = []

    for road in root.findall(".//road"):
        road_id = road.get("id")
        if road_id is None:
            continue

        # Get road links
        road_link = road.find("link")
        pred_road_id = None
        succ_road_id = None

        if road_link is not None:
            pred = road_link.find("predecessor")
            if pred is not None and pred.get("elementType") == "road":
                pred_road_id = pred.get("elementId")

            succ = road_link.find("successor")
            if succ is not None and succ.get("elementType") == "road":
                succ_road_id = succ.get("elementId")

        # Check lane links
        for lane_section in road.findall(".//laneSection"):
            for side in ["left", "center", "right"]:
                for lane in lane_section.findall(f".//{side}/lane"):
                    lane_id_str = lane.get("id")
                    if lane_id_str is None:
                        continue
                    lane_id = int(lane_id_str)
                    link = lane.find("link")

                    if link is None:
                        continue

                    # Check predecessor
                    pred = link.find("predecessor")
                    if pred is not None:
                        pred_lane_id_str = pred.get("id")
                        if pred_lane_id_str is None:
                            continue
                        pred_lane_id = int(pred_lane_id_str)

                        # Validate against predecessor road
                        if (
                            pred_road_id
                            and pred_road_id in road_lanes
                            and pred_lane_id not in road_lanes[pred_road_id]
                        ):
                            broken.append(
                                (
                                    road_id,
                                    str(lane_id),
                                    pred_road_id,
                                    str(pred_lane_id),
                                    "predecessor",
                                )
                            )

                    # Check successor
                    succ = link.find("successor")
                    if succ is not None:
                        succ_lane_id_str = succ.get("id")
                        if succ_lane_id_str is None:
                            continue
                        succ_lane_id = int(succ_lane_id_str)

                        # Validate against successor road
                        if (
                            succ_road_id
                            and succ_road_id in road_lanes
                            and succ_lane_id not in road_lanes[succ_road_id]
                        ):
                            broken.append(
                                (
                                    road_id,
                                    str(lane_id),
                                    succ_road_id,
                                    str(succ_lane_id),
                                    "successor",
                                )
                            )

    return broken


def check_lht_vs_rht_consistency(xodr_path: str):
    """Check if lane IDs are consistent with traffic rule."""
    tree = ET.parse(xodr_path)
    root = tree.getroot()

    issues = []

    for road in root.findall(".//road"):
        road_id = road.get("id")
        rule = road.get("rule", "RHT")

        for lane_section in road.findall(".//laneSection"):
            s = lane_section.get("s", "0.0")

            # Get lane IDs
            left_lanes = []
            right_lanes = []

            for lane in lane_section.findall(".//left/lane"):
                lane_id = lane.get("id")
                if lane_id is not None:
                    left_lanes.append(int(lane_id))

            for lane in lane_section.findall(".//right/lane"):
                lane_id = lane.get("id")
                if lane_id is not None:
                    right_lanes.append(int(lane_id))

            # Check consistency
            if rule == "RHT":
                # RHT: lanes should be on the right (negative IDs)
                if left_lanes:
                    issues.append(
                        f"Road {road_id} (s={s}): RHT traffic but has LEFT lanes "
                        f"(positive IDs): {left_lanes}"
                    )
                if right_lanes and any(id >= 0 for id in right_lanes):
                    issues.append(
                        f"Road {road_id} (s={s}): RHT traffic but has positive "
                        f"lane IDs on right: {right_lanes}"
                    )

            elif rule == "LHT":
                # LHT: lanes should be on the left (positive IDs)
                if right_lanes:
                    issues.append(
                        f"Road {road_id} (s={s}): LHT traffic but has RIGHT lanes "
                        f"(negative IDs): {right_lanes}"
                    )
                if left_lanes and any(id <= 0 for id in left_lanes):
                    issues.append(
                        f"Road {road_id} (s={s}): LHT traffic but has non-positive "
                        f"lane IDs on left: {left_lanes}"
                    )

    return issues


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Diagnose lane link issues in OpenDRIVE files"
    )
    parser.add_argument("xodr_file", type=str, help="Path to OpenDRIVE file")
    parser.add_argument(
        "--road", type=str, help="Analyze a specific road in detail", metavar="ROAD_ID"
    )
    parser.add_argument("--broken", action="store_true", help="Find broken lane links")
    parser.add_argument(
        "--rule-check",
        action="store_true",
        help="Check LHT/RHT consistency",
    )

    args = parser.parse_args()

    if args.road:
        analyze_specific_road(args.xodr_file, args.road)

    if args.broken:
        print("\n=== Finding Broken Lane Links ===\n")
        broken = find_broken_links(args.xodr_file)
        if broken:
            print(f"Found {len(broken)} broken lane links:\n")
            for road_id, lane_id, target_road, target_lane, link_type in broken:
                print(
                    f"  Road {road_id}, Lane {lane_id} -> {link_type} "
                    f"Road {target_road}, Lane {target_lane} (DOES NOT EXIST)"
                )
        else:
            print("✓ No broken lane links found!")

    if args.rule_check:
        print("\n=== Checking LHT/RHT Consistency ===\n")
        issues = check_lht_vs_rht_consistency(args.xodr_file)
        if issues:
            print(f"Found {len(issues)} consistency issues:\n")
            for issue in issues:
                print(f"  {issue}")
        else:
            print("✓ All roads are consistent with their traffic rules!")

    if not args.road and not args.broken and not args.rule_check:
        print("Please specify --road ROAD_ID, --broken, or --rule-check")
        sys.exit(1)


if __name__ == "__main__":
    main()
