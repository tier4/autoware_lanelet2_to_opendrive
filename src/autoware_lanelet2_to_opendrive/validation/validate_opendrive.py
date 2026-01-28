#!/usr/bin/env python3
"""OpenDRIVE validation script to detect lane link inconsistencies."""

import argparse
import sys
from pathlib import Path
from typing import Dict, List, Optional
import xml.etree.ElementTree as ET
from collections import defaultdict


class OpenDriveValidator:
    """Validator for OpenDRIVE files to detect lane link issues."""

    def __init__(self, xodr_path: str):
        """Initialize validator with OpenDRIVE file path."""
        self.xodr_path = Path(xodr_path)
        self.tree: Optional[ET.ElementTree] = None
        self.root: Optional[ET.Element] = None
        self.errors: List[str] = []
        self.warnings: List[str] = []

    def load(self) -> bool:
        """Load and parse OpenDRIVE file."""
        try:
            self.tree = ET.parse(self.xodr_path)
            self.root = self.tree.getroot()
            print(f"✓ Loaded OpenDRIVE file: {self.xodr_path}")
            return True
        except ET.ParseError as e:
            self.errors.append(f"XML parse error: {e}")
            return False
        except FileNotFoundError:
            self.errors.append(f"File not found: {self.xodr_path}")
            return False

    def get_road_info(self) -> Dict[str, Dict]:
        """Extract road information including available lanes."""
        assert self.root is not None, "Must call load() before get_road_info()"
        roads = {}
        for road_elem in self.root.findall(".//road"):
            road_id = road_elem.get("id")
            if road_id is None:
                continue
            junction = road_elem.get("junction", "-1")

            # Get lanes for this road
            lanes_dict = defaultdict(set)  # section_s -> set of lane_ids
            for lane_section in road_elem.findall(".//laneSection"):
                s = lane_section.get("s", "0.0")

                # Get all lane IDs in this section
                for side in ["left", "center", "right"]:
                    for lane in lane_section.findall(f".//{side}/lane"):
                        lane_id_str = lane.get("id")
                        if lane_id_str is not None:
                            lanes_dict[s].add(int(lane_id_str))

            roads[road_id] = {
                "junction": junction,
                "lane_sections": dict(lanes_dict),
            }

        return roads

    def validate_lane_links_within_road(self) -> int:
        """Validate lane links within each road (predecessor/successor)."""
        assert (
            self.root is not None
        ), "Must call load() before validate_lane_links_within_road()"
        error_count = 0

        for road_elem in self.root.findall(".//road"):
            road_id = road_elem.get("id")

            lane_sections = road_elem.findall(".//laneSection")
            for i, lane_section in enumerate(lane_sections):
                s = lane_section.get("s", "0.0")

                # Get available lanes in current section
                current_lanes = set()
                for side in ["left", "center", "right"]:
                    for lane in lane_section.findall(f".//{side}/lane"):
                        lane_id_str = lane.get("id")
                        if lane_id_str is not None:
                            current_lanes.add(int(lane_id_str))

                # Get available lanes in next section (if exists)
                next_lanes = set()
                if i + 1 < len(lane_sections):
                    next_section = lane_sections[i + 1]
                    for side in ["left", "center", "right"]:
                        for lane in next_section.findall(f".//{side}/lane"):
                            lane_id_str = lane.get("id")
                            if lane_id_str is not None:
                                next_lanes.add(int(lane_id_str))

                # Get available lanes in previous section (if exists)
                prev_lanes = set()
                if i > 0:
                    prev_section = lane_sections[i - 1]
                    for side in ["left", "center", "right"]:
                        for lane in prev_section.findall(f".//{side}/lane"):
                            lane_id_str = lane.get("id")
                            if lane_id_str is not None:
                                prev_lanes.add(int(lane_id_str))

                # Validate each lane's links
                for side in ["left", "center", "right"]:
                    for lane in lane_section.findall(f".//{side}/lane"):
                        lane_id_str = lane.get("id")
                        if lane_id_str is None:
                            continue
                        lane_id = int(lane_id_str)
                        link = lane.find("link")

                        if link is not None:
                            # Check predecessor
                            pred = link.find("predecessor")
                            if pred is not None:
                                pred_id_str = pred.get("id")
                                if pred_id_str is None:
                                    continue
                                pred_id = int(pred_id_str)

                                # For multi-section roads, predecessor should be in previous section
                                if i > 0 and pred_id not in prev_lanes:
                                    error_msg = (
                                        f"Road {road_id}, Lane {lane_id} (s={s}): "
                                        f"predecessor {pred_id} not in previous section. "
                                        f"Available: {sorted(prev_lanes)}"
                                    )
                                    self.errors.append(error_msg)
                                    error_count += 1

                            # Check successor
                            succ = link.find("successor")
                            if succ is not None:
                                succ_id_str = succ.get("id")
                                if succ_id_str is None:
                                    continue
                                succ_id = int(succ_id_str)

                                # For multi-section roads, successor should be in next section
                                if (
                                    i + 1 < len(lane_sections)
                                    and succ_id not in next_lanes
                                ):
                                    error_msg = (
                                        f"Road {road_id}, Lane {lane_id} (s={s}): "
                                        f"successor {succ_id} not in next section. "
                                        f"Available: {sorted(next_lanes)}"
                                    )
                                    self.errors.append(error_msg)
                                    error_count += 1

        return error_count

    def validate_lane_links_between_roads(self, roads_info: Dict) -> int:
        """Validate lane links between connected roads."""
        assert (
            self.root is not None
        ), "Must call load() before validate_lane_links_between_roads()"
        error_count = 0

        for road_elem in self.root.findall(".//road"):
            road_id = road_elem.get("id")

            # Get road links
            road_link = road_elem.find("link")
            if road_link is None:
                continue

            pred_road = road_link.find("predecessor")
            succ_road = road_link.find("successor")

            # Get lanes in first and last sections
            lane_sections = road_elem.findall(".//laneSection")
            if not lane_sections:
                continue

            first_section = lane_sections[0]
            last_section = lane_sections[-1]

            # Get lanes in first section (for predecessor check)
            first_section_lanes = {}
            for side in ["left", "center", "right"]:
                for lane in first_section.findall(f".//{side}/lane"):
                    lane_id_str = lane.get("id")
                    if lane_id_str is not None:
                        lane_id = int(lane_id_str)
                        first_section_lanes[lane_id] = lane

            # Get lanes in last section (for successor check)
            last_section_lanes = {}
            for side in ["left", "center", "right"]:
                for lane in last_section.findall(f".//{side}/lane"):
                    lane_id_str = lane.get("id")
                    if lane_id_str is not None:
                        lane_id = int(lane_id_str)
                        last_section_lanes[lane_id] = lane

            # Validate predecessor road connections
            if pred_road is not None:
                pred_road_id = pred_road.get("elementId")
                pred_road_type = pred_road.get("elementType")

                # Skip junction predecessors (they don't have lanes)
                if pred_road_type == "road" and pred_road_id in roads_info:
                    pred_info = roads_info[pred_road_id]

                    # Get last section of predecessor road (connects to our first section)
                    pred_last_section_lanes = set()
                    if pred_info["lane_sections"]:
                        last_s = max(pred_info["lane_sections"].keys())
                        pred_last_section_lanes = pred_info["lane_sections"][last_s]

                    # Check each lane's predecessor
                    for lane_id, lane in first_section_lanes.items():
                        link = lane.find("link")
                        if link is not None:
                            pred_lane = link.find("predecessor")
                            if pred_lane is not None:
                                pred_lane_id_str = pred_lane.get("id")
                                if pred_lane_id_str is None:
                                    continue
                                pred_lane_id = int(pred_lane_id_str)

                                if pred_lane_id not in pred_last_section_lanes:
                                    error_msg = (
                                        f"Road {road_id}, Lane {lane_id}: "
                                        f"predecessor lane {pred_lane_id} does not exist in "
                                        f"predecessor road {pred_road_id}. "
                                        f"Available: {sorted(pred_last_section_lanes)}"
                                    )
                                    self.errors.append(error_msg)
                                    error_count += 1

            # Validate successor road connections
            if succ_road is not None:
                succ_road_id = succ_road.get("elementId")
                succ_road_type = succ_road.get("elementType")

                # Skip junction successors (they don't have lanes)
                if succ_road_type == "road" and succ_road_id in roads_info:
                    succ_info = roads_info[succ_road_id]

                    # Get first section of successor road (connects to our last section)
                    succ_first_section_lanes = set()
                    if succ_info["lane_sections"]:
                        first_s = min(succ_info["lane_sections"].keys())
                        succ_first_section_lanes = succ_info["lane_sections"][first_s]

                    # Check each lane's successor
                    for lane_id, lane in last_section_lanes.items():
                        link = lane.find("link")
                        if link is not None:
                            succ_lane = link.find("successor")
                            if succ_lane is not None:
                                succ_lane_id_str = succ_lane.get("id")
                                if succ_lane_id_str is None:
                                    continue
                                succ_lane_id = int(succ_lane_id_str)

                                if succ_lane_id not in succ_first_section_lanes:
                                    error_msg = (
                                        f"Road {road_id}, Lane {lane_id}: "
                                        f"successor lane {succ_lane_id} does not exist in "
                                        f"successor road {succ_road_id}. "
                                        f"Available: {sorted(succ_first_section_lanes)}"
                                    )
                                    self.errors.append(error_msg)
                                    error_count += 1

        return error_count

    def validate_bidirectional_links(self) -> int:
        """Check if lane links are bidirectional where appropriate."""
        assert (
            self.root is not None
        ), "Must call load() before validate_bidirectional_links()"
        warning_count = 0

        # Build mapping of lane links: (road_id, lane_id, succ_road_id, succ_lane_id)
        lane_links = []

        for road_elem in self.root.findall(".//road"):
            road_id = road_elem.get("id")

            # Get road link
            road_link = road_elem.find("link")
            succ_road_id = None
            if road_link is not None:
                succ_elem = road_link.find("successor")
                if succ_elem is not None and succ_elem.get("elementType") == "road":
                    succ_road_id = succ_elem.get("elementId")

            if not succ_road_id:
                continue

            # Get last lane section
            lane_sections = road_elem.findall(".//laneSection")
            if not lane_sections:
                continue

            last_section = lane_sections[-1]

            # Get all lanes with successor links
            for side in ["left", "center", "right"]:
                for lane in last_section.findall(f".//{side}/lane"):
                    lane_id_str = lane.get("id")
                    if lane_id_str is None:
                        continue
                    lane_id = int(lane_id_str)
                    link = lane.find("link")

                    if link is not None:
                        succ_lane = link.find("successor")
                        if succ_lane is not None:
                            succ_lane_id_str = succ_lane.get("id")
                            if succ_lane_id_str is None:
                                continue
                            succ_lane_id = int(succ_lane_id_str)
                            lane_links.append(
                                (road_id, lane_id, succ_road_id, succ_lane_id)
                            )

        # Check for reverse links
        for road_id, lane_id, succ_road_id, succ_lane_id in lane_links:
            # Look for reverse link
            reverse_found = False
            for r_id, l_id, s_r_id, s_l_id in lane_links:
                if (
                    r_id == succ_road_id
                    and l_id == succ_lane_id
                    and s_r_id == road_id
                    and s_l_id == lane_id
                ):
                    reverse_found = True
                    break

            if not reverse_found:
                warn_msg = (
                    f"Road {road_id}, Lane {lane_id} → "
                    f"Road {succ_road_id}, Lane {succ_lane_id}: "
                    f"No reverse link found (may be intentional for one-way roads)"
                )
                self.warnings.append(warn_msg)
                warning_count += 1

        return warning_count

    def validate(self) -> bool:
        """Run all validations."""
        if not self.load():
            return False

        print("\n=== Running OpenDRIVE Validation ===\n")

        # Get road information
        roads_info = self.get_road_info()
        print(f"✓ Found {len(roads_info)} roads\n")

        # Validation 1: Lane links within roads
        print("1. Validating lane links within roads...")
        errors = self.validate_lane_links_within_road()
        print(f"   Found {errors} errors\n")

        # Validation 2: Lane links between roads
        print("2. Validating lane links between roads...")
        errors = self.validate_lane_links_between_roads(roads_info)
        print(f"   Found {errors} errors\n")

        # Validation 3: Bidirectional link check
        print("3. Checking bidirectional lane links...")
        warnings = self.validate_bidirectional_links()
        print(f"   Found {warnings} warnings\n")

        return len(self.errors) == 0

    def print_report(self):
        """Print validation report."""
        print("\n" + "=" * 80)
        print("VALIDATION REPORT")
        print("=" * 80)

        if self.errors:
            print(f"\n❌ ERRORS ({len(self.errors)}):")
            for i, error in enumerate(self.errors, 1):
                print(f"  {i}. {error}")

        if self.warnings:
            print(f"\n⚠️  WARNINGS ({len(self.warnings)}):")
            for i, warning in enumerate(self.warnings, 1):
                print(f"  {i}. {warning}")

        if not self.errors and not self.warnings:
            print("\n✅ No issues found! OpenDRIVE file is valid.")

        print("\n" + "=" * 80)


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="Validate OpenDRIVE files for lane link inconsistencies"
    )
    parser.add_argument(
        "xodr_file", type=str, help="Path to OpenDRIVE (.xodr) file to validate"
    )
    parser.add_argument(
        "-v", "--verbose", action="store_true", help="Enable verbose output"
    )

    args = parser.parse_args()

    validator = OpenDriveValidator(args.xodr_file)
    is_valid = validator.validate()
    validator.print_report()

    sys.exit(0 if is_valid else 1)


if __name__ == "__main__":
    main()
