"""Tests for lane predecessor and successor link functionality."""

import lanelet2
from autoware_lanelet2_to_opendrive.opendrive.lane_section import LaneSection
from autoware_lanelet2_to_opendrive.opendrive.lane import Lane
from autoware_lanelet2_to_opendrive.opendrive.road import Road


def test_lane_has_lanelet_id(lanelet_map):
    """Test that Lane stores the corresponding lanelet ID."""

    # Get a specific lanelet
    lanelet = lanelet_map.laneletLayer.get(3002094)

    # Create a lane from this lanelet
    lane = Lane.construct_from_lanelet(lanelet_map, lanelet)

    # Check that the lanelet_id is set
    assert lane.lanelet_id == 3002094


def test_lane_section_lanelet_to_lane_mapping(lanelet_map):
    """Test that LaneSection provides correct lanelet to lane ID mapping."""

    # Use two adjacent lanelets
    lanelet_group = [
        lanelet_map.laneletLayer.get(3002094),
        lanelet_map.laneletLayer.get(3002093),
    ]

    lane_section = LaneSection.construct_from_lanelet_groups(
        lanelet_map, lanelet_group, s_offset=0.0
    )

    # Get the mapping
    mapping = lane_section.get_lanelet_to_lane_mapping()

    # Check that both lanelets are mapped
    assert 3002094 in mapping
    assert 3002093 in mapping

    # Lane IDs should be negative (right lanes)
    assert mapping[3002094] < 0
    assert mapping[3002093] < 0

    # The lanelets should map to different lane IDs
    assert mapping[3002094] != mapping[3002093]


def test_road_lanelet_to_lane_mapping(lanelet_map):
    """Test that Road provides correct lanelet to lane ID mapping."""

    # Use two adjacent lanelets
    lanelet_group = [
        lanelet_map.laneletLayer.get(3002094),
        lanelet_map.laneletLayer.get(3002093),
    ]

    road = Road.construct_from_lanelet_groups(
        lanelet_map, lanelet_group, road_id=0, s_offset=0.0
    )

    # Get the mapping
    mapping = road.get_lanelet_to_lane_mapping()

    # Check that both lanelets are mapped
    assert 3002094 in mapping
    assert 3002093 in mapping


def test_lane_links_set_correctly(lanelet_map):
    """Test that lane predecessor/successor are set correctly in Road.construct_from_lanelet_map."""

    # Construct roads from the map
    roads = Road.construct_from_lanelet_map(lanelet_map)

    # Track statistics for lane links
    lanes_with_predecessor = 0
    lanes_with_successor = 0
    total_lanes = 0

    for road in roads:
        if road.lanes is None:
            continue

        for lane_section in road.lanes.lane_sections:
            for lane in lane_section.right_lanes.values():
                total_lanes += 1
                if lane.predecessor is not None:
                    lanes_with_predecessor += 1
                if lane.successor is not None:
                    lanes_with_successor += 1

            for lane in lane_section.left_lanes.values():
                total_lanes += 1
                if lane.predecessor is not None:
                    lanes_with_predecessor += 1
                if lane.successor is not None:
                    lanes_with_successor += 1

    # We should have some lanes with predecessor/successor links
    # (not all lanes will have links - only those connected to other roads)
    print(f"Total lanes: {total_lanes}")
    print(f"Lanes with predecessor: {lanes_with_predecessor}")
    print(f"Lanes with successor: {lanes_with_successor}")

    # Verify at least some connections were made (unless it's a single-road map)
    if len(roads) > 1:
        # With multiple roads, we expect some lane connections
        assert (
            lanes_with_predecessor > 0 or lanes_with_successor > 0
        ), "Expected at least some lane connections between roads"


def test_lane_link_xml_output(lanelet_map):
    """Test that lane links are correctly output in XML."""

    # Construct roads from the map
    roads = Road.construct_from_lanelet_map(lanelet_map)

    # Find a road with lane links
    road_with_links = None
    for road in roads:
        if road.lanes is None:
            continue
        for lane_section in road.lanes.lane_sections:
            for lane in lane_section.right_lanes.values():
                if lane.predecessor is not None or lane.successor is not None:
                    road_with_links = road
                    break
            if road_with_links:
                break
        if road_with_links:
            break

    if road_with_links is not None:
        # Convert to XML and check structure
        xml = road_with_links.to_xml()

        # Find lane elements with links
        lanes_elem = xml.find("lanes")
        assert lanes_elem is not None

        for lane_section_elem in lanes_elem.findall("laneSection"):
            right_elem = lane_section_elem.find("right")
            if right_elem is not None:
                for lane_elem in right_elem.findall("lane"):
                    link_elem = lane_elem.find("link")
                    if link_elem is not None:
                        # Check that predecessor or successor exists
                        predecessor = link_elem.find("predecessor")
                        successor = link_elem.find("successor")

                        if predecessor is not None:
                            # Verify it has an id attribute
                            assert predecessor.get("id") is not None

                        if successor is not None:
                            # Verify it has an id attribute
                            assert successor.get("id") is not None


def test_connected_lanelets_have_lane_links(lanelet_map):
    """Test that lanelets with previous/following connections have corresponding lane links."""

    # Find lanelets 228, 229, 230 which are known to have connections
    lanelet_228 = lanelet_map.laneletLayer.get(228)
    lanelet_229 = lanelet_map.laneletLayer.get(229)
    lanelet_230 = lanelet_map.laneletLayer.get(230)

    if lanelet_228 is None or lanelet_229 is None or lanelet_230 is None:
        # Skip test if these specific lanelets don't exist
        return

    # Create routing graph
    traffic_rules = lanelet2.traffic_rules.create(
        lanelet2.traffic_rules.Locations.Germany,
        lanelet2.traffic_rules.Participants.Vehicle,
    )
    routing_graph = lanelet2.routing.RoutingGraph(
        lanelet_map, traffic_rules, [lanelet2.routing.RoutingCostDistance(0.0)]
    )

    # Verify these lanelets have connections in the lanelet map
    following_228 = routing_graph.following(lanelet_228)
    previous_228 = routing_graph.previous(lanelet_228)

    # Build roads
    roads = Road.construct_from_lanelet_map(lanelet_map)

    # Find the road containing lanelet 228
    road_with_228 = None
    lane_for_228 = None

    for road in roads:
        if road.lanes is None:
            continue
        for lane_section in road.lanes.lane_sections:
            for lane in lane_section.right_lanes.values():
                if lane.lanelet_id == 228:
                    road_with_228 = road
                    lane_for_228 = lane
                    break
            if lane_for_228:
                break
        if lane_for_228:
            break

    if lane_for_228 is not None:
        # If lanelet 228 has following lanelets in other roads, it should have a successor
        if following_228:
            # Check if any following lanelet is in a different road
            for following_ll in following_228:
                for road in roads:
                    if road.id != road_with_228.id:
                        mapping = road.get_lanelet_to_lane_mapping()
                        if following_ll.id in mapping:
                            # There should be a successor link
                            assert lane_for_228.successor is not None, (
                                f"Lane for lanelet 228 should have successor link "
                                f"(following lanelet {following_ll.id} is in road {road.id})"
                            )
                            break

        if previous_228:
            # Check if any previous lanelet is in a different road
            for prev_ll in previous_228:
                for road in roads:
                    if road.id != road_with_228.id:
                        mapping = road.get_lanelet_to_lane_mapping()
                        if prev_ll.id in mapping:
                            # There should be a predecessor link
                            assert lane_for_228.predecessor is not None, (
                                f"Lane for lanelet 228 should have predecessor link "
                                f"(previous lanelet {prev_ll.id} is in road {road.id})"
                            )
                            break
