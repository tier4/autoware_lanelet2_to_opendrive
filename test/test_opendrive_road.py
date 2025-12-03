"""Tests for OpenDRIVE lane section functions."""

from pathlib import Path
import lanelet2
from autoware_lanelet2_extension_python.projection import MGRSProjector
from autoware_lanelet2_to_opendrive.opendrive.road import Road


def load_test_map():
    """Load the test lanelet2 map."""
    test_data_path = Path(__file__).parent / "data" / "lanelet2_map.osm"
    projector = MGRSProjector(
        lanelet2.io.Origin(35.23, 139.16)
    )  # MGRS origin for Tokyo area (54SUE)
    return lanelet2.io.load(str(test_data_path), projector)


def test_construct_road_from_two_lanes():
    """Test constructing a Road from two adjacent lanelets."""
    lanelet_map = load_test_map()

    # Use two adjacent lanelets
    lanelet_group = [
        lanelet_map.laneletLayer.get(3002094),
        lanelet_map.laneletLayer.get(3002093),
    ]

    road = Road.construct_from_lanelet_groups(  # noqa F841
        lanelet_map, lanelet_group, road_id=0, s_offset=0.0
    )

    # print("")
    # print(etree.tostring(road.to_xml(), pretty_print=True).decode())
