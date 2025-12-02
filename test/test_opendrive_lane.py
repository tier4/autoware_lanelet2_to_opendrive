from pathlib import Path
import lanelet2
from autoware_lanelet2_extension_python.projection import MGRSProjector
from autoware_lanelet2_to_opendrive.opendrive.lane import Lane, LaneType


def load_test_map():
    """Load the test lanelet2 map."""
    test_data_path = Path(__file__).parent / "data" / "lanelet2_map.osm"
    projector = MGRSProjector(
        lanelet2.io.Origin(35.23, 139.16)
    )  # MGRS origin for Tokyo area (54SUE)
    return lanelet2.io.load(str(test_data_path), projector)


def test_construct_lane_from_lanelet():
    # Create a mock Lanelet2 map and lanelet
    lanelet_map = load_test_map()

    lanelet_555 = lanelet_map.laneletLayer.get(555)

    # Construct the Lane from the lanelet
    lane = Lane.construct_from_lanelet(lanelet_map, lanelet_555)

    # Assertions to verify correct construction
    # TODO : Currently lane ID for OpenDRIVE was not calculated from lanelet ID.
    assert lane.lane_id == 0
    assert lane.lane_type == LaneType.DRIVING
    assert lane.predecessor is None
    assert lane.successor is None
    assert isinstance(lane, Lane)
