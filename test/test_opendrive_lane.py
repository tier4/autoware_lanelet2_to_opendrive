"""Tests for OpenDRIVE lane functions."""

from pathlib import Path
import lanelet2
from autoware_lanelet2_extension_python.projection import MGRSProjector


def load_test_map():
    """Load the test lanelet2 map."""
    test_data_path = Path(__file__).parent / "data" / "lanelet2_map.osm"
    projector = MGRSProjector(
        lanelet2.io.Origin(35.23, 139.16)
    )  # MGRS origin for Tokyo area (54SUE)
    return lanelet2.io.load(str(test_data_path), projector)


# def test_to_standard_lane():
#     lanelet_map = load_test_map()
#     lanelet_555 = lanelet_map.laneletLayer.get(555)
#     lane_opendrive = Lane.construct_from_lanelet(
#         lanelet_map, lanelet_555
#     ).to_standard_lane()
