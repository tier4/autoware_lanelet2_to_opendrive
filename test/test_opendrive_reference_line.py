"""Tests for OpenDRIVE reference line functions."""

from pathlib import Path
import lanelet2
from autoware_lanelet2_extension_python.projection import MGRSProjector
from autoware_lanelet2_to_opendrive.opendrive.reference_line import ReferenceLine


def load_test_map():
    """Load the test lanelet2 map."""
    test_data_path = Path(__file__).parent / "data" / "lanelet2_map.osm"
    projector = MGRSProjector(
        lanelet2.io.Origin(35.23, 139.16)
    )  # MGRS origin for Tokyo area (54SUE)
    return lanelet2.io.load(str(test_data_path), projector)


def test_to_standard_lane():
    lanelet_map = load_test_map()
    ReferenceLine.construct_from_lanelet_groups(
        lanelet_map,
        [lanelet_map.laneletLayer.get(3002094), lanelet_map.laneletLayer.get(3002093)],
    ).to_standard_lane()
