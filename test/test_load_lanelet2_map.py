from pathlib import Path
from autoware_lanelet2_extension_python.projection import MGRSProjector
import lanelet2


def test_load_lanelet2_map():
    """Test loading a lanelet2 map from OSM file."""
    # Get the path to the test data file
    test_data_path = Path(__file__).parent / "data" / "lanelet2_map.osm"
    projector = MGRSProjector(
        lanelet2.io.Origin(35.23, 139.16)
    )  # MGRS origin for Tokyo area (54SUE)
    lanelet2.io.load(str(test_data_path), projector)
