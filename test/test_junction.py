"""Tests for junction functions."""

from pathlib import Path
from autoware_lanelet2_extension_python.projection import MGRSProjector
import lanelet2


def load_test_map():
    """Load the test lanelet2 map."""
    test_data_path = Path(__file__).parent / "data" / "lanelet2_map.osm"
    projector = MGRSProjector(
        lanelet2.io.Origin(35.23, 139.16)
    )  # MGRS origin for Tokyo area (54SUE)
    return lanelet2.io.load(str(test_data_path), projector)


def test_filter_lanelets_inside_junction():
    """Test filtering lanelets inside a junction."""
    lanelet_map = load_test_map()
    lanelets = list(lanelet_map.laneletLayer)

    from autoware_lanelet2_to_opendrive.junction import filter_lanelets_inside_junction

    junction_lanelets = filter_lanelets_inside_junction(lanelets)

    junction_ids = {ll.id for ll in junction_lanelets}
    assert 3002084 in junction_ids  # Example junction lanelet ID

    # Check that the filtered lanelets have the 'turn_direction' attribute
    for lanelet in junction_lanelets:
        assert "turn_direction" in lanelet.attributes
