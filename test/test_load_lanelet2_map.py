import lanelet2
from pathlib import Path


def test_load_lanelet2_map():
    """Test loading a lanelet2 map from OSM file."""
    # Get the path to the test data file
    test_data_path = Path(__file__).parent / "data" / "lanelet2_map.osm"
    # Use standard UTM projector for simplicity
    # For real-world usage, you should use the correct projection for your map's location
    projector = lanelet2.projection.UtmProjector(lanelet2.io.Origin(35.0, 139.0))
    # Note: loadRobust allows loading maps with unsupported regulatory elements
    map_data, errors = lanelet2.io.loadRobust(str(test_data_path), projector)
    assert map_data is not None
    assert isinstance(map_data, lanelet2.core.LaneletMap)
    # It's OK to have errors for custom Autoware regulatory elements
    print(f"Loaded map with {len(map_data.laneletLayer)} lanelets")
    if errors:
        print(f"Encountered {len(errors)} parsing errors (expected for Autoware maps)")
