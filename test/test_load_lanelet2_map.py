import lanelet2
from pathlib import Path


def test_load_lanelet2_map():
    """Test loading a lanelet2 map from OSM file."""
    # Get the path to the test data file
    test_data_path = Path(__file__).parent / "data" / "lanelet2_map.osm"

    # Check that the file exists
    assert test_data_path.exists(), f"Test file not found: {test_data_path}"

    # Create a projector (using UTM projection as an example)
    # You may need to adjust the projection based on your map's location
    projector = lanelet2.projection.UtmProjector(lanelet2.io.Origin(35.0, 139.0))

    # Load the lanelet2 map (with error handling for unsupported regulatory elements)
    # The loadRobust function will load the map even if there are parsing errors
    map_data, errors = lanelet2.io.loadRobust(str(test_data_path), projector)

    # Basic assertions to verify the map was loaded
    assert map_data is not None, "Failed to load lanelet2 map"

    # Check that the map contains some basic elements
    # These assertions may need to be adjusted based on actual map content
    assert isinstance(
        map_data, lanelet2.core.LaneletMap
    ), "Loaded data is not a LaneletMap"

    # Optional: Check for specific map elements if known
    # For example:
    # assert len(map_data.laneletLayer) > 0, "Map contains no lanelets"
    # assert len(map_data.pointLayer) > 0, "Map contains no points"

    print("Successfully loaded lanelet2 map with:")
    print(f"  - {len(map_data.laneletLayer)} lanelets")
    print(f"  - {len(map_data.lineStringLayer)} linestrings")
    print(f"  - {len(map_data.pointLayer)} points")
    print(f"  - {len(map_data.polygonLayer)} polygons")
    print(f"  - {len(map_data.regulatoryElementLayer)} regulatory elements")
    if errors:
        print(
            f"  - {len(errors)} parsing errors (this is expected for Autoware maps with custom regulatory elements)"
        )


def test_lanelet2_map_structure():
    """Test the structure and content of the loaded lanelet2 map."""
    test_data_path = Path(__file__).parent / "data" / "lanelet2_map.osm"
    projector = lanelet2.projection.UtmProjector(lanelet2.io.Origin(35.0, 139.0))
    map_data, errors = lanelet2.io.loadRobust(str(test_data_path), projector)

    # Test that we can iterate through lanelets
    for lanelet in map_data.laneletLayer:
        assert lanelet.id is not None, "Lanelet has no ID"
        assert lanelet.leftBound is not None, "Lanelet has no left boundary"
        assert lanelet.rightBound is not None, "Lanelet has no right boundary"
        break  # Just test the first one if it exists

    # Test that we can access map layers
    assert hasattr(map_data, "laneletLayer"), "Map has no lanelet layer"
    assert hasattr(map_data, "pointLayer"), "Map has no point layer"
    assert hasattr(map_data, "lineStringLayer"), "Map has no linestring layer"
    assert hasattr(map_data, "polygonLayer"), "Map has no polygon layer"
    assert hasattr(
        map_data, "regulatoryElementLayer"
    ), "Map has no regulatory element layer"
