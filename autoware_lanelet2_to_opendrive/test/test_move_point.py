"""Test module for move_point functionality."""

import lanelet2
from autoware_lanelet2_to_opendrive.lanelet import (
    move_point_in_map,
)
from autoware_lanelet2_to_opendrive.preprocess_lanelet import (
    PreprocessOperation,
    MovePointOperation,
)


class TestMovePointBasic:
    """Test basic point movement functionality."""

    def test_move_single_point(self):
        """Test moving a single point to new coordinates."""
        # Create test points
        p1 = lanelet2.core.Point3d(1, 0, 0, 0)
        p2 = lanelet2.core.Point3d(2, 1, 0, 0)
        p3 = lanelet2.core.Point3d(3, 2, 0, 0)

        p4 = lanelet2.core.Point3d(4, 0, 1, 0)
        p5 = lanelet2.core.Point3d(5, 1, 1, 0)
        p6 = lanelet2.core.Point3d(6, 2, 1, 0)

        # Create linestrings and lanelet
        left_bound = lanelet2.core.LineString3d(101, [p1, p2, p3])
        right_bound = lanelet2.core.LineString3d(102, [p4, p5, p6])
        lanelet = lanelet2.core.Lanelet(201, left_bound, right_bound)

        lanelet_map = lanelet2.core.LaneletMap()
        lanelet_map.add(lanelet)

        # Move point 2 from (1, 0, 0) to (1.5, 0.5, 0.2)
        success = move_point_in_map(lanelet_map, 2, 1.5, 0.5, 0.2)

        assert success is True

        # Verify the point was moved
        updated_lanelet = list(lanelet_map.laneletLayer)[0]
        updated_point = None
        for p in updated_lanelet.leftBound:
            if p.id == 2:
                updated_point = p
                break

        assert updated_point is not None
        assert abs(updated_point.x - 1.5) < 1e-10
        assert abs(updated_point.y - 0.5) < 1e-10
        assert abs(updated_point.z - 0.2) < 1e-10

    def test_move_point_without_z(self):
        """Test moving a point without specifying Z coordinate."""
        # Create test point
        p1 = lanelet2.core.Point3d(1, 0, 0, 5.0)  # Original Z = 5.0
        p2 = lanelet2.core.Point3d(2, 1, 0, 0)

        left_bound = lanelet2.core.LineString3d(101, [p1, p2])
        right_bound = lanelet2.core.LineString3d(102, [p1, p2])
        lanelet = lanelet2.core.Lanelet(201, left_bound, right_bound)

        lanelet_map = lanelet2.core.LaneletMap()
        lanelet_map.add(lanelet)

        # Move point 1 without changing Z
        success = move_point_in_map(lanelet_map, 1, 2.0, 3.0, None)

        assert success is True

        # Verify the point was moved but Z was preserved
        updated_lanelet = list(lanelet_map.laneletLayer)[0]
        updated_point = None
        for p in updated_lanelet.leftBound:
            if p.id == 1:
                updated_point = p
                break

        assert updated_point is not None
        assert abs(updated_point.x - 2.0) < 1e-10
        assert abs(updated_point.y - 3.0) < 1e-10
        assert abs(updated_point.z - 5.0) < 1e-10  # Z preserved

    def test_move_nonexistent_point(self):
        """Test moving a point that doesn't exist."""
        p1 = lanelet2.core.Point3d(1, 0, 0, 0)
        p2 = lanelet2.core.Point3d(2, 1, 0, 0)

        left_bound = lanelet2.core.LineString3d(101, [p1, p2])
        right_bound = lanelet2.core.LineString3d(102, [p1, p2])
        lanelet = lanelet2.core.Lanelet(201, left_bound, right_bound)

        lanelet_map = lanelet2.core.LaneletMap()
        lanelet_map.add(lanelet)

        # Try to move non-existent point
        success = move_point_in_map(lanelet_map, 999, 5.0, 6.0, 7.0)

        assert success is False


class TestMovePointAttributes:
    """Test point movement preserves attributes."""

    def test_move_point_preserves_attributes(self):
        """Test that moving a point preserves its attributes and updates local coordinates."""
        # Create test point with attributes
        p1 = lanelet2.core.Point3d(1, 0, 0, 0)
        p1.attributes["local_x"] = "0.0000"
        p1.attributes["local_y"] = "0.0000"
        p1.attributes["ele"] = "0.000000000000"
        p1.attributes["custom_attr"] = "test_value"

        p2 = lanelet2.core.Point3d(2, 1, 0, 0)

        left_bound = lanelet2.core.LineString3d(101, [p1, p2])
        right_bound = lanelet2.core.LineString3d(102, [p1, p2])
        lanelet = lanelet2.core.Lanelet(201, left_bound, right_bound)

        lanelet_map = lanelet2.core.LaneletMap()
        lanelet_map.add(lanelet)

        # Move point 1
        success = move_point_in_map(lanelet_map, 1, 5.0, 6.0, 7.0)

        assert success is True

        # Verify attributes were preserved and local coordinates updated
        updated_lanelet = list(lanelet_map.laneletLayer)[0]
        updated_point = None
        for p in updated_lanelet.leftBound:
            if p.id == 1:
                updated_point = p
                break

        assert updated_point is not None

        # Check custom attribute was preserved
        assert updated_point.attributes["custom_attr"] == "test_value"

        # Check local coordinates were updated
        assert updated_point.attributes["local_x"] == "5.0000"
        assert updated_point.attributes["local_y"] == "6.0000"
        assert updated_point.attributes["ele"] == "7.000000000000"


class TestYamlConfig:
    """Test YAML configuration for move_point operations."""

    def test_create_and_load_config(self, tmp_path):
        """Test creating and loading configuration with move_point operations."""
        # Create configuration
        config = PreprocessOperation(
            input_map_path="test_input.osm",
            output_map_path="test_output.osm",
            mgrs_code="54SUE815501",
            move_point_operations=[
                MovePointOperation(point_id=100, new_x=10.0, new_y=20.0, new_z=30.0),
                MovePointOperation(point_id=200, new_x=11.0, new_y=21.0, new_z=None),
            ],
        )

        # Save to YAML
        yaml_path = tmp_path / "test_move_config.yaml"
        config.to_yaml(yaml_path)

        # Load from YAML
        loaded_config = PreprocessOperation.from_yaml(yaml_path)

        assert len(loaded_config.move_point_operations) == 2
        assert loaded_config.move_point_operations[0].point_id == 100
        assert loaded_config.move_point_operations[0].new_x == 10.0
        assert loaded_config.move_point_operations[0].new_y == 20.0
        assert loaded_config.move_point_operations[0].new_z == 30.0
        assert loaded_config.move_point_operations[1].point_id == 200
        assert loaded_config.move_point_operations[1].new_x == 11.0
        assert loaded_config.move_point_operations[1].new_y == 21.0
        assert loaded_config.move_point_operations[1].new_z is None

    def test_empty_move_operations(self, tmp_path):
        """Test configuration with no move operations."""
        config = PreprocessOperation(
            input_map_path="test_input.osm",
            output_map_path="test_output.osm",
            mgrs_code="54SUE815501",
            move_point_operations=[],
        )

        yaml_path = tmp_path / "test_empty_move.yaml"
        config.to_yaml(yaml_path)

        loaded_config = PreprocessOperation.from_yaml(yaml_path)

        assert len(loaded_config.move_point_operations) == 0
