"""Test module for remove_lanelet functionality."""

import lanelet2
from autoware_lanelet2_to_opendrive.preprocess_lanelet import (
    PreprocessOperation,
    RemoveLaneletOperation,
    LaneletPreprocessor,
)


class TestRemoveLaneletBasic:
    """Test basic lanelet removal functionality."""

    def test_remove_single_lanelet(self):
        """Test removing a single lanelet from the map."""
        # Create test lanelets
        p1 = lanelet2.core.Point3d(1, 0, 0, 0)
        p2 = lanelet2.core.Point3d(2, 1, 0, 0)
        p3 = lanelet2.core.Point3d(3, 0, 1, 0)
        p4 = lanelet2.core.Point3d(4, 1, 1, 0)

        left_bound = lanelet2.core.LineString3d(101, [p1, p2])
        right_bound = lanelet2.core.LineString3d(102, [p3, p4])

        lanelet1 = lanelet2.core.Lanelet(201, left_bound, right_bound)
        lanelet2_obj = lanelet2.core.Lanelet(202, left_bound, right_bound)
        lanelet3 = lanelet2.core.Lanelet(203, left_bound, right_bound)

        # Create map
        lanelet_map = lanelet2.core.LaneletMap()
        lanelet_map.add(lanelet1)
        lanelet_map.add(lanelet2_obj)
        lanelet_map.add(lanelet3)

        assert len(list(lanelet_map.laneletLayer)) == 3

        # Create config to remove lanelet 202
        config = PreprocessOperation(
            input_map_path="dummy.osm",
            output_map_path="dummy_out.osm",
            mgrs_code="54SUE",
            remove_lanelet_operations=[RemoveLaneletOperation(lanelet_ids=[202])],
        )

        # Execute removal
        preprocessor = LaneletPreprocessor(config)
        preprocessor.lanelet_map = lanelet_map
        new_map, _log = preprocessor.execute_remove_lanelet_operations(lanelet_map)

        # Verify lanelet was removed
        assert len(list(new_map.laneletLayer)) == 2

        # Check specific lanelets
        remaining_ids = {ll.id for ll in new_map.laneletLayer}
        assert 201 in remaining_ids
        assert 202 not in remaining_ids
        assert 203 in remaining_ids

    def test_remove_multiple_lanelets(self):
        """Test removing multiple lanelets from the map."""
        # Create test lanelets
        p1 = lanelet2.core.Point3d(1, 0, 0, 0)
        p2 = lanelet2.core.Point3d(2, 1, 0, 0)
        p3 = lanelet2.core.Point3d(3, 0, 1, 0)
        p4 = lanelet2.core.Point3d(4, 1, 1, 0)

        left_bound = lanelet2.core.LineString3d(101, [p1, p2])
        right_bound = lanelet2.core.LineString3d(102, [p3, p4])

        lanelets = []
        for i in range(5):
            ll = lanelet2.core.Lanelet(300 + i, left_bound, right_bound)
            lanelets.append(ll)

        # Create map
        lanelet_map = lanelet2.core.LaneletMap()
        for ll in lanelets:
            lanelet_map.add(ll)

        assert len(list(lanelet_map.laneletLayer)) == 5

        # Create config to remove lanelets 301, 302, 304
        config = PreprocessOperation(
            input_map_path="dummy.osm",
            output_map_path="dummy_out.osm",
            mgrs_code="54SUE",
            remove_lanelet_operations=[
                RemoveLaneletOperation(lanelet_ids=[301, 302, 304])
            ],
        )

        # Execute removal
        preprocessor = LaneletPreprocessor(config)
        preprocessor.lanelet_map = lanelet_map
        new_map, _log = preprocessor.execute_remove_lanelet_operations(lanelet_map)

        # Verify lanelets were removed
        assert len(list(new_map.laneletLayer)) == 2

        # Check specific lanelets
        remaining_ids = {ll.id for ll in new_map.laneletLayer}
        assert 300 in remaining_ids
        assert 301 not in remaining_ids
        assert 302 not in remaining_ids
        assert 303 in remaining_ids
        assert 304 not in remaining_ids

    def test_remove_nonexistent_lanelet(self):
        """Test removing a lanelet that doesn't exist."""
        # Create test lanelet
        p1 = lanelet2.core.Point3d(1, 0, 0, 0)
        p2 = lanelet2.core.Point3d(2, 1, 0, 0)
        p3 = lanelet2.core.Point3d(3, 0, 1, 0)
        p4 = lanelet2.core.Point3d(4, 1, 1, 0)

        left_bound = lanelet2.core.LineString3d(101, [p1, p2])
        right_bound = lanelet2.core.LineString3d(102, [p3, p4])
        lanelet = lanelet2.core.Lanelet(201, left_bound, right_bound)

        # Create map
        lanelet_map = lanelet2.core.LaneletMap()
        lanelet_map.add(lanelet)

        assert len(list(lanelet_map.laneletLayer)) == 1

        # Create config to remove non-existent lanelet 999
        config = PreprocessOperation(
            input_map_path="dummy.osm",
            output_map_path="dummy_out.osm",
            mgrs_code="54SUE",
            remove_lanelet_operations=[RemoveLaneletOperation(lanelet_ids=[999])],
        )

        # Execute removal
        preprocessor = LaneletPreprocessor(config)
        preprocessor.lanelet_map = lanelet_map
        new_map, _log = preprocessor.execute_remove_lanelet_operations(lanelet_map)

        # Verify nothing was removed
        assert len(list(new_map.laneletLayer)) == 1
        assert list(new_map.laneletLayer)[0].id == 201

    def test_multiple_remove_operations(self):
        """Test multiple remove operations in sequence."""
        # Create test lanelets
        p1 = lanelet2.core.Point3d(1, 0, 0, 0)
        p2 = lanelet2.core.Point3d(2, 1, 0, 0)
        p3 = lanelet2.core.Point3d(3, 0, 1, 0)
        p4 = lanelet2.core.Point3d(4, 1, 1, 0)

        left_bound = lanelet2.core.LineString3d(101, [p1, p2])
        right_bound = lanelet2.core.LineString3d(102, [p3, p4])

        lanelets = []
        for i in range(10):
            ll = lanelet2.core.Lanelet(400 + i, left_bound, right_bound)
            lanelets.append(ll)

        # Create map
        lanelet_map = lanelet2.core.LaneletMap()
        for ll in lanelets:
            lanelet_map.add(ll)

        assert len(list(lanelet_map.laneletLayer)) == 10

        # Create config with multiple remove operations
        config = PreprocessOperation(
            input_map_path="dummy.osm",
            output_map_path="dummy_out.osm",
            mgrs_code="54SUE",
            remove_lanelet_operations=[
                RemoveLaneletOperation(lanelet_ids=[401, 402]),
                RemoveLaneletOperation(lanelet_ids=[405]),
                RemoveLaneletOperation(lanelet_ids=[407, 408, 409]),
            ],
        )

        # Execute removal
        preprocessor = LaneletPreprocessor(config)
        preprocessor.lanelet_map = lanelet_map
        new_map, _log = preprocessor.execute_remove_lanelet_operations(lanelet_map)

        # Verify lanelets were removed
        assert len(list(new_map.laneletLayer)) == 4

        # Check specific lanelets
        remaining_ids = {ll.id for ll in new_map.laneletLayer}
        assert 400 in remaining_ids
        assert 401 not in remaining_ids
        assert 402 not in remaining_ids
        assert 403 in remaining_ids
        assert 404 in remaining_ids
        assert 405 not in remaining_ids
        assert 406 in remaining_ids
        assert 407 not in remaining_ids
        assert 408 not in remaining_ids
        assert 409 not in remaining_ids


class TestYamlConfig:
    """Test YAML configuration for remove_lanelet operations."""

    def test_create_and_load_config(self, tmp_path):
        """Test creating and loading configuration with remove_lanelet operations."""
        # Create configuration
        config = PreprocessOperation(
            input_map_path="test_input.osm",
            output_map_path="test_output.osm",
            mgrs_code="54SUE815501",
            remove_lanelet_operations=[
                RemoveLaneletOperation(lanelet_ids=[100, 101, 102]),
                RemoveLaneletOperation(lanelet_ids=[200, 201]),
            ],
        )

        # Save to YAML
        yaml_path = tmp_path / "test_remove_config.yaml"
        config.to_yaml(yaml_path)

        # Load from YAML
        loaded_config = PreprocessOperation.from_yaml(yaml_path)

        assert len(loaded_config.remove_lanelet_operations) == 2
        assert loaded_config.remove_lanelet_operations[0].lanelet_ids == [100, 101, 102]
        assert loaded_config.remove_lanelet_operations[1].lanelet_ids == [200, 201]

    def test_empty_remove_operations(self, tmp_path):
        """Test configuration with no remove operations."""
        config = PreprocessOperation(
            input_map_path="test_input.osm",
            output_map_path="test_output.osm",
            mgrs_code="54SUE815501",
            remove_lanelet_operations=[],
        )

        yaml_path = tmp_path / "test_empty_remove.yaml"
        config.to_yaml(yaml_path)

        loaded_config = PreprocessOperation.from_yaml(yaml_path)

        assert len(loaded_config.remove_lanelet_operations) == 0
