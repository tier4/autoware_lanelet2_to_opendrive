"""Test module for the legacy remove operation functionality (execute_remove_operations).

Note: this is the old-style `RemoveOperation` / `execute_remove_operations`, which
is distinct from the newer `RemoveLaneletOperation` / `execute_remove_lanelet_operations`
covered in test_remove_lanelet.py. The two differ in how they handle missing IDs:
execute_remove_lanelet_operations logs a warning for IDs not found in the map, while
execute_remove_operations (tested here) does not - it delegates to remove_lanelets(),
which builds an exclude-set and silently ignores IDs that never matched anything.
"""

import logging

import lanelet2
from autoware_lanelet2_to_opendrive.preprocess_lanelet import (
    LaneletPreprocessor,
    PreprocessOperation,
    RemoveOperation,
)


class TestRemoveOperationsBasic:
    """Test basic lanelet removal via the legacy remove_operations."""

    def test_remove_single_lanelet(self):
        """Test removing a single lanelet from the map."""
        p1 = lanelet2.core.Point3d(1, 0, 0, 0)
        p2 = lanelet2.core.Point3d(2, 1, 0, 0)
        p3 = lanelet2.core.Point3d(3, 0, 1, 0)
        p4 = lanelet2.core.Point3d(4, 1, 1, 0)

        left_bound = lanelet2.core.LineString3d(101, [p1, p2])
        right_bound = lanelet2.core.LineString3d(102, [p3, p4])

        lanelet1 = lanelet2.core.Lanelet(201, left_bound, right_bound)
        lanelet2_obj = lanelet2.core.Lanelet(202, left_bound, right_bound)
        lanelet3 = lanelet2.core.Lanelet(203, left_bound, right_bound)

        lanelet_map = lanelet2.core.LaneletMap()
        lanelet_map.add(lanelet1)
        lanelet_map.add(lanelet2_obj)
        lanelet_map.add(lanelet3)

        config = PreprocessOperation(
            input_map_path="dummy.osm",
            output_map_path="dummy_out.osm",
            mgrs_code="54SUE",
            remove_operations=[RemoveOperation(lanelet_ids=[202])],
        )
        preprocessor = LaneletPreprocessor(config)
        preprocessor.lanelet_map = lanelet_map
        new_map, log_entries = preprocessor.execute_remove_operations(lanelet_map)

        remaining_ids = {ll.id for ll in new_map.laneletLayer}
        assert remaining_ids == {201, 203}

        assert len(log_entries) == 1
        assert log_entries[0].operation == "remove"
        assert log_entries[0].input_ids == [202]

    def test_remove_multiple_lanelets_in_one_operation(self):
        """Test removing several lanelets specified in a single operation."""
        p1 = lanelet2.core.Point3d(1, 0, 0, 0)
        p2 = lanelet2.core.Point3d(2, 1, 0, 0)
        p3 = lanelet2.core.Point3d(3, 0, 1, 0)
        p4 = lanelet2.core.Point3d(4, 1, 1, 0)

        left_bound = lanelet2.core.LineString3d(101, [p1, p2])
        right_bound = lanelet2.core.LineString3d(102, [p3, p4])

        lanelets = [
            lanelet2.core.Lanelet(300 + i, left_bound, right_bound) for i in range(5)
        ]

        lanelet_map = lanelet2.core.LaneletMap()
        for ll in lanelets:
            lanelet_map.add(ll)

        config = PreprocessOperation(
            input_map_path="dummy.osm",
            output_map_path="dummy_out.osm",
            mgrs_code="54SUE",
            remove_operations=[RemoveOperation(lanelet_ids=[301, 302, 304])],
        )
        preprocessor = LaneletPreprocessor(config)
        preprocessor.lanelet_map = lanelet_map
        new_map, _log = preprocessor.execute_remove_operations(lanelet_map)

        remaining_ids = {ll.id for ll in new_map.laneletLayer}
        assert remaining_ids == {300, 303}

    def test_multiple_remove_operations_in_sequence(self):
        """Test that multiple remove operations are all applied in order."""
        p1 = lanelet2.core.Point3d(1, 0, 0, 0)
        p2 = lanelet2.core.Point3d(2, 1, 0, 0)
        p3 = lanelet2.core.Point3d(3, 0, 1, 0)
        p4 = lanelet2.core.Point3d(4, 1, 1, 0)

        left_bound = lanelet2.core.LineString3d(101, [p1, p2])
        right_bound = lanelet2.core.LineString3d(102, [p3, p4])

        lanelets = [
            lanelet2.core.Lanelet(400 + i, left_bound, right_bound) for i in range(10)
        ]

        lanelet_map = lanelet2.core.LaneletMap()
        for ll in lanelets:
            lanelet_map.add(ll)

        config = PreprocessOperation(
            input_map_path="dummy.osm",
            output_map_path="dummy_out.osm",
            mgrs_code="54SUE",
            remove_operations=[
                RemoveOperation(lanelet_ids=[401, 402]),
                RemoveOperation(lanelet_ids=[405]),
                RemoveOperation(lanelet_ids=[407, 408, 409]),
            ],
        )
        preprocessor = LaneletPreprocessor(config)
        preprocessor.lanelet_map = lanelet_map
        new_map, log_entries = preprocessor.execute_remove_operations(lanelet_map)

        remaining_ids = {ll.id for ll in new_map.laneletLayer}
        assert remaining_ids == {400, 403, 404, 406}
        assert len(log_entries) == 3


class TestRemoveOperationsInvalidId:
    """Test that missing lanelet IDs are silently ignored (no raise, no warning)."""

    def test_invalid_id_mixed_with_valid_id_is_silently_ignored(self, caplog):
        """A nonexistent ID alongside a valid one is dropped with no error or warning."""
        p1 = lanelet2.core.Point3d(1, 0, 0, 0)
        p2 = lanelet2.core.Point3d(2, 1, 0, 0)
        p3 = lanelet2.core.Point3d(3, 0, 1, 0)
        p4 = lanelet2.core.Point3d(4, 1, 1, 0)

        left_bound = lanelet2.core.LineString3d(101, [p1, p2])
        right_bound = lanelet2.core.LineString3d(102, [p3, p4])
        lanelet1 = lanelet2.core.Lanelet(201, left_bound, right_bound)

        lanelet_map = lanelet2.core.LaneletMap()
        lanelet_map.add(lanelet1)

        config = PreprocessOperation(
            input_map_path="dummy.osm",
            output_map_path="dummy_out.osm",
            mgrs_code="54SUE",
            remove_operations=[RemoveOperation(lanelet_ids=[201, 999])],
        )
        preprocessor = LaneletPreprocessor(config)
        preprocessor.lanelet_map = lanelet_map

        with caplog.at_level(logging.WARNING):
            new_map, _log = preprocessor.execute_remove_operations(lanelet_map)

        # 201 was removed; 999 simply never matched anything - no error either way.
        assert {ll.id for ll in new_map.laneletLayer} == set()
        assert caplog.records == []

    def test_only_invalid_ids_result_in_no_op(self, caplog):
        """An operation containing only nonexistent IDs leaves the map untouched."""
        p1 = lanelet2.core.Point3d(1, 0, 0, 0)
        p2 = lanelet2.core.Point3d(2, 1, 0, 0)
        p3 = lanelet2.core.Point3d(3, 0, 1, 0)
        p4 = lanelet2.core.Point3d(4, 1, 1, 0)

        left_bound = lanelet2.core.LineString3d(101, [p1, p2])
        right_bound = lanelet2.core.LineString3d(102, [p3, p4])
        lanelet1 = lanelet2.core.Lanelet(201, left_bound, right_bound)

        lanelet_map = lanelet2.core.LaneletMap()
        lanelet_map.add(lanelet1)

        config = PreprocessOperation(
            input_map_path="dummy.osm",
            output_map_path="dummy_out.osm",
            mgrs_code="54SUE",
            remove_operations=[RemoveOperation(lanelet_ids=[999])],
        )
        preprocessor = LaneletPreprocessor(config)
        preprocessor.lanelet_map = lanelet_map

        with caplog.at_level(logging.WARNING):
            new_map, _log = preprocessor.execute_remove_operations(lanelet_map)

        assert {ll.id for ll in new_map.laneletLayer} == {201}
        assert caplog.records == []


class TestYamlConfig:
    """Test YAML configuration round-tripping for remove operations."""

    def test_create_and_load_config(self, tmp_path):
        """Test creating and loading configuration with remove operations."""
        config = PreprocessOperation(
            input_map_path="test_input.osm",
            output_map_path="test_output.osm",
            mgrs_code="54SUE815501",
            remove_operations=[
                RemoveOperation(lanelet_ids=[100, 101, 102]),
                RemoveOperation(lanelet_ids=[200, 201]),
            ],
        )

        yaml_path = tmp_path / "test_remove_ops_config.yaml"
        config.to_yaml(yaml_path)

        loaded_config = PreprocessOperation.from_yaml(yaml_path)

        assert len(loaded_config.remove_operations) == 2
        assert loaded_config.remove_operations[0].lanelet_ids == [100, 101, 102]
        assert loaded_config.remove_operations[1].lanelet_ids == [200, 201]

    def test_empty_remove_operations(self, tmp_path):
        """Test configuration with no remove operations."""
        config = PreprocessOperation(
            input_map_path="test_input.osm",
            output_map_path="test_output.osm",
            mgrs_code="54SUE815501",
            remove_operations=[],
        )

        yaml_path = tmp_path / "test_empty_remove_ops.yaml"
        config.to_yaml(yaml_path)

        loaded_config = PreprocessOperation.from_yaml(yaml_path)

        assert len(loaded_config.remove_operations) == 0
