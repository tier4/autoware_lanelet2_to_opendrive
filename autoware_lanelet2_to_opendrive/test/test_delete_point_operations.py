"""Test module for delete point operation functionality (execute_delete_point_operations)."""

import logging

import lanelet2
from autoware_lanelet2_to_opendrive.preprocess_lanelet import (
    DeletePointOperation,
    LaneletPreprocessor,
    PreprocessOperation,
)


class TestDeletePointBasic:
    """Test that delete point operations mutate the map in place."""

    def _build_three_point_lanelet_map(self):
        # Three points per boundary so removing one still leaves >= 2.
        p1 = lanelet2.core.Point3d(1, 0, 0, 0)
        p2 = lanelet2.core.Point3d(2, 1, 0, 0)
        p3 = lanelet2.core.Point3d(3, 2, 0, 0)
        p4 = lanelet2.core.Point3d(4, 0, 1, 0)
        p5 = lanelet2.core.Point3d(5, 1, 1, 0)
        p6 = lanelet2.core.Point3d(6, 2, 1, 0)

        left_bound = lanelet2.core.LineString3d(101, [p1, p2, p3])
        right_bound = lanelet2.core.LineString3d(102, [p4, p5, p6])
        lanelet = lanelet2.core.Lanelet(601, left_bound, right_bound)

        lanelet_map = lanelet2.core.LaneletMap()
        lanelet_map.add(lanelet)
        return lanelet_map

    def test_delete_existing_point_removes_it_from_boundary(self):
        """Deleting an existing point removes it and mutates the map in place."""
        lanelet_map = self._build_three_point_lanelet_map()

        config = PreprocessOperation(
            input_map_path="dummy.osm",
            output_map_path="dummy_out.osm",
            mgrs_code="54SUE",
            delete_point_operations=[DeletePointOperation(point_ids=[2])],
        )
        preprocessor = LaneletPreprocessor(config)
        preprocessor.lanelet_map = lanelet_map
        new_map, log_entries = preprocessor.execute_delete_point_operations(lanelet_map)

        # delete_points_from_map mutates lanelet boundaries in place - no new map.
        assert new_map is lanelet_map

        updated_lanelet = new_map.laneletLayer.get(601)
        assert [p.id for p in updated_lanelet.leftBound] == [1, 3]

        assert len(log_entries) == 1
        assert log_entries[0].operation == "delete_point"
        assert log_entries[0].input_ids == [2]
        assert "1/1" in log_entries[0].details

    def test_delete_nonexistent_point_is_recorded_as_failed_without_exception(self):
        """A point ID that doesn't exist anywhere is recorded as a failed deletion."""
        lanelet_map = self._build_three_point_lanelet_map()

        config = PreprocessOperation(
            input_map_path="dummy.osm",
            output_map_path="dummy_out.osm",
            mgrs_code="54SUE",
            delete_point_operations=[DeletePointOperation(point_ids=[999])],
        )
        preprocessor = LaneletPreprocessor(config)
        preprocessor.lanelet_map = lanelet_map
        new_map, log_entries = preprocessor.execute_delete_point_operations(lanelet_map)

        updated_lanelet = new_map.laneletLayer.get(601)
        assert [p.id for p in updated_lanelet.leftBound] == [1, 2, 3]
        assert "0/1" in log_entries[0].details


class TestDeletePointDryRunGating:
    """Test the (reversed) dry_run gating for the failed-deletion warning.

    Unlike merge/replace (which raise when dry_run=False and suppress the
    raise when dry_run=True), execute_delete_point_operations never raises -
    it only *warns* about failed deletions, and only when dry_run=False
    (preprocess_lanelet.py:950). This is the opposite gating direction.
    """

    def _build_map_with_missing_point_op(self, dry_run: bool):
        p1 = lanelet2.core.Point3d(1, 0, 0, 0)
        p2 = lanelet2.core.Point3d(2, 1, 0, 0)
        p3 = lanelet2.core.Point3d(3, 2, 0, 0)
        p4 = lanelet2.core.Point3d(4, 0, 1, 0)
        p5 = lanelet2.core.Point3d(5, 1, 1, 0)
        p6 = lanelet2.core.Point3d(6, 2, 1, 0)

        left_bound = lanelet2.core.LineString3d(101, [p1, p2, p3])
        right_bound = lanelet2.core.LineString3d(102, [p4, p5, p6])
        lanelet = lanelet2.core.Lanelet(601, left_bound, right_bound)

        lanelet_map = lanelet2.core.LaneletMap()
        lanelet_map.add(lanelet)

        config = PreprocessOperation(
            input_map_path="dummy.osm",
            output_map_path="dummy_out.osm",
            mgrs_code="54SUE",
            delete_point_operations=[DeletePointOperation(point_ids=[999])],
            dry_run=dry_run,
        )
        preprocessor = LaneletPreprocessor(config)
        preprocessor.lanelet_map = lanelet_map
        return preprocessor, lanelet_map

    def test_dry_run_false_warns_on_failed_deletion(self, caplog):
        """With dry_run=False, a failed deletion logs a warning (no raise)."""
        preprocessor, lanelet_map = self._build_map_with_missing_point_op(dry_run=False)

        with caplog.at_level(logging.WARNING):
            preprocessor.execute_delete_point_operations(lanelet_map)

        assert any("Failed to delete points" in r.message for r in caplog.records)

    def test_dry_run_true_suppresses_failed_deletion_warning(self, caplog):
        """With dry_run=True, the same failed deletion does not log a warning."""
        preprocessor, lanelet_map = self._build_map_with_missing_point_op(dry_run=True)

        with caplog.at_level(logging.WARNING):
            preprocessor.execute_delete_point_operations(lanelet_map)

        assert not any("Failed to delete points" in r.message for r in caplog.records)


class TestYamlConfig:
    """Test YAML configuration round-tripping for delete point operations."""

    def test_create_and_load_config(self, tmp_path):
        """Test creating and loading configuration with delete point operations."""
        config = PreprocessOperation(
            input_map_path="test_input.osm",
            output_map_path="test_output.osm",
            mgrs_code="54SUE815501",
            delete_point_operations=[
                DeletePointOperation(point_ids=[100, 101, 102]),
                DeletePointOperation(point_ids=[200]),
            ],
        )

        yaml_path = tmp_path / "test_delete_point_config.yaml"
        config.to_yaml(yaml_path)

        loaded_config = PreprocessOperation.from_yaml(yaml_path)

        assert len(loaded_config.delete_point_operations) == 2
        assert loaded_config.delete_point_operations[0].point_ids == [100, 101, 102]
        assert loaded_config.delete_point_operations[1].point_ids == [200]

    def test_empty_delete_point_operations(self, tmp_path):
        """Test configuration with no delete point operations."""
        config = PreprocessOperation(
            input_map_path="test_input.osm",
            output_map_path="test_output.osm",
            mgrs_code="54SUE815501",
            delete_point_operations=[],
        )

        yaml_path = tmp_path / "test_empty_delete_point.yaml"
        config.to_yaml(yaml_path)

        loaded_config = PreprocessOperation.from_yaml(yaml_path)

        assert len(loaded_config.delete_point_operations) == 0
