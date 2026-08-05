"""Test module for validate operation functionality (execute_validate_operations)."""

import logging

import lanelet2
from autoware_lanelet2_to_opendrive.preprocess_lanelet import (
    LaneletPreprocessor,
    PreprocessOperation,
    ValidateOperation,
)


class TestValidateOperationsBasic:
    """Test that validate operations never mutate the map, regardless of outcome."""

    def _build_continuous_map(self):
        p1 = lanelet2.core.Point3d(1, 0, 0, 0)
        p2 = lanelet2.core.Point3d(2, 0, 1, 0)
        p3 = lanelet2.core.Point3d(3, 1, 0, 0)
        p4 = lanelet2.core.Point3d(4, 1, 1, 0)
        left1 = lanelet2.core.LineString3d(101, [p1, p2])
        right1 = lanelet2.core.LineString3d(102, [p3, p4])
        lanelet1 = lanelet2.core.Lanelet(501, left1, right1)

        p5 = lanelet2.core.Point3d(5, 0, 2, 0)
        p6 = lanelet2.core.Point3d(6, 1, 2, 0)
        left2 = lanelet2.core.LineString3d(103, [p2, p5])
        right2 = lanelet2.core.LineString3d(104, [p4, p6])
        lanelet2_obj = lanelet2.core.Lanelet(502, left2, right2)

        lanelet_map = lanelet2.core.LaneletMap()
        lanelet_map.add(lanelet1)
        lanelet_map.add(lanelet2_obj)
        return lanelet_map

    def _build_discontinuous_map(self):
        p1 = lanelet2.core.Point3d(1, 0, 0, 0)
        p2 = lanelet2.core.Point3d(2, 0, 1, 0)
        p3 = lanelet2.core.Point3d(3, 1, 0, 0)
        p4 = lanelet2.core.Point3d(4, 1, 1, 0)
        left1 = lanelet2.core.LineString3d(101, [p1, p2])
        right1 = lanelet2.core.LineString3d(102, [p3, p4])
        lanelet1 = lanelet2.core.Lanelet(501, left1, right1)

        p5 = lanelet2.core.Point3d(5, 100, 100, 0)
        p6 = lanelet2.core.Point3d(6, 100, 101, 0)
        p7 = lanelet2.core.Point3d(7, 101, 100, 0)
        p8 = lanelet2.core.Point3d(8, 101, 101, 0)
        left2 = lanelet2.core.LineString3d(103, [p5, p6])
        right2 = lanelet2.core.LineString3d(104, [p7, p8])
        lanelet2_obj = lanelet2.core.Lanelet(502, left2, right2)

        lanelet_map = lanelet2.core.LaneletMap()
        lanelet_map.add(lanelet1)
        lanelet_map.add(lanelet2_obj)
        return lanelet_map

    def test_continuous_lanelets_return_none_and_leave_map_unchanged(self, caplog):
        """A continuous pair logs CONTINUOUS at info level and never warns."""
        lanelet_map = self._build_continuous_map()
        ids_before = {ll.id for ll in lanelet_map.laneletLayer}

        config = PreprocessOperation(
            input_map_path="dummy.osm",
            output_map_path="dummy_out.osm",
            mgrs_code="54SUE",
            validate_operations=[
                ValidateOperation(first_lanelet_id=501, second_lanelet_id=502)
            ],
        )
        preprocessor = LaneletPreprocessor(config)
        preprocessor.lanelet_map = lanelet_map

        with caplog.at_level(
            logging.INFO, logger="autoware_lanelet2_to_opendrive.preprocess_lanelet"
        ):
            result = preprocessor.execute_validate_operations(lanelet_map)

        assert result is None
        assert {ll.id for ll in lanelet_map.laneletLayer} == ids_before
        assert any("CONTINUOUS" in r.message for r in caplog.records)
        assert not any(r.levelno == logging.WARNING for r in caplog.records)

    def test_discontinuous_lanelets_logs_info_not_warning(self, caplog):
        """A discontinuous pair logs DISCONTINUOUS at info level - not a warning."""
        lanelet_map = self._build_discontinuous_map()
        ids_before = {ll.id for ll in lanelet_map.laneletLayer}

        config = PreprocessOperation(
            input_map_path="dummy.osm",
            output_map_path="dummy_out.osm",
            mgrs_code="54SUE",
            validate_operations=[
                ValidateOperation(first_lanelet_id=501, second_lanelet_id=502)
            ],
        )
        preprocessor = LaneletPreprocessor(config)
        preprocessor.lanelet_map = lanelet_map

        with caplog.at_level(
            logging.INFO, logger="autoware_lanelet2_to_opendrive.preprocess_lanelet"
        ):
            result = preprocessor.execute_validate_operations(lanelet_map)

        assert result is None
        assert {ll.id for ll in lanelet_map.laneletLayer} == ids_before
        assert any("DISCONTINUOUS" in r.message for r in caplog.records)
        assert not any(r.levelno == logging.WARNING for r in caplog.records)


class TestValidateOperationsInvalidId:
    """Test that a missing lanelet ID warns without raising."""

    def _build_single_lanelet_map(self):
        p1 = lanelet2.core.Point3d(1, 0, 0, 0)
        p2 = lanelet2.core.Point3d(2, 0, 1, 0)
        p3 = lanelet2.core.Point3d(3, 1, 0, 0)
        p4 = lanelet2.core.Point3d(4, 1, 1, 0)
        left1 = lanelet2.core.LineString3d(101, [p1, p2])
        right1 = lanelet2.core.LineString3d(102, [p3, p4])
        lanelet1 = lanelet2.core.Lanelet(501, left1, right1)

        lanelet_map = lanelet2.core.LaneletMap()
        lanelet_map.add(lanelet1)
        return lanelet_map

    def test_invalid_first_id_warns_without_raising(self, caplog):
        """A missing first_lanelet_id logs a warning and does not raise."""
        lanelet_map = self._build_single_lanelet_map()
        config = PreprocessOperation(
            input_map_path="dummy.osm",
            output_map_path="dummy_out.osm",
            mgrs_code="54SUE",
            validate_operations=[
                ValidateOperation(first_lanelet_id=999, second_lanelet_id=501)
            ],
        )
        preprocessor = LaneletPreprocessor(config)
        preprocessor.lanelet_map = lanelet_map

        with caplog.at_level(logging.WARNING):
            result = preprocessor.execute_validate_operations(lanelet_map)

        assert result is None
        assert {ll.id for ll in lanelet_map.laneletLayer} == {501}
        assert any(
            "Validation failed - lanelet not found" in r.message for r in caplog.records
        )

    def test_invalid_second_id_warns_without_raising(self, caplog):
        """A missing second_lanelet_id logs a warning and does not raise."""
        lanelet_map = self._build_single_lanelet_map()
        config = PreprocessOperation(
            input_map_path="dummy.osm",
            output_map_path="dummy_out.osm",
            mgrs_code="54SUE",
            validate_operations=[
                ValidateOperation(first_lanelet_id=501, second_lanelet_id=999)
            ],
        )
        preprocessor = LaneletPreprocessor(config)
        preprocessor.lanelet_map = lanelet_map

        with caplog.at_level(logging.WARNING):
            result = preprocessor.execute_validate_operations(lanelet_map)

        assert result is None
        assert {ll.id for ll in lanelet_map.laneletLayer} == {501}
        assert any(
            "Validation failed - lanelet not found" in r.message for r in caplog.records
        )


class TestYamlConfig:
    """Test YAML configuration round-tripping for validate operations."""

    def test_create_and_load_config(self, tmp_path):
        """Test creating and loading configuration with validate operations."""
        config = PreprocessOperation(
            input_map_path="test_input.osm",
            output_map_path="test_output.osm",
            mgrs_code="54SUE815501",
            validate_operations=[
                ValidateOperation(
                    first_lanelet_id=500, second_lanelet_id=501, tolerance=0.01
                ),
                ValidateOperation(first_lanelet_id=600, second_lanelet_id=601),
            ],
        )

        yaml_path = tmp_path / "test_validate_config.yaml"
        config.to_yaml(yaml_path)

        loaded_config = PreprocessOperation.from_yaml(yaml_path)

        assert len(loaded_config.validate_operations) == 2
        assert loaded_config.validate_operations[0].first_lanelet_id == 500
        assert loaded_config.validate_operations[0].second_lanelet_id == 501
        assert loaded_config.validate_operations[0].tolerance == 0.01
        assert loaded_config.validate_operations[1].first_lanelet_id == 600
        assert loaded_config.validate_operations[1].second_lanelet_id == 601

    def test_empty_validate_operations(self, tmp_path):
        """Test configuration with no validate operations."""
        config = PreprocessOperation(
            input_map_path="test_input.osm",
            output_map_path="test_output.osm",
            mgrs_code="54SUE815501",
            validate_operations=[],
        )

        yaml_path = tmp_path / "test_empty_validate.yaml"
        config.to_yaml(yaml_path)

        loaded_config = PreprocessOperation.from_yaml(yaml_path)

        assert len(loaded_config.validate_operations) == 0
