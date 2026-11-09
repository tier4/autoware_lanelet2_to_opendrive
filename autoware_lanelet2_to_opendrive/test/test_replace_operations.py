"""Test module for replace operation functionality (execute_replace_operations)."""

import logging

import lanelet2
import pytest
from autoware_lanelet2_to_opendrive.preprocess_lanelet import (
    LaneletPreprocessor,
    PreprocessOperation,
    ReplaceOperation,
)


class TestReplaceBasic:
    """Test that replace operations swap the source lanelets for one merged lanelet."""

    def _build_map_with_untouched_lanelet(self):
        # Lanelets 401, 402 are continuous and will be replaced.
        p1 = lanelet2.core.Point3d(1, 0, 0, 0)
        p2 = lanelet2.core.Point3d(2, 0, 1, 0)
        p3 = lanelet2.core.Point3d(3, 1, 0, 0)
        p4 = lanelet2.core.Point3d(4, 1, 1, 0)
        left1 = lanelet2.core.LineString3d(101, [p1, p2])
        right1 = lanelet2.core.LineString3d(102, [p3, p4])
        lanelet1 = lanelet2.core.Lanelet(401, left1, right1)

        p5 = lanelet2.core.Point3d(5, 0, 2, 0)
        p6 = lanelet2.core.Point3d(6, 1, 2, 0)
        left2 = lanelet2.core.LineString3d(103, [p2, p5])
        right2 = lanelet2.core.LineString3d(104, [p4, p6])
        lanelet2_obj = lanelet2.core.Lanelet(402, left2, right2)

        # Lanelet 403 is unrelated and should be left untouched.
        p7 = lanelet2.core.Point3d(7, 10, 10, 0)
        p8 = lanelet2.core.Point3d(8, 10, 11, 0)
        p9 = lanelet2.core.Point3d(9, 11, 10, 0)
        p10 = lanelet2.core.Point3d(10, 11, 11, 0)
        left3 = lanelet2.core.LineString3d(105, [p7, p8])
        right3 = lanelet2.core.LineString3d(106, [p9, p10])
        lanelet3 = lanelet2.core.Lanelet(403, left3, right3)

        lanelet_map = lanelet2.core.LaneletMap()
        for ll in (lanelet1, lanelet2_obj, lanelet3):
            lanelet_map.add(ll)
        return lanelet_map

    def test_replace_two_continuous_lanelets(self):
        """Replacing 401+402 removes both and adds one merged lanelet; 403 is untouched.

        replace_lanelets() uses new_base_id = max_id (403, not max_id+1), and
        merge_lanelet assigns left=base+1, right=base+2, lanelet=base+3, so the
        merged lanelet gets ID 406.
        """
        lanelet_map = self._build_map_with_untouched_lanelet()

        config = PreprocessOperation(
            input_map_path="dummy.osm",
            output_map_path="dummy_out.osm",
            mgrs_code="54SUE",
            replace_operations=[ReplaceOperation(lanelet_ids=[401, 402])],
        )
        preprocessor = LaneletPreprocessor(config)
        preprocessor.lanelet_map = lanelet_map
        new_map, log_entries = preprocessor.execute_replace_operations(lanelet_map)

        remaining_ids = {ll.id for ll in new_map.laneletLayer}
        assert remaining_ids == {403, 406}

        merged = new_map.laneletLayer.get(406)
        assert [p.id for p in merged.leftBound] == [1, 2, 5]
        assert [p.id for p in merged.rightBound] == [3, 4, 6]

        assert len(log_entries) == 1
        assert log_entries[0].operation == "replace"
        assert log_entries[0].input_ids == [401, 402]
        assert log_entries[0].output_ids == [406]


class TestReplaceInvalidId:
    """Test the dry_run-gated error handling for missing lanelet IDs."""

    def _build_single_lanelet_map(self):
        p1 = lanelet2.core.Point3d(1, 0, 0, 0)
        p2 = lanelet2.core.Point3d(2, 0, 1, 0)
        p3 = lanelet2.core.Point3d(3, 1, 0, 0)
        p4 = lanelet2.core.Point3d(4, 1, 1, 0)
        left1 = lanelet2.core.LineString3d(101, [p1, p2])
        right1 = lanelet2.core.LineString3d(102, [p3, p4])
        lanelet1 = lanelet2.core.Lanelet(401, left1, right1)

        lanelet_map = lanelet2.core.LaneletMap()
        lanelet_map.add(lanelet1)
        return lanelet_map

    def test_invalid_id_raises_when_dry_run_is_false(self, caplog):
        """A missing lanelet ID raises ValueError when dry_run is False (the default)."""
        lanelet_map = self._build_single_lanelet_map()
        config = PreprocessOperation(
            input_map_path="dummy.osm",
            output_map_path="dummy_out.osm",
            mgrs_code="54SUE",
            replace_operations=[ReplaceOperation(lanelet_ids=[401, 999])],
            dry_run=False,
        )
        preprocessor = LaneletPreprocessor(config)
        preprocessor.lanelet_map = lanelet_map

        with (
            caplog.at_level(
                logging.WARNING,
                logger="autoware_lanelet2_to_opendrive.preprocess_lanelet",
            ),
            pytest.raises(ValueError, match="999"),
        ):
            preprocessor.execute_replace_operations(lanelet_map)

        assert any("Replace operation failed" in r.message for r in caplog.records)
        assert {ll.id for ll in lanelet_map.laneletLayer} == {401}

    def test_invalid_id_dry_run_suppresses_raise(self, caplog):
        """A missing lanelet ID only warns (no raise) when dry_run is True."""
        lanelet_map = self._build_single_lanelet_map()
        config = PreprocessOperation(
            input_map_path="dummy.osm",
            output_map_path="dummy_out.osm",
            mgrs_code="54SUE",
            replace_operations=[ReplaceOperation(lanelet_ids=[401, 999])],
            dry_run=True,
        )
        preprocessor = LaneletPreprocessor(config)
        preprocessor.lanelet_map = lanelet_map

        with caplog.at_level(
            logging.WARNING, logger="autoware_lanelet2_to_opendrive.preprocess_lanelet"
        ):
            new_map, log_entries = preprocessor.execute_replace_operations(lanelet_map)

        assert any("Replace operation failed" in r.message for r in caplog.records)
        # No replace succeeded, so the original map is returned untouched.
        assert new_map is lanelet_map
        assert log_entries == []

    def test_fewer_than_two_lanelets_raises_when_dry_run_is_false(self):
        """replace_lanelets requires >= 2 lanelet IDs; a single ID raises ValueError."""
        lanelet_map = self._build_single_lanelet_map()
        config = PreprocessOperation(
            input_map_path="dummy.osm",
            output_map_path="dummy_out.osm",
            mgrs_code="54SUE",
            replace_operations=[ReplaceOperation(lanelet_ids=[401])],
            dry_run=False,
        )
        preprocessor = LaneletPreprocessor(config)
        preprocessor.lanelet_map = lanelet_map

        with pytest.raises(ValueError, match="At least 2 lanelets required"):
            preprocessor.execute_replace_operations(lanelet_map)


class TestReplaceValidateFlag:
    """Test that the `validate` flag controls continuity enforcement."""

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

    def test_validate_true_rejects_discontinuous_lanelets(self):
        """With validate=True (default), a continuity gap raises ValueError."""
        lanelet_map = self._build_discontinuous_map()
        config = PreprocessOperation(
            input_map_path="dummy.osm",
            output_map_path="dummy_out.osm",
            mgrs_code="54SUE",
            replace_operations=[
                ReplaceOperation(lanelet_ids=[501, 502], validate=True)
            ],
        )
        preprocessor = LaneletPreprocessor(config)
        preprocessor.lanelet_map = lanelet_map

        with pytest.raises(ValueError, match="not continuous"):
            preprocessor.execute_replace_operations(lanelet_map)

    def test_validate_false_allows_discontinuous_replace(self):
        """With validate=False, the continuity check is skipped and the replace succeeds."""
        lanelet_map = self._build_discontinuous_map()
        config = PreprocessOperation(
            input_map_path="dummy.osm",
            output_map_path="dummy_out.osm",
            mgrs_code="54SUE",
            replace_operations=[
                ReplaceOperation(lanelet_ids=[501, 502], validate=False)
            ],
        )
        preprocessor = LaneletPreprocessor(config)
        preprocessor.lanelet_map = lanelet_map

        new_map, log_entries = preprocessor.execute_replace_operations(lanelet_map)

        # max_id=502 -> new_base_id=502 -> merged id=505.
        assert {ll.id for ll in new_map.laneletLayer} == {505}
        assert log_entries[0].output_ids == [505]


class TestYamlConfig:
    """Test YAML configuration round-tripping for replace operations."""

    def test_create_and_load_config(self, tmp_path):
        """Test creating and loading configuration with replace operations."""
        config = PreprocessOperation(
            input_map_path="test_input.osm",
            output_map_path="test_output.osm",
            mgrs_code="54SUE815501",
            replace_operations=[
                ReplaceOperation(
                    lanelet_ids=[100, 101, 102], validate=False, tolerance=0.01
                ),
                ReplaceOperation(lanelet_ids=[200, 201]),
            ],
        )

        yaml_path = tmp_path / "test_replace_config.yaml"
        config.to_yaml(yaml_path)

        loaded_config = PreprocessOperation.from_yaml(yaml_path)

        assert len(loaded_config.replace_operations) == 2
        assert loaded_config.replace_operations[0].lanelet_ids == [100, 101, 102]
        assert loaded_config.replace_operations[0].validate is False
        assert loaded_config.replace_operations[0].tolerance == 0.01
        assert loaded_config.replace_operations[1].lanelet_ids == [200, 201]
        assert loaded_config.replace_operations[1].validate is True

    def test_empty_replace_operations(self, tmp_path):
        """Test configuration with no replace operations."""
        config = PreprocessOperation(
            input_map_path="test_input.osm",
            output_map_path="test_output.osm",
            mgrs_code="54SUE815501",
            replace_operations=[],
        )

        yaml_path = tmp_path / "test_empty_replace.yaml"
        config.to_yaml(yaml_path)

        loaded_config = PreprocessOperation.from_yaml(yaml_path)

        assert len(loaded_config.replace_operations) == 0
