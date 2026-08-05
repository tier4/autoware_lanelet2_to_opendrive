"""Test module for merge operation functionality (execute_merge_operations)."""

import logging

import lanelet2
import pytest
from autoware_lanelet2_to_opendrive.preprocess_lanelet import (
    LaneletPreprocessor,
    MergeOperation,
    PreprocessOperation,
)


class TestMergeBasic:
    """Test that merge operations create the expected merged lanelet."""

    def test_merge_two_continuous_lanelets(self):
        """Merging two continuous lanelets creates one new lanelet with combined boundaries.

        With no explicit base_id, execute_merge_operations picks
        base_id = max_lanelet_id + 1 (here 202 + 1 = 203), and merge_lanelet
        assigns left=base_id+1, right=base_id+2, lanelet=base_id+3.
        """
        p1 = lanelet2.core.Point3d(1, 0, 0, 0)
        p2 = lanelet2.core.Point3d(2, 0, 1, 0)
        p3 = lanelet2.core.Point3d(3, 1, 0, 0)
        p4 = lanelet2.core.Point3d(4, 1, 1, 0)
        left1 = lanelet2.core.LineString3d(101, [p1, p2])
        right1 = lanelet2.core.LineString3d(102, [p3, p4])
        lanelet1 = lanelet2.core.Lanelet(201, left1, right1)

        # lanelet2_obj continues from lanelet1's endpoints (p2, p4).
        p5 = lanelet2.core.Point3d(5, 0, 2, 0)
        p6 = lanelet2.core.Point3d(6, 1, 2, 0)
        left2 = lanelet2.core.LineString3d(103, [p2, p5])
        right2 = lanelet2.core.LineString3d(104, [p4, p6])
        lanelet2_obj = lanelet2.core.Lanelet(202, left2, right2)

        lanelet_map = lanelet2.core.LaneletMap()
        lanelet_map.add(lanelet1)
        lanelet_map.add(lanelet2_obj)

        config = PreprocessOperation(
            input_map_path="dummy.osm",
            output_map_path="dummy_out.osm",
            mgrs_code="54SUE",
            merge_operations=[MergeOperation(lanelet_ids=[201, 202])],
        )
        preprocessor = LaneletPreprocessor(config)
        preprocessor.lanelet_map = lanelet_map
        new_map, log_entries = preprocessor.execute_merge_operations(lanelet_map)

        remaining_ids = {ll.id for ll in new_map.laneletLayer}
        assert remaining_ids == {206}

        merged = new_map.laneletLayer.get(206)
        assert [p.id for p in merged.leftBound] == [1, 2, 5]
        assert [p.id for p in merged.rightBound] == [3, 4, 6]

        assert len(log_entries) == 1
        assert log_entries[0].operation == "merge"
        assert log_entries[0].input_ids == [201, 202]
        assert log_entries[0].output_ids == [206]

    def test_merge_deletes_original_lanelets(self):
        """A successful merge removes the source lanelets from the resulting map.

        Note: nishishinjuku.yaml:15 has a comment claiming merge originals
        "are kept", but copy_map_excluding is used to build the new map
        (preprocess_lanelet.py execute_merge_operations, step 2), so the
        originals are in fact dropped. This test locks in the real behavior.
        """
        p1 = lanelet2.core.Point3d(11, 0, 0, 0)
        p2 = lanelet2.core.Point3d(12, 0, 1, 0)
        p3 = lanelet2.core.Point3d(13, 1, 0, 0)
        p4 = lanelet2.core.Point3d(14, 1, 1, 0)
        left1 = lanelet2.core.LineString3d(111, [p1, p2])
        right1 = lanelet2.core.LineString3d(112, [p3, p4])
        lanelet1 = lanelet2.core.Lanelet(211, left1, right1)

        p5 = lanelet2.core.Point3d(15, 0, 2, 0)
        p6 = lanelet2.core.Point3d(16, 1, 2, 0)
        left2 = lanelet2.core.LineString3d(113, [p2, p5])
        right2 = lanelet2.core.LineString3d(114, [p4, p6])
        lanelet2_obj = lanelet2.core.Lanelet(212, left2, right2)

        lanelet_map = lanelet2.core.LaneletMap()
        lanelet_map.add(lanelet1)
        lanelet_map.add(lanelet2_obj)

        config = PreprocessOperation(
            input_map_path="dummy.osm",
            output_map_path="dummy_out.osm",
            mgrs_code="54SUE",
            merge_operations=[MergeOperation(lanelet_ids=[211, 212])],
        )
        preprocessor = LaneletPreprocessor(config)
        preprocessor.lanelet_map = lanelet_map
        new_map, _log = preprocessor.execute_merge_operations(lanelet_map)

        remaining_ids = {ll.id for ll in new_map.laneletLayer}
        assert 211 not in remaining_ids
        assert 212 not in remaining_ids


class TestMergeMultipleOperations:
    """Test that multiple merge operations in one call get non-colliding IDs."""

    def test_multiple_merge_operations_produce_unique_ids(self):
        """Two independent merge operations must not collide on generated IDs."""
        # Pair A: lanelets 201, 202.
        p1 = lanelet2.core.Point3d(1, 0, 0, 0)
        p2 = lanelet2.core.Point3d(2, 0, 1, 0)
        p3 = lanelet2.core.Point3d(3, 1, 0, 0)
        p4 = lanelet2.core.Point3d(4, 1, 1, 0)
        left1 = lanelet2.core.LineString3d(101, [p1, p2])
        right1 = lanelet2.core.LineString3d(102, [p3, p4])
        lanelet_a1 = lanelet2.core.Lanelet(201, left1, right1)

        p5 = lanelet2.core.Point3d(5, 0, 2, 0)
        p6 = lanelet2.core.Point3d(6, 1, 2, 0)
        left2 = lanelet2.core.LineString3d(103, [p2, p5])
        right2 = lanelet2.core.LineString3d(104, [p4, p6])
        lanelet_a2 = lanelet2.core.Lanelet(202, left2, right2)

        # Pair B: lanelets 203, 204 (offset in x to avoid overlapping geometry).
        p7 = lanelet2.core.Point3d(7, 10, 0, 0)
        p8 = lanelet2.core.Point3d(8, 10, 1, 0)
        p9 = lanelet2.core.Point3d(9, 11, 0, 0)
        p10 = lanelet2.core.Point3d(10, 11, 1, 0)
        left3 = lanelet2.core.LineString3d(105, [p7, p8])
        right3 = lanelet2.core.LineString3d(106, [p9, p10])
        lanelet_b1 = lanelet2.core.Lanelet(203, left3, right3)

        p11 = lanelet2.core.Point3d(11, 10, 2, 0)
        p12 = lanelet2.core.Point3d(12, 11, 2, 0)
        left4 = lanelet2.core.LineString3d(107, [p8, p11])
        right4 = lanelet2.core.LineString3d(108, [p10, p12])
        lanelet_b2 = lanelet2.core.Lanelet(204, left4, right4)

        lanelet_map = lanelet2.core.LaneletMap()
        for ll in (lanelet_a1, lanelet_a2, lanelet_b1, lanelet_b2):
            lanelet_map.add(ll)

        config = PreprocessOperation(
            input_map_path="dummy.osm",
            output_map_path="dummy_out.osm",
            mgrs_code="54SUE",
            merge_operations=[
                MergeOperation(lanelet_ids=[201, 202]),
                MergeOperation(lanelet_ids=[203, 204]),
            ],
        )
        preprocessor = LaneletPreprocessor(config)
        preprocessor.lanelet_map = lanelet_map
        new_map, log_entries = preprocessor.execute_merge_operations(lanelet_map)

        # max_id=204 -> counter starts at 205; op1 base=205 -> id 208, counter -> 215;
        # op2 base=215 -> id 218.
        remaining_ids = {ll.id for ll in new_map.laneletLayer}
        assert remaining_ids == {208, 218}

        assert len(log_entries) == 2
        assert log_entries[0].output_ids == [208]
        assert log_entries[1].output_ids == [218]


class TestMergeInvalidId:
    """Test the dry_run-gated error handling for missing lanelet IDs."""

    def _build_single_lanelet_map(self):
        p1 = lanelet2.core.Point3d(1, 0, 0, 0)
        p2 = lanelet2.core.Point3d(2, 0, 1, 0)
        p3 = lanelet2.core.Point3d(3, 1, 0, 0)
        p4 = lanelet2.core.Point3d(4, 1, 1, 0)
        left1 = lanelet2.core.LineString3d(101, [p1, p2])
        right1 = lanelet2.core.LineString3d(102, [p3, p4])
        lanelet1 = lanelet2.core.Lanelet(201, left1, right1)

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
            merge_operations=[MergeOperation(lanelet_ids=[201, 999])],
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
            preprocessor.execute_merge_operations(lanelet_map)

        assert any("Merge operation failed" in r.message for r in caplog.records)
        # The map passed in is never mutated by a failed merge.
        assert {ll.id for ll in lanelet_map.laneletLayer} == {201}

    def test_invalid_id_dry_run_suppresses_raise(self, caplog):
        """A missing lanelet ID only warns (no raise) when dry_run is True."""
        lanelet_map = self._build_single_lanelet_map()
        config = PreprocessOperation(
            input_map_path="dummy.osm",
            output_map_path="dummy_out.osm",
            mgrs_code="54SUE",
            merge_operations=[MergeOperation(lanelet_ids=[201, 999])],
            dry_run=True,
        )
        preprocessor = LaneletPreprocessor(config)
        preprocessor.lanelet_map = lanelet_map

        with caplog.at_level(
            logging.WARNING, logger="autoware_lanelet2_to_opendrive.preprocess_lanelet"
        ):
            new_map, log_entries = preprocessor.execute_merge_operations(lanelet_map)

        assert any("Merge operation failed" in r.message for r in caplog.records)
        # No merge succeeded, so the original map is returned untouched.
        assert new_map is lanelet_map
        assert log_entries == []


class TestMergeValidateFlag:
    """Test that the `validate` flag controls continuity enforcement."""

    def _build_discontinuous_map(self):
        p1 = lanelet2.core.Point3d(1, 0, 0, 0)
        p2 = lanelet2.core.Point3d(2, 0, 1, 0)
        p3 = lanelet2.core.Point3d(3, 1, 0, 0)
        p4 = lanelet2.core.Point3d(4, 1, 1, 0)
        left1 = lanelet2.core.LineString3d(101, [p1, p2])
        right1 = lanelet2.core.LineString3d(102, [p3, p4])
        lanelet1 = lanelet2.core.Lanelet(301, left1, right1)

        # Far away from lanelet1's endpoints - not continuous.
        p5 = lanelet2.core.Point3d(5, 100, 100, 0)
        p6 = lanelet2.core.Point3d(6, 100, 101, 0)
        p7 = lanelet2.core.Point3d(7, 101, 100, 0)
        p8 = lanelet2.core.Point3d(8, 101, 101, 0)
        left2 = lanelet2.core.LineString3d(103, [p5, p6])
        right2 = lanelet2.core.LineString3d(104, [p7, p8])
        lanelet2_obj = lanelet2.core.Lanelet(302, left2, right2)

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
            merge_operations=[MergeOperation(lanelet_ids=[301, 302], validate=True)],
        )
        preprocessor = LaneletPreprocessor(config)
        preprocessor.lanelet_map = lanelet_map

        with pytest.raises(ValueError, match="not continuous"):
            preprocessor.execute_merge_operations(lanelet_map)

    def test_validate_false_allows_discontinuous_merge(self):
        """With validate=False, the continuity check is skipped and the merge succeeds."""
        lanelet_map = self._build_discontinuous_map()
        config = PreprocessOperation(
            input_map_path="dummy.osm",
            output_map_path="dummy_out.osm",
            mgrs_code="54SUE",
            merge_operations=[MergeOperation(lanelet_ids=[301, 302], validate=False)],
        )
        preprocessor = LaneletPreprocessor(config)
        preprocessor.lanelet_map = lanelet_map

        new_map, log_entries = preprocessor.execute_merge_operations(lanelet_map)

        # max_id=302 -> base=303 -> merged id=306.
        assert {ll.id for ll in new_map.laneletLayer} == {306}
        assert log_entries[0].output_ids == [306]


class TestYamlConfig:
    """Test YAML configuration round-tripping for merge operations."""

    def test_create_and_load_config(self, tmp_path):
        """Test creating and loading configuration with merge operations.

        Note: PreprocessOperation.to_yaml only serializes lanelet_ids,
        validate, and tolerance for merge_operations - base_id is not
        persisted, so it reverts to None (its default) after a round trip.
        """
        config = PreprocessOperation(
            input_map_path="test_input.osm",
            output_map_path="test_output.osm",
            mgrs_code="54SUE815501",
            merge_operations=[
                MergeOperation(
                    lanelet_ids=[100, 101, 102],
                    validate=False,
                    tolerance=0.01,
                    base_id=555,
                ),
                MergeOperation(lanelet_ids=[200, 201]),
            ],
        )

        yaml_path = tmp_path / "test_merge_config.yaml"
        config.to_yaml(yaml_path)

        loaded_config = PreprocessOperation.from_yaml(yaml_path)

        assert len(loaded_config.merge_operations) == 2
        assert loaded_config.merge_operations[0].lanelet_ids == [100, 101, 102]
        assert loaded_config.merge_operations[0].validate is False
        assert loaded_config.merge_operations[0].tolerance == 0.01
        # base_id is dropped by to_yaml, so it comes back as the default.
        assert loaded_config.merge_operations[0].base_id is None
        assert loaded_config.merge_operations[1].lanelet_ids == [200, 201]
        assert loaded_config.merge_operations[1].validate is True

    def test_empty_merge_operations(self, tmp_path):
        """Test configuration with no merge operations."""
        config = PreprocessOperation(
            input_map_path="test_input.osm",
            output_map_path="test_output.osm",
            mgrs_code="54SUE815501",
            merge_operations=[],
        )

        yaml_path = tmp_path / "test_empty_merge.yaml"
        config.to_yaml(yaml_path)

        loaded_config = PreprocessOperation.from_yaml(yaml_path)

        assert len(loaded_config.merge_operations) == 0
