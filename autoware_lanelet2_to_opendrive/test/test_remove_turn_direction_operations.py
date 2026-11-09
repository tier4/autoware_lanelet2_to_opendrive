"""Test module for remove turn_direction operation functionality.

Covers execute_remove_turn_direction_operations().
"""

import logging

import lanelet2
import pytest
from autoware_lanelet2_to_opendrive.preprocess_lanelet import (
    LaneletPreprocessor,
    PreprocessOperation,
    RemoveTurnDirectionOperation,
)


def _make_lanelet(lanelet_id: int, turn_direction: str | None) -> lanelet2.core.Lanelet:
    """Build a minimal lanelet, optionally tagged with a turn_direction attribute."""
    p1 = lanelet2.core.Point3d(lanelet2.core.getId(), 0, 0, 0)
    p2 = lanelet2.core.Point3d(lanelet2.core.getId(), 1, 0, 0)
    p3 = lanelet2.core.Point3d(lanelet2.core.getId(), 0, 1, 0)
    p4 = lanelet2.core.Point3d(lanelet2.core.getId(), 1, 1, 0)
    left_bound = lanelet2.core.LineString3d(lanelet2.core.getId(), [p1, p2])
    right_bound = lanelet2.core.LineString3d(lanelet2.core.getId(), [p3, p4])
    lanelet = lanelet2.core.Lanelet(lanelet_id, left_bound, right_bound)
    if turn_direction is not None:
        lanelet.attributes["turn_direction"] = turn_direction
    return lanelet


class TestRemoveTurnDirectionBasic:
    """Test the attribute-removal behavior on specific and all lanelets."""

    def test_removes_attribute_from_specified_lanelets(self):
        """Only the lanelets named in lanelet_ids lose the turn_direction attribute."""
        lanelet1 = _make_lanelet(701, "left")
        lanelet2_obj = _make_lanelet(702, "right")
        lanelet3 = _make_lanelet(703, "straight")

        lanelet_map = lanelet2.core.LaneletMap()
        for ll in (lanelet1, lanelet2_obj, lanelet3):
            lanelet_map.add(ll)

        config = PreprocessOperation(
            input_map_path="dummy.osm",
            output_map_path="dummy_out.osm",
            mgrs_code="54SUE",
            remove_turn_direction_operations=[
                RemoveTurnDirectionOperation(lanelet_ids=[701, 702])
            ],
        )
        preprocessor = LaneletPreprocessor(config)
        preprocessor.lanelet_map = lanelet_map
        new_map, log_entries = preprocessor.execute_remove_turn_direction_operations(
            lanelet_map
        )

        assert "turn_direction" not in new_map.laneletLayer.get(701).attributes
        assert "turn_direction" not in new_map.laneletLayer.get(702).attributes
        # Lanelet 703 was never named, so it keeps its own attribute untouched.
        assert new_map.laneletLayer.get(703).attributes["turn_direction"] == "straight"

        assert len(log_entries) == 1
        assert log_entries[0].operation == "remove_turn_direction"
        assert log_entries[0].input_ids == [701, 702]

    def test_skips_lanelet_without_the_attribute(self):
        """A named lanelet that never had turn_direction is silently skipped."""
        lanelet1 = _make_lanelet(701, "left")
        lanelet3 = _make_lanelet(703, None)  # No turn_direction attribute at all.

        lanelet_map = lanelet2.core.LaneletMap()
        lanelet_map.add(lanelet1)
        lanelet_map.add(lanelet3)

        config = PreprocessOperation(
            input_map_path="dummy.osm",
            output_map_path="dummy_out.osm",
            mgrs_code="54SUE",
            remove_turn_direction_operations=[
                RemoveTurnDirectionOperation(lanelet_ids=[701, 703])
            ],
        )
        preprocessor = LaneletPreprocessor(config)
        preprocessor.lanelet_map = lanelet_map
        new_map, log_entries = preprocessor.execute_remove_turn_direction_operations(
            lanelet_map
        )

        assert "turn_direction" not in new_map.laneletLayer.get(701).attributes
        assert "turn_direction" not in new_map.laneletLayer.get(703).attributes
        # Only the lanelet that actually had the attribute is recorded as affected.
        assert log_entries[0].input_ids == [701]

    def test_empty_lanelet_ids_targets_all_lanelets_in_map(self):
        """An empty lanelet_ids list means ALL lanelets in the map, not none."""
        lanelet1 = _make_lanelet(701, "left")
        lanelet2_obj = _make_lanelet(702, "right")
        lanelet3 = _make_lanelet(703, "straight")

        lanelet_map = lanelet2.core.LaneletMap()
        for ll in (lanelet1, lanelet2_obj, lanelet3):
            lanelet_map.add(ll)

        config = PreprocessOperation(
            input_map_path="dummy.osm",
            output_map_path="dummy_out.osm",
            mgrs_code="54SUE",
            remove_turn_direction_operations=[
                RemoveTurnDirectionOperation(lanelet_ids=[])
            ],
        )
        preprocessor = LaneletPreprocessor(config)
        preprocessor.lanelet_map = lanelet_map
        new_map, log_entries = preprocessor.execute_remove_turn_direction_operations(
            lanelet_map
        )

        for lanelet_id in (701, 702, 703):
            assert (
                "turn_direction" not in new_map.laneletLayer.get(lanelet_id).attributes
            )
        assert set(log_entries[0].input_ids) == {701, 702, 703}

    def test_returns_same_map_object_mutated_in_place(self):
        """Unlike merge/replace/remove, this operation mutates the map in place."""
        lanelet1 = _make_lanelet(701, "left")

        lanelet_map = lanelet2.core.LaneletMap()
        lanelet_map.add(lanelet1)

        config = PreprocessOperation(
            input_map_path="dummy.osm",
            output_map_path="dummy_out.osm",
            mgrs_code="54SUE",
            remove_turn_direction_operations=[
                RemoveTurnDirectionOperation(lanelet_ids=[701])
            ],
        )
        preprocessor = LaneletPreprocessor(config)
        preprocessor.lanelet_map = lanelet_map
        new_map, _log = preprocessor.execute_remove_turn_direction_operations(
            lanelet_map
        )

        assert new_map is lanelet_map


class TestRemoveTurnDirectionInvalidId:
    """Test that a missing lanelet ID always warns, regardless of dry_run."""

    @pytest.mark.parametrize("dry_run", [True, False])
    def test_invalid_id_warns_regardless_of_dry_run(self, caplog, dry_run):
        """A nonexistent lanelet ID in the list warns unconditionally (no dry_run gate)."""
        lanelet1 = _make_lanelet(701, "left")

        lanelet_map = lanelet2.core.LaneletMap()
        lanelet_map.add(lanelet1)

        config = PreprocessOperation(
            input_map_path="dummy.osm",
            output_map_path="dummy_out.osm",
            mgrs_code="54SUE",
            remove_turn_direction_operations=[
                RemoveTurnDirectionOperation(lanelet_ids=[701, 999])
            ],
            dry_run=dry_run,
        )
        preprocessor = LaneletPreprocessor(config)
        preprocessor.lanelet_map = lanelet_map

        with caplog.at_level(logging.WARNING):
            new_map, _log = preprocessor.execute_remove_turn_direction_operations(
                lanelet_map
            )

        assert any("Lanelet 999 not found in map" in r.message for r in caplog.records)
        # The valid lanelet in the same operation is still processed correctly.
        assert "turn_direction" not in new_map.laneletLayer.get(701).attributes


class TestYamlConfig:
    """Test YAML configuration round-tripping for remove turn_direction operations."""

    def test_create_and_load_config(self, tmp_path):
        """Test creating and loading configuration with specific and empty lanelet_ids."""
        config = PreprocessOperation(
            input_map_path="test_input.osm",
            output_map_path="test_output.osm",
            mgrs_code="54SUE815501",
            remove_turn_direction_operations=[
                RemoveTurnDirectionOperation(lanelet_ids=[]),
                RemoveTurnDirectionOperation(lanelet_ids=[600, 601, 602]),
            ],
        )

        yaml_path = tmp_path / "test_remove_turn_direction_config.yaml"
        config.to_yaml(yaml_path)

        loaded_config = PreprocessOperation.from_yaml(yaml_path)

        assert len(loaded_config.remove_turn_direction_operations) == 2
        # The empty list must round-trip as [], not be dropped or become None.
        assert loaded_config.remove_turn_direction_operations[0].lanelet_ids == []
        assert loaded_config.remove_turn_direction_operations[1].lanelet_ids == [
            600,
            601,
            602,
        ]

    def test_empty_remove_turn_direction_operations(self, tmp_path):
        """Test configuration with no remove turn_direction operations at all."""
        config = PreprocessOperation(
            input_map_path="test_input.osm",
            output_map_path="test_output.osm",
            mgrs_code="54SUE815501",
            remove_turn_direction_operations=[],
        )

        yaml_path = tmp_path / "test_empty_remove_turn_direction.yaml"
        config.to_yaml(yaml_path)

        loaded_config = PreprocessOperation.from_yaml(yaml_path)

        assert len(loaded_config.remove_turn_direction_operations) == 0
