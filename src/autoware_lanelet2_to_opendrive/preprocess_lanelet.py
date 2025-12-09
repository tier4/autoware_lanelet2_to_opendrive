"""Preprocessing script for Lanelet2 maps using configurable operations."""

from dataclasses import dataclass, field
from typing import List, Optional, Union
from pathlib import Path
import yaml  # type: ignore
import lanelet2
import logging
from enum import Enum

from .lanelet import (
    merge_lanelets_from_ids,
    remove_lanelets,
    replace_lanelets,
    get_max_lanelet_id,
    validate_lanelet_continuity,
)

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class OperationType(Enum):
    """Enum for different preprocessing operation types."""

    MERGE = "merge"
    REMOVE = "remove"
    REPLACE = "replace"
    VALIDATE = "validate"


@dataclass
class MergeOperation:
    """Configuration for merge operations."""

    lanelet_ids: List[int]
    validate: bool = True
    tolerance: float = 1e-3
    base_id: Optional[int] = None


@dataclass
class RemoveOperation:
    """Configuration for remove operations."""

    lanelet_ids: List[int]


@dataclass
class ReplaceOperation:
    """Configuration for replace operations."""

    lanelet_ids: List[int]
    validate: bool = True
    tolerance: float = 1e-3


@dataclass
class ValidateOperation:
    """Configuration for validate operations."""

    first_lanelet_id: int
    second_lanelet_id: int
    tolerance: float = 1e-3


@dataclass
class PreprocessOperation:
    """Main configuration class for preprocessing operations.

    This dataclass defines all preprocessing operations to be performed
    on a Lanelet2 map. It can be loaded from a YAML configuration file.
    """

    # Input/Output paths
    input_map_path: str
    output_map_path: str

    # Operations to perform
    merge_operations: List[MergeOperation] = field(default_factory=list)
    remove_operations: List[RemoveOperation] = field(default_factory=list)
    replace_operations: List[ReplaceOperation] = field(default_factory=list)
    validate_operations: List[ValidateOperation] = field(default_factory=list)

    # Global settings
    dry_run: bool = False  # If True, only validate without saving
    verbose: bool = False  # Enable verbose logging

    @classmethod
    def from_yaml(cls, yaml_path: Union[str, Path]) -> "PreprocessOperation":
        """Load preprocessing configuration from a YAML file.

        Args:
            yaml_path: Path to the YAML configuration file

        Returns:
            PreprocessOperation instance configured from the YAML file

        Example YAML format:
            input_map_path: /path/to/input.osm
            output_map_path: /path/to/output.osm

            merge_operations:
              - lanelet_ids: [100, 101, 102]
                validate: true
                tolerance: 0.001
              - lanelet_ids: [200, 201]
                validate: false

            remove_operations:
              - lanelet_ids: [300, 301]

            replace_operations:
              - lanelet_ids: [400, 401, 402]
                validate: true

            validate_operations:
              - first_lanelet_id: 500
                second_lanelet_id: 501
                tolerance: 0.01

            dry_run: false
            verbose: true
        """
        yaml_path = Path(yaml_path)
        if not yaml_path.exists():
            raise FileNotFoundError(f"Configuration file not found: {yaml_path}")

        with open(yaml_path, "r") as f:
            config = yaml.safe_load(f)

        # Parse operations
        merge_ops = []
        for op in config.get("merge_operations", []):
            merge_ops.append(MergeOperation(**op))

        remove_ops = []
        for op in config.get("remove_operations", []):
            remove_ops.append(RemoveOperation(**op))

        replace_ops = []
        for op in config.get("replace_operations", []):
            replace_ops.append(ReplaceOperation(**op))

        validate_ops = []
        for op in config.get("validate_operations", []):
            validate_ops.append(ValidateOperation(**op))

        return cls(
            input_map_path=config["input_map_path"],
            output_map_path=config["output_map_path"],
            merge_operations=merge_ops,
            remove_operations=remove_ops,
            replace_operations=replace_ops,
            validate_operations=validate_ops,
            dry_run=config.get("dry_run", False),
            verbose=config.get("verbose", False),
        )

    def to_yaml(self, yaml_path: Union[str, Path]) -> None:
        """Save the configuration to a YAML file.

        Args:
            yaml_path: Path where to save the YAML configuration
        """
        yaml_path = Path(yaml_path)

        config = {
            "input_map_path": self.input_map_path,
            "output_map_path": self.output_map_path,
            "dry_run": self.dry_run,
            "verbose": self.verbose,
        }

        # Convert operations to dict format
        if self.merge_operations:
            config["merge_operations"] = [
                {
                    "lanelet_ids": op.lanelet_ids,
                    "validate": op.validate,
                    "tolerance": op.tolerance,
                }
                for op in self.merge_operations
            ]

        if self.remove_operations:
            config["remove_operations"] = [
                {"lanelet_ids": op.lanelet_ids} for op in self.remove_operations
            ]

        if self.replace_operations:
            config["replace_operations"] = [
                {
                    "lanelet_ids": op.lanelet_ids,
                    "validate": op.validate,
                    "tolerance": op.tolerance,
                }
                for op in self.replace_operations
            ]

        if self.validate_operations:
            config["validate_operations"] = [
                {
                    "first_lanelet_id": op.first_lanelet_id,
                    "second_lanelet_id": op.second_lanelet_id,
                    "tolerance": op.tolerance,
                }
                for op in self.validate_operations
            ]

        with open(yaml_path, "w") as f:
            yaml.dump(config, f, default_flow_style=False, sort_keys=False)


class LaneletPreprocessor:
    """Preprocessor class that executes operations on Lanelet2 maps."""

    def __init__(self, config: PreprocessOperation):
        """Initialize the preprocessor with a configuration.

        Args:
            config: PreprocessOperation configuration
        """
        self.config = config
        self.lanelet_map: Optional[lanelet2.core.LaneletMap] = None

        if config.verbose:
            logging.getLogger().setLevel(logging.DEBUG)

    def load_map(self) -> lanelet2.core.LaneletMap:
        """Load the input Lanelet2 map.

        Returns:
            Loaded LaneletMap

        Raises:
            FileNotFoundError: If input map file doesn't exist
            RuntimeError: If map loading fails
        """
        input_path = Path(self.config.input_map_path)
        if not input_path.exists():
            raise FileNotFoundError(f"Input map not found: {input_path}")

        logger.info(f"Loading map from: {input_path}")

        # Load using lanelet2 io
        projector = lanelet2.projection.UtmProjector(lanelet2.io.Origin(0, 0))
        self.lanelet_map = lanelet2.io.load(str(input_path), projector)

        logger.info(f"Loaded map with {len(self.lanelet_map.laneletLayer)} lanelets")
        return self.lanelet_map

    def save_map(self, lanelet_map: lanelet2.core.LaneletMap) -> None:
        """Save the processed Lanelet2 map.

        Args:
            lanelet_map: The processed LaneletMap to save
        """
        if self.config.dry_run:
            logger.info("Dry run mode - skipping save")
            return

        output_path = Path(self.config.output_map_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)

        logger.info(f"Saving map to: {output_path}")

        # Save using lanelet2 io
        projector = lanelet2.projection.UtmProjector(lanelet2.io.Origin(0, 0))
        lanelet2.io.write(str(output_path), lanelet_map, projector)

        logger.info(f"Saved map with {len(lanelet_map.laneletLayer)} lanelets")

    def execute_merge_operations(
        self, lanelet_map: lanelet2.core.LaneletMap
    ) -> lanelet2.core.LaneletMap:
        """Execute all merge operations.

        First performs all merges to create new lanelets, then removes the original
        lanelets that were merged.

        Args:
            lanelet_map: Current lanelet map

        Returns:
            Updated lanelet map
        """
        # Track all lanelets to be removed after merging
        all_lanelets_to_remove = set()
        merged_lanelets = []

        # Step 1: Perform all merge operations and collect merged lanelets
        logger.info(
            f"Step 1: Creating {len(self.config.merge_operations)} merged lanelets"
        )
        for i, op in enumerate(self.config.merge_operations):
            logger.info(
                f"  Merge operation {i+1}/{len(self.config.merge_operations)}: "
                f"lanelets {op.lanelet_ids}"
            )

            try:
                # Create merged lanelet
                merged_lanelet = merge_lanelets_from_ids(
                    lanelet_map,
                    op.lanelet_ids,
                    base_id=op.base_id,
                    validate=op.validate,
                    tolerance=op.tolerance,
                )

                merged_lanelets.append(merged_lanelet)
                all_lanelets_to_remove.update(op.lanelet_ids)
                logger.info(f"    Created merged lanelet with ID: {merged_lanelet.id}")

            except ValueError as e:
                logger.warning(f"    Merge operation failed: {e}")
                if not self.config.dry_run:
                    raise

        # Step 2: Remove original lanelets and add merged ones
        if merged_lanelets:
            logger.info(
                f"Step 2: Removing {len(all_lanelets_to_remove)} original lanelets"
            )

            # Create new map with merged lanelets replacing originals
            new_map = lanelet2.core.LaneletMap()

            # Copy all lanelets except those that were merged
            for ll in lanelet_map.laneletLayer:
                if ll.id not in all_lanelets_to_remove:
                    new_map.add(ll)

            # Add all the merged lanelets
            for merged_lanelet in merged_lanelets:
                new_map.add(merged_lanelet)

            # Copy regulatory elements
            for reg_elem in lanelet_map.regulatoryElementLayer:
                new_map.add(reg_elem)

            logger.info(
                f"  Replaced {len(all_lanelets_to_remove)} original lanelets "
                f"with {len(merged_lanelets)} merged lanelets"
            )

            return new_map

        return lanelet_map

    def execute_remove_operations(
        self, lanelet_map: lanelet2.core.LaneletMap
    ) -> lanelet2.core.LaneletMap:
        """Execute all remove operations.

        Args:
            lanelet_map: Current lanelet map

        Returns:
            Updated lanelet map
        """
        for i, op in enumerate(self.config.remove_operations):
            logger.info(
                f"Executing remove operation {i+1}/{len(self.config.remove_operations)}"
            )
            logger.debug(f"  Removing lanelets: {op.lanelet_ids}")

            # Remove operations return a new map
            lanelet_map = remove_lanelets(lanelet_map, op.lanelet_ids)
            logger.info(f"  Removed {len(op.lanelet_ids)} lanelets")

        return lanelet_map

    def execute_replace_operations(
        self, lanelet_map: lanelet2.core.LaneletMap
    ) -> lanelet2.core.LaneletMap:
        """Execute all replace operations.

        Args:
            lanelet_map: Current lanelet map

        Returns:
            Updated lanelet map
        """
        for i, op in enumerate(self.config.replace_operations):
            logger.info(
                f"Executing replace operation {i+1}/{len(self.config.replace_operations)}"
            )
            logger.debug(f"  Replacing lanelets: {op.lanelet_ids}")

            try:
                # Replace operations return a new map
                lanelet_map = replace_lanelets(
                    lanelet_map,
                    op.lanelet_ids,
                    validate=op.validate,
                    tolerance=op.tolerance,
                )

                # Find the new merged lanelet ID (should be max ID)
                max_id = get_max_lanelet_id(lanelet_map)
                logger.info(f"  Replaced with new lanelet ID: {max_id}")

            except ValueError as e:
                logger.warning(f"  Replace operation failed: {e}")
                if not self.config.dry_run:
                    raise

        return lanelet_map

    def execute_validate_operations(
        self, lanelet_map: lanelet2.core.LaneletMap
    ) -> None:
        """Execute all validate operations.

        Args:
            lanelet_map: Current lanelet map
        """
        for i, op in enumerate(self.config.validate_operations):
            logger.info(
                f"Executing validate operation {i+1}/{len(self.config.validate_operations)}"
            )

            try:
                # Get lanelets
                ll1 = lanelet_map.laneletLayer.get(op.first_lanelet_id)
                ll2 = lanelet_map.laneletLayer.get(op.second_lanelet_id)

                # Validate continuity
                is_continuous = validate_lanelet_continuity(ll1, ll2, op.tolerance)

                logger.info(
                    f"  Lanelets {op.first_lanelet_id} -> {op.second_lanelet_id}: "
                    f"{'CONTINUOUS' if is_continuous else 'DISCONTINUOUS'}"
                )

            except RuntimeError as e:
                logger.warning(f"  Validation failed - lanelet not found: {e}")

    def process(self) -> lanelet2.core.LaneletMap:
        """Execute all preprocessing operations.

        Returns:
            The processed LaneletMap

        Raises:
            FileNotFoundError: If input file doesn't exist
            RuntimeError: If processing fails
        """
        # Load the map
        lanelet_map = self.load_map()

        # Execute operations in order
        # 1. Validations (just for checking)
        if self.config.validate_operations:
            logger.info("Running validation checks...")
            self.execute_validate_operations(lanelet_map)

        # 2. Replace operations (modifies map)
        if self.config.replace_operations:
            logger.info("Running replace operations...")
            lanelet_map = self.execute_replace_operations(lanelet_map)

        # 3. Merge operations (adds new lanelets)
        if self.config.merge_operations:
            logger.info("Running merge operations...")
            lanelet_map = self.execute_merge_operations(lanelet_map)

        # 4. Remove operations (removes lanelets)
        if self.config.remove_operations:
            logger.info("Running remove operations...")
            lanelet_map = self.execute_remove_operations(lanelet_map)

        # Save the processed map
        self.save_map(lanelet_map)

        return lanelet_map


def main():
    """Main function for command-line usage."""
    import argparse

    parser = argparse.ArgumentParser(
        description="Preprocess Lanelet2 maps using configurable operations"
    )
    parser.add_argument(
        "config",
        type=str,
        help="Path to YAML configuration file",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Run without saving the output (validation only)",
    )
    parser.add_argument(
        "--verbose",
        "-v",
        action="store_true",
        help="Enable verbose logging",
    )
    parser.add_argument(
        "--output-config",
        type=str,
        help="Save the loaded configuration to a new YAML file",
    )

    args = parser.parse_args()

    # Load configuration
    try:
        config = PreprocessOperation.from_yaml(args.config)
    except Exception as e:
        logger.error(f"Failed to load configuration: {e}")
        return 1

    # Override settings from command line
    if args.dry_run:
        config.dry_run = True
    if args.verbose:
        config.verbose = True

    # Save configuration if requested
    if args.output_config:
        config.to_yaml(args.output_config)
        logger.info(f"Saved configuration to: {args.output_config}")

    # Process the map
    try:
        preprocessor = LaneletPreprocessor(config)
        processed_map = preprocessor.process()

        logger.info("Preprocessing completed successfully")
        logger.info(f"Final map has {len(processed_map.laneletLayer)} lanelets")

    except Exception as e:
        logger.error(f"Preprocessing failed: {e}")
        return 1

    return 0


if __name__ == "__main__":
    exit(main())
