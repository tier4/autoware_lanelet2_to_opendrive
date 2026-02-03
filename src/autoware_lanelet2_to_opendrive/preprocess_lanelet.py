"""Preprocessing script for Lanelet2 maps using configurable operations."""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import List, Optional, Union, Any
from pathlib import Path
import yaml  # type: ignore
import logging
from enum import Enum

# Import autoware extensions before lanelet2 to ensure proper registration
from autoware_lanelet2_extension_python.projection import MGRSProjector
import lanelet2

from .config import DEFAULT_CONFIG
from .lanelet import (
    merge_lanelets_from_ids,
    remove_lanelets,
    replace_lanelets,
    get_max_lanelet_id,
    validate_lanelet_continuity,
)
from .util import mgrs_to_lanelet2_origin

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
class PreprocessingOperation(ABC):
    """Base class for lanelet preprocessing operations with tolerance initialization.

    Provides common tolerance initialization logic from config. Subclasses must implement
    _get_default_tolerance() to specify their default tolerance value.
    """

    tolerance: Optional[float] = None

    def __post_init__(self):
        """Set default tolerance from config if not specified."""
        if self.tolerance is None:
            self.tolerance = self._get_default_tolerance()

    @abstractmethod
    def _get_default_tolerance(self) -> float:
        """Get the default tolerance for this operation type.

        Returns:
            The default tolerance value from configuration.
        """
        pass


@dataclass
class MergeOperation(PreprocessingOperation):
    """Configuration for merge operations."""

    lanelet_ids: List[int]
    validate: bool = True
    base_id: Optional[int] = None

    def _get_default_tolerance(self) -> float:
        """Get the default tolerance for merge operations."""
        return DEFAULT_CONFIG.preprocessing.merge_tolerance_default


@dataclass
class RemoveOperation:
    """Configuration for remove operations."""

    lanelet_ids: List[int]


@dataclass
class ReplaceOperation(PreprocessingOperation):
    """Configuration for replace operations."""

    lanelet_ids: List[int]
    validate: bool = True

    def _get_default_tolerance(self) -> float:
        """Get the default tolerance for replace operations."""
        return DEFAULT_CONFIG.preprocessing.replace_tolerance_default


@dataclass
class ValidateOperation(PreprocessingOperation):
    """Configuration for validate operations."""

    first_lanelet_id: int
    second_lanelet_id: int

    def _get_default_tolerance(self) -> float:
        """Get the default tolerance for validate operations."""
        return DEFAULT_CONFIG.preprocessing.validate_tolerance_default


@dataclass
class MovePointOperation:
    """Configuration for move point operations."""

    point_id: int
    new_x: float
    new_y: float
    new_z: Optional[float] = None  # If None, keep original z


@dataclass
class DeletePointOperation:
    """Configuration for delete point operations."""

    point_ids: List[int]


@dataclass
class RemoveLaneletOperation:
    """Configuration for removing entire lanelets from the map."""

    lanelet_ids: List[int]


@dataclass
class RemoveTurnDirectionOperation:
    """Configuration for removing turn_direction attribute from lanelets.

    If lanelet_ids is empty, the turn_direction attribute will be removed from ALL lanelets.
    Otherwise, only removes from the specified lanelet IDs.
    """

    lanelet_ids: List[int] = field(
        default_factory=list
    )  # Empty list means all lanelets


@dataclass
class LatLonOrigin:
    """Geographic origin specified by latitude and longitude.

    Attributes:
        lat: Latitude in degrees (WGS84)
        lon: Longitude in degrees (WGS84)
    """

    lat: float
    lon: float

    def __post_init__(self) -> None:
        """Validate latitude and longitude values."""
        if not -90 <= self.lat <= 90:
            raise ValueError(f"Latitude must be between -90 and 90, got {self.lat}")
        if not -180 <= self.lon <= 180:
            raise ValueError(f"Longitude must be between -180 and 180, got {self.lon}")


# Metadata for operation parsing - maps config key to operation class
OPERATION_TYPES = {
    "merge_operations": MergeOperation,
    "remove_operations": RemoveOperation,
    "replace_operations": ReplaceOperation,
    "validate_operations": ValidateOperation,
    "move_point_operations": MovePointOperation,
    "delete_point_operations": DeletePointOperation,
    "remove_lanelet_operations": RemoveLaneletOperation,
    "remove_turn_direction_operations": RemoveTurnDirectionOperation,
}


def parse_operations(config: dict, skip_empty: bool = False) -> dict:
    """Parse all operation types from config dictionary.

    Args:
        config: Configuration dictionary containing operation definitions
        skip_empty: If True, skip None/empty entries (for Hydra configs)

    Returns:
        Dictionary mapping operation type names to lists of operation instances
    """
    result = {}
    for config_key, op_class in OPERATION_TYPES.items():
        ops_config = config.get(config_key, []) or []
        if skip_empty:
            result[config_key] = [op_class(**op) for op in ops_config if op]
        else:
            result[config_key] = [op_class(**op) for op in ops_config]
    return result


@dataclass
class PreprocessOperation:
    """Main configuration class for preprocessing operations.

    This dataclass defines all preprocessing operations to be performed
    on a Lanelet2 map. It can be loaded from a YAML configuration file.

    The projection origin can be specified in two ways (mutually exclusive):
    1. Using MGRS code: Set mgrs_code to a valid MGRS grid reference
    2. Using exact lat/lon: Set both origin_lat and origin_lon

    Specifying both methods will raise a ValueError.
    """

    # Input/Output paths
    input_map_path: str
    output_map_path: str

    # Projection origin - specify EITHER mgrs_code OR origin (mutually exclusive)
    mgrs_code: Optional[str] = None  # MGRS code for projection
    origin: Optional[LatLonOrigin] = None  # Exact lat/lon origin

    # Operations to perform
    merge_operations: List[MergeOperation] = field(default_factory=list)
    remove_operations: List[RemoveOperation] = field(default_factory=list)
    replace_operations: List[ReplaceOperation] = field(default_factory=list)
    validate_operations: List[ValidateOperation] = field(default_factory=list)
    move_point_operations: List[MovePointOperation] = field(default_factory=list)
    delete_point_operations: List[DeletePointOperation] = field(default_factory=list)
    remove_lanelet_operations: List[RemoveLaneletOperation] = field(
        default_factory=list
    )
    remove_turn_direction_operations: List[RemoveTurnDirectionOperation] = field(
        default_factory=list
    )

    # Global settings
    dry_run: bool = False  # If True, only validate without saving
    verbose: bool = False  # Enable verbose logging

    # CARLA compatibility settings
    exclude_non_junction_signals: bool = (
        False  # If True, exclude signals not in junctions
    )

    def __post_init__(self) -> None:
        """Validate that origin is specified correctly after initialization."""
        self._validate_origin()

    def _validate_origin(self) -> None:
        """Validate that exactly one origin specification method is used.

        Raises:
            ValueError: If both mgrs_code and origin are specified, or if neither is specified
        """
        has_mgrs = self.mgrs_code is not None and self.mgrs_code != ""
        has_origin = self.origin is not None

        if has_mgrs and has_origin:
            raise ValueError(
                "Cannot specify both mgrs_code and origin. "
                "Use either MGRS code or exact lat/lon coordinates, not both."
            )

        if not has_mgrs and not has_origin:
            raise ValueError(
                "Must specify projection origin using either mgrs_code or origin."
            )

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

            # Option 1: Use MGRS code for projection origin
            mgrs_code: 54SUE815501

            # Option 2: Use exact lat/lon for projection origin (mutually exclusive with mgrs_code)
            # origin:
            #   lat: 35.6762
            #   lon: 139.6503

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

            remove_turn_direction_operations:
              - lanelet_ids: []  # Empty list = remove from ALL lanelets
              - lanelet_ids: [600, 601, 602]  # Remove from specific lanelets only

            dry_run: false
            verbose: true
        """
        yaml_path = Path(yaml_path)
        if not yaml_path.exists():
            raise FileNotFoundError(f"Configuration file not found: {yaml_path}")

        with open(yaml_path, "r") as f:
            config = yaml.safe_load(f)

        # Parse all operations using the metadata-driven approach
        operations = parse_operations(config)

        # Parse origin if specified
        origin = None
        origin_config = config.get("origin")
        if origin_config is not None:
            origin = LatLonOrigin(lat=origin_config["lat"], lon=origin_config["lon"])

        return cls(
            input_map_path=config["input_map_path"],
            output_map_path=config["output_map_path"],
            mgrs_code=config.get("mgrs_code"),
            origin=origin,
            merge_operations=operations["merge_operations"],
            remove_operations=operations["remove_operations"],
            replace_operations=operations["replace_operations"],
            validate_operations=operations["validate_operations"],
            move_point_operations=operations["move_point_operations"],
            delete_point_operations=operations["delete_point_operations"],
            remove_lanelet_operations=operations["remove_lanelet_operations"],
            remove_turn_direction_operations=operations[
                "remove_turn_direction_operations"
            ],
            dry_run=config.get("dry_run", False),
            verbose=config.get("verbose", False),
            exclude_non_junction_signals=config.get(
                "exclude_non_junction_signals", False
            ),
        )

    @classmethod
    def from_hydra_config(cls, cfg: Any) -> "PreprocessOperation":
        """Create PreprocessOperation from Hydra DictConfig.

        Args:
            cfg: Hydra DictConfig object containing the configuration

        Returns:
            PreprocessOperation instance configured from the Hydra config
        """
        from omegaconf import OmegaConf

        # Convert OmegaConf to dict for easier processing
        config = OmegaConf.to_container(cfg, resolve=True)

        # Parse all operations using the metadata-driven approach
        # Use skip_empty=True to handle Hydra's None/empty entries
        operations = parse_operations(config, skip_empty=True)

        # Parse origin if specified
        origin = None
        origin_config = config.get("origin")
        if origin_config is not None:
            origin = LatLonOrigin(lat=origin_config["lat"], lon=origin_config["lon"])

        # Support both mgrs_code (legacy) and mgrs_grid (current)
        mgrs_code = config.get("mgrs_code") or config.get("mgrs_grid") or None

        return cls(
            input_map_path=config.get("input_map_path") or "",
            output_map_path=config.get("output_map_path") or "",
            mgrs_code=mgrs_code,
            origin=origin,
            merge_operations=operations["merge_operations"],
            remove_operations=operations["remove_operations"],
            replace_operations=operations["replace_operations"],
            validate_operations=operations["validate_operations"],
            move_point_operations=operations["move_point_operations"],
            delete_point_operations=operations["delete_point_operations"],
            remove_lanelet_operations=operations["remove_lanelet_operations"],
            remove_turn_direction_operations=operations[
                "remove_turn_direction_operations"
            ],
            dry_run=config.get("dry_run", False),
            verbose=config.get("verbose", False),
            exclude_non_junction_signals=config.get(
                "exclude_non_junction_signals", False
            ),
        )

    def to_yaml(self, yaml_path: Union[str, Path]) -> None:
        """Save the configuration to a YAML file.

        Args:
            yaml_path: Path where to save the YAML configuration
        """
        yaml_path = Path(yaml_path)

        config: dict = {
            "input_map_path": self.input_map_path,
            "output_map_path": self.output_map_path,
            "dry_run": self.dry_run,
            "verbose": self.verbose,
            "exclude_non_junction_signals": self.exclude_non_junction_signals,
        }

        # Add projection origin (either mgrs_code or origin)
        if self.mgrs_code is not None:
            config["mgrs_code"] = self.mgrs_code
        elif self.origin is not None:
            config["origin"] = {"lat": self.origin.lat, "lon": self.origin.lon}

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

        if self.move_point_operations:
            config["move_point_operations"] = [
                {
                    "point_id": op.point_id,
                    "new_x": op.new_x,
                    "new_y": op.new_y,
                    "new_z": op.new_z,
                }
                for op in self.move_point_operations
            ]

        if self.delete_point_operations:
            config["delete_point_operations"] = [
                {"point_ids": op.point_ids} for op in self.delete_point_operations
            ]

        if self.remove_lanelet_operations:
            config["remove_lanelet_operations"] = [
                {"lanelet_ids": op.lanelet_ids} for op in self.remove_lanelet_operations
            ]

        if self.remove_turn_direction_operations:
            config["remove_turn_direction_operations"] = [
                {"lanelet_ids": op.lanelet_ids}
                for op in self.remove_turn_direction_operations
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
        self.projector: Optional[Any] = None  # MGRSProjector or UtmProjector

        if config.verbose:
            logging.getLogger().setLevel(logging.DEBUG)

    def load_map(self) -> lanelet2.core.LaneletMap:
        """Load the input Lanelet2 map using MGRS projection.

        The projection origin can be specified either via MGRS code or
        exact lat/lon coordinates (mutually exclusive).

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

        # Create origin from either MGRS code or lat/lon
        if self.config.mgrs_code is not None:
            logger.info(f"Using MGRS projection with code: {self.config.mgrs_code}")
            origin = mgrs_to_lanelet2_origin(self.config.mgrs_code)
        elif self.config.origin is not None:
            logger.info(
                f"Using lat/lon origin: lat={self.config.origin.lat}, "
                f"lon={self.config.origin.lon}"
            )
            origin = lanelet2.io.Origin(self.config.origin.lat, self.config.origin.lon)
        else:
            # This should never happen due to __post_init__ validation
            raise RuntimeError("No projection origin specified")

        self.projector = MGRSProjector(origin)
        self.lanelet_map = lanelet2.io.load(str(input_path), self.projector)

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

        # Use the same projector that was used to load the map
        lanelet2.io.write(str(output_path), lanelet_map, self.projector)

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

        # Get the maximum ID from the current map to ensure unique IDs
        max_id = get_max_lanelet_id(lanelet_map)
        current_id_counter = max_id + 1

        # Step 1: Perform all merge operations and collect merged lanelets
        logger.info(
            f"Step 1: Creating {len(self.config.merge_operations)} merged lanelets"
        )
        for i, op in enumerate(self.config.merge_operations):
            logger.info(
                f"  Merge operation {i + 1}/{len(self.config.merge_operations)}: "
                f"lanelets {op.lanelet_ids}"
            )

            try:
                # Use unique base_id for each merge operation
                unique_base_id = (
                    op.base_id if op.base_id is not None else current_id_counter
                )

                # Tolerance is guaranteed to be set by __post_init__
                assert op.tolerance is not None

                # Create merged lanelet
                merged_lanelet = merge_lanelets_from_ids(
                    lanelet_map,
                    op.lanelet_ids,
                    base_id=unique_base_id,
                    validate=op.validate,
                    tolerance=op.tolerance,
                )

                merged_lanelets.append(merged_lanelet)
                all_lanelets_to_remove.update(op.lanelet_ids)
                logger.info(f"    Created merged lanelet with ID: {merged_lanelet.id}")

                # Increment counter to ensure next merge gets unique IDs
                # Each merge uses base_id, base_id+1, base_id+2, base_id+3
                current_id_counter = unique_base_id + 10  # Leave gap for safety

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
                f"Executing remove operation {i + 1}/{len(self.config.remove_operations)}"
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
                f"Executing replace operation {i + 1}/{len(self.config.replace_operations)}"
            )
            logger.debug(f"  Replacing lanelets: {op.lanelet_ids}")

            try:
                # Tolerance is guaranteed to be set by __post_init__
                assert op.tolerance is not None

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
                f"Executing validate operation {i + 1}/{len(self.config.validate_operations)}"
            )

            try:
                # Get lanelets
                ll1 = lanelet_map.laneletLayer.get(op.first_lanelet_id)
                ll2 = lanelet_map.laneletLayer.get(op.second_lanelet_id)

                # Tolerance is guaranteed to be set by __post_init__
                assert op.tolerance is not None

                # Validate continuity
                is_continuous = validate_lanelet_continuity(ll1, ll2, op.tolerance)

                logger.info(
                    f"  Lanelets {op.first_lanelet_id} -> {op.second_lanelet_id}: "
                    f"{'CONTINUOUS' if is_continuous else 'DISCONTINUOUS'}"
                )

            except RuntimeError as e:
                logger.warning(f"  Validation failed - lanelet not found: {e}")

    def execute_move_point_operations(
        self, lanelet_map: lanelet2.core.LaneletMap
    ) -> lanelet2.core.LaneletMap:
        """Execute all move point operations.

        Args:
            lanelet_map: Current lanelet map

        Returns:
            Updated lanelet map
        """
        from .geometry import move_point_in_map

        for i, op in enumerate(self.config.move_point_operations):
            logger.info(
                f"Executing move point operation {i + 1}/{len(self.config.move_point_operations)}"
            )
            logger.info(
                f"  Moving point {op.point_id} to ({op.new_x}, {op.new_y}, {op.new_z})"
            )

            success = move_point_in_map(
                lanelet_map, op.point_id, op.new_x, op.new_y, op.new_z
            )

            if success:
                logger.info(f"  Successfully moved point {op.point_id}")
            else:
                logger.warning(f"  Failed to move point {op.point_id}")
                if not self.config.dry_run:
                    raise RuntimeError(f"Failed to move point {op.point_id}")

        return lanelet_map

    def execute_delete_point_operations(
        self, lanelet_map: lanelet2.core.LaneletMap
    ) -> lanelet2.core.LaneletMap:
        """Execute all delete point operations.

        Args:
            lanelet_map: Current lanelet map

        Returns:
            Updated lanelet map
        """
        from .geometry import delete_points_from_map

        for i, op in enumerate(self.config.delete_point_operations):
            logger.info(
                f"Executing delete point operation {i + 1}/{len(self.config.delete_point_operations)}"
            )
            logger.info(f"  Points to delete: {op.point_ids}")

            results = delete_points_from_map(lanelet_map, op.point_ids)

            successful = sum(1 for s in results.values() if s)
            logger.info(
                f"  Deleted {successful}/{len(op.point_ids)} points successfully"
            )

            if not self.config.dry_run and successful < len(op.point_ids):
                failed_points = [pid for pid, success in results.items() if not success]
                logger.warning(f"  Failed to delete points: {failed_points}")

        return lanelet_map

    def execute_remove_lanelet_operations(
        self, lanelet_map: lanelet2.core.LaneletMap
    ) -> lanelet2.core.LaneletMap:
        """Execute all remove lanelet operations.

        Args:
            lanelet_map: Current lanelet map

        Returns:
            Updated lanelet map with specified lanelets removed
        """
        if not self.config.remove_lanelet_operations:
            return lanelet_map

        # Collect all lanelet IDs to remove
        all_lanelets_to_remove = set()
        for op in self.config.remove_lanelet_operations:
            all_lanelets_to_remove.update(op.lanelet_ids)

        logger.info(f"Removing {len(all_lanelets_to_remove)} lanelets from map")

        # Create new map without the specified lanelets
        new_map = lanelet2.core.LaneletMap()

        # Copy all lanelets except those to be removed
        removed_count = 0
        for ll in lanelet_map.laneletLayer:
            if ll.id not in all_lanelets_to_remove:
                new_map.add(ll)
            else:
                removed_count += 1
                logger.debug(f"  Removed lanelet {ll.id}")

        # Copy regulatory elements
        for reg_elem in lanelet_map.regulatoryElementLayer:
            new_map.add(reg_elem)

        logger.info(
            f"Successfully removed {removed_count}/{len(all_lanelets_to_remove)} lanelets"
        )

        if removed_count < len(all_lanelets_to_remove):
            missing = all_lanelets_to_remove - {
                ll.id
                for ll in lanelet_map.laneletLayer
                if ll.id in all_lanelets_to_remove
            }
            if missing:
                logger.warning(f"  Lanelets not found: {missing}")

        return new_map

    def execute_remove_turn_direction_operations(
        self, lanelet_map: lanelet2.core.LaneletMap
    ) -> lanelet2.core.LaneletMap:
        """Execute all remove turn direction operations.

        Args:
            lanelet_map: Current lanelet map

        Returns:
            Updated lanelet map with turn_direction attributes removed
        """
        if not self.config.remove_turn_direction_operations:
            return lanelet_map

        for i, op in enumerate(self.config.remove_turn_direction_operations):
            logger.info(
                f"Executing remove turn_direction operation {i + 1}/"
                f"{len(self.config.remove_turn_direction_operations)}"
            )

            # Determine which lanelets to process
            if not op.lanelet_ids:
                # Empty list means remove from ALL lanelets
                logger.info("  Removing turn_direction attribute from ALL lanelets")
                target_lanelets = list(lanelet_map.laneletLayer)
            else:
                # Remove from specific lanelet IDs
                logger.info(
                    f"  Removing turn_direction attribute from {len(op.lanelet_ids)} lanelets"
                )
                target_lanelets = []
                for lanelet_id in op.lanelet_ids:
                    try:
                        ll = lanelet_map.laneletLayer.get(lanelet_id)
                        target_lanelets.append(ll)
                    except RuntimeError:
                        logger.warning(f"    Lanelet {lanelet_id} not found in map")

            # Remove turn_direction attribute from target lanelets
            removed_count = 0
            skipped_count = 0

            for ll in target_lanelets:
                if "turn_direction" in ll.attributes:
                    del ll.attributes["turn_direction"]
                    removed_count += 1
                    logger.debug(f"    Removed turn_direction from lanelet {ll.id}")
                else:
                    skipped_count += 1

            logger.info(
                f"  Removed turn_direction from {removed_count} lanelets "
                f"(skipped {skipped_count} without the attribute)"
            )

        return lanelet_map

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
        # 1. Point operations (modify individual points)
        if self.config.move_point_operations:
            logger.info("Running move point operations...")
            lanelet_map = self.execute_move_point_operations(lanelet_map)

        if self.config.delete_point_operations:
            logger.info("Running delete point operations...")
            lanelet_map = self.execute_delete_point_operations(lanelet_map)

        # 2. Validations (just for checking)
        if self.config.validate_operations:
            logger.info("Running validation checks...")
            self.execute_validate_operations(lanelet_map)

        # 3. Replace operations (modifies map)
        if self.config.replace_operations:
            logger.info("Running replace operations...")
            lanelet_map = self.execute_replace_operations(lanelet_map)

        # 4. Merge operations (adds new lanelets)
        if self.config.merge_operations:
            logger.info("Running merge operations...")
            lanelet_map = self.execute_merge_operations(lanelet_map)

        # 5. Remove operations (removes lanelets - old style)
        if self.config.remove_operations:
            logger.info("Running remove operations...")
            lanelet_map = self.execute_remove_operations(lanelet_map)

        # 6. Remove lanelet operations (removes entire lanelets)
        if self.config.remove_lanelet_operations:
            logger.info("Running remove lanelet operations...")
            lanelet_map = self.execute_remove_lanelet_operations(lanelet_map)

        # 7. Remove turn direction operations (removes turn_direction attributes)
        if self.config.remove_turn_direction_operations:
            logger.info("Running remove turn_direction operations...")
            lanelet_map = self.execute_remove_turn_direction_operations(lanelet_map)

        # Save the processed map
        self.save_map(lanelet_map)

        return lanelet_map


def main() -> int:
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
    # Projection origin options (mutually exclusive)
    origin_group = parser.add_mutually_exclusive_group()
    origin_group.add_argument(
        "--mgrs",
        type=str,
        help="MGRS code for projection (overrides config file)",
    )
    origin_group.add_argument(
        "--origin",
        type=str,
        metavar="LAT,LON",
        help="Lat/lon origin for projection as 'lat,lon' (overrides config file)",
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
    if args.mgrs:
        config.mgrs_code = args.mgrs
        config.origin = None  # Clear lat/lon origin if MGRS is specified
    elif args.origin:
        try:
            lat_str, lon_str = args.origin.split(",")
            config.origin = LatLonOrigin(lat=float(lat_str), lon=float(lon_str))
            config.mgrs_code = None  # Clear MGRS code if lat/lon is specified
        except ValueError:
            logger.error(
                f"Invalid --origin format. Expected 'lat,lon', got: {args.origin}"
            )
            return 1
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
