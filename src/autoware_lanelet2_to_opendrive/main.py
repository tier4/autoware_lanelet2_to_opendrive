#!/usr/bin/env python3
"""Main script to convert Lanelet2 maps to OpenDRIVE format."""

import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple
import tempfile
import logging
from datetime import datetime

import hydra
from omegaconf import DictConfig, OmegaConf

# Import autoware extensions before loading maps to ensure proper registration
# The order matters: projection module must be imported to register extensions
from autoware_lanelet2_extension_python.projection import MGRSProjector
import lanelet2

from autoware_lanelet2_to_opendrive.util import (
    mgrs_to_lanelet2_origin,
    mgrs_grid_with_offset_to_lanelet2_origin,
    latlon_to_lanelet2_origin,
    mgrs_to_proj_string,
    RoadLaneletMapping,
)
from autoware_lanelet2_to_opendrive.config import COORDINATE_OFFSET
from autoware_lanelet2_to_opendrive.preprocess_lanelet import (
    PreprocessOperation,
    LaneletPreprocessor,
)

from autoware_lanelet2_to_opendrive.opendrive.opendrive_dataclass import (
    OpenDRIVE,
    Header,
    save_opendrive_to_file,
)
from autoware_lanelet2_to_opendrive.opendrive.road import Road
from autoware_lanelet2_to_opendrive.opendrive.junction import Junction
from autoware_lanelet2_to_opendrive.opendrive.signals_and_controllers import (
    SignalsAndControllers,
)
from autoware_lanelet2_to_opendrive.conversion_config import (
    ConversionConfig,
    OriginSpec,
    ParamPoly3Config,
    WidthEstimationConfig,
)

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def load_lanelet2_map(
    lanelet2_path: Path, origin: lanelet2.io.Origin
) -> lanelet2.core.LaneletMap:
    """
    Load Lanelet2 map from file with specified origin projection.

    Args:
        lanelet2_path: Path to the Lanelet2 OSM file
        origin: lanelet2.io.Origin object specifying the map origin

    Returns:
        Loaded Lanelet2 map

    Raises:
        FileNotFoundError: If the Lanelet2 file doesn't exist
        Exception: If map loading fails
    """
    if not lanelet2_path.exists():
        raise FileNotFoundError(f"Lanelet2 file not found: {lanelet2_path}")

    try:
        projector = MGRSProjector(origin)
        lanelet_map = lanelet2.io.load(str(lanelet2_path), projector)
        print(f"Successfully loaded Lanelet2 map: {lanelet2_path}")
        print(f"  - Lanelets: {len(lanelet_map.laneletLayer)}")
        print(f"  - Linestrings: {len(lanelet_map.lineStringLayer)}")
        print(f"  - Points: {len(lanelet_map.pointLayer)}")
        return lanelet_map
    except Exception as e:
        raise Exception(f"Failed to load Lanelet2 map: {e}")


class _Lanelet2ToOpenDRIVEConverter:
    """
    Helper class for converting Lanelet2 maps to OpenDRIVE format.

    This class breaks down the conversion process into focused, testable methods.
    """

    def __init__(
        self,
        lanelet_map: lanelet2.core.LaneletMap,
        config: ConversionConfig,
        mgrs_code: Optional[str] = None,
    ):
        """
        Initialize the converter.

        Args:
            lanelet_map: Loaded Lanelet2 map
            config: ConversionConfig object containing all conversion parameters
            mgrs_code: Optional MGRS code for generating geoReference PROJ string
        """
        self.lanelet_map = lanelet_map
        self.config = config
        self.mgrs_code = mgrs_code

    def _build_regular_roads(
        self,
    ) -> Tuple[List[Road], Dict[int, int]]:
        """
        Build roads from non-junction lanelets.

        Returns:
            Tuple of:
                - List of Road objects for non-junction lanelets
                - Dictionary mapping lanelet ID to road ID for regular roads
        """
        from autoware_lanelet2_to_opendrive.junction import (
            filter_lanelets_outside_junction,
        )
        from autoware_lanelet2_to_opendrive.util import (
            find_adjacent_groups,
            filter_lanelets_by_subtype,
        )

        print("\n=== Building regular roads ===")
        regular_roads = Road.construct_from_lanelet_map(
            self.lanelet_map,
            traffic_rule=self.config.traffic_rule,
            parampoly3_config=self.config.parampoly3,
            width_config=self.config.width_estimation,
        )

        # Build lanelet-to-road mapping for regular roads
        all_lanelets = list(self.lanelet_map.laneletLayer)
        road_lanelets = filter_lanelets_outside_junction(
            filter_lanelets_by_subtype(all_lanelets, ["road"])
        )
        adjacent_groups = find_adjacent_groups(self.lanelet_map, set(road_lanelets))

        lanelet_to_road_id: Dict[int, int] = {}
        for road_id, adjacent_group in enumerate(adjacent_groups):
            for lanelet in adjacent_group:
                lanelet_to_road_id[lanelet.id] = road_id

        return regular_roads, lanelet_to_road_id

    def _build_junction_structure(
        self,
        regular_roads: List[Road],
        lanelet_to_road_id: Dict[int, int],
    ) -> Tuple[
        List[Road],
        List[Junction],
        Dict[int, List[int]],
        Dict[int, int],
        List[lanelet2.core.Lanelet],
    ]:
        """
        Build junction structure from junction lanelets.

        Args:
            regular_roads: Already-built regular roads
            lanelet_to_road_id: Existing lanelet-to-road mapping from regular roads

        Returns:
            Tuple of:
                - List of connecting Road objects
                - List of Junction objects
                - Dictionary mapping junction ID to connecting road IDs
                - Dictionary mapping junction lanelet IDs to road IDs
                - List of junction lanelets
        """
        from autoware_lanelet2_to_opendrive.junction import (
            filter_lanelets_inside_junction,
            find_junction_groups,
        )

        # Get junction groups
        print("\n=== Finding junctions ===")
        all_lanelets = list(self.lanelet_map.laneletLayer)
        junction_lanelets = filter_lanelets_inside_junction(all_lanelets)
        junction_groups = find_junction_groups(junction_lanelets)
        print(f"Found {len(junction_groups)} junctions")

        # Create connecting roads
        print("\n=== Building connecting roads inside junctions ===")
        starting_junction_road_id = len(regular_roads)
        junction_id_offset = self.config.junction_id_offset

        (
            connecting_roads,
            junction_to_roads,
            junction_lanelet_to_road,
        ) = Road.construct_connecting_roads_from_junctions(
            lanelet_map=self.lanelet_map,
            junction_groups=junction_groups,
            starting_road_id=starting_junction_road_id,
            junction_id_offset=junction_id_offset,
            traffic_rule=self.config.traffic_rule,
            parampoly3_config=self.config.parampoly3,
            width_config=self.config.width_estimation,
        )

        # Merge lanelet-to-road mappings
        lanelet_to_road_id.update(junction_lanelet_to_road)

        # Build junctions with connections
        print("\n=== Building junction connections ===")
        print(
            f"Using junction ID offset: {junction_id_offset} "
            f"(junction IDs will be {junction_id_offset}+)"
        )

        junctions = []
        for junction_index, junction_group in enumerate(junction_groups):
            junction_id = junction_index + junction_id_offset

            # Create base junction
            junction = Junction.construct_from_lanelet_groups(
                junction_id=junction_id,
                lanelet_group=junction_group,
            )

            # Build connections for this junction
            connecting_road_ids = junction_to_roads.get(junction_id, [])
            all_roads_for_junction = regular_roads + connecting_roads
            connections = Junction.build_connections_from_roads(
                lanelet_map=self.lanelet_map,
                junction_lanelet_group=junction_group,
                junction_id=junction_id,
                lanelet_to_road_id=lanelet_to_road_id,
                connecting_road_ids=connecting_road_ids,
                roads=all_roads_for_junction,
            )

            junction.connections = connections
            junctions.append(junction)

        print(
            f"Built {len(junctions)} junctions with {sum(len(j.connections) for j in junctions)} total connections"
        )

        return (
            connecting_roads,
            junctions,
            junction_to_roads,
            junction_lanelet_to_road,
            junction_lanelets,
        )

    def _build_road_lanelet_mappings(
        self,
        lanelet_to_road_id: Dict[int, int],
    ) -> RoadLaneletMapping:
        """
        Create bidirectional mappings between roads and lanelets.

        Args:
            lanelet_to_road_id: Dictionary mapping lanelet IDs to road IDs

        Returns:
            RoadLaneletMapping dataclass with bidirectional maps
        """
        print("\n=== Building Road-Lanelet mapping ===")

        # Build reverse mapping (road_id -> list of lanelet IDs)
        road_to_lanelet_ids: Dict[int, List[int]] = {}
        for lanelet_id, road_id in lanelet_to_road_id.items():
            if road_id not in road_to_lanelet_ids:
                road_to_lanelet_ids[road_id] = []
            road_to_lanelet_ids[road_id].append(lanelet_id)

        # Sort lanelet IDs for each road for consistency
        for road_id in road_to_lanelet_ids:
            road_to_lanelet_ids[road_id].sort()

        print(
            f"Created mapping for {len(road_to_lanelet_ids)} roads covering {len(lanelet_to_road_id)} lanelets"
        )

        return RoadLaneletMapping(
            road_to_lanelets=road_to_lanelet_ids,
            lanelet_to_road=lanelet_to_road_id,
        )

    def _setup_connections(
        self,
        all_roads: List[Road],
        connecting_roads: List[Road],
        road_to_lanelet_ids: Dict[int, List[int]],
        lanelet_to_road_id: Dict[int, int],
        junctions: List[Junction],
    ) -> None:
        """
        Set up predecessor/successor connections for roads and lanes.

        Args:
            all_roads: All roads (regular + connecting)
            connecting_roads: Connecting roads only
            road_to_lanelet_ids: Dictionary mapping road IDs to lanelet IDs
            lanelet_to_road_id: Dictionary mapping lanelet IDs to road IDs
            junctions: All junctions
        """
        # Set road links for connecting roads
        print("\n=== Building road links for connecting roads ===")
        Road.set_connecting_road_links(
            lanelet_map=self.lanelet_map,
            connecting_roads=connecting_roads,
            lanelet_to_road_id=lanelet_to_road_id,
            road_to_lanelet_ids=road_to_lanelet_ids,
        )

        # Set junction links for incoming roads
        print("\n=== Setting junction links for incoming roads ===")
        Road.set_incoming_road_junction_links(
            roads=all_roads,
            junctions=junctions,
        )

        # Set lane links for all roads
        print("\n=== Building lane links for all roads ===")
        Road.set_all_lane_links(self.lanelet_map, all_roads)

    def _extract_and_assign_signals(
        self,
        all_roads: List[Road],
        mapping: RoadLaneletMapping,
        junction_lanelets: List[lanelet2.core.Lanelet],
    ) -> SignalsAndControllers:
        """
        Extract traffic signals and assign to roads.

        Args:
            all_roads: All roads
            mapping: Road-lanelet bidirectional mapping
            junction_lanelets: List of junction lanelets

        Returns:
            SignalsAndControllers object with all signals and controllers
        """
        print("\n=== Extracting signals and controllers ===")

        # Get junction lanelet IDs for filtering
        junction_lanelet_ids = {ll.id for ll in junction_lanelets}
        if self.config.exclude_non_junction_signals:
            print(
                f"CARLA compatibility mode: excluding signals not in {len(junction_lanelet_ids)} junction lanelets"
            )

        # Extract signals and controllers
        signals_and_controllers = SignalsAndControllers.construct_from_lanelet_map(
            lanelet_map=self.lanelet_map,
            road_lanelet_mapping=mapping,
            roads=all_roads,
            exclude_non_junction_signals=self.config.exclude_non_junction_signals,
            junction_lanelet_ids=junction_lanelet_ids,
        )
        print(
            f"Extracted {len(signals_and_controllers.signals)} signals and "
            f"{len(signals_and_controllers.controllers)} controllers"
        )

        # Assign signals to roads
        print("\n=== Assigning signals to roads ===")
        road_signals: Dict[int, List] = {}
        for signal in signals_and_controllers.signals:
            signal_road_id: Optional[int] = (
                signals_and_controllers.signal_to_road_id.get(signal.id)
            )
            if signal_road_id is not None:
                if signal_road_id not in road_signals:
                    road_signals[signal_road_id] = []
                road_signals[signal_road_id].append(signal)

        # Assign signals to road objects
        signals_assigned_count = 0
        for road in all_roads:
            if road.id in road_signals:
                road.signals = road_signals[road.id]
                signals_assigned_count += len(road.signals)

        print(f"Assigned {signals_assigned_count} signals to {len(road_signals)} roads")

        return signals_and_controllers

    def _assign_controllers_to_junctions(
        self,
        signals_and_controllers: SignalsAndControllers,
        junctions: List[Junction],
        all_roads: List[Road],
    ) -> None:
        """
        Create controllers and assign to junctions.

        Args:
            signals_and_controllers: All extracted signals and controllers
            junctions: All junctions
            all_roads: All roads
        """
        print("\n=== Associating controllers with junctions ===")

        controllers_assigned_count = 0
        for junction in junctions:
            # Get all road IDs related to this junction
            junction_incoming_road_ids = {
                conn.incoming_road for conn in junction.connections
            }
            junction_connecting_road_ids = {
                conn.connecting_road for conn in junction.connections
            }

            # Include roads that belong to this junction by attribute
            junction_roads_by_attribute = {
                road.id for road in all_roads if road.junction == junction.id
            }

            junction_related_road_ids = (
                junction_incoming_road_ids
                | junction_connecting_road_ids
                | junction_roads_by_attribute
            )

            # Find controllers whose signals are on roads related to this junction
            junction_controller_ids: List[int] = []
            for controller in signals_and_controllers.controllers:
                if controller.controls:
                    # Get road IDs for all signals controlled by this controller
                    controller_road_ids = set()
                    for control_entry in controller.controls:
                        signal_road_id = signals_and_controllers.signal_to_road_id.get(
                            control_entry.signal_id
                        )
                        if signal_road_id is not None:
                            controller_road_ids.add(signal_road_id)

                    # If any of the controller's roads are related to this junction,
                    # associate the controller with the junction
                    if controller_road_ids & junction_related_road_ids:
                        junction_controller_ids.append(controller.id)

            junction.controller_ids = junction_controller_ids
            controllers_assigned_count += len(junction_controller_ids)

        print(
            f"Associated {controllers_assigned_count} controller references across {len(junctions)} junctions"
        )

    def _write_opendrive_output(
        self,
        all_roads: List[Road],
        junctions: List[Junction],
        signals_and_controllers: SignalsAndControllers,
    ) -> OpenDRIVE:
        """
        Write final OpenDRIVE XML output.

        Args:
            all_roads: All roads to write
            junctions: All junctions to write
            signals_and_controllers: All signals and controllers

        Returns:
            OpenDRIVE object
        """
        from autoware_lanelet2_to_opendrive.util import latlon_to_proj_string

        # Generate PROJ string for geoReference
        if self.config.origin.lat is not None and self.config.origin.lon is not None:
            geo_reference_proj = latlon_to_proj_string(
                self.config.origin.lat, self.config.origin.lon
            )
            print(f"Using geoReference (from origin lat/lon): {geo_reference_proj}")
        elif self.mgrs_code is not None:
            geo_reference_proj = mgrs_to_proj_string(self.mgrs_code)
            print(f"Using geoReference (from MGRS code): {geo_reference_proj}")
        elif self.config.origin.mgrs_code is not None:
            geo_reference_proj = mgrs_to_proj_string(self.config.origin.mgrs_code)
            print(f"Using geoReference (from config MGRS code): {geo_reference_proj}")
        else:
            raise ValueError("Must provide either origin lat/lon or MGRS code")

        # Create header
        header = Header(
            rev_major="1",
            rev_minor="4",
            name="Converted from Lanelet2",
            version="1.0",
            date=datetime.now().strftime("%Y-%m-%dT%H:%M:%S"),
            north="0.0",
            south="0.0",
            east="0.0",
            west="0.0",
            geo_reference=geo_reference_proj,
        )

        # Create OpenDRIVE object
        opendrive = OpenDRIVE(
            header=header,
            roads=all_roads,
            junctions=junctions,
            controllers=signals_and_controllers.controllers,
        )

        print("\nConversion completed successfully!")

        # Save to file if output path is provided
        if self.config.output_path:
            save_opendrive_to_file(opendrive, self.config.output_path)
            print(f"OpenDRIVE file saved to: {self.config.output_path}")

        return opendrive

    def convert(self) -> Tuple[OpenDRIVE, RoadLaneletMapping]:
        """
        Convert Lanelet2 map to OpenDRIVE format.

        High-level orchestration of conversion pipeline.

        Returns:
            Tuple of:
                - OpenDRIVE object representing the converted map
                - RoadLaneletMapping containing bidirectional mapping
        """
        print("Converting Lanelet2 map to OpenDRIVE format...")

        # Step 1: Build regular roads from non-junction lanelets
        regular_roads, lanelet_to_road_id = self._build_regular_roads()

        # Step 2: Build junction structure
        (
            connecting_roads,
            junctions,
            junction_to_roads,
            junction_lanelet_to_road,
            junction_lanelets,
        ) = self._build_junction_structure(regular_roads, lanelet_to_road_id)

        # Step 3: Create bidirectional mappings
        mapping = self._build_road_lanelet_mappings(lanelet_to_road_id)

        # Combine all roads
        all_roads = regular_roads + connecting_roads
        print(
            f"\nTotal roads: {len(all_roads)} ({len(regular_roads)} regular + {len(connecting_roads)} connecting)"
        )

        # Step 4: Set up road and lane connections
        self._setup_connections(
            all_roads,
            connecting_roads,
            mapping.road_to_lanelets,
            lanelet_to_road_id,
            junctions,
        )

        # Step 5: Extract and assign signals
        signals_and_controllers = self._extract_and_assign_signals(
            all_roads, mapping, junction_lanelets
        )

        # Step 6: Create and assign controllers
        self._assign_controllers_to_junctions(
            signals_and_controllers, junctions, all_roads
        )

        # Step 7: Write OpenDRIVE output
        opendrive = self._write_opendrive_output(
            all_roads, junctions, signals_and_controllers
        )

        return opendrive, mapping


def convert_lanelet2_to_opendrive(
    lanelet_map: lanelet2.core.LaneletMap,
    config: ConversionConfig,
    mgrs_code: Optional[str] = None,
) -> Tuple[OpenDRIVE, RoadLaneletMapping]:
    """
    Convert Lanelet2 map to OpenDRIVE format.

    Args:
        lanelet_map: Loaded Lanelet2 map
        config: ConversionConfig object containing all conversion parameters
        mgrs_code: Optional MGRS code for generating geoReference PROJ string.
            If not provided, will be derived from config.origin.

    Returns:
        Tuple of:
            - OpenDRIVE object representing the converted map
            - RoadLaneletMapping containing bidirectional mapping between roads and lanelets
    """
    converter = _Lanelet2ToOpenDRIVEConverter(lanelet_map, config, mgrs_code)
    return converter.convert()


def parse_origin_from_config(
    cfg: DictConfig,
) -> Tuple[lanelet2.io.Origin, str, float, float, float, float, float]:
    """
    Parse origin specification from Hydra config with mutual exclusion validation.

    Supports three methods of origin specification (mutually exclusive):
    1. mgrs_grid: Simple MGRS grid code (e.g., "54SUE")
       - With optional offset: mgrs_grid + offset {x, y, z}
    2. lat_lon: Latitude/longitude {latitude, longitude, altitude}

    Args:
        cfg: Hydra configuration object with map settings

    Returns:
        Tuple of (lanelet2.io.Origin, mgrs_code_for_proj_string, origin_lat, origin_lon,
                  offset_x, offset_y, offset_z)
        The mgrs_code_for_proj_string is used for generating the OpenDRIVE geoReference
        The origin_lat and origin_lon are the actual origin coordinates (with offset applied)
        The offset values are used to convert coordinates to local coordinate system

    Raises:
        ValueError: If origin specification is invalid or multiple methods are specified
    """
    map_cfg = cfg.map

    # Check which origin specification methods are provided
    has_mgrs_grid = "mgrs_grid" in map_cfg and map_cfg.mgrs_grid is not None
    has_mgrs_code = (
        "mgrs_code" in map_cfg and map_cfg.mgrs_code is not None
    )  # Legacy support
    has_offset = "offset" in map_cfg and map_cfg.offset is not None
    has_lat_lon = "lat_lon" in map_cfg and map_cfg.lat_lon is not None

    # Support legacy mgrs_code field
    if has_mgrs_code and not has_mgrs_grid:
        has_mgrs_grid = True
        map_cfg.mgrs_grid = map_cfg.mgrs_code

    # Count how many base methods are specified (offset is an optional modifier for mgrs_grid)
    specified_methods = sum([has_mgrs_grid, has_lat_lon])

    if specified_methods == 0:
        raise ValueError(
            "Origin must be specified using one of: mgrs_grid (with optional offset), or lat_lon"
        )

    if specified_methods > 1:
        raise ValueError(
            "Multiple origin specification methods detected. "
            "Please specify only one of: mgrs_grid (with optional offset), or lat_lon"
        )

    # Offset can only be used with mgrs_grid
    if has_offset and not has_mgrs_grid:
        raise ValueError(
            "The 'offset' field can only be used together with 'mgrs_grid'"
        )

    # Parse the specified method
    if has_mgrs_grid:
        mgrs_grid = map_cfg.mgrs_grid

        # Check if offset is specified
        if has_offset:
            offset_cfg = map_cfg.offset
            offset_x = offset_cfg.x
            offset_y = offset_cfg.y
            offset_z = offset_cfg.get("z", 0.0)

            logger.info(
                f"Using MGRS grid with offset: {mgrs_grid}, "
                f"offset x={offset_x} y={offset_y} z={offset_z}"
            )
            # Get lat/lon with offset applied
            from autoware_lanelet2_to_opendrive.util import (
                mgrs_grid_with_offset_to_latlon,
            )

            origin_lat, origin_lon = mgrs_grid_with_offset_to_latlon(
                mgrs_grid, offset_x, offset_y
            )
            origin = mgrs_grid_with_offset_to_lanelet2_origin(
                mgrs_grid, offset_x, offset_y, offset_z
            )
            logger.info(f"Origin coordinates: lat={origin_lat}, lon={origin_lon}")
            return (
                origin,
                mgrs_grid,
                origin_lat,
                origin_lon,
                offset_x,
                offset_y,
                offset_z,
            )
        else:
            logger.info(f"Using MGRS grid origin: {mgrs_grid}")
            origin = mgrs_to_lanelet2_origin(mgrs_grid)
            # Get lat/lon from MGRS grid origin
            import mgrs as mgrs_lib

            m = mgrs_lib.MGRS()
            # Pad with zeros to get grid origin
            processed_mgrs = mgrs_grid + "0000000000"
            origin_lat, origin_lon = m.toLatLon(processed_mgrs)
            logger.info(f"Origin coordinates: lat={origin_lat}, lon={origin_lon}")
            # No offset specified
            return origin, mgrs_grid, origin_lat, origin_lon, 0.0, 0.0, 0.0

    elif has_lat_lon:
        latlon_cfg = map_cfg.lat_lon
        latitude = latlon_cfg.latitude
        longitude = latlon_cfg.longitude
        altitude = latlon_cfg.get("altitude", 0.0)

        logger.info(
            f"Using lat/lon origin: latitude={latitude}, longitude={longitude}, altitude={altitude}"
        )
        origin = latlon_to_lanelet2_origin(latitude, longitude, altitude)

        # For lat/lon origin, we need to generate an approximate MGRS code for the PROJ string
        # Convert lat/lon back to MGRS to get the grid zone
        import mgrs as mgrs_lib

        m = mgrs_lib.MGRS()
        mgrs_code = m.toMGRS(latitude, longitude)
        # Extract just the grid zone designator (first 5 characters: zone + band + square)
        # Format: 54SUE1234567890 -> we want 54SUE
        import re

        match = re.match(r"^(\d+[A-Z][A-Z][A-Z])", mgrs_code)
        if match:
            mgrs_grid = match.group(1)
        else:
            mgrs_grid = mgrs_code[:5]  # Fallback to first 5 chars

        logger.info(f"Derived MGRS grid for PROJ string: {mgrs_grid}")
        # No offset for lat/lon origin
        return origin, mgrs_grid, latitude, longitude, 0.0, 0.0, 0.0

    # Should never reach here due to the checks above
    raise ValueError("Invalid origin configuration")


def preprocess_and_convert_with_hydra(
    cfg: DictConfig,
    lanelet2_file: Path,
    output_file: Path,
) -> None:
    """
    Run preprocessing (if configured) and convert Lanelet2 to OpenDRIVE using Hydra config.

    Args:
        cfg: Hydra configuration object
        lanelet2_file: Path to input Lanelet2 OSM file
        output_file: Path to output OpenDRIVE file
    """
    input_map_path = lanelet2_file

    # Parse origin from config (with mutual exclusion validation)
    origin, mgrs_code, origin_lat, origin_lon, offset_x, offset_y, offset_z = (
        parse_origin_from_config(cfg)
    )

    # Set global coordinate offset for conversion
    # This will be applied to all coordinates during OpenDRIVE export
    COORDINATE_OFFSET.set(offset_x, offset_y, offset_z)
    if COORDINATE_OFFSET.is_active:
        logger.info(
            f"Coordinate offset enabled: x={offset_x}, y={offset_y}, z={offset_z}"
        )

    # Get target-specific settings
    exclude_non_junction_signals = cfg.target.get("exclude_non_junction_signals", False)
    # Priority: map config > target config > default (RHT)
    traffic_rule = cfg.map.get("traffic_rule") or cfg.target.get("traffic_rule", "RHT")

    # Build PreprocessOperation from Hydra map config
    config = PreprocessOperation.from_hydra_config(cfg.map)

    # Check if any preprocessing operations are configured
    has_preprocessing = any(
        [
            config.merge_operations,
            config.remove_operations,
            config.replace_operations,
            config.move_point_operations,
            config.delete_point_operations,
            config.remove_lanelet_operations,
            config.remove_turn_direction_operations,
        ]
    )

    if has_preprocessing:
        logger.info("Running preprocessing operations...")

        # Set input/output paths for preprocessing
        config.input_map_path = str(lanelet2_file)

        # Create a temporary file for preprocessed output
        if config.output_map_path:
            preprocessed_path = Path(config.output_map_path)
        else:
            with tempfile.NamedTemporaryFile(suffix=".osm", delete=False) as tmp_file:
                preprocessed_path = Path(tmp_file.name)
                config.output_map_path = str(preprocessed_path)

        # Run preprocessing
        preprocessor = LaneletPreprocessor(config)
        preprocessor.process()

        # Update input path to use preprocessed map
        input_map_path = preprocessed_path
        logger.info(
            f"Preprocessing completed. Using preprocessed map from: {input_map_path}"
        )

    # Load the (possibly preprocessed) Lanelet2 map
    logger.info(f"Loading Lanelet2 map from: {input_map_path}")
    logger.info(f"Using origin with MGRS code for PROJ: {mgrs_code}")

    lanelet_map = load_lanelet2_map(input_map_path, origin)

    # Convert to OpenDRIVE
    logger.info("Converting to OpenDRIVE format...")

    # Build ParamPoly3Config from Hydra config
    # Priority: map config > target config > default
    parampoly3_dict = cfg.map.get("parampoly3") or cfg.target.get("parampoly3", {})
    if parampoly3_dict:
        parampoly3_config = ParamPoly3Config(
            min_segment_length=parampoly3_dict.get("min_segment_length", 0.5),
            default_segment_length=parampoly3_dict.get("default_segment_length", 1.0),
            max_segments=parampoly3_dict.get("max_segments", 100),
            min_segments=parampoly3_dict.get("min_segments", 1),
            coefficient_epsilon=parampoly3_dict.get("coefficient_epsilon", 1e-8),
            enabled=parampoly3_dict.get("enabled", True),
        )
        logger.info(
            f"ParamPoly3 config: default_length={parampoly3_config.default_segment_length}m, "
            f"max_segments={parampoly3_config.max_segments}"
        )
    else:
        parampoly3_config = ParamPoly3Config()

    # Build WidthEstimationConfig from Hydra config
    # Priority: map config > target config > default
    width_dict = cfg.map.get("width_estimation") or cfg.target.get(
        "width_estimation", {}
    )
    if width_dict:
        width_config = WidthEstimationConfig(
            adaptive_sampling=width_dict.get("adaptive_sampling", False),
            min_samples=width_dict.get("min_samples", 5),
            max_samples=width_dict.get("max_samples", 50),
            default_sample_interval=width_dict.get("default_sample_interval", 5.0),
        )
        logger.info(
            f"Width sampling config: adaptive={width_config.adaptive_sampling}, "
            f"interval={width_config.default_sample_interval}m, "
            f"max_samples={width_config.max_samples}"
        )
    else:
        width_config = WidthEstimationConfig()

    # Build ConversionConfig from parameters
    conversion_config = ConversionConfig(
        output_path=output_file,
        origin=OriginSpec(
            mgrs_code=mgrs_code,
            lat=origin_lat,
            lon=origin_lon,
        ),
        exclude_non_junction_signals=exclude_non_junction_signals,
        traffic_rule=traffic_rule,
        parampoly3=parampoly3_config,
        width_estimation=width_config,
    )

    opendrive, mapping = convert_lanelet2_to_opendrive(
        lanelet_map,
        conversion_config,
        mgrs_code=mgrs_code,
    )

    logger.info("Conversion completed successfully!")
    logger.info(
        f"Road-Lanelet mapping: {len(mapping.road_to_lanelets)} roads, "
        f"{len(mapping.lanelet_to_road)} lanelets"
    )


@hydra.main(version_base=None, config_path="conf", config_name="config")
def main(cfg: DictConfig) -> None:
    """
    Main entry point for the conversion script using Hydra.

    Usage:
        # Basic usage with default settings:
        uv run python -m autoware_lanelet2_to_opendrive.main input_map_path=/path/to/map.osm

        # With CARLA target:
        uv run python -m autoware_lanelet2_to_opendrive.main \\
            input_map_path=/path/to/map.osm target=carla

        # With custom map config:
        uv run python -m autoware_lanelet2_to_opendrive.main \\
            input_map_path=/path/to/map.osm map=my_map target=carla

        # Override output path:
        uv run python -m autoware_lanelet2_to_opendrive.main \\
            input_map_path=/path/to/map.osm output_map_path=/path/to/output.xodr
    """
    # Print resolved configuration
    if cfg.verbose:
        logging.getLogger().setLevel(logging.DEBUG)
        logger.debug("Resolved configuration:")
        logger.debug(OmegaConf.to_yaml(cfg))

    # Get input/output paths
    input_path = Path(cfg.input_map_path)

    # Determine output path
    if cfg.output_map_path:
        output_path = Path(cfg.output_map_path)
    else:
        output_path = input_path.with_suffix(".xodr")

    try:
        preprocess_and_convert_with_hydra(
            cfg=cfg,
            lanelet2_file=input_path,
            output_file=output_path,
        )

    except FileNotFoundError as e:
        logger.error(f"File not found: {e}")
        sys.exit(1)
    except ValueError as e:
        logger.error(f"Configuration error: {e}")
        sys.exit(1)
    except Exception as e:
        logger.error(f"Error during conversion: {e}")
        if cfg.verbose:
            import traceback

            traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
