#!/usr/bin/env python3
"""Main script to convert Lanelet2 maps to OpenDRIVE format."""

import argparse
import sys
from pathlib import Path
from typing import Optional, Dict, List, Tuple
import tempfile
import logging

# Import autoware extensions before loading maps to ensure proper registration
# The order matters: projection module must be imported to register extensions
from autoware_lanelet2_extension_python.projection import MGRSProjector
import lanelet2

from autoware_lanelet2_to_opendrive.util import (
    mgrs_to_lanelet2_origin,
    mgrs_to_proj_string,
    RoadLaneletMapping,
)
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

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def load_lanelet2_map(lanelet2_path: Path, mgrs: str) -> lanelet2.core.LaneletMap:
    """
    Load Lanelet2 map from file with MGRS projection.

    Args:
        lanelet2_path: Path to the Lanelet2 OSM file
        mgrs: MGRS grid reference code (e.g., "54SUE815501")

    Returns:
        Loaded Lanelet2 map

    Raises:
        FileNotFoundError: If the Lanelet2 file doesn't exist
        Exception: If map loading fails
    """
    if not lanelet2_path.exists():
        raise FileNotFoundError(f"Lanelet2 file not found: {lanelet2_path}")

    try:
        projector = MGRSProjector(mgrs_to_lanelet2_origin(mgrs))
        lanelet_map = lanelet2.io.load(str(lanelet2_path), projector)
        print(f"Successfully loaded Lanelet2 map: {lanelet2_path}")
        print(f"  - Lanelets: {len(lanelet_map.laneletLayer)}")
        print(f"  - Linestrings: {len(lanelet_map.lineStringLayer)}")
        print(f"  - Points: {len(lanelet_map.pointLayer)}")
        return lanelet_map
    except Exception as e:
        raise Exception(f"Failed to load Lanelet2 map: {e}")


def convert_lanelet2_to_opendrive(
    lanelet_map: lanelet2.core.LaneletMap,
    mgrs_code: str,
    output_path: Optional[Path] = None,
) -> Tuple[OpenDRIVE, RoadLaneletMapping]:
    """
    Convert Lanelet2 map to OpenDRIVE format.

    Args:
        lanelet_map: Loaded Lanelet2 map
        mgrs_code: MGRS code for generating geoReference PROJ string
        output_path: Optional output path for saving the OpenDRIVE file

    Returns:
        Tuple of:
            - OpenDRIVE object representing the converted map
            - RoadLaneletMapping containing bidirectional mapping between roads and lanelets
    """
    print("Converting Lanelet2 map to OpenDRIVE format...")

    # Generate PROJ string from MGRS code for geoReference
    geo_reference_proj = mgrs_to_proj_string(mgrs_code)
    print(f"Using geoReference: {geo_reference_proj}")

    # Create header
    header = Header(
        rev_major="1",
        rev_minor="4",
        name="Converted from Lanelet2",
        version="1.0",
        date="2024-01-01T00:00:00",
        north="0.0",
        south="0.0",
        east="0.0",
        west="0.0",
        geo_reference=geo_reference_proj,
    )

    # Step 1: Create regular roads (outside junctions)
    print("\n=== Building regular roads ===")
    regular_roads = Road.construct_from_lanelet_map(lanelet_map)
    starting_junction_road_id = len(regular_roads)

    # Build lanelet-to-road mapping for regular roads
    from autoware_lanelet2_to_opendrive.junction import (
        filter_lanelets_inside_junction,
        filter_lanelets_outside_junction,
        find_junction_groups,
    )
    from autoware_lanelet2_to_opendrive.util import (
        find_adjacent_groups,
        filter_lanelets_by_subtype,
    )

    all_lanelets = list(lanelet_map.laneletLayer)
    road_lanelets = filter_lanelets_outside_junction(
        filter_lanelets_by_subtype(all_lanelets, ["road"])
    )
    adjacent_groups = find_adjacent_groups(lanelet_map, set(road_lanelets))

    lanelet_to_road_id: dict[int, int] = {}
    for road_id, adjacent_group in enumerate(adjacent_groups):
        for lanelet in adjacent_group:
            lanelet_to_road_id[lanelet.id] = road_id

    # Step 2: Get junction groups
    print("\n=== Finding junctions ===")
    junction_lanelets = filter_lanelets_inside_junction(all_lanelets)
    junction_groups = find_junction_groups(junction_lanelets)
    print(f"Found {len(junction_groups)} junctions")

    # Step 3: Create connecting roads (inside junctions)
    print("\n=== Building connecting roads inside junctions ===")
    (
        connecting_roads,
        junction_to_roads,
        junction_lanelet_to_road,
    ) = Road.construct_connecting_roads_from_junctions(
        lanelet_map=lanelet_map,
        junction_groups=junction_groups,
        starting_road_id=starting_junction_road_id,
    )

    # Step 4: Merge lanelet-to-road mappings
    lanelet_to_road_id.update(junction_lanelet_to_road)

    # Step 4.5: Build reverse mapping (road_id -> list of lanelet IDs)
    print("\n=== Building Road-Lanelet mapping ===")
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

    # Step 5: Build junctions with connections
    print("\n=== Building junction connections ===")
    junctions = []
    for junction_id, junction_group in enumerate(junction_groups):
        # Create base junction
        junction = Junction.construct_from_lanelet_groups(
            junction_id=junction_id,
            lanelet_group=junction_group,
        )

        # Build connections for this junction
        connecting_road_ids = junction_to_roads.get(junction_id, [])
        connections = Junction.build_connections_from_roads(
            lanelet_map=lanelet_map,
            junction_lanelet_group=junction_group,
            junction_id=junction_id,
            lanelet_to_road_id=lanelet_to_road_id,
            connecting_road_ids=connecting_road_ids,
        )

        junction.connections = connections
        junctions.append(junction)

    print(
        f"Built {len(junctions)} junctions with {sum(len(j.connections) for j in junctions)} total connections"
    )

    # Step 6: Combine all roads
    all_roads = regular_roads + connecting_roads
    print(
        f"\nTotal roads: {len(all_roads)} ({len(regular_roads)} regular + {len(connecting_roads)} connecting)"
    )

    # Step 6.5: Set lane links for all roads (including junction roads)
    # This must be done after combining all roads so that connections between
    # regular roads and junction roads are properly established
    print("\n=== Building lane links for all roads ===")
    Road.set_all_lane_links(lanelet_map, all_roads)

    # Step 6.6: Set road links for connecting roads (predecessor/successor)
    print("\n=== Building road links for connecting roads ===")
    Road.set_connecting_road_links(
        lanelet_map=lanelet_map,
        connecting_roads=connecting_roads,
        lanelet_to_road_id=lanelet_to_road_id,
        road_to_lanelet_ids=road_to_lanelet_ids,
    )

    # Step 6.7: Set junction links for incoming roads
    # This ensures CARLA compatibility by setting successor/predecessor to junction
    # for roads that connect to junctions as incoming roads
    print("\n=== Setting junction links for incoming roads ===")
    Road.set_incoming_road_junction_links(
        roads=all_roads,
        junctions=junctions,
    )

    # Create mapping object
    mapping = RoadLaneletMapping(
        road_to_lanelets=road_to_lanelet_ids, lanelet_to_road=lanelet_to_road_id
    )

    # Step 7: Extract signals and controllers from Lanelet2 map
    print("\n=== Extracting signals and controllers ===")
    signals_and_controllers = SignalsAndControllers.construct_from_lanelet_map(
        lanelet_map=lanelet_map,
        road_lanelet_mapping=mapping,
        roads=all_roads,
    )
    print(
        f"Extracted {len(signals_and_controllers.signals)} signals and "
        f"{len(signals_and_controllers.controllers)} controllers"
    )

    # Step 8: Assign signals to roads
    print("\n=== Assigning signals to roads ===")
    road_signals: Dict[int, List] = {}
    for signal in signals_and_controllers.signals:
        signal_road_id: Optional[int] = signals_and_controllers.signal_to_road_id.get(
            signal.id
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

    # Step 9: Associate controllers with junctions
    # Controllers that manage signals on incoming or connecting roads should be referenced
    print("\n=== Associating controllers with junctions ===")
    controllers_assigned_count = 0
    for junction in junctions:
        # Get all road IDs related to this junction (both incoming and connecting)
        junction_incoming_road_ids = {
            conn.incoming_road for conn in junction.connections
        }
        junction_connecting_road_ids = {
            conn.connecting_road for conn in junction.connections
        }
        junction_related_road_ids = (
            junction_incoming_road_ids | junction_connecting_road_ids
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

    # Create OpenDRIVE object
    opendrive = OpenDRIVE(
        header=header,
        roads=all_roads,
        junctions=junctions,
        controllers=signals_and_controllers.controllers,
    )

    print("\nConversion completed successfully!")

    # Save to file if output path is provided
    if output_path:
        save_opendrive_to_file(opendrive, output_path)
        print(f"OpenDRIVE file saved to: {output_path}")

    return opendrive, mapping


def preprocess_and_convert(
    lanelet2_file: Path,
    output_file: Path,
    preprocess_config_path: Optional[Path] = None,
    mgrs_code: Optional[str] = None,
    verbose: bool = False,
) -> None:
    """
    Run preprocessing (if configured) and convert Lanelet2 to OpenDRIVE.

    Args:
        lanelet2_file: Path to input Lanelet2 OSM file
        output_file: Path to output OpenDRIVE file
        preprocess_config_path: Optional path to preprocessing config YAML
        mgrs_code: Optional MGRS code (required if not in preprocess config)
        verbose: Enable verbose output
    """
    # Determine the input file and MGRS code
    input_map_path = lanelet2_file

    # If preprocessing config is provided, run preprocessing first
    if preprocess_config_path:
        logger.info(
            f"Loading preprocessing configuration from: {preprocess_config_path}"
        )

        try:
            config = PreprocessOperation.from_yaml(preprocess_config_path)

            # Override verbose setting if specified
            if verbose:
                config.verbose = True

            # Extract MGRS code from config
            mgrs_code = config.mgrs_code

            # Create a temporary file for preprocessed output if needed
            # We'll use the configured output path or a temp file
            if config.output_map_path:
                preprocessed_path = Path(config.output_map_path)
            else:
                # Create a temporary file for preprocessed map
                with tempfile.NamedTemporaryFile(
                    suffix=".osm", delete=False
                ) as tmp_file:
                    preprocessed_path = Path(tmp_file.name)
                    config.output_map_path = str(preprocessed_path)

            # Run preprocessing
            logger.info("Running preprocessing operations...")
            preprocessor = LaneletPreprocessor(config)
            preprocessor.process()

            # Update input path to use preprocessed map
            input_map_path = preprocessed_path
            logger.info(
                f"Preprocessing completed. Using preprocessed map from: {input_map_path}"
            )

        except Exception as e:
            logger.error(f"Preprocessing failed: {e}")
            raise

    # Ensure we have an MGRS code
    if not mgrs_code:
        raise ValueError(
            "MGRS code must be provided either via --preprocess-config or directly. "
            "Add 'mgrs_code' to your preprocess config YAML file."
        )

    # Load the (possibly preprocessed) Lanelet2 map
    logger.info(f"Loading Lanelet2 map from: {input_map_path}")
    logger.info(f"Using MGRS code: {mgrs_code}")

    lanelet_map = load_lanelet2_map(input_map_path, mgrs_code)

    # Convert to OpenDRIVE
    logger.info("Converting to OpenDRIVE format...")
    opendrive, mapping = convert_lanelet2_to_opendrive(
        lanelet_map, mgrs_code, output_file
    )

    logger.info("Conversion completed successfully!")
    logger.info(
        f"Road-Lanelet mapping: {len(mapping.road_to_lanelets)} roads, "
        f"{len(mapping.lanelet_to_road)} lanelets"
    )

    # Clean up temporary file if we created one
    if (
        preprocess_config_path
        and not PreprocessOperation.from_yaml(preprocess_config_path).output_map_path
    ):
        try:
            preprocessed_path.unlink()
            logger.debug(f"Cleaned up temporary file: {preprocessed_path}")
        except Exception:
            pass


def main():
    """Main entry point for the conversion script."""
    parser = argparse.ArgumentParser(
        description="Convert Lanelet2 map to OpenDRIVE format with optional preprocessing",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Direct conversion without preprocessing (MGRS must be in preprocess config):
  python main.py input.osm --preprocess-config config.yaml

  # Direct conversion with preprocessing:
  python main.py input.osm --preprocess-config preprocess_config.yaml -o output.xodr

  # Specify output file:
  python main.py input.osm --preprocess-config config.yaml -o custom_output.xodr

Note: MGRS code must be specified in the preprocess configuration YAML file.
        """,
    )

    parser.add_argument(
        "lanelet2_file",
        type=Path,
        help="Path to the input Lanelet2 OSM file",
    )

    parser.add_argument(
        "--preprocess-config",
        type=Path,
        help="Path to preprocessing configuration YAML file (contains MGRS code)",
    )

    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        help="Output path for the OpenDRIVE file (default: input_file.xodr)",
    )

    parser.add_argument(
        "--verbose",
        "-v",
        action="store_true",
        help="Enable verbose output",
    )

    args = parser.parse_args()

    # Set up logging
    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    # Set default output path if not provided
    if not args.output:
        args.output = args.lanelet2_file.with_suffix(".xodr")

    # Validate that preprocessing config is provided
    if not args.preprocess_config:
        parser.error(
            "Preprocessing configuration file is required. "
            "Please provide --preprocess-config with a YAML file containing MGRS code and optional preprocessing operations."
        )

    try:
        preprocess_and_convert(
            lanelet2_file=args.lanelet2_file,
            output_file=args.output,
            preprocess_config_path=args.preprocess_config,
            verbose=args.verbose,
        )

    except FileNotFoundError as e:
        logger.error(f"File not found: {e}")
        sys.exit(1)
    except ValueError as e:
        logger.error(f"Configuration error: {e}")
        sys.exit(1)
    except Exception as e:
        logger.error(f"Error during conversion: {e}")
        if args.verbose:
            import traceback

            traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
