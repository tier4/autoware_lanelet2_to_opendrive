#!/usr/bin/env python3
"""Main script to convert Lanelet2 maps to OpenDRIVE format."""

import argparse
import sys
from pathlib import Path
from typing import Optional
import tempfile
import logging

# Import autoware extensions before loading maps to ensure proper registration
# The order matters: projection module must be imported to register extensions
from autoware_lanelet2_extension_python.projection import MGRSProjector
import lanelet2

from autoware_lanelet2_to_opendrive.util import mgrs_to_lanelet2_origin
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

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def load_lanelet2_map(lanelet2_path: Path, mgrs: str) -> lanelet2.core.LaneletMap:
    """
    Load Lanelet2 map from file with MGRS projection.

    Args:
        lanelet2_path: Path to the Lanelet2 OSM file
        mgrs_lat: MGRS latitude coordinate
        mgrs_lon: MGRS longitude coordinate

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
    lanelet_map: lanelet2.core.LaneletMap, output_path: Optional[Path] = None
) -> OpenDRIVE:
    """
    Convert Lanelet2 map to OpenDRIVE format.

    Args:
        lanelet_map: Loaded Lanelet2 map
        output_path: Optional output path for saving the OpenDRIVE file

    Returns:
        OpenDRIVE object representing the converted map

    Note:
        This is currently a stub implementation.
    """
    print("Converting Lanelet2 map to OpenDRIVE format...")

    # TODO: Implement actual conversion logic
    # This is a stub implementation that creates a minimal OpenDRIVE structure

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
    )

    # Create minimal road structure
    # TODO: Extract roads from lanelet groups

    # For now, create an empty OpenDRIVE structure
    opendrive = OpenDRIVE(
        header=header, roads=Road.construct_from_lanelet_map(lanelet_map)
    )

    print("Conversion completed (stub implementation)")

    # Save to file if output path is provided
    if output_path:
        save_opendrive_to_file(opendrive, output_path)
        print(f"OpenDRIVE file saved to: {output_path}")

    return opendrive


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
    convert_lanelet2_to_opendrive(lanelet_map, output_file)

    logger.info("Conversion completed successfully!")

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
