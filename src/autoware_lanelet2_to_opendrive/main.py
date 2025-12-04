#!/usr/bin/env python3
"""Main script to convert Lanelet2 maps to OpenDRIVE format."""

import argparse
import sys
from pathlib import Path
from typing import Optional

# Import autoware extensions before loading maps to ensure proper registration
# The order matters: projection module must be imported to register extensions
from autoware_lanelet2_extension_python.projection import MGRSProjector
import lanelet2

from autoware_lanelet2_to_opendrive.util import mgrs_to_lanelet2_origin

from autoware_lanelet2_to_opendrive.opendrive.opendrive_dataclass import (
    OpenDRIVE,
    Header,
    save_opendrive_to_file,
)


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
    opendrive = OpenDRIVE(header=header, roads=[])

    print("Conversion completed (stub implementation)")

    # Save to file if output path is provided
    if output_path:
        save_opendrive_to_file(opendrive, output_path)
        print(f"OpenDRIVE file saved to: {output_path}")

    return opendrive


def main():
    """Main entry point for the conversion script."""
    parser = argparse.ArgumentParser(
        description="Convert Lanelet2 map to OpenDRIVE format",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python main.py input.osm --mgrs-lat 35.23 --mgrs-lon 139.16
  python main.py input.osm --mgrs-lat 35.23 --mgrs-lon 139.16 -o output.xodr
        """,
    )

    parser.add_argument(
        "lanelet2_file", type=Path, help="Path to the input Lanelet2 OSM file"
    )

    parser.add_argument("mgrs", type=str, help="MGRS grid reference for projection")

    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        help="Output path for the OpenDRIVE file (default: input_file.xodr)",
    )

    parser.add_argument(
        "--verbose", "-v", action="store_true", help="Enable verbose output"
    )

    args = parser.parse_args()

    # Set default output path if not provided
    if not args.output:
        args.output = args.lanelet2_file.with_suffix(".xodr")

    try:
        # Load Lanelet2 map
        if args.verbose:
            print(f"Loading Lanelet2 map from: {args.lanelet2_file}")
            print(f"Using MGRS coordinates: mgrs grid={args.mgrs}")

        lanelet_map = load_lanelet2_map(args.lanelet2_file, args.mgrs)

        # Convert to OpenDRIVE
        if args.verbose:
            print(f"Output will be saved to: {args.output}")

        convert_lanelet2_to_opendrive(lanelet_map, Path(args.output))

        print("Conversion completed successfully!")

    except FileNotFoundError as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"Error during conversion: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
