#!/usr/bin/env python3
"""Visualize boundary errors between Lanelet2 and OpenDRIVE conversion.

This script compares original Lanelet2 boundaries with converted OpenDRIVE
lane boundaries to assess conversion accuracy. It uses 3D nearest-neighbor
matching to automatically identify corresponding lanes, then calculates and
visualizes fitting errors with color-coded plots and statistical histograms.

Usage:
    uv run visualize <opendrive_file> <lanelet2_file> [options]

Examples:
    # Basic usage with interactive display
    uv run visualize lanelet2_map.xodr lanelet2_map.osm

    # Specify lanelet IDs and save outputs
    uv run visualize map.xodr map.osm --lanelet-id 120 121 122 \\
        --output-json errors.json --output-png visualization.png

    # Adjust sampling interval and colormap
    uv run visualize map.xodr map.osm --sample-interval 1.0 \\
        --colormap viridis --no-show
"""

import argparse
import json
import logging
import pickle
import sys
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import lanelet2
import matplotlib.pyplot as plt
import numpy as np
from pyxodr.road_objects.network import RoadNetwork

from .config import DEFAULT_CONFIG
from .util import extract_points_3d

# Configure logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


class BoundaryVisualizationError(Exception):
    """Base exception for boundary visualization errors."""

    pass


class OpenDRIVEParsingError(BoundaryVisualizationError):
    """Failed to parse OpenDRIVE file."""

    pass


class Lanelet2LoadingError(BoundaryVisualizationError):
    """Failed to load Lanelet2 map."""

    pass


class NoMatchingLaneError(BoundaryVisualizationError):
    """No OpenDRIVE lane found within search radius."""

    pass


def extract_opendrive_lane_boundaries(
    opendrive_path: Path,
) -> List[np.ndarray]:
    """Extract lane boundary points from OpenDRIVE file using pyxodr.

    Args:
        opendrive_path: Path to OpenDRIVE .xodr file

    Returns:
        List of numpy arrays, each (N, 3) with [x, y, z] boundary coordinates

    Raises:
        OpenDRIVEParsingError: If parsing fails
    """
    try:
        logger.info(f"Loading OpenDRIVE file: {opendrive_path}")
        network = RoadNetwork.from_opendrive(str(opendrive_path))

        boundaries = []

        for road in network.roads:
            logger.debug(f"Processing road ID {road.id}")

            # Sample reference line at regular intervals
            s_step = 0.5  # Sample every 0.5m
            s_vals = np.arange(0, road.length, s_step)

            # Add the end point
            if s_vals[-1] < road.length:
                s_vals = np.append(s_vals, road.length)

            # Get reference line points
            ref_line_points = []
            for s in s_vals:
                try:
                    x, y, hdg = road.get_geometry(s)
                    # For now, assume z=0 (2D)
                    ref_line_points.append([x, y, 0.0])
                except Exception as e:
                    logger.warning(
                        f"Failed to get geometry at s={s} for road {road.id}: {e}"
                    )
                    continue

            ref_line_points = np.array(ref_line_points)

            # Extract lane boundaries for each lane section
            for lane_section in road.lane_sections:
                # Process left lanes
                for lane in lane_section.left_lanes:
                    try:
                        boundary_points = []

                        # Get s values within this lane section
                        s_start = lane_section.s_offset
                        s_end = (
                            lane_section.s_offset + lane_section.length
                            if hasattr(lane_section, "length")
                            else road.length
                        )

                        for s in s_vals:
                            if s < s_start or s > s_end:
                                continue

                            try:
                                # Get reference line position and heading
                                x_ref, y_ref, hdg = road.get_geometry(s)

                                # Get lane width at this s position
                                # Calculate offset for outer boundary
                                offset = 0.0
                                for i in range(abs(lane.id)):
                                    other_lane = next(
                                        (
                                            ln
                                            for ln in lane_section.left_lanes
                                            if ln.id == i + 1
                                        ),
                                        None,
                                    )
                                    if other_lane:
                                        # Get width at relative s position
                                        s_relative = s - s_start
                                        width = other_lane.get_width(s_relative)
                                        offset += width

                                # Calculate boundary position perpendicular to reference line
                                x_boundary = x_ref - offset * np.sin(hdg)
                                y_boundary = y_ref + offset * np.cos(hdg)
                                boundary_points.append([x_boundary, y_boundary, 0.0])

                            except Exception as e:
                                logger.debug(
                                    f"Failed to calculate boundary at s={s}: {e}"
                                )
                                continue

                        if len(boundary_points) > 1:
                            boundaries.append(np.array(boundary_points))

                    except Exception as e:
                        logger.warning(
                            f"Failed to process left lane {lane.id} in section at s={lane_section.s_offset}: {e}"
                        )

                # Process right lanes
                for lane in lane_section.right_lanes:
                    try:
                        boundary_points = []

                        s_start = lane_section.s_offset
                        s_end = (
                            lane_section.s_offset + lane_section.length
                            if hasattr(lane_section, "length")
                            else road.length
                        )

                        for s in s_vals:
                            if s < s_start or s > s_end:
                                continue

                            try:
                                x_ref, y_ref, hdg = road.get_geometry(s)

                                # Calculate offset for outer boundary
                                offset = 0.0
                                for i in range(abs(lane.id)):
                                    other_lane = next(
                                        (
                                            ln
                                            for ln in lane_section.right_lanes
                                            if ln.id == -(i + 1)
                                        ),
                                        None,
                                    )
                                    if other_lane:
                                        s_relative = s - s_start
                                        width = other_lane.get_width(s_relative)
                                        offset += width

                                # For right lanes, offset is in the opposite direction
                                x_boundary = x_ref + offset * np.sin(hdg)
                                y_boundary = y_ref - offset * np.cos(hdg)
                                boundary_points.append([x_boundary, y_boundary, 0.0])

                            except Exception as e:
                                logger.debug(
                                    f"Failed to calculate boundary at s={s}: {e}"
                                )
                                continue

                        if len(boundary_points) > 1:
                            boundaries.append(np.array(boundary_points))

                    except Exception as e:
                        logger.warning(
                            f"Failed to process right lane {lane.id} in section at s={lane_section.s_offset}: {e}"
                        )

        logger.info(f"Extracted {len(boundaries)} lane boundaries from OpenDRIVE")
        return boundaries

    except Exception as e:
        raise OpenDRIVEParsingError(f"Failed to parse OpenDRIVE file: {e}") from e


def extract_lanelet_boundaries(
    lanelet_map: lanelet2.core.LaneletMap,
    lanelet_ids: Optional[List[int]] = None,
) -> Dict[int, Dict[str, np.ndarray]]:
    """Extract boundary points from lanelets.

    Args:
        lanelet_map: Loaded Lanelet2 map
        lanelet_ids: Optional list of specific lanelet IDs to extract

    Returns:
        Dictionary mapping lanelet_id -> {'left': (N,3), 'right': (N,3)}
    """
    boundaries = {}
    lanelets = lanelet_map.laneletLayer

    for lanelet in lanelets:
        if lanelet_ids and lanelet.id not in lanelet_ids:
            continue

        boundaries[lanelet.id] = {
            "left": extract_points_3d(lanelet.leftBound),
            "right": extract_points_3d(lanelet.rightBound),
        }

    logger.info(f"Extracted boundaries from {len(boundaries)} lanelets")
    return boundaries


def interpolate_points_at_s(
    points: np.ndarray, cumulative_s: np.ndarray, sample_s: np.ndarray
) -> np.ndarray:
    """Interpolate 3D points at specified arc length positions.

    Args:
        points: (N, 3) array of [x, y, z] coordinates
        cumulative_s: (N,) array of cumulative arc lengths
        sample_s: (K,) array of arc lengths to sample at

    Returns:
        (K, 3) array of interpolated points
    """
    x_interp = np.interp(sample_s, cumulative_s, points[:, 0])
    y_interp = np.interp(sample_s, cumulative_s, points[:, 1])
    z_interp = np.interp(sample_s, cumulative_s, points[:, 2])

    return np.column_stack([x_interp, y_interp, z_interp])


def calculate_boundary_errors(
    lanelet_boundary: np.ndarray,
    opendrive_boundaries: List[np.ndarray],
    sample_interval: float = 0.5,
) -> Tuple[np.ndarray, np.ndarray, int]:
    """Calculate errors between Lanelet2 and nearest OpenDRIVE boundary.

    Args:
        lanelet_boundary: (M, 3) Lanelet2 points
        opendrive_boundaries: List of (N, 3) OpenDRIVE lane boundaries
        sample_interval: Arc length sampling interval (m)

    Returns:
        s_coords: (K,) S-coordinates where errors were calculated
        errors: (K,) 3D Euclidean distances at each sample point
        matched_lane_idx: Index of the matched OpenDRIVE lane boundary

    Raises:
        NoMatchingLaneError: If no matching lane found within search radius
    """
    # Calculate 2D arc length for Lanelet2 boundary
    diffs = np.diff(lanelet_boundary[:, :2], axis=0)
    segment_lengths = np.linalg.norm(diffs, axis=1)
    cumulative_s = np.concatenate(([0], np.cumsum(segment_lengths)))

    # Sample Lanelet2 boundary at regular S intervals
    max_s = cumulative_s[-1]
    sample_s = np.arange(0, max_s + sample_interval, sample_interval)

    # Ensure we don't exceed the boundary
    sample_s = sample_s[sample_s <= max_s]

    sampled_points = interpolate_points_at_s(lanelet_boundary, cumulative_s, sample_s)

    # For each sample point, find closest OpenDRIVE boundary point (3D)
    min_distances = []
    lane_votes = []

    search_radius = DEFAULT_CONFIG.visualization.nearest_neighbor_search_radius

    for point in sampled_points:
        closest_dist = float("inf")
        closest_lane_idx = -1

        for lane_idx, opendrive_boundary in enumerate(opendrive_boundaries):
            # Calculate 3D distances to all points in this boundary
            distances = np.linalg.norm(opendrive_boundary - point, axis=1)
            min_dist = distances.min()

            if min_dist < closest_dist:
                closest_dist = min_dist
                closest_lane_idx = lane_idx

        if closest_dist > search_radius:
            logger.warning(
                f"Nearest OpenDRIVE point distance {closest_dist:.2f}m exceeds search radius {search_radius}m"
            )

        min_distances.append(closest_dist)
        lane_votes.append(closest_lane_idx)

    # Majority vote to determine matched lane
    if not lane_votes or all(idx == -1 for idx in lane_votes):
        raise NoMatchingLaneError(
            f"No OpenDRIVE lane found within {search_radius}m search radius"
        )

    # Filter out -1 votes before majority vote
    valid_votes = [v for v in lane_votes if v != -1]
    if not valid_votes:
        raise NoMatchingLaneError("No valid lane matches found")

    matched_lane_idx = max(set(valid_votes), key=valid_votes.count)

    return sample_s, np.array(min_distances), matched_lane_idx


def visualize_boundary_errors(
    lanelet_boundaries: Dict[int, Dict[str, np.ndarray]],
    errors_data: Dict[int, Dict[str, Tuple[np.ndarray, np.ndarray, int]]],
    colormap: str = "coolwarm",
) -> plt.Figure:
    """Create visualization with color-coded error display.

    Args:
        lanelet_boundaries: Lanelet boundary points
        errors_data: Dictionary with error calculation results
        colormap: Matplotlib colormap name

    Returns:
        matplotlib Figure object
    """
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(18, 8))

    # Collect all errors for normalization
    all_errors_list: List[float] = []
    for lanelet_id, sides in errors_data.items():
        for side in ["left", "right"]:
            if side in sides:
                _, errors, _ = sides[side]
                all_errors_list.extend(errors.tolist())

    all_errors = np.array(all_errors_list)

    # Determine normalization range (95th percentile)
    if len(all_errors) > 0:
        vmax = np.percentile(all_errors, 95)
    else:
        vmax = 1.0

    # Left plot: Boundary comparison with color-coded errors
    cmap = plt.get_cmap(colormap)
    norm = plt.Normalize(vmin=0, vmax=vmax)

    for lanelet_id, boundaries in lanelet_boundaries.items():
        if lanelet_id not in errors_data:
            continue

        for side in ["left", "right"]:
            if side not in boundaries or side not in errors_data[lanelet_id]:
                continue

            boundary = boundaries[side]
            s_coords, errors, _ = errors_data[lanelet_id][side]

            # Plot Lanelet2 boundary as thin gray baseline
            ax1.plot(
                boundary[:, 0],
                boundary[:, 1],
                color="gray",
                linewidth=0.5,
                alpha=0.3,
                zorder=1,
            )

            # Color-code segments by error magnitude
            for i in range(len(s_coords) - 1):
                # Interpolate position for visualization
                # Map s_coords to boundary indices
                total_length = s_coords[-1]
                if total_length > 0:
                    idx_start = int((s_coords[i] / total_length) * (len(boundary) - 1))
                    idx_end = int(
                        (s_coords[i + 1] / total_length) * (len(boundary) - 1)
                    )
                    idx_end = max(idx_start + 1, idx_end)
                    idx_end = min(idx_end, len(boundary) - 1)
                else:
                    continue

                color = cmap(norm(errors[i]))
                ax1.plot(
                    boundary[idx_start : idx_end + 1, 0],
                    boundary[idx_start : idx_end + 1, 1],
                    color=color,
                    linewidth=3,
                    alpha=0.8,
                    zorder=2,
                )

    # Add colorbar
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
    sm.set_array([])
    cbar = plt.colorbar(sm, ax=ax1)
    cbar.set_label("Error (m)", rotation=270, labelpad=20)

    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Y (m)")
    ax1.set_title("Lanelet2 vs OpenDRIVE Boundary Comparison")
    ax1.axis("equal")
    ax1.grid(True, alpha=0.3)

    # Right plot: Error histogram
    bins = DEFAULT_CONFIG.visualization.histogram_bins
    ax2.hist(all_errors, bins=bins, color="steelblue", alpha=0.7, edgecolor="black")
    ax2.set_xlabel("Error (m)")
    ax2.set_ylabel("Frequency")
    ax2.set_title("Error Distribution")

    # Add statistics
    if len(all_errors) > 0:
        mean_error = all_errors.mean()
        p95_error = np.percentile(all_errors, 95)
        max_error = all_errors.max()

        ax2.axvline(
            mean_error,
            color="red",
            linestyle="--",
            linewidth=2,
            label=f"Mean: {mean_error:.3f}m",
        )
        ax2.axvline(
            p95_error,
            color="orange",
            linestyle="--",
            linewidth=2,
            label=f"95th percentile: {p95_error:.3f}m",
        )
        ax2.legend()

        # Add text box with statistics
        stats_text = (
            f"Statistics:\n"
            f"Mean: {mean_error:.3f}m\n"
            f"Std Dev: {all_errors.std():.3f}m\n"
            f"Max: {max_error:.3f}m\n"
            f"95th percentile: {p95_error:.3f}m"
        )
        ax2.text(
            0.98,
            0.97,
            stats_text,
            transform=ax2.transAxes,
            fontsize=10,
            verticalalignment="top",
            horizontalalignment="right",
            bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5),
        )

    plt.tight_layout()
    return fig


def save_error_data_json(
    errors_data: Dict[int, Dict[str, Tuple[np.ndarray, np.ndarray, int]]],
    metadata: Dict[str, Any],
    output_path: Path,
) -> None:
    """Save error data to JSON file.

    Args:
        errors_data: Dictionary with error calculation results
        metadata: Metadata about the analysis
        output_path: Path to save JSON file
    """
    data = {
        "metadata": {
            "opendrive_file": str(metadata["opendrive_file"]),
            "lanelet2_file": str(metadata["lanelet2_file"]),
            "sample_interval": metadata["sample_interval"],
            "timestamp": metadata["timestamp"],
        },
        "lanelets": {},
    }

    for lanelet_id, sides in errors_data.items():
        data["lanelets"][str(lanelet_id)] = {}

        for side in ["left", "right"]:
            if side not in sides:
                continue

            s, errors, matched_lane_idx = sides[side]

            data["lanelets"][str(lanelet_id)][side] = {
                "s_coordinates": s.tolist(),
                "errors": errors.tolist(),
                "matched_opendrive_lane_idx": int(matched_lane_idx),
                "statistics": {
                    "min": float(errors.min()),
                    "max": float(errors.max()),
                    "mean": float(errors.mean()),
                    "std": float(errors.std()),
                    "p95": float(np.percentile(errors, 95)),
                },
            }

    output_path.write_text(json.dumps(data, indent=2))
    logger.info(f"Saved error data to {output_path}")


def save_figure_pickle(fig: plt.Figure, output_path: Path) -> None:
    """Save matplotlib figure as pickle for later loading.

    Args:
        fig: matplotlib Figure object
        output_path: Path to save pickle file
    """
    with open(output_path, "wb") as f:
        pickle.dump(fig, f)
    logger.info(f"Saved figure pickle to {output_path}")


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments.

    Returns:
        Parsed arguments
    """
    parser = argparse.ArgumentParser(
        description="Visualize boundary errors between Lanelet2 and OpenDRIVE",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Basic usage with interactive display
  uv run visualize lanelet2_map.xodr lanelet2_map.osm

  # Specify lanelet IDs and save outputs
  uv run visualize map.xodr map.osm --lanelet-id 120 121 122 \\
      --output-json errors.json --output-png visualization.png

  # Adjust sampling interval and colormap
  uv run visualize map.xodr map.osm --sample-interval 1.0 \\
      --colormap viridis --no-show
        """,
    )

    parser.add_argument(
        "opendrive_file", type=Path, help="Path to OpenDRIVE .xodr file"
    )
    parser.add_argument("lanelet2_file", type=Path, help="Path to Lanelet2 .osm file")

    parser.add_argument(
        "--lanelet-id",
        type=int,
        nargs="+",
        metavar="ID",
        help="Specific lanelet IDs to visualize (default: all)",
    )
    parser.add_argument(
        "--boundary",
        choices=["left", "right", "both"],
        default="both",
        help="Which boundary to compare (default: both)",
    )
    parser.add_argument(
        "--sample-interval",
        type=float,
        default=DEFAULT_CONFIG.visualization.sample_interval_default,
        metavar="METERS",
        help=f"S-coordinate sampling interval in meters (default: {DEFAULT_CONFIG.visualization.sample_interval_default})",
    )

    parser.add_argument(
        "--output-json", type=Path, metavar="PATH", help="Save error data as JSON"
    )
    parser.add_argument(
        "--output-pickle",
        type=Path,
        metavar="PATH",
        help="Save matplotlib figure as pickle",
    )
    parser.add_argument(
        "--output-png",
        type=Path,
        metavar="PATH",
        help="Save visualization as PNG image",
    )

    parser.add_argument(
        "--colormap",
        default=DEFAULT_CONFIG.visualization.colormap_default,
        metavar="NAME",
        help=f"Matplotlib colormap name (default: {DEFAULT_CONFIG.visualization.colormap_default})",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Do not show interactive plot window",
    )
    parser.add_argument(
        "--mgrs-code",
        type=str,
        metavar="CODE",
        help="MGRS code for Lanelet2 projection (e.g., 54SUE)",
    )

    return parser.parse_args()


def main() -> int:
    """Main entry point for boundary error visualization.

    Returns:
        Exit code (0 for success, 1 for error)
    """
    args = parse_args()

    # Validate input files
    if not args.opendrive_file.exists():
        logger.error(f"OpenDRIVE file not found: {args.opendrive_file}")
        return 1

    if not args.lanelet2_file.exists():
        logger.error(f"Lanelet2 file not found: {args.lanelet2_file}")
        return 1

    try:
        # Load OpenDRIVE file
        logger.info("Step 1/4: Loading OpenDRIVE file...")
        opendrive_boundaries = extract_opendrive_lane_boundaries(args.opendrive_file)

        if not opendrive_boundaries:
            logger.error("No lane boundaries found in OpenDRIVE file")
            return 1

        # Load Lanelet2 file
        logger.info("Step 2/4: Loading Lanelet2 file...")

        # Determine origin based on MGRS code if provided
        if args.mgrs_code:
            from .util import mgrs_to_lanelet2_origin

            origin = mgrs_to_lanelet2_origin(args.mgrs_code)
            logger.info(f"Using MGRS origin: {args.mgrs_code}")
        else:
            origin = lanelet2.io.Origin(0, 0)
            logger.info("Using default origin (0, 0)")

        projector = lanelet2.projection.UtmProjector(origin)
        lanelet_map = lanelet2.io.load(str(args.lanelet2_file), projector)

        lanelet_boundaries = extract_lanelet_boundaries(lanelet_map, args.lanelet_id)

        if not lanelet_boundaries:
            logger.error("No lanelets found with specified IDs")
            return 1

        # Calculate errors
        logger.info("Step 3/4: Calculating boundary errors...")
        errors_data: Dict[int, Dict[str, Tuple[np.ndarray, np.ndarray, int]]] = {}

        for lanelet_id, boundaries in lanelet_boundaries.items():
            errors_data[lanelet_id] = {}

            sides_to_process = []
            if args.boundary in ["left", "both"]:
                sides_to_process.append("left")
            if args.boundary in ["right", "both"]:
                sides_to_process.append("right")

            for side in sides_to_process:
                boundary = boundaries[side]

                try:
                    s_coords, errors, matched_idx = calculate_boundary_errors(
                        boundary, opendrive_boundaries, args.sample_interval
                    )

                    errors_data[lanelet_id][side] = (s_coords, errors, matched_idx)

                    logger.info(
                        f"  Lanelet {lanelet_id} ({side}): "
                        f"mean={errors.mean():.3f}m, "
                        f"max={errors.max():.3f}m, "
                        f"matched_lane={matched_idx}"
                    )

                except NoMatchingLaneError as e:
                    logger.warning(f"  Lanelet {lanelet_id} ({side}): {e}")

        # Visualize results
        logger.info("Step 4/4: Creating visualization...")
        fig = visualize_boundary_errors(
            lanelet_boundaries, errors_data, colormap=args.colormap
        )

        # Save outputs
        metadata = {
            "opendrive_file": args.opendrive_file,
            "lanelet2_file": args.lanelet2_file,
            "sample_interval": args.sample_interval,
            "timestamp": datetime.now().isoformat(),
        }

        if args.output_json:
            save_error_data_json(errors_data, metadata, args.output_json)

        if args.output_pickle:
            save_figure_pickle(fig, args.output_pickle)

        if args.output_png:
            dpi = DEFAULT_CONFIG.visualization.figure_dpi
            fig.savefig(args.output_png, dpi=dpi, bbox_inches="tight")
            logger.info(f"Saved visualization to {args.output_png}")

        # Show interactive plot
        if not args.no_show:
            plt.show()

        logger.info("Visualization complete!")
        return 0

    except (
        OpenDRIVEParsingError,
        Lanelet2LoadingError,
        BoundaryVisualizationError,
    ) as e:
        logger.error(f"Error: {e}")
        return 1
    except KeyboardInterrupt:
        logger.info("Interrupted by user")
        return 1
    except Exception as e:
        logger.exception(f"Unexpected error: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
