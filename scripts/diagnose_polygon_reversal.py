#!/usr/bin/env python3
"""Diagnostic tool for identifying lanelets at risk of polygon reversal.

This script analyzes a Lanelet2 map and identifies lanelets that may
experience polygon reversal issues during OpenDRIVE conversion due to:
- Asymmetric left/right boundary lengths
- Varying lane widths
- Sharp curves with outer/inner boundary length differences

Usage:
    python scripts/diagnose_polygon_reversal.py <lanelet2_map.osm>
"""

import argparse
import sys
from pathlib import Path
from typing import Dict, List, Tuple

import lanelet2
import numpy as np
from lanelet2.io import Origin, load

# Add src to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from autoware_lanelet2_to_opendrive.config import DEFAULT_CONFIG
from autoware_lanelet2_to_opendrive.util import extract_points_2d


def calculate_boundary_lengths(lanelet: lanelet2.core.Lanelet) -> Tuple[float, float]:
    """Calculate total arc length of left and right boundaries.

    Args:
        lanelet: Lanelet to analyze

    Returns:
        Tuple of (left_length, right_length) in meters
    """
    left_points = extract_points_2d(lanelet.leftBound)
    right_points = extract_points_2d(lanelet.rightBound)

    # Calculate cumulative arc lengths
    left_diffs = np.diff(left_points, axis=0)
    left_lengths = np.sqrt(np.sum(left_diffs**2, axis=1))
    left_total = np.sum(left_lengths)

    right_diffs = np.diff(right_points, axis=0)
    right_lengths = np.sqrt(np.sum(right_diffs**2, axis=1))
    right_total = np.sum(right_lengths)

    return left_total, right_total


def calculate_width_variation(
    lanelet: lanelet2.core.Lanelet, num_samples: int = 10
) -> float:
    """Calculate lane width variation ratio (max/min).

    Args:
        lanelet: Lanelet to analyze
        num_samples: Number of sample points along the lanelet

    Returns:
        Width variation ratio (max_width / min_width)
    """
    left_points = extract_points_2d(lanelet.leftBound)
    right_points = extract_points_2d(lanelet.rightBound)

    # Sample widths at uniform intervals
    widths = []
    for t in np.linspace(0, 1, num_samples):
        # Simple linear interpolation for sampling
        left_idx = min(int(t * (len(left_points) - 1)), len(left_points) - 1)
        right_idx = min(int(t * (len(right_points) - 1)), len(right_points) - 1)

        left_pos = left_points[left_idx]
        right_pos = right_points[right_idx]

        width = np.linalg.norm(left_pos - right_pos)
        widths.append(width)

    if len(widths) == 0 or min(widths) == 0:
        return 1.0

    return max(widths) / min(widths)


def calculate_curvature_estimate(points: np.ndarray) -> float:
    """Estimate maximum curvature of a polyline.

    Args:
        points: Array of points (N x 2)

    Returns:
        Maximum curvature estimate (1/radius in 1/meters)
    """
    if len(points) < 3:
        return 0.0

    max_curvature = 0.0

    # Use three-point circles to estimate curvature
    for i in range(len(points) - 2):
        p1, p2, p3 = points[i], points[i + 1], points[i + 2]

        # Calculate circumradius using cross product
        a = np.linalg.norm(p2 - p1)
        b = np.linalg.norm(p3 - p2)
        c = np.linalg.norm(p3 - p1)

        # Avoid division by zero
        if a < 1e-6 or b < 1e-6 or c < 1e-6:
            continue

        # Heron's formula for area
        s = (a + b + c) / 2.0
        area_sq = s * (s - a) * (s - b) * (s - c)

        if area_sq <= 0:
            continue

        area = np.sqrt(area_sq)

        # Circumradius R = abc / (4 * area)
        # Curvature = 1 / R
        if area > 1e-6:
            radius = (a * b * c) / (4.0 * area)
            curvature = 1.0 / radius
            max_curvature = max(max_curvature, curvature)

    return max_curvature


def analyze_lanelet(lanelet: lanelet2.core.Lanelet) -> Dict:
    """Analyze a lanelet for polygon reversal risk factors.

    Args:
        lanelet: Lanelet to analyze

    Returns:
        Dictionary with analysis results
    """
    left_length, right_length = calculate_boundary_lengths(lanelet)
    width_variation = calculate_width_variation(lanelet)

    # Calculate boundary length ratio
    if min(left_length, right_length) < 1e-6:
        length_ratio = 1.0
    else:
        length_ratio = max(left_length, right_length) / min(left_length, right_length)

    # Estimate curvature
    left_points = extract_points_2d(lanelet.leftBound)
    right_points = extract_points_2d(lanelet.rightBound)
    left_curvature = calculate_curvature_estimate(left_points)
    right_curvature = calculate_curvature_estimate(right_points)
    max_curvature = max(left_curvature, right_curvature)

    # Calculate risk score (0-100)
    risk_score = 0.0

    # Asymmetry contributes to risk
    if length_ratio > DEFAULT_CONFIG.geometry.boundary_length_ratio_threshold:
        asymmetry_risk = (length_ratio - 1.0) * 30.0
        risk_score += min(asymmetry_risk, 40.0)

    # Width variation contributes to risk
    if width_variation > 1.2:
        width_risk = (width_variation - 1.0) * 20.0
        risk_score += min(width_risk, 30.0)

    # High curvature with asymmetry is especially risky
    if max_curvature > 0.05 and length_ratio > 1.3:
        curvature_risk = max_curvature * 100.0
        risk_score += min(curvature_risk, 30.0)

    risk_score = min(risk_score, 100.0)

    return {
        "lanelet_id": lanelet.id,
        "left_length": left_length,
        "right_length": right_length,
        "length_ratio": length_ratio,
        "width_variation": width_variation,
        "max_curvature": max_curvature,
        "risk_score": risk_score,
    }


def report_problematic_lanelets(
    lanelet_map: lanelet2.core.LaneletMap, risk_threshold: float = 20.0
) -> List[Dict]:
    """Generate report of lanelets with polygon reversal risk.

    Args:
        lanelet_map: Lanelet2 map to analyze
        risk_threshold: Minimum risk score to report (0-100)

    Returns:
        List of analysis dictionaries for problematic lanelets
    """
    problematic = []

    for lanelet in lanelet_map.laneletLayer:
        analysis = analyze_lanelet(lanelet)

        if analysis["risk_score"] >= risk_threshold:
            problematic.append(analysis)

    # Sort by risk score (highest first)
    problematic.sort(key=lambda x: x["risk_score"], reverse=True)

    return problematic


def main():
    """Main entry point for diagnostic tool."""
    parser = argparse.ArgumentParser(
        description="Diagnose lanelets at risk of polygon reversal"
    )
    parser.add_argument("input_map", type=str, help="Path to Lanelet2 OSM map file")
    parser.add_argument(
        "--risk-threshold",
        type=float,
        default=20.0,
        help="Minimum risk score to report (0-100, default: 20.0)",
    )
    parser.add_argument(
        "--verbose",
        "-v",
        action="store_true",
        help="Show detailed analysis for all lanelets",
    )

    args = parser.parse_args()

    # Load map
    print(f"Loading map: {args.input_map}")
    try:
        lanelet_map = load(args.input_map, Origin(0, 0))
    except Exception as e:
        print(f"Error loading map: {e}", file=sys.stderr)
        return 1

    total_lanelets = len(lanelet_map.laneletLayer)
    print(f"Total lanelets: {total_lanelets}\n")

    # Analyze lanelets
    print("Analyzing lanelets for polygon reversal risk...\n")
    problematic = report_problematic_lanelets(
        lanelet_map, risk_threshold=args.risk_threshold
    )

    if len(problematic) == 0:
        print(
            f"✅ No lanelets found with risk score >= {args.risk_threshold}. "
            f"Map appears safe for conversion."
        )
        return 0

    # Print report
    print(f"⚠️  Found {len(problematic)} lanelet(s) at risk of polygon reversal:\n")
    print(
        f"{'ID':<8} {'Risk':<6} {'L/R Ratio':<10} {'Width Var':<10} {'Curvature':<10} {'Status'}"
    )
    print("-" * 70)

    for analysis in problematic:
        status = ""
        if analysis["risk_score"] >= 60:
            status = "🔴 HIGH"
        elif analysis["risk_score"] >= 40:
            status = "🟡 MEDIUM"
        else:
            status = "🟢 LOW"

        print(
            f"{analysis['lanelet_id']:<8} "
            f"{analysis['risk_score']:5.1f}  "
            f"{analysis['length_ratio']:9.2f}  "
            f"{analysis['width_variation']:9.2f}  "
            f"{analysis['max_curvature']:9.4f}  "
            f"{status}"
        )

    print("\n" + "=" * 70)
    print(f"Summary: {len(problematic)}/{total_lanelets} lanelets at risk")
    print(
        f"Threshold: Length ratio > {DEFAULT_CONFIG.geometry.boundary_length_ratio_threshold:.1f}, "
        f"Quality > {DEFAULT_CONFIG.geometry.correspondence_quality_threshold:.1f}"
    )

    if args.verbose:
        print("\n" + "=" * 70)
        print("Detailed Analysis:")
        for analysis in problematic:
            print(f"\nLanelet {analysis['lanelet_id']}:")
            print(f"  Left boundary length:  {analysis['left_length']:.2f} m")
            print(f"  Right boundary length: {analysis['right_length']:.2f} m")
            print(f"  Length ratio:          {analysis['length_ratio']:.2f}")
            print(f"  Width variation:       {analysis['width_variation']:.2f}")
            print(f"  Max curvature:         {analysis['max_curvature']:.4f} (1/m)")
            print(f"  Risk score:            {analysis['risk_score']:.1f}/100")

    return 0


if __name__ == "__main__":
    sys.exit(main())
